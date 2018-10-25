"""
Sample module for implementing a robot that coordinates with a CNC and conveyors.
"""
from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import os, sys

from hurco_bridge import *

from interfaces.material import *
from interfaces.tool import *
from interfaces.door import *
from interfaces.chuck import *

from collaborationModel.collaborator import *
from collaborationModel.coordinator import *
from collaborationModel.priority import priority
from collaborationModel.archetypeToInstance import archetypeToInstance
from collaborationModel.from_long_pull import from_long_pull, from_long_pull_asset

from adapter.mtconnect_adapter import Adapter
from adapter.long_pull import LongPull
from adapter.data_item import Event, SimpleCondition, Sample, ThreeDSample

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, re, copy, datetime, collections
import xml.etree.ElementTree as ET
import requests, urllib2, uuid

RobotEvent = collections.namedtuple('RobotEvent', ['source', 'component', 'name', 'value', 'code', 'text'])

class Robot:
    class StateModel:
        """The model for MTConnect behavior in the robot."""
        def __init__(self,host,port,parent,sim):

            self.parent = parent

            self.sim = sim

            self.initiate_adapter(host,port)
            self.adapter.start()
            self.initiate_dataitems()

            self.initiate_interfaces()
            self.events = []

            self.lp ={}

            self.master_tasks ={}
            self.nextsequence='1'
            self.deviceUuid = "r1"

            # Maximum amount of time for material handling to complete before marking it complete regardless
            self.material_load_interface.superstate.simulated_duration = 900
            self.material_unload_interface.superstate.simulated_duration = 900

            self.master_uuid = str()

            self.iscoordinator = False

            self.iscollaborator = True

            self.fail_next = False

            self.reset_required = False

            self.initial_execution_state()

            self.set_priority()

            self.low_level_event_list = []

            self.initiate_pull_thread()

        def set_priority(self):
            self.priority = None
            self.priority = priority(self, self.robot_binding)

        def initial_execution_state(self):
            self.execution = {}
            self.execution['cnc1'] = None
            self.execution['cmm1'] = None
            self.execution['b1'] = None
            self.execution['conv1'] = None
            self.execution['r1'] = None

        def initiate_interfaces(self):
            self.material_load_interface = MaterialLoadResponse(self, self.sim)
            self.material_unload_interface = MaterialUnloadResponse(self, self.sim)
            self.open_chuck_interface = OpenChuckRequest(self)
            self.close_chuck_interface = CloseChuckRequest(self)
            self.open_door_interface = OpenDoorRequest(self)
            self.close_door_interface = CloseDoorRequest(self)
            self.change_tool_interface = ChangeToolRequest(self)

        def initiate_adapter(self, host, port):
            self.adapter = Adapter((host,port))

            self.mode1 = Event('mode')
            self.adapter.add_data_item(self.mode1)

            self.e1 = Event('exec')
            self.adapter.add_data_item(self.e1)

            self.avail1 = Event('avail')
            self.adapter.add_data_item(self.avail1)

            self.binding_state_material = Event('binding_state_material')
            self.adapter.add_data_item(self.binding_state_material)

            self.robot_binding = Event('robot_binding')
            self.adapter.add_data_item(self.robot_binding)

            self.open_chuck = Event('open_chuck')
            self.adapter.add_data_item(self.open_chuck)

            self.close_chuck = Event('close_chuck')
            self.adapter.add_data_item(self.close_chuck)

            self.open_door = Event('open_door')
            self.adapter.add_data_item(self.open_door)

            self.close_door = Event('close_door')
            self.adapter.add_data_item(self.close_door)

            self.change_tool = Event('change_tool')
            self.adapter.add_data_item(self.change_tool)

            self.material_load = Event('material_load')
            self.adapter.add_data_item(self.material_load)

            self.material_unload = Event('material_unload')
            self.adapter.add_data_item(self.material_unload)

            self.material_state = Event('material_state')
            self.adapter.add_data_item(self.material_state)

        def initiate_dataitems(self):
            self.adapter.begin_gather()

            self.avail1.set_value("AVAILABLE")
            self.e1.set_value("READY")
            self.mode1.set_value("AUTOMATIC")
            self.binding_state_material.set_value("INACTIVE")
            self.open_chuck.set_value("NOT_READY")
            self.close_chuck.set_value("NOT_READY")
            self.open_door.set_value("NOT_READY")
            self.close_door.set_value("NOT_READY")
            self.change_tool.set_value("NOT_READY")
            self.material_load.set_value("NOT_READY")
            self.material_unload.set_value("NOT_READY")
            self.material_state.set_value("UNLOADED")

            self.adapter.complete_gather()

        def initiate_pull_thread(self):

            self.thread= Thread(target = self.start_pull,args=("http://localhost:5000","""/cnc/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",from_long_pull))
            self.thread.start()

            self.thread2= Thread(target = self.start_pull,args=("http://localhost:5000","""/conv/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",from_long_pull))
            self.thread2.start()

            self.thread3= Thread(target = self.start_pull,args=("http://localhost:5000","""/buffer/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",from_long_pull))
            self.thread3.start()

            self.thread4= Thread(target = self.start_pull,args=("http://localhost:5000","""/cmm/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",from_long_pull))
            self.thread4.start()

        def interface_type(self, value = None, subtype = None):
            self.interfaceType = value

        def start_pull(self,addr,request, func, stream = True):

            response = requests.get(addr+request+"&from="+self.nextsequence, stream=stream)
            self.lp[request.split('/')[1]] = None
            self.lp[request.split('/')[1]] = LongPull(response, addr, self)
            self.lp[request.split('/')[1]].long_pull(func)

        def start_pull_asset(self, addr, request, assetId, stream_root):
            response = urllib2.urlopen(addr+request).read()
            from_long_pull_asset(self, response, stream_root)


        def ACTIVATE(self):
            self.make_operational()
            self.open_chuck_interface.superstate.start()
            self.close_chuck_interface.superstate.start()
            self.open_door_interface.superstate.start()
            self.close_door_interface.superstate.start()
            self.change_tool_interface.superstate.start()


        def FAULT(self):

            self.collaborator.superstate.unavailable()
            self.open_chuck_interface.superstate.DEACTIVATE()
            self.close_chuck_interface.superstate.DEACTIVATE()
            self.open_door_interface.superstate.DEACTIVATE()
            self.close_door_interface.superstate.DEACTIVATE()
            self.material_load_interface.superstate.DEACTIVATE()
            self.material_unload_interface.superstate.DEACTIVATE()
            self.change_tool_interface.superstate.DEACTIVATE()


        def OPERATIONAL(self):
            self.make_idle()

        def IDLE(self):
            if self.check_tool_change_req():
                return

            self.adapter.begin_gather()
            self.e1.set_value("READY")
            self.adapter.complete_gather()

            if self.reset_required:
                time.sleep(2)
                self.reset_statemachine()
                self.reset_required = False

            #toolchange edit
            self.material_unload_interface.superstate.not_ready()
            self.material_load_interface.superstate.not_ready()
            self.collaborator = collaborator(parent = self, interface = self.binding_state_material, collaborator_name = self.deviceUuid)
            self.collaborator.create_statemachine()
            time.sleep(0.1)
            self.collaborator.superstate.unavailable()

            self.priority.collab_check()

        def check_tool_change_req(self):
            if self.master_uuid in self.master_tasks and 'ToolChange' in str(self.master_tasks[self.master_uuid]):
                def activate():
                    if self.collaborator:
                        time.sleep(2)
                        self.collaborator.superstate.currentSubTaskState = "active"
                thread = Thread(target=activate)
                thread.start()
                print ("Tool Change")
                return True
            else:
                return False


        def LOADING(self):
            self.material_unload_interface.superstate.not_ready()
            self.material_load_interface.superstate.ready()

        def UNLOADING(self):
            self.material_load_interface.superstate.not_ready()
            self.material_unload_interface.superstate.ready()
            self.adapter.begin_gather()
            self.e1.set_value("ACTIVE")
            self.adapter.complete_gather()


        def LOADING_COMPLETE(self):
            coordinator = self.master_tasks[self.master_uuid]['coordinator'].keys()[0]
            for k,v in self.master_tasks[self.master_uuid]['coordinator'][coordinator]['SubTask'].iteritems():
                if 'MaterialLoad' in v:
                    self.priority.binding_state(k,has_material = True)
                    break

            #imts_demo part rotation: reset after every cycle: good->bad->rework part cycle
            if coordinator == 'cmm1' and self.master_tasks[self.master_uuid]['part_quality'] == 'rework':
                self.reset_required = True

        #imts_demo part rotation: method to reset statemachine after a 3-part cycle is completed
        def reset_statemachine(self):

            coordinator = self.master_tasks[self.master_uuid]['coordinator'].keys()[0]
            uuid = copy.deepcopy(self.master_uuid)

            if coordinator == 'cmm1' and self.master_tasks[uuid]['part_quality'] == 'rework':
                print ("Resetting Robot")
                self.events = []
                self.low_level_event_list = []

                for k,v in self.master_tasks.iteritems():
                    if k != uuid:
                        self.master_tasks[k] = None

                for k,v in self.lp.iteritems():
                    self.lp[k]._response = None

                nsx = urllib2.urlopen("http://localhost:5000/current").read()
                nsr = ET.fromstring(nsx)
                ns = nsr[0].attrib['nextSequence']

                self.nextsequence = ns

                self.priority = None

                self.set_priority()

                try:
                    self.FAULT()
                except Exception as e:
                    print ("Error resetting interfaces",e)

                self.adapter.removeAllAsset('Task')

                self.initiate_pull_thread()

                print ("robot reset")


        def UNLOADING_COMPLETE(self):
            self.priority.binding_state(self.master_tasks[self.master_uuid]['coordinator'].keys()[0],has_material = False)

        def COMPLETED(self):
            if "request" in self.interfaceType.lower():
                pass

            elif "response" in self.interfaceType.lower() and "material" in self.interfaceType.lower():
                if "unloaded" not in self.interfaceType.lower():
                    self.event(self.deviceUuid, 'material_interface', 'SubTask_'+'MaterialUnload','COMPLETE')
                elif "unloaded" in self.interfaceType.lower():
                    self.event(self.deviceUuid, 'material_interface', 'SubTask_'+'MaterialLoad','COMPLETE')

        def event(self, source, comp, name, value, code = None, text = None):
            ev = RobotEvent(source, comp, name, value, code, text)

            self.events.append(ev)

            action = value.lower()

            if comp == "Coordinator" and value.lower() == 'preparing':
                self.priority.event_list([source, comp, name, value, code, text])

            if action == "fail":
                action = "failure"

            if "Collaborator" in comp and action!='unavailable':
                try:
                    self.coordinator.superstate.event(source, comp, name, value, code, text)
                except Exception as e:
                    time.sleep(0.2)
                    print ("Error in Coordinator event: sending back to the event method for a retry")
		    print(e)
                    self.event(source, comp, name, value, code, text)

            elif "Coordinator" in comp and action!='unavailable':
                try:
                    if value.lower() != 'preparing':
                        self.collaborator.superstate.event(source, comp, name, value, code, text)

                    if 'binding_state' in name and value.lower() == 'committed' and text == self.master_tasks[self.master_uuid]['coordinator'].keys()[0]:
                        if self.material_state.value() == "LOADED":
                            self.material_load_ready()
                        else:
                            self.material_unload_ready()
                except Exception as e:
                    print ("Error in Collaborator event: sending back to the event method for a retry in 0.2 s.")
		    print (e)
                    time.sleep(0.2)
                    self.event(source, comp, name, value, code, text)

            elif 'SubTask' in name and action!='unavailable':
                try:
                    if comp == 'interface_initialization' and source == self.deviceUuid:
                        if 'CloseChuck' in name:
                            print ("CloseChuck Request to cnc1")
                        elif 'CloseDoor' in name:
                            print ("CloseDoor Request to cnc1")
                        elif 'OpenChuck' in name:
                            print ("OpenChuck Request to cnc1")
                        elif 'OpenDoor' in name:
                            print ("OpenDoor Request to cnc1")
                        elif 'ChangeTool' in name:
                            print ("ChangeTool Request to cnc1")

                    if self.iscoordinator:
                        self.coordinator.superstate.event(source, comp, name, value, code, text)

                    elif self.iscollaborator:
                        self.collaborator.superstate.event(source, comp, name, value, code, text)

                except Exception as e:
                    print ("Error in SubTask event: sending back to the event method for a retry in 0.5 s.")
                    time.sleep(0.5)
                    self.event(source, comp, name, value, code, text)


            elif ev.name.startswith('Material') and action!='unavailable':
                self.material_event(ev)

            elif comp == 'internal_event':
                self.internal_event(ev)

            elif ('Chuck' in name or 'Door' in name or 'ChangeTool' in name) and action!='unavailable':

                if 'Chuck' in name:
                    if 'Open' in name:
                        eval('self.open_chuck_interface.superstate.'+action+'()')
                    elif 'Close' in name:
                        eval('self.close_chuck_interface.superstate.'+action+'()')
                elif 'Door' in name:
                    if 'Open' in name:
                        eval('self.open_door_interface.superstate.'+action+'()')
                    elif 'Close' in name:
                        eval('self.close_door_interface.superstate.'+action+'()')

                elif 'ChangeTool' in name:
                    eval('self.change_tool_interface.superstate.'+action+'()')


            elif ev.component.startswith('Controller'):
                self.controller_event(ev)

            else:
                pass

        def internal_event(self, ev):
            status = None
            action = ev.value.lower()
            if ev.name == "MoveIn":
                print ("Moving In " + ev.text)

                if ['move_in',ev.text,self.master_tasks[self.master_uuid]['part_quality']] not in self.low_level_event_list:
                    self.low_level_event_list.append(['move_in',ev.text,self.master_tasks[self.master_uuid]['part_quality']])

                status = self.parent.move_in(ev.text, self.master_tasks[self.master_uuid]['part_quality'])
                if status != True:
                    self.fault()
                print ("Moved in")

            elif "MoveInCT" in ev.name:
                print ("Moving In cnc1_t1")

                if ['move_in','cnc1_t1',self.master_tasks[self.master_uuid]['part_quality']] not in self.low_level_event_list:
                    self.low_level_event_list.append(['move_in','cnc1_t1',self.master_tasks[self.master_uuid]['part_quality']])

                status = self.parent.move_in('cnc1_t1', self.master_tasks[self.master_uuid]['part_quality'])
                if status != True:
                    self.fault()
                print ("Moved in")

	    elif ev.name == "MoveOut":
                print ("Moving Out From " + ev.text)

                if ['move_out',ev.text,self.master_tasks[self.master_uuid]['part_quality']] not in self.low_level_event_list:
                    self.low_level_event_list.append(['move_out',ev.text,self.master_tasks[self.master_uuid]['part_quality']])

                status = self.parent.move_out(ev.text, self.master_tasks[self.master_uuid]['part_quality'])
                if status != True:
                    self.fault()
                print ("Moved out")

	    elif "MoveOutCT" in ev.name:
                print ("Moving Out From cnc1_t1")

                if ['move_out','cnc1_t1',self.master_tasks[self.master_uuid]['part_quality']] not in self.low_level_event_list:
                    self.low_level_event_list.append(['move_out','cnc1_t1',self.master_tasks[self.master_uuid]['part_quality']])

                status = self.parent.move_out('cnc1_t1', self.master_tasks[self.master_uuid]['part_quality'])
                if status != True:
                    self.fault()
                print ("Moved out")

            elif ev.name == "PickUpTool":
                print ("Picking Up Tool")

                status = self.internal_event(RobotEvent('ToolHolder', ev.component, 'MoveIn', ev.value, ev.code, 't1'))
                if status != True:
                    self.fault()

                if status == True:
                    status = self.internal_event(RobotEvent('ToolHolder', ev.component, 'GrabPart', ev.value, ev.code, 't1'))
                if status != True:
                    self.fault()

                if status == True:
                    status = self.internal_event(RobotEvent('ToolHolder', ev.component, 'MoveOut', ev.value, ev.code, 't1'))
                if status != True:
                    self.fault()

            elif ev.name == "DropOffTool":
                print ("Droping Off Tool")

                status = self.internal_event(RobotEvent('ToolHolder', ev.component, 'MoveIn', ev.value, ev.code, 't1'))
                if status != True:
                    self.fault()

                if status == True:
                    self.internal_event(RobotEvent('ToolHolder', ev.component, 'ReleasePart', ev.value, ev.code, 't1'))
                if status != True:
                    self.fault()

                if status == True:
                    self.internal_event(RobotEvent('ToolHolder', ev.component, 'MoveOut', ev.value, ev.code, 't1'))
                if status != True:
                    self.fault()

                if status == True:
                    self.collaborator.superstate.subTask['ToolChange'].superstate.success()
                    self.adapter.begin_gather()
                    self.e1.set_value("READY")
                    self.adapter.complete_gather()
                else:
                    self.fault()


            elif ev.name == "ReleasePart":
                print ("Releasing the Part onto " + ev.text)

                if ['release',ev.text,None] not in self.low_level_event_list:
                    self.low_level_event_list.append(['release',ev.text,None])

                status = self.parent.release(ev.text, self.master_tasks[self.master_uuid]['part_quality'])
                if status != True:
                    self.fault()

                print ("Released")

            elif ev.name == "GrabPart":
                print ("Grabbing Part from " + ev.text)

                if ['grab',ev.text,None] not in self.low_level_event_list:
                    self.low_level_event_list.append(['grab',ev.text,None])

                status = self.parent.grab(ev.text, self.master_tasks[self.master_uuid]['part_quality'])
                if status != True:
                    self.fault()
                print ("Grabbed")

            return status

        def material_event(self, ev):
            action = ev.value.lower()
            timeout = 2.0
            if action == "fail":
                action = "failure"

            if ev.name == "MaterialLoad":
                eval('self.material_load_interface.superstate.'+action+'()')
                if ev.value.lower() == 'complete' and ev.component != "interface_completion":
                    self.complete()

            elif ev.name == "MaterialUnload":
                eval('self.material_unload_interface.superstate.'+action+'()')
                if ev.value.lower() == 'complete' and ev.component != "interface_completion":
                    self.complete()

            else:
                pass

        def controller_event(self, ev):
            if ev.name == "ControllerMode":
                if ev.source.lower() == 'robot':

                    self.adapter.begin_gather()
                    self.mode1.set_value(ev.value.upper())
                    self.adapter.complete_gather()

            elif ev.name == "Execution":
                if ev.source.lower() == 'robot':

                    self.adapter.begin_gather()
                    self.e1.set_value(ev.value.upper())
                    self.adapter.complete_gather()

                elif ev.text in self.execution:
                    self.execution[ev.text] = ev.value.lower()

            else:
                pass

        def device_event(self, ev):
            if ev.name == 'Availability':
                if ev.source.lower() == 'robot':

                    self.adapter.begin_gather()
                    self.avail1.set_value(ev.value.upper())
                    self.adapter.complete_gather()

            else:
                raise(Exception('Unknown Device event: ' + str(ev)))

        #end StateModel class definition

    def __init__(self,host,port,parent=None,sim=True):
        self.superstate = Robot.StateModel(host,port,parent,sim)
        self.statemachine = self.create_state_machine(self.superstate)

    def draw(self):
        self.statemachine.get_graph().draw('robot.png', prog='dot')

    def set_state_trigger(self, state, callback):
        """
        Allows a user to set a function to be called when the device enters a particular state. Returns a function
        that the user should call at the end of the callback to signal that the callback is done.
        """
        self.statemachine.on_enter(state, callback)

    @staticmethod
    def create_state_machine(state_machine_model):
        """Create and initialize the robot state machine"""

        NestedState.separator = ':'
        states = [
            {
                'name': 'base',
                'children': [
                    'activated',
                    {
                        'name': 'operational',
                        'children': [
                            'idle',
                            {
                                'name': 'loading',
                                'children': ['moving_in', 'waiting_chuck', 'moving_out']
                            },
                            {
                                'name': 'unloading',
                                'children': ['moving_in', 'waiting_chuck', 'moving_out']
                            }
                        ]
                    },
                    {
                        'name': 'disabled',
                        'children': [
                            'not_ready',
                            {
                                'name': 'fault',
                                'children': ['software', 'hardware', 'e_stop']
                            },
                        ]
                    },
                ]
            }
        ]

        transitions = [
            ['start', 'base', 'base:disabled:not_ready'],
            ['activate', 'base:disabled:not_ready', 'base:activated'],
            ['make_operational', 'base:activated', 'base:operational'],
            ['make_idle', 'base:operational', 'base:operational:idle'],

            ['enable', 'base', 'base:activated'],

            ['fault', 'base', 'base:disabled:fault'],
            ['safety_violation', 'base', 'base:disabled:soft'],
            ['collision', 'base', 'base:disabled:fault:soft'],
            ['hard_fault', 'base', 'base:disable:fault:hard'],
            ['e_stop', 'base', 'base:disabled:fault:e_stop'],
            ['clear_fault', 'base:disabled:fault', 'base:disabled:not_ready'],

            {
                'trigger': 'material_unload_ready',
                'source': 'base:operational:idle',
                'dest': 'base:operational:unloading',
            },
            {
                'trigger': 'material_load_ready',
                'source': 'base:operational:idle',
                'dest': 'base:operational:loading',
            },
            {
                'trigger': 'complete',
                'source': 'base:operational:loading',
                'dest': 'base:operational:idle',
                'before': 'LOADING_COMPLETE',
            },
            {
                'trigger': 'complete',
                'source': 'base:operational:unloading',
                'dest': 'base:operational:loading',
                'before': 'UNLOADING_COMPLETE'
            },

        ]

        statemachine = Machine(
            model = state_machine_model,
            states = states,
            transitions = transitions,
            initial = 'base',
            ignore_invalid_triggers=True
        )

        statemachine.on_enter('base:activated', 'ACTIVATE')
        statemachine.on_enter('base:operational', 'OPERATIONAL')
        statemachine.on_enter('base:operational:idle','IDLE')
        statemachine.on_enter('base:operational:loading', 'LOADING')
        statemachine.on_enter('base:operational:unloading', 'UNLOADING')
        statemachine.on_enter('base:disabled:fault', 'FAULT')

        return statemachine

if __name__ == '__main__':
    robot = Robot('localhost',7996,RobotInterface(), sim = True)
    robot.superstate.material_load_interface.superstate.simulated_duration = 40
    robot.superstate.material_unload_interface.superstate.simulated_duration = 40
    robot.superstate.enable()


