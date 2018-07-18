"""
Sample module for implementing a robot that coordinates with a CNC and conveyors.
"""
#from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

from material import *
from door import *
from chuck import *
from coordinator import *
from collaborator import *
from mtconnect_adapter import Adapter
from long_pull import LongPull
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from archetypeToInstance import archetypeToInstance
from from_long_pull import from_long_pull, from_long_pull_asset

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, re, datetime
import requests, urllib2, collections
import xml.etree.ElementTree as ET


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

            self.master_tasks ={}

            self.deviceUuid = "r1"

            self.material_load_interface.superstate.simulated_duration = 60
            self.material_unload_interface.superstate.simulated_duration = 60

            self.master_uuid = str()

            self.iscoordinator = False
            
            self.iscollaborator = True

            self.fail_next = False

            self.initiate_pull_thread()

        def initiate_interfaces(self):
            self.material_load_interface = MaterialLoadResponse(self, self.sim)
            self.material_unload_interface = MaterialUnloadResponse(self, self.sim)
            self.open_chuck_interface = OpenChuckRequest(self)
            self.close_chuck_interface = CloseChuckRequest(self)
            self.open_door_interface = OpenDoorRequest(self)
            self.close_door_interface = CloseDoorRequest(self)
            
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

            self.open_chuck = Event('open_chuck')
            self.adapter.add_data_item(self.open_chuck)

            self.close_chuck = Event('close_chuck')
            self.adapter.add_data_item(self.close_chuck)

            self.open_door = Event('open_door')
            self.adapter.add_data_item(self.open_door)

            self.close_door = Event('close_door')
            self.adapter.add_data_item(self.close_door)

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
            self.material_load.set_value("NOT_READY")
            self.material_unload.set_value("NOT_READY")
            self.material_state.set_value("UNLOADED")

            self.adapter.complete_gather()

        def initiate_pull_thread(self):

            thread= Thread(target = self.start_pull,args=("http://localhost:5000","/cnc/sample?interval=100&count=1000",from_long_pull))
            thread.start()

            thread2= Thread(target = self.start_pull,args=("http://localhost:5000","/conv/sample?interval=100&count=1000",from_long_pull))
            thread2.start()

            thread3= Thread(target = self.start_pull,args=("http://localhost:5000","/buffer/sample?interval=100&count=1000",from_long_pull))
            thread3.start()

            thread4= Thread(target = self.start_pull,args=("http://localhost:5000","/cmm/sample?interval=100&count=1000",from_long_pull))
            thread4.start()

            thread5= Thread(target = self.start_pull,args=("http://localhost:5000","/conv2/sample?interval=100&count=1000",from_long_pull))
            thread5.start()

        def interface_type(self, value = None, subtype = None):
            self.interfaceType = value

        def start_pull(self,addr,request, func, stream = True):

            response = requests.get(addr+request, stream=stream)
            lp = LongPull(response, addr, self)
            lp.long_pull(func)

        def start_pull_asset(self, addr, request, assetId, stream_root):
            response = urllib2.urlopen(addr+request).read()
            from_long_pull_asset(self, response, stream_root)


        def ACTIVATE(self):
            self.make_operational()
            self.open_chuck_interface.superstate.start()
            self.close_chuck_interface.superstate.start()
            self.open_door_interface.superstate.start()
            self.close_door_interface.superstate.start()

        def OPERATIONAL(self):
            self.make_idle()

        def IDLE(self):
            self.material_unload_interface.superstate.not_ready()
            self.material_load_interface.superstate.not_ready()
            self.master_tasks = {}
            self.collaborator = collaborator(parent = self, interface = self.binding_state_material, collaborator_name = 'r1')
            self.collaborator.create_statemachine()
            self.collaborator.superstate.unavailable()

        def LOADING(self):
            self.material_unload_interface.superstate.not_ready()
            self.material_load_interface.superstate.ready()

        def UNLOADING(self):
            self.material_load_interface.superstate.not_ready()
            self.material_unload_interface.superstate.ready()

        def LOADING_COMPLETE(self):
            self.CHECK_COMPLETION()

        def CHECK_COMPLETION(self):
            #temporary fix till task/subtask sequencing is determined
            while self.master_tasks[self.master_uuid]['collaborators'][self.deviceUuid]['state'][2] != 'COMPLETE':
                pass
                
        def CHECK_COMPLETION_UL(self):
            #temporary fix till task/subtask sequencing is determined
            #print 'checking completion'
            coordinator = self.master_tasks[self.master_uuid]['coordinator'].keys()[0]
            unload_task = self.master_tasks[self.master_uuid]['coordinator'][coordinator]['SubTask'][coordinator][0]
            test = None
            while not test:
                if unload_task in self.master_tasks[self.master_uuid]['collaborators'][self.deviceUuid]['SubTask']:
                    for x in self.master_tasks[self.master_uuid]['collaborators'][self.deviceUuid]['SubTask'][unload_task]:
                        if x[2] == 'COMPLETE' and (test== None or test == True):
                            test = True
                        else:
                            test = False
                else:
                    test = True
            
        def UNLOADING_COMPLETE(self):
            self.CHECK_COMPLETION_UL()

        def LOAD_READY(self):
            """Function triggered when the CNC is ready to be loaded"""
            #TODO: verify that it's ok to start loading
            "self.loading()"

        def UNLOAD_READY(self):
            """Function triggered when the CNC is ready to be unloaded"""
            #TODO: verify that it's ok to start unloading
            "self.unloading()"

        def COMPLETED(self):
            if "request" in self.interfaceType.lower():
                "self.complete()" #What to do with the requests!?
            elif "response" in self.interfaceType.lower() and "material" in self.interfaceType.lower():
                if "unloaded" not in self.interfaceType.lower():
                    self.material_state.set_value("LOADED")
                    self.event(self.deviceUuid, 'material_interface', 'SubTask_'+'MaterialUnload','COMPLETE')
                elif "unloaded" in self.interfaceType.lower():
                    self.material_state.set_value("UNLOADED")
                    self.event(self.deviceUuid, 'material_interface', 'SubTask_'+'MaterialLoad','COMPLETE')

        def event(self, source, comp, name, value, code = None, text = None):
            """Process events.

            :type ev: .event.Event
            """
            #print "\nROBOT Event Enter",source,comp,name,value,datetime.datetime.now().isoformat()
            ev = RobotEvent(source, comp, name, value, code, text)

            #print('Robot received: ', source, comp, name, value)
            self.events.append(ev)
            
            action = value.lower()

            if action == "fail":
                action = "failure"

            if "Collaborator" in comp and action!='unavailable':
                self.coordinator.superstate.event(source, comp, name, value, code, text)

            elif "Coordinator" in comp and action!='unavailable':
                
                if 'binding_state' in name and value.lower() == 'committed' and text == self.master_tasks[self.master_uuid]['coordinator'].keys()[0]:
                    if self.material_state.value() == "LOADED":
                        self.material_load_ready()
                    else:
                        self.material_unload_ready()
                self.collaborator.superstate.event(source, comp, name, value, code, text)


            elif 'SubTask' in name and action!='unavailable':
                if comp == 'interface_initialization' and source == self.deviceUuid:
                    if 'CloseChuck' in name:
                        print "\n Robot has moved in!"
                        print "\n Robot has requested CNC to Close Chuck!"
                    elif 'CloseDoor' in name:
                        print "\n Robot has released the Part!"
                        print "\n Robot has moved out!"
                        print "\n Robot has requested CNC to Close Door!"
                    elif 'OpenChuck' in name:
                        print "\n Robot has moved in!"
                        print "\n Robot has grabbed the part!"
                        print "\n Robot has requested CNC to Open Chuck!"
                    elif 'OpenDoor' in name:
                        print "\n Robot has requested CNC to Open Door!"

                        
                if self.iscoordinator:
                    self.coordinator.superstate.event(source, comp, name, value, code, text)

                elif self.iscollaborator:
                    self.collaborator.superstate.event(source, comp, name, value, code, text)



            elif ev.name.startswith('Material') and action!='unavailable':
                #print "in material method"
                self.material_event(ev)

            elif comp == 'internal_event':
                self.internal_event(ev)

            elif ('Chuck' in name or 'Door' in name) and action!='unavailable':

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


            #elif ev.component.startswith('Controller'):
                #self.controller_event(ev)

            #elif ev.component.startswith('Device'):
                #self.device_event(ev)

            else:
                """#print('Unknown event: ' + str(ev))"""

            #print "\nRobotEvent Exit",source,comp,name,value,datetime.datetime.now().isoformat()

        def internal_event(self, ev):
            action = ev.value.lower()
            if ev.name == "MoveIn":
                print "Moving In " + ev.text
                if self.sim:
                    time.sleep(2)
                else:
                    self.parent.move_in(ev.text, self.master_tasks[code]['part_quality'])
                print "Moved in"

            elif ev.name == "MoveOut":
                print "Moving Out From " + ev.text
                if self.sim:
                    time.sleep(2)
                else:
                    #Moveout will confirm the completion of unloading/loading tasks
                    complete = None
                    complete = self.parent.move_out(ev.text, self.master_tasks[code]['part_quality'])
                    if complete:
                        self.COMPLETED() #get confirmation from robot
                print "Moved out"

            elif ev.name == "PickUpTool":
                print "Picking Up Tool"
                self.internal_event(RobotEvent('ToolHolder', ev.component, 'MoveIn', ev.value, ev.code, 't1'))

                self.internal_event(RobotEvent('ToolHolder', ev.component, 'GrabPart', ev.value, ev.code, 't1'))

                self.internal_event(RobotEvent('ToolHolder', ev.component, 'MoveOut', ev.value, ev.code, 't1'))

                """
                if self.sim:
                    time.sleep(2)
                else:
                    self.parent.move_in(ev.text)
                """

            elif ev.name == "DropOffTool":
                print "Droping Off Tool"
                self.internal_event(RobotEvent('ToolHolder', ev.component, 'MoveIn', ev.value, ev.code, 't1'))

                self.internal_event(RobotEvent('ToolHolder', ev.component, 'ReleasePart', ev.value, ev.code, 't1'))

                self.internal_event(RobotEvent('ToolHolder', ev.component, 'MoveOut', ev.value, ev.code, 't1'))

                self.collaborator.superstate.subTask['ToolChange'].superstate.success()
                """
                if self.sim:
                    time.sleep(2)
                else:
                    self.parent.move_in(ev.text)
                """
            elif ev.name == "ReleasePart":
                print "Releasing the Part onto " + ev.text
                if self.sim:
                    time.sleep(2)
                else:
                    self.parent.release(ev.text)
                print "Released"

            elif ev.name == "GrabPart":
                print "Grabbing Part from " + ev.text
                if self.sim:
                    time.sleep(2)
                else:
                    self.parent.grab(ev.text)
                print "Grabbed"

            elif ev.name == "OpenDoor":
                eval('self.open_door_interface.superstate.'+action+'()')
                self.open_door_interface.superstate.active()
                print "Opening Door"
                if self.sim:
                    time.sleep(2)
                else:
                    self.parent.open_door(ev.text)
                self.open_door_interface.superstate.complete()
                print "Opened Door"
                self.open_door_interface.superstate.not_ready()

            elif ev.name == "CloseDoor":
                eval('self.close_door_interface.superstate.'+action+'()')
                self.close_door_interface.superstate.active()
                print "Closing Door"
                if self.sim:
                    time.sleep(2)
                else:
                    self.parent.close_door(ev.text)
                self.close_door_interface.superstate.complete()
                print "Closed Door"
                self.close_door_interface.superstate.not_ready()
                    
        def material_event(self, ev):
            action = ev.value.lower()
            if action == "fail":
                action = "failure"
                
            if ev.name == "MaterialLoad":
                if ev.value.lower() == 'complete':
                    self.complete()
                else:
                    eval('self.material_load_interface.superstate.'+action+'()')

            elif ev.name == "MaterialUnload":
                if ev.value.lower() == 'complete':
                    self.complete()
                else:
                    eval('self.material_unload_interface.superstate.'+action+'()')

            else:
                """#print "raise(Exception('Unknown Material event: ' + str(ev)))'"""

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
                    
            else:
                """raise(Exception('Unknown Controller event: ' + str(ev)))"""


        def device_event(self, ev):
            if ev.name == 'Availability':
                if ev.source.lower() == 'robot':
                    
                    self.adapter.begin_gather()
                    self.avail1.set_value(ev.value.upper())
                    self.adapter.complete_gather()
                #exec('self.'+ev.source.lower()+'_availability_'+value.lower()+'()')

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

            ['safety_violation', 'base', 'base:disabled:soft'],
            ['collision', 'base', 'base:disabled:fault:soft'],
            ['hard_fault', 'base', 'base:disable:fault:hard'],
            ['e_stop', 'base', 'base:disabled:fault:e_stop'],
            ['clear_fault', 'base:disabled:fault', 'base:disabled:not_ready'],

            {
                'trigger': 'material_unload_ready',
                'source': 'base:operational:idle',
                'dest': 'base:operational:unloading',
                'after': 'UNLOAD_READY'
            },
            {
                'trigger': 'material_load_ready',
                'source': 'base:operational:idle',
                'dest': 'base:operational:loading',
                'after': 'LOAD_READY'
            },
            {
                'trigger': 'complete',
                'source': 'base:operational:loading',
                'dest': 'base:operational:idle',
                'before': 'LOADING_COMPLETE'
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

        return statemachine

if __name__ == '__main__':
    robot1 = Robot('localhost',7971)
    time.sleep(1)
    robot1.superstate.enable()

    sample_mtconnect_demo = None
    if sample_mtconnect_demo:
        #Sample MTConnect Demo Code
        class MTConnectDemo:
            def __init__(self):
                self._bridge = mtconnect_bridge.Bridge()
                self._robot = Robot(host, port, self, False)

            def move_in(self, device = None, destination = None):
                if device == 'conv':
                    rospy.loginfo("Demo moving to 'input_conveyor'")
                    self._bridge.do_work('move', 'input_conveyor')
                    return True
                elif device == 'cnc1':
                    rospy.loginfo("Demo moving to 'cnc'")
                    self._bridge.do_work('move', 'cnc')
                    return True
                else:
                    #repeat for different devices
                    return

            def move_out(self, device = None, destination = None):
                
                rospy.loginfo("Demo moving to 'all_zeros'")
                self._bridge.do_work('move', 'all_zeros')
                #This method return indicates unload/load task completion
                return True


