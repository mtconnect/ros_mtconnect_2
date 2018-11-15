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
import functools, time, re, copy
import xml.etree.ElementTree as ET
import requests, urllib2, uuid


class cnc:

    class StateMachineModel:

        def __init__(self,host,port,sim):

            self.initiate_adapter(host,port)
            self.adapter.start()

            self.sim = sim

            self.initiate_dataitems()

            self.initiate_interfaces()

            self.system = []

            self.cycle_time = 10

            self.load_time_limit(20)
            self.unload_time_limit(20)

            self.load_failed_time_limit(2)
            self.unload_failed_time_limit(2)

            self.next_sequence = str(1)

            self.events = []

            self.master_tasks = {}

            self.device_uuid = "cnc1"

            self.master_uuid = str()

            self.is_coordinator = False

            self.is_collaborator = False

            self.system_normal = True

            self.has_material = False

            self.fail_next = False

            #long pull dict for maintaining agent threads for other devices
            self.lp = {}

            self.wait_for_completion = False

            self.initial_execution_state()

            self.set_priority()

            self.initiate_cnc_client()

            self.initiate_pull_thread()

        def set_priority(self):
            self.priority = None
            self.priority = priority(self, self.cnc_binding)

        def initial_execution_state(self):
            self.execution = {}
            self.execution['cnc1'] = None
            self.execution['cmm1'] = None
            self.execution['b1'] = None
            self.execution['conv1'] = None
            self.execution['r1'] = None

        def initiate_cnc_client(self):
            if not self.sim:
                configFile = open('configFiles/clients.cfg','r')
                device = json.loads(configFile.read())['devices'][self.device_uuid]
                self.cnc_client = hurcoClient(str(device['host']),int(device['port']))

        def initiate_interfaces(self):
            self.material_load_interface = MaterialLoad(self)
            self.material_unload_interface = MaterialUnload(self)

            if self.sim:
                self.open_chuck_interface = OpenChuck(self)
                self.close_chuck_interface = CloseChuck(self)
                self.open_door_interface = OpenDoor(self)
                self.close_door_interface = CloseDoor(self)
                self.change_tool_interface = ChangeTool(self)

            else:
                self.open_chuck_interface = OpenChuck(self, self.sim)
                self.close_chuck_interface = CloseChuck(self, self.sim)
                self.open_door_interface = OpenDoor(self, self.sim)
                self.close_door_interface = CloseDoor(self, self.sim)
                self.change_tool_interface = ChangeTool(self, self.sim)


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

            self.cnc_binding = Event('cnc_binding')
            self.adapter.add_data_item(self.cnc_binding)

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

            self.tool_state = Event('tool_state')
            self.adapter.add_data_item(self.tool_state)

            self.chuck_state = Event('chuck_state')
            self.adapter.add_data_item(self.chuck_state)

            self.door_state = Event('door_state')
            self.adapter.add_data_item(self.door_state)

            self.material_load = Event('material_load')
            self.adapter.add_data_item(self.material_load)

            self.material_unload = Event('material_unload')
            self.adapter.add_data_item(self.material_unload)

        def initiate_dataitems(self):

            self.adapter.begin_gather()

            self.door_state.set_value("CLOSED")
            self.chuck_state.set_value("OPEN")
            self.tool_state.set_value("AVAILABLE")
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

            self.adapter.complete_gather()

        def initiate_pull_thread(self):
            #Pull MTconnect data from other devices

            self.thread= Thread(
                target = self.start_pull,
                args=(
                    "http://localhost:5000",
                    """/conv/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",
                    from_long_pull
                    )
                )
            self.thread.start()

            self.thread2= Thread(
                target = self.start_pull,
                args=(
                    "http://localhost:5000",
                    """/robot/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",
                    from_long_pull
                    )
                )
            self.thread2.start()

            self.thread3= Thread(
                target = self.start_pull,
                args=(
                    "http://localhost:5000",
                    """/buffer/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",
                    from_long_pull
                    )
                )
            self.thread3.start()

            self.thread4= Thread(
                target = self.start_pull,
                args=(
                    "http://localhost:5000",
                    """/cmm/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",
                    from_long_pull))
            self.thread4.start()

        def start_pull(self,addr,request, func, stream = True):

            response = requests.get(addr+request+"&from="+self.next_sequence, stream=stream)
            self.lp[request.split('/')[1]] = None
            self.lp[request.split('/')[1]] = LongPull(response, addr, self)
            self.lp[request.split('/')[1]].long_pull(func)

        def start_pull_asset(self, addr, request, assetId, stream_root):
            response = urllib2.urlopen(addr+request).read()
            from_long_pull_asset(self, response, stream_root)


        def CNC_NOT_READY(self):
            self.change_tool_interface.superstate.DEACTIVATE()
            self.open_chuck_interface.superstate.DEACTIVATE()
            self.close_chuck_interface.superstate.DEACTIVATE()
            self.open_door_interface.superstate.DEACTIVATE()
            self.close_door_interface.superstate.DEACTIVATE()
            self.material_load_interface.superstate.DEACTIVATE()
            self.material_unload_interface.superstate.DEACTIVATE()

        def ACTIVATE(self):

            if self.mode1.value() == "AUTOMATIC" and self.avail1.value() == "AVAILABLE":

                self.make_operational()

            elif self.system_normal:

                self.still_not_ready()

            else:

                self.faulted()

        def OPERATIONAL(self):
            self.open_chuck_interface.superstate.ACTIVATE()
            self.close_chuck_interface.superstate.ACTIVATE()
            self.open_door_interface.superstate.ACTIVATE()
            self.close_door_interface.superstate.ACTIVATE()
            self.change_tool_interface.superstate.ACTIVATE()

            if self.has_material:
                self.unloading()

                self.is_coordinator = True
                self.is_collaborator = False

                if self.master_uuid in self.master_tasks:
                    del self.master_tasks[self.master_uuid]

                self.master_uuid = self.device_uuid+'_'+str(uuid.uuid4())

                master_task_uuid = copy.deepcopy(self.master_uuid)


                self.adapter.begin_gather()
                self.cnc_binding.set_value(master_task_uuid)
                self.adapter.complete_gather()

                self.coordinator_task = "MoveMaterial_2"

                self.coordinator = coordinator(
                    parent = self,
                    master_task_uuid = master_task_uuid,
                    interface = self.binding_state_material,
                    coordinator_name = self.device_uuid
                    )

                self.coordinator.superstate.task_name = "UnloadCnc"

                self.coordinator.superstate.unavailable()

            elif self.has_material == False:
                self.loading()

                self.is_coordinator = False
                self.is_collaborator = True
                self.collaborator = collaborator(
                    parent = self,
                    interface = self.binding_state_material,
                    collaborator_name = 'cnc1'
                    )

                self.collaborator.superstate.task_name = "LoadCnc"
                self.collaborator.superstate.unavailable()

                self.priority.collab_check()

            else:
                self.start()

        def IDLE(self):

            if self.has_material:
                self.material_unload_interface.superstate.IDLE()

            else:
                self.material_load_interface.superstate.IDLE()

        def CYCLING(self):
            if self.fail_next:
                self.system.append(['cnc', 'Device', 'SYSTEM', 'FAULT', 'Cycle failed to start', 'CYCLE'])
                self.cnc_fault()
                self.fail_next = False

            elif self.door_state.value() != "CLOSED" or self.chuck_state.value() != "CLOSED":

                self.system.append(['cnc', 'Device', 'SYSTEM', 'FAULT', 'Door or Chuck in invalid state', 'CYCLE'])
                self.cnc_fault()

            else:
                self.adapter.begin_gather()
                self.e1.set_value("ACTIVE")
                self.adapter.complete_gather()

                def func(self = self):
                    self.cnc_execution_ready()
                    self.is_coordinator = True
                    self.is_collaborator = False

                    if self.master_uuid in self.master_tasks:
                        del self.master_tasks[self.master_uuid]

                    self.master_uuid = self.device_uuid+'_'+str(uuid.uuid4())
                    master_task_uuid = copy.deepcopy(self.master_uuid)
                    self.coordinator_task = "MoveMaterial_2"

                    self.adapter.begin_gather()
                    self.cnc_binding.set_value(master_task_uuid)
                    self.adapter.complete_gather()

                    self.coordinator = coordinator(
                        parent = self,
                        master_task_uuid = master_task_uuid,
                        interface = self.binding_state_material,
                        coordinator_name = self.device_uuid
                        )

                    self.coordinator.superstate.task_name = "UnloadCnc"

                    self.coordinator.superstate.unavailable()

                    self.adapter.begin_gather()
                    self.e1.set_value("READY")
                    self.adapter.complete_gather()

                if self.sim:
                    if not self.cycle_time:
                        return func()
                    timer_cycling = Timer(self.cycle_time,func)
                    timer_cycling.daemon = True
                    timer_cycling.start()
                else:
                    cycle_completion = self.cnc_client.load_run_pgm(tasks.cycle)
                    if cycle_completion == True:
                        time.sleep(1)
                        func()

        def LOADING(self):
            if not self.has_material:
                self.material_load_interface.superstate.idle()


        def UNLOADING(self):
            if self.has_material:
                self.material_unload_interface.superstate.idle()

        def load_time_limit(self, limit):
            self.material_load_interface.superstate.processing_time_limit = limit

        def load_failed_time_limit(self, limit):
            self.material_load_interface.superstate.fail_time_limit = limit

        def unload_time_limit(self, limit):
            self.material_unload_interface.superstate.processing_time_limit = limit

        def unload_failed_time_limit(self, limit):
            self.material_unload_interface.superstate.fail_time_limit = limit

        def interface_type(self, value = None, subtype = None):
            self.interfaceType = value

        def COMPLETED(self):
            if self.interfaceType == "Request":
                self.complete()

            elif "Response" and "chuck" in self.interfaceType:
                self.adapter.begin_gather()

                if "open" in self.interfaceType:
                    self.has_material = False
                    self.chuck_state.set_value("OPEN")

                elif "close" in self.interfaceType:
                    self.has_material = True
                    self.chuck_state.set_value("CLOSED")

                self.adapter.complete_gather()

            elif "Response" and "door" in self.interfaceType:
                self.adapter.begin_gather()

                if "open" in self.interfaceType:
                    self.door_state.set_value("OPEN")

                elif "close" in self.interfaceType:
                    self.door_state.set_value("CLOSED")

                self.adapter.complete_gather()

            elif "Response" and "tool" in self.interfaceType:
                self.adapter.begin_gather()
                self.tool_state.set_value("AVAILABLE")
                self.adapter.complete_gather()

        def EXITING_IDLE(self):

            if self.has_material:
                self.unloading()
                self.is_coordinator = True
                self.is_collaborator = False

                if self.master_uuid in self.master_tasks:
                    del self.master_tasks[self.master_uuid]

                self.master_uuid = self.device_uuid+'_'+str(uuid.uuid4())
                master_task_uuid = copy.deepcopy(self.master_uuid)

                self.adapter.begin_gather()
                self.cnc_binding.set_value(master_task_uuid)
                self.adapter.complete_gather()

                self.coordinator_task = "MoveMaterial_2"

                self.coordinator = coordinator(
                    parent = self,
                    master_task_uuid = master_task_uuid,
                    interface = self.binding_state_material,
                    coordinator_name = self.device_uuid
                    )

                self.coordinator.superstate.task_name = "UnloadCnc"

                self.coordinator.superstate.unavailable()

            else:
                self.loading()
                self.is_coordinator = False
                self.is_collaborator = True
                self.collaborator = collaborator(
                    parent = self,
                    interface = self.binding_state_material,
                    collaborator_name = 'cnc1'
                    )

                self.collaborator.superstate.task_name = "LoadCnc"
                self.collaborator.superstate.unavailable()
                self.priority.collab_check()

        def LOADED(self):
            self.has_material = True
            self.material_load_interface.superstate.DEACTIVATE()


        def IN_TRANSITION(self):
            pass

        def EXIT_TRANSITION(self):
            if self.has_material == False:
                self.is_coordinator = False
                self.is_collaborator = True
                self.collaborator = collaborator(
                    parent = self,
                    interface = self.binding_state_material,
                    collaborator_name = 'cnc1'
                    )

                self.collaborator.superstate.task_name = "LoadCnc"
                self.collaborator.superstate.unavailable()
                self.priority.collab_check()


        def UNLOADED(self):
            self.has_material = False
            self.material_unload_interface.superstate.DEACTIVATE()

        def FAILED(self):
            if "Request" in self.interfaceType:
                self.failed()
            elif "Response" in self.interfaceType:
                self.fault()

        def void(self):
            pass


        def event(self, source, comp, name, value, code = None, text = None):

            self.events.append([source, comp, name, value, code, text])

            action= value.lower()

            if action == "fail":
                action = "failure"

            if comp == "Coordinator" and value.lower() == 'preparing':
                self.priority.event_list([source, comp, name, value, code, text])

            if comp == "Task_Collaborator" and action!='unavailable':
                self.coordinator.superstate.event(source, comp, name, value, code, text)

            elif comp == "Coordinator" and action!='unavailable':
                if value.lower() == 'committed':
                    self.collaborator.superstate.event(source, comp, name, value, code, text)
                    if 'ToolChange' in str(self.master_tasks[self.master_uuid]):
                        self.event('robot','ToolInterface','SubTask_ToolChange','ACTIVE',self.master_uuid,'r1')

                elif value.lower() != 'preparing':
                    self.collaborator.superstate.event(source, comp, name, value, code, text)

            elif 'SubTask' in name and action!='unavailable':
                if self.is_coordinator == True:
                    self.coordinator.superstate.event(source, comp, name, value, code, text)

                elif self.is_collaborator == True:
                    self.collaborator.superstate.event(source, comp, name, value, code, text)

            elif "Open" in name and action!='unavailable':
                if 'door' in name.lower():
                    eval('self.open_door_interface.superstate.'+action+'()')

                    if not self.sim and action == 'active':
                        openDoor_completion = self.cnc_client.load_run_pgm(tasks.openDoor)
                        if openDoor_completion == True:
                            time.sleep(2)
                            eval('self.open_door_interface.superstate.complete()')

                        elif openDoor_completion == False:
                            time.sleep(2)
                            eval('self.open_door_interface.superstate.DEFAULT()')

                if 'chuck' in name.lower():
                    eval('self.open_chuck_interface.superstate.'+action+'()')

                    if not self.sim and action == 'active':
                        openChuck_completion = self.cnc_client.load_run_pgm(tasks.openChuck)
                        if openChuck_completion == True:
                            time.sleep(2)
                            eval('self.open_chuck_interface.superstate.complete()')

                        elif openChuck_completion == False:
                            time.sleep(2)
                            eval('self.open_chuck_interface.superstate.DEFAULT()')

            elif "Close" in name and action!='unavailable':
                if 'door' in name.lower():
                    eval('self.close_door_interface.superstate.'+action+'()')

                    if not self.sim and action == 'active':
                        closeDoor_completion = self.cnc_client.load_run_pgm(tasks.closeDoor)
                        if closeDoor_completion == True:
                            time.sleep(1)
                            eval('self.close_door_interface.superstate.complete()')

                        elif closeDoor_completion == False:
                            time.sleep(1)
                            eval('self.close_door_interface.superstate.DEFAULT()')

                if 'chuck' in name.lower():
                    eval('self.close_chuck_interface.superstate.'+action+'()')
                    if not self.sim and action == 'active':
                        closeChuck_completion = self.cnc_client.load_run_pgm(tasks.closeChuck)

                        if closeChuck_completion == True:
                            time.sleep(1)
                            eval('self.close_chuck_interface.superstate.complete()')

                        elif closeChuck_completion == False:
                            time.sleep(1)
                            eval('self.close_chuck_interface.superstate.DEFAULT()')

            elif name == "MaterialLoad" and action!='unavailable':
                try:
                    if action=='ready' and self.state =='base:operational:idle':
                        eval('self.robot_material_load_ready()')
                    eval('self.material_load_interface.superstate.'+action+'()')

                except Exception as e:
                    print ("Incorrect event")
                    print (e)

            elif name == "MaterialUnload" and action!='unavailable':
                try:
                    if action =='ready' and self.state =='base:operational:idle':
                        eval('self.robot_material_unload_ready()')
                    eval('self.material_unload_interface.superstate.'+action+'()')

                except Exception as e:
                    print ("incorrect event")
                    print (e)

            elif name == "ChangeTool" and action!='unavailable':
                eval('self.change_tool_interface.superstate.'+action+'()')

                if not self.sim and action == 'active':
                    toolChange_completion = self.cnc_client.load_run_pgm(tasks.toolChange)
                    if toolChange_completion == True:
                        time.sleep(2)
                        eval('self.change_tool_interface.superstate.complete()')

                    elif toolChange_completion == False:
                        time.sleep(2)
                        eval('self.change_tool_interface.superstate.DEFAULT()')

            elif comp == "Controller":

                if name == "ControllerMode":
                    if source.lower() == 'cnc':
                        self.adapter.begin_gather()
                        self.mode1.set_value(value.upper())
                        self.adapter.complete_gather()

                elif name == "Execution":
                    if source.lower() == 'cnc':
                        self.adapter.begin_gather()
                        self.e1.set_value(value.upper())
                        self.adapter.complete_gather()
                    elif text in self.execution:
                        self.execution[text]  = value.lower()

            elif comp == "Device":

                if name == "SYSTEM" and action!='unavailable':
                    try:
                        eval('self.'+source.lower()+'_system_'+value.lower()+'()')

                    except Exception as e:
                        print ("Not a valid Device trigger",e)

                elif name == "Availability":
                    if source.lower() == 'cnc':
                        self.adapter.begin_gather()
                        self.avail1.set_value(value.upper())
                        self.adapter.complete_gather()


    def __init__(self,host,port,sim=True):

        self.superstate = cnc.StateMachineModel(host,port,sim)
        self.statemachine = self.create_statemachine(self.superstate)


    def create_statemachine(self, state_machine_model):
        NestedState.separator = ':'

        states = [
            {
                'name':'base',
                'children':[
                    'activated',
                    {
                        'name':'operational',
                        'children':[
                            'loading',
                            'cycle_start',
                            'unloading',
                            'idle',
                            'in_transition'
                            ]
                    },
                    {
                        'name':'disabled',
                        'children':[
                            'fault', 'not_ready'
                        ]
                    }
                ]
            }
        ]

        transitions= [
            ['start', 'base', 'base:disabled'],

            ['cnc_controller_mode_automatic', 'base', 'base:activated'],


            ['reset_cnc', 'base', 'base:activated'],
            ['enable', 'base', 'base:activated'],
            ['disable', 'base', 'base:activated'],
            ['cnc_controller_mode_manual', 'base', 'base:activated'],
            ['cnc_controller_mode_manual_data_input', 'base', 'base:activated'],
            ['cnc_controller_mode_automatic', 'base:disabled', 'base:activated'],
            ['robot_material_load_ready', 'base:disabled', 'base:activated'],
            ['robot_material_unload_ready', 'base:disabled', 'base:activated'],

            ['default', 'base:operational:cycle_start', 'base:operational:cycle_start'],

            {
                'trigger':'complete',
                'source':'base:operational:loading',
                'dest':'base:operational:cycle_start',
                'before':'LOADED'
            },

            ['complete', 'base:operational:in_transition', 'base:operational:loading'],

            {
                'trigger':'complete',
                'source':'base:operational:unloading',
                'dest':'base:operational:in_transition',
                'before':'UNLOADED'
            },

            ['fault', 'base', 'base:disabled:fault'],
            ['robot_system_fault', 'base', 'base:disabled:fault'],
            ['default', 'base:disabled:fault', 'base:disabled:fault'],
            ['faulted', 'base:activated', 'base:disabled:fault'],
            ['cnc_fault', 'base:operational:cycle_start','base:disabled:fault'],

            ['start', 'base:disabled', 'base:disabled:not_ready'],
            ['default', 'base:disabled:not_ready', 'base:disabled:not_ready'],
            ['default', 'base:disabled', 'base:disabled:not_ready'],
            ['still_not_ready', 'base:activated', 'base:disabled:not_ready'],

            ['loading', 'base:operational', 'base:operational:loading'],
            ['default', 'base:operational:loading', 'base:operational:loading'],
            ['unloading', 'base:operational', 'base:operational:unloading'],
            ['default', 'base:operational:unloading', 'base:operational:unloading'],
            ['cnc_execution_ready', 'base:operational:cycle_start', 'base:operational:unloading'],

            ['failed', 'base:operational:loading', 'base:operational:idle'],
            ['failed', 'base:operational:unloading', 'base:operational:idle'],
            ['start', 'base:operational', 'base:operational:idle'],

            {
                'trigger':'robot_material_unload_ready',
                'source':'base:operational:idle',
                'dest':'base:operational',
                'after':'EXITING_IDLE'
            },

            {
                'trigger':'robot_material_load_ready',
                'source':'base:operational:idle',
                'dest':'base:operational',
                'after':'EXITING_IDLE'
            },

            ['default', 'base:operational:idle', 'base:operational:idle'],

            ['make_operational', 'base:activated', 'base:operational']

        ]

        statemachine = Machine(
            model = state_machine_model,
            states = states,
            transitions = transitions,
            initial = 'base',
            ignore_invalid_triggers=True
            )

        statemachine.on_enter('base:disabled', 'CNC_NOT_READY')
        statemachine.on_enter('base:disabled:not_ready', 'CNC_NOT_READY')
        statemachine.on_enter('base:disabled:fault', 'CNC_NOT_READY')
        statemachine.on_enter('base:activated', 'ACTIVATE')
        statemachine.on_enter('base:operational', 'OPERATIONAL')
        statemachine.on_enter('base:operational:idle','IDLE')
        statemachine.on_enter('base:operational:cycle_start', 'CYCLING')
        statemachine.on_enter('base:operational:loading', 'LOADING')
        statemachine.on_enter('base:operational:unloading', 'UNLOADING')
        statemachine.on_enter('base:operational:in_transition', 'IN_TRANSITION')
        statemachine.on_exit('base:operational:in_transition', 'EXIT_TRANSITION')

        return statemachine

if __name__ == '__main__':
    cnc = cnc('localhost',7896)
    cnc.superstate.load_time_limit(600)
    cnc.superstate.unload_time_limit(600)
    cnc.superstate.enable()
