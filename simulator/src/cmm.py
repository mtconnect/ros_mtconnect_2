from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import os, sys

from . import cmm_bridge

from .interfaces.material import *

from .collaborationModel.collaborator import *
from .collaborationModel.coordinator import *
from .collaborationModel.priority import priority
from .collaborationModel.archetypeToInstance import archetypeToInstance
from .collaborationModel.from_long_pull import from_long_pull, from_long_pull_asset

from .adapter.mtconnect_adapter import Adapter
from .adapter.long_pull import LongPull
from .adapter.data_item import Event, SimpleCondition, Sample, ThreeDSample

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, re, copy
import xml.etree.ElementTree as ET
import requests, urllib, uuid


class cmm:

    class StateMachineModel:

        def __init__(self, host, port, sim, cell_part):

            self.initiate_adapter(host,port)
            self.adapter.start()

            self.sim = sim

            self.cell_part = cell_part

            self.initiate_dataitems()

            self.initiate_interfaces()

            self.system = []

            self.cycle_time = 10

            self.load_time_limit(20)
            self.unload_time_limit(20)

            self.next_sequence = str(1)

            self.load_failed_time_limit(2)
            self.unload_failed_time_limit(2)

            self.events = []

            self.master_tasks ={}

            self.device_uuid = "cmm1"

            self.master_uuid = str()

            self.is_coordinator = False

            self.is_collaborator = False

            self.system_normal = True

            self.has_material = False

            self.fail_next = False

            self.part_quality = None

            #List for part's expected quality sequence; IMTS
            self.pt_ql_seq = []

            #long pull dict for maintaining agent threads for other devices
            self.lp = {}

            self.part_quality_sequence()

            self.initial_execution_state()

            self.set_priority()

            self.initiate_cmm_client()

            self.initiate_pull_thread()

        def set_priority(self):
            self.priority = None
            self.priority = priority(self, self.cmm_binding)

        def part_quality_sequence(self):
            self.pt_ql_seq = []
            self.pt_ql_seq.append(['good',False])
            self.pt_ql_seq.append(['bad',False])
            self.pt_ql_seq.append(['rework', False])
            self.pt_ql_seq.append(['good', False])
            self.pt_ql_seq.append(['reworked', False])

        def part_quality_next(self, index = None):
            if index != None:
                self.pt_ql_seq[index][1] = True

            else:
                output = None
                for i,x in enumerate(self.pt_ql_seq):
                    if not x[1]:
                        output = [i,x[0]]
                        self.pt_ql_seq[i][1] = True
                        break
                if output:
                    return output

                else:
                    self.part_quality_sequence()
                    return self.part_quality_next()

        def initial_execution_state(self):
            self.execution = {}
            self.execution['cnc1'] = None
            self.execution['cmm1'] = None
            self.execution['b1'] = None
            self.execution['conv1'] = None
            self.execution['r1'] = None

        def initiate_cmm_client(self):
            if not self.sim:
                configFile = open('configFiles/clients.cfg','r')
                device = json.loads(configFile.read())['devices'][self.device_uuid]
                self.cmm_client = cmm_bridge.hexagonClient(
                    str(device['host']),
                    int(device['port']),
                    int(device['port'])
                    )
                self.cmm_client.connect()

        def initiate_interfaces(self):
            self.material_load_interface = MaterialLoad(self)
            self.material_unload_interface = MaterialUnload(self)

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

            self.cmm_binding = Event('cmm_binding')
            self.adapter.add_data_item(self.cmm_binding)

            self.material_load = Event('material_load')
            self.adapter.add_data_item(self.material_load)

            self.material_unload = Event('material_unload')
            self.adapter.add_data_item(self.material_unload)

        def initiate_dataitems(self):

            self.adapter.begin_gather()

            self.avail1.set_value("AVAILABLE")
            self.e1.set_value("READY")
            self.mode1.set_value("AUTOMATIC")
            self.binding_state_material.set_value("INACTIVE")
            self.material_load.set_value("NOT_READY")
            self.material_unload.set_value("NOT_READY")

            self.adapter.complete_gather()

        def initiate_pull_thread(self):
            #Pull MTConnect data from other devices

            self.thread = Thread(
                target = self.start_pull,
                args=(
                    "http://localhost:5000",
                    """/conv/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",
                    from_long_pull
                    )
                )
            self.thread.start()

            self.thread2 = Thread(
                target = self.start_pull,
                args=(
                    "http://localhost:5000",
                    """/robot/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",
                    from_long_pull
                    )
                )
            self.thread2.start()

            self.thread3 = Thread(
                target = self.start_pull,
                args=(
                    "http://localhost:5000",
                    """/buffer/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",
                    from_long_pull
                    )
                )
            self.thread3.start()

            self.thread4 = Thread(
                target = self.start_pull,
                args=(
                    "http://localhost:5000",
                    """/cnc/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",
                    from_long_pull
                    )
                )
            self.thread4.start()

        def start_pull(self,addr,request, func, stream = True):

            response = requests.get(addr+request+"&from="+self.next_sequence, stream=stream)
            self.lp[request.split('/')[1]] = None
            self.lp[request.split('/')[1]] = LongPull(response, addr, self)
            self.lp[request.split('/')[1]].long_pull(func)

        def start_pull_asset(self, addr, request, assetId, stream_root):
            response = urllib.request.urlopen(addr+request).read()
            from_long_pull_asset(self, response, stream_root)

        def CMM_NOT_READY(self):
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

            if self.has_material:
                self.unloading()

                self.is_coordinator = True
                self.is_collaborator = False

                if self.master_uuid in self.master_tasks:
                    del self.master_tasks[self.master_uuid]

                self.master_uuid = self.device_uuid+'_'+str(uuid.uuid4())
                master_task_uuid = copy.deepcopy(self.master_uuid)

                self.adapter.begin_gather()
                self.cmm_binding.set_value(master_task_uuid)
                self.adapter.complete_gather()

                self.part_quality = self.part_quality_next()[1]

                if self.part_quality:
                    if self.part_quality == 'rework':
                        self.coordinator_task = "MoveMaterial_5"
                    else:
                        self.coordinator_task = "MoveMaterial_4"+"_"+self.part_quality

                    self.coordinator = coordinator(
                        parent = self,
                        master_task_uuid = master_task_uuid,
                        interface = self.binding_state_material,
                        coordinator_name = self.device_uuid
                        )

                    self.coordinator.superstate.task_name = "UnloadCmm"

                    self.coordinator.superstate.unavailable()

            elif self.has_material == False:
                self.loading()

                self.is_coordinator = False
                self.is_collaborator = True
                self.collaborator = collaborator(
                    parent = self,
                    interface = self.binding_state_material,
                    collaborator_name = self.device_uuid
                    )

                self.collaborator.superstate.task_name = "LoadCmm"
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
                self.system.append(['cmm', 'Device', 'SYSTEM', 'FAULT', 'Cycle failed to start', 'CYCLE'])
                self.cmm_fault()
                self.fail_next = False

            else:
                self.adapter.begin_gather()
                self.e1.set_value("ACTIVE")
                self.adapter.complete_gather()

                def func(self = self):
                    self.cmm_execution_ready()
                    self.is_coordinator = True
                    self.is_collaborator = False

                    if self.master_uuid in self.master_tasks:
                        del self.master_tasks[self.master_uuid]
                    self.master_uuid = self.device_uuid+'_'+str(uuid.uuid4())
                    master_task_uuid = copy.deepcopy(self.master_uuid)

                    self.adapter.begin_gather()
                    self.cmm_binding.set_value(master_task_uuid)
                    self.adapter.complete_gather()

                    self.part_quality = self.part_quality_next()[1]
                    if self.part_quality:
                        if self.part_quality == 'rework':
                            self.coordinator_task = "MoveMaterial_5"
                        else:
                            self.coordinator_task = "MoveMaterial_4"+"_"+self.part_quality

                        self.coordinator = coordinator(
                            parent = self,
                            master_task_uuid = master_task_uuid,
                            interface = self.binding_state_material,
                            coordinator_name = self.device_uuid
                            )

                        self.coordinator.superstate.task_name = "UnloadCmm"
                        self.coordinator.superstate.unavailable()

                    self.adapter.begin_gather()
                    self.e1.set_value("READY")
                    self.adapter.complete_gather()


                if self.sim:
                    if not self.cycle_time:
                        func()
                    else:
                        timer_cycling = Timer(self.cycle_time,func)
                        timer_cycling.daemon = True
                        timer_cycling.start()
                else:

                    if self.part_quality in [None, 'good', 'reworked']:
                        cycle = self.cmm_client.load_run_pgm(taskcmm.startProgramA)

                    elif self.part_quality == 'bad':
                        cycle = self.cmm_client.load_run_pgm(taskcmm.startProgramB)

                    elif self.part_quality == 'rework':
                        cycle = self.cmm_client.load_run_pgm(taskcmm.startProgramC)

                    time.sleep(30)
                    status = (self.cmm_client.load_run_pgm(taskcmm.getStatus)).lower()

                    print ("State before Completion",status)
                    while ('good' or 'bad' or 'rework') not in status:
                        status = (self.cmm_client.load_run_pgm(taskcmm.getStatus)).lower()
                        time.sleep(3)

                    print ("State after completion",status)
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
                self.cmm_binding.set_value(master_task_uuid)
                self.adapter.complete_gather()

                self.part_quality = self.part_quality_next()[1]

                if self.part_quality:
                    if self.part_quality == 'rework':
                        self.coordinator_task = "MoveMaterial_5"
                    else:
                        self.coordinator_task = "MoveMaterial_4"+"_"+self.part_quality


                    self.coordinator = coordinator(
                        parent = self,
                        master_task_uuid = master_task_uuid,
                        interface = self.binding_state_material,
                        coordinator_name = self.device_uuid
                        )

                    self.coordinator.superstate.task_name = "UnloadCmm"
                    self.coordinator.superstate.unavailable()

            else:
                self.loading()
                self.is_coordinator = False
                self.is_collaborator = True
                self.collaborator = collaborator(
                    parent = self,
                    interface = self.binding_state_material,
                    collaborator_name = self.device_uuid
                    )

                self.collaborator.superstate.task_name = "LoadCmm"
                self.collaborator.superstate.unavailable()
                self.priority.collab_check()

        def LOADED(self):
            self.has_material = True
            self.material_load_interface.superstate.DEACTIVATE()

        def UNLOADED(self):
            self.has_material = False
            self.material_unload_interface.superstate.DEACTIVATE()

        def IN_TRANSITION(self):
            pass

        def EXIT_TRANSITION(self):
            if self.has_material == False:
                self.is_coordinator = False
                self.is_collaborator = True
                self.collaborator = collaborator(
                    parent = self,
                    interface = self.binding_state_material,
                    collaborator_name = self.device_uuid
                    )

                self.collaborator.superstate.task_name = "LoadCmm"
                self.collaborator.superstate.unavailable()
                self.priority.collab_check()


        def FAILED(self):
            if "Request" in self.interfaceType:
                self.failed()

        def void(self):
            pass


        def event(self, source, comp, name, value, code = None, text = None):

            self.events.append([source, comp, name, value, code, text])

            action= value.lower()

            if action == "fail":
                action = "failure"

            if comp == "Coordinator" and value.lower() == 'preparing':
                self.priority.event_list([source, comp, name, value, code, text])

            #tool_change: to be updated
            elif self.is_coordinator and self.binding_state_material.value().lower() == "committed" and text == 'r1':
                if value.lower() == 'inactive' and 'ToolChange' in str(self.master_tasks[self.master_uuid]):
                    self.master_tasks[self.master_uuid]['coordinator'][self.device_uuid]['SubTask']['cnc1'][1] = 'COMPLETE'
                    self.coordinator.superstate.task.superstate.success()
                    self.complete()


            if comp == "Task_Collaborator" and action!='unavailable':
                self.coordinator.superstate.event(source, comp, name, value, code, text)

            elif comp == "Coordinator" and action!='unavailable':
                if value.lower() != 'preparing':
                    self.collaborator.superstate.event(source, comp, name, value, code, text)

            elif 'SubTask' in name and action!='unavailable':
                if self.is_coordinator == True:
                    self.coordinator.superstate.event(source, comp, name, value, code, text)

                elif self.is_collaborator == True:
                    self.collaborator.superstate.event(source, comp, name, value, code, text)

            elif name == "MaterialLoad" and action!='unavailable':
                try:
                    if value.lower() == 'ready' and self.state == 'base:operational:idle':
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

            elif comp == "Controller":
                if name == "ControllerMode":
                    if source.lower() == 'cmm':
                        self.adapter.begin_gather()
                        self.mode1.set_value(value.upper())
                        self.adapter.complete_gather()

                elif name == "Execution":
                    if source.lower() == 'cmm':
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
                    if source.lower() == 'cmm':
                        self.adapter.begin_gather()
                        self.avail1.set_value(value.upper())
                        self.adapter.complete_gather()


    def __init__(self,host,port, sim = True, cell_part = None):
        
        self.superstate = cmm.StateMachineModel(host, port, sim, cell_part)
        self.statemachine = self.create_statemachine(self.superstate)

    def draw(self):
        print ("Creating cmm.png diagram")
        self.statemachine.get_graph().draw('cmm.png', prog='dot')

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
                            'fault',
                            'not_ready'
                            ]
                    }
                ]
            }
        ]

        transitions = [
            ['start', 'base', 'base:disabled'],

            ['cmm_controller_mode_automatic', 'base', 'base:activated'],

            ['reset_cmm', 'base', 'base:activated'],
            ['enable', 'base', 'base:activated'],
            ['disable', 'base', 'base:activated'],
            ['cmm_controller_mode_manual', 'base', 'base:activated'],
            ['cmm_controller_mode_manual_data_input', 'base', 'base:activated'],
            ['cmm_controller_mode_automatic', 'base:disabled', 'base:activated'],
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

            ['fault', 'base', 'base:disabled:fault'],
            ['robot_system_fault', 'base', 'base:disabled:fault'],
            ['default', 'base:disabled:fault', 'base:disabled:fault'],
            ['faulted', 'base:activated', 'base:disabled:fault'],
            ['cmm_fault', 'base:operational:cycle_start','base:disabled:fault'],

            ['start', 'base:disabled', 'base:disabled:not_ready'],
            ['default', 'base:disabled:not_ready', 'base:disabled:not_ready'],
            ['default', 'base:disabled', 'base:disabled:not_ready'],
            ['still_not_ready', 'base:activated', 'base:disabled:not_ready'],

            ['loading', 'base:operational', 'base:operational:loading'],
            ['default', 'base:operational:loading', 'base:operational:loading'],

            {
                'trigger':'complete',
                'source':'base:operational:unloading',
                'dest':'base:operational:in_transition',
                'before':'UNLOADED'
            },

            ['unloading', 'base:operational', 'base:operational:unloading'],
            ['default', 'base:operational:unloading', 'base:operational:unloading'],
            ['cmm_execution_ready', 'base:operational:cycle_start', 'base:operational:unloading'],

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

        statemachine.on_enter('base:disabled', 'CMM_NOT_READY')
        statemachine.on_enter('base:disabled:not_ready', 'CMM_NOT_READY')
        statemachine.on_enter('base:disabled:fault', 'CMM_NOT_READY')
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
    cmm = cmm('localhost',7596)
    cmm.superstate.load_time_limit(600)
    cmm.superstate.unload_time_limit(600)
    cmm.superstate.enable()
