from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import os, sys

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

class Buffer:

    class StateMachineModel:

        def __init__(self,host,port):

            self.initiate_adapter(host,port)
            self.adapter.start()
            self.initiate_dataitems()

            self.initiate_interfaces()

            self.system = []

            self.load_time_limit(15)
            self.unload_time_limit(15)

            self.next_sequence = str(1)

            self.load_failed_time_limit(2)
            self.unload_failed_time_limit(2)

            self.events = []

            self.master_tasks = {}

            self.device_uuid = "b1"

            self.buffer = []

            #long pull dict for maintaining agent threads for other devices
            self.lp = {}

            self.buffer_size = 100

            self.master_uuid = str()

            self.is_coordinator = False

            self.is_collaborator = False

            self.system_normal = True

            self.has_material = False

            self.fail_next = False

            self.timer_check = str()

            self.initial_execution_state()

            self.set_priority()

            self.initiate_pull_thread()

        def set_priority(self):
            self.priority = None
            self.priority = priority(self, self.buffer_binding)

        def initial_execution_state(self):
            self.execution = {}
            self.execution['cnc1'] = None
            self.execution['cmm1'] = None
            self.execution['b1'] = None
            self.execution['conv1'] = None
            self.execution['r1'] = None

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

            self.buffer_binding = Event('buffer_binding')
            self.adapter.add_data_item(self.buffer_binding)

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
                    """/cnc/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",
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
                    """/cmm/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",
                    from_long_pull
                    )
                )
            self.thread3.start()

        def start_pull(self,addr,request, func, stream = True):

            response = requests.get(addr+request+"&from="+self.next_sequence, stream=stream)
            self.lp[request.split('/')[1]] = None
            self.lp[request.split('/')[1]] = LongPull(response, addr, self)
            self.lp[request.split('/')[1]].long_pull(func)

        def start_pull_asset(self, addr, request, assetId, stream_root):
            response = urllib.request.urlopen(addr+request).read()
            from_long_pull_asset(self, response, stream_root)

        def BUFFER_NOT_READY(self):
            self.material_load_interface.superstate.DEACTIVATE()
            self.material_unload_interface.superstate.DEACTIVATE()

        def ACTIVATE(self):
            if self.mode1.value() == "AUTOMATIC":
                self.make_operational()

            elif self.system_normal:
                self.still_not_ready()

            else:
                self.faulted()

        def buffer_append(self):
            if len(self.buffer)<100:
                self.buffer.append([len(self.buffer)+1])

        def buffer_pop(self):
            if len(self.buffer)>0:
                self.buffer.pop(0)

        def OPERATIONAL(self):
            self.make_idle()

        def IDLE(self):
            if len(self.buffer)>0: self.has_material = True
            else: self.has_material = False

            if self.binding_state_material.value() != "COMMITTED":
                if self.has_material:
                    self.is_coordinator = True
                    self.is_collaborator = False
                    self.material_load_interface.superstate.DEACTIVATE()

                else:
                    self.is_coordinator = False
                    self.is_collaborator = True
                    self.material_unload_interface.superstate.DEACTIVATE()

                if not self.has_material:
                    self.collaborator = collaborator(
                        parent = self,
                        interface = self.binding_state_material,
                        collaborator_name = self.device_uuid
                        )

                    self.collaborator.superstate.task_name = "LoadBuffer"
                    self.collaborator.superstate.unavailable()
                    self.material_load_interface.superstate.IDLE()
                    self.priority.collab_check()

                if self.has_material:

                    if self.master_uuid in self.master_tasks:
                        del self.master_tasks[self.master_uuid]

                    self.master_uuid = self.device_uuid+'_'+str(uuid.uuid4())
                    master_task_uuid = copy.deepcopy(self.master_uuid)
                    self.coordinator_task = "MoveMaterial_3"

                    self.adapter.begin_gather()
                    self.buffer_binding.set_value(master_task_uuid)
                    self.adapter.complete_gather()

                    self.coordinator = coordinator(
                        parent = self,
                        master_task_uuid = master_task_uuid,
                        interface = self.binding_state_material,
                        coordinator_name = self.device_uuid
                        )

                    self.coordinator.superstate.task_name = "UnloadBuffer"
                    self.coordinator.superstate.unavailable()
                    self.material_unload_interface.superstate.IDLE()

        def load_time_limit(self, limit):
            self.material_load_interface.superstate.processing_time_limit = limit

        def load_failed_time_limit(self, limit):
            self.material_load_interface.superstate.fail_time_limit = limit

        def unload_time_limit(self, limit):
            self.material_unload_interface.superstate.processing_time_limit = limit

        def unload_failed_time_limit(self, limit):
            self.material_unload_interface.superstate.fail_time_limit = limit

        def status(self):
            pass

        def interface_type(self, value = None, subtype = None):
            self.interfaceType = value

        def COMPLETED(self):
            if self.interfaceType == "Request":
                self.complete()

        def LOADED(self):
            self.buffer_append()
            self.material_load_interface.superstate.DEACTIVATE()

        def UNLOADED(self):
            self.buffer_pop()
            self.material_unload_interface.superstate.DEACTIVATE()

        def IN_TRANSITION(self):
            if self.buffer:
                self.complete()

        def EXIT_TRANSITION(self):
            pass

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

            if comp == "Task_Collaborator" and self.is_coordinator == True:
                self.coordinator.superstate.event(source, comp, name, value, code, text)


            elif comp == "Coordinator" and self.is_collaborator == True:
                if value.lower() != 'preparing':
                    self.collaborator.superstate.event(source, comp, name, value, code, text)
                    if value.lower() == 'inactive':
                        self.IDLE()

            elif 'SubTask' in name:
                if self.is_coordinator == True:
                    self.coordinator.superstate.event(source, comp, name, value, code, text)

                elif self.is_collaborator == True:
                    self.collaborator.superstate.event(source, comp, name, value, code, text)

            elif name == "MaterialLoad":
                try:
                    if value.lower() == 'ready' and self.state == 'base:operational:idle':
                        eval('self.robot_material_load_ready()')
                    eval('self.material_load_interface.superstate.'+action+'()')

                except Exception as e:
                    print ("Incorrect Event")
                    print (e)

            elif name == "MaterialUnload":
                try:
                    if value.lower() == 'ready' and self.state == 'base:operational:idle':
                        eval('self.robot_material_unload_ready()')
                    eval('self.material_unload_interface.superstate.'+action+'()')

                except Exception as e:
                    print ("Incorrect Event")
                    print (e)

            elif comp == "Controller":
                if name == "ControllerMode":

                    if source == 'Buffer':
                        self.adapter.begin_gather()
                        self.mode1.set_value(value.upper())
                        self.adapter.complete_gather()

                elif name == "Execution":
                    if source == 'Buffer':
                        self.adapter.begin_gather()
                        self.e1.set_value(value.upper())
                        self.adapter.complete_gather()

                    elif text in self.execution:
                        self.execution[text]  = value.lower()

            elif comp == "Device":
                if name == "SYSTEM":
                    eval('self.'+source+'_system_'+value.lower()+'()')

                elif name == "Availability":
                    if source == 'Buffer':
                        self.adapter.begin_gather()
                        self.avail1.set_value(value.upper())
                        self.adapter.complete_gather()


    def __init__(self,host,port):

        self.superstate = Buffer.StateMachineModel(host,port)
        self.statemachine = self.create_statemachine(self.superstate)

    def draw(self):
        print ("Creating Buffer.png diagram")
        self.statemachine.get_graph().draw('Buffer.png', prog='dot')

    def create_statemachine(self, state_machine_model):
        NestedState.separator = ':'

        states = [
            {
                'name':
                'base',
                'children':[
                    'activated',
                    {
                        'name':'operational',
                        'children':[
                            'loading',
                            'in_transition',
                            'unloading',
                            'idle'
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

            ['enable', 'base', 'base:activated'],
            ['disable', 'base', 'base:activated'],
            ['Buffer_controller_mode_manual', 'base', 'base:activated'],
            ['Buffer_controller_mode_manual_data_input', 'base', 'base:activated'],
            ['Buffer_controller_mode_automatic', 'base', 'base:activated'],
            ['robot_material_load_ready', 'base:disabled', 'base:activated'],
            ['robot_material_unload_ready', 'base:disabled', 'base:activated'],

            ['make_idle', 'base:operational', 'base:operational:idle'],

            ['fault', 'base', 'base:disabled:fault'],
            ['robot_system_fault', 'base', 'base:disabled:fault'],
            ['default', 'base:disabled:fault', 'base:disabled:fault'],
            ['faulted', 'base:activated', 'base:disabled:fault'],

            ['start', 'base:disabled', 'base:disabled:not_ready'],
            ['default', 'base:disabled:not_ready', 'base:disabled:not_ready'],
            ['default', 'base:disabled', 'base:disabled:not_ready'],
            ['still_not_ready', 'base:activated', 'base:disabled:not_ready'],

            ['loading', 'base:operational', 'base:operational:loading'],
            ['default', 'base:operational:loading', 'base:operational:loading'],

            ['unloading', 'base:operational', 'base:operational:unloading'],
            ['default', 'base:operational:unloading', 'base:operational:unloading'],

            ['failed', 'base:operational:loading', 'base:operational:idle'],

            ['failed', 'base:operational:unloading', 'base:operational:idle'],


            ['complete', 'base:operational:in_transition', 'base:operational:idle'],

            {
                'trigger':'complete',
                'source':'base:operational:unloading',
                'dest':'base:operational:in_transition',
                'before':'UNLOADED'
            },

            {
                'trigger':'complete',
                'source':'base:operational:loading',
                'dest':'base:operational:in_transition',
                'before':'LOADED'
            },


            ['start', 'base:operational', 'base:operational:idle'],

            {
                'trigger':'robot_material_unload_ready',
                'source':'base:operational:idle',
                'dest':'base:operational:unloading'
            },

            {
                'trigger':'robot_material_load_ready',
                'source':'base:operational:idle',
                'dest':'base:operational:loading'
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

        statemachine.on_enter('base:disabled', 'BUFFER_NOT_READY')
        statemachine.on_enter('base:activated', 'ACTIVATE')
        statemachine.on_enter('base:operational', 'OPERATIONAL')
        statemachine.on_enter('base:operational:idle','IDLE')
        statemachine.on_enter('base:operational:in_transition', 'IN_TRANSITION')
        statemachine.on_exit('base:operational:in_transition', 'EXIT_TRANSITION')

        return statemachine


if __name__ == '__main__':
    b = Buffer('localhost',7696)
    b.superstate.load_time_limit(600)
    b.superstate.unload_time_limit(600)
    b.superstate.enable()
