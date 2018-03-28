"""
Sample module for implementing a robot that coordinates with a CNC and conveyors.
"""
from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

from .material import MaterialLoadResponse, MaterialUnloadResponse
from .door import OpenDoorRequest, CloseDoorRequest
from .chuck import OpenChuckRequest, CloseChuckRequest

from transitions.extensions import HierarchicalGraphMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, re

class adapter:
    """Adapter skeleton"""
    def __init__(self, value):
        self.interface = value

class interface:
    """Interface skeleton"""
    def __init__(self):
        self._value = ""

class Robot:
    class StateModel:
        """The model for MTConnect behavior in the robot."""
        def __init__(self):
            #initializing the interfaces
            self.open_door = interface()
            self.close_door = interface()
            self.open_chuck = interface()
            self.close_chuck = interface()
            self.material_load = interface()
            self.material_unload = interface()

            self.material_load_interface = MaterialLoadResponse(self)
            self.material_unload_interface = MaterialUnloadResponse(self)
            self.open_chuck_interface = OpenChuckRequest(self)
            self.close_chuck_interface = CloseChuckRequest(self)
            self.open_door_interface = OpenDoorRequest(self)
            self.close_door_interface = CloseDoorRequest(self)

            self.fail_next = False

            #State variables of the robot
            self.availability = "AVAILABLE"
            self.execution = "READY"
            self.controller_mode = "AUTOMATIC"
            self.link = "ENABLED"

            #State variables of the cnc
            #TODO: these should not be here
            self.cnc_availability = "AVAILABLE" #intialized for testing
            self.cnc_execution = "ACTIVE"
            self.cnc_controller_mode = "AUTOMATIC"
            self.cnc_door_state = "OPEN"
            self.cnc_chuck_state = "OPEN"

            self.material_load_interface.superstate.set_processing_time_limit(2)
            self.material_unload_interface.superstate.set_processing_time_limit(2)

            self.material_load_interface.superstate.set_fail_time_limit(2)
            self.material_unload_interface.superstate.set_fail_time_limit(2)

            self.events = []

        def ACTIVATE(self):
            if (self.controller_mode == "AUTOMATIC" and
                self.link == "ENABLED" and
                self.cnc_controller_mode =="AUTOMATIC" and
                self.cnc_execution == "ACTIVE" and
                self.cnc_availability == "AVAILABLE"):
                self.make_operational()
            else:
                self.faulted()

        def OPERATIONAL(self):
            self.open_chuck_interface.superstate.ACTIVATE()
            self.close_chuck_interface.superstate.ACTIVATE()
            self.open_door_interface.superstate.ACTIVATE()
            self.close_door_interface.superstate.ACTIVATE()

        def LOADING(self):
            self.material_load_interface.superstate.ACTIVATE()
            self.material_unload_interface.superstate.DEACTIVATE()

        def UNLOADING(self):
            self.material_unload_interface.superstate.ACTIVATE()
            self.material_load_interface.superstate.DEACTIVATE()

        def LOADING_COMPLETE(self):
            self.material_load_interface.superstate.DEACTIVATE()

        def UNLOADING_COMPLETE(self):
            self.material_unload_interface.superstate.DEACTIVATE()

        def CNC_NOT_READY(self):
            self.open_chuck_interface.superstate.DEACTIVATE()
            self.close_chuck_interface.superstate.DEACTIVATE()
            self.open_door_interface.superstate.DEACTIVATE()
            self.close_door_interface.superstate.DEACTIVATE()
            self.material_load_interface.superstate.DEACTIVATE()
            self.material_unload_interface.superstate.DEACTIVATE()

        def CNC_LOAD_READY(self):
            """Function triggered when the CNC is ready to be loaded"""
            #TODO: verify that it's ok to start loading
            self.loading()

        def CNC_UNLOAD_READY(self):
            """Function triggered when the CNC is ready to be unloaded"""
            #TODO: verify that it's ok to start unloading
            self.unloading()

        def event(self, ev):
            """Process events.

            :type ev: .event.Event
            """
            print('Robot received: ', ev)
            self.events.append(ev)

            if ev.name.startswith('Material'):
                self.material_event(ev)

            elif ev.component.startswith('Controller'):
                self.controller_event(ev)

            elif ev.component.startswith('Device'):
                self.device_event(ev)

            elif ev.source == 'cnc': #other general CNC events
                self.cnc_event(ev)

            else:
                raise(Exception('Unknown event: ' + str(ev)))


        def material_event(self, ev):
            if ev.name == "MaterialLoad":
                if ev.value.lower() == 'ready' and self.state == 'base:operational:idle':
                    self.cnc_material_load_ready()
                else:
                    self.material_load_interface.superstate.event(ev)
            elif ev.name == "MaterialUnload":
                if ev.value.lower() == 'ready' and self.state == 'base:operational:idle':
                    self.cnc_material_unload_ready()
                else:
                    self.material_unload_interface.superstate.event(ev)
            else:
                raise(Exception('Unknown Material event: ' + str(ev)))

        def controller_event(self, ev):
            if ev.name == "ControllerMode":
                if ev.source.lower() == 'cnc':
                    self.controller_mode = ev.value.upper()
                    if ev.value.lower() == 'automatic':
                        self.cnc_controller_mode_automatic()
                    else:
                        raise(Exception('Unknown controller mode: "{}" in event: {}'.format(
                            ev.value.lower(), str(ev))))
            elif ev.name == "Execution":
                if ev.source.lower() == 'cnc':
                    self.execution = ev.value.upper()
                    if ev.value.lower() == 'active':
                        self.cnc_execution_active()
                    else:
                        raise(Exception('Unknown controller mode: "{}" in event: {}'.format(
                            ev.value.lower(), str(ev))))
            else:
                raise(Exception('Unknown Controller event: ' + str(ev)))


        def device_event(self, ev):
            if ev.name == 'System':
                pass
                #exec('self.'+ev.source.lower()+'_system_'+value.lower()+'()')

            elif ev.name == 'Availability':
                if ev.source.lower() == 'robot':
                    self.robot_availability = value.upper()
                #exec('self.'+ev.source.lower()+'_availability_'+value.lower()+'()')

            else:
                raise(Exception('Unknown Device event: ' + str(ev)))

        def cnc_event(self, ev):
            if ev.name == "ChuckState":
                self.cnc_chuck_state = ev.value.upper()
                if self.cnc_chuck_state == "OPEN":
                    self.open_chuck_interface.statemachine.set_state('base:active')
                elif self.cnc_chuck_state == "CLOSED":
                    self.close_chuck_interface.statemachine.set_state('base:not_ready')

            elif ev.name == "DoorState":
                self.cnc_door_state = ev.value.upper()
                if self.cnc_door_state == "OPEN":
                    self.open_door_interface.statemachine.set_state('base:active')
                elif self.cnc_door_state == "CLOSED":
                    self.close_door_interface.statemachine.set_state('base:not_ready')

            else:
                raise(Exception('Unknown CNC event: ' + str(ev)))
        #end StateModel class definition

    def __init__(self):
        self.superstate = Robot.StateModel()
        self.statemachine = self.create_state_machine(self.superstate)

    def draw(self):
        print("Creating robot.png diagram")
        self.statemachine.get_graph().draw('robot.png', prog='dot')

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
                        'children': ['idle', 'loading', 'unloading']
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

            #['cnc_execution_interrupted', 'base', 'base:activated'],
            ['cnc_execution_stopped', 'base', 'base:activated'],
            ['cnc_execution_active', 'base', 'base:activated'],
            ['cnc_execution_ready', 'base', 'base:activated'],
            #['robot_controller_mode_manual_data_input', 'base', 'base:activated'],
            ['cnc_controller_mode_manual', 'base', 'base:activated'],
            ['cnc_controller_mode_automatic', 'base', 'base:activated'],
            ['cnc_availability_available', 'base', 'base:activated'],
            ['cnc_availability_unavailable', 'base', 'base:activated'],
            #['robot_system_warning', 'base', 'base:activated'],
            #['robot_system_normal', 'base', 'base:activated'],
            #['reset_cnc', 'base', 'base:activated'],
            #['enable', 'base', 'base:activated'],
            #['disable', 'base', 'base:activated'],
            #['cnc_controller_mode_manual', 'base', 'base:activated'],
            #['cnc_controller_mode_manual_data_input', 'base', 'base:activated'],

            ['cnc_controller_mode_automatic', 'base:disabled', 'base:activated'],
            ['cnc_material_load_ready', 'base:disabled', 'base:activated'],
            ['cnc_material_unload_ready', 'base:disabled', 'base:activated'],

            #['default', 'base:operational:cycle_start', 'base:operational:cycle_start'],
            #['complete', 'base:operational:loading', 'base:operational:cycle_start'],

            ['safety_violation', 'base', 'base:disabled:soft'],
            ['collision', 'base', 'base:disabled:fault:soft'],
            ['hard_fault', 'base', 'base:disable:fault:hard'],
            ['e_stop', 'base', 'base:disabled:fault:e_stop'],
            ['clear_fault', 'base:disabled:fault', 'base:disabled:not_ready'],

            #['robot_system_fault', 'base', 'base:disabled:fault'],
            #['default', 'base:disabled:fault', 'base:disabled:fault'],
            #['faulted', 'base:activated', 'base:disabled:fault'],
            #['cnc_fault', 'base:operational:cycle_start','base:disabled:fault'],

            #['start', 'base:disabled', 'base:disabled:not_ready'],
            #['default', 'base:disabled:not_ready', 'base:disabled:not_ready'],
            #['default', 'base:disabled', 'base:disabled:not_ready'],
            #['still_not_ready', 'base:activated', 'base:disabled:not_ready'],

            #['plan', 'base:operational:idle', 'base:operational:planning'],
            #['execute', 'base:operational', 'base:operational:moving'],
            #['grab', 'base:operational', 'base:operational:manipulating'],
            #['place', 'base:operational', 'base:operational:manipulating'],
            #['manipulation_done', 'base:operational:manipulating', 'base:operational:idle'],

            ['loading', 'base:operational', 'base:operational:loading'],
            #['default', 'base:operational:loading', 'base:operational:loading'],
            ['loading_complete', 'base:operational:loading', 'base:operational:idle'],

            ['unloading', 'base:operational', 'base:operational:unloading'],
            ['unloading_complete', 'base:operational:unloading', 'base:operational:idle'],
            #['default', 'base:operational:unloading', 'base:operational:unloading'],
            #['cnc_execution_ready', 'base:operational:cycle_start', 'base:operational:unloading'],

            #['failed', 'base:operational:loading', 'base:operational:idle'],
            #['failed', 'base:operational:unloading', 'base:operational:idle'],
            #['start', 'base:operational', 'base:operational:idle'],
            {
                'trigger': 'cnc_material_unload_ready',
                'source': 'base:operational:idle',
                'dest': 'base:operational',
                'after': 'CNC_UNLOAD_READY'
            },
            {
                'trigger': 'cnc_material_load_ready',
                'source': 'base:operational:idle',
                'dest': 'base:operational',
                'after': 'CNC_LOAD_READY'
            },
            #['default', 'base:operational:idle', 'base:operational:idle'],
        ]

        statemachine = Machine(
            model = state_machine_model,
            states = states,
            transitions = transitions,
            initial = 'base',
            ignore_invalid_triggers=True
        )

        #statemachine.on_enter('base:disabled', 'CNC_NOT_READY')
        #statemachine.on_enter('base:disabled:not_ready', 'CNC_NOT_READY')
        #statemachine.on_enter('base:disabled:fault', 'CNC_NOT_READY')
        statemachine.on_enter('base:activated', 'ACTIVATE')
        statemachine.on_enter('base:operational', 'OPERATIONAL')
        #statemachine.on_enter('base:operational:idle','IDLE')
        #statemachine.on_enter('base:operational:cycle_start', 'CYCLING')
        #statemachine.on_enter('base:operational:loading', 'LOADING')
        #statemachine.on_exit('base:operational:loading', 'EXIT_LOADING')
        #statemachine.on_enter('base:operational:unloading', 'UNLOADING')
        #statemachine.on_exit('base:operational:unloading', 'EXIT_UNLOADING')

        return statemachine
