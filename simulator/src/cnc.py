"""
from material import *
from door import *
from chuck import *
"""

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time


class adapter(object):

    def __init__(self, value):
        self.interface = value
        

class cnc(object):

    def __init__(self, adapter):
        #self.open_door = str() #adapter dataitems???
        #self.close_door = str()
        #self.open_chuck = str()
        #self.close_chuck = str()
        #self.material_load = str()
        #self.material_unload = str()
        """
        self.material_load_interface = MaterialLoad
        self.material_unload_interface = MaterialUnload
        self.open_chuck_interface = OpenChuck
        self.close_chuck_interface = CloseChuck
        self.open_door_interface = OpenDoor
        self.close_door_interface = CloseDoor
        """
        
        self.door_state = "OPEN"
        self.chuck_state = "OPEN"
        self.has_material = False
        self.fail_next = False

        self.availability = "AVAILABLE"
        self.execution = "READY"
        self.cycle_time = 4
        
        """
        self.adapter = adapter

        self.load_time_limit(5 * 60)
        self.unload_time_limit(5 * 60)

        self.load_failed_time_limit(30)
        self.unload_failed_time_limit(30)
        """
        class statemachineModel(object):

            def __init__(self, ):
                self.

            def 
        

        self.superstate = statemachineModel()

    def load_time_limit(self, limit):
        self.material_load_interface.processing_time_limit = limit

    def load_failed_time_limit(self, limit):
        self.material_load_interface.fail_time_limit = limit

    def unload_time_limit(self, limit):
        self.material_unload_interface.processing_time_limit = limit

    def unload_failed_time_limit(self, limit):
        self.material_unload_interface.fail_time_limit = limit

    def status(self):
        'state'
        #return all the states. Necessary for the first draft?

    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['cycle_start', 'activated']}, {'name':'operational', 'children':['loading', 'unloading', 'idle']}, {'name':'disabled', 'children':['fault', 'not_ready']}]

        transitions= [['start', 'base', 'disabled'],
                      
                      ['cnc_controller_mode_automatic', 'base', 'base:activated'],
                      ['robot_execution_interrupted', 'base', 'base:activated'],
                      ['robot_execution_stopped', 'base', 'base:activated'],
                      ['robot_execution_active', 'base', 'base:activated'],
                      ['robot_controller_mode_manual_data_input', 'base', 'base:activated'],
                      ['robot_controller_mode_manual', 'base', 'base:activated'],
                      ['robot_controller_mode_automatic', 'base', 'base:activated'],
                      ['cnc_controller_mode_automatic', 'base', 'base:activated'],
                      ['robot_availability_available', 'base', 'base:activated'],
                      ['robot_availability_unavailable', 'base', 'base:activated'],
                      ['robot_system_warning', 'base', 'base:activated'],
                      ['robot_system_normal', 'base', 'base:activated'],
                      ['reset_cnc', 'base', 'base:activated'],
                      ['enable', 'base', 'base:activated'],
                      ['disable', 'base', 'base:activated'],
                      ['cnc_controller_mode_manual', 'base', 'base:activated'],
                      ['cnc_controller_mode_manual_data_input', 'base', 'base:activated'],
                      ['cnc_controller_mode_automatic', 'disabled', 'base:activated'],
                      ['robot_material_load_ready', 'disabled', 'base:activated'],
                      ['robot_material_unload_ready', 'disabled', 'base:activated'],

                      ['default', 'operational:cycle_start', 'operational:cycle_start'],
                      ['unloading', 'operational:loading', 'operational:cycle_start'],

                      ['fault', 'base', 'disabled:fault'],
                      ['robot_system_fault', 'base', 'disabled:fault'],
                      ['default', 'disabled:fault', 'disabled:fault'],
                      ['faulted', 'base:activated', 'disabled:fault'],
                      ['cnc_fault', 'operational:cycle_start','disabled:fault'],
                      
                      ['start', 'disabled', 'disabled:not_ready'],
                      ['default', 'disabled:not_ready', 'disabled:not_ready'],
                      ['default', 'disabled', 'disabled:not_ready'],
                      ['still_not_ready', 'base:activated', 'disabled:not_ready'],

                      ['loading', 'operational', 'operational:loading'],
                      ['default', 'operational:loading', 'operational:loading'],
                      ['complete', 'operational:unloading', 'operational:loading'],

                      ['unloading', 'operational', 'operational:unloading'],
                      ['default', 'operational_unloading', 'operational_unloading'],
                      ['cnc_execution_ready', 'operational:cycle_start', 'operational:unloading'],

                      ['failed', 'operational:loading', 'operational_idle'],
                      ['failed', 'operational:unloading', 'operational_idle'],
                      ['start', 'operational', 'operational_idle'],
                      ['robot_material_unload_ready', 'operational:idle', 'operational_idle'],
                      ['robot_material_load_ready', 'operational:idle', 'operational_idle'],
                      ['default', 'operational:idle', 'operational_idle'],
                      
                      ['make_operational', 'operational', 'base:activated']
      
                      
                      ]

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)            
            
        self.statemachine.on_enter('disabled', 'CNC_NOT_READY')
        self.statemachine.on_enter('disabled:not_ready', 'CNC_NOT_READY')
        self.statemachine.on_enter('disabled:fault', 'CNC_NOT_READY')
        self.statemachine.on_enter('base:activated', 'ACTIVATE')
        self.statemachine.on_enter('operational', 'OPERATIONAL')
        self.statemachine.on_enter('operational:idle','IDLE')
        self.statemachine.on_enter('operational:cycle_start', 'CYCLING')
        self.statemachine.on_enter('operational:loading', 'LOADING')
        self.statemachine.on_exit('operational:loading', 'EXIT_LOADING')
        self.statemachine.on_enter('operational:unloading', 'UNLOADING')
        self.statemachine.on_exit('operational:unloading', 'EXIT_UNLOADING')

        
        
