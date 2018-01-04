
from material import *
from door import *
from chuck import *


from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time


class adapter(object):

    def __init__(self, value):
        self.interface = value
        

class cnc(object):

    def __init__(self, interface):

        class statemachineModel(object):

            def __init__(self):
                #initializing the interfaces
                self.open_door = interface() 
                self.close_door = interface()
                self.open_chuck = interface()
                self.close_chuck = interface()
                self.material_load = interface()
                self.material_unload = interface()

                self.material_load_interface = MaterialLoad(self)
                self.material_unload_interface = MaterialUnload(self)
                self.open_chuck_interface = OpenChuck(self)
                self.close_chuck_interface = CloseChuck(self)
                self.open_door_interface = OpenDoor(self)
                self.close_door_interface = CloseDoor(self)
                
                self.door_state = "OPEN"
                self.chuck_state = "OPEN"
                self.has_material = False
                self.fail_next = False

                self.availability = "AVAILABLE"
                self.execution = "READY"
                self.controller_mode = "AUTOMATIC"
                
                self.robot_availability = str() #intialized
                self.robot_execution = str()
                self.robot_controller_mode = str() 
                
                self.cycle_time = 4

                self.system = []

                self.system_normal = True

                self.link = "ENABLED"

                self.adapter = adapter

                self.load_time_limit(5 * 60)
                self.unload_time_limit(5 * 60)

                self.load_failed_time_limit(30)
                self.unload_failed_time_limit(30)

                self.events = []

            def CNC_NOT_READY(self):
                self.open_chuck_interface.superstate.DEACTIVATE()
                self.close_chuck_interface.superstate.DEACTIVATE()
                self.open_door_interface.superstate.DEACTIVATE()
                self.close_door_interface.superstate.DEACTIVATE()
                self.material_load_interface.superstate.DEACTIVATE()
                self.material_unload_interface.superstate.DEACTIVATE()

            def ACTIVATE(self):
                if self.controller_mode == "AUTOMATIC" and self.link == "ENABLED" and self.robot_controller_mode =="AUTOMATIC" and self.robot_execution == "ACTIVE" and self.robot_availability == "AVAILABLE":
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

                if self.has_material:
                    self.unloading()
                else:
                    self.loading()

            def IDLE(self):
                if self.has_material:
                    self.material_load_interface.superstate.DEACTIVATE()
                    self.material_unload_interface.superstate.IDLE()

                else:
                    self.material_unload_interface.superstate.DEACTIVATE()
                    self.material_load_interface.superstate.IDLE()

            def CYCLING(self):
                if self.fail_next:
                    self.system.append(['FAULT', 'Cycle failed to start', 'CYCLE'])
                    self.fault()
                    self.fail_next = False

                elif self.door_state != "CLOSED" or self.chuck_state != "CLOSED":
                    self.system.append(['FAULT', 'Door or Chuck in invalid state', 'CYCLE'])
                    self.fault()

                else:
                    self.execution = "ACTIVE"
                    
                    time.sleep(self.cycle_time)
                    self.execution = "READY"

                    self.cnc_execution_ready()

            def LOADING(self):
                if not self.has_material:
                    self.material_unload_interface.superstate.DEACTIVATE()
                    self.material_load_interface.superstate.ACTIVATE()

            def UNLOADING(self):
                if self.has_material:
                    self.material_load_interface.superstate.DEACTIVATE()
                    self.material_unload_interface.superstate.ACTIVATE()

            def EXIT_LOADING(self):
                self.material_load_interface.superstate.DEACTIVATE()
                self.has_material = True #here vs under completed method

            def EXIT_UNLOADING(self):
                self.material_unload_interface.superstate.DEACTIVATE()
                self.has_material = False #here vs under completed method

            #might be useful later. 
            def timer_thread(self, input_time):
                def timer(input_time):
                    time.sleep(input_time)
                thread= Thread(target = timer,args=(input_time,))
                thread.start()                

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

            def interface_type(self, value = None, subtype = None):
                self.interfaceType = value

            def COMPLETED(self):
                if self.interfaceType == "Request":
                    self.complete()

            def FAILED(self):
                if self.interfaceType == "Request":
                    self.failed()
                elif self.interfaceType == "Response":
                    self.fault()


            def event(self, source, comp, name, value, code = None, text = None):
                print "CNC received " + comp + name + value + "from " + source
                self.events.append([source, comp, name, value, code, text])

                action= value.lower()

                if action == "fail":
                    action = "failure"

                if name == "Open":
                    if comp == "DoorInterface":
                        eval('self.open_door_interface.superstate.'+action+'()')
                    elif comp == "ChuckInterface":
                        eval('self.open_chuck_interface.superstate.'+action+'()')

                elif name == "Close":
                    if comp == "DoorInterface":
                        eval('self.close_door_interface.superstate.'+action+'()')
                    elif comp == "ChuckInterface":
                        eval('self.close_chuck_interface.superstate.'+action+'()')
                    
                elif name == "MaterialLoad":
                    eval('self.material_load_interface.superstate.'+action+'()')

                elif name == "MaterialUnload":
                    eval('self.material_unload_interface.superstate.'+action+'()')
                

        self.superstate = statemachineModel()


    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['activated']}, {'name':'operational', 'children':['loading', 'cycle_start', 'unloading', 'idle']}, {'name':'disabled', 'children':['fault', 'not_ready']}]

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

        
        
