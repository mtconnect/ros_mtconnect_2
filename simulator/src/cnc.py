
from material import *
from door import *
from chuck import *


from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, re


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
                
                self.robot_availability = "AVAILABLE" #intialized for testing
                self.robot_execution = "ACTIVE"
                self.robot_controller_mode = "AUTOMATIC"
                
                self.cycle_time = 2.0

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
                    self.system.append(['cnc', 'Device', 'SYSTEM', 'FAULT', 'Cycle failed to start', 'CYCLE'])
                    self.cnc_fault()
                    self.fail_next = False

                elif self.close_door_interface.superstate.response_state != "CLOSED" or self.close_chuck_interface.superstate.response_state != "CLOSED":
                    self.system.append(['cnc', 'Device', 'SYSTEM', 'FAULT', 'Door or Chuck in invalid state', 'CYCLE'])
                    self.cnc_fault()

                else:
                    self.execution = "ACTIVE"
                    def func(self = self):
                        self.execution = "READY"
                        self.cnc_execution_ready()
                    timer_cycling = Timer(self.cycle_time,func)
                    timer_cycling.start()
                    
                    

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

            def EXIT_UNLOADING(self):
                self.material_unload_interface.superstate.DEACTIVATE()

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
                elif "Response" and "chuck" in self.interfaceType:
                    if "open" in self.interfaceType:
                        self.has_material = False
                        self.chuck_state = "OPEN"
                    elif "close" in self.interfaceType:
                        self.has_material = True
                        self.chuck_state = "CLOSED"

                elif "Response" and "door" in self.interfaceType:
                    if "open" in self.interfaceType:
                        self.door_state = "OPEN"
                    elif "close" in self.interfaceType:
                        self.door_state = "CLOSED"
                    
            
            def EXITING_IDLE(self):
                if self.has_material:
                    self.unloading()
                else:
                    self.loading()
              
            def LOADED(self):
                self.has_material = True

            def UNLOADED(self):
                self.has_material = False

            def FAILED(self):
                if "Request" in self.interfaceType:
                    self.failed()
                elif "Response" in self.interfaceType:
                    self.fault()

            def void(self):
                pass


            def event(self, source, comp, name, value, code = None, text = None):
                print "CNC received " + comp + " " + name + " " + value + " from " + source
                self.events.append([source, comp, name, value, code, text])

                action= value.lower()

                if action == "fail":
                    action = "failure"

                if name == "Open":
                    if comp == "DoorInterface":
                        exec('self.open_door_interface.superstate.'+action+'()')
                        
                    elif comp == "ChuckInterface":
                        exec('self.open_chuck_interface.superstate.'+action+'()')

                elif name == "Close":
                    if comp == "DoorInterface":
                        exec('self.close_door_interface.superstate.'+action+'()')

                    elif comp == "ChuckInterface":
                        exec('self.close_chuck_interface.superstate.'+action+'()')

                elif name == "MaterialLoad":
                    if value.lower() == 'ready':
                        exec('self.robot_material_load_ready()')
                        
                    exec('self.material_load_interface.superstate.'+action+'()')

                elif name == "MaterialUnload":
                    if value.lower() == 'ready':
                        exec('self.robot_material_unload_ready()')
                    exec('self.material_unload_interface.superstate.'+action+'()')

                elif comp == "Controller":
                    
                    if name == "ControllerMode":
                        if source.lower() == 'cnc':
                            self.controller_mode = value.upper()
                        elif source.lower() == 'robot':
                            self.robot_controller_mode = value.upper()
                        exec('self.'+source.lower()+'_controller_mode_'+value.lower()+'()')

                    elif name == "Execution":
                        if source.lower() == 'cnc':
                            self.execution = value.upper()
                        elif source.lower() == 'robot':
                            self.robot_execution = value.upper()
                        exec('self.'+source.lower()+'_execution_'+value.lower()+'()')

                elif comp == "Device":

                    if name == "SYSTEM":
                        exec('self.'+source.lower()+'_system_'+value.lower()+'()')

                    elif name == "Availability":
                        if source.lower() == 'cnc':
                            self.availability = value.upper()
                        elif source.lower() == 'robot':
                            self.robot_availability = value.upper()
                        exec('self.'+source.lower()+'_availability_'+value.lower()+'()')

                elif source == "cnc" and name == "ChuckState":
                    self.chuck_state = value.upper()
                    if self.chuck_state == "OPEN":
                        self.open_chuck_interface.statemachine.set_state('base:active')
                    elif self.chuck_state == "CLOSED":
                        self.close_chuck_interface.statemachine.set_state('base:not_ready')
                    

                elif source == "cnc" and name == "DoorState":
                    self.door_state = value.upper()
                    if self.door_state == "OPEN":
                        self.open_door_interface.statemachine.set_state('base:active')
                    elif self.door_state == "CLOSED":
                        self.close_door_interface.statemachine.set_state('base:not_ready')
                    
      

        self.superstate = statemachineModel()


    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['activated',{'name':'operational', 'children':['loading', 'cycle_start', 'unloading', 'idle']}, {'name':'disabled', 'children':['fault', 'not_ready']}]} ]

        transitions= [['start', 'base', 'base:disabled'],
                      
                      ['cnc_controller_mode_automatic', 'base', 'base:activated'],
                      ['robot_execution_interrupted', 'base', 'base:activated'],
                      ['robot_execution_stopped', 'base', 'base:activated'],
                      ['robot_execution_active', 'base', 'base:activated'],
                      ['robot_execution_ready', 'base', 'base:activated'],
                      ['robot_controller_mode_manual_data_input', 'base', 'base:activated'],
                      ['robot_controller_mode_manual', 'base', 'base:activated'],
                      ['robot_controller_mode_automatic', 'base', 'base:activated'],
                      ['robot_availability_available', 'base', 'base:activated'],
                      ['robot_availability_unavailable', 'base', 'base:activated'],
                      ['robot_system_warning', 'base', 'base:activated'],
                      ['robot_system_normal', 'base', 'base:activated'],
                      ['reset_cnc', 'base', 'base:activated'],
                      ['enable', 'base', 'base:activated'],
                      ['disable', 'base', 'base:activated'],
                      ['cnc_controller_mode_manual', 'base', 'base:activated'],
                      ['cnc_controller_mode_manual_data_input', 'base', 'base:activated'],
                      ['cnc_controller_mode_automatic', 'base:disabled', 'base:activated'],
                      ['robot_material_load_ready', 'base:disabled', 'base:activated'],
                      ['robot_material_unload_ready', 'base:disabled', 'base:activated'],

                      ['default', 'base:operational:cycle_start', 'base:operational:cycle_start'],
                      ['complete', 'base:operational:loading', 'base:operational:cycle_start'],

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
                      ['complete', 'base:operational:unloading', 'base:operational:loading'],
                    
                      ['unloading', 'base:operational', 'base:operational:unloading'],
                      ['default', 'base:operational:unloading', 'base:operational:unloading'],
                      ['cnc_execution_ready', 'base:operational:cycle_start', 'base:operational:unloading'],

                      ['failed', 'base:operational:loading', 'base:operational:idle'],
                      ['failed', 'base:operational:unloading', 'base:operational:idle'],
                      ['start', 'base:operational', 'base:operational:idle'],
                      {'trigger':'robot_material_unload_ready','source':'base:operational:idle','dest':'base:operational', 'after':'EXITING_IDLE'},
                      {'trigger':'robot_material_load_ready','source':'base:operational:idle','dest':'base:operational', 'after':'EXITING_IDLE'},
                      ['default', 'base:operational:idle', 'base:operational:idle'],
                      
                      ['make_operational', 'base:activated', 'base:operational']
      
                      
                      ]

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)            
            
        self.statemachine.on_enter('base:disabled', 'CNC_NOT_READY')
        self.statemachine.on_enter('base:disabled:not_ready', 'CNC_NOT_READY')
        self.statemachine.on_enter('base:disabled:fault', 'CNC_NOT_READY')
        self.statemachine.on_enter('base:activated', 'ACTIVATE')
        self.statemachine.on_enter('base:operational', 'OPERATIONAL')
        self.statemachine.on_enter('base:operational:idle','IDLE')
        self.statemachine.on_enter('base:operational:cycle_start', 'CYCLING')
        self.statemachine.on_enter('base:operational:loading', 'LOADING')
        self.statemachine.on_exit('base:operational:loading', 'EXIT_LOADING')
        self.statemachine.on_enter('base:operational:unloading', 'UNLOADING')
        self.statemachine.on_exit('base:operational:unloading', 'EXIT_UNLOADING')

        
        
