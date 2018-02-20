
from material import *

from collaborator import *
from coordinator import *

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, re, copy


class adapter(object):

    def __init__(self, value = None):
        self.interface = value
        

class inputConveyor(object):

    def __init__(self, interface):

        class statemachineModel(object):

            def __init__(self):
                #initializing the interfaces
                self.material_load = interface()
                self.material_unload = interface()

                self.material_load_interface = MaterialLoad(self)
                self.material_unload_interface = MaterialUnload(self)
                
                self.has_material = False
                self.fail_next = False

                self.availability = "AVAILABLE"
                self.execution = "READY"
                self.controller_mode = "AUTOMATIC"
                
                self.robot_availability = "AVAILABLE" #intialized for testing
                self.robot_execution = "ACTIVE"
                self.robot_controller_mode = "AUTOMATIC"
                
                self.system = []

                self.system_normal = True

                self.link = "ENABLED"

                self.adapter = adapter

                self.load_time_limit(2)
                self.unload_time_limit(2)

                self.load_failed_time_limit(2)
                self.unload_failed_time_limit(2)

                self.events = []

                self.master_tasks = {}

                self.master_uuid = 1

                self.binding_state = "INACTIVE"

                self.iscoordinator = False
                self.iscollaborator = False
                

            def InputConveyor_NOT_READY(self):
                self.material_load_interface.superstate.DEACTIVATE()
                self.material_unload_interface.superstate.DEACTIVATE()

            def ACTIVATE(self):
                if self.controller_mode == "AUTOMATIC":
                    self.make_operational()

                elif self.system_normal:
                    self.still_not_ready()

                else:
                    self.faulted()

            def OPERATIONAL(self):

                if self.has_material:
                    self.unloading()
                    master_task_uuid = copy.deepcopy(self.master_uuid)
                    self.iscoordinator = True
                    self.iscollaborator = False
                    self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = interface , coordinator_name = self.master_tasks[master_task_uuid]['coordinator'].keys()[0])
                    self.coordinator.create_statemachine()
                    self.coordinator.superstate.task_name = "MaterialUnload"
                    
                    self.coordinator.superstate.unavailable()
                else:
                    print "Waiting for a part to arrive!"


            def IDLE(self):
                if self.has_material:
                    self.material_load_interface.superstate.DEACTIVATE()
                    self.material_unload_interface.superstate.IDLE()

                else:
                    print "Waiting for a part to arrive!"
                    self.material_unload_interface.superstate.DEACTIVATE()
                    self.material_load_interface.superstate.IDLE()

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

            def load_time_limit(self, limit):
                self.material_load_interface.superstate.processing_time_limit = limit

            def load_failed_time_limit(self, limit):
                self.material_load_interface.superstate.fail_time_limit = limit

            def unload_time_limit(self, limit):
                self.material_unload_interface.superstate.processing_time_limit = limit

            def unload_failed_time_limit(self, limit):
                self.material_unload_interface.superstate.fail_time_limit = limit

            def status(self):
                'state'
                #return all the states. Necessary for the first draft?

            def interface_type(self, value = None, subtype = None):
                self.interfaceType = value

            def COMPLETED(self):
                if self.interfaceType == "Request":
                    self.complete()
            
            def EXITING_IDLE(self):
                if self.has_material:
                    self.unloading()
                    master_task_uuid = copy.deepcopy(self.master_uuid)
                    self.iscoordinator = True
                    self.iscollaborator = False
                    self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = interface , coordinator_name = self.master_tasks[master_task_uuid]['coordinator'].keys()[0])
                    self.coordinator.create_statemachine()
                    self.coordinator.superstate.task_name = "MaterialUnload"
                    
                    self.coordinator.superstate.unavailable()
                
              
            def LOADED(self):
                self.has_material = True

            def UNLOADED(self):
                self.has_material = False

            def FAILED(self):
                if "Request" in self.interfaceType:
                    self.failed()

            def void(self):
                pass


            def event(self, source, comp, name, value, code = None, text = None):
                print "InputConveyor received " + comp + " " + name + " " + value + " from " + source
                self.events.append([source, comp, name, value, code, text])

                action= value.lower()

                if action == "fail":
                    action = "failure"

                if comp == "Task_Collaborator":
                    self.coordinator.superstate.event(source, comp, name, value, code, text)

                elif comp == "Coordinator":
                    self.collaborator.superstate.event(source, comp, name, value, code, text)

                elif 'SubTask' in name:
                    if self.iscoordinator == True:
                        self.coordinator.superstate.event(source, comp, name, value, code, text)

                    elif self.iscollaborator == True:
                        self.collaborator.superstate.event(source, comp, name, value, code, text)

                elif name == "MaterialLoad":
                    if value.lower() == 'ready' and self.state == 'base:operational:idle':
                        eval('self.robot_material_load_ready()')
                    else:
                        eval('self.material_load_interface.superstate.'+action+'()')

                elif name == "MaterialUnload":
                    if value.lower() == 'ready' and self.state == 'base:operational:idle':
                        eval('self.robot_material_unload_ready()')
                    eval('self.material_unload_interface.superstate.'+action+'()')

                elif comp == "Controller":
                    
                    if name == "ControllerMode":
                        if source == 'inputConveyor':
                            self.controller_mode = value.upper()
                        elif source == 'robot':
                            self.robot_controller_mode = value.upper()
                        eval('self.'+source+'_controller_mode_'+value.lower()+'()')

                    elif name == "Execution":
                        if source == 'inputConveyor':
                            self.execution = value.upper()
                        elif source == 'robot':
                            self.robot_execution = value.upper()
                        eval('self.'+source+'_execution_'+value.lower()+'()')

                elif comp == "Device":

                    if name == "SYSTEM":
                        eval('self.'+source+'_system_'+value.lower()+'()')

                    elif name == "Availability":
                        if source == 'inputConveyor':
                            self.availability = value.upper()
                        elif source == 'robot':
                            self.robot_availability = value.upper()
                        eval('self.'+source+'_availability_'+value.lower()+'()')

                     

        self.superstate = statemachineModel()

    def draw(self):
        print "Creating inputConveyor.png diagram"
        self.statemachine.get_graph().draw('inputConveyor.png', prog='dot')

    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['activated',{'name':'operational', 'children':['loading', 'unloading', 'idle']}, {'name':'disabled', 'children':['fault', 'not_ready']}]} ]

        transitions= [['start', 'base', 'base:disabled'],
                      
                      ['inputConveyor_controller_mode_automatic', 'base', 'base:activated'],
                      
                      ['enable', 'base', 'base:activated'],
                      ['disable', 'base', 'base:activated'],
                      ['inputConveyor_controller_mode_manual', 'base', 'base:activated'],
                      ['inputConveyor_controller_mode_manual_data_input', 'base', 'base:activated'],
                      ['inputConveyor_controller_mode_automatic', 'base', 'base:activated'],
                      ['robot_material_load_ready', 'base:disabled', 'base:activated'],
                      ['robot_material_unload_ready', 'base:disabled', 'base:activated'],

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
                      ['complete', 'base:operational:unloading', 'base:operational:idle'],
                      
                      ['start', 'base:operational', 'base:operational:idle'],
                      {'trigger':'robot_material_unload_ready','source':'base:operational:idle','dest':'base:operational', 'after':'EXITING_IDLE'},
                      {'trigger':'robot_material_load_ready','source':'base:operational:idle','dest':'base:operational', 'after':'EXITING_IDLE'},
                      ['default', 'base:operational:idle', 'base:operational:idle'],
                      
                      ['make_operational', 'base:activated', 'base:operational']
      
                      
                      ]

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)            
            
        self.statemachine.on_enter('base:disabled', 'InputConveyor_NOT_READY')
        self.statemachine.on_enter('base:disabled:not_ready', 'InputConveyor_NOT_READY')
        self.statemachine.on_enter('base:disabled:fault', 'InputConveyor_NOT_READY')
        self.statemachine.on_enter('base:activated', 'ACTIVATE')
        self.statemachine.on_enter('base:operational', 'OPERATIONAL')
        self.statemachine.on_enter('base:operational:idle','IDLE')
        self.statemachine.on_enter('base:operational:loading', 'LOADING')
        self.statemachine.on_exit('base:operational:loading', 'EXIT_LOADING')
        self.statemachine.on_enter('base:operational:unloading', 'UNLOADING')
        self.statemachine.on_exit('base:operational:unloading', 'EXIT_UNLOADING')

        
        
