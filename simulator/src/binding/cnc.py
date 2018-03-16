
from material import *
from door import *
from chuck import *
from coordinator import *
from collaborator import *
from mtconnect_adapter import Adapter
from long_pull import LongPull
from data_item import Event, SimpleCondition, Sample, ThreeDSample

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, re, copy
import requests
import xml.etree.ElementTree as ET


class interface(object):

    def __init__(self, value = None):
        self.value = value
        

class cnc(object):

    def __init__(self, interface):

        class statemachineModel(object):

            def __init__(self):
                
                self.adapter = Adapter(('localhost',7780))

                self.mode1 = Event('mode')
                self.adapter.add_data_item(self.mode1)

                self.e1 = Event('execution')
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

                self.chuck_state = Event('chuck_state')
                self.adapter.add_data_item(self.chuck_state)

                self.door_state = Event('door_state')
                self.adapter.add_data_item(self.door_state)

                self.material_load = Event('material_load')
                self.adapter.add_data_item(self.material_load)

                self.material_unload = Event('material_unload')
                self.adapter.add_data_item(self.material_unload)

                self.adapter.start()

                self.material_load_interface = MaterialLoad(self)
                self.material_unload_interface = MaterialUnload(self)
                self.open_chuck_interface = OpenChuck(self)
                self.close_chuck_interface = CloseChuck(self)
                self.open_door_interface = OpenDoor(self)
                self.close_door_interface = CloseDoor(self)

                self.has_material = False
                self.fail_next = False

                self.robot_availability = "AVAILABLE" #intialized for testing
                self.robot_execution = "READY"
                self.robot_controller_mode = "AUTOMATIC"
                
                self.cycle_time = 2.0

                self.system = []

                self.system_normal = True

                self.link = "ENABLED"

                self.load_time_limit(10)
                self.unload_time_limit(10)

                self.load_failed_time_limit(2)
                self.unload_failed_time_limit(2)

                self.events = []

                self.master_tasks ={}

                self.master_uuid = 1 #w.r.t PnP?

                self.iscoordinator = False
                self.iscollaborator = False

                #adapter: adding dataitems to adapter: should be unique
                

                self.adapter.begin_gather()

                self.door_state.set_value("OPEN")
                self.chuck_state.set_value("OPEN")
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

                self.adapter.complete_gather()
                
                self.device_pull =[]
                
                thread= Thread(target = self.start_pull,args=("http://localhost:5006/sample?interval=10&count=1000",))
                thread.start()
                

            def start_pull(self,addr):
                
                response = requests.get(addr, stream=True)
                lp = LongPull(response)
                lp.long_pull(self.from_long_pull)
                
                

            def CNC_NOT_READY(self):
                self.open_chuck_interface.superstate.DEACTIVATE()
                self.close_chuck_interface.superstate.DEACTIVATE()
                self.open_door_interface.superstate.DEACTIVATE()
                self.close_door_interface.superstate.DEACTIVATE()
                self.material_load_interface.superstate.DEACTIVATE()
                self.material_unload_interface.superstate.DEACTIVATE()

            #change ACTIVATE?
            def ACTIVATE(self):
                print 'in activate'
                if self.mode1.value() == "AUTOMATIC" and self.avail1.value() == "AVAILABLE":
                    print 'making operational'
                    self.make_operational()

                elif self.system_normal:
                    print 'not ready'
                    self.still_not_ready()

                else:
                    print 'faulted'
                    self.faulted()

            def OPERATIONAL(self):
                self.open_chuck_interface.superstate.ACTIVATE()
                self.close_chuck_interface.superstate.ACTIVATE()
                self.open_door_interface.superstate.ACTIVATE()
                self.close_door_interface.superstate.ACTIVATE()
                print 'in operational'

                if self.has_material and self.link == "ENABLED" and self.robot_controller_mode =="AUTOMATIC" and self.robot_execution == "ACTIVE" and self.robot_availability == "AVAILABLE":
                    self.unloading()
                    print 'in unloading'
                    self.iscoordinator = True
                    self.iscollaborator = False
                    master_task_uuid = copy.deepcopy(self.master_uuid)
                    
                    self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = self.binding_state_material , coordinator_name = self.master_tasks[master_task_uuid]['coordinator'].keys()[0])
                    self.coordinator.create_statemachine()
                    self.coordinator.superstate.task_name = "MaterialUnload"
                    
                    self.coordinator.superstate.unavailable()
                elif self.has_material == False and self.link == "ENABLED" and self.robot_controller_mode =="AUTOMATIC" and self.robot_execution == "ACTIVE" and self.robot_availability == "AVAILABLE":
                    self.loading()
                    print 'in loading'
                    self.iscoordinator = False
                    self.iscollaborator = True
                    self.collaborator = collaborator(parent = self, interface = self.binding_state_material, collaborator_name = 'cnc1')
                    self.collaborator.create_statemachine()
                    self.collaborator.superstate.task_name = "MaterialLoad"
                    self.collaborator.superstate.unavailable()

                else:
                    self.start()

            def IDLE(self):
                print 'in idle'
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
                    self.adapter.begin_gather()
                    self.e1.set_value("ACTIVE")
                    self.adapter.complete_gather()

                    def func(self = self):
                        
                        self.adapter.begin_gather()
                        self.e1.set_value("READY")
                        self.adapter.complete_gather()

                        master_task_uuid = copy.deepcopy(self.master_uuid)
                        self.cnc_execution_ready()
                        self.iscoordinator = True
                        self.iscollaborator = False
                        self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = self.binding_state_material , coordinator_name = self.master_tasks[master_task_uuid]['coordinator'].keys()[0])
                        self.coordinator.create_statemachine()
                        self.coordinator.superstate.task_name = "MaterialUnload"
                        
                        self.coordinator.superstate.unavailable()
                        
                        
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
                    if self.has_material == False:
                        self.iscoordinator = False
                        self.iscollaborator = True
                        self.collaborator = collaborator(parent = self, interface = self.binding_state_material, collaborator_name = 'cnc1')
                        self.collaborator.create_statemachine()
                        self.collaborator.superstate.task_name = "MaterialLoad"
                        self.collaborator.superstate.unavailable()
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
                #what about "before" clause in the unloading trigger ??????
                if self.has_material:
                    self.unloading()
                    master_task_uuid = copy.deepcopy(self.master_uuid)
                    self.iscoordinator = True
                    self.iscollaborator = False
                    self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = self.binding_state_material , coordinator_name = self.master_tasks[master_task_uuid]['coordinator'].keys()[0])
                    self.coordinator.create_statemachine()
                    self.coordinator.superstate.task_name = "MaterialUnload"
                    
                    self.coordinator.superstate.unavailable()
                else:
                    self.loading()
                    self.iscoordinator = False
                    self.iscollaborator = True
                    self.collaborator = collaborator(parent = self, interface = self.binding_state_material, collaborator_name = 'cnc1')
                    self.collaborator.create_statemachine()
                    self.collaborator.superstate.task_name = "MaterialLoad"
                    self.collaborator.superstate.unavailable()
              
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
                print "CNC received " + comp + " " + name + " " + value + " from " + source + "\n"
                self.events.append([source, comp, name, value, code, text])

                action= value.lower()

                if action == "fail":
                    action = "failure"

                if comp == "Collaborator" and action!='unavailable':
                    self.coordinator.superstate.event(source, comp, name, value, code, text)

                elif comp == "Coordinator" and action!='unavailable':
                    self.collaborator.superstate.event(source, comp, name, value, code, text)

                elif 'SubTask' in name and action!='unavailable':
                    if self.iscoordinator == True:
                        self.coordinator.superstate.event(source, comp, name, value, code, text)

                    elif self.iscollaborator == True:
                        self.collaborator.superstate.event(source, comp, name, value, code, text)
                    
                elif name == "Open" and action!='unavailable':
                    if comp == "DoorInterface":
                        eval('self.open_door_interface.superstate.'+action+'()')
                        
                    elif comp == "ChuckInterface":
                        eval('self.open_chuck_interface.superstate.'+action+'()')

                elif name == "Close" and action!='unavailable':
                    if comp == "DoorInterface":
                        eval('self.close_door_interface.superstate.'+action+'()')

                    elif comp == "ChuckInterface":
                        eval('self.close_chuck_interface.superstate.'+action+'()')

                elif name == "MaterialLoad" and action!='unavailable':
                    print 'executing'+action+'at'+self.material_load_interface.superstate.state
                    try:
                        if action=='ready' and self.state =='base:operational:idle':
                            eval('self.robot_material_load_ready()')
                        eval('self.material_load_interface.superstate.'+action+'()')
                    except:
                        "Incorrect event"

                elif name == "MaterialUnload" and action!='unavailable':
                    try:
                        if action =='ready' and self.state =='base:operational:idle':
                            eval('self.robot_material_unload_ready()')
                        eval('self.material_unload_interface.superstate.'+action+'()')
                    except:
                        "incorrect event"

                elif comp == "Controller":
                    
                    if name == "ControllerMode":
                        if source.lower() == 'cnc':
                            self.adapter.begin_gather()
                            self.mode1.set_value(value.upper())
                            self.adapter.complete_gather()
                            
                        elif source.lower() == 'robot':
                            self.robot_controller_mode = value.upper()

                        if action!='unavailable':
                            try:
                                eval('self.'+source.lower()+'_controller_mode_'+value.lower()+'()')
                            except:
                                "Not a valid trigger"

                    elif name == "Execution":
                        if source.lower() == 'cnc':
                            self.adapter.begin_gather()
                            self.e1.set_value(value.upper())
                            self.adapter.complete_gather()
                    
                        elif source.lower() == 'robot':
                            self.robot_execution = value.upper()
                        if action!='unavailable':
                            try:
                                eval('self.'+source.lower()+'_execution_'+value.lower()+'()')
                            except:
                                "Not a valid trigger"

                elif comp == "Device":

                    if name == "SYSTEM" and action!='unavailable':
                        try:
                            eval('self.'+source.lower()+'_system_'+value.lower()+'()')
                        except:
                            "Not a valid trigger"

                    elif name == "Availability":
                        if source.lower() == 'cnc':
                            self.adapter.begin_gather()
                            self.avail1.set_value(value.upper())
                            self.adapter.complete_gather()
                    
                        elif source.lower() == 'robot':
                            self.robot_availability = value.upper()

                        if action!='unavailable':
                            try:
                                eval('self.'+source.lower()+'_availability_'+value.lower()+'()')
                            except:
                                "Not a valid trigger"

                elif source == "cnc" and name == "ChuckState" and action!='unavailable':
                    self.chuck_state = value.upper()
                    if self.chuck_state == "OPEN":
                        self.open_chuck_interface.statemachine.set_state('base:active')
                    elif self.chuck_state == "CLOSED":
                        self.close_chuck_interface.statemachine.set_state('base:not_ready')
                    

                elif source == "cnc" and name == "DoorState" and action!='unavailable':
                    self.door_state = value.upper()
                    if self.door_state == "OPEN":
                        self.open_door_interface.statemachine.set_state('base:active')
                    elif self.door_state == "CLOSED":
                        self.close_door_interface.statemachine.set_state('base:not_ready')

            def from_long_pull(self, chunk):
                root=ET.fromstring(chunk)
                xmlns =root.tag.split('}')[0]+'}'
                s=root.findall('.//'+xmlns+'Streams')[0]

                for x in s:
                    source = x.attrib['name']

                    for y in x:
                        component = y.attrib['component']

                        events = y.find('.//'+xmlns+'Events')
                        for event in events:
                            try:
                                #self.event(source.lower(), component, event.tag.split('}')[-1], event.text)
                                thread1= Thread(target = self.event,args=(source.lower(), component, event.tag.split('}')[-1], event.text))
                                thread1.start()
                            except:
                                "Invalid attribute"


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

        
if __name__ == '__main__':
    cnc1 = cnc(interface)
    cnc1.create_statemachine()
    cnc1.superstate.has_material = False
    cnc1.superstate.load_time_limit(10)
    cnc1.superstate.unload_time_limit(10)
