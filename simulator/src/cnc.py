import os, sys
sys.path.insert(0,os.getcwd()+'\\utils')

from material import *
from tool import *
from door import *
from chuck import *
from coordinator import *
from collaborator import *
from mtconnect_adapter import Adapter
from long_pull import LongPull
from priority import priority
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from archetypeToInstance import archetypeToInstance
from from_long_pull import from_long_pull, from_long_pull_asset

from hurco_bridge import *

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, re, copy, uuid
import requests, urllib2, json
import xml.etree.ElementTree as ET


class cnc(object):

    def __init__(self,host,port,sim=True):

        class statemachineModel(object):

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
                self.nextsequence = '1'
                self.events = []

                self.master_tasks ={}

                self.deviceUuid = "cnc1"

                self.master_uuid = str()

                self.iscoordinator = False
                
                self.iscollaborator = False

                self.system_normal = True

                self.has_material = False
                
                self.fail_next = False

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
                    device = json.loads(configFile.read())['devices'][self.deviceUuid]
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

                self.thread= Thread(target = self.start_pull,args=("http://localhost:5000","""/conv/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",from_long_pull))
                self.thread.start()

                self.thread2= Thread(target = self.start_pull,args=("http://localhost:5000","""/robot/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",from_long_pull))
                self.thread2.start()

                self.thread3= Thread(target = self.start_pull,args=("http://localhost:5000","""/buffer/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",from_long_pull))
                self.thread3.start()

                self.thread4= Thread(target = self.start_pull,args=("http://localhost:5000","""/cmm/sample?path=//DataItem[@category="EVENT"]&interval=10&count=1000""",from_long_pull))
                self.thread4.start()

            def start_pull(self,addr,request, func, stream = True):

                response = requests.get(addr+request+"&from="+self.nextsequence, stream=stream)
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
                    
                    self.iscoordinator = True
                    self.iscollaborator = False

                    if self.master_uuid in self.master_tasks:
                        del self.master_tasks[self.master_uuid]

                    self.master_uuid = self.deviceUuid+'_'+str(uuid.uuid4())
                    
                    master_task_uuid = copy.deepcopy(self.master_uuid)

                    
                    self.adapter.begin_gather()
                    self.cnc_binding.set_value(master_task_uuid)
                    self.adapter.complete_gather()

                    self.coordinator_task = "MoveMaterial_2"
                    
                    self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = self.binding_state_material , coordinator_name = self.deviceUuid)
                    self.coordinator.create_statemachine()
                    

                    self.coordinator.superstate.task_name = "UnloadCnc"

                    self.coordinator.superstate.unavailable()
                    
                    
                elif self.has_material == False:
                    self.loading()
                    
                    self.iscoordinator = False
                    self.iscollaborator = True
                    self.collaborator = collaborator(parent = self, interface = self.binding_state_material, collaborator_name = 'cnc1')
                    self.collaborator.create_statemachine()
                    self.collaborator.superstate.task_name = "LoadCnc"
                    self.collaborator.superstate.unavailable()

                    self.priority.collab_check()
                    

                else:
                    self.start()

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

                elif self.door_state.value() != "CLOSED" or self.close_chuck_interface.superstate.response_state.value() != "CLOSED":
                    
                    self.system.append(['cnc', 'Device', 'SYSTEM', 'FAULT', 'Door or Chuck in invalid state', 'CYCLE'])
                    self.cnc_fault()                    

                else:
                    self.adapter.begin_gather()
                    self.e1.set_value("ACTIVE")
                    self.adapter.complete_gather()

                    def func(self = self):
                        
                        #self.adapter.begin_gather()
                        #self.e1.set_value("READY")
                        #self.adapter.complete_gather()

                        self.cnc_execution_ready()
                        self.iscoordinator = True
                        self.iscollaborator = False

                        if self.master_uuid in self.master_tasks:
                            del self.master_tasks[self.master_uuid]
            
                        self.master_uuid = self.deviceUuid+'_'+str(uuid.uuid4())
                        master_task_uuid = copy.deepcopy(self.master_uuid)
                        self.coordinator_task = "MoveMaterial_2"

                        self.adapter.begin_gather()
                        self.cnc_binding.set_value(master_task_uuid)
                        self.adapter.complete_gather()
                        
                        self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = self.binding_state_material , coordinator_name = self.deviceUuid)
                        self.coordinator.create_statemachine()
                        

                        self.coordinator.superstate.task_name = "UnloadCnc"

                        self.coordinator.superstate.unavailable()

                        self.adapter.begin_gather()
                        self.e1.set_value("READY")
                        self.adapter.complete_gather()

                    if self.sim:
                        timer_cycling = Timer(self.cycle_time,func)
                        timer_cycling.start()
                    else:
                        cycle_completion = self.cnc_client.load_run_pgm(tasks.cycle)
                        if cycle_completion == True:
                            time.sleep(1)
                            func()
                        


            def LOADING(self):
                if not self.has_material:
                    self.material_unload_interface.superstate.DEACTIVATE()
                    self.material_load_interface.superstate.idle()


            def UNLOADING(self):
                if self.has_material:
                    self.material_load_interface.superstate.DEACTIVATE()
                    self.material_unload_interface.superstate.idle()
                    

            def EXIT_LOADING(self):
                self.material_load_interface.superstate.DEACTIVATE()

            def EXIT_UNLOADING(self):
                self.material_unload_interface.superstate.DEACTIVATE()

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
                        self.collaborator.superstate.task_name = "LoadCnc"
                        self.collaborator.superstate.unavailable()
                        self.priority.collab_check()
                        
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
                    self.iscoordinator = True
                    self.iscollaborator = False

                    if self.master_uuid in self.master_tasks:
                        del self.master_tasks[self.master_uuid]
                        
                    self.master_uuid = self.deviceUuid+'_'+str(uuid.uuid4())
                    master_task_uuid = copy.deepcopy(self.master_uuid)

                    self.adapter.begin_gather()
                    self.cnc_binding.set_value(master_task_uuid)
                    self.adapter.complete_gather()
                    
                    self.coordinator_task = "MoveMaterial_2"
                    

                    self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = self.binding_state_material , coordinator_name = self.deviceUuid)
                    self.coordinator.create_statemachine()

                    self.coordinator.superstate.task_name = "UnloadCnc"
                    
                    self.coordinator.superstate.unavailable()

                else:
                    self.loading()
                    self.iscoordinator = False
                    self.iscollaborator = True
                    self.collaborator = collaborator(parent = self, interface = self.binding_state_material, collaborator_name = 'cnc1')
                    self.collaborator.create_statemachine()
                    self.collaborator.superstate.task_name = "LoadCnc"
                    self.collaborator.superstate.unavailable()
                    self.priority.collab_check()
              
            def LOADED(self):
                self.has_material = True

		timer_timeout = Timer(120,self.collaborator.superstate.completed)
                timer_timeout.start()

                while self.collaborator.superstate.state != 'base:inactive' or self.binding_state_material.value().lower() != 'inactive':
                    pass

                if self.binding_state_material.value().lower() != 'committed' and timer_timeout.isAlive():
                    timer_timeout.cancel()

            def UNLOADED(self):
                self.has_material = False
                while self.binding_state_material.value() == "COMMITTED":
                    pass

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
                    if value.lower() != 'preparing':
                        self.collaborator.superstate.event(source, comp, name, value, code, text)

                elif 'SubTask' in name and action!='unavailable':
                    """ #for manually operated door
                    if 'door' in name.lower(): #add self.sim later
                        if 'open' in name.lower():
                            if self.wait_for_completion:
                                self.adapter.begin_gather()
                                self.door_state.set_value("OPEN")
                                self.adapter.complete_gather()
                                self.wait_for_completion = False
                                
                            if action == 'active':
                                self.wait_for_completion = True
                                
                        elif 'close' in name.lower():
                            if self.wait_for_completion:
                                self.adapter.begin_gather()
                                self.door_state.set_value("CLOSED")
                                self.adapter.complete_gather()
                                
                            if action == 'active':
                                self.wait_for_completion = True
                    """
                    if self.iscoordinator == True:
                        self.coordinator.superstate.event(source, comp, name, value, code, text)

                    elif self.iscollaborator == True:
                        self.collaborator.superstate.event(source, comp, name, value, code, text)
                    
                elif "Open" in name and action!='unavailable':
                    
                    if 'door' in name.lower():
                        
                        eval('self.open_door_interface.superstate.'+action+'()')
                        if not self.sim and action == 'active': #where should this be : reconsider later
                            openDoor_completion = self.cnc_client.load_run_pgm(tasks.openDoor)
                            if openDoor_completion == True:
                                time.sleep(2)
                                eval('self.open_door_interface.superstate.complete()')
                            elif openDoor_completion == False:
                                time.sleep(2)
                                eval('self.open_door_interface.superstate.DEFAULT()')
                    
                    if 'chuck' in name.lower():
                        eval('self.open_chuck_interface.superstate.'+action+'()')

                        if not self.sim and action == 'active': #where should this be : reconsider later
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
                        
                        if not self.sim and action == 'active': #where should this be : reconsider later
                            closeDoor_completion = self.cnc_client.load_run_pgm(tasks.closeDoor)
                            if closeDoor_completion == True:
                                time.sleep(1)
                                eval('self.close_door_interface.superstate.complete()')
                            elif closeDoor_completion == False:
                                time.sleep(1)
                                eval('self.close_door_interface.superstate.DEFAULT()')
                        
                    if 'chuck' in name.lower():
                        eval('self.close_chuck_interface.superstate.'+action+'()')
                        
                        if not self.sim and action == 'active': #where should this be : reconsider later
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
                        
                    if not self.sim and action == 'active': #where should this be : reconsider later
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
                        except:
                            "Not a valid trigger"

                    elif name == "Availability":
                        if source.lower() == 'cnc':
                            self.adapter.begin_gather()
                            self.avail1.set_value(value.upper())
                            self.adapter.complete_gather()
                

 

        self.superstate = statemachineModel(host,port,sim)


    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['activated',{'name':'operational', 'children':['loading', 'cycle_start', 'unloading', 'idle']}, {'name':'disabled', 'children':['fault', 'not_ready']}]} ]

        transitions= [['start', 'base', 'base:disabled'],
                      
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
                      
                      {'trigger':'complete', 'source':'base:operational:loading', 'dest':'base:operational:cycle_start', 'before':'LOADED'},

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
                      {'trigger':'complete', 'source':'base:operational:unloading', 'dest':'base:operational:loading','before':'UNLOADED'},
                    
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
    """
    #collaborator
    cnc1 = cnc('localhost',7880)
    cnc1.create_statemachine()
    cnc1.superstate.has_material = False
    cnc1.superstate.load_time_limit(200)
    cnc1.superstate.unload_time_limit(200)
    time.sleep(10)
    cnc1.superstate.enable()
    

    #Coordinator
    cnc1 = cnc('localhost',7871)
    cnc1.create_statemachine()
    cnc1.superstate.has_material = True
    cnc1.superstate.load_time_limit(200)
    cnc1.superstate.unload_time_limit(200)
    
    cnc1.superstate.adapter.begin_gather()
    cnc1.superstate.door_state.set_value("CLOSED")
    cnc1.superstate.chuck_state.set_value("CLOSED")
    cnc1.superstate.adapter.complete_gather()
    
    time.sleep(15)
    cnc1.superstate.enable()
    """
    cnc = cnc('localhost',7896)
    cnc.create_statemachine()
    cnc.superstate.load_time_limit(600)
    cnc.superstate.unload_time_limit(600)
    cnc.superstate.enable()

    
    
    
