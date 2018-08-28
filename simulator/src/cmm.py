import os, sys
sys.path.insert(0,os.getcwd()+'\\configFiles')

from material import *
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

from cmm_bridge import *
from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, re, copy, uuid
import requests, urllib2, json
import xml.etree.ElementTree as ET
        

class cmm(object):

    def __init__(self,host,port, sim = True, cell_part = None):

        class statemachineModel(object):

            def __init__(self,host,port,sim, cell_part):
                
                self.initiate_adapter(host,port)
                self.adapter.start()

                self.sim = sim
                self.cell_part = cell_part
                
                self.initiate_dataitems()

                self.initiate_interfaces()

                self.system = []
                
                self.cycle_time = 120

                self.load_time_limit(20)
                self.unload_time_limit(20)

                self.load_failed_time_limit(2)
                self.unload_failed_time_limit(2)

                self.events = []

                self.master_tasks ={}

                self.deviceUuid = "cmm1"

                self.master_uuid = str()

                self.iscoordinator = False
                
                self.iscollaborator = False

                self.system_normal = True

                self.has_material = False
                
                self.fail_next = False

                self.part_quality = None

                self.pt_ql_seq = []

                self.part_quality_sequence() #updated for Hurco Demo

                self.initial_execution_state()

                self.priority = priority(self, self.cmm_binding)

                self.initiate_cmm_client()
                
                self.initiate_pull_thread()

            def part_quality_sequence(self):
                self.pt_ql_seq = []
                self.pt_ql_seq.append(['good', False])
                self.pt_ql_seq.append(['bad', False])
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
		    device = json.loads(configFile.read())['devices'][self.deviceUuid]
                    self.cmm_client = hexagonClient(str(device['host']), int(device['port']),int(device['port']))
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

                thread= Thread(target = self.start_pull,args=("http://localhost:5000","/conv/sample?interval=100&count=1000",from_long_pull))
                thread.start()

                thread2= Thread(target = self.start_pull,args=("http://localhost:5000","/robot/sample?interval=100&count=1000",from_long_pull))
                thread2.start()

                thread3= Thread(target = self.start_pull,args=("http://localhost:5000","/buffer/sample?interval=100&count=1000",from_long_pull))
                thread3.start()

                thread4= Thread(target = self.start_pull,args=("http://localhost:5000","/cnc/sample?interval=100&count=1000",from_long_pull))
                thread4.start()

            def start_pull(self,addr,request, func, stream = True):

                response = requests.get(addr+request, stream=stream)
                lp = LongPull(response, addr, self)
                lp.long_pull(func)

            def start_pull_asset(self, addr, request, assetId, stream_root):
                response = urllib2.urlopen(addr+request).read()
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
                    
                    self.iscoordinator = True
                    self.iscollaborator = False

                    if self.master_uuid in self.master_tasks:
                        del self.master_tasks[self.master_uuid]

                    self.master_uuid = self.deviceUuid+'_'+str(uuid.uuid4())
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
                        
                        self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = self.binding_state_material , coordinator_name = self.deviceUuid)
                        self.coordinator.create_statemachine()
                        
                        self.coordinator.superstate.task_name = "UnloadCmm"
                        
                        self.coordinator.superstate.unavailable()
                    
                elif self.has_material == False:
                    self.loading()
                    
                    self.iscoordinator = False
                    self.iscollaborator = True
                    self.collaborator = collaborator(parent = self, interface = self.binding_state_material, collaborator_name = self.deviceUuid)
                    self.collaborator.create_statemachine()
                    self.collaborator.superstate.task_name = "LoadCmm"
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
                    self.system.append(['cmm', 'Device', 'SYSTEM', 'FAULT', 'Cycle failed to start', 'CYCLE'])
                    self.cmm_fault()
                    self.fail_next = False

                else:
                    self.adapter.begin_gather()
                    self.e1.set_value("ACTIVE")
                    self.adapter.complete_gather()
                    
                    def func(self = self):
                        self.adapter.begin_gather()
                        self.e1.set_value("READY")
                        self.adapter.complete_gather()

                        self.cmm_execution_ready()
                        self.iscoordinator = True
                        self.iscollaborator = False

                        if self.master_uuid in self.master_tasks:
                            del self.master_tasks[self.master_uuid]
                    
                        self.master_uuid = self.deviceUuid+'_'+str(uuid.uuid4())
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

                            self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = self.binding_state_material , coordinator_name = self.deviceUuid)
                            self.coordinator.create_statemachine()
                        
                            self.coordinator.superstate.task_name = "UnloadCmm"
                        
                            self.coordinator.superstate.unavailable()

                    if self.sim:
                        timer_cycling = Timer(self.cycle_time,func)
                        timer_cycling.start()
                        
                    else:

                        if self.part_quality== 'good' or self.part_quality == 'reworked':
                            cycle = self.cmm_client.load_run_pgm(taskcmm.startProgramA)
                        elif self.part_quality == 'bad':
                            cycle = self.cmm_client.load_run_pgm(taskcmm.startProgramB)
                        elif self.part_quality == 'rework':
                            cycle = self.cmm_client.load_run_pgm(taskcmm.startProgramC)
                            
                        time.sleep(4)
                        status = (self.cmm_client.load_run_pgm(taskcmm.getStatus)).lower()
                        while 'good' not in status and 'bad' not in status and 'rework' not in status:
                            status = (self.cmm_client.load_run_pgm(taskcmm.getStatus)).lower()
                        
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
                        self.collaborator = collaborator(parent = self, interface = self.binding_state_material, collaborator_name = self.deviceUuid)
                        self.collaborator.create_statemachine()
                        self.collaborator.superstate.task_name = "LoadCmm"
                        self.collaborator.superstate.unavailable()
                        self.priority.collab_check()
            
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
                    self.cmm_binding.set_value(master_task_uuid)
                    self.adapter.complete_gather()

                    self.part_quality = self.part_quality_next()[1]
                            
                    if self.part_quality:
                        if self.part_quality == 'rework':
                            self.coordinator_task = "MoveMaterial_5"
                        else:
                            self.coordinator_task = "MoveMaterial_4"+"_"+self.part_quality

                        
                        self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = self.binding_state_material , coordinator_name = self.deviceUuid)
                        self.coordinator.create_statemachine()
                        
                        self.coordinator.superstate.task_name = "UnloadCmm"
                        
                        self.coordinator.superstate.unavailable()
                    
                else:
                    self.loading()
                    
                    self.iscoordinator = False
                    self.iscollaborator = True
                    self.collaborator = collaborator(parent = self, interface = self.binding_state_material, collaborator_name = self.deviceUuid)
                    self.collaborator.create_statemachine()
                    self.collaborator.superstate.task_name = "LoadCmm"
                    self.collaborator.superstate.unavailable()
                    self.priority.collab_check()
              
            def LOADED(self):
                self.has_material = True
		timer_timeout = Timer(60,self.collaborator.superstate.completed)
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

            def void(self):
                pass


            def event(self, source, comp, name, value, code = None, text = None):
                #print "CMM received " + comp + " " + name + " " + value + " from " + source
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
                    if self.iscoordinator == True:
                        self.coordinator.superstate.event(source, comp, name, value, code, text)

                    elif self.iscollaborator == True:
                        self.collaborator.superstate.event(source, comp, name, value, code, text)
                    
                elif name == "MaterialLoad" and action!='unavailable':
                    try:
                        if value.lower() == 'ready' and self.state == 'base:operational:idle':
                            eval('self.robot_material_load_ready()')
                        #print (self.material_load_interface.superstate.state)
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
                        except:
                            "Not a valid trigger"

                    elif name == "Availability":
                        if source.lower() == 'cmm':
                            self.adapter.begin_gather()
                            self.avail1.set_value(value.upper())
                            self.adapter.complete_gather()

                     

        self.superstate = statemachineModel(host,port, sim, cell_part)

    def draw(self):
        print "Creating cmm.png diagram"
        self.statemachine.get_graph().draw('cmm.png', prog='dot')

    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['activated',{'name':'operational', 'children':['loading', 'cycle_start', 'unloading', 'idle']}, {'name':'disabled', 'children':['fault', 'not_ready']}]} ]

        transitions= [['start', 'base', 'base:disabled'],
                      
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
                      {'trigger':'complete', 'source':'base:operational:loading', 'dest':'base:operational:cycle_start', 'before':'LOADED'},

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
                      {'trigger':'complete', 'source':'base:operational:unloading', 'dest':'base:operational:loading','before':'UNLOADED'},
                    
                      ['unloading', 'base:operational', 'base:operational:unloading'],
                      ['default', 'base:operational:unloading', 'base:operational:unloading'],
                      ['cmm_execution_ready', 'base:operational:cycle_start', 'base:operational:unloading'],

                      ['failed', 'base:operational:loading', 'base:operational:idle'],
                      ['failed', 'base:operational:unloading', 'base:operational:idle'],
                      ['start', 'base:operational', 'base:operational:idle'],
                      {'trigger':'robot_material_unload_ready','source':'base:operational:idle','dest':'base:operational', 'after':'EXITING_IDLE'},
                      {'trigger':'robot_material_load_ready','source':'base:operational:idle','dest':'base:operational', 'after':'EXITING_IDLE'},
                      ['default', 'base:operational:idle', 'base:operational:idle'],
                      
                      ['make_operational', 'base:activated', 'base:operational']
      
                      
                      ]

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)            
            
        self.statemachine.on_enter('base:disabled', 'CMM_NOT_READY')
        self.statemachine.on_enter('base:disabled:not_ready', 'CMM_NOT_READY')
        self.statemachine.on_enter('base:disabled:fault', 'CMM_NOT_READY')
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
    cmm1 = cmm('localhost',7591)
    cmm1.create_statemachine()
    cmm1.superstate.has_material = False
    cmm1.superstate.load_time_limit(200)
    cmm1.superstate.unload_time_limit(200)
    time.sleep(7)
    cmm1.superstate.enable()
    

    #Coordinator
    cmm1 = cmm()
    cmm1.create_statemachine('localhost',7591)
    cmm1.superstate.has_material = True
    cmm1.superstate.load_time_limit(200)
    cmm1.superstate.unload_time_limit(200)
    
    time.sleep(15)
    cmm1.superstate.enable()
    """

    cmm = cmm('localhost',7596)
    cmm.create_statemachine()
    cmm.superstate.load_time_limit(600)
    cmm.superstate.unload_time_limit(600)
    cmm.superstate.enable()
    
    
        
        
