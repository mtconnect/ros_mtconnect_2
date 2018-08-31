import os, sys
sys.path.insert(0,os.getcwd()+'\\utils')

from material import *

from collaborator import *
from coordinator import *

from mtconnect_adapter import Adapter
from long_pull import LongPull
from priority import priority
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from archetypeToInstance import archetypeToInstance
from from_long_pull import from_long_pull, from_long_pull_asset

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, re, copy
import xml.etree.ElementTree as ET
import requests, urllib2, uuid

class inputConveyor(object):

    def __init__(self, host, port, cell_part):

        class statemachineModel(object):

            def __init__(self, host, port, cell_part):

                self.initiate_adapter(host, port)
                self.adapter.start()
                self.initiate_dataitems()

                self.cell_part = cell_part

                self.initiate_interfaces()
                
                self.system = []

                self.load_time_limit(15)
                self.unload_time_limit(15)
                self.nextsequence='1'
                self.load_failed_time_limit(2)
                self.unload_failed_time_limit(2)

                self.events = []

                self.lp = {}

                self.master_tasks = {}

                self.deviceUuid = "conv1"

                self.master_uuid = str()

                self.iscoordinator = False
                
                self.iscollaborator = False
                
                self.system_normal = True
                                
                self.has_material = False
                
                self.fail_next = False

                self.internal_buffer = {}

                self.current_part = None

                self.cycle_count = 0

                self.initial_execution_state()

                self.set_priority()

                self.initiate_internal_buffer()

                self.initiate_pull_thread()

            def set_priority(self):
                self.priority = None
                self.priority = priority(self, self.conv1_binding)
                
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

            def initiate_internal_buffer(self):
                self.internal_buffer['good'] = True
                self.internal_buffer['bad'] = True
                self.internal_buffer['rework'] = True

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

                self.conv1_binding = Event('conv1_binding')
                self.adapter.add_data_item(self.conv1_binding)

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

                self.thread= Thread(target = self.start_pull,args=("http://localhost:5000","/cnc/sample?interval=10&count=1000",from_long_pull))
                self.thread.start()

                self.thread2= Thread(target = self.start_pull,args=("http://localhost:5000","/robot/sample?interval=10&count=1000",from_long_pull))
                self.thread2.start()

                self.thread3= Thread(target = self.start_pull,args=("http://localhost:5000","/cmm/sample?interval=10&count=1000",from_long_pull))
                self.thread3.start()
                
            def start_pull(self,addr,request, func, stream = True):

                response = requests.get(addr+request+"&from="+self.nextsequence, stream=stream)
                self.lp[request.split('/')[1]] = None
                self.lp[request.split('/')[1]] = LongPull(response, addr, self)
                self.lp[request.split('/')[1]].long_pull(func)

            def start_pull_asset(self, addr, request, assetId, stream_root):
                response = urllib2.urlopen(addr+request).read()
                from_long_pull_asset(self, response, stream_root)


            def InputConveyor_NOT_READY(self):
                self.material_load_interface.superstate.DEACTIVATE()
                self.material_unload_interface.superstate.DEACTIVATE()

            def ACTIVATE(self):
                if self.mode1.value() == "AUTOMATIC" and self.avail1.value() == "AVAILABLE":

                    self.make_operational()

                elif self.system_normal:
                    
                    self.still_not_ready()

                else:
                    
                    self.faulted()

            def part_order(self):
                #scripted sequence of part order tasks
                part_quality_list = [self.internal_buffer['good'], self.internal_buffer['bad'], self.internal_buffer['rework']]

                if self.cycle_count == 1:
                    self.current_part = "reset"

                if part_quality_list == [True,True,True] and self.current_part == None:
                    self.current_part = 'good'
                    return "coordinator"
                elif part_quality_list == [False,True,True] and self.current_part == 'good':
                    self.current_part = 'bad'
                    return "coordinator"
                elif part_quality_list == [False,False,True] and self.current_part == 'bad':
                    self.current_part = 'good'
                    return "collaborator"
                elif part_quality_list == [True,False,True] and self.current_part == 'good':
                    self.current_part = 'bad'
                    return "collaborator"
                elif part_quality_list == [True,True,True] and self.current_part == 'bad':
                    self.current_part = 'rework'
                    return "coordinator"
                elif part_quality_list == [True,True,False] and self.current_part == 'rework':
                    self.current_part = 'good'
                    return "coordinator"
                elif part_quality_list == [False,True,False] and self.current_part == 'good':
                    self.current_part = 'good'
                    return "collaborator"
                elif part_quality_list == [True,True,False] and self.current_part == 'good':
                    self.current_part = 'rework'
                    return "collaborator"
                elif part_quality_list == [True,True,True] and self.current_part == 'rework':
                    self.current_part = 'good'
                    time.sleep(2)
                    print ("Cycle completed!")
                    return "coordinator"
                else:
                    return None

                
            def OPERATIONAL(self):
                part_order = self.part_order()
                    
                if part_order == "coordinator":
                    print (self.current_part)
                    self.unloading()
                    
                    self.iscoordinator = True
                    self.iscollaborator = False

                    if self.master_uuid in self.master_tasks:
                        del self.master_tasks[self.master_uuid]
                
                    self.master_uuid = self.deviceUuid+'_'+str(uuid.uuid4())
                    master_task_uuid = copy.deepcopy(self.master_uuid)


                    time.sleep(0.2)
                    self.adapter.begin_gather()
                    self.conv1_binding.set_value(self.master_uuid)
                    self.adapter.complete_gather()
                            
                    self.coordinator_task = "MoveMaterial_1"+"_"+self.current_part
                    
                    self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = self.binding_state_material , coordinator_name = self.deviceUuid)
                    self.coordinator.create_statemachine()

                    self.coordinator.superstate.task_name = "UnloadConv"

                    self.coordinator.superstate.unavailable()

                elif part_order == "collaborator":
                    print (self.current_part)
                    self.loading()
                    
                    self.iscoordinator = False
                    self.iscollaborator = True
                    self.collaborator = collaborator(parent = self, interface = self.binding_state_material, collaborator_name = self.deviceUuid)
                    self.collaborator.create_statemachine()
                    self.collaborator.superstate.task_name = "LoadConv"
                    self.collaborator.superstate.unavailable()
                    self.priority.collab_check()

                else:
                    if part_order == None:
                        self.cell_part(current_part = "reset")
                    else:
                        self.cell_part(current_part = self.current_part)
            def IDLE(self):
                if False not in self.internal_buffer.values():
                    self.material_load_interface.superstate.DEACTIVATE()

                elif False in self.internal_buffer.values() and True in self.internal_buffer.values():
                    self.material_unload_interface.superstate.DEACTIVATE()

            def LOADING(self):
                self.material_load_interface.superstate.IDLE()

            def UNLOADING(self):
                self.material_unload_interface.superstate.IDLE()

            def EXIT_LOADING(self):
                self.has_material = True

                self.internal_buffer[self.current_part] = True
                if self.current_part == 'rework':
                    self.cycle_count = self.cycle_count + 1
                    print ("Number of cycles completed: "+ str(self.cycle_count)+ " !")

                self.material_load_interface.superstate.DEACTIVATE()


		timer_timeout = Timer(40,self.collaborator.superstate.completed)
                timer_timeout.start()

                while self.collaborator.superstate.state != 'base:inactive' or self.binding_state_material.value().lower() != 'inactive':
                    pass
                if self.binding_state_material.value().lower() != 'committed' and timer_timeout.isAlive():
                    timer_timeout.cancel()

                time.sleep(1)

            def EXIT_UNLOADING(self):
                self.has_material = False #look into it later
                self.material_unload_interface.superstate.DEACTIVATE()
                self.internal_buffer[self.current_part] = False
                while self.binding_state_material.value().lower() != 'inactive':
                    pass

                time.sleep(1)
                print (self.cell_part(self.current_part))

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
                    self.has_material = False #look into it later


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

                self.events.append([source, comp, name, value, code, text])

                action= value.lower()

                if action == "fail":
                    action = "failure"

                if comp == "Coordinator" and value.lower() == 'preparing':
                    self.priority.event_list([source, comp, name, value, code, text])

                if comp == "Task_Collaborator" and action!= 'unavailable':
                    self.coordinator.superstate.event(source, comp, name, value, code, text)

                elif comp == "Coordinator" and action!= 'unavailable':
                    if value.lower() != 'preparing':
                        self.collaborator.superstate.event(source, comp, name, value, code, text)

                elif 'SubTask' in name and action!= 'unavailable':
                    if self.iscoordinator == True:
                        self.coordinator.superstate.event(source, comp, name, value, code, text)

                    elif self.iscollaborator == True:
                        self.collaborator.superstate.event(source, comp, name, value, code, text)

                elif name == "MaterialLoad" and action!= 'unavailable':
                    if value.lower() == 'ready' and self.state == 'base:operational:idle':
                        eval('self.robot_material_load_ready()')

                    eval('self.material_load_interface.superstate.'+action+'()')

                elif name == "MaterialUnload" and action!= 'unavailable':
                    if value.lower() == 'ready' and self.state == 'base:operational:idle':
                        eval('self.robot_material_unload_ready()')
                    
                    eval('self.material_unload_interface.superstate.'+action+'()')

                elif comp == "Controller":
                    
                    if name == "ControllerMode":
                        if source.lower() == 'conv':
                            self.adapter.begin_gather()
                            self.mode1.set_value(value.upper())
                            self.adapter.complete_gather()
                            

                    elif name == "Execution":
                        if source.lower() == 'conv':
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
                        if source.lower() == 'conv':
                            self.adapter.begin_gather()
                            self.avail1.set_value(value.upper())
                            self.adapter.complete_gather()


        self.superstate = statemachineModel(host, port, cell_part)

    def draw(self):
        print "Creating inputConveyor.png diagram"
        self.statemachine.get_graph().draw('inputConveyor.png', prog='dot')

    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['activated',{'name':'operational', 'children':['loading', 'unloading', 'idle']}, {'name':'disabled', 'children':['fault', 'not_ready']}]} ]

        transitions= [['start', 'base', 'base:disabled'],
                      
                      ['conveyor_controller_mode_automatic', 'base', 'base:activated'],
                      
                      ['enable', 'base', 'base:activated'],
                      ['disable', 'base', 'base:activated'],
                      ['conveyor_controller_mode_manual', 'base', 'base:activated'],
                      ['conveyor_controller_mode_manual_data_input', 'base', 'base:activated'],
                      ['conveyor_controller_mode_automatic', 'base', 'base:activated'],
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
                      {'trigger':'complete', 'source':'base:operational:unloading', 'dest':'base:operational', 'before':'EXIT_UNLOADING'},
                      {'trigger':'complete', 'source':'base:operational:loading', 'dest':'base:operational', 'before':'EXIT_LOADING'},
                      
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
        self.statemachine.on_enter('base:operational:unloading', 'UNLOADING')

        

if __name__ == '__main__':
    conv1 = inputConveyor('localhost',7780)
    conv1.create_statemachine()
    conv1.superstate.has_material = True
    conv1.superstate.load_time_limit(200)
    conv1.superstate.unload_time_limit(200)
    time.sleep(15)
    conv1.superstate.enable()
