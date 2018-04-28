import os, sys
sys.path.insert(0,os.getcwd()+'\\utils')

from material import *

from collaborator import *
from coordinator import *

from mtconnect_adapter import Adapter
from long_pull import LongPull
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from archetypeToInstance import archetypeToInstance

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, re, copy
import xml.etree.ElementTree as ET
import requests, urllib2, uuid


class adapter(object):

    def __init__(self, value = None):
        self.interface = value
        

class inputConveyor(object):

    def __init__(self, interface):

        class statemachineModel(object):

            def __init__(self):
                
                self.adapter = Adapter(('localhost',7727))

                self.mode1 = Event('mode')
                self.adapter.add_data_item(self.mode1)

                self.e1 = Event('exec')
                self.adapter.add_data_item(self.e1)

                self.avail1 = Event('avail')
                self.adapter.add_data_item(self.avail1)

                self.binding_state_material = Event('binding_state_material')
                self.adapter.add_data_item(self.binding_state_material)

                self.material_load = Event('material_load')
                self.adapter.add_data_item(self.material_load)

                self.material_unload = Event('material_unload')
                self.adapter.add_data_item(self.material_unload)

                self.adapter.start()

                self.material_load_interface = MaterialLoad(self)
                self.material_unload_interface = MaterialUnload(self)
                
                self.has_material = False
                self.fail_next = False
                
                self.robot_availability = "AVAILABLE" #intialized for testing
                self.robot_execution = "ACTIVE"
                self.robot_controller_mode = "AUTOMATIC"
                
                self.system = []

                self.system_normal = True

                self.link = "ENABLED"

                self.load_time_limit(10)
                self.unload_time_limit(10)

                self.load_failed_time_limit(2)
                self.unload_failed_time_limit(2)

                self.events = []

                self.master_tasks = {}

                self.deviceUuid = "conv1"

                self.master_uuid = 'conv1.1'

                self.iscoordinator = False
                self.iscollaborator = False

                self.adapter.begin_gather()

                self.avail1.set_value("AVAILABLE")
                self.e1.set_value("READY")
                self.mode1.set_value("AUTOMATIC")
                self.binding_state_material.set_value("INACTIVE")
                self.material_load.set_value("NOT_READY")
                self.material_unload.set_value("NOT_READY")

                self.adapter.complete_gather()
                
                self.device_pull =[]
                
                try:
                    thread= Thread(target = self.start_pull,args=("http://localhost:5000","/robot/sample?interval=100&count=1000",self.from_long_pull))
                    thread.start()
                except:
                    print "No device at http://localhost:5000\n"
                    
                try:
                    thread2= Thread(target = self.start_pull,args=("http://localhost:5000","/cnc/sample?interval=100&count=1000",self.from_long_pull))
                    thread2.start()
                except:
                    print "No device at http://localhost:5000\n"
                
                
            def start_pull(self,addr,request, func, stream = True):
                
                response = requests.get(addr+request, stream=stream)
                lp = LongPull(response, addr)
                lp.long_pull(func)

            def start_pull_asset(self, addr, request, assetId, stream_root):
                response = urllib2.urlopen(addr+request).read()
                self.from_long_pull_asset(response, stream_root)

            def InputConveyor_NOT_READY(self):
                self.material_load_interface.superstate.DEACTIVATE()
                self.material_unload_interface.superstate.DEACTIVATE()

            def ACTIVATE(self):
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

                if self.has_material:
                    self.unloading()
                    print 'in unloading'
                    self.iscoordinator = True
                    self.iscollaborator = False

                    self.master_uuid = self.deviceUuid+'_'+str(uuid.uuid4())
                    master_task_uuid = copy.deepcopy(self.master_uuid)
                    self.coordinator_task = "MoveMaterial_1"
                    print "unloading 2"+master_task_uuid
                    
                    self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = self.binding_state_material , coordinator_name = self.deviceUuid)
                    self.coordinator.create_statemachine()

                    self.coordinator.superstate.task_name = "UnloadConv"

                    print "unloading 3"

                    self.coordinator.superstate.unavailable()

                    print 'ff'+self.coordinator.superstate.state
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
                    print 'in unloading'
                    self.iscoordinator = True
                    self.iscollaborator = False

                    self.master_uuid = self.deviceUuid+'_'+str(uuid.uuid4())
                    master_task_uuid = copy.deepcopy(self.master_uuid)
                    self.coordinator_task = "MoveMaterial_1"
                    print "unloading 2"+master_task_uuid

                    self.coordinator = coordinator(parent = self, master_task_uuid = master_task_uuid, interface = self.binding_state_material , coordinator_name = self.deviceUuid)
                    self.coordinator.create_statemachine()
                    #self.current_task = "UnloadCnc"

                    self.coordinator.superstate.task_name = "UnloadConv"

                    print "unloading 3"

                    self.coordinator.superstate.unavailable()

                    print 'ff'+self.coordinator.superstate.state
                
              
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
                print "InputConveyor received " + comp + " " + name + " " + value + " from " + source + "\n"
                self.events.append([source, comp, name, value, code, text])

                action= value.lower()

                if action == "fail":
                    action = "failure"

                elif comp == "Task_Collaborator" and action!= 'unavailable':
                    self.coordinator.superstate.event(source, comp, name, value, code, text)

                elif comp == "Coordinator" and action!= 'unavailable':
                    self.collaborator.superstate.event(source, comp, name, value, code, text)

                elif 'SubTask' in name and action!= 'unavailable':
                    if self.iscoordinator == True:
                        self.coordinator.superstate.event(source, comp, name, value, code, text)

                    elif self.iscollaborator == True:
                        self.collaborator.superstate.event(source, comp, name, value, code, text)

                elif name == "MaterialLoad" and action!= 'unavailable':
                    if value.lower() == 'ready' and self.state == 'base:operational:idle':
                        eval('self.robot_material_load_ready()')
                    else:
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
                            
                        elif source.lower() == 'robot':
                            self.robot_controller_mode = value.upper()

                        if action!='unavailable':
                            try:
                                if self.robot_availability == "AVAILABLE" and self.robot_execution == "ACTIVE":
                                    eval('self.'+source.lower()+'_controller_mode_'+value.lower()+'()')
                            except:
                                "Not a valid trigger"

                    elif name == "Execution":
                        if source.lower() == 'conv':
                            self.adapter.begin_gather()
                            self.e1.set_value(value.upper())
                            self.adapter.complete_gather()
                    
                        elif source.lower() == 'robot':
                            self.robot_execution = value.upper()
                        if action!='unavailable':
                            try:
                                if self.robot_availability == "AVAILABLE" and self.robot_controller_mode == "AUTOMATIC":
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
                        if source.lower() == 'conv':
                            self.adapter.begin_gather()
                            self.avail1.set_value(value.upper())
                            self.adapter.complete_gather()
                    
                        elif source.lower() == 'robot':
                            self.robot_availability = value.upper()

                        if action!='unavailable':
                            try:
                                if self.robot_controller_mode == "AUTOMATIC" and self.robot_execution == "ACTIVE":
                                    eval('self.'+source.lower()+'_availability_'+value.lower()+'()')
                            except:
                                "Not a valid trigger"
            def from_long_pull(self, chunk, addr = None):
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
                                #THIS CLAUSE? DO WE NEED IT?
                                if 'Availability' in event.tag or 'Execution' in event.tag or 'ControllerMode' in event.tag:
                                    print "1_avail"
                                    thread1= Thread(target = self.event,args=(source.lower(), component, event.tag.split('}')[-1], event.text))
                                    thread1.start()
                                    
                                else: #if 'Asset' in event.tag:
                                    if ('AssetChanged' in event.tag or 'BindingState' in event.tag or self.binding_state_material.value() == "COMMITTED") and event.text.lower() != 'unavailable':
                                        
                                        print event.tag
                                        if 'AssetChanged' in event.tag:
                                            thread= Thread(target = self.start_pull_asset,args=(addr,"/asset/",event.text, [event,source,component,x.attrib['uuid']]))
                                            thread.start()
                                            
                                        elif 'BindingState' in event.tag:
                                            print "2_bind"
                                            thread= Thread(target = self.start_pull_asset,args=(addr,"/asset/",self.master_uuid, [event,source,component,x.attrib['uuid']]))
                                            thread.start()
                                            
                                        elif self.binding_state_material.value() == "COMMITTED":
                                            thread= Thread(target = self.start_pull_asset,args=(addr,"/asset/",self.master_uuid, [event,source,component,x.attrib['uuid']]))
                                            thread.start()
                                            
                                    elif 'AssetRemoved' in event.tag and self.binding_state_material.value() == "INACTIVE" and event.text.lower() != 'unavailable':
                                        
                                        print 'REMOVED'+event.tag+'\n'
                                        try:
                                            if self.deviceUuid in event.text:
                                                self.adapter.removeAsset(event.text)
                                        except:
                                            "THIS CLAUSE IS FOR MAKING SURE THE ASSET IS REMOVED WHEN COMPLETED."

                                    else:
                                        
                                        print 'BAD'+event.tag+'\n'
                                """
                                else: #do we need it here?
                                    if self.binding_state_material.value() == "COMMITTED" or ('Availability' or 'Execution' or 'ControllerMode' in event.tag):
                                        thread1= Thread(target = self.event,args=(source.lower(), component, event.tag.split('}')[-1], event.text))
                                        thread1.start()
                                """
                            except:
                                "Invalid attribute"

            def from_long_pull_asset(self,chunk, stream_root = None):
                root=ET.fromstring(chunk)
                xmlns =root.tag.split('}')[0]+'}'
                task = root.findall('.//'+xmlns+'Task')
                parentRef = None
                if task:
                    task = root.findall('.//'+xmlns+'Task')[0]
                    state = root.findall('.//'+xmlns+'State')[0].text
                    parentRef = root.findall('.//'+xmlns+'ParentRef')
                #if conv a collaborator
                if task and not parentRef:
                    for x in root.findall('.//'+xmlns+'Collaborator'):
                        if x.attrib['collaboratorId'] == self.deviceUuid:
                            main_task_archetype = root.findall('.//'+xmlns+'AssetArchetypeRef')[0].attrib['assetId']
                            main_task_uuid = root.findall('.//'+xmlns+'Task')[0].attrib['assetId']
                            main_task_deviceUuid = root.findall('.//'+xmlns+'Task')[0].attrib['deviceUuid']
                            coordinator = root.findall('.//'+xmlns+'Coordinator')[0]
                            component = "Coordinator"
                            name = "binding_state"
                            value = state

                            self.master_uuid = main_task_uuid
                            
                            self.master_tasks[main_task_uuid] = archetypeToInstance(main_task_archetype,"uuid", main_task_deviceUuid, main_task_uuid).jsonInstance()

                            if self.binding_state_material.value() == "PREPARING":
                                if value == "PREPARING":
                                    self.event(coordinator.text, component, name, value, [self.master_uuid, self.master_tasks[main_task_uuid]],  coordinator.attrib['collaboratorId'])
                                elif value == "COMMITTING":
                                    self.event(coordinator.text, component, name, value, self.master_uuid,  coordinator.attrib['collaboratorId'])
                                elif value == "COMMITTED":
                                    self.event(coordinator.text, component, name, value, self.master_uuid,  coordinator.attrib['collaboratorId'])
                            """
                            elif self.binding_state_material.value() == "COMMITTED":
                                self.event(coordinator.text, component, name, value, self.master_uuid,  coordinator.attrib['collaboratorId'])
                            break
                            """

                if self.binding_state_material.value() == "COMMITTED" and self.iscollaborator:
                    event = stream_root[0]
                    source = stream_root[1]
                    component = stream_root[2]
                    collabUuid = stream_root[3]
                    print "Collaborator event"
                    if self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid] and collabUuid in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid][2]:
                        if event.tag.split('}')[-1] in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid][3]:
                            print "First Filter"
                            self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid, collabUuid)
                        else:
                            try:
                                if event.tag.split('}')[-1] in str(self.master_tasks[self.master_uuid]['collaborators'][collabUuid]['SubTask'][self.collaborator.superstate.task_name]):
                                    print "Second Filter"
                                    self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid, collabUuid)
                            except:
                                "Inavlid Trigger"
                                
                #if conv is a coordinator                
                if self.iscoordinator:
                    print "3_bind"
                    event = stream_root[0]
                    source = stream_root[1]
                    component = stream_root[2]
                    collabUuid = stream_root[3]
                    print "Coord event"
                    if 'BindingState' in event.tag and event.text != "INACTIVE":
                        print "4_bind"
                        self.event(source.lower(), "Task_Collaborator", "binding_state", event.text, self.master_uuid,  collabUuid)

                    elif 'BindingState' in event.tag and event.text == "INACTIVE" and self.binding_state_material.value() == "COMMITTED":
                        self.master_tasks[self.master_uuid]['coordinator'][self.deviceUuid]['SubTask'][collabUuid][1] = 'COMPLETE'
                        self.coordinator.superstate.task.superstate.commit()
                        
                    elif self.binding_state_material.value() == "COMMITTED" and self.master_tasks[self.master_uuid]['coordinator'][self.deviceUuid]['Task'][1] == "COMMITTED":
                        self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid, collabUuid)

                    







                    """                    
                    for x in stream_root.findall('.//'+xmlns+'Streams')[0]:
                        source = x.attrib['name']
                        
                        for y in x:
                            component = y.attrib['component']

                            events = y.find('.//'+xmlns+'Events')
                            for event in events:
                                if "BindingState" in event.tag and event.text.lower() != 'unavailable':
                                    if event.text != "INACTIVE":
                                        self.event(source.lower(), "Task_Collaborator", "binding_state", event.text, self.master_uuid,  x.attrib['uuid'])

                                #NEXT UPDATE NORMAL EVENTS AS SUBTASKS
                                if event.text.lower() != 'unavailable' and self.binding_state_material.value() == "COMMITTED" and self.master_tasks[self.master_uuid]['coordinator'][self.deviceUuid]['Task'][1] == "COMMITTED":
                                    self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text)
                     """

        self.superstate = statemachineModel()

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

        

if __name__ == '__main__':
    conv1 = inputConveyor(interface)
    conv1.create_statemachine()
    conv1.superstate.has_material = True
    conv1.superstate.load_time_limit(100)
    conv1.superstate.unload_time_limit(100)
    time.sleep(7)
    conv1.superstate.enable()
    
