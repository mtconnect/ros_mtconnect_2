"""
Sample module for implementing a robot that coordinates with a CNC and conveyors.
"""

from material import *
from door import *
from chuck import *
from coordinator import *
from collaborator import *
from mtconnect_adapter import Adapter
from long_pull import LongPull
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from archetypeToInstance import archetypeToInstance

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, re
import requests, urllib2, collections
import xml.etree.ElementTree as ET

class Robot:
    class StateModel:
        """The model for MTConnect behavior in the robot."""
        def __init__(self):

            self.adapter = Adapter(('localhost',7884))

            self.mode1 = Event('mode')
            self.adapter.add_data_item(self.mode1)

            self.e1 = Event('exec')
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

            self.material_load = Event('material_load')
            self.adapter.add_data_item(self.material_load)

            self.material_unload = Event('material_unload')
            self.adapter.add_data_item(self.material_unload)

            self.adapter.start()

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

            self.events = []

            self.Event = collections.namedtuple('Event', ['source', 'component', 'name', 'value', 'code', 'text'])
            
            self.master_tasks ={}

            self.deviceUuid = "r1"

            self.master_uuid = 'r1.1'

            self.iscoordinator = False
            self.iscollaborator = False

            self.adapter.begin_gather()
            
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
                
            thread= Thread(target = self.start_pull,args=("http://localhost:5005","/sample?interval=100&count=1000",self.from_long_pull))
            thread.start()

            thread2= Thread(target = self.start_pull,args=("http://localhost:5007","/sample?interval=100&count=1000",self.from_long_pull))
            thread2.start()

        def start_pull(self,addr,request, func, stream = True):
                
            response = requests.get(addr+request, stream=stream)
            lp = LongPull(response, addr)
            lp.long_pull(func)

        def start_pull_asset(self, addr, request, assetId, stream_root):
            response = urllib2.urlopen(addr+request).read()
            self.from_long_pull_asset(response, stream_root)


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

        def event(self, source, comp, name, value, code = None, text = None):
            """Process events.

            :type ev: .event.Event
            """
            ev = self.Event(source, comp, name, value, code, text)
            
            print('Robot received: ', ev)
            self.events.append(ev)

            if comp == "Collaborator" and action!='unavailable':
                self.coordinator.superstate.event(source, comp, name, value, code, text)

            elif comp == "Coordinator" and action!='unavailable':
                self.collaborator.superstate.event(source, comp, name, value, code, text)

            elif 'SubTask' in name and action!='unavailable':
                if self.iscoordinator == True:
                    self.coordinator.superstate.event(source, comp, name, value, code, text)

                elif self.iscollaborator == True:
                    self.collaborator.superstate.event(source, comp, name, value, code, text)

            elif ev.name.startswith('Material'):
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
            #if cnc a collaborator
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

                        if self.master_uuid not in self.master_tasks:
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
                        """
                        break


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
                        
                
    
            #if cnc is a coordinator                
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
