import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd()))

from threading import Timer, Thread
import functools, time, re
import requests, urllib2, collections,datetime
import xml.etree.ElementTree as ET

from adapter.long_pull import LongPull
from adapter.data_item import Event, SimpleCondition, Sample, ThreeDSample
from archetypeToInstance import archetypeToInstance

def from_long_pull(self, chunk, addr = None):
    root=ET.fromstring(chunk)
    xmlns =root.tag.split('}')[0]+'}'
    s=root.findall('.//'+xmlns+'Streams')[0]

    for x in s:
        source = x.attrib['name']
        for y in x:
            component = y.attrib['component']
            events = y.find('.//'+xmlns+'Events')

            if events is None:
                continue

            for event in events:
                try:
                    if 'Availability' in event.tag or 'Execution' in event.tag or 'ControllerMode' in event.tag:

                        thread1= Thread(target = self.event,args=(source.lower(), component, event.tag.split('}')[-1], event.text, None, x.attrib['uuid']))
                        thread1.start()


                    elif event.text != 'UNAVAILABLE':

                        #Update device binding task in priority object
                        if 'Binding' in event.tag and 'BindingState' not in event.tag:
                            self.priority.binding_state(x.attrib['uuid'],None, event.text.lower())

                        elif 'AssetChanged' in event.tag or 'BindingState' in event.tag or self.binding_state_material.value() == "COMMITTED":

                            #device uuid of the collaborator
                            collabUuid = x.attrib['uuid']

                            coord_task_id = self.priority.binding_states[collabUuid][1]
                            if self.master_uuid in self.master_tasks:
                                coordinator = self.master_tasks[self.master_uuid]['coordinator'].keys()[0]
                                collaborators = self.master_tasks[self.master_uuid]['collaborators'].keys()

                            #read asset archetype to create asset instance
                            if 'AssetChanged' in event.tag and event.text not in self.master_tasks:
                                thread2= Thread(target = self.start_pull_asset,args=(addr,"/asset/",event.text, [event,source,component,x.attrib['uuid']]))
                                thread2.start()


                            #Collaboration related event handling
                            elif 'BindingState' in event.tag:
                                if event.text == 'PREPARING' or event.text == 'INACTIVE':
                                    self.priority.binding_state(collabUuid,event.text, None)

                                if self.is_coordinator and coord_task_id == self.master_uuid:

                                    #Before committing to the task
                                    if event.text != "INACTIVE":
                                        self.event(source.lower(), "Task_Collaborator", "binding_state", event.text, self.master_uuid,  collabUuid)

                                    #After committing to the task and/or task completion
                                    elif event.text == "INACTIVE" and self.binding_state_material.value() == "COMMITTED":
                                        if self.master_uuid in self.master_tasks:
                                            self.event(source.lower(), "Task_Collaborator", "binding_state", event.text, self.master_uuid,  collabUuid)


                                elif self.is_collaborator:

                                    #Before committing to the task
                                    if self.binding_state_material.value() == "PREPARING" and event.text == 'COMMITTING' and collabUuid == coordinator:
                                        self.event(source.lower(), "Coordinator", "binding_state", event.text, self.master_uuid,  collabUuid)

                                    #After committing to the task
                                    elif self.binding_state_material.value() == "COMMITTED" and self.master_uuid in self.master_tasks:
                                        if event.text in ['INACTIVE','COMMITTED'] and collabUuid == coordinator:
                                            self.event(source.lower(), "Coordinator", 'binding_state', event.text, self.master_uuid, collabUuid)


                            #interfaces related event handling
                            elif self.binding_state_material.value() == "COMMITTED" and self.master_uuid in self.master_tasks:

                                event_name = event.tag.split('}')[-1]

                                if self.is_collaborator:

                                    device_tasks = self.master_tasks[self.master_uuid]['coordinator'][coordinator]['SubTask'][self.device_uuid]
                                    device_subtasks = self.master_tasks[self.master_uuid]['collaborators'][self.device_uuid]['SubTask']

                                    if device_tasks: device_collaborators = device_tasks[2]
                                    else: device_collaborators = None
                                    current_device_subtask = self.collaborator.superstate.task_name

                                    if collabUuid in collaborators+[coordinator]:
                                        collab_tasks = self.master_tasks[self.master_uuid]['coordinator'][coordinator]['SubTask'][collabUuid]
                                    else:
                                        collab_tasks = None

                                    if collabUuid != coordinator and collabUuid in collaborators:
                                        collab_subtasks = self.master_tasks[self.master_uuid]['collaborators'][collabUuid]['SubTask']
                                    else:
                                        collab_subtasks = None

                                    if collab_tasks: collab_collaborators = collab_tasks[2]
                                    else: collab_collaborators = None

                                    if device_tasks and collabUuid in device_collaborators:

                                        if event_name == device_tasks[3] or event_name in str(collab_subtasks[device_tasks[0]]):
                                            self.event(source.lower(), component, 'SubTask_'+event_name, event.text, self.master_uuid, collabUuid)

                                    elif device_subtasks and collab_collaborators: #Robot / low level collaborator

                                        if collab_tasks and self.device_uuid in collab_collaborators:
                                            self.event(source.lower(), component, 'SubTask_'+event_name, event.text, self.master_uuid, collabUuid)


                                elif self.is_coordinator:

                                    self.event(source.lower(), component, 'SubTask_'+event_name, event.text, self.master_uuid, collabUuid)

                        #makes sure that completed task assets are removed
                        elif 'AssetRemoved' in event.tag and self.binding_state_material.value() == "INACTIVE":
                            try:
                                if self.device_uuid in event.text.split('_')[0]:
                                    self.adapter.removeAsset(event.text)

                            except Exception as e:
                                print ("Error removing asset!")
                                print (e)

                except Exception as e:
                    print ("Invalid Event in ", self.device_uuid, " from ",e)


def from_long_pull_asset(self,chunk, stream_root = None):

    root=ET.fromstring(chunk)
    xmlns =root.tag.split('}')[0]+'}'
    task_element = root.findall('.//'+xmlns+'Task')
    parentRef = None
    value = None

    if task_element:
        task = root.findall('.//'+xmlns+'Task')[0]
        value = root.findall('.//'+xmlns+'State')[0].text
        parentRef = root.findall('.//'+xmlns+'ParentRef')

    else:
        task = None

    if task is not None and value == "PREPARING":

        for x in root.findall('.//'+xmlns+'Collaborator'):

            if x.attrib['collaboratorId'] == self.device_uuid:

                main_task_archetype = root.findall('.//'+xmlns+'AssetArchetypeRef')[0].attrib['assetId']
                main_task_uuid = root.findall('.//'+xmlns+'Task')[0].attrib['assetId']
                main_task_device_uuid = root.findall('.//'+xmlns+'Task')[0].attrib['deviceUuid']
                coordinator = root.findall('.//'+xmlns+'Coordinator')[0]
                component = "Coordinator"
                name = "binding_state"

                #create json task instance from xml task archetype
                if main_task_uuid not in self.master_tasks:
                    self.master_tasks[main_task_uuid] = archetypeToInstance(main_task_archetype,"uuid", main_task_device_uuid, main_task_uuid).jsonInstance()

                self.event(coordinator.text, component, name, value, [main_task_uuid, self.master_tasks[main_task_uuid]],  coordinator.attrib['collaboratorId'])

                break
