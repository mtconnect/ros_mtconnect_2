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

                    #Update device binding task in priority object
                    elif 'Binding' in event.tag and 'BindingState' not in event.tag:
                        if event.text!='UNAVAILABLE':
                            self.priority.binding_state(x.attrib['uuid'],None, event.text.lower())

                    elif event.text != 'UNAVAILABLE':
                        if ('AssetChanged' in event.tag or 'BindingState' in event.tag or self.binding_state_material.value() == "COMMITTED") and event.text.lower() != 'unavailable':

                            #read asset archetype to create asset instance
                            if 'AssetChanged' in event.tag and event.text not in self.master_tasks:
                                thread2= Thread(target = self.start_pull_asset,args=(addr,"/asset/",event.text, [event,source,component,x.attrib['uuid']]))
                                thread2.start()


                            #Collaboration related event handling
                            elif 'BindingState' in event.tag:
                                stream_root = [event,source,component,x.attrib['uuid']]
                                coord_task_id = self.priority.binding_states[stream_root[3]][1]

                                if event.text == 'PREPARING' or event.text == 'INACTIVE':
                                    self.priority.binding_state(stream_root[3],event.text, None)

                                if self.iscoordinator:
                                    event = stream_root[0]
                                    source = stream_root[1]
                                    component = stream_root[2]
                                    collabUuid = stream_root[3]
                                    if 'BindingState' in event.tag and event.text != "INACTIVE" and coord_task_id == self.master_uuid:
                                        self.event(source.lower(), "Task_Collaborator", "binding_state", event.text, self.master_uuid,  collabUuid)
                                    elif 'BindingState' in event.tag and event.text == "INACTIVE" and self.binding_state_material.value() == "COMMITTED" and coord_task_id == self.master_uuid:
                                        if self.master_uuid in self.master_tasks and self.master_tasks[self.master_uuid]['coordinator'][self.deviceUuid]['SubTask'][collabUuid]:
                                            self.master_tasks[self.master_uuid]['coordinator'][self.deviceUuid]['SubTask'][collabUuid][1] = 'COMPLETE'
                                            self.master_tasks[self.master_uuid]['collaborators'][collabUuid]['state'][2] = 'INACTIVE'
                                            self.coordinator.superstate.task.superstate.commit()
                                        elif self.master_uuid in self.master_tasks and 'ToolChange' in str(self.master_tasks) and collabUuid == 'r1':
                                            self.master_tasks[self.master_uuid]['coordinator'][self.deviceUuid]['SubTask']['cnc1'][1] = 'COMPLETE'
                                            self.coordinator.superstate.task.superstate.success()

                                elif self.iscollaborator:
                                    if self.binding_state_material.value() == "PREPARING" and event.text == 'COMMITTING':# and coord_task_id == self.master_uuid:
                                        source = stream_root[1]
                                        collabUuid = stream_root[3]
                                        self.event(source.lower(), "Coordinator", "binding_state", event.text, self.master_uuid,  collabUuid)
                                    elif self.binding_state_material.value() == "COMMITTED" and self.master_uuid in self.master_tasks:# and coord_task_id == self.master_uuid:
                                        event = stream_root[0]
                                        source = stream_root[1]
                                        component = stream_root[2]
                                        collabUuid = stream_root[3]
                                        if event.text == 'INACTIVE' and 'ToolChange' in str(self.master_tasks[self.master_uuid]) and collabUuid == 'r1':
                                            self.event(source.lower(), "Coordinator", 'binding_state', event.text, self.master_uuid, collabUuid)

                                        elif self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid] and collabUuid in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid][2]:
                                            if event.tag.split('}')[-1] in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid][3]:
                                                self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid, collabUuid)
                                            else:
                                                try:
                                                    if event.tag.split('}')[-1] in str(self.master_tasks[self.master_uuid]['collaborators'][collabUuid]['SubTask'][self.collaborator.superstate.task_name]):
                                                        self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid, collabUuid)
                                                except Exception as e:
                                                    print ("Invalid Trigger",e)

                                        elif self.master_tasks[self.master_uuid]['collaborators'][self.deviceUuid]['SubTask']  or self.deviceUuid in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][collabUuid][2]: #single robot case
                                            if self.binding_state_material.value() == "COMMITTED" and event.text == "COMMITTED":
                                                if self.master_tasks[self.master_uuid]['coordinator'].keys()[0] == collabUuid:
                                                    self.event(source.lower(), 'Coordinator', 'binding_state', event.text, self.master_uuid,  collabUuid)
                                        elif self.master_tasks[self.master_uuid]['collaborators'][self.deviceUuid]:
                                            self.event(source.lower(), 'Coordinator', 'binding_state', event.text, self.master_uuid,  collabUuid)


                            #interfaces related event handling
                            elif self.binding_state_material.value() == "COMMITTED" and self.master_uuid in self.master_tasks:
                                stream_root = [event,source,component,x.attrib['uuid']]
                                coord_task_id = self.priority.binding_states[stream_root[3]][1]
                                if self.iscollaborator:# and coord_task_id == self.master_uuid:
                                    event = stream_root[0]
                                    source = stream_root[1]
                                    component = stream_root[2]
                                    collabUuid = stream_root[3]
                                    if self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid] and collabUuid in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid][2]:
                                        if event.tag.split('}')[-1] in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid][3]:
                                            self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid, collabUuid)
                                        else:
                                            try:
                                                if event.tag.split('}')[-1] in str(self.master_tasks[self.master_uuid]['collaborators'][collabUuid]['SubTask'][self.collaborator.superstate.task_name]):
                                                    self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid, collabUuid)
                                            except Exception as e:
                                                print ("Invalid Trigger",e)

                                    elif self.master_tasks[self.master_uuid]['collaborators'][self.deviceUuid]['SubTask'] or self.deviceUuid in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][collabUuid][2]: #single robot case
                                        if self.binding_state_material.value() == "COMMITTED" and event.text == "COMMITTED":
                                            if self.master_tasks[self.master_uuid]['coordinator'].keys()[0] == collabUuid:
                                                self.event(source.lower(), 'Coordinator', 'binding_state', event.text, self.master_uuid,  collabUuid)
                                        elif (collabUuid in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'] or collabUuid in self.master_tasks[self.master_uuid]['coordinator'].keys()[0]):
                                            coord = self.master_tasks[self.master_uuid]['coordinator'].keys()[0]
                                            if self.master_tasks[self.master_uuid]['coordinator'][coord]['SubTask'][coord][1] != 'COMPLETE':
                                                self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid,  collabUuid)

                                            elif self.master_tasks[self.master_uuid]['coordinator'][coord]['SubTask'][coord][1] == 'COMPLETE' and coord!=collabUuid:
                                                self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid,  collabUuid)

                                elif self.iscoordinator and self.master_uuid in self.master_tasks:# and coord_task_id == self.master_uuid:
                                    event = stream_root[0]
                                    source = stream_root[1]
                                    component = stream_root[2]
                                    collabUuid = stream_root[3]
                                    if self.binding_state_material.value() == "COMMITTED" and self.master_tasks[self.master_uuid]['coordinator'][self.deviceUuid]['Task'][1] == "COMMITTED":
                                        self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid, collabUuid)

                        #makes sure that completed task assets are removed
                        elif 'AssetRemoved' in event.tag and self.binding_state_material.value() == "INACTIVE" and event.text.lower() != 'unavailable':
                            try:
                                if self.deviceUuid in event.text.split('_')[0]:
                                        self.adapter.removeAsset(event.text)
                            except Exception as e:
                                print ("Error removing asset!")
                                print (e)

                except Exception as e:
                    print ("Invalid Event in ", self.deviceUuid, " from ",e)


def from_long_pull_asset(self,chunk, stream_root = None):
    root=ET.fromstring(chunk)
    xmlns =root.tag.split('}')[0]+'}'
    task = root.findall('.//'+xmlns+'Task')
    parentRef = None
    state = None
    if task:
        task = root.findall('.//'+xmlns+'Task')[0]
        state = root.findall('.//'+xmlns+'State')[0].text
        parentRef = root.findall('.//'+xmlns+'ParentRef')

    else:
        task = None
    if task is not None and state == "PREPARING":
        for x in root.findall('.//'+xmlns+'Collaborator'):
            if x.attrib['collaboratorId'] == self.deviceUuid:
                main_task_archetype = root.findall('.//'+xmlns+'AssetArchetypeRef')[0].attrib['assetId']
                main_task_uuid = root.findall('.//'+xmlns+'Task')[0].attrib['assetId']
                main_task_deviceUuid = root.findall('.//'+xmlns+'Task')[0].attrib['deviceUuid']
                coordinator = root.findall('.//'+xmlns+'Coordinator')[0]
                component = "Coordinator"
                name = "binding_state"
                value = state

                #create json task instance from xml task archetype
                if main_task_uuid not in self.master_tasks and value == "PREPARING":
                    self.master_tasks[main_task_uuid] = archetypeToInstance(main_task_archetype,"uuid", main_task_deviceUuid, main_task_uuid).jsonInstance()

                if value == "PREPARING":
                    self.event(coordinator.text, component, name, value, [main_task_uuid, self.master_tasks[main_task_uuid]],  coordinator.attrib['collaboratorId'])

                break
