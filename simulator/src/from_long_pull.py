from threading import Timer, Thread
import functools, time, re
import requests, urllib2, collections
import xml.etree.ElementTree as ET

from long_pull import LongPull
from data_item import Event, SimpleCondition, Sample, ThreeDSample

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
    #if robot a collaborator
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
                    elif self.binding_state_material.value() == "COMMITTED" and value == "COMMITTED":
                        self.event(coordinator.text, 'BindingState', 'SubTask_'+name, value, self.master_uuid,  coordinator.attrib['collaboratorId'])
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

        elif self.master_tasks[self.master_uuid]['collaborators'][self.deviceUuid]['SubTask']: #single robot case
            if self.binding_state_material.value() == "COMMITTED" and event.text == "COMMITTED":
                if self.master_tasks[self.master_uuid]['coordinator'].keys()[0] == collabUuid:
                    self.event(source.lower(), 'BindingState', 'SubTask_binding_state', event.text, self.master_uuid,  collabUuid)
                else:
                    print "SOURCE"+collabUuid
            elif (collabUuid in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'] or collabUuid in self.master_tasks[self.master_uuid]['coordinator'].keys()[0]) and 'bindingstate' not in event.tag:
                coord = self.master_tasks[self.master_uuid]['coordinator'].keys()[0]
                if self.master_tasks[self.master_uuid]['coordinator'][coord]['SubTask'][coord][1] != 'COMPLETE':
                    self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid,  collabUuid)

                elif self.master_tasks[self.master_uuid]['coordinator'][coord]['SubTask'][coord][1] == 'COMPLETE' and coord!=collabUuid:
                    self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid,  collabUuid)


                    



    #if robot is a coordinator
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
