from threading import Timer, Thread
import functools, time, re
import requests, urllib2, collections,datetime
import xml.etree.ElementTree as ET

from long_pull import LongPull
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from archetypeToInstance import archetypeToInstance

def from_long_pull(self, chunk, addr = None):
    #print '\nFROM PULL START'+datetime.datetime.now().isoformat()
    root=ET.fromstring(chunk)
    xmlns =root.tag.split('}')[0]+'}'
    s=root.findall('.//'+xmlns+'Streams')[0]
    #print '\nFROM PULL START2'+root[0].attrib['creationTime']+datetime.datetime.now().isoformat()
    for x in s:
        source = x.attrib['name']
        for y in x:
            component = y.attrib['component']

            events = y.find('.//'+xmlns+'Events')
            if events is None:
                continue
            for event in events:
                if True:
                    #THIS CLAUSE? DO WE NEED IT?
                    if 'Availability' in event.tag or 'Execution' in event.tag or 'ControllerMode' in event.tag:
                        #print "1_avail"

                        thread1= Thread(target = self.event,args=(source.lower(), component, event.tag.split('}')[-1], event.text))
                        thread1.start()

                    elif event.text != 'UNAVAILABLE': #if 'Asset' in event.tag:
                        if ('AssetChanged' in event.tag or 'BindingState' in event.tag or self.binding_state_material.value() == "COMMITTED") and event.text.lower() != 'unavailable':

                            #print event.tag+datetime.datetime.now().isoformat()
                            if 'AssetChanged' in event.tag and self.binding_state_material.value() != "COMMITTED" and event.text not in self.master_tasks:
                                #print '\nACEVENT:: '+event.text
                                thread2= Thread(target = self.start_pull_asset,args=(addr,"/asset/",event.text, [event,source,component,x.attrib['uuid']]))
                                thread2.start()

                            elif 'BindingState' in event.tag:
                                #print "2_bind"
                                #print '\nBSEVENT:: '+event.text
                                stream_root = [event,source,component,x.attrib['uuid']]
                                if self.iscoordinator:
                                    #print "3_bind"
                                    event = stream_root[0]
                                    source = stream_root[1]
                                    component = stream_root[2]
                                    collabUuid = stream_root[3]
                                    #print "Coord event"
                                    if 'BindingState' in event.tag and event.text != "INACTIVE":
                                        #print "4_bind"
                                        self.event(source.lower(), "Task_Collaborator", "binding_state", event.text, self.master_uuid,  collabUuid)
                                    elif 'BindingState' in event.tag and event.text == "INACTIVE" and self.binding_state_material.value() == "COMMITTED" and self.master_uuid in self.master_tasks:
                                        #print "5 bind"
                                        if self.master_tasks[self.master_uuid]['coordinator'][self.deviceUuid]['SubTask'][collabUuid]:
                                            self.master_tasks[self.master_uuid]['coordinator'][self.deviceUuid]['SubTask'][collabUuid][1] = 'COMPLETE'
                                            self.master_tasks[self.master_uuid]['collaborators'][collabUuid]['state'][2] = 'INACTIVE'
                                            self.coordinator.superstate.task.superstate.commit()
                                        elif 'ToolChange' in str(self.master_tasks) and collabUuid == 'r1':
                                            self.master_tasks[self.master_uuid]['coordinator'][self.deviceUuid]['SubTask']['cnc1'][1] = 'COMPLETE'
                                            self.coordinator.superstate.task.superstate.success()

                                elif self.iscollaborator:
                                    #print '\nBSEVENT:: '+event.text
                                    if self.binding_state_material.value() == "PREPARING" and event.text == 'COMMITTING':
                                        source = stream_root[1]
                                        collabUuid = stream_root[3]
                                        self.event(source.lower(), "Coordinator", "binding_state", event.text, self.master_uuid,  collabUuid)
                                    elif self.binding_state_material.value() == "COMMITTED" and self.master_uuid in self.master_tasks:
                                        event = stream_root[0]
                                        source = stream_root[1]
                                        component = stream_root[2]
                                        collabUuid = stream_root[3]
                                        #print "Collaborator event"
                                        if event.text == 'INACTIVE' and 'ToolChange' in str(self.master_tasks) and collabUuid == 'r1':
                                            self.event(source.lower(), component, 'SubTask_binding_state', event.text, self.master_uuid, collabUuid)

                                        elif self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid] and collabUuid in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid][2]:
                                            if event.tag.split('}')[-1] in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid][3]:
                                                #print "First Filter"
                                                self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid, collabUuid)
                                            else:
                                                if True:
                                                    if event.tag.split('}')[-1] in str(self.master_tasks[self.master_uuid]['collaborators'][collabUuid]['SubTask'][self.collaborator.superstate.task_name]):
                                                        #print "Second Filter"
                                                        self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid, collabUuid)
                                                if False:
                                                    "Inavlid Trigger"

                                        elif self.master_tasks[self.master_uuid]['collaborators'][self.deviceUuid]['SubTask']  or self.deviceUuid in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][collabUuid][2]: #single robot case
                                            if self.binding_state_material.value() == "COMMITTED" and event.text == "COMMITTED":
                                                if self.master_tasks[self.master_uuid]['coordinator'].keys()[0] == collabUuid:
                                                    self.event(source.lower(), 'Coordinator', 'binding_state', event.text, self.master_uuid,  collabUuid)
                                        elif self.master_tasks[self.master_uuid]['collaborators'][self.deviceUuid]:
                                            #print 'going to the non robot collab'
                                            self.event(source.lower(), 'Coordinator', 'binding_state', event.text, self.master_uuid,  collabUuid)


                            elif self.binding_state_material.value() == "COMMITTED" and self.master_uuid in self.master_tasks:
                                stream_root = [event,source,component,x.attrib['uuid']]
                                if self.iscollaborator:
                                    event = stream_root[0]
                                    source = stream_root[1]
                                    component = stream_root[2]
                                    collabUuid = stream_root[3]
                                    #print "Collaborator event"
                                    if self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid] and collabUuid in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid][2]:
                                        if event.tag.split('}')[-1] in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][self.deviceUuid][3]:
                                            #print "First Filter"
                                            self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid, collabUuid)
                                        else:
                                            if True:
                                                if event.tag.split('}')[-1] in str(self.master_tasks[self.master_uuid]['collaborators'][collabUuid]['SubTask'][self.collaborator.superstate.task_name]):
                                                    #print "Second Filter"
                                                    self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid, collabUuid)
                                            if False:
                                                "Inavlid Trigger"

                                    elif self.master_tasks[self.master_uuid]['collaborators'][self.deviceUuid]['SubTask'] or self.deviceUuid in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'][collabUuid][2]: #single robot case
                                        if self.binding_state_material.value() == "COMMITTED" and event.text == "COMMITTED":
                                            if self.master_tasks[self.master_uuid]['coordinator'].keys()[0] == collabUuid:
                                                self.event(source.lower(), 'Coordinator', 'binding_state', event.text, self.master_uuid,  collabUuid)
                                            else:
                                                """#print "SOURCE"+collabUuid"""
                                        elif (collabUuid in self.master_tasks[self.master_uuid]['coordinator'][self.master_tasks[self.master_uuid]['coordinator'].keys()[0]]['SubTask'] or collabUuid in self.master_tasks[self.master_uuid]['coordinator'].keys()[0]):
                                            coord = self.master_tasks[self.master_uuid]['coordinator'].keys()[0]
                                            if self.master_tasks[self.master_uuid]['coordinator'][coord]['SubTask'][coord][1] != 'COMPLETE':
                                                self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid,  collabUuid)

                                            elif self.master_tasks[self.master_uuid]['coordinator'][coord]['SubTask'][coord][1] == 'COMPLETE' and coord!=collabUuid:
                                                self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid,  collabUuid)


                                elif self.iscoordinator and self.master_uuid in self.master_tasks:
                                    #print "3_bind"
                                    event = stream_root[0]
                                    source = stream_root[1]
                                    component = stream_root[2]
                                    collabUuid = stream_root[3]
                                    #print "Coord event"
                                    if self.binding_state_material.value() == "COMMITTED" and self.master_tasks[self.master_uuid]['coordinator'][self.deviceUuid]['Task'][1] == "COMMITTED":
                                        self.event(source.lower(), component, 'SubTask_'+event.tag.split('}')[-1], event.text, self.master_uuid, collabUuid)

                                    else:
                                        """#print 'INVALID COORDINATOR EVENT'"""


                        elif 'AssetRemoved' in event.tag and self.binding_state_material.value() == "INACTIVE" and event.text.lower() != 'unavailable':

                            #print 'REMOVED'+event.tag+'\n'
                            try:
                                if self.deviceUuid in event.text.split('_')[0]:
                                        self.adapter.removeAsset(event.text)
                            except:
                                "THIS CLAUSE IS FOR MAKING SURE THE ASSET IS REMOVED WHEN COMPLETED."
                        else:

                            """print 'BAD'+event.tag+'\n'"""
                    """
                    else: #do we need it here?
                        if self.binding_state_material.value() == "COMMITTED" or ('Availability' or 'Execution' or 'ControllerMode' in event.tag):
                            thread1= Thread(target = self.event,args=(source.lower(), component, event.tag.split('}')[-1], event.text))
                            thread1.start()
                    """
                if False:
                    print "Invalid Event"
    #print '\nFROM PULL END'+root[0].attrib['creationTime']+datetime.datetime.now().isoformat()
def from_long_pull_asset(self,chunk, stream_root = None):
    root=ET.fromstring(chunk)
    #print '\nafternode'+root[0].attrib['creationTime']+datetime.datetime.now().isoformat()
    xmlns =root.tag.split('}')[0]+'}'
    task = root.findall('.//'+xmlns+'Task')
    parentRef = None
    if task:
        task = root.findall('.//'+xmlns+'Task')[0]
        state = root.findall('.//'+xmlns+'State')[0].text
        parentRef = root.findall('.//'+xmlns+'ParentRef')
    else:
        task = None
    #if robot a collaborator
    if task is not None and not parentRef:
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

                if self.master_uuid not in self.master_tasks and value == "PREPARING":
                    self.master_tasks[main_task_uuid] = archetypeToInstance(main_task_archetype,"uuid", main_task_deviceUuid, main_task_uuid).jsonInstance()

                if self.binding_state_material.value() == "INACTIVE": #was PREPARING BEFORE THE CHANGE IN COLLAB
                    if value == "PREPARING":
                        self.event(coordinator.text, component, name, value, [self.master_uuid, self.master_tasks[main_task_uuid]],  coordinator.attrib['collaboratorId'])
                    elif value == "COMMITTING":
                        self.event(coordinator.text, component, name, value, self.master_uuid,  coordinator.attrib['collaboratorId'])

                break
