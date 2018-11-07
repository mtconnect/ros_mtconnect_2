import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd()))

#path needs to be updated in each implementation
path = os.path.join(os.getenv('HOME'), 'catkin_workspace/src/ceccrebot/simulator/src')

import xml.etree.ElementTree as ET
import uuid, re
import datetime, copy

class archetypeToInstance(object):

    def __init__(self, task, uuid, deviceUuid, parentRef = "None"):

        self.parentRef = parentRef
        self.taskArch = task
        self.jsonModel = {}
        self.taskCoordinator = None
        self.root = self.readArchetype(self.taskArch)
        self.uuid = uuid
        self.deviceUuid = deviceUuid
        self.taskIns = ET.tostring(self.toInstance(deviceUuid = self.deviceUuid))

    def readArchetype(self, taskArch):
        #read xml file into an xml tree
        try:
            fileOpen = open(os.path.join(path,'taskArchetype', taskArch +'.xml'))
            fileRead = fileOpen.read()
            root = ET.fromstring(fileRead)
            return root

        except Exception as e:
            print ("No task archetype found.",e)
            return

    def formatTaskArch(self, taskArch):
        #CamelCase to UPPER_CASE
        taskFormat = taskArch.split('_')[0]
        taskFormat = re.findall('[A-Z][^A-Z]*', taskFormat)
        return '_'.join(taskFormat).upper()

    def formatTaskType(self, taskType):
        #UPPER_CASE to CamelCase
        taskType = taskType.lower().split('_')
        for i,x in enumerate(taskType):
            taskType[i] = x.capitalize()
        return ''.join(taskType)

    def toInstance(self, deviceUuid = "None"):
        #from task archetype to task instance
        if type(self.root) == ET.Element:
            taskIns = ET.Element("Task")
            taskIns.attrib["assetId"] = str(self.uuid)
            taskIns.attrib["timestamp"] = str(datetime.datetime.now().isoformat()+'Z')
            taskIns.attrib["deviceUuid"] = str(self.deviceUuid)

            assetArchRef = ET.SubElement(taskIns, "AssetArchetypeRef")
            assetArchRef.attrib["assetId"] = str(self.taskArch)

            priority = ET.SubElement(taskIns, "Priority")
            priority.text = self.root.findall('.//'+self.root.tag.split('}')[0]+'}Priority')[0].text

            taskType = ET.SubElement(taskIns, "TaskType")
            taskType.text = self.formatTaskArch(self.taskArch).upper()

            state = ET.SubElement(taskIns, "State")

            #default initial state of the task
            state.text = str("INACTIVE")

            coordinator = ET.SubElement(taskIns, "Coordinator")

            coordinator_collab = ET.SubElement(coordinator, "Collaborator")
            coordinator_collab.attrib["collaboratorId"] = self.root.findall('.//'+self.root.tag.split('}')[0]+'}Coordinator')[0].attrib['collaboratorId']
            self.taskCoordinator = coordinator_collab.attrib["collaboratorId"]
            coordinator_collab.text = self.root.findall('.//'+self.root.tag.split('}')[0]+'}Coordinator')[0][0].text

            collaborators = ET.SubElement(taskIns, "Collaborators")

            if self.root.findall('.//'+self.root.tag.split('}')[0]+'}Collaborators'):
                for i,x in enumerate(self.root.findall('.//'+self.root.tag.split('}')[0]+'}Collaborators')[0]):
                    collaborator = ET.SubElement(collaborators, "Collaborator")
                    collaborator.attrib["collaboratorId"] = str(x.attrib['collaboratorId'])
                    collaborator.text = str(x[0].text)

            self.taskIns = taskIns
            return taskIns
        else:
            return None

    def addElement(self, string):
        #add an element to the xml node
        if type(self.taskIns) == ET.Element:
            self.taskIns.append(ET.fromstring(string))

        elif type(self.taskIns) == str:
            newTaskins = ET.fromstring(copy.deepcopy(self.taskIns))
            newTaskins.append(ET.fromstring(string))
            return ET.tostring(newTaskins)


    def traverse(self, root, jsonSubTaskModel = {}):
        #traverse through all the subtask to generate a master json task object
        if root.findall('.//'+root.tag.split('}')[0]+'}SubTaskRef'):
            jsonSubTaskModel[root.findall('.//'+root.tag.split('}')[0]+'}TaskArchetype')[0].attrib['assetId']] = {}

            for x in root.findall('.//'+root.tag.split('}')[0]+'}SubTaskRef'):

                childRoot = self.readArchetype(x.text)
                collaborators = []
                for y in childRoot.findall('.//'+childRoot.tag.split('}')[0]+'}Collaborator'):
                    collaborators.append(y.attrib['collaboratorId'])
                taskType = self.formatTaskType(childRoot.findall('.//'+root.tag.split('}')[0]+'}TaskType')[0].text)

                assetId_child = childRoot.findall('.//'+root.tag.split('}')[0]+'}TaskArchetype')[0].attrib['assetId']
                assetId_parent = root.findall('.//'+root.tag.split('}')[0]+'}TaskArchetype')[0].attrib['assetId']

                if assetId_child in jsonSubTaskModel[assetId_parent]:
                    jsonSubTaskModel[assetId_parent][assetId_child+'2'] = {
                        'order':x.attrib['order'],
                        'coordinator':childRoot.findall('.//'+root.tag.split('}')[0]+'}Coordinator')[0].attrib['collaboratorId'],
                        'collaborators':collaborators,
                        'TaskType':taskType
                        }
                else:
                    jsonSubTaskModel[assetId_parent][assetId_child] = {
                        'order':x.attrib['order'],
                        'coordinator':childRoot.findall('.//'+root.tag.split('}')[0]+'}Coordinator')[0].attrib['collaboratorId'],
                        'collaborators':collaborators,
                        'TaskType':taskType
                        }

                childroot_assetId = childRoot.findall('.//'+root.tag.split('}')[0]+'}TaskArchetype')[0].attrib['assetId']

                self.traverse(
                    root = childRoot,
                    jsonSubTaskModel = jsonSubTaskModel[root.findall('.//'+root.tag.split('}')[0]+'}TaskArchetype')[0].attrib['assetId']][childroot_assetId]
                    )

            return jsonSubTaskModel

        else:
            return jsonSubTaskModel

    def jsonInstance(self):
        #xml to json conversion
        jsonModel = {}
        jsonModel['coordinator']={}
        jsonModel['collaborators']={}
        jsonModel['priority'] = self.root.findall('.//'+self.root.tag.split('}')[0]+'}Priority')[0].text
        part_quality = self.taskArch.split('_')[-1]

        if part_quality in ['good', 'bad', 'rework']:
            jsonModel['part_quality']=part_quality

        elif part_quality == 'reworked':
            jsonModel['part_quality']='rework'

        else:
            jsonModel['part_quality']=None

        subTaskModel = self.traverse(self.root,{})
        CoordinatorSubTask = {}
        CollaboratorSubTask = {}

        for x in self.root.findall('.//'+self.root.tag.split('}')[0]+'}TaskArchetype')[0]:
            if 'coordinator' in x.tag.lower():
                coordinator = x.attrib['collaboratorId']
                coordinatorType = x[0].text.lower()
                CoordinatorSubTask[x.attrib['collaboratorId']] = []

            elif 'priority' in x.tag.lower():
                priority = x.text

            elif 'tasktype' in x.tag.lower():
                tasktype = x.text.lower()

            elif 'collaborators' in x.tag.lower():
                for y in x:
                    jsonModel['collaborators'][y.attrib['collaboratorId']]={'state':[y[0].text,y.attrib['collaboratorId'],None],'SubTask':{}}

                    CoordinatorSubTask[y.attrib['collaboratorId']] = []

        for key, val in subTaskModel.values()[0].iteritems():
            if len(val['collaborators']) == 1:
                collaborators = val['collaborators'][0]

            taskType = val['TaskType']
            order = val['order']
            CoordinatorSubTask[val['coordinator']] = [key, None, collaborators,taskType, order]

            if key in val:

                for keys, vals in val[key].iteritems():
                    if not jsonModel['collaborators'][vals['coordinator']]['SubTask']:
                        jsonModel['collaborators'][vals['coordinator']]['SubTask'][key] = []

                    elif key not in jsonModel['collaborators'][vals['coordinator']]['SubTask']:
                        jsonModel['collaborators'][vals['coordinator']]['SubTask'][key] = []

                    if vals['collaborators']:
                        subtask_collaborators = vals['collaborators']
                    else:
                        subtask_collaborators = None

                    jsonModel['collaborators'][vals['coordinator']]['SubTask'][key].append(['Interface',keys, None, vals['order'], subtask_collaborators ])

                if vals:
                    jsonModel['collaborators'][vals['coordinator']]['SubTask'][key] = sorted(jsonModel['collaborators'][vals['coordinator']]['SubTask'][key], key=lambda x: x[3])

        jsonModel['coordinator']={coordinator:{'state':[coordinatorType,coordinator,None],'Task':[tasktype,None], 'SubTask':CoordinatorSubTask}}


        self.jsonModel = jsonModel

        return jsonModel


def update(taskIns, dataitem, value):
    #update xml element text
    if type(taskIns) == ET.Element:
        if taskIns.findall('.//'+dataitem)[0].text != value:
            taskIns.attrib['timestamp'] = datetime.datetime.now().isoformat() + 'Z'
            taskIns.findall('.//'+dataitem)[0].text = value
        return taskIns
    else:
        taskIns = ET.fromstring(taskIns)
        if taskIns.findall('.//'+dataitem)[0].text != value:
            taskIns.attrib['timestamp'] = datetime.datetime.now().isoformat() + 'Z'
            taskIns.findall('.//'+dataitem)[0].text = value
        return ET.tostring(taskIns)

if __name__ == "__main__":
    a2i = archetypeToInstance("MoveMaterial_4_bad","xyz","b1")
    print a2i.jsonInstance()
