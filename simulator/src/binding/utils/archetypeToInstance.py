#sys.path.insert(0,os.getcwd()+'\\utils') #child path
#sys.path.insert(0, os.path.dirname(os.getcwd())) #parent
import xml.etree.ElementTree as ET
import uuid, re
import datetime

class archetypeToInstance(object):

    def __init__(self, task):

        self.taskIns = "Task"
        self.taskArch = task
        self.jsonModel = {}
        self.root = self.readArchetype(self.taskArch)

    def readArchetype(self, taskArch):

        fileOpen = open('../taskArchetype/'+taskArch+'.xml', 'r')
        fileRead = fileOpen.read()
        root = ET.fromstring(fileRead)

        return root

    def formatTaskArch(self, taskArch):
        taskFormat = taskArch.split('_')[0]
        taskFormat = re.findall('[A-Z][^A-Z]*', taskFormat)
        
        return '_'.join(taskFormat).upper()

    def toInstance(self, parentRef = "None", deviceUuid = "None"):
        if type(self.root) == ET.Element:
            
            taskIns = ET.Element("Task")
            taskIns.attrib["assetId"] = str(uuid.uuid4())
            taskIns.attrib["timestamp"] = str(datetime.datetime.now().isoformat()+'Z')
            taskIns.attrib["deviceUuid"] = deviceUuid

            assetArchRef = ET.SubElement(taskIns, "AssetArchetypeRef")
            assetArchRef.attrib["assetId"] = str(self.taskArch)

            priority = ET.SubElement(taskIns, "Priority")
            priority.text = str(1) #to be updated later

            taskType = ET.SubElement(taskIns, "TaskType")
            taskType.text = self.formatTaskArch(self.taskArch).upper()

            state = ET.SubElement(taskIns, "State")
            state.text = str("INACTIVE")

            coordinator = ET.SubElement(taskIns, "Coordinator")
            coordinator.attrib["collaboratorId"] = "Nothing"
            coordinator.text = self.root.findall('.//'+self.root.tag.split('}')[0]+'}Coordinator')[0][0].text

            for i,x in enumerate(self.root.findall('.//'+self.root.tag.split('}')[0]+'}Collaborators')[0]):
                collaborator = ET.SubElement(taskIns, "Collaborator")
                collaborator.attrib["collaboratorId"] = str(i+1) #update this later
                collaborator.text = self.root.findall('.//'+self.root.tag.split('}')[0]+'}Collaborator')[0][0].text
            

            self.taskIns = taskIns
            return taskIns
        else:
            return None

    def addElement(self, string):
        if type(self.taskIns) == ET.Element:
            self.taskIns.append(ET.fromstring(string))


    def traverse(self, root, jsonSubTaskModel = {}):
        if root.findall('.//'+root.tag.split('}')[0]+'}SubTaskRef'):
            jsonSubTaskModel[root.findall('.//'+root.tag.split('}')[0]+'}TaskArchetype')[0].attrib['assetId']] = {}
            
            for x in root.findall('.//'+root.tag.split('}')[0]+'}SubTaskRef'):

                childRoot = self.readArchetype(x.text)
                collaborators = []
                for y in childRoot.findall('.//'+childRoot.tag.split('}')[0]+'}Collaborator'):
                    collaborators.append(y.attrib['collaboratorId'])
                    
                jsonSubTaskModel[root.findall('.//'+root.tag.split('}')[0]+'}TaskArchetype')[0].attrib['assetId']][childRoot.findall('.//'+root.tag.split('}')[0]+'}TaskArchetype')[0].attrib['assetId']] = {'order':x.attrib['order'], 'coordinator':childRoot.findall('.//'+root.tag.split('}')[0]+'}Coordinator')[0].attrib['collaboratorId'], 'collaborators':collaborators}
         
                self.traverse(childRoot,jsonSubTaskModel[root.findall('.//'+root.tag.split('}')[0]+'}TaskArchetype')[0].attrib['assetId']][childRoot.findall('.//'+root.tag.split('}')[0]+'}TaskArchetype')[0].attrib['assetId']])

            return jsonSubTaskModel
            
        else:
            return jsonSubTaskModel

        
    #very much work in progress 
    def jsonInstance(self):
        jsonModel = {}
        jsonModel['coordinator']={}
        jsonModel['collaborators']={}
        subTaskModel = self.traverse(self.root)
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
                
            CoordinatorSubTask[val['coordinator']] = [key, None, collaborators,"src/dest"]
            if key in val:
                for keys, vals in val[key].iteritems():
                    if not jsonModel['collaborators'][vals['coordinator']]['SubTask']:
                        jsonModel['collaborators'][vals['coordinator']]['SubTask'][key] = []
                    jsonModel['collaborators'][vals['coordinator']]['SubTask'][key].append(['Interface',keys, None, vals['order'], None])

                jsonModel['collaborators'][vals['coordinator']]['SubTask'][key] = sorted(jsonModel['collaborators'][vals['coordinator']]['SubTask'][key], key=lambda x: x[3])
                    

        jsonModel['coordinator']={coordinator:{'state':[coordinatorType,coordinator,None],'Task':[tasktype,None], 'SubTask':CoordinatorSubTask}}


        self.jsonModel = jsonModel

        return jsonModel
    
            

if __name__ == "__main__":
    a2i = archetypeToInstance("MoveMaterial_1")
    a2i.toInstance()
    a2i.jsonInstance()
    a2i.traverse(a2i.root)
    
    

        

        
