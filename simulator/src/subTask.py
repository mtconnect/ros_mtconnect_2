import os, sys
sys.path.insert(0,os.getcwd()+'\\utils')

from archetypeToInstance import archetypeToInstance

from archetypeToInstance import update as assetUpdate

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, uuid

#will be included under assets!?
class interface(object):

    def __init__(self, value = None):
        self.value = value

class subTask(object):

    def __init__(self, interface, parent, master_task_uuid, collaborators, taskName):

        class statemachineModel(object):

            def __init__(self, interface, parent, master_task_uuid, collaborators, taskName):

                self.interface = interface
                self.parent = parent
                self.master_task_uuid = master_task_uuid
                self.collaborators = collaborators
                self.commit_time_limit = 2.0
                self.taskName = taskName
                self.task_uuid = None
                self.taskIns = "xml/text"

            def INACTIVE(self):
                self.activated()

                self.task_uuid = self.parent.deviceUuid+'_'+str(uuid.uuid4())
                
                arch2ins = archetypeToInstance(self.taskName, self.task_uuid, self.parent.deviceUuid, self.master_task_uuid)
                self.taskIns = arch2ins.addElement('<ParentRef assetId = "' + str(self.master_task_uuid)+'" />')
                self.parent.adapter.addAsset('Task', self.task_uuid, self.taskIns)
                
                self.quorum()

            def COMMITTING(self):
                #self.interface.value = 'COMMITTING'
                self.taskIns = assetUpdate(self.taskIns, "State", "COMMITTING")
                self.parent.adapter.addAsset('Task', self.task_uuid, self.taskIns)
            
                self.all_commit()

            def COMMITTED(self):
                "self.interface.value = 'COMMITTED'"
                #self.check_for_subTasks()
                self.taskIns = assetUpdate(self.taskIns, "State", "COMMITTED")
                self.parent.adapter.addAsset('Task', self.task_uuid, self.taskIns)
                

            def event(self, source, comp, name, value, code = None , text = None):
                print source,comp,name,value,code,text
                if 'SubTask' in name:
                    if value.lower() == 'complete':
                        self.success()
                        time.sleep(0.1)
                        self.parent.event(source, comp, name.split('_')[1], value, code , text)
                        
                    elif 'fail' in value.lower():
                        self.failure()
                        time.sleep(0.1)
                        self.parent.event(source, comp, name.split('_')[1], value, code , text)

                    elif self.state == 'base:committed' and value.lower() == 'ready': #door/chuck response states.. update later
                        self.parent.event(source, comp, name.split('_')[1], value, code , text)
                        
                    elif self.state == 'base:committed' and (('chuck' in name.lower() or 'door' in name.lower()) and value.lower() == 'active'):
                        self.parent.event(source, comp, name.split('_')[1], value, code , text)
                        time.sleep(1.2)
                        self.success()

                    else:
                        self.parent.event(source, comp, name.split('_')[1], value, code , text)
                
                

            def COMPLETE(self):
                #self.interface.value = 'COMPLETE'
                self.taskIns = assetUpdate(self.taskIns, "State", "INACTIVE")
                self.parent.adapter.addAsset('Task', self.task_uuid, self.taskIns)
                time.sleep(0.1)
                self.parent.adapter.removeAsset(self.task_uuid)
                time.sleep(0.1)
                self.default()

            def FAIL(self):
                #self.interface.value = 'FAIL'
                self.taskIns = assetUpdate(self.taskIns, "State", "INACTIVE")
                self.parent.adapter.addAsset('Task', self.task_uuid, self.taskIns)
                time.sleep(0.1)
                self.parent.adapter.removeAsset(self.task_uuid)
                time.sleep(0.1)
                self.default()

            def void(self):
                pass

        self.superstate = statemachineModel(parent =parent, master_task_uuid= master_task_uuid, collaborators=collaborators, interface = interface, taskName = taskName)

    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['inactive', 'preparing', 'committing', 'committed', 'complete', 'fail']}, 'removed']

        transitions = [['create', 'base', 'base:inactive'],

                       ['activated', 'base:inactive', 'base:preparing'],
                       ['failure', 'base:inactive', 'base:fail'],
                       
                       ['quorum', 'base:preparing', 'base:committing'],
                       ['all_commit', 'base:committing', 'base:committed'],
                       ['no_commit', 'base:committing', 'base:preparing'],

                       ['success', 'base:committed', 'base:complete'],
                       ['failure', 'base:committed', 'base:fail'],

                       ['default', 'base:complete', 'removed'],
                       ['default', 'base:fail', 'removed'],

                       ['default', 'base:inactive', 'base:inactive'],
                       ['default', 'base:committed', 'base:committed']
                       ]

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)
                       
        self.statemachine.on_enter('base:inactive', 'INACTIVE')
        self.statemachine.on_enter('base:committing', 'COMMITTING')
        self.statemachine.on_enter('base:committed', 'COMMITTED')
        self.statemachine.on_enter('base:complete', 'COMPLETE')
        self.statemachine.on_enter('base:fail', 'FAIL')
