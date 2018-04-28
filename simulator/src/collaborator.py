import os, sys
sys.path.insert(0,os.getcwd()+'\\utils')
from archetypeToInstance import archetypeToInstance
import xml.etree.ElementTree as ET

from archetypeToInstance import update as assetUpdate

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, copy, uuid, re

import subTask

#will be included under assets!?
class interface(object):

    def __init__(self, value = None):
        self.value = value

class collaborator(object):

    def __init__(self, parent, interface, collaborator_name):

        class statemachineModel(object):

            def __init__(self, parent, interface, collaborator_name):

                self.interface = interface
                self.parent = parent
                self.collaborator_name = collaborator_name
                self.task_name = None
                self.id = collaborator_name #which one?
                self.subTask = {}
                self.currentSubTask = str()
                self.initialized = False
                
            def INACTIVE(self): 
                
                self.parent.adapter.begin_gather()
                self.interface.set_value("INACTIVE")
                self.parent.adapter.complete_gather()
                """
                self.task_uuid = self.parent.deviceUuid '_' + str(uuid.uuid4())
                arch2ins = archetypeToInstance(self.task_name, self.task_uuid, self.parent.deviceUuid)
                self.taskIns = arch2ins.taskIns
                self.parent.adapter.addAsset('Task', self.task_uuid, arch2ins.taskIns)
                """
                if not self.initialized:
                    self.task_created()
                    self.initialized = True

            def PREPARING(self):
                print 'enter prep'
                self.parent.adapter.begin_gather()
                self.interface.set_value("PREPARING")
                self.parent.adapter.complete_gather()
                
            def committed(self, value, code, text):
                if self.collaborator_name in value['coordinator'][text]['SubTask'] and value['coordinator'][text]['SubTask'][self.collaborator_name]:
                    self.subTask[value['coordinator'][text]['SubTask'][self.collaborator_name][0]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.parent.master_uuid, collaborators = value['coordinator'][text]['SubTask'][self.collaborator_name][2], taskName =value['coordinator'][text]['SubTask'][self.collaborator_name][0])
                    self.subTask[value['coordinator'][text]['SubTask'][self.collaborator_name][0]].create_statemachine()
                    self.subTask[value['coordinator'][text]['SubTask'][self.collaborator_name][0]].superstate.create()
                    self.currentSubTask = copy.deepcopy(value['coordinator'][text]['SubTask'][self.collaborator_name][0])
                    print "in committed method"

                    for key, val in self.parent.master_tasks[code]['collaborators'].iteritems():

                        if self.task_name in val['SubTask'].keys():
                            for i,x in enumerate(val['SubTask'][self.task_name]):
                                
                                self.subTask[x[1]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.subTask[value['coordinator'][text]['SubTask'][self.collaborator_name][0]].superstate.task_uuid, collaborators = x[4], taskName = x[1])
                                self.subTask[x[1]].create_statemachine()
                                self.subTask[x[1]].superstate.create()
                                self.currentSubTask = copy.deepcopy(x[1])

                                while self.subTask[self.currentSubTask].superstate.state != 'removed':
                                    pass
                                self.parent.master_tasks[code]['collaborators'][key]['SubTask'][self.task_name][i][2] = 'COMPLETE'

                    self.currentSubTask = copy.deepcopy(value['coordinator'][text]['SubTask'][self.collaborator_name][0])
                                
 
                    while self.subTask[self.currentSubTask].superstate.state != 'removed':
                        pass
                    self.parent.master_tasks[code]['coordinator'][text]['SubTask'][self.collaborator_name][1] = 'COMPLETE'
                    print 'exited'
                    self.completed()
                    
                    
                               
                                
            def COMMITTED(self):
                self.parent.adapter.begin_gather()
                self.interface.set_value("COMMITTED")
                self.parent.adapter.complete_gather()
                
                t1= Thread(target = self.commited_init)
                t1.start()


            def commited_init(self):
                collabUuid = False
                for key,val in self.parent.master_tasks[self.parent.master_uuid]['coordinator'][self.parent.master_tasks[self.parent.master_uuid]['coordinator'].keys()[0]]['SubTask'].iteritems():
                    print key,val
                    if val:
                        if self.parent.deviceUuid in val[2]:
                            collabUuid = True
                            self.subTask[val[0]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.parent.master_uuid, collaborators = None, taskName = val[0])
                            self.subTask[val[0]].create_statemachine()
                            self.subTask[val[0]].superstate.create()
                            self.currentSubTask = copy.deepcopy(val[0])
                            print self.currentSubTask+'in committed init'

                            if val[0] in self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.parent.deviceUuid]['SubTask']:
                                for i,x in enumerate(self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.parent.deviceUuid]['SubTask'][val[0]]):
                                    self.subTask[x[1]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.subTask[val[0]].superstate.task_uuid, collaborators = None, taskName = x[1])
                                    self.subTask[x[1]].create_statemachine()
                                    self.subTask[x[1]].superstate.create()
                                    self.currentSubTask = copy.deepcopy(x[1])
                                    self.parent.event(self.parent.deviceUuid, 'interface_intialization', 'SubTask_'+x[1],'IDLE')
                                    while self.subTask[self.currentSubTask].superstate.state != 'removed':
                                        pass
                                    self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.parent.deviceUuid]['SubTask'][val[0]][i][2] = 'COMPLETE'
                                    time.sleep(0.2)
                            self.currentSubTask = copy.deepcopy(val[0])
                            while self.subTask[self.currentSubTask].superstate.state != 'removed':
                                pass
                            coord = self.parent.master_tasks[self.parent.master_uuid]['coordinator'].keys()[0]
                            self.parent.master_tasks[self.parent.master_uuid]['coordinator'][coord]['SubTask'][key][1] = 'COMPLETE'
                            
                self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.parent.deviceUuid]['state'][2] = 'COMPLETE'

                if collabUuid == True:
                    print 'exited'
                    self.completed()


            def event(self, source, comp, name, value, code = None, text = None):
                #sample: ('cnc', 'Coordinator', 'information_model',{..}, code = 'master_task_uuid', text = 'cnc1')
                print 'event received',source, comp, name, value, code
                if comp == 'Coordinator' and 'binding_state' in name and value.lower() == 'preparing':
                    self.parent.master_tasks[code[0]] = code[1]

                #elif comp == 'Coordinator' and value.lower() == 'start':
                #        self.committed(code[1], code[0], text)
                    
                elif comp == 'Coordinator' and 'binding_state' in name:
                    if value.lower() == 'committing':
                        self.commit()

                    elif value.lower() == 'committed':
                        self.parent.master_tasks[code]['coordinator'][text]['state'][2] = value

                elif 'SubTask' in name:
                    print self.subTask
                    if not self.subTask:
                        print "no subtask"
                        def subt():
                            self.committed(self.parent.master_tasks[code],code, self.parent.master_tasks[code]['coordinator'].keys()[0])
                        t0= Thread(target = subt)
                        t0.start()
                        time.sleep(0.1)
                        self.parent.event(source, comp, name, value, code, text)
                        print self.subTask, self.currentSubTask
                    elif self.currentSubTask and self.currentSubTask in name:
                        self.subTask[self.currentSubTask].superstate.event(source, comp, name, value, code, text)
                    elif self.subTask:
                        for k,v in self.parent.master_tasks[self.parent.master_uuid]['coordinator'][self.parent.master_tasks[self.parent.master_uuid]['coordinator'].keys()[0]]['SubTask'].iteritems():
                            print k,v,name
                            if v and name.split('_')[-1] in v[3]:
                                print k,v
                                self.subTask[v[0]].superstate.event(source, comp, name, value, code, text)
                    else:
                        print "NO CURRENT SUBTASK"
                else:
                    print "in else"
                    if 'complete' in value.lower():
                        
                        self.parent.adapter.begin_gather()
                        self.interface.set_value("INACTIVE")
                        self.parent.adapter.complete_gather()
                    self.parent.event(source, comp, name, value, code, text)
                                    
                    
        self.superstate = statemachineModel(parent = parent, interface = interface, collaborator_name = collaborator_name)

    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['inactive', 'preparing', 'committed']}]

        transitions = [['unavailable', 'base', 'base:inactive'],

                       ['task_created', 'base:inactive', 'base:preparing'],

                       ['commit', 'base:preparing', 'base:committed'],

                       ['completed', 'base:committed', 'base:inactive'],
                       ['failed', 'base:committed', 'base:inactive'],

                       ['default', 'base:inactive', 'base:inactive'],
                       ['default', 'base:committed', 'base:committed']
                       ]

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)
                       
        self.statemachine.on_enter('base:inactive', 'INACTIVE')
        self.statemachine.on_enter('base:preparing', 'PREPARING')
        self.statemachine.on_enter('base:committed', 'COMMITTED')

        
