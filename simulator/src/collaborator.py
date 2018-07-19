import os, sys
sys.path.insert(0,os.getcwd()+'\\utils')
from archetypeToInstance import archetypeToInstance
import xml.etree.ElementTree as ET

from archetypeToInstance import update as assetUpdate

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, copy, uuid, re, datetime

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
                self.currentSubTaskState = None
                self.concurrent_collaborator = False
                self.concurrent_task = None
                
            def INACTIVE(self): 

                if self.concurrent_task and self.concurrent_collaborator:
                    self.parent.enable()
                self.subTask = {}
                self.currentSubTask = str()
                self.parent.adapter.begin_gather()
                self.interface.set_value("INACTIVE")
                self.parent.adapter.complete_gather()


            def PREPARING(self):
                
                self.parent.adapter.begin_gather()
                self.interface.set_value("PREPARING")
                self.parent.adapter.complete_gather()
                
            def committed(self, value, code, text):
                if self.collaborator_name in value['coordinator'][text]['SubTask'] and value['coordinator'][text]['SubTask'][self.collaborator_name]:
                    self.subTask[value['coordinator'][text]['SubTask'][self.collaborator_name][0]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.parent.master_uuid, collaborators = value['coordinator'][text]['SubTask'][self.collaborator_name][2], taskName =value['coordinator'][text]['SubTask'][self.collaborator_name][0])
                    self.subTask[value['coordinator'][text]['SubTask'][self.collaborator_name][0]].create_statemachine()
                    self.subTask[value['coordinator'][text]['SubTask'][self.collaborator_name][0]].superstate.create()
                    self.currentSubTask = copy.deepcopy(value['coordinator'][text]['SubTask'][self.collaborator_name][0])
                    
                    for key, val in self.parent.master_tasks[code]['collaborators'].iteritems():
                        
                        if self.task_name in val['SubTask'].keys():
                            for i,x in enumerate(val['SubTask'][self.task_name]):
                                
                                if x and x[4] and self.parent.deviceUuid in x[4]:
                                    self.subTask[x[1]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.subTask[value['coordinator'][text]['SubTask'][self.collaborator_name][0]].superstate.task_uuid, collaborators = x[4], taskName = x[1])
                                    self.subTask[x[1]].create_statemachine()
                                    self.subTask[x[1]].superstate.create()
                                    self.currentSubTask = copy.deepcopy(x[1])

                                    while self.currentSubTask and self.subTask[self.currentSubTask].superstate.state != 'removed':
                                        pass
                                    self.parent.master_tasks[code]['collaborators'][key]['SubTask'][self.task_name][i][2] = 'COMPLETE'

                                    self.currentSubTask = copy.deepcopy(value['coordinator'][text]['SubTask'][self.collaborator_name][0])
                                
 
                    while self.currentSubTask and self.subTask[self.currentSubTask].superstate.state != 'removed':
                        pass
                    self.parent.master_tasks[code]['coordinator'][text]['SubTask'][self.collaborator_name][1] = 'COMPLETE'


                    if 'ToolChange' in str(self.parent.master_tasks):
                        "Wait for completion"
                    else:
                        self.completed()
                    
                    
                               
                                
            def COMMITTED(self):
                self.parent.adapter.begin_gather()
                self.interface.set_value("COMMITTED")
                self.parent.adapter.complete_gather()

                
            def commited_init(self):
                collabUuid = False
                self.ordered_tasks = []
                for key,val in self.parent.master_tasks[self.parent.master_uuid]['coordinator'][self.parent.master_tasks[self.parent.master_uuid]['coordinator'].keys()[0]]['SubTask'].iteritems():
                    if val:
                        self.ordered_tasks.append([val[4],key,val])
                self.ordered_tasks.sort()
                        
                for i,z in enumerate(self.ordered_tasks):
                    key = z[1]
                    val = z[2]

                    if val:
                        if self.parent.deviceUuid in val[2]:
                            collabUuid = True
                            self.subTask[val[0]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.parent.master_uuid, collaborators = None, taskName = val[0])
                            self.subTask[val[0]].create_statemachine()
                            self.subTask[val[0]].superstate.create()
                            self.currentSubTask = copy.deepcopy(val[0])
                            coord = self.parent.master_tasks[self.parent.master_uuid]['coordinator'].keys()[0]
                            
                            if val[0] in self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.parent.deviceUuid]['SubTask']:
                                for i,x in enumerate(self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.parent.deviceUuid]['SubTask'][val[0]]):
                                    if not x[4]:
                                        #internal event
                                        
                                        while self.currentSubTaskState != 'active' and val[0] != 'ToolChange':
                                            pass
                                        
                                        self.parent.event(self.parent.deviceUuid, 'internal_event', x[1], 'ACTIVATE', None, key)
                                        self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.parent.deviceUuid]['SubTask'][val[0]][i][2] = 'COMPLETE'
                                        
                                    else:
                                        self.subTask[x[1]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.subTask[val[0]].superstate.task_uuid, collaborators = None, taskName = x[1])
                                        self.subTask[x[1]].create_statemachine()
                                        self.subTask[x[1]].superstate.create()
                                        self.currentSubTask = copy.deepcopy(x[1])
                                        
                                        self.parent.event(self.parent.deviceUuid, 'interface_initialization', 'SubTask_'+x[1],'IDLE')
                                        while self.currentSubTask and self.subTask[self.currentSubTask].superstate.state != 'removed':
                                            pass
                                        self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.parent.deviceUuid]['SubTask'][val[0]][i][2] = 'COMPLETE'
                                        
                                        
                            self.currentSubTask = copy.deepcopy(val[0])
                            while self.currentSubTask and self.subTask[self.currentSubTask].superstate.state != 'removed':
                                if 'Unload' in self.currentSubTask:
                                    self.subTask[self.currentSubTask].superstate.success()
                                    self.parent.material_unload_interface.superstate.complete()
                                    break
                                elif 'Load' in self.currentSubTask:
                                    self.subTask[self.currentSubTask].superstate.success()
                                    
                                    if 'ToolChange' in str(self.parent.master_tasks):
                                        self.parent.material_load_interface.superstate.complete()
                                    break
                            
                            self.parent.master_tasks[self.parent.master_uuid]['coordinator'][coord]['SubTask'][key][1] = 'COMPLETE'
                            self.currentSubTaskState = None

                if collabUuid == True:
                    self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.parent.deviceUuid]['state'][2] = 'COMPLETE'
                    if 'Load' in self.currentSubTask:
                        self.parent.material_load_interface.superstate.complete()
                    self.completed()
                    self.parent.master_tasks = {}
                    self.parent.IDLE()
                    
                else:
                    self.committed(self.parent.master_tasks[self.parent.master_uuid],self.parent.master_uuid, self.parent.master_tasks[self.parent.master_uuid]['coordinator'].keys()[0])


            def event(self, source, comp, name, value, code = None, text = None):

                #print "\nCollabkEvent Enter",source,comp,name,value,datetime.datetime.now().isoformat()
                if name == 'binding_state' and value.lower() == 'inactive' and self.parent.binding_state_material.value().lower() == 'committed':
                    self.subTask[self.currentSubTask].superstate.success()
                    time.sleep(0.1)
                    self.completed()
                    
                elif comp == 'Coordinator' and name == 'binding_state' and value.lower() == 'preparing':
                    self.parent.master_tasks[code[0]] = code[1]
                    self.task_created()

                elif comp == 'Coordinator' and  name == 'binding_state':
                    if value.lower() == 'committing':
                        if self.parent.deviceUuid != 'r1':
                            if self.task_name != self.parent.master_tasks[code]['coordinator'][text]['SubTask'][self.parent.deviceUuid][0]:
                                self.concurrent_task = self.task_name
                                self.task_name = self.parent.master_tasks[code]['coordinator'][text]['SubTask'][self.parent.deviceUuid][0]
                                self.concurrent_collaborator =True
                        self.commit()

                    elif value.lower() == 'committed' and text in self.parent.master_tasks[code]['coordinator']:
                        self.parent.master_tasks[code]['coordinator'][text]['state'][2] = value

                        t1= Thread(target = self.commited_init)
                        t1.start()

                elif 'SubTask' in name:
                    
                    if 'binding_state' in name and 'ToolChange' in str(self.parent.master_tasks) and value.lower() == 'inactive' and self.parent.binding_state_material.value().lower()=='committed' and text == 'r1':
                        self.subTask[self.currentSubTask].superstate.success()
                        time.sleep(0.1)
                        self.completed()

                    elif self.currentSubTask and self.currentSubTask in name:
                        self.subTask[self.currentSubTask].superstate.event(source, comp, name, value, code, text)
                        if not self.currentSubTaskState and name.split('_')[1] in self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.parent.deviceUuid]['SubTask']:
                            self.currentSubTaskState = value.lower()
                        
                    elif self.subTask:
                        
                        for k,v in self.parent.master_tasks[self.parent.master_uuid]['coordinator'][self.parent.master_tasks[self.parent.master_uuid]['coordinator'].keys()[0]]['SubTask'].iteritems():
                            
                            if v and name.split('_')[-1] in v[3] and v[0] in self.subTask:
                                try:
                                    self.subTask[v[0]].superstate.event(source, comp, name, value, code, text)
                                    self.currentSubTaskState = value.lower()
                                except:
                                    time.sleep(0.5)
                                    self.subTask[v[0]].superstate.event(source, comp, name, value, code, text)
                                    self.currentSubTaskState = value.lower()
                            elif v and name.split('_')[-1] in v[3] and v[0] not in self.subTask:
                                time.sleep(0.500)
                                self.parent.event(source, comp, name.split('_')[-1], value, code, text)
                                self.currentSubTaskState = value.lower()
                            else:
                                collab = None
                                if k == self.parent.deviceUuid:
                                    if v: collab = v[2]
                                    
                                if collab and self.parent.master_tasks[self.parent.master_uuid]['collaborators'][collab]['SubTask'][self.task_name]:
                                    for t in self.parent.master_tasks[self.parent.master_uuid]['collaborators'][collab]['SubTask'][self.task_name]:
                                        if name.split('_')[-1] == t[1]:
                                            self.parent.event(source, comp, name.split('_')[-1], value, code, text)
                                            break
                    else:
                        "Not a valid SubTask"
                else:

                    if value.lower() == 'complete' and None:
                        
                        self.parent.adapter.begin_gather()
                        self.interface.set_value("INACTIVE")
                        self.parent.adapter.complete_gather()
                        
                    
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
                       ['default', 'base:committed', 'base:committed'],
                       ['default', 'base', 'base']
                       ]

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)
                       
        self.statemachine.on_enter('base:inactive', 'INACTIVE')
        self.statemachine.on_enter('base:preparing', 'PREPARING')
        self.statemachine.on_enter('base:committed', 'COMMITTED')

        

