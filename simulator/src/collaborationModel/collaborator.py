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
                self.subTask = {}
                self.currentSubTask = str()
                self.initialized = False
                self.currentSubTaskState = None

            def INACTIVE(self):
                self.subTask = {}
                self.currentSubTask = str()
                self.parent.adapter.begin_gather()
                self.interface.set_value("INACTIVE")
                self.parent.adapter.complete_gather()

            def PREPARING(self):
                self.parent.adapter.begin_gather()
                self.interface.set_value("PREPARING")
                self.parent.adapter.complete_gather()

            def COMMITTED(self):
                self.parent.adapter.begin_gather()
                self.interface.set_value("COMMITTED")
                self.parent.adapter.complete_gather()

            def committed(self):
                subTask_collab = False
                collaborator = False
                self.ordered_tasks = []
                coordinator = self.parent.master_tasks[self.parent.master_uuid]['coordinator'].keys()[0]

                for key,val in self.parent.master_tasks[self.parent.master_uuid]['coordinator'][coordinator]['SubTask'].iteritems():
                    if val:
                        self.ordered_tasks.append([val[4],key,val])
                self.ordered_tasks.sort()

                for i,z in enumerate(self.ordered_tasks):
                    key = z[1]
                    val = z[2]
                    if val:
                        if self.collaborator_name in val[2]: #val[2] is the list of collaborators involved in the lowest level task coordinated by key
                            subTask_collab = True

                        elif self.collaborator_name == key:
                            collaborator = True

                        elif not collaborator and key == coordinator:
                            continue

                        elif collaborator and key!= self.collaborator_name:
                            continue

                        if (collaborator and key == self.collaborator_name) or subTask_collab:

                            self.subTask[val[0]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.parent.master_uuid, collaborators = None, taskName = val[0])
                            self.subTask[val[0]].create_statemachine()
                            self.subTask[val[0]].superstate.create()
                            self.currentSubTask = copy.deepcopy(val[0])

                            if val[2]:
                                if val[0] in self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.collaborator_name]['SubTask']:
                                    collaborator_name = self.collaborator_name
                                elif val[2]:
                                    collaborator_name = val[2]
                                else:
                                    collaborator_name = None
                                    continue

                                initialize_tasks = False

                                for i,x in enumerate(self.parent.master_tasks[self.parent.master_uuid]['collaborators'][collaborator_name]['SubTask'][val[0]]):

                                    if not x[4] and collaborator_name == self.collaborator_name:
                                        #internal events: no interfaces
                                        while self.currentSubTaskState != 'active' and not initialize_tasks:
                                            pass
                                        initialize_tasks = True
                                        self.parent.event(self.collaborator_name, 'internal_event', x[1], 'ACTIVATE', None, key)
                                        self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.collaborator_name]['SubTask'][val[0]][i][2] = 'COMPLETE'

                                    elif x[4]:

                                        #interfaces
                                        initialize_tasks = True
                                        self.subTask[x[1]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.subTask[val[0]].superstate.task_uuid, collaborators = None, taskName = x[1])
                                        self.subTask[x[1]].create_statemachine()
                                        self.subTask[x[1]].superstate.create()
                                        self.currentSubTask = copy.deepcopy(x[1])

                                        if subTask_collab:
                                            self.parent.event(self.collaborator_name, 'interface_initialization', 'SubTask_'+x[1],'IDLE')
                                        while self.currentSubTask and self.subTask[self.currentSubTask].superstate.state != 'removed':
                                            pass
                                        self.parent.master_tasks[self.parent.master_uuid]['collaborators'][collaborator_name]['SubTask'][val[0]][i][2] = 'COMPLETE'

                            self.currentSubTask = copy.deepcopy(val[0])

                            while self.currentSubTask and self.subTask[self.currentSubTask].superstate.state != 'removed':
                                if subTask_collab: break
                                else: pass

                            if subTask_collab:
                                self.subTask[self.currentSubTask].superstate.success()
                                self.parent.event(self.collaborator_name, 'interface_completion', val[3],'COMPLETE')

                            self.parent.master_tasks[self.parent.master_uuid]['coordinator'][coordinator]['SubTask'][key][1] = 'COMPLETE'

                if subTask_collab == True:
                    self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.parent.deviceUuid]['state'][2] = 'COMPLETE'
                    self.completed()
                    self.parent.IDLE()

                elif collaborator and len(self.ordered_tasks) == 2:
                    self.completed()

                elif collaborator:
                    pass


            def event(self, source, comp, name, value, code = None, text = None):

		try:
                    #collaboration related event handling
                    if name == 'binding_state' and value.lower() == 'inactive' and self.parent.binding_state_material.value().lower() == 'committed' and self.currentSubTask:
                        self.subTask[self.currentSubTask].superstate.success()
                        time.sleep(0.1)
                        self.completed()
                    elif comp == 'Coordinator' and name == 'binding_state' and value.lower() == 'preparing':
                        self.parent.master_tasks[code[0]] = code[1]
                        self.task_created()

                    elif comp == 'Coordinator' and  name == 'binding_state':
                        if value.lower() == 'committing':
                            self.commit()

                        elif value.lower() == 'committed' and text in self.parent.master_tasks[code]['coordinator']:
                            self.parent.master_tasks[code]['coordinator'][text]['state'][2] = value

                            t1= Thread(target = self.committed)
                            t1.start()

                    #interfaces related event handling
                    elif 'SubTask' in name:
                        if self.currentSubTask and self.currentSubTask in name:
                            try:
                                self.subTask[self.currentSubTask].superstate.event(source, comp, name, value, code, text)
                                if not self.currentSubTaskState and name.split('_')[1] in self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.parent.deviceUuid]['SubTask']:
                                    self.currentSubTaskState = value.lower()

                            except Exception as e:
				print (e)
				print ("Retrying in 0.500 sec")
                                time.sleep(0.500)
                                self.parent.event(source, comp, name, value, code, text)
                                if not self.currentSubTaskState and name.split('_')[1] in self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.parent.deviceUuid]['SubTask']:
                                    self.currentSubTaskState = value.lower()

                        elif self.subTask:

                            for k,v in self.parent.master_tasks[self.parent.master_uuid]['coordinator'][self.parent.master_tasks[self.parent.master_uuid]['coordinator'].keys()[0]]['SubTask'].iteritems():

                                if v and name.split('_')[-1] in v[3] and v[0] in self.subTask:
                                    try:
                                        self.subTask[v[0]].superstate.event(source, comp, name, value, code, text)
                                        self.currentSubTaskState = value.lower()
                                    except Exception as e:
					print (e)
					print ("Retrying in 0.5 sec")
                                        time.sleep(0.5)
                                        self.parent.event(source, comp, name, value, code, text)
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

		except Exception as e:
                    print ("Error processing collaborator event:")
                    print (e)
                    print ("Retrying in 1 sec")
                    time.sleep(1)

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

