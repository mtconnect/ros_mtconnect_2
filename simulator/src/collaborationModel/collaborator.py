from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import os, sys

from .archetypeToInstance import archetypeToInstance
from .archetypeToInstance import update as assetUpdate
from .subTask import subTask

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, copy, uuid, re, datetime
import xml.etree.ElementTree as ET

class interface:

    def __init__(self, value = None):
        self.value = value

class collaborator:

    class StateMachineModel:

        def __init__(self, parent, interface, collaborator_name):

            self.interface = interface
            self.parent = parent
            self.collaborator_name = collaborator_name
            self.task_name = None
            self.commit_time_limit = 120.0
            self.subTask = {}
            self.currentSubTask = str()
            self.initialized = False
            self.currentSubTaskState = None
            self.currentSubTaskList = None
            self.commit_timer =Timer(0,self.void)
            self.interface_completion =False

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

            self.timer_commit()

        def timer_commit(self):
            if not self.commit_time_limit:
                return self.no_commit()

            self.commit_timer = Timer(self.commit_time_limit,self.no_commit)
            self.commit_timer.daemon = True
            self.commit_timer.start()

        def committed(self):
            #creates 'self.subtasks' list for device specific tasks
            self.subTask_collab = False
            collaborator = False
            self.ordered_tasks = []
            self.subTasks = []
            self.task_coordinator = list(self.parent.master_tasks[self.parent.master_uuid]['coordinator'].keys())[0]

            master_task = self.parent.master_tasks[self.parent.master_uuid]

            for key,val in master_task['coordinator'][self.task_coordinator]['SubTask'].items():
                if val:
                    self.ordered_tasks.append([val[4],key,val])

            self.ordered_tasks.sort()

            for i,z in enumerate(self.ordered_tasks):
                key = z[1]
                val = z[2]

                if val:
                    if self.collaborator_name in val[2]:
                        #val[2] is the list of collaborators involved in the lowest level task coordinated by key
                        self.subTask_collab = True

                    elif self.collaborator_name == key:
                        collaborator = True

                    elif not collaborator and key == self.task_coordinator:
                        continue

                    elif collaborator and key!= self.collaborator_name:
                        continue

                    if (collaborator and key == self.collaborator_name) or self.subTask_collab:

                        #top level tasks
                        self.subTask[val[0]] = subTask(
                            parent = self.parent,
                            interface = interface,
                            master_task_uuid = self.parent.master_uuid,
                            collaborators = None,
                            taskName = val[0]
                            )

                        self.subTask[val[0]].superstate.create()

                        if val[2]:
                            if val[0] in master_task['collaborators'][self.collaborator_name]['SubTask']:
                                self.subtask_collaborator_name = self.collaborator_name
                            elif val[2]:
                                self.subtask_collaborator_name = val[2]
                            else:
                                self.subtask_collaborator_name = None
                                continue

                            currentSubTaskList = master_task['collaborators'][self.subtask_collaborator_name]['SubTask'][val[0]]

                            for i,x in enumerate(currentSubTaskList):

                                if x[4]:

                                    #low level tasks; interfaces only
                                    self.subTask[x[1]] = subTask(
                                        parent = self.parent,
                                        interface = interface,
                                        master_task_uuid = self.subTask[val[0]].superstate.task_uuid,
                                        collaborators = None,
                                        taskName =x[1]
                                        )

                                    self.subTask[x[1]].superstate.create()

                            self.subTasks.append([val[0],val[3],currentSubTaskList,key])

            if self.subTasks:
                self.currentSubTask = self.subTasks[0][0]
                self.currentSubTaskType = self.subTasks[0][1]
                self.currentSubTaskList = self.subTasks[0][2]
                self.collaborator_key = self.subTasks[0][3]
                self.activate = False


        def current_subtask_check(self, source = None, comp= None, name =None, value=None, code = None, text = None):
            #event handling once committed

            if (value == "ACTIVE"
                and self.currentSubTaskState == "ACTIVE"
                and name.split('_')[-1] in str(self.ordered_tasks)
                ):

                #if an untimely event received; send for retry
                if self.currentSubTaskType not in name:
                    return "RETRY"
                else:
                    pass

            if self.currentSubTaskState != 'ACTIVE' or self.currentSubTaskType in name:
                #top level task management; activation and completion

                if self.currentSubTask in self.subTask and self.currentSubTaskType in name:
                    self.subTask[self.currentSubTask].superstate.event(source, comp, name, value, code, text)

                    if value.lower() == 'active':
                        self.currentSubTaskState = value

                    elif (self.currentSubTaskList
                          and (self.currentSubTaskList[-1][2] == 'COMPLETE'
                               or not self.currentSubTaskList[-1][4])
                          and not self.subTask_collab
                          ):

                        if (self.subTask[self.currentSubTask].superstate.state == 'removed'
                            and code in self.parent.master_tasks
                            ):

                            self.parent.master_tasks[code]['coordinator'][self.task_coordinator]['SubTask'][self.collaborator_key][1] = 'COMPLETE'

                            if self.currentSubTask == self.ordered_tasks[-1][2][0]:
                                self.completed()


            if self.currentSubTaskList and self.currentSubTaskState == "ACTIVE" and not self.activate:
                #low level task management

                for i,x in enumerate(self.currentSubTaskList):

                    if x[4] and self.currentSubTaskList[i][2] == 'READY':

                        if x[1] == name.split('_')[-1]:
                            self.subTask[x[1]].superstate.event(source, comp, name, value, code, text)

                        if self.subTask[x[1]].superstate.state == "removed" and code in self.parent.master_tasks:
                            self.parent.master_tasks[code]['collaborators'][self.subtask_collaborator_name]['SubTask'][self.currentSubTask][i][2] = 'COMPLETE'
                            self.currentSubTaskList[i][2] = 'COMPLETE'

                        else:
                            break

                    elif (not x[4]
                          and self.collaborator_name == self.subtask_collaborator_name
                          and not self.currentSubTaskList[i][2]
                          ):
                        #internal tasks management; activation and completion

                        self.activate = True
                        self.currentSubTaskList[i][2] = 'COMPLETE'
                        self.parent.event(self.collaborator_name, 'internal_event',x[1],'ACTIVATE',None,self.collaborator_key)
                        self.parent.master_tasks[code]['collaborators'][self.collaborator_name]['SubTask'][self.currentSubTask][i][2] = 'COMPLETE'
                        self.activate = False

                    elif x[4] and not self.currentSubTaskList[i][2] and self.subTask_collab:
                        self.currentSubTaskList[i][2] = 'READY'
                        self.parent.event(self.collaborator_name,'interface_initialization','SubTask_'+x[1],'IDLE')
                        break

                    elif x[4] and not self.currentSubTaskList[i][2] and not self.subTask_collab:
                        self.currentSubTaskList[i][2] = 'READY'
                        break


            if self.subTask_collab and self.currentSubTaskList[-1][2] == 'COMPLETE' and not self.interface_completion:
                #top level task completion for low level task collaborators

                self.subTask[self.currentSubTask].superstate.success()
                self.interface_completion =True
                self.parent.event(self.collaborator_name, 'interface_completion',self.currentSubTaskType,'COMPLETE')
                self.interface_completion =False
                self.parent.master_tasks[self.parent.master_uuid]['coordinator'][self.task_coordinator]['SubTask'][self.collaborator_key][1] = 'COMPLETE'

                if self.subTasks[-1][0] == self.currentSubTask:
                    self.parent.master_tasks[self.parent.master_uuid]['collaborators'][self.collaborator_name]['state'][2] = 'COMPLETE'
                    self.completed()
                    self.parent.IDLE()

                else:
                    next = False
                    for subTask in self.subTasks:
                        if subTask[0] == self.currentSubTask:
                            next = True
                        elif next:
                            self.currentSubTask = subTask[0]
                            self.currentSubTaskType = subTask[1]
                            self.currentSubTaskList = subTask[2]
                            self.collaborator_key = subTask[3]
                            self.currentSubTaskState = None
                            self.parent.IDLE()
                            break

        def void():
            pass

        def event(self, source, comp, name, value, code = None, text = None):

            if True:
                #collaboration binding related event handling
                if (comp == 'Coordinator'
                    and name == 'binding_state'
                    and value.lower() == 'inactive'
                    and self.interface.value().lower() == 'committed'
                    ):

                    if self.currentSubTask:
                        self.subTask[self.currentSubTask].superstate.success()

                    self.completed()
                    self.parent.COMPLETED()

                elif comp == 'Coordinator' and name == 'binding_state' and value.lower() == 'preparing':
                    self.parent.master_tasks[code[0]] = code[1]
                    self.task_created()

                elif comp == 'Coordinator' and  name == 'binding_state':
                    if value.lower() == 'committing':
                        self.commit()

                    elif value.lower() == 'committed' and text in self.parent.master_tasks[code]['coordinator']:
                        self.parent.master_tasks[code]['coordinator'][text]['state'][2] = value

                        if self.commit_timer.isAlive():
                            self.commit_timer.cancel()

                        self.committed()

                #interfaces related event handling
                elif 'SubTask' in name and self.interface.value().lower() == 'committed':

                    if self.currentSubTask:
                        try:
                            if name.split('_')[-1] in str(self.subTasks):
                                activate = self.current_subtask_check(source, comp, name, value, code, text)
                                if activate == "RETRY":
                                    print ("Retrying task activation in 0.5 sec")
                                    time.sleep(0.5)
                                    self.parent.event(source, comp, name, value, code, text)

                        except Exception as e:
                            print (e)
                            print ("Retrying in 0.500 sec")
                            print (source, comp, name, value, code, text)
                            time.sleep(0.500)
                            self.parent.event(source, comp, name, value, code, text)

                    elif self.subTask:

                        for k,v in self.parent.master_tasks[self.parent.master_uuid]['coordinator'][self.task_coordinator]['SubTask'].items():

                            if v and name.split('_')[-1] in v[3] and v[0] in self.subTask:
                                try:
                                    self.current_subtask_check(source, comp, name, value, code, text)

                                except Exception as e:
                                    print (e)
                                    print ("Retrying in 0.5 sec")
                                    time.sleep(0.5)
                                    self.parent.event(source, comp, name, value, code, text)
                                    self.currentSubTaskState = value.lower()

                            elif v and name.split('_')[-1] in v[3] and v[0] not in self.subTask:
                                self.parent.event(source, comp, name.split('_')[-1], value, code, text)

                            else:
                                collab = None
                                if k == self.collaborator_name:
                                    if v: collab = v[2]

                                if collab and self.parent.master_tasks[self.parent.master_uuid]['collaborators'][collab]['SubTask'][self.task_name]:

                                    for t in self.parent.master_tasks[self.parent.master_uuid]['collaborators'][collab]['SubTask'][self.task_name]:
                                        if name.split('_')[-1] == t[1]:
                                            self.parent.event(source, comp, name.split('_')[-1], value, code, text)
                                            break

            if False: #except Exception as e:
                print ("Error processing collaborator event:")
                print (e)
                print ("Retrying in 1 sec")
                time.sleep(1)



    def __init__(self, parent, interface, collaborator_name):
        self.superstate = collaborator.StateMachineModel(parent, interface, collaborator_name)
        self.statemachine = self.create_statemachine(self.superstate)

    def create_statemachine(self, state_machine_model):
        NestedState.separator = ':'

        states = [
            {
                'name':'base',
                'children':[
                    'inactive',
                    'preparing',
                    'committed'
                ]
            }
        ]

        transitions = [
            ['unavailable', 'base', 'base:inactive'],

            ['task_created', 'base:inactive', 'base:preparing'],

            ['commit', 'base:preparing', 'base:committed'],
            ['no_commit', 'base:committed', 'base:preparing'],

            ['completed', 'base:committed', 'base:inactive'],
            ['failed', 'base:committed', 'base:inactive'],

            ['default', 'base:inactive', 'base:inactive'],
            ['default', 'base:committed', 'base:committed'],
            ['default', 'base', 'base']
        ]

        statemachine = Machine(
            model = state_machine_model,
            states = states,
            transitions = transitions,
            initial = 'base',
            ignore_invalid_triggers=True
            )

        statemachine.on_enter('base:inactive', 'INACTIVE')
        statemachine.on_enter('base:preparing', 'PREPARING')
        statemachine.on_enter('base:committed', 'COMMITTED')

        return statemachine
