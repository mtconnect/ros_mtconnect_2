from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import os, sys

from collaborationModel.archetypeToInstance import archetypeToInstance

from collaborationModel.archetypeToInstance import update as assetUpdate

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, copy
import collaborationModel.subTask as subTask
import xml.etree.ElementTree as ET


class interface:

    def __init__(self, value = None):
        self.value = value

class task:

    class StateMachineModel:

        def __init__(self, parent, interface, master_task_uuid, coordinator):

            self.interface = interface
            self.parent = parent
            self.commit_time_limit = 120.0
            self.master_task_uuid = master_task_uuid
            self.coordinator = coordinator
            self.subTask = {}
            self.currentSubTask = str()
            self.taskIns = "xml/text"
            self.arch2ins = {}
            self.initialize = False
            self.subTasks = []
            self.commit_timer = Timer(0,self.void)
            self.currentSubTaskState = None

        def INACTIVE(self):
            self.subTask = {}
            self.currentSubTask = str()

            if not self.initialize:
                #execute only once
                arch2ins = archetypeToInstance(self.parent.coordinator_task, self.master_task_uuid, self.coordinator.coordinator_name)
                self.arch2ins = arch2ins
                self.parent.master_tasks[self.master_task_uuid] = arch2ins.jsonInstance()
                self.taskIns = arch2ins.taskIns
                self.parent.adapter.addAsset('Task', self.master_task_uuid, arch2ins.taskIns)
                self.initialize = True
                self.activated()

        def PREPARING(self):
            self.parent.adapter.begin_gather()
            self.interface.set_value("PREPARING")
            self.parent.adapter.complete_gather()

            self.taskIns = assetUpdate(self.taskIns, "State", "PREPARING")
            self.parent.adapter.addAsset('Task', self.master_task_uuid, self.taskIns)

        def prepare(self):
            quorum = True

            for key, value in self.parent.master_tasks[self.master_task_uuid]['collaborators'].iteritems():
                if value['state'][2] == 'PREPARING':
                    quorum = True
                else:
                    quorum = False
                    break

            if quorum == True:
                self.quorum()


        def COMMITTING(self):

            self.parent.adapter.begin_gather()
            self.interface.set_value("COMMITTING")
            self.parent.adapter.complete_gather()

            self.taskIns = assetUpdate(self.taskIns, "State", "COMMITTING")
            self.parent.adapter.addAsset('Task', self.master_task_uuid, self.taskIns)

            self.timer_commit()

        #time limit to commit
        def timer_commit(self):
            if not self.commit_time_limit:
                self.no_commit()
            self.commit_timer = Timer(self.commit_time_limit,self.no_commit)
            self.commit_timer.daemon = True
            self.commit_timer.start()


        def commit_check(self):
            if self.commit_timer.isAlive():
                collaborators_commit = False

                for key, value in self.parent.master_tasks[self.master_task_uuid]['collaborators'].iteritems():

                    if value['state'][2] == 'COMMITTED':
                        collaborators_commit = True
                    else:
                        collaborators_commit = False
                        break

                if collaborators_commit == True:
                    self.commit_timer.cancel()

                    self.parent.master_tasks[self.master_task_uuid]['coordinator'][self.coordinator.coordinator_name]['Task'][1] = 'COMMITTED'
                    try:
                        self.all_commit()

                    except Exception as e:
                        print ("Error while committing:",e)
                        print ("Retrying in 0.2 s.")
                        time.sleep(0.2)
                        self.all_commit()


        def COMMITTED(self):

            self.parent.adapter.begin_gather()
            self.interface.set_value("COMMITTED")
            self.parent.adapter.complete_gather()

            self.parent.priority.commit_check()

            self.taskIns = assetUpdate(self.taskIns, "State", "COMMITTED")
            self.parent.adapter.addAsset('Task', self.master_task_uuid, self.taskIns)

            self.committed()

        def committed(self):

            #creates subtask objects wrt the parent device
            for key, value in self.parent.master_tasks[self.master_task_uuid]['coordinator'][self.coordinator.coordinator_name]['SubTask'].iteritems():
                if key == self.coordinator.coordinator_name:

                    self.subTask[value[0]] = subTask.subTask(
                        parent = self.parent,
                        interface = interface,
                        master_task_uuid = self.master_task_uuid,
                        collaborators = value[2],
                        taskName = self.coordinator.task_name
                        )

                    self.subTask[value[0]].superstate.create()

                    for key, val in self.parent.master_tasks[self.master_task_uuid]['collaborators'].iteritems():

                        if self.coordinator.task_name in val['SubTask']:

                            currentSubTaskList = val['SubTask'][self.coordinator.task_name]

                            for i,x in enumerate(currentSubTaskList):

                                if x and x[4] and self.coordinator.coordinator_name in x[4]:

                                    self.subTask[x[1]] = subTask.subTask(
                                        parent = self.parent,
                                        interface = interface,
                                        master_task_uuid = self.subTask[value[0]].superstate.task_uuid,
                                        collaborators = x[4],
                                        taskName = x[1]
                                        )

                                    self.subTask[x[1]].superstate.create()


                            self.subTasks.append([value[0],value[3],currentSubTaskList,key])


            if self.subTasks:
                self.currentSubTask = self.subTasks[0][0]
                self.currentSubTaskType = self.subTasks[0][1]
                self.currentSubTaskList = self.subTasks[0][2]
                self.collaborator_key = self.subTasks[0][3]


        def current_subtask_check(self, source, comp, name, value, code = None, text = None):

            if self.currentSubTaskState != 'ACTIVE' or self.currentSubTaskType in name:

                if self.currentSubTask in self.subTask:
                    self.subTask[self.currentSubTask].superstate.event(source, comp, name, value, code, text)

                    if value.lower() == 'active':
                        self.currentSubTaskState = value

                    elif self.currentSubTaskList and self.currentSubTaskList[-1][2] == 'COMPLETE':

                        if self.subTask[self.currentSubTask].superstate.state == 'removed' and code in self.parent.master_tasks:
                            self.parent.master_tasks[code]['coordinator'][self.coordinator.coordinator_name]['SubTask'][self.coordinator.coordinator_name][1] = 'COMPLETE'



            if self.currentSubTaskList and self.currentSubTaskState == "ACTIVE":

                for i,x in enumerate(self.currentSubTaskList):

                    if x[4] and self.currentSubTaskList[i][2] == 'READY':
                        self.subTask[x[1]].superstate.event(source, comp, name, value, code, text)

                        if self.subTask[x[1]].superstate.state == "removed" and code in self.parent.master_tasks:
                            self.parent.master_tasks[code]['collaborators'][self.collaborator_key]['SubTask'][self.currentSubTask][i][2] = 'COMPLETE'
                            self.currentSubTaskList[i][2] = 'COMPLETE'

                        else:
                            break

                    elif not x[4] and not self.currentSubTaskList[i][2]:
                        self.currentSubTaskList[i][2] = 'COMPLETE'

                    elif x[4] and not self.currentSubTaskList[i][2]:
                        self.currentSubTaskList[i][2] = 'READY'
                        break


        def commit(self):
            #check if all the tasks have been completed
            success = True
            for key, value in self.parent.master_tasks[self.master_task_uuid]['coordinator'][self.coordinator.coordinator_name]['SubTask'].iteritems():
                if value:
                    if value[1] == 'COMPLETE' or value[1] == 'FAIL' or value[1] == '':

                        if key == self.coordinator.coordinator_name or self.parent.master_tasks[self.master_task_uuid]['collaborators'][key]['state'][2] == 'INACTIVE':
                            success = True
                        else:
                            success = False
                            break
                    else:
                        success = False
                        break

            if success == True:
                self.success()
                self.parent.COMPLETED()


        def COMPLETE(self):
            #update states and remove task asset once completed
            self.taskIns = assetUpdate(self.taskIns, "State", "INACTIVE")
            self.parent.adapter.addAsset('Task', self.master_task_uuid, self.taskIns)

            self.parent.adapter.begin_gather()
            self.interface.set_value("INACTIVE")
            self.parent.adapter.complete_gather()

            self.parent.adapter.removeAsset(self.master_task_uuid)

            self.default()


        def FAIL(self):
            self.taskIns = assetUpdate(self.taskIns, "State", "INACTIVE")
            self.parent.adapter.addAsset('Task', self.master_task_uuid, self.taskIns)
            self.parent.adapter.removeAsset(self.master_task_uuid)

            self.parent.adapter.begin_gather()
            self.interface.set_value("INACTIVE")
            self.parent.adapter.complete_gather()

            self.default()


        def event(self, source, comp, name, value, code = None, text = None):

            if self.subTask and self.currentSubTask:
                if name.split('_')[-1] in str(self.subTasks):
                    self.current_subtask_check(source, comp, name, value, code, text)

            elif 'SubTask' in name:
                """Retry event"""
                time.sleep(0.5)
                self.coordinator.event(source, comp, name, value, code, text)

        def void(self):
            pass


    def __init__(self, parent, interface, master_task_uuid, coordinator):
        self.superstate = task.StateMachineModel(parent, interface, master_task_uuid, coordinator)
        self.statemachine = self.create_statemachine(self.superstate)
        
    def create_statemachine(self, state_machine_model):
        NestedState.separator = ':'

        states = [
            {
                'name':'base',
                'children':[
                    'inactive',
                    'preparing',
                    'committing',
                    'committed',
                    'complete',
                    'fail'
                ]
            },
            'removed'
        ]

        transitions = [
            ['create', 'base', 'base:inactive'],
            ['unavailable', 'base', 'base:inactive'],

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
        statemachine.on_enter('base:committing', 'COMMITTING')
        statemachine.on_enter('base:committed', 'COMMITTED')
        statemachine.on_enter('base:complete', 'COMPLETE')
        statemachine.on_enter('base:fail', 'FAIL')

        return statemachine
