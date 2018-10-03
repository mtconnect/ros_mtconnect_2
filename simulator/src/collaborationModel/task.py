import os, sys
sys.path.insert(0,os.getcwd()+'\\utils')

from archetypeToInstance import archetypeToInstance

from archetypeToInstance import update as assetUpdate

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, copy
import subTask
import xml.etree.ElementTree as ET


class interface(object):

    def __init__(self, value = None):
        self.value = value

class task(object):

    def __init__(self, parent, interface, master_task_uuid, coordinator):

        class statemachineModel(object):

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

            def INACTIVE(self):
                self.subTask = {}
                self.currentSubTask = str()

                if not self.initialize:
                    #execute only once
                    arch2ins = archetypeToInstance(self.parent.coordinator_task, self.master_task_uuid, self.parent.deviceUuid)
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

                #time limit to commit
                def commit_timer():
                    timer = Timer(self.commit_time_limit,self.void)
                    timer.start()
                    while timer.isAlive():
                        collaborators_commit = False
                        for key, value in self.parent.master_tasks[self.master_task_uuid]['collaborators'].iteritems():

                            if value['state'][2] == 'COMMITTED':
                                collaborators_commit = True
                            else:
                                collaborators_commit = False
                                break

                        if collaborators_commit == True:
                            break

                    if collaborators_commit == True:
                        self.parent.master_tasks[self.parent.master_uuid]['coordinator'][self.parent.deviceUuid]['Task'][1] = 'COMMITTED'
                        try:
                            self.all_commit()
                        except Exception as e:
                            print ("Error while committing:",e)
                            print ("Retrying in 0.2 s.")
                            time.sleep(0.2)
                            self.all_commit()
                    else:
                        self.no_commit()

                t = Thread(target = commit_timer)
                t.start()

            def COMMITTED(self):

                self.parent.adapter.begin_gather()
                self.interface.set_value("COMMITTED")
                self.parent.adapter.complete_gather()

		self.parent.priority.commit_check()

                self.taskIns = assetUpdate(self.taskIns, "State", "COMMITTED")
                self.parent.adapter.addAsset('Task', self.master_task_uuid, self.taskIns)

                #creates subtask objects wrt the parent device
                for key, value in self.parent.master_tasks[self.master_task_uuid]['coordinator'][self.coordinator.coordinator_name]['SubTask'].iteritems():
                    if key == self.coordinator.coordinator_name:
                        self.subTask[value[0]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.master_task_uuid, collaborators = value[2], taskName = self.coordinator.task_name)
                        self.subTask[value[0]].create_statemachine()
                        self.subTask[value[0]].superstate.create()
                        self.currentSubTask = copy.deepcopy(value[0])

                        for key, val in self.parent.master_tasks[self.master_task_uuid]['collaborators'].iteritems():

                            if self.coordinator.task_name in val['SubTask']:
                                for i,x in enumerate(val['SubTask'][self.coordinator.task_name]):

                                    if x and x[4] and self.parent.deviceUuid in x[4]:

                                        self.subTask[x[1]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.subTask[value[0]].superstate.task_uuid, collaborators = x[4],taskName = x[1])
                                        self.subTask[x[1]].create_statemachine()
                                        self.subTask[x[1]].superstate.create()
                                        self.currentSubTask = copy.deepcopy(x[1])

                                        while self.subTask[self.currentSubTask].superstate.state != 'removed':
                                            pass
                                        self.parent.master_tasks[self.master_task_uuid]['collaborators'][key]['SubTask'][self.coordinator.task_name][i][2] = 'COMPLETE'

                                        self.currentSubTask = copy.deepcopy(value[0])

                        while self.subTask[self.currentSubTask].superstate.state != 'removed':
                            pass
                        self.parent.master_tasks[self.master_task_uuid]['coordinator'][self.coordinator.coordinator_name]['SubTask'][self.coordinator.coordinator_name][1] = 'COMPLETE'

            def commit(self):
                #check if all the tasks have been completed
                success = True
                for key, value in self.parent.master_tasks[self.master_task_uuid]['coordinator'][self.coordinator.coordinator_name]['SubTask'].iteritems():
                    if value:
                        if value[1] == 'COMPLETE' or value[1] == 'FAIL' or value[1] == '':
                            if key == self.parent.deviceUuid or self.parent.master_tasks[self.master_task_uuid]['collaborators'][key]['state'][2] == 'INACTIVE':
                                success = True
                            else:
                                success = False
                                break
                        else:
                            success = False
                            break

                if success == True:
                    self.success()

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
                    self.subTask[self.currentSubTask].superstate.event(source, comp, name, value, code, text)
                elif 'SubTask' in name:
                    self.coordinator.event(source, comp, name, value, code, text)

            def void(self):
                pass

        self.superstate = statemachineModel(parent = parent, interface = interface, master_task_uuid = master_task_uuid, coordinator = coordinator)

    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['inactive', 'preparing', 'committing', 'committed', 'complete', 'fail']}, 'removed']

        transitions = [['create', 'base', 'base:inactive'],
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

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)

        self.statemachine.on_enter('base:inactive', 'INACTIVE')
        self.statemachine.on_enter('base:preparing', 'PREPARING')
        self.statemachine.on_enter('base:committing', 'COMMITTING')
        self.statemachine.on_enter('base:committed', 'COMMITTED')
        self.statemachine.on_enter('base:complete', 'COMPLETE')
        self.statemachine.on_enter('base:fail', 'FAIL')
