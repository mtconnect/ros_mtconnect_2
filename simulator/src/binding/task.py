from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, copy
import subTask

#will be included under assets!?

class interface(object):

    def __init__(self, value = None):
        self.value = value

class task(object):

    def __init__(self, parent, interface, master_task_uuid, coordinator):

        class statemachineModel(object):

            def __init__(self, parent, interface, master_task_uuid, coordinator):

                self.interface = interface
                self.parent = parent
                self.commit_time_limit = 2.0
                self.master_task_uuid = master_task_uuid
                self.coordinator = coordinator
                self.subTask = {}
                self.currentSubTask = str()

            def INACTIVE(self):
                self.activated()

            def PREPARING(self):
                self.parent.adapter.begin_gather()
                self.interface.set_value("PREPARING")
                self.parent.adapter.complete_gather()

            def prepare(self):
                quorum = True
                #remove while loops or add events to test
                if True:
                    for key, value in self.parent.master_tasks[self.master_task_uuid]['collaborators'].iteritems():
                            
                        if value['state'][2] == 'PREPARING':
                            quorum = True
                        else:
                            quorum = False
                            break
                        
                    if quorum == True:
                        self.quorum()
                                         
                    #self.quorum()
                    
                #t = Thread(target = prepare)
                #t.start()

            def COMMITTING(self):

                self.parent.adapter.begin_gather()
                self.interface.set_value("COMMITTING")
                self.parent.adapter.complete_gather()
                
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
                        self.all_commit()
                    else:
                        self.no_commit()
                    
                t = Thread(target = commit_timer)
                t.start()

            def COMMITTED(self):
                
                self.parent.adapter.begin_gather()
                self.interface.set_value("COMMITTED")
                self.parent.adapter.complete_gather()
            
                
                for key, value in self.parent.master_tasks[self.master_task_uuid]['coordinator'][self.coordinator.coordinator_name]['SubTask'].iteritems():
                    if key == self.coordinator.coordinator_name:
                        self.subTask[value[0]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.master_task_uuid, collaborators = value[2])
                        self.subTask[value[0]].create_statemachine()
                        self.subTask[value[0]].superstate.create()
                        self.currentSubTask = copy.deepcopy(value[0])

                        for key, val in self.parent.master_tasks[self.master_task_uuid]['collaborators'].iteritems():

                            if self.coordinator.task_name in val['SubTask']:
                                for i,x in enumerate(val['SubTask'][self.coordinator.task_name]):
                                    
                                    self.subTask[x[0]] = subTask.subTask(parent = self.parent , interface = interface, master_task_uuid = self.master_task_uuid, collaborators = x[4])
                                    self.subTask[x[0]].create_statemachine()
                                    self.subTask[x[0]].superstate.create()
                                    self.currentSubTask = copy.deepcopy(x[0])

                                    while self.subTask[self.currentSubTask].superstate.state != 'removed':
                                        pass
                                    self.parent.master_tasks[self.master_task_uuid]['collaborators'][key]['SubTask'][self.coordinator.task_name][i][2] = 'COMPLETE'
                                        
                        self.currentSubTask = copy.deepcopy(value[0])
                        
                        while self.subTask[self.currentSubTask].superstate.state != 'removed':
                            pass
                        self.parent.master_tasks[self.master_task_uuid]['coordinator'][self.coordinator.coordinator_name]['SubTask'][self.coordinator.coordinator_name][1] = 'COMPLETE'
                        
            def commit(self):
                if True: #replacing while loop
                    success = True
                    for key, value in self.parent.master_tasks[self.master_task_uuid]['coordinator'][self.coordinator.coordinator_name]['SubTask'].iteritems():
                        if value:
                            if value[1] == 'COMPLETE' or value[1] == 'FAIL' or value[1] == '':
                                success = True
                            else:
                                success = False
                                break
                    if success == True:
                        self.success()

            def COMPLETE(self):
                time.sleep(0.2)
                
                self.parent.adapter.begin_gather()
                self.interface.set_value("INACTIVE")
                self.parent.adapter.complete_gather()

                self.default()

            def FAIL(self):
                time.sleep(0.2)

                self.parent.adapter.begin_gather()
                self.interface.set_value("INACTIVE")
                self.parent.adapter.complete_gather()
                
                self.default()

            def event(self, source, comp, name, value, code = None, text = None):
                self.subTask[self.currentSubTask].superstate.event(source, comp, name, value, code, text)
                
            def void(self):
                pass

        self.superstate = statemachineModel(parent = parent, interface = interface, master_task_uuid = master_task_uuid, coordinator = coordinator)

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
        self.statemachine.on_enter('base:preparing', 'PREPARING')
        self.statemachine.on_enter('base:committing', 'COMMITTING')
        self.statemachine.on_enter('base:committed', 'COMMITTED')
        self.statemachine.on_enter('base:complete', 'COMPLETE')
        self.statemachine.on_enter('base:fail', 'FAIL')
