from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time

import task

#will be included under assets!?
class interface(object):

    def __init__(self, value = None):
        self.value = value

class coordinator(object):

    def __init__(self, parent, interface, master_task_uuid, coordinator_name): 

        class statemachineModel(object):

            def __init__(self, parent, interface, master_task_uuid, coordinator_name):

                self.interface = interface
                self.parent = parent
                self.master_task_uuid = master_task_uuid
                self.task_name = None
                self.events =[]
                self.coordinator_name = coordinator_name
                self.id = coordinator_name #which one?
                self.initialize = True

            def INACTIVE(self):
                self.parent.adapter.begin_gather()
                self.interface.set_value("INACTIVE")
                self.parent.adapter.complete_gather()
                print "HEREEEEEEEEE"
                
                if self.initialize:
                    
                    self.task = task.task(parent = self.parent, interface = interface, master_task_uuid = self.master_task_uuid, coordinator = self)
                    self.task.create_statemachine()
                    self.task.superstate.create()

                    self.bind_to_task()

            def COMMITTED(self):
                self.intialize = False
                #self.parent.adapter.begin_gather()
                #self.interface.set_value("COMMITTED")
                #self.parent.adapter.complete_gather()

                
            def event(self, source, comp, name, value, code = None, text = None):
                #samplevent('robot_r1','collaborator','subtask_address',["open_door",'COMMITTED'],execution_lines, 'execute')
                
                if comp == 'Task_Collaborator':
                    if 'binding_state' in name:
                        self.parent.master_tasks[code]['collaborators'][text]['state'][2] = value
                        if value.lower() == 'preparing':
                            self.task.superstate.prepare()

                elif 'SubTask' in name:
                    def commit_task():
                        if value.lower() == 'fail' or value.lower() == 'complete':
                            for key,val in self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'].iteritems():
                                if val and key == text and name.split('_')[-1] == val[3]:
                                    self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][key][1] = value
                                    self.task.superstate.commit()
                                    

                    t = Thread(target = commit_task)
                    t.start()
                    
                    coordinator_task = None
                    
                    if self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][self.coordinator_name][3] in name and text in self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][self.coordinator_name][2]:
                        coordinator_task = True
                        
                    else: #done only for one collaborator at a time
                        
                        collab = self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][self.coordinator_name][2]
                        taskType = self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][self.coordinator_name][3]

                        if text in collab:
                            try:
                                for x in self.parent.master_tasks[code]['collaborators'][collab]['SubTask'][taskType]:
                                    try:
                                        if x[1] in name:
                                            coordinator_task = True
                                            break
                                        else:
                                            coordinator_task = False
                                    except:
                                        coordinator_task = False
                            except:
                                coordinator_task = False
                            
                    if coordinator_task:
                        self.task.superstate.event(source, comp, name, value, code, text)
                else:
                    self.parent.event(source, comp, name, value, code, text)
             
        self.superstate = statemachineModel(parent = parent, interface = interface, master_task_uuid = master_task_uuid, coordinator_name =coordinator_name)

    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['inactive', 'committed']}]

        transitions = [['unavailable', 'base', 'base:inactive'],

                       ['bind_to_task', 'base:inactive', 'base:committed'],

                       ['completed', 'base:committed', 'base:inactive'],
                       ['failed', 'base:committed', 'base:inactive'],

                       ['default', 'base:inactive', 'base:inactive'],
                       ['default', 'base:committed', 'base:committed']
                       ]

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)
                       
        self.statemachine.on_enter('base:inactive', 'INACTIVE')
        self.statemachine.on_enter('base:committed', 'COMMITTED')

