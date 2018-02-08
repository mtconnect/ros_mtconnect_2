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

            def INACTIVE(self):
                self.interface.value = 'INACTIVE'
                self.parent.binding_state = 'INACTIVE'
                if self.parent.master_tasks[self.master_task_uuid]['coordinator'][self.coordinator_name]['Task']:
                    
                    self.task = task.task(parent = self.parent, interface = interface, master_task_uuid = self.master_task_uuid, coordinator = self)
                    self.task.create_statemachine()
                    self.task.superstate.create()

                    self.bind_to_task()

            def COMMITTED(self):
                self.interface.value = 'COMMITTED'

                
            def event(self, source, comp, name, value, code = None, text = None):
                #samplevent('robot_r1','collaborator','subtask_address',["open_door",'COMMITTED'],execution_lines, 'execute')
                
                if comp == 'Task:Collaborator':
                    if name == 'state':
                        self.parent.master_tasks[code]['collaborators'][text]['state'][2] = value
                        if value.lower() == 'preparing':
                            self.task.superstate.prepare()

                elif comp == 'SubTask:Collaborator':
                    if name == 'state':
                        self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][text][1] = value
                        if value.lower() == 'fail' or value.lower() == 'complete':
                            self.task.superstate.commit()

                else:
                    self.parent.event(source, comp, name, value, code = None, text = None)
             
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

