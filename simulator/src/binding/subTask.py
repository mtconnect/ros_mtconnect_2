from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time

#will be included under assets!?
class interface(object):

    def __init__(self, value = None):
        self.value = value

class subTask(object):

    def __init__(self, interface, parent, master_task_uuid, collaborators):

        class statemachineModel(object):

            def __init__(self, interface, parent, master_task_uuid, collaborators):

                self.interface = interface
                self.parent = parent
                self.master_task_uuid = master_task_uuid
                self.collaborators = collaborators
                self.commit_time_limit = 2.0

            def INACTIVE(self):
                self.interface.value = 'INACTIVE'
                self.activated()
                self.quorum()

            def COMMITTING(self):
                self.interface.value = 'COMMITTING'
                self.all_commit()

            def COMMITTED(self):
                self.interface.value = 'COMMITTED'
                #self.check_for_subTasks()

            def event(self, source, comp, name, value, code = None , text = None):
                if comp == 'SubTask:Collaborator' and value.lower() == 'completed':
                    self.success()
                elif comp == 'SubTask:Collaborator' and value.lower() == 'failed':
                    self.failure()
                else:
                    self.parent.event(source, comp, name, value, code = None , text = None)
                

            def COMPLETE(self):
                self.interface.value = 'COMPLETE'
                self.default()

            def FAIL(self):
                self.interface.value = 'FAIL'               
                self.default()

            def void(self):
                pass

        self.superstate = statemachineModel(parent =parent, master_task_uuid= master_task_uuid, collaborators=collaborators, interface = interface)

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
