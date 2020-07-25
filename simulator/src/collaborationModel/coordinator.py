from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time
from .task import task

"""Coordinator class for tasks"""

class coordinator:

    class StateMachineModel:

        def __init__(self, parent, interface, master_task_uuid, coordinator_name):

            self.interface = interface
            self.parent = parent
            self.master_task_uuid = master_task_uuid
            self.task_name = None
            self.events =[]
            self.coordinator_name = coordinator_name
            self.initialize = True

        def INACTIVE(self):
            self.parent.adapter.begin_gather()
            self.interface.set_value("INACTIVE")
            self.parent.adapter.complete_gather()

            if self.initialize:
                #create task only once initially when the coordinator is defined
                self.task = task(
                    parent = self.parent,
                    interface = self.interface,
                    master_task_uuid = self.master_task_uuid,
                    coordinator = self
                    )

                self.task.superstate.create()

                self.initialize = False

                self.bind_to_task()

        def COMMITTED(self):
            self.intialize = False

        def event_validity_check(self, source, comp, name, value, code, text):

            coordinator_task = None

            collab = self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][self.coordinator_name][2]
            taskType = self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][self.coordinator_name][0]

            #check if the event received is valid
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

                except Exception as e:
                    coordinator_task = False
                    print ("Error in event validity check", e)

            return coordinator_task

        def event(self, source, comp, name, value, code = None, text = None):
            if True: #try:
                #collaboration binding related event handling
                if comp == 'Task_Collaborator' and name == 'binding_state':
                    collaborators = self.parent.master_tasks[code]['collaborators']

                    if (value.lower() != 'inactive'
                        and code == self.master_task_uuid
                        and code in self.parent.master_tasks
                        and text in collaborators
                        ):

                        try:
                            self.parent.master_tasks[code]['collaborators'][text]['state'][2] = value

                        except Exception as e:
                            print ('Error', e)
                            time.sleep(0.5)
                            print ('Retrying in 0.5 sec')
                            self.parent.master_tasks[code]['collaborators'][text]['state'][2] = value

                        if value.lower() == 'preparing':
                            self.task.superstate.prepare()

                        elif value.lower() == 'committed':
                            self.task.superstate.commit_check()

                    elif value.lower() == "inactive" and self.interface.value().lower() == "committed":
                        if self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][text]:
                            self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][text][1] = 'COMPLETE'

                        if self.parent.master_tasks[code]['collaborators'][text]['state'][2] == 'COMMITTED':
                            self.parent.master_tasks[code]['collaborators'][text]['state'][2] = 'INACTIVE'

                        self.task.superstate.commit()


                #interface related event handling
                elif 'SubTask' in name and self.interface.value().lower() == 'committed':
                    if value.lower() == 'fail' or value.lower() == 'complete':

                        for key,val in self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'].items():

                            if val and (key == text or text in val[2]) and name.split('_')[-1] == val[3]:
                                self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][key][1] = value
                                self.task.superstate.commit()


                    coordinator_task = None

                    coordinator_master_task = self.parent.master_tasks[code]['coordinator'][self.coordinator_name]

                    coordinator_collab = coordinator_master_task['SubTask'][self.coordinator_name][2]
                    coordinator_subtask_type = coordinator_master_task['SubTask'][self.coordinator_name][3]

                    if coordinator_subtask_type in name and text in coordinator_collab:
                        coordinator_task = True

                    else:
                        coordinator_task = self.event_validity_check(source, comp, name, value, code, text)

                    if coordinator_task:
                        self.task.superstate.event(source, comp, name, value, code, text)

            if False: #except Exception as e:
                print ("Error processing coordinator event:")
                print (e)
                print ("Retrying in 1 sec")
                time.sleep(1)


    def __init__(self, parent, interface, master_task_uuid, coordinator_name):
        self.superstate = coordinator.StateMachineModel(parent, interface, master_task_uuid, coordinator_name)
        self.statemachine = self.create_statemachine(self.superstate)


    def create_statemachine(self, state_machine_model):
        NestedState.separator = ':'

        states = [
            {
                'name':'base',
                'children':[
                    'inactive',
                    'committed'
                ]
            }
        ]

        transitions = [
            ['unavailable', 'base', 'base:inactive'],

            ['bind_to_task', 'base:inactive', 'base:committed'],

            ['completed', 'base:committed', 'base:inactive'],
            ['failed', 'base:committed', 'base:inactive'],

            ['default', 'base:inactive', 'base:inactive'],
            ['default', 'base:committed', 'base:committed']
        ]

        statemachine = Machine(
            model = state_machine_model,
            states = states,
            transitions = transitions,
            initial = 'base',
            ignore_invalid_triggers=True
            )

        statemachine.on_enter('base:inactive', 'INACTIVE')
        statemachine.on_enter('base:committed', 'COMMITTED')

        return statemachine
