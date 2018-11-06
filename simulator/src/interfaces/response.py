from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time

class Response:

    class StateMachineModel:

        def __init__(self, parent, adapter, interface, prefix, dest_state, transition_state, response_state, rel = True, simulate = True):

            self.interface = interface
            self.adapter = adapter
            self.parent = parent
            self.dest_state = dest_state
            self.prefix = prefix
            self.response_state = response_state
            self.transition_state = transition_state
            self.simulate = simulate

            #default fail reset time
            self.fail_reset_delay = 1.0
            self.fail_next = False

            #default simulated response completion time
            self.simulated_duration = 1.0

            self.timer =Timer(0,self.void)

        def NOT_READY(self):
            self.adapter.begin_gather()
            self.interface.set_value("NOT_READY")
            self.adapter.complete_gather()

        def READY(self):
            self.adapter.begin_gather()
            self.interface.set_value("READY")
            self.adapter.complete_gather()

        def ACTIVE(self):
            if self.fail_next:
                self.adapter.begin_gather()
                self.interface.set_value("ACTIVE")
                self.adapter.complete_gather()
                self.fail_next = False
                self.FAILURE()

            elif self.response_state.value() == self.dest_state:
                self.adapter.begin_gather()
                self.interface.set_value("ACTIVE")
                self.adapter.complete_gather()
                self.complete()

            else:
                self.adapter.begin_gather()
                self.interface.set_value("ACTIVE")
                self.response_state.set_value(self.transition_state)
                self.adapter.complete_gather()

                if self.simulate:
                    #method for response completion either after a timeout or after a valid trigger is called
                    if not self.simulated_duration:
                        return self.complete()

                    self.timer = Timer(self.simulated_duration, self.complete)
                    self.timer_daemon = True
                    self.timer.start()

        def FAILURE(self):
            if self.timer.isAlive():
                self.timer.cancel()

            self.adapter.begin_gather()
            self.interface.set_value("FAIL")
            self.adapter.complete_gather()

            self.parent.interface_type(value = 'Response')
            self.parent.FAILED()

            if not self.fail_reset_delay:
                return self.not_ready()

            self.timer = Timer(self.fail_reset_delay,self.not_ready)
            self.timer.daemon =True
            self.timer.start()


        def COMPLETE(self):
            if self.timer.isAlive():
                self.timer.cancel()

            self.adapter.begin_gather()
            self.response_state.set_value(self.dest_state)
            self.adapter.complete_gather()

            self.adapter.begin_gather()
            self.interface.set_value("COMPLETE")
            self.adapter.complete_gather()

            self.parent.interface_type(value = 'Response'+self.prefix.lower()+self.dest_state.lower())
            self.parent.COMPLETED()


        def void(self):
            pass

        def DEACTIVATE(self):
            self.is_active = False
            if self.state!='base:not_ready' and self.state!='base:fail':
                self.reset()

        def ACTIVATE(self):
            self.is_active = True
            if self.state!='base:not_ready' and self.state!='base:fail':
                self.ready()

        def RESET(self):
            self.reset()

        def DEFAULT(self):
            self.default()


    def __init__(self, parent, adapter, interface, prefix, dest_state, transition_state, response_state, rel, simulate = True):

        self.superstate = Response.StateMachineModel(parent, adapter, interface, prefix, dest_state, transition_state, response_state, True, simulate)
        self.statemachine = self.create_statemachine(self.superstate)
        
        self.parent = parent
        self.adapter = adapter
        self.interface = interface
        self.prefix = prefix
        self.dest_state = prefix + dest_state
        self.transition_state = prefix + transition_state
        self.simulate = simulate
        self.is_active = True


    def create_statemachine(self, state_machine_model):
        NestedState.separator = ':'

        states = [
            {
                'name':'base',
                'children':[
                    'not_ready',
                    'ready',
                    'active',
                    'fail',
                    'complete'
                ]
            }
        ]

        transitions = [

            ['start','base','base:not_ready'],

            ['default', 'base:not_ready', 'base:not_ready'],
            ['not_ready', 'base:not_ready', 'base:not_ready'],
            ['failure', 'base:not_ready', 'base:fail'],
            ['ready', 'base:not_ready', 'base:ready'],
            ['active', 'base:not_ready', 'base:fail'],

            ['default', 'base:ready', 'base:ready'],
            ['not_ready', 'base:ready', 'base:not_ready'],
            ['unavailable', 'base:ready', 'base:not_ready'],
            ['active', 'base:ready', 'base:active'],
            ['failure', 'base:ready', 'base:fail'],

            ['default', 'base:active', 'base:fail'],
            ['complete', 'base:active', 'base:complete'],

            ['ready','base:active','base:fail'],
            ['not_ready','base:active','base:fail'],
            ['failure','base:active','base:fail'],

            ['default', 'base:complete', 'base:complete'],
            ['not_ready', 'base:complete', 'base:not_ready'],
            ['unavailable', 'base:complete', 'base:not_ready'],
            ['ready', 'base:complete', 'base:ready'],

            ['default', 'base:fail', 'base:fail'],
            ['not_ready', 'base:fail', 'base:not_ready'],
            ['failure', 'base:fail', 'base:not_ready'],

            ['reset','*','base:not_ready']
        ]

        statemachine = Machine(
            model = state_machine_model,
            states = states,
            transitions = transitions,
            initial = 'base',
            ignore_invalid_triggers=True
        )

        statemachine.on_enter('base:not_ready', 'NOT_READY')
        statemachine.on_enter('base:ready', 'READY')
        statemachine.on_enter('base:active', 'ACTIVE')
        statemachine.on_enter('base:fail', 'FAILURE')
        statemachine.on_enter('base:complete', 'COMPLETE')

        return statemachine
