from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time

"""Request Interface"""

class Request:

    class StateMachineModel:

        def __init__(self, adapter, interface, parent):
            self.interface = interface
            self.adapter = adapter

            #default request timeout
            self.processing_time_limit = 900

            #default fail reset timeout
            self.fail_time_limit = 2

            self.failing = False
            self.parent = parent

            self.timer = Timer(0,self.void)

        def DEACTIVATE(self):
            if not self.failing: self.deactivate()

        def START(self):
            self.start()

        def UNAVAILABLE(self):
            self.unavailable()

        def ACTIVATE(self):
            self.activate()

        def IDLE(self):
            self.idle()

        def NOT_READY(self):
            self.adapter.begin_gather()
            self.interface.set_value("NOT_READY")
            self.adapter.complete_gather()

        def READY(self):
            if self.timer.isAlive():
                self.timer.cancel()

            self.adapter.begin_gather()
            self.interface.set_value("READY")
            self.adapter.complete_gather()

        def ACTIVE(self):
            self.adapter.begin_gather()
            self.interface.set_value("ACTIVE")
            self.adapter.complete_gather()

        def FAILURE(self):
            if self.timer.isAlive():
                self.timer.cancel()

            self.adapter.begin_gather()
            self.interface.set_value("FAIL")
            self.adapter.complete_gather()

            if not self.fail_time_limit:
                return self.default()

            self.timer = Timer(self.fail_time_limit,self.default)
            self.timer.daemon =True
            self.timer.start()

        def complete_failed(self):

            self.failing = True

            self.parent.interface_type(value = 'Request')
            self.parent.FAILED()

            self.failing = False

        def COMPLETE(self):
            self.complete()

            self.parent.interface_type(value = 'Request')
            self.parent.COMPLETED()

        def void(self):
            pass

        def PROCESSING(self):
            if not self.processing_time_limit:
                return self.default()

            self.timer = Timer(self.processing_time_limit,self.default)
            self.timer.daemon = True
            self.timer.start()

        def RESET(self):
            self.reset()

        def DEFAULT(self):
            self.default()


    def __init__(self, parent, adapter, interface, rel):
        self.superstate = Request.StateMachineModel(adapter, interface, parent)
        self.statemachine = self.create_statemachine(self.superstate)
        self.interface = interface
        self.adapter = adapter

        #default request timeout
        self.processing_time_limit = 900

        #default fail reset timeout
        self.fail_time_limit = 2

        self.failing = False
        self.parent = parent
        

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
                    'processing'
                ]
            }
        ]

        transitions = [
            ['start','base','base:not_ready'],
            ['unavailable','base','base:not_ready'],
            ['deactivate','base','base:not_ready'],

            ['activate','base:not_ready','base:active'],
            ['idle','base:not_ready','base:ready'],

            ['ready','base:ready','base:active'],
            ['activate','base:ready','base:active'],

            ['idle','base:active','base:ready'],
            ['not_ready','base:active','base:ready'],
            ['failure','base:active','base:fail'],
            ['active','base:active','base:processing'],

            ['idle','base:processing','base:fail'],
            ['ready','base:processing','base:fail'],
            ['not_ready','base:processing','base:fail'],
            ['active','base:processing','base:fail'],
            ['activate','base:processing','base:fail'],
            ['fail','base:processing','base:fail'],

            ['idle','base:fail','base:ready'],
            ['ready','base:fail','base:ready'],
            ['not_ready','base:fail','base:ready'],
            ['active','base:fail','base:ready'],
            ['activate','base:fail','base:ready'],
            ['failure','base:fail','base:ready'],
            ['complete','base:fail','base:ready'],

            {
                'trigger':'complete',
                'source':'base:processing',
                'dest':'base:not_ready',
                'after': 'COMPLETE'
            },

            ['default','base:processing','base:fail'],
            ['default','base:fail','base:ready'],
            ['default','base:not_ready','base:not_ready'],
            ['default','base:ready','base:ready'],
            ['default','base:active','base:active'],

            ['reset','*','base:not_ready']
        ]

        statemachine = Machine(
            model = state_machine_model,
            states = states,
            transitions = transitions,
            initial = 'base',
            ignore_invalid_triggers=True
            )

        statemachine.on_enter('base:ready', 'READY')
        statemachine.on_enter('base:active', 'ACTIVE')
        statemachine.on_enter('base:not_ready', 'NOT_READY')
        statemachine.on_enter('base:fail', 'FAILURE')
        statemachine.on_exit('base:fail', 'complete_failed')
        statemachine.on_enter('base:processing', 'PROCESSING')

        return statemachine
