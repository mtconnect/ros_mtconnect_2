
from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time

"""Request Interface"""

class Request(object):

    def __init__(self, parent, adapter, interface, rel):

        class statemachineModel(object):

            def __init__(self, adapter, interface, parent):
                self.interface = interface
                self.adapter = adapter

                #default request timeout
                self.processing_time_limit = 900

                #default fail reset timeout
                self.fail_time_limit = 2

                self.failing = False
                self.parent = parent

            def check_state_calls(func):
                #wrapper to check if a method has been called

                @functools.wraps(func)
                def wrapper(*args, **kwargs):
                    if not wrapper.has_been_called:
                        wrapper.has_been_called = True
                    else:
                        wrapper.has_been_called = False
                    return func(*args, **kwargs)
                wrapper.has_been_called = False
                return wrapper

            @check_state_calls
            def DEACTIVATE(self):
                if not self.failing: self.deactivate()

            @check_state_calls
            def START(self):
                self.start()

            @check_state_calls
            def UNAVAILABLE(self):
                self.unavailable()

            @check_state_calls
            def ACTIVATE(self):
                self.activate()

            @check_state_calls
            def IDLE(self):
                self.idle()

            @check_state_calls
            def NOT_READY(self):
                self.adapter.begin_gather()
                self.interface.set_value("NOT_READY")
                self.adapter.complete_gather()

            @check_state_calls
            def READY(self):
                self.adapter.begin_gather()
                self.interface.set_value("READY")
                self.adapter.complete_gather()

            @check_state_calls
            def ACTIVE(self):
                self.adapter.begin_gather()
                self.interface.set_value("ACTIVE")
                self.adapter.complete_gather()

            @check_state_calls
            def FAILURE(self):
                self.adapter.begin_gather()
                self.interface.set_value("FAIL")
                self.adapter.complete_gather()

                #initial state of all the methods: True (called) or False (not called)
                check_state_list=[
                    self.FAILURE.has_been_called, self.ACTIVE.has_been_called, self.READY.has_been_called, self.IDLE.has_been_called, self.NOT_READY.has_been_called, self.ACTIVATE.has_been_called, self.COMPLETE.has_been_called, self.DEFAULT.has_been_called
                    ]

                #method for statemachine reset either after a timeout or after a valid trigger is called
                def reset_check():
                    timer_failure = Timer(self.fail_time_limit,self.void)
                    timer_failure.start()
                    while timer_failure.isAlive():
                        if self.FAILURE.has_been_called!=check_state_list[0]:
                            timer_failure.cancel()
                            self.DEFAULT()
                            break

                        elif self.ACTIVE.has_been_called!=check_state_list[1]:
                            timer_failure.cancel()
                            self.DEFAULT()
                            break

                        elif self.READY.has_been_called!=check_state_list[2]:
                            timer_failure.cancel()
                            self.DEFAULT()
                            break

                        elif self.IDLE.has_been_called!=check_state_list[3]:
                            timer_failure.cancel()
                            self.DEFAULT()
                            break

                        elif self.NOT_READY.has_been_called!=check_state_list[4]:
                            timer_failure.cancel()
                            self.DEFAULT()
                            break

                        elif self.ACTIVATE.has_been_called!=check_state_list[5]:
                            timer_failure.cancel()
                            self.DEFAULT()
                            break

                        elif self.COMPLETE.has_been_called!=check_state_list[6]:
                            timer_failure.cancel()
                            self.DEFAULT()
                            break

                    if self.DEFAULT.has_been_called==check_state_list[7] and self.state == 'base:fail':
                        self.DEFAULT()

                t = Thread(target = reset_check)
                t.start()


            def complete_failed(self):

                self.failing = True

                self.parent.interface_type(value = 'Request')
                self.parent.FAILED()

                self.failing = False

            @check_state_calls
            def COMPLETE(self):
                self.complete()

                self.parent.interface_type(value = 'Request')
                self.parent.COMPLETED()

            def void(self):
                pass

            def PROCESSING(self):
                #initial state of all the methods: True (called) or False (not called)
                check_state_list=[
                    self.FAILURE.has_been_called, self.ACTIVE.has_been_called, self.READY.has_been_called, self.IDLE.has_been_called, self.NOT_READY.has_been_called, self.ACTIVATE.has_been_called, self.DEFAULT.has_been_called, self.COMPLETE.has_been_called
                    ]

                #method for statemachine request complete check: either faults after a timeout or completes after a valid trigger is called
                def complete_check():
                    timer_processing = Timer(self.processing_time_limit,self.void)
                    timer_processing.start()
                    while timer_processing.isAlive():
                        if self.COMPLETE.has_been_called!=check_state_list[7]:
                            timer_processing.cancel()
                            break

                        elif self.ACTIVE.has_been_called!=check_state_list[1]:
                            timer_processing.cancel()
                            self.DEFAULT()
                            break

                        elif self.READY.has_been_called!=check_state_list[2]:
                            timer_processing.cancel()
                            self.DEFAULT()
                            break

                        elif self.IDLE.has_been_called!=check_state_list[3]:
                            timer_processing.cancel()
                            self.DEFAULT()
                            break

                        elif self.NOT_READY.has_been_called!=check_state_list[4]:
                            timer_processing.cancel()
                            self.DEFAULT()
                            break

                        elif self.ACTIVATE.has_been_called!=check_state_list[5]:
                            timer_processing.cancel()
                            self.DEFAULT()
                            break
                    if self.DEFAULT.has_been_called==check_state_list[6]:
                        self.DEFAULT()

                t = Thread(target = complete_check)
                t.start()

            @check_state_calls
            def RESET(self):
                self.reset()

            @check_state_calls
            def DEFAULT(self):
                self.default()

        self.superstate = statemachineModel(adapter = adapter, interface = interface, parent = parent)
        self.interface = self.superstate.interface
        self.related = None
        if rel: self.related = rel
        self.parent = self.superstate.parent
        self.adapter = adapter
        self.failing = self.superstate.failing

    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['not_ready', 'ready', 'active', 'fail', 'processing']}]

        transitions= [['start','base','base:not_ready'],
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

                      {'trigger':'complete','source':'base:processing','dest':'base:not_ready', 'after': 'COMPLETE'},
                      ['default','base:processing','base:fail'],
                      ['failure','base:processing','base:fail'],
                      ['default','base:fail','base:ready'],
                      ['default','base:not_ready','base:not_ready'],
                      ['default','base:ready','base:ready'],
                      ['default','base:active','base:active'],
                      ['reset','*','base:not_ready']]

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)
        self.statemachine.on_enter('base:ready', 'READY')
        self.statemachine.on_enter('base:active', 'ACTIVE')
        self.statemachine.on_enter('base:not_ready', 'NOT_READY')
        self.statemachine.on_enter('base:fail', 'FAILURE')
        self.statemachine.on_exit('base:fail', 'complete_failed')
        self.statemachine.on_enter('base:processing', 'PROCESSING')
