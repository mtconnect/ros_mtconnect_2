from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time

#intializing interface here for testing.
class interface(object):
    
    def __init__(self):
        self.value = ""
        self.bot_interface_value = ""

class Response(object):

    def __init__(self, parent, adapter, interface, prefix, dest_state, transition_state, response_state, rel, simulate = True):

        self.parent = parent
        self.adapter = adapter
        self.interface = interface
        self.prefix = prefix
        self.dest_state = prefix + dest_state
        self.transition_state = prefix + transition_state
        if rel: self.related = rel
        else: self.related = rel
        self.simulate = simulate
        self.is_active = True

        class statemachineModel(object):

            def __init__(self, adapter, interface, parent, prefix, dest_state, response_state, rel = True, simulate = True):

                self.interface = interface
                self.adapter = adapter
                self.parent = parent
                self.dest_state = dest_state
                self.prefix = prefix
                self.response_state = response_state
                self.simulate = simulate
                self.fail_reset_delay = 1
                self.fail_next = False
                if rel: self.related = rel
                else: self.related = False
                self.simulated_duration = 1.5
                #add on later

            def check_state_calls(func):
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
                    self.COMPLETE()
                    
                else:
                    self.adapter.begin_gather()
                    self.interface.set_value("ACTIVE")
                    self.adapter.complete_gather()
                    if self.simulate:
                        self.adapter.begin_gather()
                        self.response_state.set_value("UNLATCHED")
                        self.adapter.complete_gather()
                    check_state_list=[
                    self.DEFAULT.has_been_called, self.FAILURE.has_been_called, self.READY.has_been_called, self.NOT_READY.has_been_called, self.COMPLETE.has_been_called
                    ]
                    if self.simulate:
                        def sim_duration():
                            timer_processing = Timer(self.simulated_duration,self.complete)
                            timer_processing.start()
                            while timer_processing.isAlive():
                                if self.COMPLETE.has_been_called!=check_state_list[4]:
                                    timer_processing.cancel()
                                    break

                                elif self.FAILURE.has_been_called!=check_state_list[1]:
                                    timer_processing.cancel()
                                    self.DEFAULT()
                                    break
                            
                                elif self.READY.has_been_called!=check_state_list[2]:
                                    timer_processing.cancel()
                                    self.DEFAULT()
                                    break

                                elif self.NOT_READY.has_been_called!=check_state_list[3]:
                                    timer_processing.cancel()
                                    self.DEFAULT()
                                    break
                                
                        t = Thread(target = sim_duration)
                        t.start()

            @check_state_calls
            def FAILURE(self):
                self.adapter.begin_gather()
                self.interface.set_value("FAIL")
                self.adapter.complete_gather()
                print "FAILED!!!!!!!!!!!!!!!!!!!!!"
                try:
                    self.parent.FAILED()
                except:
                    "Local Spec Testing"
                def fail_reset():
                    time.sleep(self.fail_reset_delay)
                    self.not_ready()
                t = Thread(target = fail_reset)
                t.start()

            @check_state_calls  
            def COMPLETE(self):
                
                if self.simulate:
                    self.adapter.begin_gather()
                    self.response_state.set_value(self.dest_state)
                    self.adapter.complete_gather()
               
                self.parent.interface_type(value = 'Response'+self.prefix.lower()+self.dest_state.lower())
                self.parent.COMPLETED()
                self.adapter.begin_gather()
                self.interface.set_value("COMPLETE")
                self.adapter.complete_gather()

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

            @check_state_calls
            def RESET(self):
                self.reset()

            @check_state_calls
            def DEFAULT(self):
                self.default()

        
        self.superstate = statemachineModel(adapter = adapter, interface = interface, parent = parent, prefix = prefix, dest_state = dest_state, response_state = response_state, rel = rel, simulate = simulate)
    


    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['not_ready', 'ready', 'active', 'fail', 'complete']}]

        transitions = [['start','base','base:not_ready'],

                       [self.dest_state, 'base:not_ready', 'base:not_ready'],
                       [self.transition_state, 'base:not_ready', 'base:not_ready'],
                       [self.prefix+'_open', 'base:not_ready', 'base:not_ready'],
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
                       [self.transition_state, 'base:active', 'base:active'],
                       ['complete', 'base:active', 'base:complete'],
                       [self.dest_state, 'base:active', 'base:active'],

                       ['default', 'base:complete', 'base:complete'],
                       ['not_ready', 'base:complete', 'base:not_ready'],
                       ['unavailable', 'base:complete', 'base:not_ready'],
                       ['ready', 'base:complete', 'base:ready'],

                       ['default', 'base:fail', 'base:fail'],
                       ['not_ready', 'base:fail', 'base:not_ready'],
                       ['failure', 'base:fail', 'base:not_ready'],

                       ['reset','*','base:not_ready']]

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)
        self.statemachine.on_enter('base:not_ready', 'NOT_READY')
        self.statemachine.on_enter('base:ready', 'READY')
        self.statemachine.on_enter('base:active', 'ACTIVE')
        self.statemachine.on_enter('base:fail', 'FAILURE')
        self.statemachine.on_enter('base:complete', 'COMPLETE')
                                   