
from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time


#intializing interface here for testing.
class interface(object):
    
    def __init__(self):
        self.value = ""
        self.bot_interface_value = ""


class Request(object):
    
    def __init__(self, parent, adapter, interface, rel):

        class statemachineModel(object):
        
            def __init__(self,interface = interface, parent = parent):
                self.interface = interface
                self.processing_time_limit = 2
                self.fail_time_limit = 2
                self.failing = False
                self.parent = parent

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
                self.interface.value = "NOT_READY"

            @check_state_calls
            def READY(self):
                self.interface.value = "READY"

            @check_state_calls
            def ACTIVE(self):
                self.interface.value = "ACTIVE"

            @check_state_calls
            def FAILURE(self):
                self.interface.value = "FAIL"
                check_state_list=[
                    self.FAILURE.has_been_called, self.ACTIVE.has_been_called, self.READY.has_been_called, self.IDLE.has_been_called, self.NOT_READY.has_been_called, self.ACTIVATE.has_been_called, self.COMPLETE.has_been_called, self.DEFAULT.has_been_called
                    ]

                #all the triggers addressed except active,not_ready,ready which would come from the bot interface. To be done.
                def complete_check():
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
                    
                        
                t = Thread(target = complete_check)
                t.start()
                

            def complete_failed(self):
                
                self.failing = True
                try:
                    self.parent.interface_type(value = 'Request')
                    self.parent.FAILED()
                except:
                    "Local Spec Testing"
                self.failing = False
                #print 'comp fail here'

            @check_state_calls 
            def COMPLETE(self):
                self.complete()
                try:
                    self.parent.interface_type(value = 'Request')
                    self.parent.COMPLETED()
                except:
                    "Local Spec Testing"

            def void(self):
                pass

            def PROCESSING(self):
                
                check_state_list=[
                    self.FAILURE.has_been_called, self.ACTIVE.has_been_called, self.READY.has_been_called, self.IDLE.has_been_called, self.NOT_READY.has_been_called, self.ACTIVATE.has_been_called, self.DEFAULT.has_been_called
                    ]
                #all the triggers addressed except active,not_ready,ready which would come from the bot interface. To be done.
                def complete_check():
                    timer_processing = Timer(self.processing_time_limit,self.void)
                    timer_processing.start()
                    while timer_processing.isAlive():
                        if self.COMPLETE.has_been_called:
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

                
        self.superstate = statemachineModel(interface)
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
        
