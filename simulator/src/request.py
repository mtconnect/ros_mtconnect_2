
from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools


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
                self.processing_time_limit = 20.0
                self.fail_time_limit = 1.0
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
                if not self.failing: self.superstate.deactivate()

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

            def start_timer(self,timeout):
                self.timer=Timer(timeout,self.DEFAULT) #func directs it to the next state???
                self.timer.start() #no need for a 'kill the timer' method since a new timer request overwrites the timer. should i still do it? Do we need separate timer instances for fail and processing? I dont think so! 

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
                self.start_timer(self.fail_time_limit)
                check_state_list=[
                    self.FAILURE.has_been_called, self.ACTIVE.has_been_called, self.READY.has_been_called, self.IDLE.has_been_called, self.NOT_READY.has_been_called, self.ACTIVATE.has_been_called, self.COMPLETE.has_been_called
                    ]

                #all the triggers addressed except active,not_ready,ready which would come from the bot interface. To be done.
                #also the check function can be made generic. I was thinking of storing the variables as well as the values in a list and simply run it in a loop.
                #but i am running into key-error issues. function is not able to recognize the "self." methods.
                #for the time being I am just listing out all the triggers and doing comparison with their last known states.
                
                def complete_check():
                    while self.timer.isAlive():
                        if self.COMPLETE.has_been_called:
                            self.timer.cancel()

                        elif self.FAILURE.has_been_called!=check_state_list[0]:
                            self.timer.cancel()
                            self.DEFAULT()
                            
                        elif self.ACTIVE.has_been_called!=check_state_list[1]:
                            self.timer.cancel()
                            self.DEFAULT()
                            
                        elif self.READY.has_been_called!=check_state_list[2]:
                            self.timer.cancel()
                            self.DEFAULT()
                            
                        elif self.IDLE.has_been_called!=check_state_list[3]:
                            self.timer.cancel()
                            self.DEFAULT()
                            
                        elif self.NOT_READY.has_been_called!=check_state_list[4]:
                            self.timer.cancel()
                            self.DEFAULT()

                        elif self.ACTIVATE.has_been_called!=check_state_list[5]:
                            self.timer.cancel()
                            self.DEFAULT()

                        elif self.COMPLETE.has_been_called!=check_state_list[5]:
                            self.timer.cancel()
                            self.DEFAULT()
                        
                t = Thread(target = complete_check)
                t.start()

            def complete_failed(self):
                self.failing = True
                #self.parent.failed
                self.failing = False

            @check_state_calls 
            def COMPLETE(self):
                self.complete()
                #self.parent.completed

            def PROCESSING(self):
                self.start_timer(self.processing_time_limit)
                check_state_list=[
                    self.FAILURE.has_been_called, self.ACTIVE.has_been_called, self.READY.has_been_called, self.IDLE.has_been_called, self.NOT_READY.has_been_called, self.ACTIVATE.has_been_called
                    ]

                #all the triggers addressed except active,not_ready,ready which would come from the bot interface. To be done.
                def complete_check():
                    while self.timer.isAlive():
                        if self.COMPLETE.has_been_called:
                            self.timer.cancel()

                        elif self.FAILURE.has_been_called!=check_state_list[0]:
                            self.timer.cancel()
                            self.DEFAULT()
                            
                        elif self.ACTIVE.has_been_called!=check_state_list[1]:
                            self.timer.cancel()
                            self.DEFAULT()
                            
                        elif self.READY.has_been_called!=check_state_list[2]:
                            self.timer.cancel()
                            self.DEFAULT()
                            
                        elif self.IDLE.has_been_called!=check_state_list[3]:
                            self.timer.cancel()
                            self.DEFAULT()
                            
                        elif self.NOT_READY.has_been_called!=check_state_list[4]:
                            self.timer.cancel()
                            self.DEFAULT()

                        elif self.ACTIVATE.has_been_called!=check_state_list[5]:
                            self.timer.cancel()
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
                      
                      ['complete','base:processing','base:not_ready'],
                      ['default','base:processing','base:fail'],
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
        
