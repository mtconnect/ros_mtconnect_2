
from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer


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
                self.processing_time_limit = 600.0
                self.fail_time_limit = 1.0
                self.failing = False
                self.parent = parent

            def DEACTIVATE(self):
                if not self.failing: self.superstate.deactivate()
                
            def start_timer(self,timeout):
                self.timer=Timer(timeout,self.default) #func directs it to the next state???
                self.timer.start() #no need for a 'kill the timer' method since a new timer request overwrites the timer. should i still do it? Do we need separate timer instances for fail and processing? I dont think so! 
            
            def NOT_READY(self):
                self.interface.value = "NOT_READY"

            def READY(self):
                self.interface.value = "READY"

            def ACTIVE(self):
                self.interface.value = "ACTIVE"

            def FAIL(self):
                self.interface.value = "FAIL"
                self.start_timer(self.fail_time_limit)

            def COMPLETE_FAILED(self):
                self.failing = True
                #self.parent.failed
                self.failing = False

            def COMPLETE(self):
                #self.parent.completed
                pass
        
            def PROCESSING(self):
                self.start_timer(self.processing_time_limit)

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
        self.superstate.start()
        self.statemachine.on_enter('base:ready', 'READY')
        self.statemachine.on_enter('base:active', 'ACTIVE')
        self.statemachine.on_enter('base:not_ready', 'NOT_READY')
        self.statemachine.on_enter('base:fail', 'FAIL')
        self.statemachine.on_exit('base:fail', 'COMPLETE_FAILED')
        self.statemachine.on_enter('base:processing', 'PROCESSING')
        
