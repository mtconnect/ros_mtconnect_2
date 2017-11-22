
from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState


#intializing interface here for testing.
class interface(object):
    
    def __init__(self):
        self.value = ""


class Request(object):
    
    def __init__(self, interface):

        class statemachineModel(object):
        
            def __init__(self,interface = interface):
                self.interface = interface
            
            def NOT_READY(self):
                self.interface.value = "NOT_READY"

            def READY(self):
                self.interface.value = "READY"

            def ACTIVE(self):
                self.interface.value = "ACTIVE"
                
            def FAIL(self):
                self.interface.value = "FAIL"

        self.superstate = statemachineModel(interface)
        self.interface = self.superstate.interface        

    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['not_ready', 'ready', 'active', 'fail', 'processing']}]

        transitions= [['unavailable','base','base:not_ready'],
                      ['deactivate','base','base:not_ready'],
                      
                      ['activate','base:not_ready','base:active'],
                      ['idle','base:not_ready','base:ready'],
                      
                      ['ready','base:ready','base:active'],
                      ['activate','base:ready','base:active'],
                      
                      ['idle','base:active','base:ready'],
                      ['not_ready','base:active','base:ready'],
                      ['failure','base:active','base:fail'],
                      ['active','base:active','base:processing']]

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)

        self.statemachine.on_enter('base:ready', 'READY')
        self.statemachine.on_enter('base:active', 'ACTIVE')
        self.statemachine.on_enter('base:not_ready', 'NOT_READY')
        self.statemachine.on_enter('base:fail', 'FAIL')
        
