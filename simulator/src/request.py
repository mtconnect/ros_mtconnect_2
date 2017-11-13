
from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState

class Request(object):
    
    def __init__(self, parent, adapter, interface, rel):

        self.parent = parent
        self.adapter = adapter
        self.interface = interface

        self.active = True
        self.failing = False

        self.dail_time_limit = 1.0
        self.processing_time_limit = 600.0

        self.time = None

        if rel: self.related = rel

    def create_statemachine(self):
        
        global superstate

        NestedState.separator = ':'

        states = [{'name':'base', 'children':['not_ready', 'ready', 'active', 'processing', 'fail']}]

        transitions= [['unavailable','base','base:not_ready'],
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
                      ['','base:processing','base:fail'],

                      ['','base:fail','base:ready']]

        superstate = Machine(states=states, transitions=transitions, initial='base',ignore_invalid_triggers=True)

if __name__ == "__main__":
    
    statemachine = Request('parent', 'adapter', 'interface', True)
    statemachine.create_statemachine()
        
    
