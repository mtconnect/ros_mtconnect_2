from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time

#will be included under assets!?
class interface(object):

    def __init__(self, value):
        self.value = value

class coordinator(object):

    def __init__(self, interface):

        class statemachineModel(object):

            def __init__(self, interface):

                self.interface = interface

            def INACTIVE(self):
                self.interface.value = 'INACTIVE'

            def COMMITTED(self):
                self.interface.value = 'COMMITTED'

        self.superstate = statemachineModel(interface)

    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['inactive', 'committed']}]

        transitions = [['unavailable', 'base', 'base:inactive'],

                       ['committed', 'base:inactive', 'base:committed'],

                       ['completed', 'base:committed', 'base:inactive'],
                       ['failed', 'base:committed', 'base:inactive'],

                       ['default', 'base:inactive', 'base:inactive'],
                       ['default', 'base:committed', 'base:committed']
                       ]

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)
                       
        self.statemachine.on_enter('base:inactive', 'INACTIVE')
        self.statemachine.on_enter('base:committed', 'COMMITTED')
