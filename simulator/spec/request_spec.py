from expects import *

import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from src.interfaces.request import *

from mock import Mock, MagicMock
from threading import enumerate
CNC = Mock(return_value = True)

class Event(object):
    def __init__(self):
        self._value = "NOT_READY"
    def set_value(self,a):
        self._value = a
    def value(self):
        return self._value

CNC.superstate.material_unload = Event()

with description('request'):
    with before.each:
        #Taking material unload interface as an example
        self.request = Request(CNC.superstate, CNC.superstate.adapter, CNC.superstate.material_unload, True)

    with it('should have a request'):
        expect(self.request).to(be_a(Request))

    with it('should have an interface'):
        expect(self.request.interface).to(be_a(Event))

    with it('should expect interface to have default value as NOT_READY'):
        expect(self.request.interface.value()).to(equal('NOT_READY'))

    with context('state machine'):
        with before.each:
            self.request.superstate.start()
            self.request.superstate.fail_time_limit = 0


        with it('should have a statemachine'):
            expect(self.request.statemachine).to(be_a(Machine))

        with it('should expect the initial state to be not_ready'):
            expect(self.request.superstate.state).to(equal('base:not_ready'))
            expect(self.request.interface.value()).to(equal('NOT_READY'))

        with it('should remain not ready when unrecognized event'):
            expect(self.request.superstate.state).to(equal('base:not_ready'))
            expect(self.request.interface.value()).to(equal('NOT_READY'))

            self.request.superstate.unavailable()
            expect(self.request.superstate.state).to(equal('base:not_ready'))
            expect(self.request.interface.value()).to(equal('NOT_READY'))

            self.request.superstate.ready()
            expect(self.request.superstate.state).to(equal('base:not_ready'))
            expect(self.request.interface.value()).to(equal('NOT_READY'))

            self.request.superstate.active()
            expect(self.request.superstate.state).to(equal('base:not_ready'))
            expect(self.request.interface.value()).to(equal('NOT_READY'))

        with it('should become ready when idle'):
            self.request.superstate.idle()
            expect(self.request.superstate.state).to(equal('base:ready'))
            expect(self.request.interface.value()).to(equal('READY'))

        with it('should become active when activate'):
            self.request.superstate.activate()
            expect(self.request.superstate.state).to(equal('base:active'))
            expect(self.request.interface.value()).to(equal('ACTIVE'))

        with it('should become active from ready when activate'):
            self.request.superstate.idle()
            self.request.superstate.activate()
            expect(self.request.superstate.state).to(equal('base:active'))
            expect(self.request.interface.value()).to(equal('ACTIVE'))

        with it('should become ready from active when idle'):
            self.request.superstate.activate()
            self.request.superstate.idle()
            expect(self.request.superstate.state).to(equal('base:ready'))
            expect(self.request.interface.value()).to(equal('READY'))

        with it('should become fail from active when failure'):
            self.request.superstate.fail_time_limit = 1
            self.request.superstate.activate()
            self.request.superstate.failure()
            expect(self.request.superstate.state).to(equal('base:fail'))
            expect(self.request.interface.value()).to(equal('FAIL'))

        with it('should become active from ready when ready interface event is received'):
            self.request.superstate.idle()

            self.request.superstate.ready() #event processing from the parent
            expect(self.request.superstate.state).to(equal('base:active'))
            expect(self.request.interface.value()).to(equal('ACTIVE'))

        with it('should become ready from active when not ready interface event is received'):
            self.request.superstate.activate()

            self.request.superstate.not_ready() #event processing from the parent
            expect(self.request.superstate.state).to(equal('base:ready'))
            expect(self.request.interface.value()).to(equal('READY'))

        with it('should become processing from active when active interface event is received'):
            self.request.superstate.PROCESSING = Mock()

            self.request.superstate.activate()
            self.request.superstate.active() #event processing from the parent
            expect(self.request.superstate.state).to(equal('base:processing'))

        with it('should become fail from processing when unrecognized event'):
            self.request.superstate.fail_time_limit = 1
            self.request.superstate.processing_time_limit = 1

            self.request.superstate.activate()
            self.request.superstate.active()

            expect(self.request.superstate.state).to(equal('base:processing'))

            self.request.superstate.idle()

            expect(self.request.superstate.state).to(equal('base:fail'))
            expect(self.request.interface.value()).to(equal('FAIL'))

        with it('should become fail from processing when timeout'):
            self.request.superstate.processing_time_limit = 0
            self.request.superstate.fail_time_limit = 1

            self.request.superstate.activate()
            self.request.superstate.active()
            #expect(self.request.superstate.state).to(equal('base:processing')) #Test this when processing_time_limit is greater than 0 and add a sleep after

            expect(self.request.superstate.state).to(equal('base:fail'))
            expect(self.request.interface.value()).to(equal('FAIL'))

        with it('should become ready from fail when unrecognized event'):
            self.request.superstate.fail_time_limit = 1

            self.request.superstate.activate()
            self.request.superstate.failure()

            expect(self.request.superstate.state).to(equal('base:fail'))
            expect(self.request.interface.value()).to(equal('FAIL'))

            self.request.superstate.idle()

            expect(self.request.superstate.state).to(equal('base:ready'))
            expect(self.request.interface.value()).to(equal('READY'))

        with it('should become ready from fail when timeout'):
            self.request.superstate.fail_time_limit = 0
            self.request.superstate.activate()
            self.request.superstate.failure()

            #timeout # add timeout
            expect(self.request.superstate.state).to(equal('base:ready'))
            expect(self.request.interface.value()).to(equal('READY'))


        with it('should become not ready when complete'):
            self.request.superstate.processing_time_limit = 1
            self.request.superstate.activate()
            self.request.superstate.active()
            self.request.superstate.complete()
            expect(self.request.superstate.state).to(equal('base:not_ready'))
