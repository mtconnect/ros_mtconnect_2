from expects import *

import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from src.adapter.data_item import Event
from src.interfaces.request import *

#Taking a CNC statemachine to test out request interface behavior
from src.cnc import cnc
CNC = cnc('localhost',7878)
CNC.create_statemachine()

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
            #Initialize the state machine
            self.request.create_statemachine()

            self.request.adapter.begin_gather()
            self.request.interface.set_value('NOT_READY')
            self.request.adapter.complete_gather()

            self.request.superstate.start()

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
            self.request.superstate.activate()

            self.request.superstate.active() #event processing from the parent
            expect(self.request.superstate.state).to(equal('base:processing'))

            #Ending the thread: Not part of the test
            self.request.superstate.complete()
            time.sleep(1)
            self.request.parent.binding_state_material.set_value("UNAVAILABLE")

        with it('should become fail from processing when unrecognized event'):
            self.request.superstate.activate()

            self.request.superstate.active()
            expect(self.request.superstate.state).to(equal('base:processing'))

            self.request.superstate.IDLE()
            time.sleep(0.2)
            expect(self.request.superstate.state).to(equal('base:fail'))
            expect(self.request.interface.value()).to(equal('FAIL'))

        with it('should become fail from processing when timeout'):
            self.request.superstate.processing_time_limit = 1
            self.request.superstate.activate()

            self.request.superstate.active()
            expect(self.request.superstate.state).to(equal('base:processing'))

            time.sleep(1.5) #Simulation for timeout
            expect(self.request.superstate.state).to(equal('base:fail'))
            expect(self.request.interface.value()).to(equal('FAIL'))

        with it('should become ready from fail when unrecognized event'):
            self.request.superstate.activate()
            self.request.superstate.failure()

            expect(self.request.superstate.state).to(equal('base:fail'))
            expect(self.request.interface.value()).to(equal('FAIL'))

            self.request.superstate.IDLE()
            time.sleep(0.2)
            expect(self.request.superstate.state).to(equal('base:ready'))
            expect(self.request.interface.value()).to(equal('READY'))

        with it('should become ready from fail when timeout'):
            self.request.superstate.fail_time_limit = 1
            self.request.superstate.activate()
            self.request.superstate.failure()

            time.sleep(1.5) #Simulation for timeout
            expect(self.request.superstate.state).to(equal('base:ready'))
            expect(self.request.interface.value()).to(equal('READY'))
