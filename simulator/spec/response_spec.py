from expects import *

import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from src.adapter.data_item import Event
from src.interfaces.response import *

#Taking a CNC statemachine to test out response interface behavior
from src.cnc import cnc
CNC = cnc('localhost',7879)
CNC.create_statemachine()

with description('response'):
    with before.each:
        #Taking open door interface as an example
        self.response = Response(CNC.superstate, CNC.superstate.adapter, CNC.superstate.open_door, 'door', 'OPEN', 'UNLATCHED', CNC.superstate.door_state,rel = True, simulate= True)

    with it('should have a response'):
        expect(self.response).to(be_a(Response))

    with it('should have an interface'):
        expect(self.response.interface).to(be_a(Event))

    with context('state machine'):
        with before.each:
            self.response.create_statemachine()

            self.response.adapter.begin_gather()
            self.response.interface.set_value('NOT_READY')
            self.response.adapter.complete_gather()

            self.response.superstate.start()

        with it('should have a statemachine'):
            expect(self.response.statemachine).to(be_a(Machine))

        with it('should expect the initial state to be not_ready'):
            expect(self.response.superstate.state).to(equal('base:not_ready'))
            expect(self.response.interface.value()).to(equal('NOT_READY'))

        with it('should remain not ready when unrecognized event'):
            expect(self.response.superstate.state).to(equal('base:not_ready'))
            expect(self.response.interface.value()).to(equal('NOT_READY'))

            self.response.superstate.unavailable()
            expect(self.response.superstate.state).to(equal('base:not_ready'))

        with it('should become ready from not_ready when ready interface event is received'):
            self.response.superstate.ready()
            expect(self.response.superstate.state).to(equal('base:ready'))
            expect(self.response.interface.value()).to(equal('READY'))


        with it('should become fail from not_ready when failure or active'):
            self.response.superstate.failure()
            expect(self.response.superstate.state).to(equal('base:fail'))
            expect(self.response.interface.value()).to(equal('FAIL'))

            self.response.superstate.not_ready()
            self.response.superstate.active()
            expect(self.response.superstate.state).to(equal('base:fail'))
            expect(self.response.interface.value()).to(equal('FAIL'))


        with it('should become complete from active when simulated duration'):
            self.response.superstate.ready()
            self.response.superstate.active()
            expect(self.response.superstate.state).to(equal('base:active'))
            time.sleep(1.5)
            expect(self.response.superstate.state).to(equal('base:complete'))
            expect(self.response.interface.value()).to(equal('COMPLETE'))
            self.response.superstate.not_ready()


        with it('should become fail then not ready from active when unrecognized event'):
            self.response.superstate.ready()
            self.response.superstate.response_state.set_value('CLOSED')
            self.response.superstate.active()
            expect(self.response.superstate.state).to(equal('base:active'))
            expect(self.response.interface.value()).to(equal('ACTIVE'))

            self.response.superstate.READY()
            time.sleep(0.100)
            expect(self.response.superstate.state).to(equal('base:fail'))
            expect(self.response.interface.value()).to(equal('FAIL'))


            time.sleep(1.1)
            expect(self.response.superstate.state).to(equal('base:not_ready'))
            expect(self.response.interface.value()).to(equal('NOT_READY'))

