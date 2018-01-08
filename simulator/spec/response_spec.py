from expects import *
from doublex import *
from src.response import *

with description('response'):
    with before.each:
        #taking open door example
        self.response = Response('parent', 'adapter', interface(), 'door', 'OPEN', 'UNLATCHED', True, simulate= True)
        
    with it('should have a response'):
        expect(self.response).to(be_a(Response))
            
    with it('should have an interface'):
        expect(self.response.interface).to(be_a(interface))

    with context('state machine'):
        with before.each:
            self.response.create_statemachine()
            self.response.superstate.start()
            self.bot_interface = "NOT_READY"

        with it('should have a statemachine'):
            expect(self.response.statemachine).to(be_a(Machine))

        with it('should expect the intial state to be not_ready'):
            expect(self.response.superstate.state).to(equal('base:not_ready'))
            expect(self.response.interface.value).to(equal('NOT_READY'))

        with it('should remain not ready when unrecognized event'):
            expect(self.response.superstate.state).to(equal('base:not_ready'))
            expect(self.response.interface.value).to(equal('NOT_READY'))
            self.response.superstate.unavailable()
            expect(self.response.superstate.state).to(equal('base:not_ready'))

        #testing out two simple transitions
        with it('should become ready from not_ready when bot is ready'):
            self.bot_interface = "READY"
            #assuming bot state is simulated/defined externally.
            self.response.superstate.ready()
            expect(self.response.superstate.state).to(equal('base:ready'))

        with it('should become fail from not_ready when failure or active'):
            self.response.superstate.failure()
            expect(self.response.superstate.state).to(equal('base:fail'))
            self.response.superstate.not_ready()
            self.response.superstate.active()
            expect(self.response.superstate.state).to(equal('base:fail'))
            

        with it('should become complete from active when simulated duration'):
            self.bot_interface = "ACTIVE"
            #assuming bot state is simulated/defined externally.
            self.response.superstate.ready()
            self.response.superstate.active()
            expect(self.response.superstate.state).to(equal('base:active'))
            time.sleep(1.1)
            expect(self.response.superstate.state).to(equal('base:complete'))

        with it('should become fail then not ready from active when unrecognized event'):
            self.bot_interface = "ACTIVE"
            #assuming bot state is simulated/defined externally.
            self.response.superstate.ready()
            self.response.superstate.active()
            expect(self.response.superstate.state).to(equal('base:active'))
            self.response.superstate.READY()
            #takes time to change state
            time.sleep(0.100)
            expect(self.response.superstate.state).to(equal('base:fail'))
            time.sleep(1.1)
            expect(self.response.superstate.state).to(equal('base:not_ready'))


        
            
            
