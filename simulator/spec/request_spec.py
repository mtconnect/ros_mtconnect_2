from expects import *
from doublex import *

from src.request import *

with description('request'):
    with it('should pass'):
        pass
    
    with context('request'):
        with before.each:
            self.request = Request(interface())
            
        with it('should have a request'):
            expect(self.request).to(be_a(Request))
        
        with it('should have an interface'):
            expect(self.request.interface).to(be_a(interface))

        with it('should expect interface to have no value'):
            expect(self.request.interface.value).to(equal(''))

        with context('state machine'):
            with before.each:
                #Initialize the state machine
                self.request.create_statemachine()
                #assuming a robot state variable 'bot_interface'. we can account for it later when defining the robot states
                bot_interface="NOT_READY"
                self.bot_interface=bot_interface
            
            with it('should have a statemachine'):
                expect(self.request.statemachine).to(be_a(Machine))

            with it('should expect the intial state to be base'):
                expect(self.request.superstate.state).to(equal('base'))

            with it('should become not ready when unavailable'):
                self.request.superstate.unavailable()
                expect(self.request.superstate.state).to(equal('base:not_ready'))
                expect(self.request.interface.value).to(equal('NOT_READY'))

            with it('should become not ready when deactivate'):
                self.request.superstate.deactivate()
                expect(self.request.superstate.state).to(equal('base:not_ready'))
                expect(self.request.interface.value).to(equal('NOT_READY'))

            with it('should become ready when idle'):
                self.request.superstate.unavailable()
                self.request.superstate.idle()
                expect(self.request.superstate.state).to(equal('base:ready'))
                expect(self.request.interface.value).to(equal('READY'))

            with it('should become active when activate'):
                self.request.superstate.unavailable()
                self.request.superstate.activate()
                expect(self.request.superstate.state).to(equal('base:active'))
                expect(self.request.interface.value).to(equal('ACTIVE'))

            with it('should become active from ready when activate'):
                self.request.superstate.unavailable()
                self.request.superstate.idle()
                self.request.superstate.activate()
                expect(self.request.superstate.state).to(equal('base:active'))
                expect(self.request.interface.value).to(equal('ACTIVE'))

            with it('should become ready from active when idle'):
                self.request.superstate.unavailable()
                self.request.superstate.activate()
                self.request.superstate.idle()
                expect(self.request.superstate.state).to(equal('base:ready'))
                expect(self.request.interface.value).to(equal('READY'))

            with it('should become fail from active when failure'):
                self.request.superstate.unavailable()
                self.request.superstate.activate()
                self.request.superstate.failure()
                expect(self.request.superstate.state).to(equal('base:fail'))
                expect(self.request.interface.value).to(equal('FAIL'))

            with it('should become active from ready when bot is ready'):
                self.bot_interface="READY"
                self.request.superstate.unavailable()
                self.request.superstate.idle()
                #We could probably add this 'if' statement as a method in request!?
                if self.bot_interface=="READY":
                    self.request.superstate.ready()
                expect(self.request.superstate.state).to(equal('base:active'))
                expect(self.request.interface.value).to(equal('ACTIVE'))

            with it('should become ready from active when bot is not ready'):
                self.bot_interface="NOT_READY"
                self.request.superstate.unavailable()
                self.request.superstate.activate()
                #We could probably add this 'if' statement as a method in request!?
                if self.bot_interface=="NOT_READY":
                    self.request.superstate.not_ready()
                expect(self.request.superstate.state).to(equal('base:ready'))
                expect(self.request.interface.value).to(equal('READY'))

            with it('should become processing from active when bot is active'):
                self.bot_interface="ACTIVE"
                self.request.superstate.unavailable()
                self.request.superstate.activate()
                #We could probably add this 'if' statement as a method in request!?
                if self.bot_interface=="ACTIVE":
                    self.request.superstate.active()
                expect(self.request.superstate.state).to(equal('base:processing'))
