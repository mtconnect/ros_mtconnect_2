from expects import *
from doublex import *

from src.request import *

with description('request'):
    with before.each:
        self.request = Request('parent', 'adapter', interface(), True)
        
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
            self.request.superstate.start()
            #assuming a robot state variable 'bot_interface'. we can account for it later when defining the robot states
            bot_interface="NOT_READY"
            self.bot_interface=bot_interface
                
        with it('should have a statemachine'):
            expect(self.request.statemachine).to(be_a(Machine))

        with it('should expect the intial state to be not_ready'):
            expect(self.request.superstate.state).to(equal('base:not_ready'))
            expect(self.request.interface.value).to(equal('NOT_READY'))

        with it('should remain not ready when unrecognized event'):
            expect(self.request.superstate.state).to(equal('base:not_ready'))
            expect(self.request.interface.value).to(equal('NOT_READY'))
            self.request.superstate.unavailable()
            expect(self.request.superstate.state).to(equal('base:not_ready'))
            self.request.superstate.ready()
            expect(self.request.superstate.state).to(equal('base:not_ready'))
            self.request.superstate.active()
            expect(self.request.superstate.state).to(equal('base:not_ready'))     
            
        with it('should become ready when idle'):
            self.request.superstate.idle()
            expect(self.request.superstate.state).to(equal('base:ready'))
            expect(self.request.interface.value).to(equal('READY'))

        with it('should become active when activate'):
            self.request.superstate.activate()
            expect(self.request.superstate.state).to(equal('base:active'))
            expect(self.request.interface.value).to(equal('ACTIVE'))

        with it('should become active from ready when activate'):
            self.request.superstate.idle()
            self.request.superstate.activate()
            expect(self.request.superstate.state).to(equal('base:active'))
            expect(self.request.interface.value).to(equal('ACTIVE'))

        with it('should become ready from active when idle'):
            self.request.superstate.activate()
            self.request.superstate.idle()
            expect(self.request.superstate.state).to(equal('base:ready'))
            expect(self.request.interface.value).to(equal('READY'))

        with it('should become fail from active when failure'):
            self.request.superstate.activate()
            self.request.superstate.failure()
            expect(self.request.superstate.state).to(equal('base:fail'))
            expect(self.request.interface.value).to(equal('FAIL'))

        with it('should become active from ready when bot is ready'):
            self.bot_interface="READY"
            self.request.superstate.idle()
            #We could probably add this 'if' statement as a method in request!?
            if self.bot_interface=="READY":
                self.request.superstate.ready()
                expect(self.request.superstate.state).to(equal('base:active'))
                expect(self.request.interface.value).to(equal('ACTIVE'))

        with it('should become ready from active when bot is not ready'):
            self.bot_interface="NOT_READY"
            self.request.superstate.activate()
            #We could probably add this 'if' statement as a method in request!?
            if self.bot_interface=="NOT_READY":
                self.request.superstate.not_ready()
                expect(self.request.superstate.state).to(equal('base:ready'))
                expect(self.request.interface.value).to(equal('READY'))

        with it('should become processing from active when bot is active'):
            self.bot_interface="ACTIVE"
            self.request.superstate.activate()
            #We could probably add this 'if' statement as a method in request!?
            if self.bot_interface=="ACTIVE":
                self.request.superstate.active()
                expect(self.request.superstate.state).to(equal('base:processing'))
                
        with it('should become fail from processing when unrecognized event'):
            self.bot_interface="ACTIVE"
            self.request.superstate.activate()
            #We could probably add this 'if' statement as a method in request!?
            if self.bot_interface=="ACTIVE":
                self.request.superstate.active()
            expect(self.request.superstate.state).to(equal('base:processing'))
            self.request.superstate.IDLE()
            time.sleep(0.100)
            expect(self.request.superstate.state).to(equal('base:fail'))
            expect(self.request.interface.value).to(equal('FAIL'))

        with it('should become fail from processing when timeout'):
            self.bot_interface="ACTIVE"
            self.request.superstate.activate()
            #We could probably add this 'if' statement as a method in request!?
            if self.bot_interface=="ACTIVE":
                self.request.superstate.active()
                expect(self.request.superstate.state).to(equal('base:processing'))
                time.sleep(2.100)
                expect(self.request.superstate.state).to(equal('base:fail'))
                expect(self.request.interface.value).to(equal('FAIL'))

        with it('should become ready from fail when unrecognized event'):
            self.bot_interface="ACTIVE"
            self.request.superstate.activate()
            self.request.superstate.failure()
            expect(self.request.superstate.state).to(equal('base:fail'))
            expect(self.request.interface.value).to(equal('FAIL'))
            self.request.superstate.IDLE()
            time.sleep(0.100)
            expect(self.request.superstate.state).to(equal('base:ready'))
            expect(self.request.interface.value).to(equal('READY'))

        with it('should become ready from fail when timeout'):
            self.bot_interface="ACTIVE"
            self.request.superstate.activate()
            self.request.superstate.failure()
            time.sleep(2.100)
            expect(self.request.superstate.state).to(equal('base:ready'))
            expect(self.request.interface.value).to(equal('READY'))
                
         
