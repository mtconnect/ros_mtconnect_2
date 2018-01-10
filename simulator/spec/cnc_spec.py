from expects import *
from doublex import *

from src.cnc import *

with description('cnc'):
    with before.each:
        self.cnc = cnc(interface)

    with it('should have a cnc statemachine'):
        expect(self.cnc).to(be_a(cnc))

    with it('should have request interfaces'):
        expect(self.cnc.superstate.material_load_interface).to(be_a(Request))
        expect(self.cnc.superstate.material_unload_interface).to(be_a(Request))

    with it('should have response interfaces'):
        expect(self.cnc.superstate.open_chuck_interface).to(be_a(Response))
        expect(self.cnc.superstate.close_chuck_interface).to(be_a(Response))
        expect(self.cnc.superstate.open_door_interface).to(be_a(Response))
        expect(self.cnc.superstate.close_door_interface).to(be_a(Response))
    
    with it('should become ready when the link is enabled, all interfaces are ready, and the robot and machine tool are in automatic'):
        self.cnc.create_statemachine()
        self.cnc.superstate.has_material = False
        self.cnc.superstate.event('robot', 'Device', 'Availability', 'AVAILABLE')
        self.cnc.superstate.event('robot', 'Controller', 'ControllerMode', 'AUTOMATIC')
        self.cnc.superstate.event('robot', 'Controller', 'Execution', 'ACTIVE')
        self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
        self.cnc.superstate.material_load_interface.superstate.idle()
        expect(self.cnc.superstate.material_load_interface.superstate.state).to(equal('base:ready'))
        
        self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
        self.cnc.superstate.material_unload_interface.superstate.idle()
        expect(self.cnc.superstate.material_unload_interface.superstate.state).to(equal('base:ready'))

        self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
        expect(self.cnc.superstate.open_door_interface.superstate.state).to(equal('base:ready'))

        self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
        expect(self.cnc.superstate.close_door_interface.superstate.state).to(equal('base:ready'))

        self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
        expect(self.cnc.superstate.open_chuck_interface.superstate.state).to(equal('base:ready'))

        self.cnc.superstate.event('robot', 'ChuckInterface', 'Close', 'READY')
        expect(self.cnc.superstate.close_chuck_interface.superstate.state).to(equal('base:ready'))
        expect(self.cnc.superstate.close_chuck_interface.interface.value).to(equal('READY'))
    
    with context('when loading material'):
        
        with before.each:
            self.cnc.create_statemachine()
            self.cnc.superstate.has_material = False
            
            self.cnc.superstate.load_time_limit(4)
            self.cnc.superstate.event('robot', 'Device', 'Availability', 'AVAILABLE')
            self.cnc.superstate.event('robot', 'Controller', 'ControllerMode', 'AUTOMATIC')
            self.cnc.superstate.event('robot', 'Controller', 'Execution', 'ACTIVE')
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
            self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Close', 'READY')

        
        with it('should have the material load active'):
            expect(self.cnc.superstate.has_material).to(equal(False))
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            

        with it('should open door when open door becomes active'):
            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
            expect(self.cnc.superstate.open_door_interface.interface.value).to(equal('ACTIVE'))
            
            time.sleep(self.cnc.superstate.open_door_interface.superstate.simulated_duration+0.1)

            expect(self.cnc.superstate.open_door_interface.interface.value).to(equal('COMPLETE'))
            expect(self.cnc.superstate.open_door_interface.superstate.response_state).to(equal('OPEN'))

        with it('should close door when close door becomes active'):
            self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
            expect(self.cnc.superstate.close_door_interface.interface.value).to(equal('ACTIVE'))
            
            time.sleep(1.1)
            
            expect(self.cnc.superstate.close_door_interface.interface.value).to(equal('COMPLETE'))
            expect(self.cnc.superstate.close_door_interface.superstate.response_state).to(equal('CLOSED'))
        
        with it('should begin a part after material load is complete'):
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')
            
            time.sleep(1.1)

            expect(self.cnc.superstate.open_door_interface.superstate.response_state).to(equal('OPEN'))
            expect(self.cnc.superstate.open_chuck_interface.superstate.response_state).to(equal('OPEN'))

            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')

            self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')

            time.sleep(1.1)

            expect(self.cnc.superstate.close_door_interface.superstate.response_state).to(equal('CLOSED'))
            expect(self.cnc.superstate.close_chuck_interface.superstate.response_state).to(equal('CLOSED'))

            self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Close', 'READY')
            
            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))

            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')

            expect(self.cnc.superstate.state).to(equal('base:operational:cycle_start'))
            
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

            expect(self.cnc.superstate.has_material).to(equal(True))

            

            expect(self.cnc.superstate.material_load_interface.superstate.interface.value).to(equal('NOT_READY'))
            expect(self.cnc.superstate.material_unload_interface.superstate.interface.value).to(equal('NOT_READY'))

            time.sleep(2.1)

            expect(self.cnc.superstate.state).to(equal('base:operational:unloading'))

            expect(self.cnc.superstate.material_unload_interface.superstate.interface.value).to(equal('ACTIVE'))
            expect(self.cnc.superstate.material_load_interface.superstate.interface.value).to(equal('NOT_READY'))
        
        with it('should unload when after the machine has cut a part'):
            self.cnc.statemachine.set_state('base:activated')
            self.cnc.superstate.has_material = True
            self.cnc.superstate.make_operational()
            
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))
            expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('ACTIVE'))

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')

            time.sleep(1.1)

            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')

            expect(self.cnc.superstate.open_door_interface.superstate.response_state).to(equal('OPEN'))

            expect(self.cnc.superstate.open_chuck_interface.superstate.response_state).to(equal('OPEN'))

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'COMPLETE')
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')

            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))

            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))
            
        
                   
        with it('should be not ready when machine goes into manual mode'):
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            self.cnc.superstate.event('robot', 'Controller', 'ControllerMode', 'MANUAL')
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))
            expect(self.cnc.superstate.state).to(equal('base:disabled:not_ready'))

        with it('should be not ready when the execution state becomes ready'):
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            self.cnc.superstate.event('robot', 'Controller', 'Execution', 'READY')
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))
            expect(self.cnc.superstate.state).to(equal('base:disabled:not_ready'))

        with it('should be not ready when a fault occurs'):
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            self.cnc.superstate.event('robot', 'Device', 'SYSTEM', 'Fault', '1', 'failure')
            expect(self.cnc.superstate.state).to(equal('base:disabled:fault'))
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))
    
        with it('should be ready after a fault clears'):
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            self.cnc.superstate.event('robot', 'Device', 'SYSTEM', 'Fault', '1', 'failure')
            expect(self.cnc.superstate.state).to(equal('base:disabled:fault'))
            
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))
            self.cnc.superstate.event('robot', 'Device', 'SYSTEM', 'Normal')
            
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))


        #later: single faults clear
        
        with it('should fail if the chuck is open when it tries to cycle start'):
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
            self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
            time.sleep(1.2)
            expect(self.cnc.superstate.door_state).to(equal('CLOSED'))
            self.cnc.superstate.event('cnc', 'Rotary', 'ChuckState', 'OPEN')
            expect(self.cnc.superstate.chuck_state).to(equal('OPEN'))
            self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

            expect(self.cnc.superstate.state).to(equal('base:disabled:fault'))
        

        with it('should fail if the door is open when it tries to cycle start'):
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')
            time.sleep(1.2)
            self.cnc.superstate.event('cnc', 'Rotary', 'ChuckState', 'CLOSED')
            
            expect(self.cnc.superstate.chuck_state).to(equal('CLOSED'))
            expect(self.cnc.superstate.door_state).to(equal('OPEN'))

            self.cnc.superstate.event('robot', 'ChuckInterface', 'Close', 'READY')
            
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

            expect(self.cnc.superstate.state).to(equal('base:disabled:fault'))

        
        #review this test: fail to ready transition at times too fast to test. why? timers?
        with it('should fail a material load if the robot fails'):
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
            self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')
            time.sleep(1.2)

            expect(self.cnc.superstate.chuck_state).to(equal('CLOSED'))
            expect(self.cnc.superstate.door_state).to(equal('CLOSED'))

            self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Close', 'READY')

            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))
            #print self.cnc.superstate.material_load_interface.interface.value
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'FAIL')
            #print self.cnc.superstate.material_load_interface.interface.value
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('FAIL'))
            #print self.cnc.superstate.material_load_interface.interface.value
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')
            
            expect(self.cnc.superstate.state).to(equal('base:operational:idle'))
            
            time.sleep(0.1)
            
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('READY'))
            expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('READY'))

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
            time.sleep(0.1)
            expect(self.cnc.superstate.state).to(equal('base:operational:unloading'))

            expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('ACTIVE'))
        
        with it('should fail a material unload if the robot fails and return to active when ready'):
            self.cnc.statemachine.set_state('base:activated')
            self.cnc.superstate.has_material = True
            self.cnc.superstate.make_operational()
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'FAIL')
            expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('FAIL'))

            time.sleep(2.2)
            
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
            expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('ACTIVE'))
            expect(self.cnc.superstate.state).to(equal('base:operational:unloading'))

        
        with it('should fail if the load active does not complete in a certain amount of time'):
            self.cnc.superstate.load_time_limit(1)
            
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')

            time.sleep(1.5)
            
            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('FAIL'))

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'DEFAULT')

        
        with it('should fail if the unload active does not complete in a certain amount of time'):
            self.cnc.statemachine.set_state('base:activated')
            self.cnc.superstate.has_material = True
            self.cnc.superstate.make_operational()
            
            self.cnc.superstate.unload_time_limit(1)

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')

            time.sleep(1.2)
            
            expect(self.cnc.superstate.state).to(equal('base:operational:unloading'))
            expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('FAIL'))

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'DEFAULT')
        
        with it('should fault if the load fail is not resolved in a certain amount of time'):
            self.cnc.superstate.load_time_limit(1)
            self.cnc.superstate.load_failed_time_limit(1)

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
            time.sleep(1.2)

            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('FAIL'))

            time.sleep(1.2)

            expect(self.cnc.superstate.state).to(equal('base:operational:idle'))
        
        with it('should not fail a load if unload becomes not ready'):
            self.cnc.superstate.load_time_limit(1)

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
            
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'NOT_READY')

            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))
        
        with it('should fail execution if fail next is true'):
            self.cnc.superstate.fail_next = True

            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))

            self.cnc.superstate.load_time_limit(4)

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')

            time.sleep(1.2)

            self.cnc.superstate.event('cnc','Rotary', 'ChuckState', 'OPEN')

            expect(self.cnc.superstate.door_state).to(equal('OPEN'))
            expect(self.cnc.superstate.chuck_state).to(equal('OPEN'))
            
            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
            
            self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')

            time.sleep(1.2)

            self.cnc.superstate.event('cnc','Rotary', 'ChuckState', 'CLOSED')

            expect(self.cnc.superstate.door_state).to(equal('CLOSED'))
            expect(self.cnc.superstate.chuck_state).to(equal('CLOSED'))
            
            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')

            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))

            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

            expect(self.cnc.superstate.has_material).to(equal(True))

            expect(self.cnc.superstate.state).to(equal('base:disabled:fault'))
        
        with it('should not fail a unload if load becomes not ready'):
            self.cnc.statemachine.set_state('base:activated')
            self.cnc.superstate.has_material = True
            self.cnc.superstate.make_operational()

            self.cnc.superstate.unload_time_limit(1)
            self.cnc.superstate.unload_failed_time_limit(0)
            
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
            
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')

            expect(self.cnc.superstate.state).to(equal('base:operational:unloading'))
        
        with it('should be operational when material load is not ready (out of material) but can still unload'):
            self.cnc.statemachine.set_state('base:activated')
            self.cnc.superstate.has_material = True
            self.cnc.superstate.make_operational()

            self.cnc.superstate.unload_time_limit(2)
            self.cnc.superstate.unload_failed_time_limit(0)

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
            expect(self.cnc.superstate.state).to(equal('base:operational:unloading'))

            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')

            time.sleep(1.2)

            self.cnc.superstate.event('cnc','Rotary', 'ChuckState', 'OPEN')

            expect(self.cnc.superstate.door_state).to(equal('OPEN'))
            expect(self.cnc.superstate.chuck_state).to(equal('OPEN'))
            
            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'COMPLETE')
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')

            expect(self.cnc.superstate.has_material).to(equal(False))

            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))
        
        with context('and robot is out of material'):
            
            with it('should make material load not ready when it is active and the robot material load is not ready'):
                expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
                self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')
                expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('READY'))
            
            with it('should keep material unload ready when it is ready and the robot material load is not ready'):
                expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))

                self.cnc.superstate.load_time_limit(2)
                self.cnc.superstate.load_failed_time_limit(0)
                self.cnc.superstate.cycle_time = 1

                self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')

                self.cnc.superstate.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')
                self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
                time.sleep(1.2)
                
                self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
                
                self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
                
                time.sleep(1.2)
                
                expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('ACTIVE'))

                self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')

                expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))

                expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('ACTIVE'))
            
            with it('should fail if material load is not ready and current state is material load'):

                self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
                self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'FAIL')

                #a quick transition from fail to ready??
                expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('FAIL'))

                expect(self.cnc.superstate.has_material).to(equal(False))

                expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))

                self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
                time.sleep(0.1)
                expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))
                expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            
            with it('should go back to loading material if it is not available to the robot was reset or filled'):
                self.cnc.superstate.load_failed_time_limit(1)
                
                #print self.cnc.superstate.material_load_interface.superstate.state
                self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
                self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'FAIL')
                time.sleep(0.2)
                expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('FAIL'))
                time.sleep(1.2)
                
                print self.cnc.superstate.material_load_interface.superstate.state
                print self.cnc.superstate.material_load_interface.interface.value
                #expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('READY'))
                expect(self.cnc.superstate.state).to(equal('base:operational:idle'))
                
                expect(self.cnc.superstate.has_material).to(equal(False))

                expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))

                self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')
                expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))
     
                self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')                                     
                
                expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))
                expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            

                
