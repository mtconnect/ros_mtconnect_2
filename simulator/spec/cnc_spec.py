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
            
            expect(self.cnc.superstate.state).to(equal('operational:loading'))

            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')

            expect(self.cnc.superstate.state).to(equal('operational:cycle_start'))
            
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

            expect(self.cnc.superstate.has_material).to(equal(True))

            

            expect(self.cnc.superstate.material_load_interface.superstate.interface.value).to(equal('NOT_READY'))
            expect(self.cnc.superstate.material_unload_interface.superstate.interface.value).to(equal('NOT_READY'))

            time.sleep(2.1)

            expect(self.cnc.superstate.state).to(equal('operational:unloading'))

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

            expect(self.cnc.superstate.state).to(equal('operational:loading'))

            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))
            
        
            
        with it('should be not ready when machine goes into manual mode'):
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            self.cnc.superstate.event('robot', 'Controller', 'ControllerMode', 'MANUAL')
            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))
            
            
            

            

            

            

            
            

            
            
            
    
            

                   
