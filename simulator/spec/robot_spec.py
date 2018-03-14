from expects import expect, be_a, equal

from src.event import Event
from src.request import Request
from src.response import Response
from src.robot import Robot

with description('Robot testing:'):
    with before.all:
        self.robot = Robot()

    with it('should have request interfaces'):
        expect(self.robot.superstate.open_chuck_interface).to(be_a(Request))
        expect(self.robot.superstate.close_chuck_interface).to(be_a(Request))
        expect(self.robot.superstate.open_door_interface).to(be_a(Request))
        expect(self.robot.superstate.close_door_interface).to(be_a(Request))

    with it('should have response interfaces'):
        expect(self.robot.superstate.material_load_interface).to(be_a(Response))
        expect(self.robot.superstate.material_unload_interface).to(be_a(Response))

    with it('should become ready when the link is enabled, all interfaces are ready, and the robot and machine tool are in automatic'):
        self.robot.superstate.event(Event('cnc', 'Device', 'Availability', 'AVAILABLE'))
        self.robot.superstate.event(Event('cnc', 'Controller', 'ControllerMode', 'AUTOMATIC'))
        self.robot.superstate.event(Event('cnc', 'Controller', 'Execution', 'ACTIVE'))
        self.robot.superstate.event(Event('cnc', 'MaterialHandlerInterface', 'MaterialLoad', 'READY'))
        self.robot.superstate.material_load_interface.superstate.ready()
        expect(self.robot.superstate.material_load_interface.superstate.state).to(equal('base:ready'))

        self.robot.superstate.event(Event('cnc', 'MaterialHandlerInterface', 'MaterialUnload', 'READY'))
        self.robot.superstate.material_unload_interface.superstate.ready()
        expect(self.robot.superstate.material_unload_interface.superstate.state).to(equal('base:ready'))

        self.robot.superstate.event(Event('cnc', 'DoorInterface', 'DoorState', 'READY'))
        expect(self.robot.superstate.open_door_interface.superstate.state).to(equal('base:ready'))

        self.robot.superstate.event(Event('cnc', 'DoorInterface', 'Close', 'READY'))
        expect(self.robot.superstate.close_door_interface.superstate.state).to(equal('base:ready'))

        self.robot.superstate.event(Event('cnc', 'ChuckInterface', 'Open', 'READY'))
        expect(self.robot.superstate.open_chuck_interface.superstate.state).to(equal('base:ready'))

        self.robot.superstate.event(Event('cnc', 'ChuckInterface', 'Close', 'READY'))
        expect(self.robot.superstate.close_chuck_interface.superstate.state).to(equal('base:ready'))

#    with context('when loading material'):
#
#        with before.each:
#            self.robot.create_statemachine()
#            self.robot.superstate.has_material = False
#
#            self.robot.superstate.load_time_limit(4)
#            self.robot.superstate.event('robot', 'Device', 'Availability', 'AVAILABLE')
#            self.robot.superstate.event('robot', 'Controller', 'ControllerMode', 'AUTOMATIC')
#            self.robot.superstate.event('robot', 'Controller', 'Execution', 'ACTIVE')
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
#            self.robot.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
#            self.robot.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Close', 'READY')
#
#
#        with it('should have the material load active'):
#            expect(self.robot.superstate.has_material).to(equal(False))
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
#
#
#        with it('should open door when open door becomes active'):
#            self.robot.superstate.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
#            expect(self.robot.superstate.open_door_interface.interface.value).to(equal('ACTIVE'))
#
#            time.sleep(self.robot.superstate.open_door_interface.superstate.simulated_duration+0.1)
#
#            expect(self.robot.superstate.open_door_interface.interface.value).to(equal('COMPLETE'))
#            expect(self.robot.superstate.open_door_interface.superstate.response_state).to(equal('OPEN'))
#
#        with it('should close door when close door becomes active'):
#            self.robot.superstate.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
#            expect(self.robot.superstate.close_door_interface.interface.value).to(equal('ACTIVE'))
#
#            time.sleep(1.1)
#
#            expect(self.robot.superstate.close_door_interface.interface.value).to(equal('COMPLETE'))
#            expect(self.robot.superstate.close_door_interface.superstate.response_state).to(equal('CLOSED'))
#
#        with it('should begin a part after material load is complete'):
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
#            self.robot.superstate.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')
#
#            time.sleep(1.1)
#
#            expect(self.robot.superstate.open_door_interface.superstate.response_state).to(equal('OPEN'))
#            expect(self.robot.superstate.open_chuck_interface.superstate.response_state).to(equal('OPEN'))
#
#            self.robot.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
#
#            self.robot.superstate.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')
#
#            time.sleep(1.1)
#
#            expect(self.robot.superstate.close_door_interface.superstate.response_state).to(equal('CLOSED'))
#            expect(self.robot.superstate.close_chuck_interface.superstate.response_state).to(equal('CLOSED'))
#
#            self.robot.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Close', 'READY')
#
#            expect(self.robot.superstate.state).to(equal('base:operational:loading'))
#
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
#
#            expect(self.robot.superstate.state).to(equal('base:operational:cycle_start'))
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
#
#            expect(self.robot.superstate.has_material).to(equal(True))
#
#
#
#            expect(self.robot.superstate.material_load_interface.superstate.interface.value).to(equal('NOT_READY'))
#            expect(self.robot.superstate.material_unload_interface.superstate.interface.value).to(equal('NOT_READY'))
#
#            time.sleep(2.1)
#
#            expect(self.robot.superstate.state).to(equal('base:operational:unloading'))
#
#            expect(self.robot.superstate.material_unload_interface.superstate.interface.value).to(equal('ACTIVE'))
#            expect(self.robot.superstate.material_load_interface.superstate.interface.value).to(equal('NOT_READY'))
#
#        with it('should unload when after the machine has cut a part'):
#            self.robot.statemachine.set_state('base:activated')
#            self.robot.superstate.has_material = True
#            self.robot.superstate.make_operational()
#
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))
#            expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('ACTIVE'))
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
#            self.robot.superstate.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')
#
#            time.sleep(1.1)
#
#            self.robot.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
#
#            expect(self.robot.superstate.open_door_interface.superstate.response_state).to(equal('OPEN'))
#
#            expect(self.robot.superstate.open_chuck_interface.superstate.response_state).to(equal('OPEN'))
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'COMPLETE')
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
#
#            expect(self.robot.superstate.state).to(equal('base:operational:loading'))
#
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
#            expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))
#
#
#
#        with it('should be not ready when machine goes into manual mode'):
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
#            self.robot.superstate.event('robot', 'Controller', 'ControllerMode', 'MANUAL')
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))
#            expect(self.robot.superstate.state).to(equal('base:disabled:not_ready'))
#
#        with it('should be not ready when the execution state becomes ready'):
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
#            self.robot.superstate.event('robot', 'Controller', 'Execution', 'READY')
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))
#            expect(self.robot.superstate.state).to(equal('base:disabled:not_ready'))
#
#        with it('should be not ready when a fault occurs'):
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
#            self.robot.superstate.event('robot', 'Device', 'SYSTEM', 'Fault', '1', 'failure')
#            expect(self.robot.superstate.state).to(equal('base:disabled:fault'))
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))
#
#        with it('should be ready after a fault clears'):
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
#            self.robot.superstate.event('robot', 'Device', 'SYSTEM', 'Fault', '1', 'failure')
#            expect(self.robot.superstate.state).to(equal('base:disabled:fault'))
#
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))
#            self.robot.superstate.event('robot', 'Device', 'SYSTEM', 'Normal')
#
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
#            expect(self.robot.superstate.state).to(equal('base:operational:loading'))
#
#
#        #later: single faults clear
#
#        with it('should fail if the chuck is open when it tries to cycle start'):
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
#            self.robot.superstate.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
#            time.sleep(1.2)
#            expect(self.robot.superstate.door_state).to(equal('CLOSED'))
#            self.robot.superstate.event('cnc', 'Rotary', 'ChuckState', 'OPEN')
#            expect(self.robot.superstate.chuck_state).to(equal('OPEN'))
#            self.robot.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
#
#            expect(self.robot.superstate.state).to(equal('base:disabled:fault'))
#
#
#        with it('should fail if the door is open when it tries to cycle start'):
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')
#            time.sleep(1.2)
#            self.robot.superstate.event('cnc', 'Rotary', 'ChuckState', 'CLOSED')
#
#            expect(self.robot.superstate.chuck_state).to(equal('CLOSED'))
#            expect(self.robot.superstate.door_state).to(equal('OPEN'))
#
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Close', 'READY')
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
#
#            expect(self.robot.superstate.state).to(equal('base:disabled:fault'))
#
#
#
#        with it('should fail a material load if the robot fails'):
#            self.robot.superstate.load_failed_time_limit(1)
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
#            self.robot.superstate.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')
#            time.sleep(1.2)
#
#            expect(self.robot.superstate.chuck_state).to(equal('CLOSED'))
#            expect(self.robot.superstate.door_state).to(equal('CLOSED'))
#
#            self.robot.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Close', 'READY')
#
#            expect(self.robot.superstate.state).to(equal('base:operational:loading'))
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'FAIL')
#
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('FAIL'))
#
#            time.sleep(1.2)
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')
#
#            expect(self.robot.superstate.state).to(equal('base:operational:idle'))
#
#            time.sleep(0.1)
#
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('READY'))
#            expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('READY'))
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
#            time.sleep(0.1)
#            expect(self.robot.superstate.state).to(equal('base:operational:unloading'))
#
#            expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('ACTIVE'))
#
#        with it('should fail a material unload if the robot fails and return to active when ready'):
#            self.robot.statemachine.set_state('base:activated')
#            self.robot.superstate.has_material = True
#            self.robot.superstate.make_operational()
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'FAIL')
#            expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('FAIL'))
#
#            time.sleep(2.2)
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
#            expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('ACTIVE'))
#            expect(self.robot.superstate.state).to(equal('base:operational:unloading'))
#
#
#        with it('should fail if the load active does not complete in a certain amount of time'):
#            self.robot.superstate.load_time_limit(1)
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
#
#            time.sleep(1.5)
#
#            expect(self.robot.superstate.state).to(equal('base:operational:loading'))
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('FAIL'))
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'DEFAULT')
#
#
#        with it('should fail if the unload active does not complete in a certain amount of time'):
#            self.robot.statemachine.set_state('base:activated')
#            self.robot.superstate.has_material = True
#            self.robot.superstate.make_operational()
#
#            self.robot.superstate.unload_time_limit(1)
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
#
#            time.sleep(1.2)
#
#            expect(self.robot.superstate.state).to(equal('base:operational:unloading'))
#            expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('FAIL'))
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'DEFAULT')
#
#        with it('should fault if the load fail is not resolved in a certain amount of time'):
#            self.robot.superstate.load_time_limit(1)
#            self.robot.superstate.load_failed_time_limit(1)
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
#            time.sleep(1.2)
#
#            expect(self.robot.superstate.state).to(equal('base:operational:loading'))
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('FAIL'))
#
#            time.sleep(1.2)
#
#            expect(self.robot.superstate.state).to(equal('base:operational:idle'))
#
#        with it('should not fail a load if unload becomes not ready'):
#            self.robot.superstate.load_time_limit(1)
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'NOT_READY')
#
#            expect(self.robot.superstate.state).to(equal('base:operational:loading'))
#
#        with it('should fail execution if fail next is true'):
#            self.robot.superstate.fail_next = True
#
#            expect(self.robot.superstate.state).to(equal('base:operational:loading'))
#
#            self.robot.superstate.load_time_limit(4)
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
#            self.robot.superstate.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')
#
#            time.sleep(1.2)
#
#            self.robot.superstate.event('cnc','Rotary', 'ChuckState', 'OPEN')
#
#            expect(self.robot.superstate.door_state).to(equal('OPEN'))
#            expect(self.robot.superstate.chuck_state).to(equal('OPEN'))
#
#            self.robot.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
#
#            self.robot.superstate.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')
#
#            time.sleep(1.2)
#
#            self.robot.superstate.event('cnc','Rotary', 'ChuckState', 'CLOSED')
#
#            expect(self.robot.superstate.door_state).to(equal('CLOSED'))
#            expect(self.robot.superstate.chuck_state).to(equal('CLOSED'))
#
#            self.robot.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
#
#            expect(self.robot.superstate.state).to(equal('base:operational:loading'))
#
#            expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
#
#            expect(self.robot.superstate.has_material).to(equal(True))
#
#            expect(self.robot.superstate.state).to(equal('base:disabled:fault'))
#
#        with it('should not fail a unload if load becomes not ready'):
#            self.robot.statemachine.set_state('base:activated')
#            self.robot.superstate.has_material = True
#            self.robot.superstate.make_operational()
#
#            self.robot.superstate.unload_time_limit(1)
#            self.robot.superstate.unload_failed_time_limit(0)
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')
#
#            expect(self.robot.superstate.state).to(equal('base:operational:unloading'))
#
#        with it('should be operational when material load is not ready (out of material) but can still unload'):
#            self.robot.statemachine.set_state('base:activated')
#            self.robot.superstate.has_material = True
#            self.robot.superstate.make_operational()
#
#            self.robot.superstate.unload_time_limit(2)
#            self.robot.superstate.unload_failed_time_limit(0)
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
#            expect(self.robot.superstate.state).to(equal('base:operational:unloading'))
#
#            self.robot.superstate.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')
#
#            time.sleep(1.2)
#
#            self.robot.superstate.event('cnc','Rotary', 'ChuckState', 'OPEN')
#
#            expect(self.robot.superstate.door_state).to(equal('OPEN'))
#            expect(self.robot.superstate.chuck_state).to(equal('OPEN'))
#
#            self.robot.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
#            self.robot.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
#
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'COMPLETE')
#            self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
#
#            expect(self.robot.superstate.has_material).to(equal(False))
#
#            expect(self.robot.superstate.state).to(equal('base:operational:loading'))
#
#
#        with context('and robot is out of material'):
#
#            with it('should make material load not ready when it is active and the robot material load is not ready'):
#                expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
#                self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')
#                expect(self.robot.superstate.material_load_interface.interface.value).to(equal('READY'))
#
#            with it('should keep material unload ready when it is ready and the robot material load is not ready'):
#                expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
#
#                self.robot.superstate.load_time_limit(2)
#                self.robot.superstate.load_failed_time_limit(0)
#                self.robot.superstate.cycle_time = 1
#
#                self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
#
#                self.robot.superstate.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')
#                self.robot.superstate.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
#                time.sleep(1.2)
#
#                self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
#
#                self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
#
#                time.sleep(1.2)
#
#                expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('ACTIVE'))
#
#                self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')
#
#                expect(self.robot.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))
#
#                expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('ACTIVE'))
#
#            with it('should fail if material load is not ready and current state is material load'):
#
#                self.robot.superstate.load_failed_time_limit(1)
#                self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
#                self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'FAIL')
#
#                expect(self.robot.superstate.material_load_interface.interface.value).to(equal('FAIL'))
#
#                expect(self.robot.superstate.has_material).to(equal(False))
#
#                expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))
#                time.sleep(1.1)
#                self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
#                time.sleep(0.1)
#                expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))
#                expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
#
#            with it('should go back to loading material if it is not available to the robot was reset or filled'):
#
#                self.robot.superstate.load_failed_time_limit(1)
#
#                self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')
#                expect(self.robot.superstate.material_load_interface.superstate.state).to(equal('base:processing'))
#                self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'FAIL')
#                time.sleep(0.2)
#                expect(self.robot.superstate.material_load_interface.superstate.state).to(equal('base:fail'))
#                expect(self.robot.superstate.material_load_interface.interface.value).to(equal('FAIL'))
#
#                time.sleep(1.2)
#
#                expect(self.robot.superstate.material_load_interface.interface.value).to(equal('READY'))
#                expect(self.robot.superstate.state).to(equal('base:operational:idle'))
#
#                expect(self.robot.superstate.has_material).to(equal(False))
#
#                expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))
#
#                self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'NOT_READY')
#                expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))
#
#                self.robot.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
#
#                expect(self.robot.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))
#                expect(self.robot.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
