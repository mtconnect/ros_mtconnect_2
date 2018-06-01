from expects import expect, be_a, equal

from src.request import Request
from src.response import Response
from src.robot import Robot
import time

#make sure "self.initiate_pull_thread()" is commented out in robot.py
the_robot = Robot()

with description('Robot'):
    with before.all:
<<<<<<< HEAD
        self.robot = Robot('localhost',7900)
=======
        #TODO: whenever we create a new Robot the old one has to release the adapter connection
        #self.robot = Robot()
        pass
>>>>>>> a2a4df6a7b25d87d2f86dcc38d82ce3c532630c8

    with it('should be in binding state INACTIVE initially'):
        expect(the_robot.superstate.binding_state_material.value()).to(equal('INACTIVE'))

    with it('should have request interfaces'):
        expect(the_robot.superstate.open_chuck_interface).to(be_a(Request))
        expect(the_robot.superstate.close_chuck_interface).to(be_a(Request))
        expect(the_robot.superstate.open_door_interface).to(be_a(Request))
        expect(the_robot.superstate.close_door_interface).to(be_a(Request))

    with it('should have response interfaces'):
        expect(the_robot.superstate.material_load_interface).to(be_a(Response))
        expect(the_robot.superstate.material_unload_interface).to(be_a(Response))

    with it('should be in execution ready when the robot is initialized'):
        expect(the_robot.superstate.e1.value()).to(equal('READY'))

    with it('should be in controller mode automatic when the robot is initialized'):
        expect(the_robot.superstate.mode1.value()).to(equal('AUTOMATIC'))

    with context('move material from conv to cnc'):
        with before.all:
<<<<<<< HEAD
            self.robot.superstate.enable()
            self.robot.superstate.material_load_interface.superstate.simulated_duration = 4
            self.robot.superstate.material_unload_interface.superstate.simulated_duration = 4
            self.robot.superstate.master_tasks['1'] = {'coordinator': {'conv1': {'state': ['conveyor', 'conv1', None], 'Task': ['move_material', None], 'SubTask': {'conv1': ['UnloadConv', None, 'r1', 'MaterialUnload', '1'], 'r1': [], 'cnc1': ['LoadCnc', None, 'r1', 'MaterialLoad', '2']}}}, 'collaborators': {'r1': {'state': ['ROBOT', 'r1', None], 'SubTask': {'LoadCnc': [['Interface', 'CloseChuck', None, '1', None], ['Interface', 'CloseDoor', None, '2', None]]}}, 'cnc1': {'state': ['CNC', 'cnc1', None], 'SubTask': {}}}}
            self.robot.superstate.master_uuid = '1'
            
=======
            the_robot.superstate.enable()
            the_robot.superstate.material_load_interface.superstate.simulated_duration = 4
            the_robot.superstate.material_unload_interface.superstate.simulated_duration = 4
            the_robot.superstate.master_tasks['1'] = {
                'coordinator': {
                    'conv1': {
                        'state': ['conveyor', 'conv1', None],
                        'Task': ['move_material', None],
                        'SubTask': {
                            'conv1': ['UnloadConv', None, 'r1', 'MaterialUnload'],
                            'r1': [],
                            'cnc1': ['LoadCnc', None, 'r1', 'MaterialLoad']
                        }
                    }
                },
                'collaborators': {
                    'r1': {
                        'state': ['ROBOT', 'r1', None],
                        'SubTask': {
                            'LoadCnc': [
                                ['Interface', 'CloseChuck', None, '1', None],
                                ['Interface', 'CloseDoor', None, '2', None]
                            ]
                        }
                    },
                    'cnc1': {
                        'state': ['CNC', 'cnc1', None],
                        'SubTask': {}
                    }
                }
            }
            the_robot.superstate.master_uuid = '1'

>>>>>>> a2a4df6a7b25d87d2f86dcc38d82ce3c532630c8
        with it ('should complete successfully'):

            the_robot.superstate.event('conv','Coordinator', 'binding_state', 'PREPARING',['1', the_robot.superstate.master_tasks['1']],'conv1')
            time.sleep(0.2)

            expect(the_robot.superstate.binding_state_material.value()).to(equal('PREPARING'))

            the_robot.superstate.event('conv','Coordinator', 'binding_state', 'COMMITTING','1','conv1')
            expect(the_robot.superstate.binding_state_material.value()).to(equal('COMMITTED'))
            time.sleep(0.3)
<<<<<<< HEAD
            
            self.robot.superstate.event('conv','Coordinator', 'binding_state', 'COMMITTED','1','conv1')
            time.sleep(0.1)
=======

            the_robot.superstate.event('conv','BindingState', 'SubTask_binding_state', 'COMMITTED','1','conv1')
            time.sleep(0.6)
>>>>>>> a2a4df6a7b25d87d2f86dcc38d82ce3c532630c8

            expect(the_robot.superstate.material_unload.value()).to(equal('READY'))

            the_robot.superstate.event('conv','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE','1','conv1')

<<<<<<< HEAD
            self.robot.superstate.event('conv','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE','1','conv1')

            expect(self.robot.superstate.material_unload.value()).to(equal('ACTIVE'))
            time.sleep(5)

            #unload completes and state goes from active-complete-not_ready
            expect(self.robot.superstate.material_unload.value()).to(equal('NOT_READY'))
            
            time.sleep(1)
            self.robot.superstate.event('conv','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE','1','cnc1')

            expect(self.robot.superstate.material_load.value()).to(equal('ACTIVE'))
            
            time.sleep(5)
            expect(self.robot.superstate.material_load.value()).to(equal('COMPLETE'))

            self.robot.superstate.event('cnc','ChuckInterface', 'SubTask_CloseChuck', 'READY','1','cnc1')
=======
            expect(the_robot.superstate.material_unload.value()).to(equal('ACTIVE'))
            time.sleep(4)

            #unload completes and state goes from active-complete-not_ready
            expect(the_robot.superstate.material_unload.value()).to(equal('COMPLETE'))

            #the_robot.superstate.event('cnc','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE','1','cnc1')

            time.sleep(1)
            the_robot.superstate.event('conv','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE','1','cnc1')
            time.sleep(0.1)
            expect(the_robot.superstate.material_load.value()).to(equal('ACTIVE'))

            time.sleep(4)
            expect(the_robot.superstate.material_load.value()).to(equal('COMPLETE'))

            the_robot.superstate.event('cnc','ChuckInterface', 'SubTask_CloseChuck', 'READY','1','cnc1')
            print the_robot.superstate.collaborator.superstate.currentSubTask
>>>>>>> a2a4df6a7b25d87d2f86dcc38d82ce3c532630c8

            time.sleep(1)
            expect(the_robot.superstate.close_chuck.value()).to(equal('ACTIVE'))

            the_robot.superstate.event('cnc','ChuckInterface', 'SubTask_CloseChuck', 'ACTIVE','1','cnc1')
            time.sleep(0.2)
            expect(the_robot.superstate.close_chuck.value()).to(equal('ACTIVE'))

            the_robot.superstate.event('cnc','ChuckInterface', 'SubTask_CloseChuck', 'COMPLETE','1','cnc1')

            time.sleep(0.2)

            expect(the_robot.superstate.close_chuck.value()).to(equal('NOT_READY'))
            time.sleep(0.2)

            the_robot.superstate.event('cnc','ChuckInterface', 'SubTask_CloseDoor', 'READY','1','cnc1')
            time.sleep(1)
            expect(the_robot.superstate.close_door.value()).to(equal('ACTIVE'))


            the_robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'ACTIVE','1','cnc1')
            time.sleep(0.2)
            expect(the_robot.superstate.close_door.value()).to(equal('ACTIVE'))

            the_robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'COMPLETE','1','cnc1')
            time.sleep(0.2)
            expect(the_robot.superstate.close_door.value()).to(equal('NOT_READY'))


            time.sleep(0.2)

            expect(the_robot.superstate.material_load.value()).to(equal('NOT_READY'))

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
