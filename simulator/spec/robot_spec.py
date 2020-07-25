import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from expects import expect, be_a, equal

from src.interfaces.request import Request
from src.interfaces.response import Response
from src.robot import Robot

from mock import Mock

RobotInterface =Mock()
RobotInterface().move_in =Mock(return_value = True)
RobotInterface().move_out =Mock(return_value = True)
RobotInterface().grab =Mock(return_value = True)
RobotInterface().release =Mock(return_value = True)

Robot.StateMachineModel.initiate_pull_thread = Mock()

from src.collaborationModel.archetypeToInstance import archetypeToInstance

import time, threading

with description('Robot'):

    with context ('state'):

        with before.all:
            self.robot = Robot('localhost',7590, RobotInterface(),True)

        with it('should be in binding state INACTIVE initially'):
            expect(self.robot.superstate.binding_state_material.value()).to(equal('INACTIVE'))

        with it('should have request interfaces'):
            expect(self.robot.superstate.open_chuck_interface).to(be_a(Request))
            expect(self.robot.superstate.close_chuck_interface).to(be_a(Request))
            expect(self.robot.superstate.open_door_interface).to(be_a(Request))
            expect(self.robot.superstate.close_door_interface).to(be_a(Request))

        with it('should have response interfaces'):
            expect(self.robot.superstate.material_load_interface).to(be_a(Response))
            expect(self.robot.superstate.material_unload_interface).to(be_a(Response))

        with it('should be in execution ready when the robot is initialized'):
            expect(self.robot.superstate.e1.value()).to(equal('READY'))

        with it('should be in controller mode automatic when the robot is initialized'):
            expect(self.robot.superstate.mode1.value()).to(equal('AUTOMATIC'))


    with context('move material from conv to cnc'):

        with before.all:
            self.robot = None
            self.robot = Robot('localhost',7591, RobotInterface(),True)
            self.robot.superstate.priority.collab_check = Mock()
            self.robot.superstate.material_load_interface.superstate.simulated_duration = 600
            self.robot.superstate.material_unload_interface.superstate.simulated_duration = 600
            self.robot.superstate.master_uuid = '1'
            self.robot.superstate.master_tasks['1'] = archetypeToInstance('MoveMaterial_1_good','1','r1').jsonInstance()
            self.robot.superstate.enable()


        with it ('should complete successfully'):

            self.robot.superstate.event('conv','Coordinator', 'binding_state', 'PREPARING',['1', self.robot.superstate.master_tasks['1']],'conv1')

            expect(self.robot.superstate.binding_state_material.value()).to(equal('PREPARING'))

            self.robot.superstate.event('conv','Coordinator', 'binding_state', 'COMMITTING','1','conv1')

            expect(self.robot.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            self.robot.superstate.event('conv','Coordinator', 'binding_state', 'COMMITTED','1','conv1')

            expect(self.robot.superstate.material_unload.value()).to(equal('READY'))

            self.robot.superstate.event('conv','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE','1','conv1')

            expect(self.robot.superstate.material_unload.value()).to(equal('NOT_READY'))

            self.robot.superstate.event('cnc','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE','1','cnc1')
            expect(self.robot.superstate.material_load.value()).to(equal('ACTIVE'))

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_OpenDoor', 'READY','1','cnc1')

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_OpenDoor', 'ACTIVE','1','cnc1')

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_OpenDoor', 'COMPLETE','1','cnc1')


            self.robot.superstate.event('cnc','ChuckInterface', 'SubTask_CloseChuck', 'READY','1','cnc1')


            self.robot.superstate.event('cnc','ChuckInterface', 'SubTask_CloseChuck', 'ACTIVE','1','cnc1')
            expect(self.robot.superstate.close_chuck.value()).to(equal('ACTIVE'))

            self.robot.superstate.event('cnc','ChuckInterface', 'SubTask_CloseChuck', 'COMPLETE','1','cnc1')

            expect(self.robot.superstate.close_chuck.value()).to(equal('NOT_READY'))

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'READY','1','cnc1')

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'ACTIVE','1','cnc1')
            expect(self.robot.superstate.close_door.value()).to(equal('ACTIVE'))

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'COMPLETE','1','cnc1')
            expect(self.robot.superstate.close_door.value()).to(equal('NOT_READY'))

            expect(self.robot.superstate.material_load.value()).to(equal('NOT_READY'))

            expect(self.robot.superstate.binding_state_material.value()).to(equal('INACTIVE'))

    with context('move material from cnc to buffer'):

        with before.all:
            self.robot = Robot('localhost',7592, RobotInterface(),True)
            self.robot.superstate.priority.collab_check = Mock()
            self.robot.superstate.material_load_interface.superstate.simulated_duration = 10
            self.robot.superstate.material_unload_interface.superstate.simulated_duration = 25
            self.robot.superstate.master_uuid = '1'
            self.robot.superstate.master_tasks['1'] = archetypeToInstance('MoveMaterial_2','1','r1').jsonInstance()
            self.robot.superstate.enable()


        with it ('should complete successfully'):

            self.robot.superstate.event('cnc','Coordinator', 'binding_state', 'PREPARING',['1', self.robot.superstate.master_tasks['1']],'cnc1')

            expect(self.robot.superstate.binding_state_material.value()).to(equal('PREPARING'))

            self.robot.superstate.event('cnc','Coordinator', 'binding_state', 'COMMITTING','1','cnc1')

            expect(self.robot.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            self.robot.superstate.event('cnc','Coordinator', 'binding_state', 'COMMITTED','1','cnc1')

            expect(self.robot.superstate.material_unload.value()).to(equal('READY'))

            self.robot.superstate.event('cnc','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE','1','cnc1')

            expect(self.robot.superstate.material_unload.value()).to(equal('ACTIVE'))


            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_OpenDoor', 'READY','1','cnc1')

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_OpenDoor', 'ACTIVE','1','cnc1')

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_OpenDoor', 'COMPLETE','1','cnc1')

            self.robot.superstate.event('cnc','ChuckInterface', 'SubTask_OpenChuck', 'READY','1','cnc1')

            self.robot.superstate.event('cnc','ChuckInterface', 'SubTask_OpenChuck', 'ACTIVE','1','cnc1')
            expect(self.robot.superstate.open_chuck.value()).to(equal('ACTIVE'))

            self.robot.superstate.event('cnc','ChuckInterface', 'SubTask_OpenChuck', 'COMPLETE','1','cnc1')

            expect(self.robot.superstate.open_chuck.value()).to(equal('NOT_READY'))

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'READY','1','cnc1')

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'ACTIVE','1','cnc1')
            expect(self.robot.superstate.close_door.value()).to(equal('ACTIVE'))

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'COMPLETE','1','cnc1')
            expect(self.robot.superstate.close_door.value()).to(equal('NOT_READY'))

            expect(self.robot.superstate.material_unload.value()).to(equal('NOT_READY'))

            expect(self.robot.superstate.state).to(equal('base:operational:loading'))

            self.robot.superstate.event('buffer','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE','1','b1')

            expect(self.robot.superstate.material_load.value()).to(equal('NOT_READY'))

            expect(self.robot.superstate.binding_state_material.value()).to(equal('INACTIVE'))


    with context('move material from cnc to buffer'):

        with before.all:
            self.robot = Robot('localhost',7593, RobotInterface(),True)
            self.robot.superstate.priority.collab_check = Mock()
            self.robot.superstate.material_load_interface.superstate.simulated_duration = 10
            self.robot.superstate.material_unload_interface.superstate.simulated_duration = 25
            self.robot.superstate.master_uuid = '1'
            self.robot.superstate.master_tasks['1'] = archetypeToInstance('MoveMaterial_4_bad','1','r1').jsonInstance()
            self.robot.superstate.enable()


        with it ('should complete successfully'):

            self.robot.superstate.event('cmm','Coordinator', 'binding_state', 'PREPARING',['1', self.robot.superstate.master_tasks['1']],'cmm1')

            expect(self.robot.superstate.binding_state_material.value()).to(equal('PREPARING'))

            self.robot.superstate.event('cmm','Coordinator', 'binding_state', 'COMMITTING','1','cmm1')

            expect(self.robot.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            self.robot.superstate.event('cmm','Coordinator', 'binding_state', 'COMMITTED','1','cmm1')

            expect(self.robot.superstate.material_unload.value()).to(equal('READY'))

            self.robot.superstate.event('cmm','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE','1','cmm1')
            expect(self.robot.superstate.material_unload.value()).to(equal('NOT_READY'))

            self.robot.superstate.event('conv','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE','1','conv1')

            expect(self.robot.superstate.material_load.value()).to(equal('NOT_READY'))

            expect(self.robot.superstate.open_door.value()).to(equal('READY'))

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_OpenDoor', 'READY','1','cnc1')
            expect(self.robot.superstate.open_door.value()).to(equal('ACTIVE'))

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_OpenDoor', 'ACTIVE','1','cnc1')
            expect(self.robot.superstate.open_door_interface.superstate.state).to(equal('base:processing'))

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_OpenDoor', 'COMPLETE','1','cnc1')
            expect(self.robot.superstate.open_door.value()).to(equal('NOT_READY'))


            expect(self.robot.superstate.change_tool.value()).to(equal('READY'))
            self.robot.superstate.event('cnc','ToolInterface', 'SubTask_ChangeTool', 'READY','1','cnc1')
            expect(self.robot.superstate.change_tool.value()).to(equal('ACTIVE'))

            self.robot.superstate.event('cnc','ToolInterface', 'SubTask_ChangeTool', 'ACTIVE','1','cnc1')
            expect(self.robot.superstate.change_tool_interface.superstate.state).to(equal('base:processing'))

            self.robot.superstate.event('cnc','ToolInterface', 'SubTask_ChangeTool', 'COMPLETE','1','cnc1')
            expect(self.robot.superstate.change_tool.value()).to(equal('NOT_READY'))



            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'READY','1','cnc1')

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'ACTIVE','1','cnc1')

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'COMPLETE','1','cnc1')
            expect(self.robot.superstate.close_door.value()).to(equal('NOT_READY'))


            expect(self.robot.superstate.binding_state_material.value()).to(equal('INACTIVE'))

