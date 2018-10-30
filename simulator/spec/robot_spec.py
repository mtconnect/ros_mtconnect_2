import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from expects import expect, be_a, equal

from interfaces.request import Request
from interfaces.response import Response
from robot import Robot
from robot_interface import RobotInterface

from src.collaborationModel.archetypeToInstance import archetypeToInstance

import time, threading

#make sure "self.initiate_pull_thread()" is commented out in the respective S/M
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
            self.robot = Robot('localhost',7591, RobotInterface(),True)
            self.robot.superstate.material_load_interface.superstate.simulated_duration = 25
            self.robot.superstate.material_unload_interface.superstate.simulated_duration = 10
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

            expect(self.robot.superstate.material_unload.value()).to(equal('ACTIVE'))

            time.sleep(11) #Simulation for unloading

            expect(self.robot.superstate.material_unload.value()).to(equal('NOT_READY'))

            self.robot.superstate.event('cnc','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE','1','cnc1')

            expect(self.robot.superstate.material_load.value()).to(equal('ACTIVE'))


            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_OpenDoor', 'READY','1','cnc1')

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_OpenDoor', 'ACTIVE','1','cnc1')

            time.sleep(2) #Simulation for open door

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_OpenDoor', 'COMPLETE','1','cnc1')

            time.sleep(4) #Simulation for internal task: moving in

            time.sleep(4) #simulation for internal task: release part

            self.robot.superstate.event('cnc','ChuckInterface', 'SubTask_CloseChuck', 'READY','1','cnc1')

            #expect(self.robot.superstate.close_chuck.value()).to(equal('ACTIVE'))

            self.robot.superstate.event('cnc','ChuckInterface', 'SubTask_CloseChuck', 'ACTIVE','1','cnc1')
            expect(self.robot.superstate.close_chuck.value()).to(equal('ACTIVE'))

            time.sleep(2) #Simulation for close chuck

            self.robot.superstate.event('cnc','ChuckInterface', 'SubTask_CloseChuck', 'COMPLETE','1','cnc1')

            expect(self.robot.superstate.close_chuck.value()).to(equal('NOT_READY'))

            time.sleep(6) #Simulation for internal task: move out

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'READY','1','cnc1')

            #expect(self.robot.superstate.close_door.value()).to(equal('ACTIVE'))

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'ACTIVE','1','cnc1')
            expect(self.robot.superstate.close_door.value()).to(equal('ACTIVE'))

            time.sleep(2) #Simulation for close door
            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'COMPLETE','1','cnc1')
            time.sleep(0.2)
            expect(self.robot.superstate.close_door.value()).to(equal('NOT_READY'))

            time.sleep(5) #Simulaiton for material load completion
            expect(self.robot.superstate.material_load.value()).to(equal('NOT_READY'))

            expect(self.robot.superstate.binding_state_material.value()).to(equal('INACTIVE'))

            self.robot.superstate.binding_state_material.set_value("UNAVAILABLE")


    with context('move material from cnc to buffer'):

        with before.all:
            self.robot = Robot('localhost',7592, RobotInterface(),True)
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

            time.sleep(2) #Simulation for open door

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_OpenDoor', 'COMPLETE','1','cnc1')

            time.sleep(4) #Simulation for internal task: moving in

            time.sleep(4) #simulation for internal task: grab part

            self.robot.superstate.event('cnc','ChuckInterface', 'SubTask_OpenChuck', 'READY','1','cnc1')

            self.robot.superstate.event('cnc','ChuckInterface', 'SubTask_OpenChuck', 'ACTIVE','1','cnc1')
            expect(self.robot.superstate.open_chuck.value()).to(equal('ACTIVE'))

            time.sleep(2) #Simulation for open chuck

            self.robot.superstate.event('cnc','ChuckInterface', 'SubTask_OpenChuck', 'COMPLETE','1','cnc1')

            expect(self.robot.superstate.open_chuck.value()).to(equal('NOT_READY'))

            time.sleep(6) #Simulation for internal task: move out


            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'READY','1','cnc1')

            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'ACTIVE','1','cnc1')
            expect(self.robot.superstate.close_door.value()).to(equal('ACTIVE'))

            time.sleep(2) #Simulation for close door
            self.robot.superstate.event('cnc','DoorInterface', 'SubTask_CloseDoor', 'COMPLETE','1','cnc1')
            time.sleep(0.2)
            expect(self.robot.superstate.close_door.value()).to(equal('NOT_READY'))

            time.sleep(5) #Simulaiton for material unload completion

            expect(self.robot.superstate.material_unload.value()).to(equal('NOT_READY'))

            expect(self.robot.superstate.state).to(equal('base:operational:loading'))

            self.robot.superstate.event('buffer','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE','1','b1')

            expect(self.robot.superstate.material_load.value()).to(equal('ACTIVE'))

            time.sleep(11) #Simulation for unloading

            expect(self.robot.superstate.material_load.value()).to(equal('NOT_READY'))

            expect(self.robot.superstate.binding_state_material.value()).to(equal('INACTIVE'))

            self.robot.superstate.binding_state_material.set_value("UNAVAILABLE")

