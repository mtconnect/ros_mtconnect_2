import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from expects import expect, be_a, equal

from interfaces.request import Request
from interfaces.response import Response
from cnc import cnc
from collaborationModel.archetypeToInstance import archetypeToInstance

from threading import Thread
import time, threading

#make sure "self.initiate_pull_thread()" is commented out in the respective S/M
with description('CNC'):

    with context('state'):

        with before.all:
            self.cnc = cnc('localhost',7790)
            self.cnc.create_statemachine()

        with it('should be in binding state INACTIVE initially'):
            expect(self.cnc.superstate.binding_state_material.value()).to(equal('INACTIVE'))

        with it('should have request interfaces'):
            expect(self.cnc.superstate.material_load_interface).to(be_a(Request))
            expect(self.cnc.superstate.material_unload_interface).to(be_a(Request))

        with it('should be in execution ready when initialized'):
            expect(self.cnc.superstate.e1.value()).to(equal('READY'))

        with it('should be in controller mode automatic when initialized'):
            expect(self.cnc.superstate.mode1.value()).to(equal('AUTOMATIC'))


    with context('move material from conv to cnc'):

        with before.all:
            self.cnc = None
            self.cnc = cnc('localhost',7791)
            self.cnc.create_statemachine()
            self.cnc.superstate.load_time_limit(20)
            self.cnc.superstate.unload_time_limit(20)
            self.cnc.superstate.master_uuid = '1'
            self.cnc.superstate.master_tasks['1'] = archetypeToInstance('MoveMaterial_1_good','1','cnc1').jsonInstance()
            self.cnc.superstate.enable()
            time.sleep(1)

        with it ('should complete successfully'):

            self.cnc.superstate.event('conv','Coordinator', 'binding_state', 'PREPARING',['1', self.cnc.superstate.master_tasks['1']],'conv1')
            expect(self.cnc.superstate.binding_state_material.value()).to(equal('PREPARING'))

            self.cnc.superstate.event('conv','Coordinator', 'binding_state', 'COMMITTING','1','conv1')
            expect(self.cnc.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            self.cnc.superstate.event('conv','Coordinator', 'binding_state', 'COMMITTED','1','conv1')
            expect(self.cnc.superstate.material_load.value()).to(equal('READY'))

            time.sleep(2) #Simulation for conv unload

            self.cnc.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'READY', '1', 'r1')
            expect(self.cnc.superstate.material_load.value()).to(equal('ACTIVE'))

            self.cnc.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE', '1', 'r1')

            time.sleep(0.2)
            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_OpenDoor', 'READY','1','r1')
            expect(self.cnc.superstate.open_door_interface.superstate.state).to(equal('base:ready'))

            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_OpenDoor', 'ACTIVE','1','r1')
            expect(self.cnc.superstate.open_door_interface.superstate.state).to(equal('base:active'))

            time.sleep(1.5) #cycled response time

            expect(self.cnc.superstate.open_door_interface.superstate.state).to(equal('base:complete'))
            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_OpenDoor', 'NOT_READY','1','r1')

            time.sleep(0.2)

            self.cnc.superstate.event('r1','ChuckInterface', 'SubTask_CloseChuck', 'READY','1','r1')
            expect(self.cnc.superstate.close_chuck_interface.superstate.state).to(equal('base:ready'))

            self.cnc.superstate.event('r1','ChuckInterface', 'SubTask_CloseChuck', 'ACTIVE','1','r1')
            expect(self.cnc.superstate.close_chuck_interface.superstate.state).to(equal('base:active'))

            time.sleep(1.5) #cycled response time

            expect(self.cnc.superstate.close_chuck_interface.superstate.state).to(equal('base:complete'))
            self.cnc.superstate.event('r1','ChuckInterface', 'SubTask_CloseChuck', 'NOT_READY','1','r1')

            time.sleep(0.2)

            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_CloseDoor', 'READY','1','r1')
            expect(self.cnc.superstate.close_door_interface.superstate.state).to(equal('base:ready'))

            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_CloseDoor', 'ACTIVE','1','r1')
            expect(self.cnc.superstate.close_door_interface.superstate.state).to(equal('base:active'))

            time.sleep(1.5) #cycled response time

            expect(self.cnc.superstate.close_door_interface.superstate.state).to(equal('base:complete'))
            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_CloseDoor', 'NOT_READY','1','r1')

            def cnc(self = None):
                self.cnc.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE', '1', 'r1')
            thread = Thread(target =cnc,args=(self,))
            thread.start()
            time.sleep(0.5)


            expect(self.cnc.superstate.material_load.value()).to(equal('NOT_READY'))

            expect(self.cnc.superstate.binding_state_material.value()).to(equal("INACTIVE"))

            expect(self.cnc.superstate.state).to(equal("base:operational:cycle_start"))

            self.cnc.superstate.binding_state_material.set_value("UNAVAILABLE")

            time.sleep(10) #machine cycle time simulation

            expect(self.cnc.superstate.state).to(equal("base:operational:unloading"))


    with context('move material from cnc to buffer'):

        with before.all:
            self.cnc = None
            self.cnc = cnc('localhost',7792)
            self.cnc.create_statemachine()
            self.cnc.superstate.load_time_limit(20)
            self.cnc.superstate.unload_time_limit(20)
            self.cnc.superstate.has_material = True
            self.cnc.superstate.chuck_state.set_value("CLOSED")
            self.cnc.superstate.enable()
            time.sleep(1)

        with it ('should complete successfully'):
            self.cnc.superstate.event('robot','Task_Collaborator', 'binding_state', 'PREPARING',self.cnc.superstate.master_uuid,'r1')

            self.cnc.superstate.event('buffer','Task_Collaborator', 'binding_state', 'PREPARING',self.cnc.superstate.master_uuid,'b1')

            self.cnc.superstate.priority.binding_state(device = 'r1', state = 'PREPARING', binding = self.cnc.superstate.master_uuid) #Is updated from_long_pull.py
            self.cnc.superstate.priority.binding_state(device = 'b1', state = 'PREPARING', binding = self.cnc.superstate.master_uuid) #Is updated from_long_pull.py

            expect(self.cnc.superstate.binding_state_material.value()).to(equal('COMMITTING'))

            self.cnc.superstate.event('robot','Task_Collaborator', 'binding_state', 'COMMITTED',self.cnc.superstate.master_uuid,'r1')

            self.cnc.superstate.event('buffer','Task_Collaborator', 'binding_state', 'COMMITTED',self.cnc.superstate.master_uuid,'b1')

            time.sleep(0.2)

            expect(self.cnc.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            expect(self.cnc.superstate.material_unload.value()).to(equal('READY'))

            expect(self.cnc.superstate.material_load.value()).to(equal('NOT_READY'))

            self.cnc.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'READY',self.cnc.superstate.master_uuid,'r1')
            expect(self.cnc.superstate.material_unload.value()).to(equal('ACTIVE'))

            self.cnc.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE',self.cnc.superstate.master_uuid,'r1')

            time.sleep(0.2)
            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_OpenDoor', 'READY',self.cnc.superstate.master_uuid,'r1')
            expect(self.cnc.superstate.open_door_interface.superstate.state).to(equal('base:ready'))

            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_OpenDoor', 'ACTIVE',self.cnc.superstate.master_uuid,'r1')
            expect(self.cnc.superstate.open_door_interface.superstate.state).to(equal('base:active'))

            time.sleep(1.5) #cycled response time

            expect(self.cnc.superstate.open_door_interface.superstate.state).to(equal('base:complete'))
            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_OpenDoor', 'NOT_READY',self.cnc.superstate.master_uuid,'r1')

            time.sleep(0.2)

            self.cnc.superstate.event('r1','ChuckInterface', 'SubTask_OpenChuck', 'READY',self.cnc.superstate.master_uuid,'r1')
            expect(self.cnc.superstate.open_chuck_interface.superstate.state).to(equal('base:ready'))

            self.cnc.superstate.event('r1','ChuckInterface', 'SubTask_OpenChuck', 'ACTIVE',self.cnc.superstate.master_uuid,'r1')
            expect(self.cnc.superstate.open_chuck_interface.superstate.state).to(equal('base:active'))

            time.sleep(1.5) #cycled response time

            expect(self.cnc.superstate.open_chuck_interface.superstate.state).to(equal('base:complete'))
            self.cnc.superstate.event('r1','ChuckInterface', 'SubTask_OpenChuck', 'NOT_READY',self.cnc.superstate.master_uuid,'r1')

            time.sleep(0.2)

            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_CloseDoor', 'READY',self.cnc.superstate.master_uuid,'r1')
            expect(self.cnc.superstate.close_door_interface.superstate.state).to(equal('base:ready'))

            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_CloseDoor', 'ACTIVE',self.cnc.superstate.master_uuid,'r1')
            expect(self.cnc.superstate.close_door_interface.superstate.state).to(equal('base:active'))

            time.sleep(1.5) #cycled response time

            expect(self.cnc.superstate.close_door_interface.superstate.state).to(equal('base:complete'))
            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_CloseDoor', 'NOT_READY',self.cnc.superstate.master_uuid,'r1')


            def cnc2(self = None):
                self.cnc.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE', self.cnc.superstate.master_uuid, 'r1')
            thread = Thread(target =cnc2,args=(self,))
            thread.start()
            time.sleep(0.5)


            expect(self.cnc.superstate.material_unload.value()).to(equal('NOT_READY'))

            time.sleep(2) #load buffer simulation

            def cnc3(self = None):
                self.cnc.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE', self.cnc.superstate.master_uuid, 'r1')
            thread = Thread(target =cnc3,args=(self,))
            thread.start()
            time.sleep(0.5)


            self.cnc.superstate.event('buffer','Task_Collaborator', 'binding_state', 'INACTIVE',self.cnc.superstate.master_uuid,'b1')

            expect(self.cnc.superstate.coordinator.superstate.task.superstate.state).to(equal('removed'))

            time.sleep(2)

            self.cnc.superstate.binding_state_material.set_value("UNAVAILABLE")


