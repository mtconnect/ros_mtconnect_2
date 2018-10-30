import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from expects import expect, be_a, equal

from interfaces.request import Request
from interfaces.response import Response
from buffer import Buffer
from collaborationModel.archetypeToInstance import archetypeToInstance

import time, threading

#make sure "self.initiate_pull_thread()" is commented out in the respective S/M
with description('Buffer'):

    with context('state'):

        with before.all:
            self.buffer = Buffer('localhost',7990)
            self.buffer.create_statemachine()

        with it('should be in binding state INACTIVE initially'):
            expect(self.buffer.superstate.binding_state_material.value()).to(equal('INACTIVE'))

        with it('should have request interfaces'):
            expect(self.buffer.superstate.material_load_interface).to(be_a(Request))
            expect(self.buffer.superstate.material_unload_interface).to(be_a(Request))

        with it('should be in execution ready when initialized'):
            expect(self.buffer.superstate.e1.value()).to(equal('READY'))

        with it('should be in controller mode automatic when initialized'):
            expect(self.buffer.superstate.mode1.value()).to(equal('AUTOMATIC'))


    with context('move material from cnc to buffer'):

        with before.all:
            self.buffer = None
            self.buffer = Buffer('localhost',7991)
            self.buffer.create_statemachine()
            self.buffer.superstate.load_time_limit(10)
            self.buffer.superstate.unload_time_limit(10)
            self.buffer.superstate.master_uuid = '1'
            self.buffer.superstate.master_tasks['1'] = archetypeToInstance('MoveMaterial_2','1','b1').jsonInstance()
            self.buffer.superstate.enable()
            time.sleep(1)

        with it ('should complete successfully'):
            self.buffer.superstate.event('cnc','Coordinator', 'binding_state', 'PREPARING',['1', self.buffer.superstate.master_tasks['1']],'cnc1')

            expect(self.buffer.superstate.binding_state_material.value()).to(equal('PREPARING'))

            self.buffer.superstate.event('cnc','Coordinator', 'binding_state', 'COMMITTING','1','cnc1')

            expect(self.buffer.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            self.buffer.superstate.event('cnc','Coordinator', 'binding_state', 'COMMITTED','1','cnc1')

            expect(self.buffer.superstate.material_unload.value()).to(equal('NOT_READY'))
            expect(self.buffer.superstate.material_load.value()).to(equal('READY'))
            expect('LoadBuffer' in self.buffer.superstate.collaborator.superstate.subTask).to(equal(True))

            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'READY','1','r1')

            expect(self.buffer.superstate.material_load.value()).to(equal('ACTIVE'))

            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE','1','r1')

            time.sleep(5)  #Simulation for loading

            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE','1','r1')

            expect(self.buffer.superstate.material_load.value()).to(equal('NOT_READY'))

    with context('move material from buffer to cmm'):

        with before.all:
            self.buffer = None
            self.buffer = Buffer('localhost',7992)
            self.buffer.create_statemachine()
            self.buffer.superstate.load_time_limit(10)
            self.buffer.superstate.unload_time_limit(10)
            self.buffer.superstate.buffer.append([1])
            self.buffer.superstate.enable()
            time.sleep(1)

        with it ('should complete successfully'):
            self.buffer.superstate.event('robot','Task_Collaborator', 'binding_state', 'PREPARING',self.buffer.superstate.master_uuid,'r1')

            self.buffer.superstate.event('cmm','Task_Collaborator', 'binding_state', 'PREPARING',self.buffer.superstate.master_uuid,'cmm1')

            self.buffer.superstate.priority.binding_state(device = 'r1', state = 'PREPARING', binding = self.buffer.superstate.master_uuid) #Is updated from_long_pull.py
            self.buffer.superstate.priority.binding_state(device = 'cmm1', state = 'PREPARING', binding = self.buffer.superstate.master_uuid) #Is updated from_long_pull.py

            expect(self.buffer.superstate.binding_state_material.value()).to(equal('COMMITTING'))

            self.buffer.superstate.event('robot','Task_Collaborator', 'binding_state', 'COMMITTED',self.buffer.superstate.master_uuid,'r1')

            self.buffer.superstate.event('cmm','Task_Collaborator', 'binding_state', 'COMMITTED',self.buffer.superstate.master_uuid,'cmm1')

            time.sleep(0.2)

            expect(self.buffer.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            expect(self.buffer.superstate.material_unload.value()).to(equal('READY'))

            expect(self.buffer.superstate.material_load.value()).to(equal('NOT_READY'))

            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'READY',self.buffer.superstate.master_uuid,'r1')

            expect(self.buffer.superstate.material_unload.value()).to(equal('ACTIVE'))

            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE',self.buffer.superstate.master_uuid,'r1')

            time.sleep(2)

            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE',self.buffer.superstate.master_uuid,'r1')

            expect(self.buffer.superstate.material_unload.value()).to(equal('NOT_READY'))

            time.sleep(2)

            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE',self.buffer.superstate.master_uuid,'r1')

            self.buffer.superstate.event('cmm','Task_Collaborator', 'binding_state', 'INACTIVE',self.buffer.superstate.master_uuid,'cmm1')

            expect(self.buffer.superstate.coordinator.superstate.task.superstate.state).to(equal('removed'))

            time.sleep(2)

            self.buffer.superstate.binding_state_material.set_value("UNAVAILABLE")

