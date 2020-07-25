import os,sys
#sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from expects import expect, be_a, equal

from src.interfaces.request import Request
from src.interfaces.response import Response
from src.buffer import Buffer
from src.collaborationModel.archetypeToInstance import archetypeToInstance

from mock import Mock

import time, threading

Buffer.StateMachineModel.initiate_pull_thread = Mock()

with description('Buffer'):

    with context('state'):

        with before.all:
            self.buffer = Buffer('localhost',7990)

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

            self.buffer.superstate.priority.collab_check =Mock()
            self.buffer.superstate.load_time_limit(10)
            self.buffer.superstate.unload_time_limit(10)
            self.buffer.superstate.master_uuid = '1'
            self.buffer.superstate.master_tasks['1'] = archetypeToInstance('MoveMaterial_2','1','b1').jsonInstance()
            self.buffer.superstate.enable()

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

            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE','1','r1')
            expect(self.buffer.superstate.material_load.value()).to(equal('NOT_READY'))
            expect(self.buffer.superstate.state).to(equal("base:operational:idle"))

            expect(self.buffer.superstate.binding_state_material.value()).to(equal("INACTIVE"))

            self.buffer.superstate.event('cnc','Coordinator', 'binding_state', 'INACTIVE','1','cnc1')

            expect(self.buffer.superstate.binding_state_material.value()).to(equal("PREPARING"))



    with context('move material from buffer to cmm'):

        with before.all:
            self.buffer = None
            self.buffer = Buffer('localhost',7992)

            self.buffer.superstate.priority.collab_check = Mock()
            self.buffer.superstate.load_time_limit(10)
            self.buffer.superstate.unload_time_limit(10)
            self.buffer.superstate.buffer.append([1])
            self.buffer.superstate.enable()


        with it ('should complete successfully'):
            self.buffer.superstate.event('robot','Task_Collaborator', 'binding_state', 'PREPARING',self.buffer.superstate.master_uuid,'r1')

            self.buffer.superstate.event('cmm','Task_Collaborator', 'binding_state', 'PREPARING',self.buffer.superstate.master_uuid,'cmm1')

            self.buffer.superstate.priority.binding_state(device = 'r1', state = 'PREPARING', binding = self.buffer.superstate.master_uuid) #Is updated from_long_pull.py
            self.buffer.superstate.priority.binding_state(device = 'cmm1', state = 'PREPARING', binding = self.buffer.superstate.master_uuid) #Is updated from_long_pull.py
            expect(self.buffer.superstate.binding_state_material.value()).to(equal('COMMITTING'))

            self.buffer.superstate.event('robot','Task_Collaborator', 'binding_state', 'COMMITTED',self.buffer.superstate.master_uuid,'r1')

            self.buffer.superstate.event('cmm','Task_Collaborator', 'binding_state', 'COMMITTED',self.buffer.superstate.master_uuid,'cmm1')
            expect(self.buffer.superstate.binding_state_material.value()).to(equal('COMMITTED'))
            expect(self.buffer.superstate.material_unload.value()).to(equal('READY'))
            expect(self.buffer.superstate.material_load.value()).to(equal('NOT_READY'))

            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'READY',self.buffer.superstate.master_uuid,'r1')
            expect(self.buffer.superstate.material_unload.value()).to(equal('ACTIVE'))

            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE',self.buffer.superstate.master_uuid,'r1')

            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE',self.buffer.superstate.master_uuid,'r1')
            expect(self.buffer.superstate.material_unload.value()).to(equal('NOT_READY'))

            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE',self.buffer.superstate.master_uuid,'r1')

            self.buffer.superstate.event('cmm','Task_Collaborator', 'binding_state', 'INACTIVE',self.buffer.superstate.master_uuid,'cmm1')

            expect(self.buffer.superstate.coordinator.superstate.task.superstate.state).to(equal('removed'))


