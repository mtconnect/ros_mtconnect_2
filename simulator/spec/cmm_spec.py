import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from expects import expect, be_a, equal

from src.interfaces.request import Request
from src.interfaces.response import Response
from src.cmm import cmm
from src.collaborationModel.archetypeToInstance import archetypeToInstance
from mock import Mock

from threading import Thread
import time, threading

cmm.StateMachineModel.initiate_pull_thread = Mock()

with description('cmm'):

    with context('state'):

        with before.all:
            self.cmm = cmm('localhost',7690)

        with it('should be in binding state INACTIVE initially'):
            expect(self.cmm.superstate.binding_state_material.value()).to(equal('INACTIVE'))

        with it('should have request interfaces'):
            expect(self.cmm.superstate.material_load_interface).to(be_a(Request))
            expect(self.cmm.superstate.material_unload_interface).to(be_a(Request))

        with it('should be in execution ready when initialized'):
            expect(self.cmm.superstate.e1.value()).to(equal('READY'))

        with it('should be in controller mode automatic when initialized'):
            expect(self.cmm.superstate.mode1.value()).to(equal('AUTOMATIC'))

    with context('move material from buffer to cmm'):

        with before.all:
            self.cmm = None
            self.cmm = cmm('localhost',7691)

            self.cmm.superstate.priority.collab_check = Mock()
            self.cmm.superstate.load_time_limit(10)
            self.cmm.superstate.unload_time_limit(10)
            self.cmm.superstate.master_uuid = '1'
            self.cmm.superstate.master_tasks['1'] = archetypeToInstance('MoveMaterial_3','1','cmm1').jsonInstance()
            self.cmm.superstate.cycle_time = 0
            self.cmm.superstate.enable()

        with it ('should complete successfully'):
            self.cmm.superstate.event('buffer','Coordinator', 'binding_state', 'PREPARING',['1', self.cmm.superstate.master_tasks['1']],'b1')
            expect(self.cmm.superstate.binding_state_material.value()).to(equal('PREPARING'))

            self.cmm.superstate.event('buffer','Coordinator', 'binding_state', 'COMMITTING','1','b1')
            expect(self.cmm.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            self.cmm.superstate.event('buffer','Coordinator', 'binding_state', 'COMMITTED','1','b1')
            expect(self.cmm.superstate.material_unload.value()).to(equal('NOT_READY'))
            expect(self.cmm.superstate.material_load.value()).to(equal('READY'))
            expect('LoadCmm' in self.cmm.superstate.collaborator.superstate.subTask).to(equal(True))

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'READY','1','r1')
            expect(self.cmm.superstate.material_load.value()).to(equal('ACTIVE'))

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE','1','r1')

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE','1','r1')
            expect(self.cmm.superstate.material_load.value()).to(equal('NOT_READY'))

            #expect(self.cmm.superstate.state).to(equal("base:operational:cycle_start")) if sim >0
            #time.sleep(10) #machine cycle time simulation
            #self.cmm.superstate.binding_state_material.set_value("UNAVAILABLE")

            expect(self.cmm.superstate.state).to(equal("base:operational:unloading"))
            expect(self.cmm.superstate.binding_state_material.value()).to(equal("PREPARING"))



    with context('move material from cmm to conv'):

        with before.all:
            self.cmm = None
            self.cmm = cmm('localhost',7692)

            self.cmm.superstate.priority.collab_check = Mock()
            self.cmm.superstate.load_time_limit(10)
            self.cmm.superstate.unload_time_limit(10)
            self.cmm.superstate.has_material = True
            self.cmm.superstate.enable()

        with it ('should complete successfully'):
            self.cmm.superstate.event('robot','Task_Collaborator', 'binding_state', 'PREPARING',self.cmm.superstate.master_uuid,'r1')

            self.cmm.superstate.event('conv','Task_Collaborator', 'binding_state', 'PREPARING',self.cmm.superstate.master_uuid,'conv1')

            self.cmm.superstate.priority.binding_state(device = 'r1', state = 'PREPARING', binding = self.cmm.superstate.master_uuid) #Is updated from_long_pull.py
            self.cmm.superstate.priority.binding_state(device = 'conv1', state = 'PREPARING', binding = self.cmm.superstate.master_uuid) #Is updated from_long_pull.py
            expect(self.cmm.superstate.binding_state_material.value()).to(equal('COMMITTING'))

            self.cmm.superstate.event('robot','Task_Collaborator', 'binding_state', 'COMMITTED',self.cmm.superstate.master_uuid,'r1')

            self.cmm.superstate.event('conv','Task_Collaborator', 'binding_state', 'COMMITTED',self.cmm.superstate.master_uuid,'conv1')
            expect(self.cmm.superstate.binding_state_material.value()).to(equal('COMMITTED'))
            expect(self.cmm.superstate.material_unload.value()).to(equal('READY'))
            expect(self.cmm.superstate.material_load.value()).to(equal('NOT_READY'))

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'READY',self.cmm.superstate.master_uuid,'r1')
            expect(self.cmm.superstate.material_unload.value()).to(equal('ACTIVE'))

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE',self.cmm.superstate.master_uuid,'r1')

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE', self.cmm.superstate.master_uuid, 'r1')
            expect(self.cmm.superstate.material_unload.value()).to(equal('NOT_READY'))

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE', self.cmm.superstate.master_uuid, 'r1')

            self.cmm.superstate.event('conv','Task_Collaborator', 'binding_state', 'INACTIVE',self.cmm.superstate.master_uuid,'conv1')

            expect(self.cmm.superstate.coordinator.superstate.task.superstate.state).to(equal('removed'))


    with context('move bad material from cmm to conv'):

        with before.all:
            self.cmm = None
            self.cmm = cmm('localhost',7693)

            self.cmm.superstate.priority.collab_check = Mock()
            self.cmm.superstate.load_time_limit(10)
            self.cmm.superstate.unload_time_limit(10)
            self.cmm.superstate.has_material = True
            self.cmm.superstate.part_quality_next = Mock(return_value = [str(),'bad'])
            self.cmm.superstate.enable()

        with it('should complete successfully'):
            self.cmm.superstate.event('robot','Task_Collaborator', 'binding_state', 'PREPARING',self.cmm.superstate.master_uuid,'r1')

            self.cmm.superstate.event('conv','Task_Collaborator', 'binding_state', 'PREPARING',self.cmm.superstate.master_uuid,'conv1')

            self.cmm.superstate.event('cnc','Task_Collaborator', 'binding_state', 'PREPARING',self.cmm.superstate.master_uuid,'cnc1')

            self.cmm.superstate.priority.binding_state(device = 'r1', state = 'PREPARING', binding = self.cmm.superstate.master_uuid) #Is updated from_long_pull.py
            self.cmm.superstate.priority.binding_state(device = 'conv1', state = 'PREPARING', binding = self.cmm.superstate.master_uuid) #Is updated from_long_pull.py
            self.cmm.superstate.priority.binding_state(device = 'cnc1', state = 'PREPARING', binding = self.cmm.superstate.master_uuid) #Is updated from_long_pull.py

            expect(self.cmm.superstate.binding_state_material.value()).to(equal('COMMITTING'))

            self.cmm.superstate.event('robot','Task_Collaborator', 'binding_state', 'COMMITTED',self.cmm.superstate.master_uuid,'r1')
            self.cmm.superstate.event('conv','Task_Collaborator', 'binding_state', 'COMMITTED',self.cmm.superstate.master_uuid,'conv1')
            self.cmm.superstate.event('cnc','Task_Collaborator', 'binding_state', 'COMMITTED',self.cmm.superstate.master_uuid,'cnc1')


            expect(self.cmm.superstate.binding_state_material.value()).to(equal('COMMITTED'))
            expect(self.cmm.superstate.material_unload.value()).to(equal('READY'))
            expect(self.cmm.superstate.material_load.value()).to(equal('NOT_READY'))

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'READY',self.cmm.superstate.master_uuid,'r1')
            expect(self.cmm.superstate.material_unload.value()).to(equal('ACTIVE'))

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE',self.cmm.superstate.master_uuid,'r1')

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE', self.cmm.superstate.master_uuid, 'r1')
            expect(self.cmm.superstate.material_unload.value()).to(equal('NOT_READY'))

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE', self.cmm.superstate.master_uuid, 'r1')
            expect(self.cmm.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            self.cmm.superstate.event('robot','Task_Collaborator', 'binding_state', 'INACTIVE',self.cmm.superstate.master_uuid,'r1')

            expect(self.cmm.superstate.binding_state_material.value()).to(equal('INACTIVE'))
            expect(self.cmm.superstate.coordinator.superstate.task.superstate.state).to(equal('removed'))

