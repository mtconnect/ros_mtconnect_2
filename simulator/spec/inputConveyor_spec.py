import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from expects import expect, be_a, equal

from mock import Mock

from src.interfaces.request import Request
from src.interfaces.response import Response
from src.inputConveyor import inputConveyor
from src.collaborationModel.archetypeToInstance import archetypeToInstance

import time
import threading
from threading import Timer, Thread

inputConveyor.StateMachineModel.initiate_pull_thread = Mock()

with description('Conveyor'):

    with context('state'):

        with before.all:

            self.cell_part = Mock()
            self.conv = inputConveyor('localhost',7890, self.cell_part)

        with it('should be in binding state INACTIVE initially'):
            expect(self.conv.superstate.binding_state_material.value()).to(equal('INACTIVE'))

        with it('should have request interfaces'):
            expect(self.conv.superstate.material_load_interface).to(be_a(Request))
            expect(self.conv.superstate.material_unload_interface).to(be_a(Request))

        with it('should be in execution ready when initialized'):
            expect(self.conv.superstate.e1.value()).to(equal('READY'))

        with it('should be in controller mode automatic when initialized'):
            expect(self.conv.superstate.mode1.value()).to(equal('AUTOMATIC'))


    with context('move material from conv to cnc'):

        with before.all:
            self.conv = None
            self.cell_part = Mock()
            self.conv = inputConveyor('localhost',7891,self.cell_part)
            self.conv.superstate.priority.collab_check =Mock()
            self.conv.superstate.load_time_limit(100)
            self.conv.superstate.unload_time_limit(100)
            self.conv.superstate.current_part = None
            self.conv.superstate.enable()

        with it ('should complete successfully'):
            self.conv.superstate.event('cnc','Task_Collaborator', 'binding_state', 'PREPARING',self.conv.superstate.master_uuid,'cnc1')

            self.conv.superstate.event('robot','Task_Collaborator', 'binding_state', 'PREPARING',self.conv.superstate.master_uuid,'r1')

            self.conv.superstate.priority.binding_state(device = 'r1', state = 'PREPARING', binding = self.conv.superstate.master_uuid)
            self.conv.superstate.priority.binding_state(device = 'cnc1', state = 'PREPARING', binding = self.conv.superstate.master_uuid)

            self.conv.superstate.event('cnc','Task_Collaborator', 'binding_state', 'COMMITTED',self.conv.superstate.master_uuid,'cnc1')

            self.conv.superstate.event('robot','Task_Collaborator', 'binding_state', 'COMMITTED',self.conv.superstate.master_uuid,'r1')

            expect(self.conv.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            self.conv.superstate.event('r1','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'READY',self.conv.superstate.master_uuid,'r1')
            expect(self.conv.superstate.material_unload.value()).to(equal('ACTIVE'))

            self.conv.superstate.event('r1','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE',self.conv.superstate.master_uuid,'r1')
            expect(self.conv.superstate.material_unload_interface.superstate.state).to(equal('base:processing'))

            self.conv.superstate.event('r1','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE',self.conv.superstate.master_uuid,'r1')
            expect(self.conv.superstate.material_unload.value()).to(equal('NOT_READY'))

            #WAiting on task(load) completion
            expect(self.conv.superstate.state).to(equal('base:operational:in_transition'))

            self.conv.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE', self.conv.superstate.master_uuid, 'r1')

            self.conv.superstate.event('cnc','Task_Collaborator', 'binding_state', 'INACTIVE',self.conv.superstate.master_uuid,'cnc1')

            expect(self.conv.superstate.binding_state_material.value()).to(equal('PREPARING'))


    with context('move bad material from cmm to conv'):

        with before.all:
            self.conv = None
            self.cell_part = Mock()
            self.conv = inputConveyor('localhost',7892,self.cell_part)
            self.conv.superstate.priority.collab_check =Mock()
            self.conv.superstate.load_time_limit(100)
            self.conv.superstate.unload_time_limit(100)
            self.conv.superstate.current_part = 'good'
            self.conv.superstate.internal_buffer['bad'] =False
            self.conv.superstate.master_uuid = '1'
            self.conv.superstate.master_tasks['1'] = archetypeToInstance('MoveMaterial_4_bad','1','b1').jsonInstance()
            self.conv.superstate.enable()


        with it('should complete successfully'):
            self.conv.superstate.event('cmm','Coordinator', 'binding_state', 'PREPARING',['1', self.conv.superstate.master_tasks['1']],'cmm1')
            expect(self.conv.superstate.binding_state_material.value()).to(equal('PREPARING'))

            self.conv.superstate.event('cmm','Coordinator', 'binding_state', 'COMMITTING','1','cmm1')
            expect(self.conv.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            self.conv.superstate.event('cmm','Coordinator', 'binding_state', 'COMMITTED','1','cmm1')
            expect(self.conv.superstate.material_unload.value()).to(equal('NOT_READY'))
            expect(self.conv.superstate.material_load.value()).to(equal('READY'))
            expect('LoadConv' in self.conv.superstate.collaborator.superstate.subTask).to(equal(True))

            self.conv.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'READY','1','r1')
            expect(self.conv.superstate.material_load.value()).to(equal('ACTIVE'))

            self.conv.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE','1','r1')

            self.conv.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE','1','r1')

            expect(self.conv.superstate.material_load.value()).to(equal('NOT_READY'))
            expect(self.conv.superstate.state).to(equal("base:operational:in_transition"))

            expect(self.conv.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            self.conv.superstate.event('cmm','Coordinator', 'binding_state', 'INACTIVE','1','cmm1')

            expect(self.conv.superstate.state).to(equal("base:operational:unloading"))
            expect(self.conv.superstate.binding_state_material.value()).to(equal('PREPARING'))


