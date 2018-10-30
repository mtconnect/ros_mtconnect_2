import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from expects import expect, be_a, equal

from interfaces.request import Request
from interfaces.response import Response
from cmm import cmm
from collaborationModel.archetypeToInstance import archetypeToInstance

from threading import Thread
import time, threading

#make sure "self.initiate_pull_thread()" is commented out in the respective S/M
with description('cmm'):

    with context('state'):

        with before.all:
            self.cmm = cmm('localhost',7690)
            self.cmm.create_statemachine()

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
            self.cmm.create_statemachine()
            self.cmm.superstate.load_time_limit(10)
            self.cmm.superstate.unload_time_limit(10)
            self.cmm.superstate.master_uuid = '1'
            self.cmm.superstate.master_tasks['1'] = archetypeToInstance('MoveMaterial_3','1','cmm1').jsonInstance()
            self.cmm.superstate.enable()
            time.sleep(1)

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

            time.sleep(5)  #Simulation for loading

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE','1','r1')

            expect(self.cmm.superstate.material_load.value()).to(equal('NOT_READY'))

            expect(self.cmm.superstate.binding_state_material.value()).to(equal("INACTIVE"))

            expect(self.cmm.superstate.state).to(equal("base:operational:cycle_start"))

            self.cmm.superstate.binding_state_material.set_value("UNAVAILABLE")

            time.sleep(10) #machine cycle time simulation

            expect(self.cmm.superstate.state).to(equal("base:operational:unloading"))


    with context('move material from cmm to conv'):

        with before.all:
            self.cmm = None
            self.cmm = cmm('localhost',7692)
            self.cmm.create_statemachine()
            self.cmm.superstate.load_time_limit(10)
            self.cmm.superstate.unload_time_limit(10)
            self.cmm.superstate.has_material = True
            self.cmm.superstate.enable()
            time.sleep(1)

        with it ('should complete successfully'):
            self.cmm.superstate.event('robot','Task_Collaborator', 'binding_state', 'PREPARING',self.cmm.superstate.master_uuid,'r1')

            self.cmm.superstate.event('conv','Task_Collaborator', 'binding_state', 'PREPARING',self.cmm.superstate.master_uuid,'conv1')

            self.cmm.superstate.priority.binding_state(device = 'r1', state = 'PREPARING', binding = self.cmm.superstate.master_uuid) #Is updated from_long_pull.py
            self.cmm.superstate.priority.binding_state(device = 'conv1', state = 'PREPARING', binding = self.cmm.superstate.master_uuid) #Is updated from_long_pull.py

            expect(self.cmm.superstate.binding_state_material.value()).to(equal('COMMITTING'))

            self.cmm.superstate.event('robot','Task_Collaborator', 'binding_state', 'COMMITTED',self.cmm.superstate.master_uuid,'r1')

            self.cmm.superstate.event('conv','Task_Collaborator', 'binding_state', 'COMMITTED',self.cmm.superstate.master_uuid,'conv1')

            time.sleep(0.2)

            expect(self.cmm.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            expect(self.cmm.superstate.material_unload.value()).to(equal('READY'))

            expect(self.cmm.superstate.material_load.value()).to(equal('NOT_READY'))

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'READY',self.cmm.superstate.master_uuid,'r1')

            expect(self.cmm.superstate.material_unload.value()).to(equal('ACTIVE'))

            self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE',self.cmm.superstate.master_uuid,'r1')

            time.sleep(2)

            def cmm1(self = None):
                self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE', self.cmm.superstate.master_uuid, 'r1')
            thread = Thread(target =cmm1,args=(self,))
            thread.start()
            time.sleep(0.5)

            expect(self.cmm.superstate.material_unload.value()).to(equal('NOT_READY'))

            time.sleep(2)

            def cmm2(self = None):
                self.cmm.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE', self.cmm.superstate.master_uuid, 'r1')
            thread = Thread(target =cmm2,args=(self,))
            thread.start()
            time.sleep(0.5)


            self.cmm.superstate.event('conv','Task_Collaborator', 'binding_state', 'INACTIVE',self.cmm.superstate.master_uuid,'conv1')

            expect(self.cmm.superstate.coordinator.superstate.task.superstate.state).to(equal('removed'))

            time.sleep(2)

            self.cmm.superstate.binding_state_material.set_value("UNAVAILABLE")

