import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from expects import expect, be_a, equal, be_above

from src.collaborationModel.archetypeToInstance import archetypeToInstance
from src.collaborationModel.coordinator import coordinator

from mock import Mock

import time, threading

#Taking inputConveyor and robot as the Collaborators
#Taking a CNC statemachine to test out coordinator behavior
from src.cnc import cnc

cnc.StateMachineModel.initiate_pull_thread = Mock()

CNC = cnc('localhost',7779)

with description('coordinator'):

    with context('state'):

        with before.all:
            self.coordinator = coordinator(parent = CNC.superstate, master_task_uuid = '1', interface = CNC.superstate.binding_state_material , coordinator_name = CNC.superstate.device_uuid)

            self.coordinator.superstate.parent.coordinator_task = "MoveMaterial_2"
            self.coordinator.superstate.parent.priority.collab_check = Mock()
            self.coordinator.superstate.parent.master_uuid = "1"
            self.coordinator.superstate.task_name = "UnloadCnc"
            self.coordinator.superstate.unavailable()
            self.coordinator.superstate.task.superstate.commit_time_limit = 1.0

        with it('should be committed to the task when created'):
            expect(self.coordinator.superstate.state).to(equal('base:committed'))

        with it('of the task should be PREPARING when task is created'):
            expect(self.coordinator.superstate.task.superstate.state).to(equal('base:preparing'))
            expect(self.coordinator.superstate.interface.value()).to(equal('PREPARING'))

        with it('of the task should be COMMITTING when PREPARING events received from the collaborators'):
            self.coordinator.superstate.event('buffer','Task_Collaborator','binding_state','PREPARING','1','b1')
            self.coordinator.superstate.event('robot','Task_Collaborator','binding_state','PREPARING','1','r1')

            expect(self.coordinator.superstate.task.superstate.state).to(equal('base:committing'))
            expect(self.coordinator.superstate.interface.value()).to(equal('COMMITTING'))

        with it('of the task should become PREPARING from COMMITTING after timeout if collaborators do not become COMMITTED'):

            self.coordinator.superstate.task.superstate.no_commit()

            expect(self.coordinator.superstate.task.superstate.state).to(equal('base:preparing'))
            expect(self.coordinator.superstate.interface.value()).to(equal('PREPARING'))


        with it('of the task should be COMMITTED when COMMITTED events received from the collaborators'):
            self.coordinator.superstate.event('buffer','Task_Collaborator','binding_state','PREPARING','1','b1')
            self.coordinator.superstate.event('robot','Task_Collaborator','binding_state','PREPARING','1','r1')

            self.coordinator.superstate.event('buffer','Task_Collaborator','binding_state','COMMITTED','1','b1')
            self.coordinator.superstate.event('robot','Task_Collaborator','binding_state','COMMITTED','1','r1')

            expect(self.coordinator.superstate.task.superstate.state).to(equal('base:committed'))
            expect(self.coordinator.superstate.interface.value()).to(equal('COMMITTED'))

        with context('once COMMITTED'):

            with before.each:

                self.coordinator = coordinator(parent = CNC.superstate, master_task_uuid = '1', interface = CNC.superstate.binding_state_material , coordinator_name = CNC.superstate.device_uuid)

                self.coordinator.superstate.parent.priority.collab_check = Mock()
                self.coordinator.superstate.parent.coordinator_task = "MoveMaterial_2"
                self.coordinator.superstate.parent.master_uuid = "1"
                self.coordinator.superstate.task_name = "UnloadCnc"
                self.coordinator.superstate.unavailable()

                self.coordinator.superstate.task.superstate.commit_time_limit = 1

                self.coordinator.superstate.event('buffer','Task_Collaborator','binding_state','PREPARING','1','b1')
                self.coordinator.superstate.event('robot','Task_Collaborator','binding_state','PREPARING','1','r1')

                self.coordinator.superstate.event('buffer','Task_Collaborator','binding_state','COMMITTED','1','b1')
                self.coordinator.superstate.event('robot','Task_Collaborator','binding_state','COMMITTED','1','r1')


            with it('should become INACTIVE when all subTasks are successfully completed and REMOVED and all collaborators become INACTIVE'):

                self.coordinator.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'READY',self.coordinator.superstate.parent.master_uuid,'r1')

                self.coordinator.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE',self.coordinator.superstate.parent.master_uuid,'r1')

                self.coordinator.superstate.event('r1','DoorInterface', 'SubTask_OpenDoor', 'READY',self.coordinator.superstate.parent.master_uuid,'r1')

                self.coordinator.superstate.event('r1','DoorInterface', 'SubTask_OpenDoor', 'ACTIVE',self.coordinator.superstate.parent.master_uuid,'r1')

                self.coordinator.superstate.event('r1','DoorInterface', 'SubTask_OpenDoor', 'NOT_READY',self.coordinator.superstate.parent.master_uuid,'r1')
                expect(self.coordinator.superstate.task.superstate.subTask['OpenDoor'].superstate.state).to(equal('removed'))

                self.coordinator.superstate.event('r1','ChuckInterface', 'SubTask_OpenChuck', 'READY',self.coordinator.superstate.parent.master_uuid,'r1')

                self.coordinator.superstate.event('r1','ChuckInterface', 'SubTask_OpenChuck', 'ACTIVE',self.coordinator.superstate.parent.master_uuid,'r1')

                self.coordinator.superstate.event('r1','ChuckInterface', 'SubTask_OpenChuck', 'NOT_READY',self.coordinator.superstate.parent.master_uuid,'r1')
                expect(self.coordinator.superstate.task.superstate.subTask['OpenChuck'].superstate.state).to(equal('removed'))

                self.coordinator.superstate.event('r1','DoorInterface', 'SubTask_CloseDoor', 'READY',self.coordinator.superstate.parent.master_uuid,'r1')

                self.coordinator.superstate.event('r1','DoorInterface', 'SubTask_CloseDoor', 'ACTIVE',self.coordinator.superstate.parent.master_uuid,'r1')

                self.coordinator.superstate.event('r1','DoorInterface', 'SubTask_CloseDoor', 'NOT_READY',self.coordinator.superstate.parent.master_uuid,'r1')
                expect(self.coordinator.superstate.task.superstate.subTask['CloseDoor'].superstate.state).to(equal('removed'))

                self.coordinator.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE',self.coordinator.superstate.parent.master_uuid, 'r1')
                expect(self.coordinator.superstate.task.superstate.subTask['UnloadCnc'].superstate.state).to(equal('removed'))

                self.coordinator.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE',self.coordinator.superstate.parent.master_uuid, 'r1')

                self.coordinator.superstate.event('buffer','Task_Collaborator', 'binding_state', 'INACTIVE',self.coordinator.superstate.parent.master_uuid,'b1')

                expect(self.coordinator.superstate.task.superstate.state).to(equal('removed'))
                expect(self.coordinator.superstate.interface.value()).to(equal('INACTIVE'))

