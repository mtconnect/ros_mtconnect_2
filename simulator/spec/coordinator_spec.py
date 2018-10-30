import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from expects import expect, be_a, equal, be_above

from collaborationModel.archetypeToInstance import archetypeToInstance
from collaborationModel.coordinator import coordinator

import time, threading

#Taking inputConveyor and robot as the Collaborators
#Taking a CNC statemachine to test out coordinator behavior
from src.cnc import cnc
CNC = cnc('localhost',7778)
CNC.create_statemachine()

#make sure "self.initiate_pull_thread()" is commented out in the respective S/M

with description('coordinator'):

    with context('state'):

        with before.all:
            self.coordinator = coordinator(parent = CNC.superstate, master_task_uuid = '1', interface = CNC.superstate.binding_state_material , coordinator_name = CNC.superstate.device_uuid)
            self.coordinator.create_statemachine()
            self.coordinator.superstate.parent.coordinator_task = "MoveMaterial_2"
            self.coordinator.superstate.parent.master_uuid = "1"
            self.coordinator.superstate.task_name = "UnloadCnc"
            self.coordinator.superstate.unavailable()
            time.sleep(1)
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
            time.sleep(1.5)
            expect(self.coordinator.superstate.task.superstate.state).to(equal('base:preparing'))
            expect(self.coordinator.superstate.interface.value()).to(equal('PREPARING'))

            #Not part of the test: going back to previous state
            self.coordinator.superstate.event('buffer','Task_Collaborator','binding_state','PREPARING','1','b1')
            self.coordinator.superstate.event('robot','Task_Collaborator','binding_state','PREPARING','1','r1')


        with it('of the task should be COMMITTED when COMMITTED events received from the collaborators'):
            self.coordinator.superstate.event('buffer','Task_Collaborator','binding_state','COMMITTED','1','b1')
            self.coordinator.superstate.event('robot','Task_Collaborator','binding_state','COMMITTED','1','r1')
            time.sleep(0.2) #thread created

            expect(self.coordinator.superstate.task.superstate.state).to(equal('base:committed'))
            expect(self.coordinator.superstate.interface.value()).to(equal('COMMITTED'))

        with context('once COMMITTED'):

            with before.each:

                self.coordinator = coordinator(parent = CNC.superstate, master_task_uuid = '1', interface = CNC.superstate.binding_state_material , coordinator_name = CNC.superstate.device_uuid)
                self.coordinator.create_statemachine()
                self.coordinator.superstate.parent.coordinator_task = "MoveMaterial_2"
                self.coordinator.superstate.parent.master_uuid = "1"
                self.coordinator.superstate.task_name = "UnloadCnc"
                self.coordinator.superstate.unavailable()
                time.sleep(1)
                self.coordinator.superstate.task.superstate.commit_time_limit = 1.0

                self.coordinator.superstate.event('buffer','Task_Collaborator','binding_state','PREPARING','1','b1')
                self.coordinator.superstate.event('robot','Task_Collaborator','binding_state','PREPARING','1','r1')

                self.coordinator.superstate.event('buffer','Task_Collaborator','binding_state','COMMITTED','1','b1')
                self.coordinator.superstate.event('robot','Task_Collaborator','binding_state','COMMITTED','1','r1')
                time.sleep(0.5)


            with it('should become INACTIVE when all subTasks are successfully completed and REMOVED and all collaborators become INACTIVE'):

                self.coordinator.superstate.event('robot','DoorInterface','SubTask_OpenDoor','COMPLETE','1','r1')
                expect(self.coordinator.superstate.task.superstate.subTask['OpenDoor'].superstate.state).to(equal('removed'))
                time.sleep(2)

                self.coordinator.superstate.event('robot','ChuckInterface','SubTask_OpenChuck','COMPLETE','1','r1')
                expect(self.coordinator.superstate.task.superstate.subTask['OpenChuck'].superstate.state).to(equal('removed'))
                time.sleep(2)

                self.coordinator.superstate.event('robot','DoorInterface','SubTask_CloseDoor','COMPLETE','1','r1')
                expect(self.coordinator.superstate.task.superstate.subTask['CloseDoor'].superstate.state).to(equal('removed'))
                time.sleep(2)

                expect(self.coordinator.superstate.task.superstate.state).to(equal('base:committed'))
                expect(self.coordinator.superstate.interface.value()).to(equal('COMMITTED'))

                self.coordinator.superstate.event('robot','MaterialInterface','SubTask_MaterialUnload','COMPLETE','1','r1')
                expect(self.coordinator.superstate.task.superstate.subTask['UnloadCnc'].superstate.state).to(equal('removed'))
                time.sleep(2)

                self.coordinator.superstate.event('robot','MaterialInterface','SubTask_MaterialLoad','COMPLETE','1','r1')
                time.sleep(2)

                self.coordinator.superstate.event('buffer','Task_Collaborator','binding_state','INACTIVE','1','b1')
                self.coordinator.superstate.event('robot','Task_Collaborator','binding_state','INACTIVE','1','r1')

                time.sleep(2)

                expect(self.coordinator.superstate.task.superstate.state).to(equal('removed'))
                expect(self.coordinator.superstate.interface.value()).to(equal('INACTIVE'))

