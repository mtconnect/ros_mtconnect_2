import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from expects import expect, be_a, equal, be_above

from src.collaborationModel.archetypeToInstance import archetypeToInstance
from src.collaborationModel.collaborator import collaborator

import time, threading

from mock import Mock, MagicMock

CNC = Mock(return_value = True)

class Event(object):
    def __init__(self):
        self._value = "INACTIVE"
    def set_value(self,a):
        self._value = a
    def value(self):
        return self._value

CNC.superstate.binding_state_material = Event()

CNC.superstate.master_tasks = {}
CNC.superstate.master_uuid = str()
CNC.superstate.device_uuid = "cnc1"

with description('collaborator'):

    with context('state'):

        with before.all:
            self.collaborator = collaborator(CNC.superstate, CNC.superstate.binding_state_material, CNC.superstate.device_uuid)

            self.collaborator.superstate.task_name = 'LoadCnc'
            self.collaborator.superstate.commit_time_limit = 1
            self.collaborator.superstate.unavailable()

        with it('should be INACTIVE initially'):
            expect(self.collaborator.superstate.state).to(equal('base:inactive'))
            expect(self.collaborator.superstate.interface.value()).to(equal('INACTIVE'))

        with it('should be PREPARING when event received from the Coordinator'):
            self.collaborator.superstate.event('conv','Coordinator','binding_state','PREPARING',['1',archetypeToInstance('MoveMaterial_1_good','1','conv1').jsonInstance()],'conv1')
            self.collaborator.superstate.parent.master_uuid = '1'

            expect(self.collaborator.superstate.state).to(equal('base:preparing'))
            expect(self.collaborator.superstate.interface.value()).to(equal('PREPARING'))

        with it('should be COMMITTED when Coordinator is COMMITTING'):
            self.collaborator.superstate.event('conv','Coordinator','binding_state','COMMITTING','1','conv1')

            expect(self.collaborator.superstate.state).to(equal('base:committed'))
            expect(self.collaborator.superstate.interface.value()).to(equal('COMMITTED'))

        with it('should become PREPARING after a timeout from COMMITTED if Coordinator does not successfully COMMIT'):

            self.collaborator.superstate.commit_time_limit = 0
            self.collaborator.superstate.no_commit()

            self.collaborator.superstate.event('conv','Coordinator','binding_state','COMMITTING','1','conv1')

            #Committed - > Preparing
            expect(self.collaborator.superstate.state).to(equal('base:preparing'))
            expect(self.collaborator.superstate.interface.value()).to(equal('PREPARING'))

        with it('should become INACTIVE from COMMITTED if Coordinator becomes INACTIVE instead COMMITTED from COMMITTING'):
            self.collaborator.superstate.commit_time_limit = 1

            self.collaborator.superstate.event('conv','Coordinator','binding_state','COMMITTING','1','conv1')

            self.collaborator.superstate.event('conv','Coordinator','binding_state','INACTIVE','1','conv1')

            expect(self.collaborator.superstate.state).to(equal('base:inactive'))
            expect(self.collaborator.superstate.interface.value()).to(equal('INACTIVE'))


        with context('once COMMITTED'):

            with before.each:
                self.collaborator = collaborator(CNC.superstate, CNC.superstate.binding_state_material, CNC.superstate.device_uuid)
                self.collaborator.superstate.task_name = 'LoadCnc'
                self.collaborator.superstate.unavailable()

                self.collaborator.superstate.event('conv','Coordinator','binding_state','PREPARING',['1',archetypeToInstance('MoveMaterial_1_good','1','conv1').jsonInstance()],'conv1')
                self.collaborator.superstate.parent.master_uuid = '1'
                self.collaborator.superstate.event('conv','Coordinator','binding_state','COMMITTING','1','conv1')
                self.collaborator.superstate.event('conv','Coordinator','binding_state','COMMITTED','1','conv1')

            with it('must create subTask/s when Coordinator is COMMITTED'):
                expect(len(self.collaborator.superstate.subTask)).to(be_above(0))

            with it('should become INACTIVE when all subTasks are SUCCESSfully completed'):

                self.collaborator.superstate.event('robot','MaterialInterface','SubTask_MaterialLoad','READY','1','r1')

                self.collaborator.superstate.event('robot','MaterialInterface','SubTask_MaterialLoad','ACTIVE','1','r1')


                self.collaborator.superstate.event('robot','DoorInterface','SubTask_OpenDoor','READY','1','r1')

                self.collaborator.superstate.event('robot','DoorInterface','SubTask_OpenDoor','ACTIVE','1','r1')

                self.collaborator.superstate.event('robot','DoorInterface','SubTask_OpenDoor','NOT_READY','1','r1')


                expect(self.collaborator.superstate.subTask['OpenDoor'].superstate.state).to(equal('removed'))

                self.collaborator.superstate.event('robot','ChuckInterface','SubTask_CloseChuck','READY','1','r1')

                self.collaborator.superstate.event('robot','ChuckInterface','SubTask_CloseChuck','ACTIVE','1','r1')

                self.collaborator.superstate.event('robot','ChuckInterface','SubTask_CloseChuck','NOT_READY','1','r1')

                #time.sleep(2)
                expect(self.collaborator.superstate.subTask['CloseChuck'].superstate.state).to(equal('removed'))

                self.collaborator.superstate.event('robot','DoorInterface','SubTask_CloseDoor','READY','1','r1')

                self.collaborator.superstate.event('robot','DoorInterface','SubTask_CloseDoor','ACTIVE','1','r1')

                self.collaborator.superstate.event('robot','DoorInterface','SubTask_CloseDoor','NOT_READY','1','r1')

                #time.sleep(2)
                expect(self.collaborator.superstate.subTask['CloseDoor'].superstate.state).to(equal('removed'))

                expect(self.collaborator.superstate.state).to(equal('base:committed'))
                expect(self.collaborator.superstate.interface.value()).to(equal('COMMITTED'))

                self.collaborator.superstate.event('robot','MaterialInterface','SubTask_MaterialLoad','COMPLETE','1','r1')
                #time.sleep(2)

                expect(self.collaborator.superstate.state).to(equal('base:inactive'))
                expect(self.collaborator.superstate.interface.value()).to(equal('INACTIVE'))

            with it('should become INACTIVE when Coordinator becomes INACTIVE unexpectedly'):

                self.collaborator.superstate.event('conv','Coordinator','binding_state','INACTIVE','1','conv1')

                expect(self.collaborator.superstate.state).to(equal('base:inactive'))
                expect(self.collaborator.superstate.interface.value()).to(equal('INACTIVE'))


