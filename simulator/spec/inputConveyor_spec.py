from expects import expect, be_a, equal

from src.request import Request
from src.response import Response
from src.inputConveyor import inputConveyor
import time

#make sure "self.initiate_pull_thread()" 

with description('Conv'):
    with before.all:
        self.conv = inputConveyor('localhost',7904)

    with context('move material from conv to cnc'):
        with before.all:
            self.conv.create_statemachine()
            self.conv.superstate.enable()
            

        with it ('should complete successfully'):

            self.conv.superstate.event('cnc','Task_Collaborator', 'binding_state', 'PREPARING',self.conv.superstate.master_uuid,'cnc1')

            self.conv.superstate.event('robot','Task_Collaborator', 'binding_state', 'PREPARING',self.conv.superstate.master_uuid,'r1')
            time.sleep(0.2)

            self.conv.superstate.event('cnc','Task_Collaborator', 'binding_state', 'COMMITTED',self.conv.superstate.master_uuid,'cnc1')

            self.conv.superstate.event('robot','Task_Collaborator', 'binding_state', 'COMMITTED',self.conv.superstate.master_uuid,'r1')
            time.sleep(0.2)


            
            self.conv.superstate.event('r1','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'READY',self.conv.superstate.master_uuid,'r1')

            time.sleep(0.1)

            self.conv.superstate.event('r1','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE',self.conv.superstate.master_uuid,'r1')

            time.sleep(5)

            self.conv.superstate.event('r1','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE',self.conv.superstate.master_uuid,'r1')

            time.sleep(0.1)

            self.conv.superstate.event('cnc','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'NOT_READY',self.conv.superstate.master_uuid,'cnc1')

            print self.conv.superstate.coordinator.superstate.task.superstate.subTask
            expect(self.conv.superstate.state).to(equal('base:operational:loading'))

