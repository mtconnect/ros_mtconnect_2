import os,sys
sys.path.insert(0,os.path.dirname(os.getcwd())+'/src')

from expects import expect, be_a, equal

from interfaces.request import Request
from interfaces.response import Response
from inputConveyor import inputConveyor
from collaborationModel.archetypeToInstance import archetypeToInstance

import time
import threading
from threading import Timer, Thread


#make sure "self.initiate_pull_thread()" is commented out in the respective S/M
with description('Conveyor'):

    with context('state'):

        with before.all:
            def cell_part(self, value = None, current_part = None, cycle_count = None):
                return True

            self.cell_part = cell_part
            self.conv = inputConveyor('localhost',7890, self.cell_part)
            self.conv.create_statemachine()

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
            def cell_part(self, value = None, current_part = None, cycle_count = None):
                return True

            self.cell_part = cell_part
            self.conv = inputConveyor('localhost',7891,self.cell_part)
            self.conv.create_statemachine()
            self.conv.superstate.load_time_limit(100)
            self.conv.superstate.unload_time_limit(100)
            self.conv.superstate.current_part = None
            self.conv.superstate.enable()
            time.sleep(1)

        with it ('should complete successfully'):
            self.conv.superstate.event('cnc','Task_Collaborator', 'binding_state', 'PREPARING',self.conv.superstate.master_uuid,'cnc1')

            self.conv.superstate.event('robot','Task_Collaborator', 'binding_state', 'PREPARING',self.conv.superstate.master_uuid,'r1')

            self.conv.superstate.priority.binding_state(device = 'r1', state = 'PREPARING', binding = self.conv.superstate.master_uuid)
            self.conv.superstate.priority.binding_state(device = 'cnc1', state = 'PREPARING', binding = self.conv.superstate.master_uuid)

            self.conv.superstate.event('cnc','Task_Collaborator', 'binding_state', 'COMMITTED',self.conv.superstate.master_uuid,'cnc1')

            self.conv.superstate.event('robot','Task_Collaborator', 'binding_state', 'COMMITTED',self.conv.superstate.master_uuid,'r1')
            time.sleep(0.2)

            expect(self.conv.superstate.binding_state_material.value()).to(equal('COMMITTED'))

            self.conv.superstate.event('r1','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'READY',self.conv.superstate.master_uuid,'r1')

            expect(self.conv.superstate.material_unload.value()).to(equal('ACTIVE'))


            self.conv.superstate.event('r1','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE',self.conv.superstate.master_uuid,'r1')

            time.sleep(2)

            expect(self.conv.superstate.material_unload_interface.superstate.state).to(equal('base:processing'))

            def conv(self = None):
                self.conv.superstate.event('r1','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE',self.conv.superstate.master_uuid,'r1')
                return

            thread = Thread(target =conv,args=(self,))
            thread.start()
            time.sleep(2)

            expect(self.conv.superstate.material_unload.value()).to(equal('NOT_READY'))

            self.conv.superstate.event('cnc','Task_Collaborator', 'binding_state', 'INACTIVE',self.conv.superstate.master_uuid,'cnc1')

            expect(self.conv.superstate.coordinator.superstate.task.superstate.state).to(equal('removed'))
