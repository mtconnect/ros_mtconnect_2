from expects import expect, be_a, equal

from src.request import Request
from src.response import Response
from src.cnc import cnc
import time

#make sure "self.initiate_pull_thread()" 

with description('CNC'):
    with before.all:
        self.cnc = cnc('localhost',7904)

    with context('move material from conv to cnc'):
        with before.all:
            self.cnc.create_statemachine()
            self.cnc.superstate.enable()
            self.cnc.superstate.master_tasks['1'] = {'coordinator': {'conv1': {'state': ['conveyor', 'conv1', None], 'Task': ['move_material', None], 'SubTask': {'conv1': ['UnloadConv', None, 'r1', 'MaterialUnload', '1'], 'r1': [], 'cnc1': ['LoadCnc', None, 'r1', 'MaterialLoad', '2']}}}, 'collaborators': {'r1': {'state': ['ROBOT', 'r1', None], 'SubTask': {'UnloadConv': [['Interface', 'MoveIn', None, '1', None], ['Interface', 'GrabPart', None, '2', None], ['Interface', 'MoveOut', None, '3', None]], 'LoadCnc': [['Interface', 'OpenDoor', None, '1', ['cnc1']], ['Interface', 'MoveIn', None, '2', None], ['Interface', 'CloseChuck', None, '3', ['cnc1']], ['Interface', 'ReleasePart', None, '4', None], ['Interface', 'MoveOut', None, '5', None], ['Interface', 'CloseDoor', None, '6', ['cnc1']]]}}, 'cnc1': {'state': ['CNC', 'cnc1', None], 'SubTask': {}}}}
            self.cnc.superstate.master_uuid = '1'
            

        with it ('should complete successfully'):

            self.cnc.superstate.event('conv','Coordinator', 'binding_state', 'PREPARING',['1', self.cnc.superstate.master_tasks['1']],'conv1')
            time.sleep(0.2)

            expect(self.cnc.superstate.binding_state_material.value()).to(equal('PREPARING'))

            self.cnc.superstate.event('conv','Coordinator', 'binding_state', 'COMMITTING','1','conv1')
            expect(self.cnc.superstate.binding_state_material.value()).to(equal('COMMITTED'))
            time.sleep(0.3)
            
            self.cnc.superstate.event('conv','Coordinator', 'binding_state', 'COMMITTED','1','conv1')
            time.sleep(0.1)

            expect(self.cnc.superstate.material_load.value()).to(equal('READY'))

            self.cnc.superstate.event('r1','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'READY','1','r1')
            time.sleep(0.2)

            self.cnc.superstate.event('r1','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE','1','r1')

            time.sleep(1.2)

            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_OpenDoor', 'READY','1','r1')

            time.sleep(0.1)

            expect(self.cnc.superstate.open_door_interface.superstate.state).to(equal('base:ready'))


            self.cnc.superstate.event('r1','DoorInterface', 'SubTask_OpenDoor', 'ACTIVE','1','r1')

            time.sleep(0.1)
            expect(self.cnc.superstate.open_door_interface.superstate.state).to(equal('base:active'))


            time.sleep(1.5)

            expect(self.cnc.superstate.open_door_interface.superstate.state).to(equal('base:complete'))

            self.cnc.superstate.adapter.stop()

