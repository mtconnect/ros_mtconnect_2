from expects import expect, be_a, equal

from src.request import Request
from src.response import Response
from src.buffer import Buffer
import time

#make sure "self.initiate_pull_thread()" is commented out in robot.py 
with description('Buffer'):
    with before.all:
        self.buffer = Buffer()
        self.buffer.create_statemachine()

    with it('should be in binding state INACTIVE initially'):
        expect(self.buffer.superstate.binding_state_material.value()).to(equal('INACTIVE'))

    with it('should have request interfaces'):
        expect(self.buffer.superstate.material_load_interface).to(be_a(Request))
        expect(self.buffer.superstate.material_unload_interface).to(be_a(Request))

    with it('should be in execution ready when initialized'):
        expect(self.buffer.superstate.e1.value()).to(equal('READY'))

    with it('should be in controller mode automatic when initialized'):
        expect(self.buffer.superstate.mode1.value()).to(equal('AUTOMATIC'))
    """
    with context('move material from cnc to buffer'):
        with before.all:
            self.buffer.superstate.enable()
            self.buffer.superstate.load_time_limit(4)
            self.buffer.superstate.unload_time_limit(4)
            self.buffer.superstate.master_tasks['1'] = {'coordinator': {'cnc1': {'state': ['cnc', 'cnc1', None], 'Task': ['move_material', None], 'SubTask': {'b1': ['LoadBuffer', None, 'r1', 'MaterialLoad'], 'r1': [], 'cnc1': ['UnloadCnc', None, 'r1', 'MaterialUnload']}}}, 'collaborators': {'r1': {'state': ['ROBOT', 'r1', None], 'SubTask': {'UnloadCnc': [['Interface', 'OpenDoor', None, '1', None], ['Interface', 'OpenChuck', None, '2', None]]}}, 'b1': {'state': ['BUFFER', 'b1', None], 'SubTask': {}}}}
            self.buffer.superstate.master_uuid = '1'
            
        with it ('should complete successfully'):

            self.buffer.superstate.event('cnc','Coordinator', 'binding_state', 'PREPARING',['1', self.buffer.superstate.master_tasks['1']],'cnc1')
            time.sleep(0.1)
            
            expect(self.buffer.superstate.binding_state_material.value()).to(equal('PREPARING'))

            self.buffer.superstate.event('cnc','Coordinator', 'binding_state', 'COMMITTING','1','cnc1')
            time.sleep(0.1)
            expect(self.buffer.superstate.binding_state_material.value()).to(equal('COMMITTED'))
            
            
            self.buffer.superstate.event('cnc','BindingState', 'binding_state', 'COMMITTED','1','cnc1')
            time.sleep(0.1)

            expect(self.buffer.superstate.material_unload.value()).to(equal('NOT_READY'))
            expect(self.buffer.superstate.material_load.value()).to(equal('READY'))
            
            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'READY','1','r1')
            time.sleep(1)
            expect(self.buffer.superstate.material_load.value()).to(equal('ACTIVE'))
            
            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE','1','r1')

            time.sleep(2)
            
            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE','1','r1')
            #goes from NOT_READY TO READY VERY FAST
            expect(self.buffer.superstate.material_load.value()).to(equal('READY'))
    """
    with context('move material from buffer to cmm'):
        with before.all:
            self.buffer.superstate.load_time_limit(4)
            self.buffer.superstate.unload_time_limit(4)
            self.buffer.superstate.master_tasks['1'] = {'coordinator': {'b1': {'state': ['buffer', 'b1', None], 'Task': ['move_material', None], 'SubTask': {'b1': ['UnloadBuffer', None, 'r1', 'MaterialUnload'], 'cmm1': ['LoadCmm', None, 'r1', 'MaterialLoad'], 'r1': []}}}, 'collaborators': {'r1': {'state': ['ROBOT', 'r1', None], 'SubTask': {}}, 'cmm1': {'state': ['CMM', 'cmm1', None], 'SubTask': {}}}}
            self.buffer.superstate.master_uuid = '1'
            self.buffer.superstate.has_material = True
            self.buffer.superstate.enable()
            
        with it ('should complete successfully'):
            self.buffer.superstate.event('robot','Task_Collaborator', 'binding_state', 'PREPARING','1','r1')
            time.sleep(0.1)
            self.buffer.superstate.event('cmm','Task_Collaborator', 'binding_state', 'PREPARING','1','cmm1')
            time.sleep(0.1)
            
            expect(self.buffer.superstate.binding_state_material.value()).to(equal('COMMITTING'))

            self.buffer.superstate.event('robot','Task_Collaborator', 'binding_state', 'COMMITTED','1','r1')
            time.sleep(0.1)
            self.buffer.superstate.event('cmm','Task_Collaborator', 'binding_state', 'COMMITTED','1','cmm1')
            time.sleep(0.1)

            
            expect(self.buffer.superstate.binding_state_material.value()).to(equal('COMMITTED'))
            print self.buffer.superstate.iscollaborator

            expect(self.buffer.superstate.material_unload.value()).to(equal('READY'))
            
            expect(self.buffer.superstate.material_load.value()).to(equal('NOT_READY'))
            
            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'READY','1','r1')
            time.sleep(1)
            expect(self.buffer.superstate.material_unload.value()).to(equal('ACTIVE'))
            
            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE','1','r1')

            time.sleep(2)
            
            self.buffer.superstate.event('robot','MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE','1','r1')
            time.sleep(0.1)
            expect(self.buffer.superstate.material_unload.value()).to(equal('READY'))


