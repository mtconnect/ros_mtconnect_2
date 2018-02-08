from expects import *
from doublex import *

from src.binding.cnc import *
from src.binding.cmm import *
from src.binding.buffer import *
from src.binding.inputConveyor import *
from src.binding.outputConveyor import *

with description('moveMaterial'):
    with before.each:
        self.inputConveyor = inputConveyor(interface)
        self.cnc = cnc(interface)
        self.buffer = Buffer(interface)
        self.cmm = cmm(interface)
        self.outputConveyor = outputConveyor(interface)
                        
    with it('should have all the state machines'):
        expect(self.inputConveyor).to(be_a(inputConveyor))
        expect(self.cnc).to(be_a(cnc))
        expect(self.buffer).to(be_a(Buffer))
        expect(self.cmm).to(be_a(cmm))
        expect(self.outputConveyor).to(be_a(outputConveyor))
        
    with context('when from inputConveyor to cnc'):

        with before.each:
            self.master_uuid = 1
            self.information_model = {'coordinator':{'inputConveyor1':{'state':['inputConveyor', 'inputConveyor1', None], 'Task': ['move_material', None], 'SubTask': {'inputConveyor1':['MaterialUnload', None, 'r1','source'],'cnc1': ['MaterialLoad', None, 'r1','destination']}}}, 'collaborators': { 'r1':{'state':['robot','r1', None], 'SubTask': { 'MaterialLoad':[['ChuckInterface','Close', None,1, None], ['DoorInterface', 'Close', None,2, None]]}}, 'cnc1':{'state':['cnc','cnc1', None], 'SubTask':{}}}}

            #initializing input conveyor statemachine: source
            self.inputConveyor.create_statemachine()
            self.inputConveyor.superstate.has_material = True
            self.inputConveyor.superstate.master_uuid = self.master_uuid
            self.inputConveyor.superstate.master_tasks[self.master_uuid] = self.information_model
            self.inputConveyor.superstate.unload_time_limit(10)
            self.inputConveyor.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
            self.inputConveyor.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')

            #intializing cnc statemachine: destination
            self.cnc.create_statemachine()
            self.cnc.superstate.has_material = False
            self.cnc.superstate.load_time_limit(10)
            self.cnc.superstate.load_time_limit(10)
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
            self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Close', 'READY')

        with it('binding state should be inactive initially'):
            expect(self.inputConveyor.superstate.binding_state).to(equal('INACTIVE'))
            expect(self.cnc.superstate.binding_state).to(equal('INACTIVE'))

        with it('binding state should become inactive after moveMaterial'):

            #checking initial states
            expect(self.inputConveyor.superstate.binding_state).to(equal('INACTIVE'))
            expect(self.cnc.superstate.binding_state).to(equal('INACTIVE'))

            #enabling operational state in both the statemachines
            self.inputConveyor.superstate.event('inputConveyor', 'Controller', 'ControllerMode', 'AUTOMATIC')
            self.cnc.superstate.event('cnc', 'Controller', 'ControllerMode', 'AUTOMATIC')
            time.sleep(0.1)

            #inputConveyor creates unload task since it has material
            expect(self.inputConveyor.superstate.binding_state).to(equal('PREPARING'))

            #inputConveyor creates load task since it doesnt have material
            expect(self.cnc.superstate.collaborator.superstate.interface.value).to(equal('PREPARING'))

            
            #input Conveyor:coordinator informs everyone about itself and the cnc:collaborator receives the task information
            self.cnc.superstate.collaborator.superstate.event('inputConveyor', 'Coordinator', 'querry', 'INITIATION', code = [self.master_uuid, self.information_model], text = 'inputConveyor1')
            
            #all the collaborators report back their interest and state
            self.inputConveyor.superstate.coordinator.superstate.event('robot', 'Task:Collaborator', 'state', 'PREPARING', code = 1,text = 'r1')
            self.inputConveyor.superstate.coordinator.superstate.event('cnc', 'Task:Collaborator', 'state', 'PREPARING', code = 1,text = 'cnc1')
            time.sleep(0.1)

            expect(self.inputConveyor.superstate.binding_state).to(equal('COMMITTING'))


            #conveyor:coordinator starts to commmit and is reported to the collaborators
            self.cnc.superstate.collaborator.superstate.event('inputConveyor', 'Coordinator', 'state' , 'COMMITTING',self.master_uuid,text = 'inputConveyor1')

            #collaborators start to commit and report back to the coordinator when committed
            self.inputConveyor.superstate.coordinator.superstate.event('robot', 'Task:Collaborator', 'state', 'COMMITTED', code = 1,text = 'r1')
            self.inputConveyor.superstate.coordinator.superstate.event('cnc', 'Task:Collaborator', 'state', 'COMMITTED', code = 1,text = 'cnc1')
            time.sleep(0.1)
            
            expect(self.inputConveyor.superstate.binding_state).to(equal('COMMITTED'))

            #coordinator reports that its committed too and the task:subtask execution begin.
            self.cnc.superstate.collaborator.superstate.event('inputConveyor', 'Coordinator', 'state' , 'COMMITTED',self.master_uuid,text = 'inputConveyor1')

            

            #Task starts
            #subtask_unload starts
            self.inputConveyor.superstate.coordinator.superstate.task.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
            time.sleep(1.1)
            self.inputConveyor.superstate.coordinator.superstate.task.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'COMPLETE')
            self.inputConveyor.superstate.coordinator.superstate.task.superstate.subTask.superstate.event('robot', 'SubTask:Collaborator', 'MaterialUnload', 'COMPLETED')

            #subtask_unload completed
            time.sleep(0.2)
            expect(self.inputConveyor.superstate.state).to(equal('base:operational:idle'))

            #robot has finished unload and requests subtask collaborator to start its subtask execution
            self.cnc.superstate.collaborator.superstate.event('robot', 'SubTask:Collaborator', 'request' , 'START', self.master_uuid,text = 'r1')

            time.sleep(0.2)
            #subtask_load starts
            self.cnc.superstate.collaborator.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')

            time.sleep(1)

            #subtask_chuck_close starts
            self.cnc.superstate.collaborator.superstate._subTask.superstate.event('robot', 'ChuckInterface', 'Close', 'ACTIVE')

            time.sleep(1.1)
            self.cnc.superstate.collaborator.superstate._subTask.superstate.event('robot', 'ChuckInterface', 'close', 'READY')
            self.cnc.superstate.collaborator.superstate._subTask.superstate.event('robot', 'SubTask:Collaborator', 'CHUCK_CLOSE', 'COMPLETED')
            #subtask_chuck_close completed
            
            time.sleep(0.2)
            #subtask_door_close starts
            self.cnc.superstate.collaborator.superstate._subTask.superstate.event('robot', 'DoorInterface', 'Close', 'ACTIVE')
            
            time.sleep(1.1)
            self.cnc.superstate.collaborator.superstate._subTask.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
            self.cnc.superstate.collaborator.superstate._subTask.superstate.event('robot', 'SubTask:Collaborator', 'DOOR_CLOSE', 'COMPLETED')
            #subtask_door_close completed

            expect(self.cnc.superstate.close_door_interface.superstate.response_state).to(equal('CLOSED'))

            expect(self.cnc.superstate.close_chuck_interface.superstate.response_state).to(equal('CLOSED'))
            
            self.cnc.superstate.collaborator.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
            
            self.cnc.superstate.collaborator.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
            time.sleep(0.2)
            self.cnc.superstate.collaborator.superstate.subTask.superstate.event('robot', 'SubTask:Collaborator', 'MaterialLoad', 'COMPLETED')

            #subtask_load completed

            self.inputConveyor.superstate.coordinator.superstate.event('cnc', 'SubTask:Collaborator', 'state', 'COMPLETE', code = self.master_uuid, text = 'cnc1')

            #Task completed
            time.sleep(0.2)

            expect(self.cnc.superstate.state).to(equal('base:operational:cycle_start'))

            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))




            time.sleep(0.2)
                        
            expect(self.inputConveyor.superstate.binding_state).to(equal('INACTIVE'))

    with context('when from cnc to cmm'):

        with before.each:
            self.information_model = {'coordinator':{'cnc1':{'state':['cnc', 'cnc1', None], 'Task': ['move_material', None], 'SubTask': {'cnc1':['MaterialUnload', None, 'r1','source'],'cmm1': ['MaterialLoad', None, 'r1','destination']}}}, 'collaborators': { 'r1':{'state':['robot','r1', None], 'SubTask': { 'MaterialUnload':[['DoorInterface','Open', None,1, None], ['ChuckInterface', 'Open', None,2, None]]}}, 'cmm1':{'state':['cmm','cmm1', None], 'SubTask':{}}}}
            self.master_uuid = 1
            self.cnc.create_statemachine()
            self.cnc.superstate.has_material = True
            self.cnc.superstate.master_uuid = self.master_uuid
            self.cnc.superstate.master_tasks[self.master_uuid] = self.information_model

            self.cnc.superstate.load_time_limit(6)
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
            self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Close', 'READY')


            self.cmm.create_statemachine()
            self.cmm.superstate.has_material = False

            self.cmm.superstate.load_time_limit(6)
            self.cmm.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

            self.cmm.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')


        with it('binding state should become inactive after moveMaterial'):

            expect(self.cnc.superstate.binding_state).to(equal('INACTIVE'))
            expect(self.cmm.superstate.binding_state).to(equal('INACTIVE'))
            
            self.cnc.superstate.event('cnc', 'Controller', 'ControllerMode', 'AUTOMATIC')
            self.cmm.superstate.event('cmm', 'Controller', 'ControllerMode', 'AUTOMATIC')
            time.sleep(0.1)
            expect(self.cnc.superstate.binding_state).to(equal('PREPARING'))
            expect(self.cmm.superstate.collaborator.superstate.interface.value).to(equal('PREPARING'))

            
            self.cmm.superstate.collaborator.superstate.event('cnc', 'Coordinator', 'querry', 'INITIATION', code = [self.master_uuid, self.information_model], text = 'cnc1')
            
            self.cnc.superstate.coordinator.superstate.event('robot', 'Task:Collaborator', 'state', 'PREPARING', code = 1,text = 'r1')
            self.cnc.superstate.coordinator.superstate.event('cmm', 'Task:Collaborator', 'state', 'PREPARING', code = 1,text = 'cmm1')
            time.sleep(0.1)
            
            expect(self.cnc.superstate.binding_state).to(equal('COMMITTING'))

            self.cmm.superstate.collaborator.superstate.event('cnc', 'Coordinator', 'state' , 'COMMITTING',self.master_uuid,text = 'cnc1')
            
            self.cnc.superstate.coordinator.superstate.event('robot', 'Task:Collaborator', 'state', 'COMMITTED', code = 1,text = 'r1')
            self.cnc.superstate.coordinator.superstate.event('cmm', 'Task:Collaborator', 'state', 'COMMITTED', code = 1,text = 'cmm1')
            time.sleep(0.1)
            
            expect(self.cnc.superstate.binding_state).to(equal('COMMITTED'))
            
            self.cmm.superstate.collaborator.superstate.event('cnc', 'Coordinator', 'state' , 'COMMITTED',self.master_uuid,text = 'cnc1')









            self.cnc.superstate.coordinator.superstate.task.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
            self.cnc.superstate.coordinator.superstate.task.superstate._subTask.superstate.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
            
            time.sleep(1.1)
            self.cnc.superstate.coordinator.superstate.task.superstate._subTask.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
            self.cnc.superstate.coordinator.superstate.task.superstate._subTask.superstate.event('robot', 'SubTask:Collaborator', 'DOOR_OPEN', 'COMPLETED')
            
            

            self.cnc.superstate.coordinator.superstate.task.superstate._subTask.superstate.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')

            time.sleep(1.1)
            self.cnc.superstate.coordinator.superstate.task.superstate._subTask.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
            self.cnc.superstate.coordinator.superstate.task.superstate._subTask.superstate.event('robot', 'SubTask:Collaborator', 'CHUCK_OPEN', 'COMPLETED')


            expect(self.cnc.superstate.open_door_interface.superstate.response_state).to(equal('OPEN'))

            expect(self.cnc.superstate.open_chuck_interface.superstate.response_state).to(equal('OPEN'))

            self.cnc.superstate.coordinator.superstate.task.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'COMPLETE')
            
            self.cnc.superstate.coordinator.superstate.task.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
            time.sleep(0.2)
            self.cnc.superstate.coordinator.superstate.task.superstate.subTask.superstate.event('robot', 'SubTask:Collaborator', 'MaterialUnload', 'COMPLETED')

                        




            self.cmm.superstate.collaborator.superstate.event('robot', 'SubTask:Collaborator', 'request' , 'START', self.master_uuid,text = 'r1')

            time.sleep(0.2)
            self.cmm.superstate.collaborator.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')

            time.sleep(1)
            
            self.cmm.superstate.collaborator.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
            
            self.cmm.superstate.collaborator.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')
            time.sleep(0.2)
            self.cmm.superstate.collaborator.superstate.subTask.superstate.event('robot', 'SubTask:Collaborator', 'MaterialLoad', 'COMPLETED')

            self.cnc.superstate.coordinator.superstate.event('cmm', 'SubTask:Collaborator', 'state', 'COMPLETE', code = self.master_uuid, text = 'cmm1')

            time.sleep(0.2)

            expect(self.cmm.superstate.state).to(equal('base:operational:cycle_start'))

            expect(self.cmm.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))

            

            
                        
            time.sleep(0.2)
            
            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))

            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))

            
            expect(self.cnc.superstate.binding_state).to(equal('INACTIVE'))


    with context('when from cnc to buffer'):

        with before.each:
            self.information_model = {'coordinator':{'cnc1':{'state':['cnc', 'cnc1', None], 'Task': ['move_material', None], 'SubTask': {'cnc1':['MaterialUnload', None, 'r1','source'],'Buffer1': ['MaterialLoad', None, 'r1','destination']}}}, 'collaborators': { 'r1':{'state':['robot','r1', None], 'SubTask': { 'MaterialUnload':[['DoorInterface','Open', None,1, None], ['ChuckInterface', 'Open', None,2, None]]}}, 'Buffer1':{'state':['Buffer','Buffer1', None], 'SubTask':{}}}}
            self.master_uuid = 1
            self.cnc.create_statemachine()
            self.cnc.superstate.has_material = True
            self.cnc.superstate.master_uuid = self.master_uuid
            self.cnc.superstate.master_tasks[self.master_uuid] = self.information_model

            self.cnc.superstate.load_time_limit(6)
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
            self.cnc.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
            self.cnc.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'Close', 'READY')


            self.buffer.create_statemachine()

            self.buffer.superstate.load_time_limit(6)
            self.buffer.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

            self.buffer.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')

        with it('binding state should become inactive after moveMaterial'):

            expect(self.cnc.superstate.binding_state).to(equal('INACTIVE'))
            expect(self.buffer.superstate.binding_state).to(equal('INACTIVE'))
            
            self.cnc.superstate.event('cnc', 'Controller', 'ControllerMode', 'AUTOMATIC')
            self.buffer.superstate.event('Buffer', 'Controller', 'ControllerMode', 'AUTOMATIC')
            time.sleep(0.1)
            expect(self.cnc.superstate.binding_state).to(equal('PREPARING'))
            expect(self.buffer.superstate.collaborator.superstate.interface.value).to(equal('PREPARING'))

            
            self.buffer.superstate.collaborator.superstate.event('cnc', 'Coordinator', 'querry', 'INITIATION', code = [self.master_uuid, self.information_model], text = 'cnc1')
            
            self.cnc.superstate.coordinator.superstate.event('robot', 'Task:Collaborator', 'state', 'PREPARING', code = 1,text = 'r1')
            self.cnc.superstate.coordinator.superstate.event('Buffer', 'Task:Collaborator', 'state', 'PREPARING', code = 1,text = 'Buffer1')
            time.sleep(0.1)
            
            expect(self.cnc.superstate.binding_state).to(equal('COMMITTING'))

            self.buffer.superstate.collaborator.superstate.event('cnc', 'Coordinator', 'state' , 'COMMITTING',self.master_uuid,text = 'cnc1')
            
            self.cnc.superstate.coordinator.superstate.event('robot', 'Task:Collaborator', 'state', 'COMMITTED', code = 1,text = 'r1')
            self.cnc.superstate.coordinator.superstate.event('Buffer', 'Task:Collaborator', 'state', 'COMMITTED', code = 1,text = 'Buffer1')
            time.sleep(0.1)
            
            expect(self.cnc.superstate.binding_state).to(equal('COMMITTED'))
            
            self.buffer.superstate.collaborator.superstate.event('cnc', 'Coordinator', 'state' , 'COMMITTED',self.master_uuid,text = 'cnc1')









            self.cnc.superstate.coordinator.superstate.task.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'ACTIVE')
            self.cnc.superstate.coordinator.superstate.task.superstate._subTask.superstate.event('robot', 'DoorInterface', 'Open', 'ACTIVE')
            
            time.sleep(1.1)
            self.cnc.superstate.coordinator.superstate.task.superstate._subTask.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
            self.cnc.superstate.coordinator.superstate.task.superstate._subTask.superstate.event('robot', 'SubTask:Collaborator', 'DOOR_OPEN', 'COMPLETED')
            
            

            self.cnc.superstate.coordinator.superstate.task.superstate._subTask.superstate.event('robot', 'ChuckInterface', 'Open', 'ACTIVE')

            time.sleep(1.1)
            self.cnc.superstate.coordinator.superstate.task.superstate._subTask.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
            self.cnc.superstate.coordinator.superstate.task.superstate._subTask.superstate.event('robot', 'SubTask:Collaborator', 'CHUCK_OPEN', 'COMPLETED')


            expect(self.cnc.superstate.open_door_interface.superstate.response_state).to(equal('OPEN'))

            expect(self.cnc.superstate.open_chuck_interface.superstate.response_state).to(equal('OPEN'))

            self.cnc.superstate.coordinator.superstate.task.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'COMPLETE')
            
            self.cnc.superstate.coordinator.superstate.task.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
            time.sleep(0.2)
            self.cnc.superstate.coordinator.superstate.task.superstate.subTask.superstate.event('robot', 'SubTask:Collaborator', 'MaterialUnload', 'COMPLETED')

                        




            self.buffer.superstate.collaborator.superstate.event('robot', 'SubTask:Collaborator', 'request' , 'START', self.master_uuid,text = 'r1')

            time.sleep(0.2)
            self.buffer.superstate.collaborator.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'ACTIVE')

            time.sleep(1)
            
            self.buffer.superstate.collaborator.superstate.subTask.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'COMPLETE')
            

            time.sleep(0.2)
            self.buffer.superstate.collaborator.superstate.subTask.superstate.event('robot', 'SubTask:Collaborator', 'MaterialLoad', 'COMPLETED')

            self.cnc.superstate.coordinator.superstate.event('Buffer', 'SubTask:Collaborator', 'state', 'COMPLETE', code = self.master_uuid, text = 'Buffer1')

            time.sleep(0.2)

            expect(self.buffer.superstate.state).to(equal('base:operational:idle'))
            

            
                        
            time.sleep(0.2)
            
            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))

            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))

            
            expect(self.cnc.superstate.binding_state).to(equal('INACTIVE'))
            

            





            
