from expects import *
from doublex import *
import threading

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
            self.information_model = {
                'coordinator': {
                    'inputConveyor1': {
                        'state': ['inputConveyor', 'inputConveyor1', None],
                        'Task': ['move_material', None],
                        'SubTask': {
                            'r1':[],
                            'inputConveyor1':['MaterialUnload', None, 'r1','source'],
                            'cnc1': ['MaterialLoad', None, 'r1','destination']
                        }
                    }
                },
                'collaborators': {
                    'r1': {
                        'state':['robot','r1', None],
                        'SubTask': {
                            'MaterialLoad':[
                                ['ChuckInterface','Close', None,1, None],
                                ['DoorInterface', 'Close', None,2, None]
                            ]
                        }
                    },
                    'cnc1': {
                        'state': ['cnc','cnc1', None],
                        'SubTask':{}
                    }
                }
            }

            #initializing input conveyor statemachine: source
            self.inputConveyor.create_statemachine()
            self.inputConveyor.superstate.has_material = True
            self.inputConveyor.superstate.master_uuid = self.master_uuid
            self.inputConveyor.superstate.master_tasks[self.master_uuid] = self.information_model
            self.inputConveyor.superstate.unload_time_limit(10)
            self.inputConveyor.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')

            #for cascade failure
            self.inputConveyor.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

            #intializing cnc statemachine: destination
            self.cnc.create_statemachine()
            self.cnc.superstate.has_material = False
            self.cnc.superstate.load_time_limit(10)
            self.cnc.superstate.unload_time_limit(10)
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
            self.cnc.superstate.event('inputConveyor', 'Coordinator', 'binding_state', 'PREPARING', code = [self.master_uuid, self.information_model], text = 'inputConveyor1')
            #self.robot.superstate.event('inputConveyor', 'Coordinator', 'binding_state', 'PREPARING', code = [self.master_uuid, self.information_model], text = 'inputConveyor1')

            #all the collaborators report back their interest and state
            self.inputConveyor.superstate.event('robot', 'Task_Collaborator', 'binding_state', 'PREPARING', code = self.master_uuid, text = 'r1')
            self.inputConveyor.superstate.event('cnc', 'Task_Collaborator', 'binding_state', 'PREPARING', code = self.master_uuid, text = 'cnc1')
            time.sleep(0.1)

            expect(self.inputConveyor.superstate.binding_state).to(equal('COMMITTING'))


            #conveyor:coordinator starts to commmit and is reported to the collaborators
            self.cnc.superstate.event('inputConveyor', 'Coordinator', 'binding_state' , 'COMMITTING', code = self.master_uuid , text = 'inputConveyor1')

            #collaborators start to commit and report back to the coordinator when committed
            self.inputConveyor.superstate.event('robot', 'Task_Collaborator', 'binding_state', 'COMMITTED', code = self.master_uuid, text = 'r1')
            self.inputConveyor.superstate.event('cnc', 'Task_Collaborator', 'binding_state', 'COMMITTED', code = self.master_uuid, text = 'cnc1')
            time.sleep(0.1)

            expect(self.inputConveyor.superstate.binding_state).to(equal('COMMITTED'))

            #coordinator reports that its committed too and the task:subtask execution begin.
            self.cnc.superstate.event('inputConveyor', 'Coordinator', 'binding_state' , 'COMMITTED',code = self.master_uuid, text = 'inputConveyor1')



            #Task starts
            #subtask_unload starts

            self.inputConveyor.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE', code = self.master_uuid, text = 'r1')
            time.sleep(1.1)

            self.inputConveyor.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE', code = self.master_uuid, text = 'r1')

            #subtask_unload completed
            time.sleep(0.2)
            expect(self.inputConveyor.superstate.state).to(equal('base:operational:idle'))

            #robot has finished unload and requests subtask collaborator to start its subtask execution

            time.sleep(0.2)
            #subtask_load starts
            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE', code = self.master_uuid, text = 'r1')

            time.sleep(1)

            #subtask_chuck_close starts

            self.cnc.superstate.event('robot', 'ChuckInterface', 'SubTask_Close', 'ACTIVE', code = self.master_uuid, text = 'r1')

            time.sleep(1.1)
            #subtask_chuck_close completed

            self.cnc.superstate.event('robot', 'ChuckInterface', 'SubTask_Close', 'READY', code = self.master_uuid, text = 'r1')


            self.cnc.superstate.event('robot', 'ChuckInterface', 'SubTask_Close', 'COMPLETED', code = self.master_uuid, text = 'r1')

            time.sleep(0.2)
            #subtask_door_close starts

            self.cnc.superstate.event('robot', 'DoorInterface', 'SubTask_Close', 'ACTIVE', code = self.master_uuid, text = 'r1')

            time.sleep(1.1)

            self.cnc.superstate.event('robot', 'DoorInterface', 'SubTask_Close', 'READY', code = self.master_uuid, text = 'r1')
            self.cnc.superstate.event('robot', 'DoorInterface', 'SubTask_Close', 'COMPLETED', code = self.master_uuid, text = 'r1')

            #subtask_door_close completed

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE', code = self.master_uuid, text = 'r1')
            time.sleep(0.3)

            expect(self.cnc.superstate.close_door_interface.superstate.response_state).to(equal('CLOSED'))

            expect(self.cnc.superstate.close_chuck_interface.superstate.response_state).to(equal('CLOSED'))

            time.sleep(0.2)
            #subtask_load completed

            self.inputConveyor.superstate.event('cnc', 'Task_Collaborator', 'state', 'COMPLETE', code = self.master_uuid, text = 'cnc1')

            #the robot reports that it has completed tasks : not yet implemented!
            #self.inputConveyor.superstate.event('robot', 'Task_Collaborator', 'state', 'COMPLETE', code = self.master_uuid, text = 'r1')


            #Task completed
            time.sleep(0.2)

            expect(self.cnc.superstate.state).to(equal('base:operational:cycle_start'))

            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))




            time.sleep(0.2)

            expect(self.inputConveyor.superstate.binding_state).to(equal('INACTIVE'))


    with context('when from cnc to cmm'):

        with before.each:
            self.information_model = {'coordinator':{'cnc1':{'state':['cnc', 'cnc1', None], 'Task': ['move_material', None], 'SubTask': {'r1':[], 'cnc1':['MaterialUnload', None, 'r1','source'],'cmm1': ['MaterialLoad', None, 'r1','destination']}}}, 'collaborators': { 'r1':{'state':['robot','r1', None], 'SubTask': { 'MaterialUnload':[['DoorInterface','Open', None,1, None], ['ChuckInterface', 'Open', None,2, None]]}}, 'cmm1':{'state':['cmm','cmm1', None], 'SubTask':{}}}}
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
            time.sleep(0.2)
            expect(self.cnc.superstate.binding_state).to(equal('PREPARING'))
            expect(self.cmm.superstate.collaborator.superstate.interface.value).to(equal('PREPARING'))

            self.cmm.superstate.event('cnc', 'Coordinator', 'binding_state', 'PREPARING', code = [self.master_uuid, self.information_model], text = 'cnc1')

            self.cnc.superstate.event('robot', 'Task_Collaborator', 'binding_state', 'PREPARING', code = self.master_uuid, text = 'r1')
            self.cnc.superstate.event('cmm', 'Task_Collaborator', 'binding_state', 'PREPARING', code = self.master_uuid, text = 'cmm1')
            time.sleep(0.1)

            expect(self.cnc.superstate.binding_state).to(equal('COMMITTING'))

            self.cmm.superstate.event('cnc', 'Coordinator', 'binding_state' , 'COMMITTING', code = self.master_uuid , text = 'cnc1')

            self.cnc.superstate.event('robot', 'Task_Collaborator', 'binding_state', 'COMMITTED', code = self.master_uuid, text = 'r1')
            self.cnc.superstate.event('cmm', 'Task_Collaborator', 'binding_state', 'COMMITTED', code = self.master_uuid, text = 'cmm1')
            time.sleep(0.1)

            expect(self.cnc.superstate.binding_state).to(equal('COMMITTED'))

            self.cmm.superstate.event('cnc', 'Coordinator', 'binding_state' , 'COMMITTED',code = self.master_uuid, text = 'cnc1')









            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE', code = self.master_uuid, text = 'r1')

            self.cnc.superstate.event('robot', 'DoorInterface', 'SubTask_Open', 'ACTIVE', code = self.master_uuid, text = 'r1')

            time.sleep(1.1)
            self.cnc.superstate.event('robot', 'DoorInterface', 'SubTask_Open', 'READY', code = self.master_uuid, text = 'r1')
            self.cnc.superstate.event('robot', 'DoorInterface', 'SubTask_Open', 'COMPLETED', code = self.master_uuid, text = 'r1')


            self.cnc.superstate.event('robot', 'ChuckInterface', 'SubTask_Open', 'ACTIVE', code = self.master_uuid, text = 'r1')

            time.sleep(1.1)
            self.cnc.superstate.event('robot', 'ChuckInterface', 'SubTask_Open', 'READY', code = self.master_uuid, text = 'r1')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'SubTask_Open', 'COMPLETED', code = self.master_uuid, text = 'r1')


            expect(self.cnc.superstate.open_door_interface.superstate.response_state).to(equal('OPEN'))

            expect(self.cnc.superstate.open_chuck_interface.superstate.response_state).to(equal('OPEN'))

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE', code = self.master_uuid, text = 'r1')

            time.sleep(0.2)

            self.cmm.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE', code = self.master_uuid, text = 'r1')

            time.sleep(1)

            self.cmm.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE', code = self.master_uuid, text = 'r1')

            self.cmm.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialLoad', 'READY', code = self.master_uuid, text = 'r1')
            time.sleep(0.2)

            self.cnc.superstate.event('cmm', 'Task_Collaborator', 'state', 'COMPLETE', code = self.master_uuid, text = 'cmm1')


            time.sleep(0.2)

            expect(self.cmm.superstate.state).to(equal('base:operational:cycle_start'))

            expect(self.cmm.superstate.binding_state).to(equal('INACTIVE'))

            expect(self.cmm.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))





            time.sleep(0.2)

            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))

            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))



            expect(self.cnc.superstate.binding_state).to(equal('INACTIVE'))


    with context('when from cnc to buffer'):

        with before.each:
            self.information_model = {'coordinator':{'cnc1':{'state':['cnc', 'cnc1', None], 'Task': ['move_material', None], 'SubTask': {'r1':[], 'cnc1':['MaterialUnload', None, 'r1','source'],'Buffer1': ['MaterialLoad', None, 'r1','destination']}}}, 'collaborators': { 'r1':{'state':['robot','r1', None], 'SubTask': { 'MaterialUnload':[['DoorInterface','Open', None,1, None], ['ChuckInterface', 'Open', None,2, None]]}}, 'Buffer1':{'state':['Buffer','Buffer1', None], 'SubTask':{}}}}
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

            self.buffer.superstate.event('cnc', 'Coordinator', 'binding_state', 'PREPARING', code = [self.master_uuid, self.information_model], text = 'cnc1')

            self.cnc.superstate.event('robot', 'Task_Collaborator', 'binding_state', 'PREPARING', code = self.master_uuid, text = 'r1')
            self.cnc.superstate.event('Buffer', 'Task_Collaborator', 'binding_state', 'PREPARING', code = self.master_uuid, text = 'Buffer1')
            time.sleep(0.1)

            expect(self.cnc.superstate.binding_state).to(equal('COMMITTING'))

            self.buffer.superstate.event('cnc', 'Coordinator', 'binding_state' , 'COMMITTING', code = self.master_uuid , text = 'cnc1')

            self.cnc.superstate.event('robot', 'Task_Collaborator', 'binding_state', 'COMMITTED', code = self.master_uuid, text = 'r1')
            self.cnc.superstate.event('Buffer', 'Task_Collaborator', 'binding_state', 'COMMITTED', code = self.master_uuid, text = 'Buffer1')
            time.sleep(0.1)

            expect(self.cnc.superstate.binding_state).to(equal('COMMITTED'))

            self.buffer.superstate.event('cnc', 'Coordinator', 'binding_state' , 'COMMITTED',code = self.master_uuid, text = 'cnc1')









            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE', code = self.master_uuid, text = 'r1')

            self.cnc.superstate.event('robot', 'DoorInterface', 'SubTask_Open', 'ACTIVE', code = self.master_uuid, text = 'r1')

            time.sleep(1.1)
            self.cnc.superstate.event('robot', 'DoorInterface', 'SubTask_Open', 'READY', code = self.master_uuid, text = 'r1')
            self.cnc.superstate.event('robot', 'DoorInterface', 'SubTask_Open', 'COMPLETED', code = self.master_uuid, text = 'r1')


            self.cnc.superstate.event('robot', 'ChuckInterface', 'SubTask_Open', 'ACTIVE', code = self.master_uuid, text = 'r1')

            time.sleep(1.1)
            self.cnc.superstate.event('robot', 'ChuckInterface', 'SubTask_Open', 'READY', code = self.master_uuid, text = 'r1')
            self.cnc.superstate.event('robot', 'ChuckInterface', 'SubTask_Open', 'COMPLETED', code = self.master_uuid, text = 'r1')


            expect(self.cnc.superstate.open_door_interface.superstate.response_state).to(equal('OPEN'))

            expect(self.cnc.superstate.open_chuck_interface.superstate.response_state).to(equal('OPEN'))

            self.cnc.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE', code = self.master_uuid, text = 'r1')

            time.sleep(0.2)





            time.sleep(0.2)
            self.buffer.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE', code = self.master_uuid, text = 'r1')

            time.sleep(1)

            self.buffer.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE', code = self.master_uuid, text = 'r1')

            time.sleep(0.2)


            self.cnc.superstate.event('Buffer', 'Task_Collaborator', 'state', 'COMPLETE', code = self.master_uuid, text = 'Buffer1')


            time.sleep(0.2)

            expect(self.buffer.superstate.state).to(equal('base:operational:idle'))

            time.sleep(0.2)

            expect(self.buffer.superstate.binding_state).to(equal('INACTIVE'))

            expect(self.buffer.superstate.material_load_interface.interface.value).to(equal('NOT_READY'))





            time.sleep(0.2)

            expect(self.cnc.superstate.state).to(equal('base:operational:loading'))

            expect(self.cnc.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            expect(self.cnc.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))



            expect(self.cnc.superstate.binding_state).to(equal('INACTIVE'))


    with context('when from cmm to outputConveyor'):

        with before.each:
            self.information_model = {'coordinator':{'cmm1':{'state':['cmm', 'cmm1', None], 'Task': ['move_material', None], 'SubTask': {'r1':[], 'cmm1':['MaterialUnload', None, 'r1','source'],'outputConveyor1': ['MaterialLoad', None, 'r1','destination']}}}, 'collaborators': { 'r1':{'state':['robot','r1', None], 'SubTask': {}}, 'outputConveyor1':{'state':['outputConveyor','outputConveyor1', None], 'SubTask':{}}}}
            self.master_uuid = 1
            self.cmm.create_statemachine()
            self.cmm.superstate.has_material = True
            self.cmm.superstate.master_uuid = self.master_uuid
            self.cmm.superstate.master_tasks[self.master_uuid] = self.information_model

            self.cmm.superstate.load_time_limit(6)
            self.cmm.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

            self.cmm.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')
            self.cmm.superstate.event('robot', 'DoorInterface', 'Open', 'READY')
            self.cmm.superstate.event('robot', 'DoorInterface', 'Close', 'READY')
            self.cmm.superstate.event('robot', 'ChuckInterface', 'Open', 'READY')
            self.cmm.superstate.event('robot', 'ChuckInterface', 'Close', 'READY')


            self.outputConveyor.create_statemachine()
            self.outputConveyor.superstate.has_material = False

            self.outputConveyor.superstate.load_time_limit(6)
            self.outputConveyor.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialLoad', 'READY')

            self.outputConveyor.superstate.event('robot', 'MaterialHandlerInterface', 'MaterialUnload', 'READY')


        with it('binding state should become inactive after moveMaterial'):

            expect(self.cmm.superstate.binding_state).to(equal('INACTIVE'))
            expect(self.outputConveyor.superstate.binding_state).to(equal('INACTIVE'))

            self.cmm.superstate.event('cmm', 'Controller', 'ControllerMode', 'AUTOMATIC')
            self.outputConveyor.superstate.event('outputConveyor', 'Controller', 'ControllerMode', 'AUTOMATIC')
            time.sleep(0.1)
            expect(self.cmm.superstate.binding_state).to(equal('PREPARING'))
            expect(self.outputConveyor.superstate.collaborator.superstate.interface.value).to(equal('PREPARING'))

            self.outputConveyor.superstate.event('cmm', 'Coordinator', 'binding_state', 'PREPARING', code = [self.master_uuid, self.information_model], text = 'cmm1')

            self.cmm.superstate.event('robot', 'Task_Collaborator', 'binding_state', 'PREPARING', code = self.master_uuid, text = 'r1')
            self.cmm.superstate.event('outputConveyor', 'Task_Collaborator', 'binding_state', 'PREPARING', code = self.master_uuid, text = 'outputConveyor1')
            time.sleep(0.1)

            expect(self.cmm.superstate.binding_state).to(equal('COMMITTING'))

            self.outputConveyor.superstate.event('cmm', 'Coordinator', 'binding_state' , 'COMMITTING', code = self.master_uuid , text = 'cmm1')

            self.cmm.superstate.event('robot', 'Task_Collaborator', 'binding_state', 'COMMITTED', code = self.master_uuid, text = 'r1')
            self.cmm.superstate.event('outputConveyor', 'Task_Collaborator', 'binding_state', 'COMMITTED', code = self.master_uuid, text = 'outputConveyor1')
            time.sleep(0.1)

            expect(self.cmm.superstate.binding_state).to(equal('COMMITTED'))

            self.outputConveyor.superstate.event('cmm', 'Coordinator', 'binding_state' , 'COMMITTED',code = self.master_uuid, text = 'cmm1')







            expect(self.cmm.superstate.state).to(equal('base:operational:unloading'))
            self.cmm.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialUnload', 'ACTIVE', code = self.master_uuid, text = 'r1')

            time.sleep(1.1)

            self.cmm.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialUnload', 'COMPLETE', code = self.master_uuid, text = 'r1')

            time.sleep(0.2)
            expect(self.outputConveyor.superstate.state).to(equal('base:operational:loading'))

            self.outputConveyor.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialLoad', 'ACTIVE', code = self.master_uuid, text = 'r1')

            time.sleep(1)

            self.outputConveyor.superstate.event('robot', 'MaterialHandlerInterface', 'SubTask_MaterialLoad', 'COMPLETE', code = self.master_uuid, text = 'r1')

            time.sleep(0.2)

            self.cmm.superstate.event('outputConveyor', 'Task_Collaborator', 'state', 'COMPLETE', code = self.master_uuid, text = 'outputConveyor1')


            time.sleep(0.2)

            expect(self.outputConveyor.superstate.state).to(equal('base:operational:idle'))

            expect(self.outputConveyor.superstate.binding_state).to(equal('INACTIVE'))



            time.sleep(0.2)

            expect(self.cmm.superstate.state).to(equal('base:operational:loading'))

            expect(self.cmm.superstate.material_load_interface.interface.value).to(equal('ACTIVE'))
            expect(self.cmm.superstate.material_unload_interface.interface.value).to(equal('NOT_READY'))



            expect(self.cmm.superstate.binding_state).to(equal('INACTIVE'))







