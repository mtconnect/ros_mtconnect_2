#----FYI----
device_priority_ratings = {}
device_priority_ratings['conv1'] = '4'
device_priority_ratings['cnc1'] = '14'
device_priority_ratings['cmm1'] = '6'
device_priority_ratings['b1'] = '2'

part_quality_ratings = {}
part_quality_ratings[None] = '5'
part_quality_ratings['good'] = '5'
part_quality_ratings['bad'] = '12'
part_quality_ratings['rework'] = '15'

#----FYI----

from copy import deepcopy
from threading import Thread, Timer
import time, datetime

class priority(object):

    def __init__(self, parent, interface):
        self.tasks_list = []
        self.tasks_check = []
        self.tasks_pop_check = []
        self.binding_states = {}
        self.initiate_binding_state()
        self.parent = parent
        self.interface = interface
        self.priority_task = None
	self.collaborators = []
	self.current_collaborators = []

    def event_list(self, event):
        if event:
            #[task_priority, assetID, device_collaborators, event]
            task_list = [float(event[4][1]['priority']), event[4][0], event[4][1]['coordinator'].keys()+event[4][1]['collaborators'].keys(), event]
            if task_list[1] not in str(self.tasks_list) and self.parent.deviceUuid in task_list[2]:
                self.tasks_list.append(task_list)
                self.tasks_check.append([task_list, datetime.datetime.now().isoformat()])
                self.event_priority_update()

                self.collab_check2()
		self.commit_check()

    def event_priority_update(self):
        for i,x in enumerate(self.tasks_list):
            task = deepcopy(self.tasks_list[i])
            task[0] = 2*x[0]
            self.tasks_list[i] = task


    def priority_event(self):
        if self.parent.binding_state_material.value().lower() != 'inactive' or self.parent.iscoordinator or self.parent.e1.value().lower()=='active':
            self.priority_task = "task_queued"
        else:
            self.priority_task = None
            self.tasks_list.sort(reverse = True)
            for i,x in enumerate(self.tasks_list):
                devices_avail = False
                for y in x[2]:
                    if (not self.binding_states[y][0] or self.binding_states[y][0].lower() not in ['committing','committed']) and self.parent.execution[y] != 'active':
                        if self.binding_states[y][2] and y != x[2][0] and y!='conv1':
                            devices_avail = False
                            break
			elif self.binding_states[y][1] and self.binding_states[y][0]:
                            if self.binding_states[y][1] in str(self.tasks_list) and self.binding_states[y][1] != x[1] and y!=self.parent.deviceUuid:
                                devices_avail = False
                                print ("device uuid check fail",self.parent.deviceUuid,y,self.binding_states[y][1],x[1])
                                break
                    else:
                        devices_avail = False
                        break
                    devices_avail = True
                    
                if devices_avail and self.tasks_list:
                    self.priority_task = deepcopy(self.tasks_list[i][3])
		    self.collaborators = deepcopy(self.tasks_list[i][2])
                    self.tasks_list.pop(i)
                    if self.priority_task[4][0] in str(self.tasks_pop_check):
			print ("event already popped once")
                        self.priority_event()
                    self.tasks_pop_check.append([x[3], datetime.datetime.now().isoformat()])
                    break
            

    def collab_check2(self):
        if self.tasks_list and self.parent.iscollaborator and not self.parent.iscoordinator and self.parent.collaborator.superstate.state == 'base:inactive' and self.parent.binding_state_material.value().lower() == 'inactive':
            self.priority_event()
            if self.priority_task and self.priority_task != 'task_queued':
                event = self.priority_task
                self.parent.adapter.begin_gather()
                self.interface.set_value(event[4][0])
                self.parent.adapter.complete_gather()
                time.sleep(0.1)
                self.parent.master_uuid = event[4][0].lower()
                self.parent.collaborator.superstate.event(event[0],event[1],event[2],event[3],event[4],event[5])

    def collab_check(self):
        def wait():
            if self.parent.binding_state_material.value().lower() == 'inactive':

                timer = Timer(2, self.collab_check2)
                timer.start()

                while timer.isAlive():
                    if self.parent.binding_state_material.value().lower() != 'inactive':
                        timer.cancel()
			break

        thread= Thread(target = wait)
        thread.start()

    def commit_check(self):
	def all_commit():
	    self.current_collaborators = self.parent.master_tasks[self.parent.master_uuid]['coordinator'].keys()+self.parent.master_tasks[self.parent.master_uuid]['collaborators'].keys()
	    current_uuid = deepcopy(self.parent.master_uuid)
	    check = False
	    time.sleep(5)
	    while self.parent.binding_state_material.value().lower() == 'committed':
		if self.binding_states['r1'][1].lower() != self.parent.master_uuid or self.binding_states['r1'][0].lower() == 'inactive':
		    time.sleep(10)
		    print ("task stuck, no success",self.binding_states['r1'][1].lower())
		    if self.parent.binding_state_material.value().lower() == 'committed' and self.parent.iscoordinator and current_uuid == self.parent.master_uuid:
		        self.parent.coordinator.superstate.task.superstate.success()
			print ("task committed from priority method",self.parent.deviceUuid)
		    check = True
		else:
		    for x in self.current_collaborators:
			if x not in [self.parent.deviceUuid,'r1']:
			    if self.binding_states[x][1].lower() != self.parent.master_uuid:
				time.sleep(10)
				print ("task stuck2, no success",self.binding_states[x][1].lower())
	                        if self.parent.binding_state_material.value().lower() == 'committed' and self.parent.iscoordinator and current_uuid == self.parent.master_uuid:
            		            self.parent.coordinator.superstate.task.superstate.success()
                        	    print ("task committed from priority method2",self.parent.deviceUuid)
				check = True

		if check == True:
		    break
		else:
		    pass

	if self.parent.iscoordinator:
	    thread = Thread(target = all_commit)
	    thread.start()


    def initiate_binding_state(self):
        self.binding_states['conv1'] = [None,None,None]
        self.binding_states['cnc1'] = [None,None,None]
        self.binding_states['cmm1'] = [None,None,None]
        self.binding_states['b1'] = [None,None,None]
        self.binding_states['r1'] = [None,None,None]
        
    def binding_state(self, device = None, state = None, binding = None, has_material = None):
        if device:
            if state:
                self.binding_states[device][0] = state
            if binding:
                self.binding_states[device][1] = binding
            if has_material != None:
                self.binding_states[device][2] = has_material

        


if __name__ == '__main__':
    a=priority()
    
    
    a.event_list(['cnc','Coordinator', 'binding_state','PREPARING',['cnc1_123',{'priority': '20', 'coordinator': {'cnc1': {'state': ['cnc', 'cnc1', None], 'Task': ['move_material', None], 'SubTask': {'b1': ['LoadBuffer', None, 'r1', 'MaterialLoad', '2'], 'r1': [], 'cnc1': ['UnloadCnc', None, 'r1', 'MaterialUnload', '1']}}}, 'collaborators': {'r1': {'state': ['ROBOT', 'r1', None], 'SubTask': {'UnloadCnc': [['Interface', 'OpenDoor', None, '1', ['cnc1']], ['Interface', 'MoveIn', None, '2', None], ['Interface', 'GrabPart', None, '3', None], ['Interface', 'OpenChuck', None, '4', ['cnc1']], ['Interface', 'MoveOut', None, '5', None], ['Interface', 'CloseDoor', None, '6', ['cnc1']]], 'LoadBuffer': [['Interface', 'MoveIn', None, '1', None], ['Interface', 'ReleasePart', None, '2', None], ['Interface', 'MoveOut', None, '3', None]]}}, 'b1': {'state': ['BUFFER', 'b1', None], 'SubTask': {}}}, 'part_quality': None}],'cnc1'])
    a.event_list(['cmm','Coordinator', 'binding_state','PREPARING',['cmm1_12de',{'priority': '100', 'coordinator': {'cmm1': {'state': ['cmm', 'cmm1', None], 'Task': ['move_material', None], 'SubTask': {'r1': [], 'cmm1': ['UnloadCmm', None, 'r1', 'MaterialUnload', '1'], 'cnc1': ['LoadCnc', None, 'r1', 'MaterialLoad', '2']}}}, 'collaborators': {'r1': {'state': ['ROBOT', 'r1', None], 'SubTask': {'UnloadCmm': [['Interface', 'MoveIn', None, '1', None], ['Interface', 'GrabPart', None, '2', None], ['Interface', 'MoveOut', None, '3', None]], 'LoadCnc': [['Interface', 'OpenDoor', None, '1', ['cnc1']], ['Interface', 'MoveIn', None, '2', None], ['Interface', 'CloseChuck', None, '3', ['cnc1']], ['Interface', 'ReleasePart', None, '4', None], ['Interface', 'MoveOut', None, '5', None], ['Interface', 'CloseDoor', None, '6', ['cnc1']]]}}, 'cnc1': {'state': ['CNC', 'cnc1', None], 'SubTask': {}}}, 'part_quality': None}],'cmm1'])
    a.event_list(['buffer','Coordinator','binding_state','PREPARING',['b1_1222',{'priority': '40', 'coordinator': {'b1': {'state': ['buffer', 'b1', None], 'Task': ['move_material', None], 'SubTask': {'b1': ['UnloadBuffer', None, 'r1', 'MaterialUnload', '1'], 'cmm1': ['LoadCmm', None, 'r1', 'MaterialLoad', '2'], 'r1': []}}}, 'collaborators': {'r1': {'state': ['ROBOT', 'r1', None], 'SubTask': {'LoadCmm': [['Interface', 'MoveIn', None, '1', None], ['Interface', 'ReleasePart', None, '2', None], ['Interface', 'MoveOut', None, '3', None]], 'UnloadBuffer': [['Interface', 'MoveIn', None, '1', None], ['Interface', 'GrabPart', None, '2', None], ['Interface', 'MoveOut', None, '3', None]]}}, 'cmm1': {'state': ['CMM', 'cmm1', None], 'SubTask': {}}}, 'part_quality': None}],'b1'])

    print (a.priority_event())

    
        
        
