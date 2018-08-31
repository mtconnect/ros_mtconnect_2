from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time

import task

#will be included under assets!?
class interface(object):

    def __init__(self, value = None):
        self.value = value

class coordinator(object):

    def __init__(self, parent, interface, master_task_uuid, coordinator_name): 

        class statemachineModel(object):

            def __init__(self, parent, interface, master_task_uuid, coordinator_name):

                self.interface = interface
                self.parent = parent
                self.master_task_uuid = master_task_uuid
                self.task_name = None
                self.events =[]
                self.coordinator_name = coordinator_name
                self.id = coordinator_name #which one?
                self.initialize = True

            def INACTIVE(self):
                self.parent.adapter.begin_gather()
                self.interface.set_value("INACTIVE")
                self.parent.adapter.complete_gather()
                #print "HEREEEEEEEEE"
                
                if self.initialize:
                    time.sleep(0.1)
                    self.task = task.task(parent = self.parent, interface = self.interface, master_task_uuid = self.master_task_uuid, coordinator = self)
                    self.task.create_statemachine()
                    self.task.superstate.create()

                    self.initialize = False

                    self.bind_to_task()

            def COMMITTED(self):
                self.intialize = False
                #self.parent.adapter.begin_gather()
                #self.interface.set_value("COMMITTED")
                #self.parent.adapter.complete_gather()

            def event_validity_check(self, source, comp, name, value, code, text):

                coordinator_task = None
                
                collab = self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][self.coordinator_name][2]
                taskType = self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][self.coordinator_name][0]

                if text in collab:
                    try:
                        for x in self.parent.master_tasks[code]['collaborators'][collab]['SubTask'][taskType]:
                            try:
                                if x[1] in name:
                                    coordinator_task = True
                                    break
                                else:
                                    coordinator_task = False
                            except:
                                coordinator_task = False
                    except:
                        coordinator_task = False

                return coordinator_task
                    

                
            def event(self, source, comp, name, value, code = None, text = None):
		if True:
                    #samplevent('robot_r1','collaborator','subtask_address',["open_door",'COMMITTED'],execution_lines, 'execute')
                    if comp == 'Task_Collaborator':
                        if 'binding_state' in name and value.lower() != 'inactive' and code == self.parent.master_uuid and code in self.parent.master_tasks and text in self.parent.master_tasks[code]['collaborators']:
                            try:
				self.parent.master_tasks[code]['collaborators'][text]['state'][2] = value
			    except Exception as e:
				print (e)
				time.sleep(0.5)
				print ("Master Task: "+str(self.parent.master_tasks))
				print (code+text+self.parent.master_uuid)
				self.parent.master_tasks[code]['collaborators'][text]['state'][2] = value
                            if value.lower() == 'preparing':
                                self.task.superstate.prepare()


                    elif 'SubTask' in name:
                        def commit_task():
                            if value.lower() == 'fail' or value.lower() == 'complete':
                                for key,val in self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'].iteritems():
                                    if val and (key == text or text in val[2]) and name.split('_')[-1] == val[3]:
                                        self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][key][1] = value
                                        self.task.superstate.commit()
                                    

                        t = Thread(target = commit_task)
                        t.start()
                        
                        coordinator_task = None
                    
                        if self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][self.coordinator_name][3] in name and text in self.parent.master_tasks[code]['coordinator'][self.coordinator_name]['SubTask'][self.coordinator_name][2]:
                            coordinator_task = True
                                
                        else: #done only for one collaborator at a time
                                
                            coordinator_task = self.event_validity_check(source, comp, name, value, code, text)
                                
                        if coordinator_task:
                            self.task.superstate.event(source, comp, name, value, code, text)
                    else:
                        #print "\nEVENT\n IN\n COORD\n ",source, comp, name, value, code, text
                        self.parent.event(source, comp, name, value, code, text)
		if False:
		    print ("Error processing coordinator event:")
		    print (e)
		    print ("Retrying in 1 sec")
		    time.sleep(1)
		    #self.parent.event(source, comp, name, value, code, text)

        self.superstate = statemachineModel(parent = parent, interface = interface, master_task_uuid = master_task_uuid, coordinator_name =coordinator_name)

    def create_statemachine(self):
        NestedState.separator = ':'
        states = [{'name':'base', 'children':['inactive', 'committed']}]

        transitions = [['unavailable', 'base', 'base:inactive'],

                       ['bind_to_task', 'base:inactive', 'base:committed'],

                       ['completed', 'base:committed', 'base:inactive'],
                       ['failed', 'base:committed', 'base:inactive'],

                       ['default', 'base:inactive', 'base:inactive'],
                       ['default', 'base:committed', 'base:committed']
                       ]

        self.statemachine = Machine(model = self.superstate, states = states, transitions = transitions, initial = 'base',ignore_invalid_triggers=True)
                       
        self.statemachine.on_enter('base:inactive', 'INACTIVE')
        self.statemachine.on_enter('base:committed', 'COMMITTED')

