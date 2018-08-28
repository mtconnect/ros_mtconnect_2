"""Copyright 2012, System Insights, Inc.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License."""
    
import sys, os, time

sys.path.append(os.path.join(os.getenv('HOME'), 'mtconnect_dev/src/ceccrebot/'))
sys.path.append(os.path.join(os.getenv('HOME'), 'mtconnect_dev/src/ceccrebot/simulator/src'))

from data_item import Event, SimpleCondition, Sample
from mtconnect_adapter import Adapter

import roslib

import rospy
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray



def create_item(name, adapt):
    tmp = Sample(name)
    adapter.add_data_item(tmp)
    return tmp

adapter = Adapter(('127.0.0.1', 7996))

# Joints
J1_6 = create_item('J1_6', adapter)
J2_9 = create_item('J2_9', adapter)
J3_12 = create_item('J3_12', adapter)
J4_15 = create_item('J4_15', adapter)
J5_18 = create_item('J5_18', adapter)
J6_21 = create_item('J6_21', adapter)

# Path
controller_mode = create_item('task1_25', adapter)
execution = create_item('task1_26', adapter)

#Gripper
gripper = create_item('gripper_29', adapter)



# Update joint values
def callback(msg):
    adapter.begin_gather()
    J1_6.set_value((msg.position[0] ))
    J2_9.set_value((msg.position[1]))
    J3_12.set_value((msg.position[2]))
    J4_15.set_value((msg.position[3]))
    J5_18.set_value((msg.position[4]))
    J6_21.set_value((msg.position[5]))
    adapter.complete_gather()

# Update Path status
def status_callback(msg):
    if msg.status_list:
        execution.set_value(msg.status_list[-1].status)

# Update gripper status
def gripper_callback(msg):
    gripper.set_value('IDLE')

if __name__ == "__main__":
    avail = Event('avail')
    adapter.add_data_item(avail)
    avail.set_value('AVAILABLE')
    controller_mode.set_value('AUTOMATIC')



#    adapter.start()
    rospy.init_node('mtconnect_adapter')
    rospy.Subscriber('/ur/joint_states', JointState, callback)
    rospy.Subscriber('/ur/execute_trajectory/status', GoalStatusArray, status_callback)
    rospy.spin()

    
