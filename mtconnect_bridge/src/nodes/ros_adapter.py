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

sys.path.append(os.path.join(os.getenv('HOME'), 'catkin_ws/src/ceccrebot/'))
sys.path.append(os.path.join(os.getenv('HOME'), 'catkin_ws/src/ceccrebot/simulator/src'))

from data_item import Event, SimpleCondition, Sample
from mtconnect_adapter import Adapter

import roslib, rospy

from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray


adapter = Adapter(('127.0.0.1', 7997))

def create_item(name, adapt):
    tmp = Sample(name)
    adapter.add_data_item(tmp)
    return tmp

# Joints
J1_6 = create_item('j1_angle', adapter)
J2_9 = create_item('j2_angle', adapter)
J3_12 = create_item('j3_angle', adapter)
J4_15 = create_item('j4_angle', adapter)
J5_18 = create_item('j5_angle', adapter)
J6_21 = create_item('j6_angle', adapter)

# Gripper
gripper = create_item('gripper_state', adapter)

# Controller
controller_mode = create_item('mode', adapter)
execution = create_item('exec', adapter)


# Update joint values
def callback(msg):
    adapter.begin_gather()
    J1_6.set_value((msg.position[0]))
    J2_9.set_value((msg.position[1]))
    J3_12.set_value((msg.position[2]))
    J4_15.set_value((msg.position[3]))
    J5_18.set_value((msg.position[4]))
    J6_21.set_value((msg.position[5]))
    adapter.complete_gather()

# Update gripper status
def gripper_callback(msg):
    gripper.set_value('IDLE')  #Input update here


if __name__ == "__main__":
    adapter.start()
    gripper.set_value("INITIALIZED")

    rospy.init_node('mtconnect_adapter')
    rospy.Subscriber('/ur/joint_states', JointState, callback)

    rospy.spin()

    
