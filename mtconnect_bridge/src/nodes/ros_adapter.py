#!/usr/bin/env python
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
    
import mtconnect_bridge
from data_item import Event, Sample
from mtconnect_adapter import Adapter

import roslib, rospy

from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray

class ROSAdapter:
    def __init__(self):
        # Get all of the ROS Params
        self.address = rospy.get_param('~address')
        self.port = rospy.get_param('~port')
        self.joint_topic = rospy.get_param('~joint_topic')
        self.joint_names = rospy.get_param('~joint_data_items')
        self.gripper_topic = rospy.get_param('~gripper_topic')
        self.gripper_names = rospy.get_param('~gripper_data_items')

        # Start the adapter
        self.adapter = Adapter((self.address, self.port))
        self.adapter.start()

        # Start ROS Subscribers
        rospy.Subscriber(self.joint_topic, JointState, self.joint_callback)

        # Create Gripper
        self.gripper = self.create_event_item('gripper_state', self.adapter)
        self.gripper.set_value("INITIALIZED")

        # Create Joint data items
        self.joint_data_items = []
        for name in self.joint_names:
            self.joint_data_items.append(self.create_sample_item(name, self.adapter))



    # Creates a "Sample" type data item
    def create_sample_item(self, name, adapt):
        tmp = Sample(name)
        self.adapter.add_data_item(tmp)
        return tmp

    # Creates an Event type data item
    def create_event_item(self, name, adapt):
        tmp = Event(name)
        self.adapter.add_data_item(tmp)
        return tmp


    # Update joint values
    def joint_callback(self, msg):
        # check if the lengths are correct
        if len(self.joint_names) != len(msg.position):
            rospy.logwarn('Mismatched number of joint names and joint values check mtconnect publisher yaml file')
            return
        self.adapter.begin_gather()
        ind = 0
        #Assign joint values to the associated data item
        for name in self.joint_names:
            self.joint_data_items[ind].set_value((msg.position[ind]))
            ind = ind + 1
        self.adapter.complete_gather()

    # Update gripper status
    def gripper_callback(self, msg):
        self.gripper.set_value('IDLE')  #Input update here


if __name__ == "__main__":


    rospy.init_node('mtconnect_adapter')
    ros_adapter = ROSAdapter()

    rospy.spin()

    
