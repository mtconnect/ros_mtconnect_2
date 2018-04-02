from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import rospy
import actionlib
from mtconnect_bridge.msg import DeviceWorkAction, DeviceWorkGoal

from simulator.src import robot

ACTIONS = [
    'move',
    'grab',
    'release',
]

class Bridge:
    def __init__(self):
        self.action_clients = {}
        for action in ACTIONS:
            self.action_clients[action] = actionlib.SimpleActionClient(action, DeviceWorkAction)

    def do_work(self, action_type, work_info):
        action_goal = DeviceWorkGoal()
        action_goal.type = action_type
        action_goal.data = work_info
        self.action_clients[action_type].send_goal(action_goal)
        self.action_clients[action_type].wait_for_result(rospy.Duration.from_sec(10.0))

    def spin(self):
        rospy.spin()
