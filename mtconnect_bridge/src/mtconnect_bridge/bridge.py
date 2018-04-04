from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import rospy
import actionlib
from mtconnect_bridge.msg import DeviceWorkAction, DeviceWorkGoal

from simulator.src.binding import robot

ACTIONS = [
    'move',
    'grab',
    'release',
]

class Bridge:
    def __init__(self):
        self.action_clients = {}
        self.current_work = {}
        for action in ACTIONS:
            self.action_clients[action] = actionlib.SimpleActionClient(action, DeviceWorkAction)
            self.current_work[action] = None
            #Work done callback

    def do_work(self, action_type, work_info):
        action_goal = DeviceWorkGoal()
        action_goal.type = action_type
        action_goal.data = work_info
        self.current_work[action_type] = action_goal
        self.action_clients[action_type].send_goal(action_goal)

    def wait_for_work(self, action_type):
        self.action_clients[action_type].wait_for_result(rospy.Duration.from_sec(10.0))
        self.current_work[action_type] = None

    def is_working(self, action_type):
        return self.current_work[action_type] != None

    def work_done_cb(self, action_type, action_msg):
        self.current_work[action_type] = None

    def spin(self):
        rospy.spin()
