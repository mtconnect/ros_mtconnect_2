from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import rospy
import actionlib
from mtconnect_bridge.msg import DeviceWorkAction, DeviceWorkGoal

from simulator.src import request, response

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

    def spin(self):
        pass
