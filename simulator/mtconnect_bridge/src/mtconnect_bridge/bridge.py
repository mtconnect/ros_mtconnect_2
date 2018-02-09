from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import rospy
import mtconnect_msgs.msg

import simulator.src

class Bridge:
    def __init__(self):
        print('Creating a MTConnect-ROS bridge')

    def spin(self):
        print('Spinning...')
