#from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import sys
import os
import time
from robot import Robot
#import rospy
#import mtconnect_bridge
#from simulator.src import Robot

devices = ['conv1','cnc1','b1','cmm1','t1']
destinations = ['good', 'bad', 'rework']

class RobotInterface:
    def __init__(self, sim = True):
        self.sim = sim

    def move_in(self, device, destination):
        time.sleep(2)
        return True 
        
    def move_out(self, device, destination):
        time.sleep(2)
        return True

    def grab(self, device):
        time.sleep(2)
        return True

    def release(self, device):
        time.sleep(2)
        return True



def main():
    robot_state_machine = Robot(host, port, RobotInterface())

if __name__ == '__main__':
    main()
