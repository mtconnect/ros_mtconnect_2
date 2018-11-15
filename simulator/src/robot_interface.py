from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import time

class RobotInterface:
    """
    A mock robot interface to use in offline testing. Also specifies the functions the real
    interface must implement.
    """
    def __init__(self, sim = True):
        self.sim = sim

    def move_in(self, device, destination):
        print("Robot moving into device={}, dest={}".format(device, destination))
        time.sleep(2)
        return True

    def move_out(self, device, destination):
        print("Robot moving out of device={}, dest={}".format(device, destination))
        time.sleep(2)
        return True

    def grab(self, device, destination):
        print("Robot grabbing at device={}".format(device))
        time.sleep(2)
        return True

    def release(self, device, destination):
        print("Robot releasing at device={}".format(device))
        time.sleep(2)
        return True
