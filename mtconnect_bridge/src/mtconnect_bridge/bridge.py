from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import rospy
import mtconnect_msgs.msg

from simulator.src import request, response

class Request:
    def __init__(self):
        self.request = request.Request('parent', 'adapter', request.interface(), True)

        #Initialize the state machine
        self.request.create_statemachine()
        self.request.superstate.start()

class Response:
    def __init__(self):
        self.material_load_response = response.Response('parent', 'adapter', response.interface(), 'door', 'OPEN', 'UNLATCHED', True, simulate= True)

class Bridge:
    def __init__(self):
        pass

    def spin(self):
        pass
