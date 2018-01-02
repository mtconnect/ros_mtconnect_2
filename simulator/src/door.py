from response import *


#create parent centrally

OpenDoor = Response('cnc', 'cnc.adapter', interface, 'door', 'OPEN', 'UNLATCHED', True, simulate= True)
OpenDoor.create_statemachine()
OpenDoor.superstate.start()


"""
CloseDoor = Response('cnc', 'cnc.adapter', cnc.close_door, 'door', 'CLOSED', 'UNLATCHED', True, simulate= True)
CloseDoor.create_statemachine()
CloseDoor.superstate.start()
"""
