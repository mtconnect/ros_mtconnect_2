from response import *
from request import *

"""Request and Response Interfaces for a Door"""

def OpenDoor(parent, simulate = True):
    OpenDoor = Response(
        parent = parent,
        adapter = parent.adapter,
        interface = parent.open_door,
        prefix = 'door',
        dest_state = 'OPEN',
        transition_state = 'UNLATCHED',
        response_state = parent.door_state,
        rel = True,
        simulate = simulate
        )
    OpenDoor.superstate.start()
    return OpenDoor

def OpenDoorRequest(parent):
    OpenDoor = Request(
        parent = parent,
        adapter = parent.adapter,
        interface = parent.open_door,
        rel = True
        )
    OpenDoor.superstate.start()
    return OpenDoor

def CloseDoor(parent, simulate = True):
    CloseDoor = Response(
        parent = parent,
        adapter = parent.adapter,
        interface = parent.close_door,
        prefix = 'door',
        dest_state = 'CLOSED',
        transition_state = 'UNLATCHED',
        response_state = parent.door_state,
        rel = True,
        simulate = simulate
        )
    CloseDoor.superstate.start()
    return CloseDoor

def CloseDoorRequest(parent):
    CloseDoor = Request(
        parent = parent,
        adapter = parent.adapter,
        interface = parent.close_door,
        rel = True
        )
    CloseDoor.superstate.start()
    return CloseDoor
