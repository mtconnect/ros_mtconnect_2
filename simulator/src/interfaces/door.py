from response import *
from request import *


def OpenDoor(parent, simulate = True):
    OpenDoor = Response(parent, parent.adapter, parent.open_door, 'door', 'OPEN', 'UNLATCHED', parent.door_state, rel = True, simulate = simulate)
    OpenDoor.create_statemachine()
    OpenDoor.superstate.start()
    return OpenDoor

def OpenDoorRequest(parent):
    OpenDoor = Request(parent, parent.adapter, parent.open_door, rel = True)
    OpenDoor.create_statemachine()
    OpenDoor.superstate.start()
    return OpenDoor

def CloseDoor(parent, simulate = True):
    CloseDoor = Response(parent, parent.adapter, parent.close_door, 'door', 'CLOSED', 'UNLATCHED',parent.door_state, rel = True, simulate = simulate)
    CloseDoor.create_statemachine()
    CloseDoor.superstate.start()
    return CloseDoor

def CloseDoorRequest(parent):
    CloseDoor = Request(parent, parent.adapter, parent.close_door, rel = True)
    CloseDoor.create_statemachine()
    CloseDoor.superstate.start()
    return CloseDoor
