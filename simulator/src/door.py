from request import Request
from response import Response

def OpenDoor(parent):
    OpenDoor = Response(parent, 'adapter', parent.open_door, 'door', 'OPEN', 'UNLATCHED', rel = True, simulate = True)
    OpenDoor.create_statemachine()
    OpenDoor.superstate.start()
    return OpenDoor

def OpenDoorRequest(parent):
    OpenDoor = Request(parent, "adapter", parent.open_door, rel = True)
    OpenDoor.create_statemachine()
    OpenDoor.superstate.start()
    return OpenDoor

def CloseDoor(parent):
    CloseDoor = Response(parent, 'adapter', parent.close_door, 'door', 'CLOSED', 'UNLATCHED', rel = True, simulate = True)
    CloseDoor.create_statemachine()
    CloseDoor.superstate.start()
    return CloseDoor

def CloseDoorRequest(parent):
    CloseDoor = Request(parent, "adapter", parent.close_door, rel = True)
    CloseDoor.create_statemachine()
    CloseDoor.superstate.start()
    return CloseDoor
