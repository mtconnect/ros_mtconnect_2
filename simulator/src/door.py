from response import *


def OpenDoor(parent):
    OpenDoor = Response(parent, 'adapter', parent.open_door, 'door', 'OPEN', 'UNLATCHED', rel = True, simulate = True)
    OpenDoor.create_statemachine()
    OpenDoor.superstate.start()
    return OpenDoor

def CloseDoor(parent):
    CloseDoor = Response(parent, 'adapter', parent.close_door, 'door', 'CLOSED', 'UNLATCHED', rel = True, simulate = True)
    CloseDoor.create_statemachine()
    CloseDoor.superstate.start()
    return CloseDoor
