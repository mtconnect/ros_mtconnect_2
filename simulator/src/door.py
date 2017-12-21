from response import *


#create parent centrally

OpenDoor = Response(cnc, cnc.adapter, cnc.open_door, 'door', 'OPEN', 'UNLATCHED', rel, simulate= True)
OpenDoor.create_statemachine()
OpenDoor.start()

CloseDoor = Response(cnc, cnc.adapter, cnc.open_door, 'door', 'CLOSED', 'UNLATCHED', rel, simulate= True)
CloseDoor.create_statemachine()
CloseDoor.start()
