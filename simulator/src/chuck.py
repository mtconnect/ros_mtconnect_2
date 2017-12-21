from response import *


#create parent centrally

OpenChuck = Response(cnc, cnc.adapter, cnc.open_chuck, 'chuck', 'OPEN', 'UNLATCHED', rel, simulate= True)
OpenChuck.create_statemachine()
OpenChuck.start()

CloseChuck = Response(cnc, cnc.adapter, cnc.close_chuck, 'chuck', 'CLOSED', 'UNLATCHED', rel, simulate= True)
CloseChuck.create_statemachine()
CloseChuck.start()
