from response import *
from request import *

def OpenChuck(parent):
    OpenChuck = Response(parent, parent.adapter, parent.open_chuck, 'chuck', 'OPEN', 'UNLATCHED', parent.chuck_state, rel = True, simulate = True)
    OpenChuck.create_statemachine()
    OpenChuck.superstate.start()
    return OpenChuck

def OpenChuckRequest(parent):
    OpenChuck = Request(parent, parent.adapter, parent.open_chuck, rel = True)
    OpenChuck.create_statemachine()
    OpenChuck.superstate.start()
    return OpenChuck

def CloseChuck(parent):
    CloseChuck = Response(parent, parent.adapter, parent.close_chuck, 'chuck', 'CLOSED', 'UNLATCHED', parent.chuck_state, rel = True, simulate = True)
    CloseChuck.create_statemachine()
    CloseChuck.superstate.start()
    return CloseChuck

def CloseChuckRequest(parent):
    CloseChuck = Request(parent, parent.adapter, parent.close_chuck, rel = True)
    CloseChuck.create_statemachine()
    CloseChuck.superstate.start()
    return CloseChuck
