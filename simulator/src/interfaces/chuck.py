from response import *
from request import *

"""Request and Response Interfaces for a Chuck/Vice/Fixture"""

def OpenChuck(parent, simulate = True):
    OpenChuck = Response(parent, parent.adapter, parent.open_chuck, 'chuck', 'OPEN', 'UNLATCHED', parent.chuck_state, rel = True, simulate = simulate)
    OpenChuck.superstate.start()
    return OpenChuck

def OpenChuckRequest(parent):
    OpenChuck = Request(parent, parent.adapter, parent.open_chuck, rel = True)
    OpenChuck.superstate.start()
    return OpenChuck

def CloseChuck(parent, simulate = True):
    CloseChuck = Response(parent, parent.adapter, parent.close_chuck, 'chuck', 'CLOSED', 'UNLATCHED', parent.chuck_state, rel = True, simulate = simulate)
    CloseChuck.superstate.start()
    return CloseChuck

def CloseChuckRequest(parent):
    CloseChuck = Request(parent, parent.adapter, parent.close_chuck, rel = True)
    CloseChuck.superstate.start()
    return CloseChuck
