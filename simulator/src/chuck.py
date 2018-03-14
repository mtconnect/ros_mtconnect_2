from response import Response
from request import Request

def OpenChuck(parent):
    OpenChuck = Response(parent, 'adapter', parent.open_chuck, 'chuck', 'OPEN', 'UNLATCHED', rel = True, simulate = True)
    OpenChuck.create_statemachine()
    OpenChuck.superstate.start()
    return OpenChuck

def OpenChuckRequest(parent):
    OpenChuck = Request(parent, "adapter", parent.open_chuck, rel = True)
    OpenChuck.create_statemachine()
    OpenChuck.superstate.start()
    return OpenChuck

def CloseChuck(parent):
    CloseChuck = Response(parent, 'adapter', parent.close_chuck, 'chuck', 'CLOSED', 'UNLATCHED', rel = True, simulate = True)
    CloseChuck.create_statemachine()
    CloseChuck.superstate.start()
    return CloseChuck

def CloseChuckRequest(parent):
    CloseChuck = Request(parent, "adapter", parent.close_chuck, rel = True)
    CloseChuck.create_statemachine()
    CloseChuck.superstate.start()
    return CloseChuck
