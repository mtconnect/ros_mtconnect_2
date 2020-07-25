from .response import *
from .request import *

"""Request and Response Interfaces for Tool Handling"""

def ChangeTool(parent, simulate = True):
    ChangeTool = Response(
        parent = parent,
        adapter = parent.adapter,
        interface = parent.change_tool,
        prefix = 'tool',
        dest_state = 'CHANGED',
        transition_state = 'CHANGING',
        response_state = parent.tool_state,
        rel = True,
        simulate = simulate
        )
    ChangeTool.superstate.start()
    return ChangeTool

def ChangeToolRequest(parent):
    ChangeTool = Request(
        parent = parent,
        adapter = parent.adapter,
        interface = parent.change_tool,
        rel = True
        )
    ChangeTool.superstate.start()
    return ChangeTool
