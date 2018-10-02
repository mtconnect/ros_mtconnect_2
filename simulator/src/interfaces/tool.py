from response import *
from request import *

"""Request and Response Interfaces for Tool Handling"""

def ChangeTool(parent, simulate = True):
    ChangeTool = Response(parent, parent.adapter, parent.change_tool, 'tool', 'CHANGED', 'CHANGING',parent.tool_state, rel = True, simulate = simulate)
    ChangeTool.create_statemachine()
    ChangeTool.superstate.start()
    return ChangeTool

def ChangeToolRequest(parent):
    ChangeTool = Request(parent, parent.adapter, parent.change_tool, rel = True)
    ChangeTool.create_statemachine()
    ChangeTool.superstate.start()
    return ChangeTool
