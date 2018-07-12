from request import *
from response import *

def MaterialLoad(parent):
    MaterialLoad = Request(parent, parent.adapter, parent.material_load, rel = True)
    MaterialLoad.create_statemachine()
    MaterialLoad.superstate.start()
    return MaterialLoad

def MaterialUnload(parent):
    MaterialUnload = Request(parent, parent.adapter, parent.material_unload, rel = True)
    MaterialUnload.create_statemachine()
    MaterialUnload.superstate.start()
    return MaterialUnload

def MaterialLoadResponse(parent, simulate = True):
    MaterialLoad = Response(parent, parent.adapter, parent.material_load, 'material', 'UNLOADED', 'HAS_MATERIAL', parent.material_state, rel = True, simulate = simulate)
    MaterialLoad.create_statemachine()
    MaterialLoad.superstate.start()
    return MaterialLoad


def MaterialUnloadResponse(parent, simulate = True):
    MaterialUnload = Response(parent, parent.adapter, parent.material_unload, 'material', 'LOADED', 'HAS_MATERIAL', parent.material_state, rel = True, simulate = simulate)
    MaterialUnload.create_statemachine()
    MaterialUnload.superstate.start()
    return MaterialUnload
