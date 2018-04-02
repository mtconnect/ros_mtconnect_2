from request import Request
from response import Response

def MaterialLoad(parent):
    MaterialLoad = Request(parent, 'adapter', parent.material_load, rel = True)
    MaterialLoad.create_statemachine()
    MaterialLoad.superstate.start()
    return MaterialLoad

def MaterialLoadResponse(parent):
    MaterialLoad = Response(parent, 'adapter', parent.material_load, 'material', 'LOADED', '', rel = True, simulate = True)
    MaterialLoad.create_statemachine()
    MaterialLoad.superstate.start()
    return MaterialLoad

def MaterialUnload(parent):
    MaterialUnload = Request(parent, 'adapter', parent.material_unload, rel = True)
    MaterialUnload.create_statemachine()
    MaterialUnload.superstate.start()
    return MaterialUnload

def MaterialUnloadResponse(parent):
    MaterialUnload = Response(parent, 'adapter', parent.material_unload, 'material', 'UNLOADED', '', rel = True, simulate = True)
    MaterialUnload.create_statemachine()
    MaterialUnload.superstate.start()
    return MaterialUnload
