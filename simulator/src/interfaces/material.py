from request import *
from response import *

"""Request and Response Interfaces for Material Handling"""

def MaterialLoad(parent):
    MaterialLoad = Request(
        parent = parent,
        adapter = parent.adapter,
        interface = parent.material_load,
        rel = True
        )
    MaterialLoad.superstate.start()
    return MaterialLoad

def MaterialUnload(parent):
    MaterialUnload = Request(
        parent = parent,
        adapter = parent.adapter,
        interface = parent.material_unload,
        rel = True
        )
    MaterialUnload.superstate.start()
    return MaterialUnload

def MaterialLoadResponse(parent, simulate = True):
    MaterialLoad = Response(
        parent = parent,
        adapter = parent.adapter,
        interface = parent.material_load,
        prefix = 'material',
        dest_state = 'UNLOADED',
        transition_state = 'HAS_MATERIAL',
        response_state = parent.material_state,
        rel = True,
        simulate = simulate
        )
    MaterialLoad.superstate.start()
    return MaterialLoad


def MaterialUnloadResponse(parent, simulate = True):
    MaterialUnload = Response(
        parent = parent,
        adapter = parent.adapter,
        interface = parent.material_unload,
        prefix = 'material',
        dest_state = 'LOADED',
        transition_state = 'HAS_MATERIAL',
        response_state = parent.material_state,
        rel = True,
        simulate = simulate
        )
    MaterialUnload.superstate.start()
    return MaterialUnload
