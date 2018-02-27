from request import *

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
