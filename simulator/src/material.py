from request import *

#Material Load
material_load = interface() #will eventually be defined in cnc.py
#parent = cnc
#adapter = cnc.adapter
#material_load = cnc.material_load
MaterialLoad = Request("parent", "adapter", material_load, None)
MaterialLoad.create_statemachine()
MaterialLoad.superstate.start()

#Material Unload
material_unload = interface() #will eventually be defined in cnc.py
#parent = cnc
#adapter = cnc.adapter
#material_unload = cnc.material_unload
MaterialUnload = Request("parent", "adapter", material_unload, None)
MaterialUnload.create_statemachine()
MaterialUnload.superstate.start()
