import sys, os, time
from data_item import Event, SimpleCondition, Sample
from mtconnect_adapter import Adapter

if __name__ == "__main__":
    adapter = Adapter(('localhost', 7778))
    avail = Event('avail')
    adapter.add_data_item(avail)
    e1 = Event('exec')
    adapter.add_data_item(e1)
    mode = Event('mode')
    adapter.add_data_item(mode)


    binding_state1 = Event('bind')
    adapter.add_data_item(binding_state1)

    open_chuck = Event('open_chuck')
    adapter.add_data_item(open_chuck)

    close_chuck = Event('close_chuck')
    adapter.add_data_item(close_chuck)

    open_door = Event('open_door')
    adapter.add_data_item(open_door)

    close_door = Event('close_door')
    adapter.add_data_item(close_door)

    material_load = Event('material_load')
    adapter.add_data_item(material_load)

    material_unload = Event('material_unload')
    adapter.add_data_item(material_unload)
    
    adapter.start()
    
    adapter.begin_gather()
    
    
    open_chuck.set_value("READY")
    close_chuck.set_value("READY")
    open_door.set_value("READY")
    close_door.set_value("READY")
    material_unload.set_value("READY")
    
    adapter.complete_gather()

    if True:
        time.sleep(20)
        adapter.begin_gather()
        material_load.set_value("READY")
        adapter.complete_gather()

        adapter.begin_gather()
        mode.set_value('AUTOMATIC')
        avail.set_value('AVAILABLE')
        e1.set_value('AUTOMATIC')
        adapter.complete_gather()
       
        adapter.begin_gather()
        
        material_load.set_value("ACTIVE")

        adapter.complete_gather()
        
        time.sleep(1)

        adapter.begin_gather()
        close_chuck.set_value("ACTIVE")

        adapter.complete_gather()
        
        time.sleep(0.7)
        
        adapter.begin_gather()
        close_chuck.set_value("COMPLETE")
        adapter.complete_gather()

        adapter.begin_gather()
        close_door.set_value("ACTIVE")

        adapter.complete_gather()
        
        time.sleep(0.7)
        adapter.begin_gather()
        close_door.set_value("COMPLETE")
        adapter.complete_gather()
        
        adapter.begin_gather()
        material_load.set_value("COMPLETE")
        
        adapter.complete_gather()

        

        

        

        
