import sys, os, time
from data_item import Event, SimpleCondition, Sample
from mtconnect_adapter import Adapter
from threading import Timer, Thread


def bot():
    adapter = Adapter(('localhost', 7880))
    avail = Event('avail')
    adapter.add_data_item(avail)
    e1 = Event('exec')
    adapter.add_data_item(e1)
    mode = Event('mode')
    adapter.add_data_item(mode)

    material_load = Event('material_load')
    adapter.add_data_item(material_load)

    material_unload = Event('material_unload')
    adapter.add_data_item(material_unload)

    open_chuck = Event('open_chuck')
    adapter.add_data_item(open_chuck)

    close_chuck = Event('close_chuck')
    adapter.add_data_item(close_chuck)

    open_door = Event('open_door')
    adapter.add_data_item(open_door)

    close_door = Event('close_door')
    adapter.add_data_item(close_door)


    binding_state_material = Event('binding_state_material')
    adapter.add_data_item(binding_state_material)


    
    adapter.start()
    
    adapter.begin_gather()

    avail.set_value("AVAILABLE")
    binding_state_material.set_value("INACTIVE")
    material_load.set_value('READY')
    material_unload.set_value('READY')

    
    adapter.complete_gather()

    if True:
        
        time.sleep(35)


        adapter.begin_gather()
        
        binding_state_material.set_value("PREPARING")
        
        adapter.complete_gather()
        
        time.sleep(0.2)


        time.sleep(10)

        adapter.begin_gather()
        
        binding_state_material.set_value("COMMITTED")
        
        adapter.complete_gather()
        
        time.sleep(0.2)

        time.sleep(2)


        
        material_unload.set_value("ACTIVE")
        
        adapter.complete_gather()

        time.sleep(0.5)

        adapter.begin_gather()
        
        material_unload.set_value("COMPLETE")
        
        adapter.complete_gather()

        
        time.sleep(2)
        
        adapter.begin_gather()
        
        material_load.set_value("ACTIVE")
        
        adapter.complete_gather()

        time.sleep(2.5)
        """
        ####open door
        adapter.begin_gather()
        
        open_door.set_value("ACTIVE")
        
        adapter.complete_gather()
        time.sleep(1.2)

        adapter.begin_gather()
        
        open_door.set_value("READY")
        
        adapter.complete_gather()

        time.sleep(0.5)

        adapter.begin_gather()

        ###open chuck
        open_chuck.set_value("ACTIVE")
        
        adapter.complete_gather()
        time.sleep(1.2)

        adapter.begin_gather()
        
        open_chuck.set_value("READY")
        
        adapter.complete_gather()

        time.sleep(0.5)
        """
        #close chuck
        adapter.begin_gather()

        close_chuck.set_value('READY')
        adapter.complete_gather()

        time.sleep(0.3)
        adapter.begin_gather()
        
        close_chuck.set_value("ACTIVE")
        
        adapter.complete_gather()
        time.sleep(2)

        ###close door
        
        adapter.begin_gather()
        
        close_door.set_value("READY")
        
        adapter.complete_gather()

        time.sleep(0.3)

        adapter.begin_gather()
        
        close_door.set_value("ACTIVE")
        
        adapter.complete_gather()
        time.sleep(2.2)

        adapter.begin_gather()
        
        material_load.set_value("COMPLETE")
        
        adapter.complete_gather()

        

if __name__ == '__main__':
    thread= Thread(target = bot)
    thread.start()
