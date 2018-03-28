import sys, os, time
from data_item import Event, SimpleCondition, Sample
from mtconnect_adapter import Adapter
from threading import Timer, Thread


def bot():
    adapter = Adapter(('localhost', 7882))
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




    binding_state_material = Event('binding_state_material')
    adapter.add_data_item(binding_state_material)


    
    adapter.start()
    
    adapter.begin_gather()
       
    binding_state_material.set_value("INACTIVE")
    material_load.set_value('READY')
    material_unload.set_value('READY')

    
    adapter.complete_gather()

    if True:
        
        time.sleep(40)

        adapter.begin_gather()
        
        binding_state_material.set_value("PREPARING")
        
        adapter.complete_gather()
        
        time.sleep(0.2)


        time.sleep(3)

        adapter.begin_gather()
        
        binding_state_material.set_value("COMMITTED")
        
        adapter.complete_gather()
        
        time.sleep(0.2)

        time.sleep(2)

        adapter.begin_gather()
        
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

        time.sleep(0.5)

        adapter.begin_gather()
        
        material_load.set_value("COMPLETE")
        
        adapter.complete_gather()
        
   


def cnc():
    
    adapter1 = Adapter(('localhost', 7883))
    avail1 = Event('avail')
    adapter1.add_data_item(avail1)
    e11 = Event('exec')
    adapter1.add_data_item(e11)
    mode1 = Event('mode')
    adapter1.add_data_item(mode1)



    binding_state_material1 = Event('binding_state_material')
    adapter1.add_data_item(binding_state_material1)

    material_load1 = Event('material_load')
    adapter1.add_data_item(material_load1)
    
    adapter1.start()
    
    adapter1.begin_gather()
       
    binding_state_material1.set_value("INACTIVE")
    material_load1.set_value("READY")
    
    adapter1.complete_gather()

    if True:
        
        time.sleep(40)

        time.sleep(0.2)

        adapter1.begin_gather()
        
        binding_state_material1.set_value("PREPARING")
        
        adapter1.complete_gather()

        time.sleep(3)

        time.sleep(0.2)

        adapter1.begin_gather()
        
        binding_state_material1.set_value("COMMITTED")
        
        adapter1.complete_gather()

        time.sleep(4.5)

        adapter1.begin_gather()
        
        material_load1.set_value("ACTIVE")
        
        adapter1.complete_gather()

        time.sleep(0.5)

        adapter1.begin_gather()
        
        material_load1.set_value("COMPLETE")
        
        adapter1.complete_gather()


if __name__ == '__main__':
    thread= Thread(target = bot)
    thread.start()

    thread1= Thread(target = cnc)
    thread1.start()
