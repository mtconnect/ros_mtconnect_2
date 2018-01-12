from src.request import *
from src.response import *
from src.cnc import *

req = Request('parent', 'adapter', interface(), True)
req.create_statemachine()
req.draw()

rsp = Response('parent', 'adapter', interface(), 'door', 'OPEN', 'UNLATCHED', True, simulate= True)
rsp.create_statemachine()
rsp.draw()

cnc = cnc(interface)
cnc.create_statemachine()
cnc.draw()

