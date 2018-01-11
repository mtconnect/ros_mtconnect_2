import os, sys, inspect, io

cmd_folder = os.path.realpath(
    os.path.dirname(
        os.path.abspath(os.path.split(inspect.getfile( inspect.currentframe() ))[0])))

if cmd_folder not in sys.path:
    sys.path.insert(0, cmd_folder)
    
from transitions.extensions import MachineFactory
from IPython.display import Image, display, display_png

class Matter(object):
    def is_hot(self):
        return True
    def is_too_hot(self):
        return False
    def show_graph(self, **kwargs):
        #print(self.get_graph(**kwargs).string())
        stream = io.BytesIO()
        self.get_graph(**kwargs).draw(stream, prog='dot', format='png')
        display(Image(stream.getvalue())) 

GraphMachine = MachineFactory.get_predefined(graph=True, nested=True)

states = ['standing', 'walking', {'name': 'caffeinated', 'children':['dithering', 'running']}]
transitions = [
  ['walk', 'standing', 'walking'],
  ['go', 'standing', 'walking'],
  ['stop', 'walking', 'standing'],
  {'trigger': 'drink', 'source': '*', 'dest': 'caffeinated_dithering',
   'conditions':'is_hot', 'unless': 'is_too_hot'},
  ['walk', 'caffeinated_dithering', 'caffeinated_running'],
  ['relax', 'caffeinated', 'standing'],
  ['sip', 'standing', 'caffeinated']
]

model = Matter()
machine = GraphMachine(model=model,
                       states=states, 
                       transitions=transitions, 
                       auto_transitions=False, 
                       initial='standing', 
                       title="Mood Matrix",
                       show_conditions=True)
model.show_graph()
