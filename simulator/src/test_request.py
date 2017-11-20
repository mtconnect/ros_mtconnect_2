import unittest
import request

class TestRequest(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.req=request.Request(request.interface()) 
        self.req.create_statemachine()

    @classmethod
    def tearDownClass(self):
        pass

    def test_create_statemachine(self):
        self.assertEqual(self.req.superstate.state,'base')


    def test_states(self):
        self.req.superstate.unavailable()
        self.assertEqual(self.req.superstate.state,'base:not_ready')

        self.req.superstate.deactivate()
        self.assertEqual(self.req.superstate.state,'base:not_ready')

        self.req.superstate.idle()
        self.assertEqual(self.req.superstate.state,'base:ready')


    def test_methods(self):
        self.req.superstate.unavailable()
        self.req.superstate.idle()
        self.assertEqual(self.req.superstate.state,'base:ready')
        self.assertEqual(self.req.interface.value,'READY')

        self.req.superstate.ready()
        self.assertEqual(self.req.interface.value,'ACTIVE')

        self.req.superstate.not_ready()
        self.assertEqual(self.req.interface.value,'READY')

        

if __name__ == "__main__":
    unittest.main()
