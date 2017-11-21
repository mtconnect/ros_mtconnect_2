from expects import *
from doublex import *

from src.request import Request, interface

with description('request'):
    with it('should pass'):
        pass
    
    with context('request'):
        with before.each:
            self.request = Request(interface())
            
        with it('should have a request'):
            expect(self.request).to(be_a(Request))
      
    
  
