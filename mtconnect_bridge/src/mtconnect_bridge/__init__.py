#Bring in imports so they can be accessed directly from the `mtconnect_bridge` package
from __future__ import print_function

import sys
import os

#HACK to access simulator code
sys.path.append(os.path.join(os.getenv('HOME'), 'Workspaces/ceccrebot/src/ceccrebot'))

from bridge import Bridge
