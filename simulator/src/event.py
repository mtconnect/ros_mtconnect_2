"""
Common data type for simulating events between MTConnect devices.

Right now the event is just a collection of names, implemented with a collections.namedtuple. If the
interface requires adding functions it can be converted to a class.
"""
from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import collections

Event = collections.namedtuple('Event', ['source', 'component', 'name', 'value', 'code', 'text'])
#default values for the last two fields (not needed in the constructor)
Event.__new__.__defaults__ = (None, None)
