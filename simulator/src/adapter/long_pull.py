import re
from . import scanner
import requests, datetime
from threading import Timer, Thread
from xml.etree import ElementTree as ET

class LongPullException(Exception):
    pass

class LongPull:
    def __init__(self, response, addr = None, parent = None):
        self._response = response
        self._buffer = ''
        self._addr = addr
        self._parent = parent

    def long_pull(self, callback, user_data = None, addr = None, parent = None):
        content_type = self._response.headers.get('Content-Type')
        match = re.search('boundary=([0-9A-Fa-f]+)', content_type)
        if not match:
            raise LongPullException('Cannot find boundary in content-type')

        boundary = '--' + match.group(1)
        boundary_pat = re.compile('^' + boundary)
        header = True
        length = len(boundary)
        document = scanner.Scanner('')

        for chunk in self._response.iter_content(chunk_size=None):
            document.string += chunk.decode("utf-8")

            while document.rest_len() >= length:
                if header:
                    if not document.check(boundary_pat):
                        print ("Framing error!")
                        raise LongPullException('Framing error')

                    head = document.scan_until('\r\n\r\n')
                    mime_headers = head.split('\r\n')
                    values = dict([(v.lower().split(':')) for v in mime_headers[1:] if v.find(':') > 0])
                    header = False
                    try:
                        length = int(values['content-length'])
                    except ValueError:
                        raise LongPullException('Cannot get length from mime header: ' + mime_headers)

                else:
                    rest = document.rest()
                    body = rest[:length]

                    document.reset()
                    document.string = rest[length:]

                    callback(self._parent, body, self._addr)

                    length = len(boundary)
                    header = True

                if not self._response:
                    break
            if not self._response:
                break
