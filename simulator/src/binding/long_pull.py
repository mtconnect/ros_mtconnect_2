import re
import scanner
import requests

class LongPullException(Exception):
    pass

class LongPull:
    def __init__(self, response):
        self._response = response
        self._buffer = ''

    def long_pull(self, callback, user_data = None):
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
            document.string += chunk

            while document.rest_len() >= length:
                if header:
                    if not document.check(boundary_pat):
                        print "Framing error!"
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

                    callback(body)

                    length = len(boundary)
                    header = True

if __name__ == "__main__":
    response = requests.get("http://127.0.0.1:5005/sample?interval=1000&count=1000", stream=True)

    lp = LongPull(response)
    def callback(chunk):
        print chunk

    lp.long_pull(callback)
