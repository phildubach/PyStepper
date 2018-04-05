#!/usr/bin/env python

# Copyright 2018 Phil Dubach
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from PyStepper import PyStepper
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import os.path, json, sys, socket
import RPi.GPIO as Gpio

class GetResponse:
    def __init__(self, handler):
        self.handler = handler
        path = self.handler.path
        self.fname = None
        self.json = None
        self.ftype = 'text/plain'
        self.flength = 0
        self.code = 404
        if path == '/':
            self.code = 301
            self.redirect_url = 'http://%s/static/index.html' % (self.handler.headers.dict['host'],)
        elif path.startswith('/static/'):
            self.fname = path[1:]
            if os.path.isfile(self.fname):
                self.code = 200
                self.flength = os.path.getsize(self.fname)
                if self.fname.endswith('.html'):
                    self.ftype = 'text/html'
                elif self.fname.endswith('.css'):
                    self.ftype = 'text/css'
                elif self.fname.endswith('.js'):
                    self.ftype = 'text/javascript'
        elif path.startswith('/api/'):
            if path == '/api/status':
                self.code = 200
                status = self.handler.get_stepper().status()
                self.json = json.dumps([
                    { 'key': 'Current position', 'value': status['position'] },
                    { 'key': 'Target position', 'value': status['target'] },
                    { 'key': 'Current speed', 'value': status['speed'] }
                ])
                self.ftype = 'application/json'

class RequestHandler(BaseHTTPRequestHandler):
    def set_headers(self, code=200, content_type=None, content_length=None, redirect_url=None):
        self.send_response(code)
        if content_type:
            self.send_header('Content-Type', content_type)
        if content_length:
            self.send_header('Content-Length', content_length)
        if (code/100 == 3) and redirect_url:
            self.send_header('Location', redirect_url);
        self.end_headers()

    def get_stepper(self):
        return self.server.stepper

    def do_GET(self):
        response = GetResponse(self)
        if response.code == 200:
            if response.fname and response.flength:
                self.set_headers(response.code, content_type=response.ftype, content_length=response.flength)
                f = open(response.fname)
                # TODO chunks?
                data = f.read()
                self.wfile.write(data)
            elif response.json:
                self.set_headers(response.code, content_type=response.ftype, content_length=len(response.json))
                self.wfile.write(response.json)
        elif response.code/100 == 3:
            self.set_headers(response.code, redirect_url=response.redirect_url)
        else:
            self.send_response(response.code)

    def do_HEAD(self):
        response = GetResponse(self)
        if response.code == 200:
            if response.fname and response.flength:
                self.set_headers(response.code, response.ftype, response.flength)
            elif response.json:
                self.set_headers(response.code, content_type=response.ftype, content_length=len(response.json))
        else:
            self.send_response(response.code)
        
    def do_POST(self):
        if self.path == '/api/move':
            length = int(self.headers.getheader('Content-Length'))
            json_data = self.rfile.read(length)
            try:
                data = json.loads(json_data)
                # required field
                target = data['target_position']
                # optional fields
                speed = data.get('speed', 0)
                accel = data.get('acceleration', 0)
                absolute = data.get('absolute', True)
                self.get_stepper().queue(target, speed, accel, absolute)
                self.set_headers(code=204)
            except:
                print "Error in post: ", sys.exc_info()[0]
                self.set_headers(code=400, content_type="text/plain")
                self.wfile.write("Invalid post data")
        elif self.path == '/api/stop':
            self.get_stepper().stop();
            self.set_headers(code=204);
        else:            
            self.set_headers(404)
        
def run_server(stepper, port=8080):
    server_address = ('', port)
    print "".join(("Listening on http://", get_ip_addr(), ":", str(port), "/"))
    print "Exit with <ctrl>-c"
    server = HTTPServer(server_address, RequestHandler)
    server.stepper = stepper
    server.serve_forever()

def get_ip_addr():
    # TODO: there must be a better way to get the local IP address
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    addr = s.getsockname()[0]
    s.close()
    return addr

if __name__ == "__main__":
    stepper = PyStepper('BIPOLAR', [ 40, 38, 36, 37 ])
    stepper.start_daemon()
    try:
        run_server(stepper)
    except KeyboardInterrupt:
        Gpio.cleanup()
