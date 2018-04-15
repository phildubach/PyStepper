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
import tornado.ioloop
import tornado.web
import sockjs.tornado

class StatusHandler(tornado.web.RequestHandler):
    def get(self):
        stepper = self.settings['stepper']
        self.write(stepper.status())

class MoveHandler(tornado.web.RequestHandler):
    def post(self):
        target = int(self.get_argument('target_position'))
        speed = int(self.get_argument('speed', default=0))
        accel = int(self.get_argument('acceleration', default=0))
        mode = self.get_argument('mode', default='absolute')
        stepper = self.settings['stepper']
        stepper.queue(target, speed, accel, mode)

class StopHandler(tornado.web.RequestHandler):
    def post(self):
        stepper = self.settings['stepper']
        stepper.stop()

class StatusConnection(sockjs.tornado.SockJSConnection):
    def on_open(self, info):
        self.stepper = self.session.server.stepper
        self.stepper.set_callback(self)
        # self.loop = tornado.ioloop.PeriodicCallback(self.send_status, 1000)
        #self.loop.start()

    def on_close(self):
        self.stepper.set_callback(None)
        # self.loop.stop()

    def on_message(self, msg):
        pass

    def send_status(self):
        self.session.send_message(self.stepper.status())

    def callback(self):
        self.send_status()

class StatusRouter(sockjs.tornado.SockJSRouter):
    def __init__(self, path, stepper):
        sockjs.tornado.SockJSRouter.__init__(self, StatusConnection, path)
        self.stepper = stepper

if __name__ == '__main__':
    stepper = PyStepper('BIPOLAR', [ 40, 38, 36, 37 ], [ 22, 11 ])
    stepper.start_daemon()
    statusRouter = StatusRouter('/statusconn', stepper)
    app = tornado.web.Application([
        ( r"/", tornado.web.RedirectHandler, { "url": "/static/index.html" }),
        ( r"/api/status", StatusHandler ),
        ( r"/api/move", MoveHandler ),
        ( r"/api/stop", StopHandler ),
    ] + statusRouter.urls, static_path="./static")
    app.settings['stepper'] = stepper
    app.listen(8080)
    try:
        tornado.ioloop.IOLoop.current().start()
    except KeyboardInterrupt:
        stepper.stop_daemon()
        stepper.exit()

