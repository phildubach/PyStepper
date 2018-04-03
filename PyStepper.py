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

import RPi.GPIO as Gpio
import time, math
from Queue import Queue
from threading import Thread

class PyStepperDaemon(Thread):
    """Provides a server daemon that executes advanced movements"""

    def __init__(self, stepper, position=0, max_speed=800, max_accel=1600, upper_limit=None, lower_limit=None):
        Thread.__init__(self)
        self.daemon = True
        self.stepper = stepper         # the stepper instance
        self.shutdown = False          # flag to stop execution and exit
        self.max_speed = max_speed     # maximum speed in steps/s
        self.max_accel = max_accel     # maximum acceleration in steps/s/s
        self.position = position       # current absolute position
        self.target = position         # absolute target position
        self.speed = 0                 # current speed
        self.ulimit = upper_limit      # upper limit (steps from zero)
        self.llimit = lower_limit      # lower limit (steps from zero)
        self.tasks = Queue()           # movement queue
        if self.ulimit != None and self.position > self.ulimit:
            raise RuntimeException("position (%d) > upper limit (%d)" % (self.position, self.ulimit))
        if self.llimit != None and self.position < self.llimit:
            raise RuntimeException("position (%d) < lower limit (%d)" % (self.position, self.llimit))

    def run(self):
        """The daemon's main loop; should not be called directly!"""
        # TODO: fix shutdown while blocked on queue or during movement
        while not self.shutdown:
            (target, speed, accel, absolute) = self.tasks.get()
            start = self.position
            if not absolute:
                target = start + target
            self.target = target
            # TODO: handle limits
            dist = abs(target - start)
            if not dist == 0:
                sign = (target - start) / dist
                done = 0
                accel_time = float(speed) / accel
                accel_dist = speed * accel_time / 2
                if (2 * accel_dist > dist):
                    # cannot accelerate to full speed
                    accel_dist = (dist + 1) / 2
                    speed = math.sqrt(2 * accel_dist * accel)
                    accel_time = float(speed) / accel
                # acceleration loop
                while done < accel_dist:
                    self.speed = speed / accel_dist * (done + 1)
                    incr_time = 1.0 / self.speed
                    self.stepper.step(sign)
                    self.position += sign
                    time.sleep(incr_time)
                    done += 1
                # uniform speed
                self.speed = speed
                incr_time = 1.0 / self.speed
                while done < (dist - accel_dist):
                    self.stepper.step(sign)
                    self.position += sign
                    time.sleep(incr_time)
                    done += 1
                # deceleration loop
                while done < dist:
                    self.speed = speed / accel_dist * (dist - done)
                    incr_time = 1.0 / self.speed
                    self.stepper.step(sign)
                    self.position += sign
                    time.sleep(incr_time)
                    done += 1
            # movement completed
            self.speed = 0
            self.tasks.task_done()

    def shutdown(self):
        """Ask the daemon to shut down and wait for it to terminate"""
        self.shutdown = True
        self.join()

    def queue(self, target, speed, accel, absolute=True):
        if speed == 0:
            speed = self.max_speed
        if accel == 0:
            accel = self.max_accel
        self.tasks.put((target, speed, accel, absolute))

class PyStepper:
    """Provides low-level functions for controlling a stepper motor connected to
    GPIO pins on a Raspberry Pi"""

    FORWARD = 1
    BACKWARD = -1

    sequences = { 'UNIPOLAR': [
                    [ True,  False, False, False ],
                    [ True,  True,  False, False ],
                    [ False, True,  False, False ],
                    [ False, True,  True,  False ],
                    [ False, False, True,  False ],
                    [ False, False, True,  True  ],
                    [ False, False, False, True  ],
                    [ True,  False, False, True  ] ],
                  'BIPOLAR': [
                    [ False, True,  False, True  ],
                    [ False, False, False, True  ],
                    [ True,  False, False, True  ],
                    [ True,  False, False, False ],
                    [ True,  False, True , False ],
                    [ False, False, True , False ],
                    [ False, True , True , False ],
                    [ False, True , False, False ] ]
                }

    def __init__(self, sequence_name, pins):
        """Create an instance of PyStepper.

        Arguments:
        sequence_name -- either 'UNIPOLAR' or 'BIPOLAR'
        pins          -- an array of 4 pin numbers referring to the pin numbers on
                         the connector
        """
        
        self.sequence = PyStepper.sequences[sequence_name]
        self.pins = pins
        Gpio.setmode(Gpio.BOARD)
        self.position = 0;
        self._server = None
        for pin in self.pins:
            Gpio.setup(pin, Gpio.OUT)
            Gpio.output(pin, Gpio.LOW)

    def step(self, increment=FORWARD):
        """Perform one step in the given direction

        Valid directions are: PyStepper.FORWARD and PyStepper.BACKWARD. Those
        refer to the direction the code advances through the step sequence
        array. The actual direction of rotation of the motor is dependent on
        how the motor is wired up. If no direction is given, FORWARD is assumed
        """
        # TODO: check increment range?
        self.position = (self.position + increment) % len(self.sequence)
        for idx in range(len(self.pins)):
            Gpio.output(self.pins[idx], self.sequence[self.position][idx])

    def forward(self):
        """Convenience method to take one step forward"""
        self.step(self.FORWARD)

    def backward(self):
        """Convenience method to take one step backward"""
        self.step(self.BACKWARD)

    def get_server(self):
        if not self._server:
            raise RuntimeError("Server not running")
        return self._server

    def start_daemon(self):
        """Start a daemon thread for scheduling advanced movements"""
        # TODO: pass start position or run calibration
        if self._server:
            raise RuntimeError("Daemon already running")
        self._server = PyStepperDaemon(self)
        self._server.start()

    def stop_daemon(self):
        """Stop the previously started daemon thread"""
        server = self.get_server()
        server.stop()
        self._server = None

    def queue(self, target, speed=0, accel=0, absolute=True):
        server = self.get_server()
        server.queue(target, speed, accel, absolute)

    def sync(self):
        server = self.get_server()
        server.tasks.join()

    def status(self):
        server = self.get_server()
        return dict(position=server.position, target=server.target, speed=server.speed)

if __name__ == '__main__':
    # Create an instance of PyStepper for a unipolar motor connected to the
    # pins given in the array. Note that the pin numbers refer to the numers
    # on the 40-pin connector/header, NOT the GPIO numbers
    stepper = PyStepper('BIPOLAR', [ 40, 38, 36, 37 ])

    # deactivated test code
    if False:
        # Test by taking a number of steps
        for i in range(200):
            # arbitrarily choose to step forward
            stepper.forward()

            # sleep for 10 milliseconds between steps; this determines the speed of
            # rotation; maximum speed depends on the motor and power supply
            time.sleep(0.01)

    # start the daemon
    stepper.start_daemon()
    stepper.queue(800, speed=400, accel=200)
    stepper.queue(400, speed=800)
    stepper.sync()

    # Before exiting, clean up, redefining all pins as INPUTS
    Gpio.cleanup()

