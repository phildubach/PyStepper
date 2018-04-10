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

import time, math, logging, sys
import Queue
from threading import Thread

try:
    import RPi.GPIO as Gpio
except ImportError:
    print "Python module RPi.GPIO not found. Using dummy implementation for testing"
    import DummyGpio as Gpio

class PyStepperDaemon(Thread):
    """Provides a server daemon that executes advanced movements"""

    def __init__(self, stepper, position=0, max_speed=800, max_accel=1600, upper_limit=None, lower_limit=None, active_hold=False):
        Thread.__init__(self)
        self.daemon = True
        self.stepper = stepper         # the stepper instance
        self.shutdownFlag = False      # flag to stop execution and exit
        self.stop = False              # flag to stop current movement
        self.max_speed = max_speed     # maximum speed in steps/s
        self.max_accel = max_accel     # maximum acceleration in steps/s/s
        self.position = position       # current absolute position
        self.target = position         # absolute target position
        self.speed = 0                 # current speed
        self.ulimit = upper_limit      # upper limit (steps from zero)
        self.llimit = lower_limit      # lower limit (steps from zero)
        self.active_hold = active_hold # true if motor should be powered while idle
        self.tasks = Queue.Queue()     # movement queue
        if self.ulimit != None and self.position > self.ulimit:
            raise RuntimeException("position (%d) > upper limit (%d)" % (self.position, self.ulimit))
        if self.llimit != None and self.position < self.llimit:
            raise RuntimeException("position (%d) < lower limit (%d)" % (self.position, self.llimit))

    def run(self):
        """The daemon's main loop; should not be called directly!"""
        while not self.shutdownFlag:
            try:
                (target, speed, accel, mode) = self.tasks.get()
                start = self.position
                if mode == 'relative':
                    target = start + target
                elif mode == 'calibrate':
                    target = -sys.maxint-1
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
                    while done < accel_dist and not self.stop:
                        self.speed = speed / accel_dist * (done + 1)
                        incr_time = 1.0 / self.speed
                        self.step(sign, mode)
                        time.sleep(incr_time)
                        done += 1
                    # uniform speed
                    self.speed = speed
                    incr_time = 1.0 / self.speed
                    while done < (dist - accel_dist) and not self.stop:
                        self.step(sign, mode)
                        time.sleep(incr_time)
                        done += 1
                    # deceleration loop
                    while done < dist and not self.stop:
                        self.speed = speed / accel_dist * (dist - done)
                        incr_time = 1.0 / self.speed
                        self.step(sign, mode)
                        time.sleep(incr_time)
                        done += 1
            except Exception:
                logging.exception("Error in PyStepperDaemon")
            # movement completed
            self.speed = 0
            if not self.active_hold:
                self.stepper.set_idle(True)
            if self.stop:
                self.stop = False
                self.target = self.position
            self.tasks.task_done()

    def step(self, sign, mode):
        switches = self.stepper.active_switches()
        if 0 in switches and sign < 0:
            okay = False
        elif 1 in switches and sign > 0:
            okay = False
        else:
            okay = True

        if okay:
            self.stepper.step(sign)
            self.position += sign
        else:
            self.stop = True
            if sign < 0 and mode == 'calibrate':
                self.position = 0
            # TODO: set max for upper switch?

    def stop_and_flush(self):
        empty = False
        while not empty:
            try:
                task = self.tasks.get(False)
                self.tasks.task_done()
            except Queue.Empty:
                empty = True
        self.stop = True

    def shutdown(self):
        """Ask the daemon to shut down and wait for it to terminate"""
        self.stop_and_flush()
        self.shutdownFlag = True
        # queue dummy movement to wake up daemon
        self.queue(0, self.max_speed, self.max_accel, mode='relative');
        self.join()

    def queue(self, target, speed, accel, mode='absolute'):
        if speed == 0:
            speed = self.max_speed
        if accel == 0:
            accel = self.max_accel
        self.tasks.put((target, speed, accel, mode))

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
                    [ False, False, False, True  ],
                    [ False, True,  False, True  ],
                    [ False, True , False, False ],
                    [ False, True , True , False ],
                    [ False, False, True , False ],
                    [ True,  False, True , False ],
                    [ True,  False, False, False ],
                    [ True,  False, False, True  ] ]

                }

    def __init__(self, sequence_name, motor_pins, switch_pins):
        """Create an instance of PyStepper.

        Arguments:
        sequence_name -- either 'UNIPOLAR' or 'BIPOLAR'
        motor_pins    -- an array of 4 pin numbers used for motor control,
                         referring to the pin numbers on the connector
        switch_pins   -- an array of 1 or 2 pin numbers used for end switches,
                         referring to the pin numbers on the connector
        """
        
        self.sequence = PyStepper.sequences[sequence_name]
        self.motor_pins = motor_pins
        self.switch_pins = switch_pins
        Gpio.setmode(Gpio.BOARD)
        self.position = 0;
        self._server = None
        for pin in self.motor_pins:
            Gpio.setup(pin, Gpio.OUT, initial=Gpio.LOW)
        self.idle = True
        for pin in self.switch_pins:
            Gpio.setup(pin, Gpio.IN, pull_up_down=Gpio.PUD_UP)

    def step(self, increment=FORWARD):
        """Perform one step in the given direction

        Valid directions are: PyStepper.FORWARD and PyStepper.BACKWARD. Those
        refer to the direction the code advances through the step sequence
        array. The actual direction of rotation of the motor is dependent on
        how the motor is wired up. If no direction is given, FORWARD is assumed
        """
        # TODO: check increment range?
        self.set_idle(False)
        self.position = (self.position + increment) % len(self.sequence)
        for idx in range(len(self.motor_pins)):
            Gpio.output(self.motor_pins[idx], self.sequence[self.position][idx])

    def forward(self):
        """Convenience method to take one step forward"""
        self.step(self.FORWARD)

    def backward(self):
        """Convenience method to take one step backward"""
        self.step(self.BACKWARD)

    def set_idle(self, idle=True):
        """Set requested idle state"""
        if not idle == self.idle:
            if idle:
                for pin in self.motor_pins:
                    Gpio.output(pin, Gpio.LOW)
            else:
                for idx in range(len(self.motor_pins)):
                    Gpio.output(self.motor_pins[idx], self.sequence[self.position][idx])
            self.idle = idle

    def active_switches(self):
        switches = []
        for idx in range(len(self.switch_pins)):
            if Gpio.input(self.switch_pins[idx]) == Gpio.LOW:
                print "switch %d on" % (idx,)
                switches.append(idx)
        return switches

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
        server.shutdown()
        self._server = None

    def stop(self):
        server = self.get_server();
        server.stop_and_flush();

    def queue(self, target, speed=0, accel=0, mode='absolute'):
        server = self.get_server()
        server.queue(target, speed, accel, mode)

    def sync(self):
        server = self.get_server()
        server.tasks.join()

    def status(self):
        server = self.get_server()
        return dict(position=server.position, target=server.target, speed=server.speed)

    def exit(self):
        Gpio.cleanup()

if __name__ == '__main__':
    # Create an instance of PyStepper for a unipolar motor connected to the
    # pins given in the array. Note that the pin numbers refer to the numers
    # on the 40-pin connector/header, NOT the GPIO numbers
    stepper = PyStepper('BIPOLAR', [ 40, 38, 36, 37 ], [ 22, 11 ])

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
    stepper.cleanup()

