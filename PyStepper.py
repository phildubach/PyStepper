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
import time

class PyStepper:
    """Provides low-level functions for controlling a stepper motor connected to
    GPIO pins on a Raspberry Pi"""

    UNIPOLAR = 0
    BIPOLAR = 1
    FORWARD = 1
    BACKWARD = -1

    sequences = [ [ # UNIPOLAR
                    [ True,  False, False, False ],
                    [ True,  True,  False, False ],
                    [ False, True,  False, False ],
                    [ False, True,  True,  False ],
                    [ False, False, True,  False ],
                    [ False, False, True,  True  ],
                    [ False, False, False, True  ],
                    [ True,  False, False, True  ] ],
                  [ # BIPOLAR
                    [ False, True,  False, True  ],
                    [ False, False, False, True  ],
                    [ True,  False, False, True  ],
                    [ True,  False, False, False ],
                    [ True,  False, True , False ],
                    [ False, False, True , False ],
                    [ False, True , True , False ],
                    [ False, True , False, False ] ]
                ]


    def __init__(self, sequence, pins):
        """Create an instance of PyStepper.

        Arguments:
        sequence -- either PyStepper.UNIPOLAR or PyStepper.BIPOLAR
        pins     -- an array of 4 pin numbers referring to the pin numbers on
                    the connector
        """
        
        self.sequence = PyStepper.sequences[sequence]
        self.pins = pins
        Gpio.setmode(Gpio.BOARD)
        Gpio.setwarnings(False)
        self.position = 0;
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

if __name__ == '__main__':
    # Create an instance of PyStepper for a unipolar motor connected to the
    # pins given in the array. Note that the pin numbers refer to the numers
    # on the 40-pin connector/header, NOT the GPIO numbers
    stepper = PyStepper(PyStepper.BIPOLAR, [ 40, 38, 36, 37 ])

    # Test by taking a number of steps
    for i in range(200):
	# arbitrarily choose to step forward
        stepper.forward()

	# sleep for 10 milliseconds between steps; this determines the speed of
	# rotation; maximum speed depends on the motor and power supply
        time.sleep(0.01)

    # Before exiting, clean up, redefining all pins as INPUTS
    Gpio.cleanup()

