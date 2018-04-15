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

"""Dummy RPi.GPIO package for testing.

Partially emulate the interface of RPi.GPIO to allow testing on platforms other
than Raspberry Pi. Generally prints API calls rather than toggling any actual
I/O pins.
"""

BOARD = "BOARD"
BCM = "BCM"

OUT = "OUT"
IN = "IN"

HIGH = True
LOW = False

PUD_OFF = "PUD_OFF"
PUD_UP = "PUD_UP"
PUD_DOWN = "PUD_DOWN"

SEQUENCE = [ [ False, False, False, True  ],
             [ False, True,  False, True  ],
             [ False, True , False, False ],
             [ False, True , True , False ],
             [ False, False, True , False ],
             [ True,  False, True , False ],
             [ True,  False, False, False ],
             [ True,  False, False, True  ] ]

outputs = []
output_values = {}
inputs = []
position = 42  # arbitrary initial position
index = None   # index of last known output set

def __log(str):
    print __name__, ":", str

def setmode(mode):
    __log("Setting mode to %s" % (str(mode),))

def setup(channel, direction, pull_up_down=PUD_OFF, initial=None):
    # TODO log all args
    __log("Setting up channel %d for %s" % (channel, direction))
    # record output init sequence and assume they correspond to motor
    # outputs in the order given in the SEQUENCE array
    if direction == OUT and len(outputs) < 4:
        value = initial
        if value == None:
            value = False
        outputs.append(channel)
        output(channel, value)
    elif direction == IN and len(inputs) < 2:
        inputs.append(channel)

def find_index():
    # TODO more efficient way
    current = []
    for output in outputs:
        current.append(output_values[output])
    for index in range(len(SEQUENCE)):
        if SEQUENCE[index] == current:
            return index
    return None

def output(channel, value):
    __log("Setting output %d to %s" % (channel, value))
    global index, position, output_values
    output_values[channel] = value
    new_index = find_index()
    if not new_index == None:
        if not index == None:
            temp = new_index
            diff = new_index - index
            if abs(diff) > 1:
                # overflow
                diff = -diff / abs(diff)
            position += diff
            print "New position:", position, "index:", new_index
        index = new_index

def input(channel):
    value = HIGH
    if channel == inputs[0] and position <= 0:
        value = LOW
    __log("Reading input %d as %s" % (channel, value))
    return value

def cleanup():
    __log("Cleanup called")
