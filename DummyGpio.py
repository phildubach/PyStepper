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

def __log(str):
    print __name__, ":", str

def setmode(mode):
    __log("Setting mode to %s" % (str(mode),))

def setup(channel, direction, pull_up_down=PUD_OFF, initial=None):
    # TODO log all args
    __log("Setting up channel %d for %s" % (channel, direction))

def output(channel, value):
    __log("Setting output %d to %s" % (channel, value))

def input(channel):
    value = True
    __log("Reading input %d as %s" % (channel, value))
    return value

def cleanup():
    __log("Cleanup called")
