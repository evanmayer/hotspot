# Numbers that don't change from mirror to mirror
# MKS units throughout if not specified.

import numpy as np
import os
import sys


# -----------------------------------------------------------------------------
# Stepper drivers
# -----------------------------------------------------------------------------
# EZStepper microstepping multiplier. Divides each single step by the modifier:
# E.g. a value of 8 splits one 1.8 deg step into eight .225 deg steps.
# Default value is 256, and it is discouraged to change it. If you do, you must
# send a command to each EZStepper to match your decision here.
MICROSTEP_NUM = 256
ENCODER_TICKS_PER_REV = 40000
# Used for converting rotational changes into stepper commands
DEG_PER_STEP = 360. / 200. / MICROSTEP_NUM
DEG_PER_RAD = 180. / np.pi
STEP_PER_TICK = 200. * MICROSTEP_NUM / ENCODER_TICKS_PER_REV
# ticks / sec, experimentally determined. EZStepper driver takes 10-40V
# more PSU volts = faster current rise time = faster max speed.
MAX_SPEED_TICKS = 300000
# Default for EZSteppers
SERIAL_BAUD = 9600
# if you get garbage replies from stepper drivers, increase this timeout.
SERIAL_TIMEOUT = 0.07

# -----------------------------------------------------------------------------
# Physical constants
# -----------------------------------------------------------------------------
# Used for converting linear distances into rotational ones.
# This is the measured value of the helical drum minor diameter.
PULLEY_RADIUS = .030132 / 2.
# The helical drum means the length unspooled per radian is longer.
# 1.5 mm pitch
DRUM_PITCH = 1.5 / 1000.
# delta x distance from the NW eyelet to the raft centroid, when it is driven
# to the NW limit and the raft isn't cockeyed
HOMING_OFFSET_X = 0.0276
# same, but for y
HOMING_OFFSET_Y = -0.0252

# -----------------------------------------------------------------------------
# Software constants
# -----------------------------------------------------------------------------
# Limits the total number of commands that may be in the command queue at once.
# Pretty much only limited by memory.
MAX_QLEN = 2**16
# If the serial-RS485 converter is not found, change this accordingly
if sys.platform.startswith('win'):
    SERIAL_PORT = 'COM8'
else:
    SERIAL_PORT = '/dev/ttyUSB0'
# Helps with file input/outputs
TOPLEVEL_DIR = os.path.abspath(os.path.join(__file__, '..', '..'))