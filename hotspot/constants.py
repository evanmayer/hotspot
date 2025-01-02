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
# encoder ticks / sec, experimentally determined. EZStepper driver takes 10-40V
# more PSU volts = faster current rise time = faster max speed.
MAX_SPEED_TICKS = 25000
MAX_ACCEL_TICKS = 400
# Default for EZSteppers and hawkeye driver peripheral
SERIAL_BAUD = 9600
# BIG MODE
SERIAL_BAUD_FAST = 115200
# if you get garbage replies from stepper drivers, increase this timeout.
SERIAL_TIMEOUT = 0.1
# time before performing any actions at each position reached
SETTLE_TIME = 1e-3
# for long moves, approximate the intermediate path by moves of this maximum length, m
CHUNK_DIST = 0.005
# stepper move current = 2.0 A * (this / 100)
MOVE_CURRENT_PCT = 50
HOLD_CURRENT_PCT = 50

# -----------------------------------------------------------------------------
# Physical constants
# -----------------------------------------------------------------------------
# Used for converting linear distances into rotational ones.
# This is the measured value of the helical drum minor diameter.
PULLEY_RADIUS = .03014 / 2.
# The helical drum means the length unspooled per radian is longer.
# 1.5 mm pitch
DRUM_PITCH = 1.5 / 1000.
LENGTH_PER_REV = np.sqrt(DRUM_PITCH ** 2. + (np.pi * 2. * PULLEY_RADIUS) ** 2.)

# delta x distance from the NW eyelet to the raft centroid, when it is driven
# to the NW limit and the raft isn't cockeyed
HOMING_OFFSET_X = 0.0276
# same, but for y
HOMING_OFFSET_Y = -0.0252

# used for reducing error incurred by spool to eyelet span
# changing as a function of cable spooled on
SPOOL_CENTER_TO_EYELET_X = 0.494 # mm

# -----------------------------------------------------------------------------
# Software constants
# -----------------------------------------------------------------------------
# Hawkeye flasher frequency
HAWKEYE_FLASH_FREQ_HZ = 5.
# Hawkeye number of flashes
HAWKEYE_NUM_FLASHES = 10
# Limits the total number of commands that may be in the command queue at once.
# Pretty much only limited by memory.
MAX_QLEN = 2**16
# Change these accordingly to point to the serial ports of the microcontroller
# for the Hawkeyes and and serial-RS485 converter for the steppers. 
if sys.platform.startswith('win'):
    HAWKEYE_SERIAL_PORT = 'COM8'
    STEPPER_SERIAL_PORT = 'COM9'
else:
    HAWKEYE_SERIAL_PORT = '/dev/ttyACM0'
    STEPPER_SERIAL_PORT = '/dev/ttyUSB0'
# Helps with file input/outputs
TOPLEVEL_DIR = os.path.abspath(os.path.join(__file__, '..', '..'))
