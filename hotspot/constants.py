# Numbers that don't change from mirror to mirror
# MKS units throughout if not specified.

import numpy as np
import os

from hotspot.hw_context import stepper


# Adafruit stacking motor driver hats are addressable.
HAT_0_ADDR = 0x60
HAT_1_ADDR = 0x61

PWM_FREQ = 1600

# Adafruit stepper style. Used to enable/disable microstepping.
STEPPER_STYLE = stepper.INTERLEAVE

# Adafruit microstepping modifier. Divides each single step by the modifier:
# E.g. a value of 8 splits one 1.8 deg step into eight .225 deg steps.
# Even numbers from 2-8.
MICROSTEP_NUM = 2

# Used for converting rotational changes into stepper commands
DEG_PER_STEP = 360. / 200. / MICROSTEP_NUM

DEG_PER_RAD = 180. / np.pi

# Used for converting linear distances into rotational ones
PULLEY_RADIUS = 0.03 / 2.

# Limits the total number of commands that may be in the command queue at once.
# Pretty much only limited by memory.
MAX_QLEN = 2**16

# delta x distance from the NW eyelet to the raft centroid, when it is driven
# to the NW limit and the raft isn't cockeyed
HOMING_OFFSET_X = 0.0276
# same, but for y
HOMING_OFFSET_Y = -0.0252

# Not sure how issuing step commands to multiple motors too fast will mess
# with the motor hat timing, so wait a bit just to be safe
STEP_WAIT = 1e-3

# Backup vars in case we need to switch to arduino motor control over serial
# SERIAL_PORT = '/dev/ttyACM0'
# SERIAL_BAUD = 115200

# Helps with I/O
TOPLEVEL_DIR = os.path.abspath(os.path.join(__file__, '..', '..'))