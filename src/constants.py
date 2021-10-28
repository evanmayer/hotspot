# Numbers that don't change from mirror to mirror
# MKS units throughout if not specified.
from adafruit_motor import stepper
import numpy as np
import os

# Adafruit stacking motor driver hats are addressable.
HAT_0_ADDR = 0x60
HAT_1_ADDR = 0x61

# Adafruit stepper style. Used to enable/disable microstepping.
# options: MICROSTEP, INTERLEAVE, DOUBLE, SINGLE
STEPPER_STYLE = stepper.MICROSTEP

PWM_FREQ = 2400

# Adafruit microstepping modifier. Divides each single step by the modifier:
# E.g. a value of 8 splits one 1.8 deg step into eight .225 deg steps.
# Even numbers from 2-8.
MICROSTEP_NUM = 4

# Used for converting rotational changes into stepper commands
DEG_PER_STEP = 360. / 200. / MICROSTEP_NUM

DEG_PER_RAD = 180. / np.pi

# Used for converting linear distances into rotational ones
PULLEY_RADIUS = 0.03

# Limits the total number of commands that may be in the command queue at once.
# Pretty much only limited by memory.
MAX_COMMANDS = 2**17

# Helps with I/O
TOPLEVEL_DIR = os.path.abspath(os.path.join(__file__, '..', '..'))