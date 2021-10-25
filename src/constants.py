# Numbers that don't change from mirror to mirror
# MKS units throughout if not specified.
from adafruit_motor import stepper
import numpy as np

# Adafruit stepper style. Used to enable/disable microstepping.
STEPPER_STYLE = stepper.MICROSTEP

# Adafruit microstepping modifier. Divides each single step by the modifier:
# E.g. a value of 8 splits one 1.8 deg step into eight .225 deg steps.
# Even numbers from 2-8.
MICROSTEP_NUM = 2 

# Used for converting rotational changes into stepper commands
DEG_PER_STEP = 360. / 200. / MICROSTEP_NUM

DEG_PER_RAD = 180. / np.pi
# Used for converting linear distances into rotational ones
PULLEY_RADIUS = 0.03