# This file defines the interface used to command the hardware. It also provides
# "dummy" versions of the interfaces for use in test environments without the
# hardware.

from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import constants as const
import numpy as np
import time

