# Numbers that don't change from mirror to mirror
import numpy as np

DEG_PER_RAD = 180. / np.pi
# Used for converting linear distances into rotational ones
PULLEY_RADIUS = 1.
# Used for converting rotational changes into stepper commands
DEG_PER_STEP = 1.8 # be careful, different if microstepping used