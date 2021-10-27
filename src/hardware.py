# This file defines the interface used to command the hardware. It also provides
# "dummy" versions of the interfaces for use in test environments without the
# hardware.

from adafruit_motor import stepper
import constants as const
import logging
import numpy as np
import time


# Conventions:
# - positive steps/angular rates (stepper.FORWARD) spin the motor shaft 
#     clockwise (CW) when viewed from the rear.
# - negative steps/angular rates (stepper.BACKWARD) spin the motor shaft 
#     counterclockwise (CCW) when viewed from the rear.


logger = logging.getLogger(__name__)
logger.setLevel(getattr(logging, 'DEBUG'))


def move_motor(stepper_n, radians: float, rad_per_sec: float):
    '''
    Perform a timed while loop to move the commanded angle at the commanded
    rate.

    Parameters
    ----------
    stepper_n
        Adafruit MotorKit stepper instance (not adafruit_motor.stepper module)
    radians
        Signed angle to move stepper (radians)
    rad_per_sec
        Unsigned angular rate (radians per second)
    
    Returns
    -------
    '''
    direction = np.sign(radians)
    steps_to_go = np.round(abs(radians) * const.DEG_PER_RAD / const.DEG_PER_STEP).astype(int)
    deg_per_sec = abs(rad_per_sec) * const.DEG_PER_RAD
    
    if deg_per_sec < np.finfo(float).eps:
        raise ValueError('Angular rate must be greater than zero.') 
    
    stepper_dir = stepper.FORWARD
    if direction == -1:
        stepper_dir = stepper.BACKWARD

    while steps_to_go:
        # fudge more accurate steps/sec by subtracting off execution time of non-sleeping part of loop.
        loop_start = time.time()
        stepper_n.onestep(style=const.STEPPER_STYLE, direction=stepper_dir)
        steps_to_go -= 1
        # sleep for the remaining time to keep issuing steps at proper rate.
        time_rem = 1./(deg_per_sec / const.DEG_PER_STEP) - (time.time() - loop_start)
        if time_rem <= 0.:
            logger.warning('Commanded angular rate exceeds what can be commanded'
                + ' reliably.')
            continue
        time.sleep(time_rem)

    logger.info(f'Stepper {stepper_n} move complete.')
