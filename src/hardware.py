# This file defines the interface used to command the hardware. It also provides
# "dummy" versions of the interfaces for use in test environments without the
# hardware.

from adafruit_motor import stepper
import asyncio
import constants as const
import logging
import numpy as np
from time import time, sleep


# Conventions:
# - positive steps/angular rates (stepper.FORWARD) spin the motor shaft 
#     clockwise (CW) when viewed from the rear.
# - negative steps/angular rates (stepper.BACKWARD) spin the motor shaft 
#     counterclockwise (CCW) when viewed from the rear.


logger = logging.getLogger(__name__)
logger.setLevel(getattr(logging, 'DEBUG'))#const.LOGLEVEL))


async def move_motor(stepper_n, radians: float, rad_per_sec: float):
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
        logger.debug('Commanded angular rate close to zero. Motor doesn\'t move.')
        return
    
    stepper_dir = stepper.FORWARD
    if direction == -1:
        stepper_dir = stepper.BACKWARD

    logger.debug(f'([{time()}]: Stepper {stepper_n} move started.')
    deg_per_step = const.DEG_PER_STEP
    style = const.STEPPER_STYLE
    for i in range(steps_to_go):
        # fudge more accurate steps/sec by subtracting off execution time of non-sleeping part of loop.
        loop_start = time()
        stepper_n.onestep(style=style, direction=stepper_dir)
        # sleep for the remaining time to keep issuing steps at proper rate.
        time_rem = deg_per_step / deg_per_sec - (time() - loop_start)
        if time_rem <= 0.:
            logger.warning('Commanded angular rate exceeded what can be commanded'
                + ' reliably.')
            continue
        await asyncio.sleep(time_rem)

    logger.debug(f'([{time()}]: Stepper {stepper_n} move complete.')


def all_steppers(steppers: list, radians: list, rad_per_secs: list):
    '''
    Perform a timed while loop to move the commanded angle at the commanded
    rate.

    Parameters
    ----------
    steppers
        iterable of 4x Adafruit MotorKit stepper instances 
        (not adafruit_motor.stepper module)
    radians
        iterable of signed angle to move each stepper (radians)
    rad_per_secs
        iterable of unsigned angular rates (radians per second)
    
    Returns
    -------
    '''
    deg_per_step = const.DEG_PER_STEP
    style = const.STEPPER_STYLE
    
    directions = np.sign(radians)
    steps_to_go = np.round(np.abs(radians) * const.DEG_PER_RAD / const.DEG_PER_STEP).astype(int)
    deg_per_secs = np.abs(rad_per_secs) * const.DEG_PER_RAD
    
    stepper_dirs = [stepper.FORWARD] * 4
    for j, direction in enumerate(directions):
        if direction == -1:
            stepper_dirs[j] = stepper.BACKWARD

    # determine the minimum number of steps any motor must take
    min_steps = np.min(steps_to_go)
    steps_per_iter = np.round(steps_to_go / min_steps).astype(int)
    print(steps_to_go, min_steps, steps_per_iter)
    # and the amount of time it has to take them
    # then execute a loop of that many steps, adding steps as necessary in
    # order to make all steppers move the required number of steps.
    for i in range(min_steps):
        for j, stepper_n in enumerate(steppers):
            # print(f'stepping stepper {j} {steps_per_iter[j]} steps')
            for k in range(steps_per_iter[j]):
                stepper_n.onestep(style=style, direction=stepper_dirs[j])
                sleep(1e-9)

    # logger.debug(f'([{time()}]: Stepper {stepper_n} move complete.')