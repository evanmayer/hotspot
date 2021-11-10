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


def all_steppers(steppers: list, radians: list):
    '''
    The number of steps any motor must take on each loop execution can be cast
    as a special case of Bresenham's Line Algorithm, in the pos quadrant only,
    with all lines starting at (0, 0).
    All motors will either step or not according to the algorithm.
    (We are kind of forgoing linear travel speed control here)

    Parameters
    ----------
    steppers
        iterable of 4x Adafruit MotorKit stepper instances 
        (not adafruit_motor.stepper module)
    radians
        iterable of signed angle to move each stepper (radians)
    
    Returns
    -------
    '''
    style = const.STEPPER_STYLE
    
    directions = np.sign(radians)
    steps_to_go = np.round(np.abs(radians) * const.DEG_PER_RAD / const.DEG_PER_STEP).astype(int)
    
    stepper_dirs = [stepper.FORWARD] * 4
    for i, direction in enumerate(directions):
        if direction == -1:
            stepper_dirs[i] = stepper.BACKWARD

    # Normalize the slopes of the lines by making the motor that must take
    # the most total steps have a slope = 1, or 1 step per loop cycle.
    dx = np.max(steps_to_go)
    dy = steps_to_go
    steps_taken = [0] * 4
    deltas = 2 * dy - dx # 2x to allow integer arithmetic
    for _ in range(dx):
        for i, stepper_n in enumerate(steppers):
            # decide whether to step or not
            if deltas[i] > 0:
                # stepper_n.onestep(style=style, direction=stepper_dirs[i])
                steps_taken[i] += 1
                deltas[i] -= 2 * dx
                # print('stepping:' + ' ' * i + f'{i}')
            deltas[i] += 2 * dy[i]

        sleep(1e-9)
        print(steps_taken)


if __name__ == '__main__':
    all_steppers([None] * 4, [(4 - i+1) * np.pi / 8 for i in range(4)])