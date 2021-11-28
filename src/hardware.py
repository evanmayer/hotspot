# This file defines the interface used to command the hardware. It also provides
# "dummy" versions of the interfaces for use in test environments without the
# hardware.

from hw_context import stepper
import constants as const
import logging
import numpy as np


# Conventions:
# - positive steps/angular rates (stepper.FORWARD) spin the motor shaft 
#     clockwise (CW) when viewed from the rear.
# - negative steps/angular rates (stepper.BACKWARD) spin the motor shaft 
#     counterclockwise (CCW) when viewed from the rear.


logger = logging.getLogger(__name__)


def all_steppers(steppers: list, radians: list):
    '''
    The number of steps any motor must take on each loop execution can be cast
    as a special case of Bresenham's Line Algorithm, in the pos quadrant only,
    with all lines starting at (0, 0).
    All motors will either step or not according to the algorithm.
    (We are kind of forgoing linear travel speed control here, but we never
    had reliable speed control anyway, because RPI Debian is not a RT OS.)

    Parameters
    ----------
    steppers
        iterable of 4x Adafruit MotorKit stepper instances 
        (not adafruit_motor.stepper module)
    radians
        iterable of signed angle to move each stepper (radians)
    
    Returns
    -------
    steps_taken
        iterable of integers reporting the number of steps taken by each
        stepper
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
                stepper_n.onestep(style=style, direction=stepper_dirs[i])
                steps_taken[i] += 1
                deltas[i] -= 2 * dx
            deltas[i] += 2 * dy[i]

    return steps_taken * directions


def all_steppers_serial(ser, radians: list):
    '''
    Step by passing the number of steps as a sequence of integers over serial.

    Parameters
    ----------
    serial
        pySerial Serial instance
    radians
        iterable of signed angle to move each stepper (radians)
    
    Returns
    -------
    '''

    steps_to_go = np.round(radians * const.DEG_PER_RAD / (360. / 200. / 1.)).astype(int).astype(str)
    step_str = ','.join(steps_to_go) + '\n'
    ser.write(step_str.encode())


if __name__ == '__main__':
    import serial
    import time

    with serial.Serial('COM5', 115200) as ser:
        time.sleep(2)
        all_steppers_serial(ser, np.array(4*[2. * np.pi / 10.]))
        all_steppers_serial(ser, np.array(4*[-2. * np.pi]))
