# This file defines the interfaces used to command the hardware.

import logging
import numpy as np
import serial
import threading
import time

from labjack import ljm

import hotspot.constants as const
from hotspot.hw_context import stepper, openS, eWriteAddress


# Conventions:
# - positive steps/angular rates (stepper.FORWARD) spin the motor shaft 
#     clockwise (CW) when viewed from the rear.
# - negative steps/angular rates (stepper.BACKWARD) spin the motor shaft 
#     counterclockwise (CCW) when viewed from the rear.

logger = logging.getLogger(__name__)

# Map relay number to modbus register on the LJ
RELAY_DICT = {
    '1': 2008,'2': 2009,'3': 2010,'4': 2011,'5': 2012,'6': 2013,
    '7': 2014,'8': 2015,'9': 2016,'10': 2017,'11': 2018,'12': 2019
}
# Needed to open connection to labjack board
MODEL_NAME = 'T7'
MODE = 'USB'

# -----------------------------------------------------------------------------
# Stepper functions
# -----------------------------------------------------------------------------
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

    # Perform steps in an order that avoids over-tension to mitigate skipping
    # steps: positive steps first to unwind cable, then negative
    order = np.argsort(directions)[::-1].astype(int)
    directions = np.array(directions)[order]
    radians = np.array(radians)[order]
    steppers = np.array(steppers)[order]

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
    This is included as a contingency in the event that Arduino is necessary.

    Parameters
    ----------
    serial
        pySerial Serial instance
    radians
        iterable of signed angle to move each stepper (radians)
    
    Returns
    -------
    '''
    # arduino has stepping mode hardcoded, so dont allow runtime changes of steps to radian modifier
    steps_to_go = np.round(np.array(radians) * const.DEG_PER_RAD / (360. / 200. / 8.)).astype(int).astype(str)
    step_str = ','.join(steps_to_go) + '\n'
    ser.write(step_str.encode())
    return steps_to_go.astype(int)


# -----------------------------------------------------------------------------
# LabJack Functions
# -----------------------------------------------------------------------------
def try_open(model: str, mode: str, retry=True):
    '''
    Try and open a connection to a LabJack.

    Parameters
    ----------
    model
        LabJack board model name, typ. 'T7'
    mode
        LabJack communication mode, typ. 'USB'

    Returns
    -------
    name
        handle to opened LabJack board instance
    '''
    try:
        name = openS(model, mode)
    except ljm.LJMError as err:
        print('Error opening LJ connection')
        print(err)
        if retry:
            time.sleep(1)
            print('Trying again in 1s.')
            name = try_open(model, mode, retry=False)
        else:
            return -1
            
    return name


def write_value(handle, addr: int, value=0):
    '''
    Write a value to a LabJack, catching errors.
    
    Parameters
    ----------
    handle
        LabJack board model handle from `try_open`
    addr
        LabJack relay address integer

    Kwargs
    ------
    value: int
        value to write to LabJack relay

    Returns
    -------
    bool
        True if successful, False if not
    '''
    try:
        eWriteAddress(handle, addr, 0, value)
    except ljm.LJMError as err:
        print("Error in write to LJ, specific error is:")
        print(err)
        return False
    return True


def threaded_write(handle, target: int, value: int):
    '''
    Wrapper around `write_value` for use in threaded calls.
    
    Parameters
    ----------
    handle
        LabJack board model handle from `try_open`
    target
        LabJack relay address integer
    '''
    logger.debug(f'Trying address {target} state {value}')
    written = False
    while(1):
        try:
            if written == False:
                written = write_value(handle, target, value=value)
                if written == False:
                    time.sleep(1)
                else:
                    break
            else:
                break
        except KeyboardInterrupt:
            answer = input('Do you want to interrupt? y/n')
            if answer.lower() == 'y':
                break
    return


def spawn_all_threads(handle, states: list):
    '''
    Spawns threads and passes the states each relay will need to have
    
    Parameters
    ----------
    handle
        LabJack board model handle from `try_open`
    states
        iterable of integers describing the states each relay should take
    '''
    for key in RELAY_DICT.keys():
        thread = threading.Thread(
            target=threaded_write,
            args=(handle, RELAY_DICT[key],
            states[int(key) - 1]),
            daemon=True
        )
        thread.start()

    return


def spawn_all_threads_off(handle):
    '''
    Spawns threads and sets all relay states off.
    
    Parameters
    ----------
    handle
        LabJack board model handle from `try_open`
    '''
    for key in RELAY_DICT.keys():
        thread = threading.Thread(
            target=threaded_write,
            args=(handle,RELAY_DICT[key], 0),
            daemon=True
        )
        thread.start()
    return


if __name__ == '__main__':
    pass
    # import serial
    # import time

    # with serial.Serial('COM5', 115200) as ser:
    #     time.sleep(2)
    #     all_steppers_serial(ser, np.array(4*[2. * np.pi / 10.]))
    #     all_steppers_serial(ser, np.array(4*[-2. * np.pi]))

    # from hw_context import MotorKit
    # kit0 = MotorKit(address=const.HAT_0_ADDR, steppers_microsteps=const.MICROSTEP_NUM, pwm_frequency=const.PWM_FREQ)
    # kit1 = MotorKit(address=const.HAT_0_ADDR, steppers_microsteps=const.MICROSTEP_NUM, pwm_frequency=const.PWM_FREQ)
    # steppers = [
    #     kit1.stepper2, # sw
    #     kit0.stepper1, # ne
    #     kit0.stepper2, # nw
    #     kit1.stepper1] # se
    # all_steppers(steppers, [np.pi/4., 0, np.pi/4., 0])
    # all_steppers(steppers, [0, -np.pi/4., 0, -np.pi/4.])
    # [stepper.release() for stepper in steppers]
