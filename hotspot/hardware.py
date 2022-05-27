# This file defines the interfaces used to command the hardware.

import logging
import numpy as np
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
def ezstepper_write(ser, command_str: str):
    '''
    ser
        pyserial instance, the port to send bytes over from
    '''
    ser.write(command_str.encode())
    logging.debug(f'EZStepper cmd: {command_str}')
    resp = ser.readline().decode('latin-1')
    logging.debug(f'EZStepper response: {resp}')
    return resp


def all_steppers_ez(ser, addresses, radians: list):
    '''
    Send serial commands to AllMotion EZHR17EN driver boards.
    Driver boards should already be in position-correction mode, so will take
    encoder ticks as commands. The driver boards handle achieving the requested
    position.
    Commands in radians are converted to encoder ticks and sent to driver
    boards in order, based on their addresses [1,2,3,4].

    Parameters
    ----------
    ser
        pyserial instance, the port to send bytes over from
    addresses
        iterable of addresses e.g. [1,2,3,4] to route commands to
    radians
        iterable of signed angle to move each stepper (radians)

    Returns
    -------
    ticks_to_go
        iterable of integers reporting the number of encoder ticks moved by
        each stepper
    err
        iterable of rounding errors incurred by converting radians to encoder
        ticks
    '''
    radians = np.array(radians)
    ticks_float = radians * const.DEG_PER_RAD / const.DEG_PER_STEP / const.STEP_PER_TICK
    ticks_to_go = np.round(ticks_float).astype(int)
    err = ticks_float - ticks_to_go

    # Set the velocity of each motor such that they arrive at the final
    # position at the same time.
    # The motor with the longest move moves at the maximum velocity.
    max_ticks = np.max(np.abs(ticks_to_go))
    t_move = max(max_ticks / const.MAX_SPEED_TICKS, 1e-9)
    vels = np.round(ticks_to_go / t_move).astype(int)
    logger.debug(f'Motor velocities: {vels} ticks / sec')

    for i in range(len(ticks_to_go)):
        if (ticks_to_go[i] and vels[i]):
            vel_cmd = f'/{addresses[i]}V{vels[i]}'
            resp = ezstepper_write(ser, vel_cmd)
            # Command pos/neg encoder ticks relative to pos direction
            pos_cmd = f'/{addresses[i]}P{ticks_to_go[i]}'
            resp = ezstepper_write(ser, pos_cmd)
    # Execute buffered move commands for all addresses
    ser.write('/_R\n'.encode())
    time.sleep(t_move + 1e-1)

    return ticks_to_go, err


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

    steps_float = radians * const.DEG_PER_RAD / const.DEG_PER_STEP
    # We can detect and correct at the end of each move, using the accumulated
    # rounding errors.
    steps_to_go = np.round(steps_float).astype(int)
    err = steps_float - steps_to_go
    steps_to_go = np.abs(steps_to_go)

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
                time.sleep(const.STEP_WAIT)
                steps_taken[order[i]] += directions[i].astype(int)
                deltas[i] -= 2 * dx
            deltas[i] += 2 * dy[i]

    return order, steps_taken, err[order]


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