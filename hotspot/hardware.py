# This file defines the interfaces used to command the hardware.

import logging
import numpy as np
import threading
import time

from labjack import ljm

import hotspot.constants as const
from hotspot.hw_context import Serial, openS, eWriteAddress


# Conventions:
# - positive steps/angular rates spin the motor shaft clockwise (CW) when
#   viewed from the rear.
# - negative steps/angular rates spin the motor shaft counterclockwise (CCW)
#   when viewed from the rear.

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
def ezstepper_write(ser: Serial, command_str: str):
    '''
    ser
        pyserial instance, the port to send bytes over from
    '''
    ser.write(command_str.encode())
    logging.debug('EZStepper cmd: {}'.format(command_str.rstrip('\r\n')))
    resp = ser.readline()
    logging.debug('EZStepper response: {}'.format(resp.rstrip(b'\r\n')))
    return resp


def get_encoder_pos(ser: Serial, address):
    resp = ezstepper_write(ser, f'/{address}?8\r\n')
    if resp:
        status = resp[3]
        payload = resp[4:-3]
        ticks = int(payload.decode('utf-8'))
        logger.debug(f'Encoder status: {bin(status)}, encoder counts: {ticks}')
        return ticks
    else:
        logger.warning(f'Encoder {address} didn\'t respond. Assuming 0 pos.')
        return 0


def all_steppers_ez(ser: Serial, addresses, radians: list):
    '''
    Send serial commands to AllMotion EZHR17EN driver boards.
    Driver boards should already be in position-correction mode, so will take
    encoder ticks as commands. The driver boards handle achieving the requested
    absolute position.
    Commands in radians are converted to encoder ticks and sent to driver
    boards in order, based on their addresses [1,2,3,4].

    Parameters
    ----------
    ser
        pyserial instance, the port to send bytes over from
    addresses
        iterable of addresses e.g. [1,2,3,4] to route commands to
    radians
        iterable of signed angle to move each stepper to (radians)

    Returns
    -------
    ticks_to_go
        iterable of integers reporting the number of encoder ticks moved by
        each stepper
    err
        iterable of rounding errors incurred by converting radians to encoder
        ticks
    '''
    # Get the current absolute encoder position
    current_ticks_raw = [get_encoder_pos(ser, address) for address in addresses]
    current_ticks = current_ticks_raw
    logger.debug(f'Current encoder positions before move: {current_ticks}')

    # Get the final absolute position
    radians = np.array(radians)
    ticks_float = radians * const.DEG_PER_RAD / const.DEG_PER_STEP / const.STEP_PER_TICK
    ticks_int = np.round(ticks_float).astype(int)
    ticks_to_go = np.round(ticks_float - current_ticks).astype(int)
    err = (ticks_float - current_ticks) - ticks_to_go

    logger.debug(f'Commanded encoder positions: {ticks_int}')
    logger.debug(f'Delta encoder positions: {ticks_to_go}')

    if any([tick_int < 0 for tick_int in ticks_int]):
        logger.warning(f'Move command past hard stop detected: {addresses} : {ticks_int}')

    # Set the velocity of each motor such that they arrive at the final
    # position at the same time.
    # The motor with the longest move moves at the maximum velocity.
    max_ticks = np.max(np.abs(ticks_to_go))
    t_move = max(max_ticks / const.MAX_SPEED_TICKS, 1e-9)
    vels = np.round(np.abs(ticks_to_go) / t_move).astype(int)
    logger.debug(f'Motor speeds: {vels} ticks / sec')

    for i in range(len(ticks_to_go)):
        if (ticks_to_go[i] and vels[i]):
            # Set velocity
            resp = ezstepper_write(ser, f'/{addresses[i]}V{vels[i]}R\r\n')
            # Command absolute encoder ticks
            resp = ezstepper_write(ser, f'/{addresses[i]}A{ticks_int[i]}\r\n')
    # Execute buffered move commands for all addresses
    ezstepper_write(ser, '/_R\r\n')
    logger.debug(f'Sleeping {t_move:.2f} sec for move')
    time.sleep(t_move + 1e-1)

    return ticks_to_go, err


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