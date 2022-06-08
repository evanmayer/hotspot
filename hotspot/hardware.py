# This file defines the interfaces used to command the hardware.

import logging
import numpy as np
import sys
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
def ezstepper_check_status(resp):
    status_good = True
    if resp:
        status = resp[3] & 0b0001111 # bits 0-3 form an error code
    else:
        status = b''
        status_good = False
    if 9 == status:
        logger.warning('Overload Error (Physical system could not keep up with commanded position)')
        status_good = False
    if 15 == status:
        logger.warning('Command overflow (unit was already executing a command when another command was received)')
        status_good = False
    if status:
        logger.warning(f'Unrecognized error code {status}. Consult EZStepper documentation.')
    return status_good


def ezstepper_check_ready(resp):
    '''
    The EZStepper is ready to accept a new command if the ready bit is set. 
    '''
    return bool(resp[3] & 0b0100000)


def wait_for_ready(ser, address, ready_timeout=1):
    if '_' != address:
        # only send a new command if ezstepper not busy
        start_busywait = time.time()
        ready = False
        while not ready and (time.time() - start_busywait) < ready_timeout:
            ser.write((f'/{address}Q\r\n').encode())
            resp = ser.readline()
            if resp:
                ready = ezstepper_check_ready(resp)
        # logger.debug(f'Waited {time.time() - start_busywait:.2f} sec for ready signal')
        if not ready:
            logger.warning(f'EZStepper {address} was not ready in allotted time {ready_timeout} sec')
    return


def ezstepper_write(ser: Serial, address, command_str: str):
    '''
    ser
        pyserial instance, the port to send bytes over from
    address
        str, address of ezstepper
    command_str
        string, chars to send over EZStepper bus
    busy_timeout
        float, stop trying to wait for busy ezstepper after this time
    '''
    wait_for_ready(ser, address)
    ser.write((f'/{address}' + command_str).encode())
    logger.debug('EZStepper cmd: {}{}'.format(address, command_str.rstrip('\r\n')))
    resp = ser.readline()
    status_good = ezstepper_check_status(resp)
    logger.debug('EZStepper response: {}'.format(resp.rstrip(b'\r\n')))
    return resp


def get_encoder_pos(ser: Serial, address):
    resp = ezstepper_write(ser, address, '?8R\r\n')
    if not ezstepper_check_status(resp):
        logger.critical(f'Encoder {address} status bad: {resp}.')
        sys.exit(1)
    else:
        payload = resp[4:-3]
        if payload:
            ticks = int(payload.decode('utf-8'))
        else:
            logger.critical(f'Encoder {address} didn\'t respond.')
            sys.exit(1)
        return ticks
        


def bump_hard_stop(ser: Serial, address: int, ticks: int, speed: int, hold_current=50, move_current=50):
    '''
    Run the motor backward until it can't anymore.

    Parameters
    ----------
    ser
        pyserial instance, the port to send bytes over from
    address
        ezstepper selector switch number to send command to
    ticks
        encoder ticks to move on each trial while searching for hard stop
    speed
        encoder ticks per second
    hold_current (optional)
        target current, % of 2 A limit for motor when stationary
    move_current (optional)
        target current, % of 2 A limit for motor when moving

    Returns
    -------
    prev_ticks
        The last encoder position before the hard stop was hit
    '''
    t_move = ticks / speed
    max_tries = 10000
    tries = 0
    prev_ticks = 800000
    curr_ticks = prev_ticks - 1

    while ((curr_ticks < prev_ticks) and (tries < max_tries)):
        prev_ticks = curr_ticks
        resp = ezstepper_write(
            ser,
            address,
            (
                f'm{move_current}' +
                f'h{hold_current}' +
                f'V{speed}' +
                f'D{ticks}' +
                'R\r\n'
            )
        )
        curr_ticks = get_encoder_pos(ser, address)
        tries += 1

    return prev_ticks


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
            # Set velocity and command absolute encoder ticks
            resp = ezstepper_write(
                ser,
                addresses[i],
                (
                    f'V{vels[i]}' + 
                    f'A{ticks_int[i]}'
                  + '\r\n'
                )
            )
    # Execute buffered move commands for all addresses
    ezstepper_write(ser, '_', 'R\r\n')

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