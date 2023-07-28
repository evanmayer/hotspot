# This file defines the interfaces used to command the hardware.

import logging
import numpy as np
import sys
import time

import hotspot.constants as const
from hotspot.hw_context import StepperSerial, HawkeyeSerial

logger = logging.getLogger(__name__)

IDX_READY_BIT = 2
IDX_STATUS_BYTE = 2
NUM_TRAILING_BYTES = 3

# Conventions:
# - positive steps/angular rates spin the motor shaft clockwise (CW) when
#   viewed from the rear.
# - negative steps/angular rates spin the motor shaft counterclockwise (CCW)
#   when viewed from the rear.

# -----------------------------------------------------------------------------
# Helper classes
# -----------------------------------------------------------------------------
class CommandBuffer:
    '''
    Handles building a string of commands to send over serial.

    Continuous movement requires a sequence of carefully orchestrated
    absolute angle and velocity commands. In order to avoid serial transfer/
    ready waiting overhead, it's most efficient to send a long sequence to each
    stepper controller, and then run them all together.

    All methods deal in strings, not byte objects.
    '''
    def __init__(self, addresses):
        '''Addresses are EZStepper bus addresses, e.g. [2,1,3,4]'''
        self.buffer_map = {address : '' for address in addresses}

    def put(self, address, command_str):
        self.buffer_map[address] += command_str
        logger.debug(f'CommandBuffer:put() {address}: {command_str} -> {self.buffer_map[address]}')

    def empty(self, address):
        '''Use after sending'''
        self.buffer_map[address] = ''

    def terminate(self, address):
        '''Use before sending'''
        self.buffer_map[address] += '\r\n'

    def get(self, address):
        return self.buffer_map[address]


# -----------------------------------------------------------------------------
# Stepper functions
# -----------------------------------------------------------------------------
def ezstepper_check_status(resp):
    '''
    Compare the status byte from the EZStepper to some common error codes.

    Parameters
    ----------
    resp
        bytes, EZStepper response, after throwing away garbage bytes

    Returns
    -------
    status_good
        bool, True if response is nonempty and no unusual status byte patterns found
    '''
    status_good = True
    if resp:
        # bits 0-3 form an error code: see EZStepper docs
        status = resp[IDX_STATUS_BYTE] & 0b00001111
    else:
        status = b''
        status_good = False
    if 2 == status:
        logger.warning('Bad command error (illegal command was sent)')
        status_good = False
    if 5 == status:
        logger.warning('Communications Error (Internal communications error)')
        status_good = False
    if 9 == status:
        logger.warning('Overload error (Physical system could not keep up with commanded position)')
        status_good = False
    if 11 == status:
        logger.warning('Move not allowed')
        status_good = False
    if 15 == status:
        logger.warning('Command overflow (unit was already executing a command when another command was received)')
        status_good = False
    if status not in [b'', 0, 2, 9, 11, 15]:
        logger.warning(f'Unrecognized error code {status}. Consult EZStepper documentation.')
    return status_good


def ezstepper_check_ready(resp):
    '''
    The EZStepper is ready to accept a new command if the ready bit is set. 
    
    Parameters
    ----------
    resp
        bytes, EZStepper response, after throwing away garbage bytes

    Returns
    -------
    bool
        True if ready bit is set
    '''
    return bool(resp[IDX_READY_BIT] & 0b00100000)


def wait_for_ready(ser, address, ready_timeout=10.0):
    '''
    Parameters
    ----------
    ser
        pyserial instance, the port to send bytes over from
    address
        str, address of ezstepper
    ready_timeout (optional)
        float, time in seconds before deciding the stepper is unresponsive
    '''
    resp = b''
    ready = False
    if '_' != address:
        # only send a new command if ezstepper not busy
        start_busywait = time.time()
        query = (f'/{address}Q\r\n').encode()
        while not ready and (time.time() - start_busywait) < ready_timeout:
            ser.write(query)
            line = ser.readline()
            if line:
                resp = look_for_response(line)
            logger.debug(f'Ready response: {resp}')
            if resp:
                ready = ezstepper_check_ready(resp)
        status_good = ezstepper_check_status(resp)
        logger.debug(f'Waited {time.time() - start_busywait:.4f} sec for ready signal')
        if not ready:
            logger.warning(f'EZStepper {address} was not ready in allotted time {ready_timeout} sec')
    return


def look_for_response(resp):
    '''
    Responses addressed to the controller node begin with /0, with possible
    junk before.

    Parameters
    ----------
    resp
        bytes, EZStepper response, straight out of serial

    Returns
    -------
    resp
        Any bytes after /0
    '''
    if (b'/0' in resp) and (b'\x03' in resp):
        resp = resp[resp.find(b'/0'):]
    else:
        resp = b''
    return resp


def ezstepper_write(ser: StepperSerial, address, command_str: str):
    '''
    Builds a serial command to send to EZStepper(s), checks status bytes in
    response, and returns response bytes.

    Parameters
    ----------
    ser
        pyserial instance, the port to send bytes over from
    address
        str, address of ezstepper
    command_str
        string, chars to send over EZStepper bus

    Returns
    -------
    resp
        bytes, serial response from EZStepper

    Example:

    ```
    ezstepper.write(ser, 1, 'A1000R\r\n')
    ezstepper.write(ser, '_', 'A12000\r\n')
    ezstepper.write(ser, '_', 'R\r\n')
    ```
    '''
    ser.write((f'/{address}' + command_str).encode())
    logger.debug('EZStepper cmd: {}'.format((f'/{address}' + command_str).rstrip('\r\n')))
    # we do not anticipate a response from "all call" commands.
    if '_' == address:
        resp = b''
    else:
        resp = look_for_response(ser.readline())
        status_good = ezstepper_check_status(resp)
    logger.debug('EZStepper response: {}'.format(resp.rstrip(b'\r\n')))
    return resp


def get_encoder_pos(ser: StepperSerial, address):
    '''
    Gets EZStepper's encoder counts in terms of ticks. It is critical for
    accurate operation that this call succeeds, so the program will exit
    if it fails.

    Parameters
    ----------
    ser
        pyserial instance, the port to send bytes over from
    address
        str, address of ezstepper

    Returns
    -------
    ticks
        Number of encoder ticks counted by encoder. A 10000 line encoder
        has 4 * 10000 = 40000 lines per revolution.
    '''
    ticks = 0
    resp = ezstepper_write(ser, address, '?8R\r\n')
    
    retries = 10
    while not resp and retries > 0:
        resp = ezstepper_write(ser, address, '?8R\r\n')
        retries -= 1
        logger.warning(f'Encoder {address} was slow to respond. Retrying, {retries} remaining.')
    retries = 10
    while not ezstepper_check_status(resp) and retries > 0:
        retries -= 1
        logger.warning(f'Encoder {address} status was bad: {resp}. Retrying, {retries} remaining.')
    if retries < 0:
        logger.critical(f'Encoder {address} status bad: {resp}. Is EZStepper/encoder connected?')
        sys.exit(1)
    else:
        payload = resp[IDX_STATUS_BYTE+1:-NUM_TRAILING_BYTES]
        logger.debug(f'Encoder reading: {payload}')
        if payload:
            ticks = int(payload.decode('utf-8'))
        else:
            logger.critical(f'Encoder {address} didn\'t respond.')
            sys.exit(1)
        return ticks


def bump_hard_stop(ser: StepperSerial, address: int, ticks: int, speed: int, hold_current=50, move_current=50):
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
    max_tries = 10000 # may take several moves to hit a hard stop
    tries = 0
    prev_ticks = 800000 # more ticks than we have total available cable
    curr_ticks = prev_ticks - 1

    while ((curr_ticks < prev_ticks) and (tries < max_tries)):
        prev_ticks = curr_ticks
        wait_for_ready(ser, address)
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


def all_steppers_ez(ser: StepperSerial, addresses: list, radians: list, run=True):
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
    run (optional)
        If True, execute EZStepper command buffer after adding to it.

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
            wait_for_ready(ser, addresses[i])
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
    if run:
        ezstepper_write(ser, '_', 'R\r\n')
    return ticks_to_go, err


# -----------------------------------------------------------------------------
# Hawkeye functions
# -----------------------------------------------------------------------------
def send_hawkeye_byte(ser: HawkeyeSerial, data: int):
    '''
    Control three groups of Hawkeye IR sources on the raft. Each of the first
    three bits of the `data` byte turns on one of the three available groups of
    Hawkeyes on the raft: center, inner ring, and outer ring.
    The byte is sent over serial to a microcontroller, which forwards the byte
    to a shift register on the raft via SPI.

    For example:
    - data = 1 -> 001 -> center Hawkeye only
    - data = 2 -> 010 -> inner ring only
    - data = 3 -> 011 -> center and inner
    - data = 7 -> 111 -> center, inner, outer

    Parameters
    ----------
    ser
        pyserial instance, the port to send byte over from
    data (int)
        integer whose binary representation will appear at the shift register
        outputs on the raft
    '''

    assert isinstance(data, int)
    if data > 255:
        data = 255
    if data < 0:
        data = 0

    ser.write(f'{data}\r\n'.encode())
    return