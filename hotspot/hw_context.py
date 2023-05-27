# This file abstracts the hardware module imports away and provides contingency
# modules so dev and testing can be done on a machine without hardware
# attached.

import serial

import hotspot.constants as const


class DummySerial:
    def __init__(self, *args, **kwargs):
        pass
    def write(self, bytes):
        pass
    def readline(self):
        return b''
    def close(self):
        pass

try:
    stepper_serial_instance = serial.Serial(const.STEPPER_SERIAL_PORT, const.SERIAL_BAUD, timeout=const.SERIAL_TIMEOUT)
    StepperSerial = serial.Serial
except serial.serialutil.SerialException as err:
    print(err)
    print(f'Open constants.py and try another STEPPER_SERIAL_PORT, or connect'
        + ' a device to the serial port.')
    print('This program cannot continue without steppers connected.')
    stepper_serial_instance = DummySerial()
    StepperSerial = DummySerial


try:
    hawkeye_serial_instance = serial.Serial(const.HAWKEYE_SERIAL_PORT, const.SERIAL_BAUD, timeout=const.SERIAL_TIMEOUT)
    HawkeyeSerial = serial.Serial
except serial.serialutil.SerialException as err:
    print(err)
    print(f'Open constants.py and try another HAWKEYE_SERIAL_PORT, or connect'
        + ' a device to the serial port.')
    print('Continuing with dummy serial port.')
    hawkeye_serial_instance = DummySerial()
    HawkeyeSerial = DummySerial
