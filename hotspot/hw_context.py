# This file abstracts the hardware module imports away and provides contingency
# modules so dev and testing can be done on a machine without hardware
# attached.

import serial

import hotspot.constants as const


try:
    stepper_serial_instance = serial.Serial(const.STEPPER_SERIAL_PORT, const.SERIAL_BAUD, timeout=const.SERIAL_TIMEOUT)
    Serial = serial.Serial
except serial.serialutil.SerialException as err:
    print(err)
    print(f'Open constants.py and try another STEPPER_SERIAL_PORT, or connect'
        + ' a device to the serial port.')
    print('Continuing with dummy serial port.')
    class DummySerial:
        def __init__(self):
            pass
        def write(self, bytes):
            pass
        def readline(self):
            return b''
        def close(self):
            pass
    stepper_serial_instance = DummySerial()
    Serial = DummySerial


try:
    hawkeye_serial_instance = serial.Serial(const.HAWKEYE_SERIAL_PORT, const.SERIAL_BAUD, timeout=const.SERIAL_TIMEOUT)
    Serial = serial.Serial
except serial.serialutil.SerialException as err:
    print(err)
    print(f'Open constants.py and try another HAWKEYE_SERIAL_PORT, or connect'
        + ' a device to the serial port.')
    print('Continuing with dummy serial port.')
    class DummySerial:
        def __init__(self):
            pass
        def write(self, bytes):
            pass
        def readline(self):
            return b''
        def close(self):
            pass
    hawkeye_serial_instance = DummySerial()
    Serial = DummySerial
