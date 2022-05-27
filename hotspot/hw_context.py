# This file abstracts the hardware module imports away and provides contingency
# modules so dev and testing can be done on a machine without hardware
# attached.

from adafruit_motor import stepper
from labjack import ljm
import serial
import sys


try:
    name = ljm.openS('T7', 'USB')
    eWriteAddress = ljm.eWriteAddress
    openS = ljm.openS
except ljm.LJMError as err:
    print(err)
    print('No LabJack modules found at import. Falling back to dummy class for simulation support.')
    def dummyEwriteAddress(handle, addr, mode, value):
        return
    def dummyOpenS(model: str, mode: str):
        return None
    # Overwrite
    eWriteAddress = dummyEwriteAddress
    openS = dummyOpenS


if sys.platform.startswith('win'):
    SERIAL_PORT = 'COM8'
else:
    SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUD = 9600
try:
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=.1)
    Serial = serial.Serial
except serial.serialutil.SerialException as err:
    print(err)
    print(f'Open {__file__} and try another SERIAL_PORT, or connect a device'
        + ' to the serial port.')
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
    ser = DummySerial()
    Serial = DummySerial
