# This file abstracts the hardware module imports away and provides contingency
# modules so dev and testing can be done on a machine without hardware
# attached.

from labjack import ljm
import serial

import hotspot.constants as const


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

try:
    serial_instance = serial.Serial(const.SERIAL_PORT, const.SERIAL_BAUD, timeout=const.SERIAL_TIMEOUT)
    Serial = serial.Serial
except serial.serialutil.SerialException as err:
    print(err)
    print(f'Open constants.py and try another SERIAL_PORT, or connect a device'
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
    serial_instance = DummySerial()
    Serial = DummySerial
