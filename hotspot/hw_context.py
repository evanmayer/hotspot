# This file abstracts the hardware module imports away and provides contingency
# modules so dev and testing can be done on a non-rpi machine without hardware
# attached.

from adafruit_motor import stepper
from labjack import ljm

try:
    from adafruit_motorkit import MotorKit
except NotImplementedError as err:
    print(err)
    print('No Adafruit MotorKit module compatibility found at import. Falling back to dummy class for simulation support.')
    class DummyStepperMotor:
        def __init__(self):
            pass
        def onestep(self, direction=1, style=1):
            pass
        def release(self):
            pass
    # Dummy motorkit
    class MotorKit:
        def __init__(self, address=0x0, steppers_microsteps=1, pwm_frequency=1600):
            self.address = address
            self.steppers_microsteps = steppers_microsteps
            self.pwm_frequency = pwm_frequency
            self.stepper1 = DummyStepperMotor()
            self.stepper2 = DummyStepperMotor()

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
