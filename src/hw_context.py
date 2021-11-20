# This file abstracts the hardware module imports away and provides contingency
# modules so dev and testing can be done on a non-rpi machine without hardware
# attached.

from adafruit_motor import stepper

try:
    from adafruit_motorkit import MotorKit
except NotImplementedError:
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

