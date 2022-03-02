import asyncio
import numpy as np
import pytest

from context import hardware as hw
from context import constants as const
from context import hw_context


# ------------------------------------------------------------------------------
# Fixtures
# ------------------------------------------------------------------------------
@pytest.fixture(scope='class')
def stepper():
    # Class scope cuts down on time spent init-ing
    # Used by any test function that needs a default motorkit instance
    motorkit = hw_context.MotorKit(steppers_microsteps=const.MICROSTEP_NUM)
    motorkit.stepper1.release()
    yield motorkit.stepper1


# -----------------------------------------------------------------------------
# Default test class: reuses one default-init motorkit instance
# -----------------------------------------------------------------------------
@ pytest.mark.usefixtures('stepper')
class TestDefault(object):
    # Cases that are expected to pass
    @pytest.mark.parametrize('radians', 
        [( np.pi / 4.,),
         (-np.pi / 4.,),
         ( 1e-9,      ),
         ( 0.,        ),
         ( np.pi,     ),
         (-np.pi,     ),
        ]
    )
    def test_move_motor_good(self, stepper, radians): 
        try:
            hw.all_steppers([stepper], radians)
        finally:
            stepper.release()

if __name__ == '__main__':
    # wiggle steppers to ID them
    kit0 = hw_context.MotorKit(address=const.HAT_0_ADDR, steppers_microsteps=const.MICROSTEP_NUM)
    kit1 = hw_context.MotorKit(address=const.HAT_1_ADDR, steppers_microsteps=const.MICROSTEP_NUM)
    steppers = [kit0.stepper1, kit0.stepper2, kit1.stepper1, kit1.stepper2]
    [stepper.release() for stepper in steppers]
    [hw.all_steppers([stepper], (np.pi / 2.,)) for stepper in steppers]
    [hw.all_steppers([stepper], (-np.pi / 2.,)) for stepper in steppers]
    [stepper.release() for stepper in steppers]