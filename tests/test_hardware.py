from hw_context import MotorKit
import asyncio
import numpy as np
import pytest

from context import hardware as hw
from context import constants as const


# ------------------------------------------------------------------------------
# Fixtures
# ------------------------------------------------------------------------------
@pytest.fixture(scope='class')
def stepper():
    # Class scope cuts down on time spent init-ing
    # Used by any test function that needs a default motorkit instance
    motorkit = MotorKit(steppers_microsteps=const.MICROSTEP_NUM)
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

