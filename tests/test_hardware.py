from adafruit_motorkit import MotorKit
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
    @pytest.mark.parametrize('radians, rad_per_sec', 
        [( np.pi / 4., 0.5),
         (-np.pi / 4., 0.5),
         ( 1e-9,       0.5),
         ( 0.,         0.5),
         ( np.pi,      2.0),
         (-np.pi,      2.0),
        ]
    )
    def test_move_motor_good(self, stepper, radians, rad_per_sec): 
        try:
            hw.move_motor(stepper, radians, rad_per_sec)
        finally:
            stepper.release()

    # Cases that are expected to throw an error
    @pytest.mark.parametrize('radians, rad_per_sec', 
        [( np.pi/4.,   0.),
         (-np.pi / 4., np.inf),
        ]
    )
    def test_move_motor_bad(self, stepper, radians, rad_per_sec): 
        try:
            with pytest.raises(ValueError):
                hw.move_motor(stepper, radians, rad_per_sec) 
        finally:
            stepper.release()