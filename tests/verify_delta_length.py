import numpy as np

from context import hardware as hw
from context import constants as const
from context import hw_context


kit0 = hw_context.MotorKit(address=const.HAT_0_ADDR, steppers_microsteps=const.MICROSTEP_NUM, pwm_frequency=const.PWM_FREQ)
steppers = {
    # 'sw': kit1.stepper2,
    # 'ne': kit0.stepper1,
    'nw': kit0.stepper2,
    # 'se': kit1.stepper1
}

spool_angles = np.array([6]) * 2. * np.pi # revolutions to rads
# deploy some string
delta_lengths = np.array([.05])
if abs(const.RADIUS_M_PER_RAD) < 1e-9:
    delta_angles = delta_lengths / const.PULLEY_RADIUS
else:
    # requires solution of quadratic eqn., so some combos are disallowed
    for s in delta_lengths.flatten():
        radicand = 2. * const.RADIUS_M_PER_RAD * s + const.PULLEY_RADIUS ** 2.
        assert (radicand > 0.), f"Angle calculation will fail, disallowed radicand {radicand}"
    print(f'Uncorrected motor commands: {delta_lengths / const.PULLEY_RADIUS}')
    # Constraint that dtheta and dl have the same sign imposes choice of
    # the positive quadratic solution
    final_angles = (
        - const.PULLEY_RADIUS
        + np.sqrt(
            2. * const.RADIUS_M_PER_RAD * const.PULLEY_RADIUS * spool_angles
            + const.RADIUS_M_PER_RAD * (const.RADIUS_M_PER_RAD * spool_angles ** 2. + 2. * delta_lengths)
            + const.PULLEY_RADIUS ** 2.)
    ) / const.RADIUS_M_PER_RAD
    delta_angles = final_angles - spool_angles
    print(f'Corrected motor commands: {delta_angles}')

order, steps_taken, err = hw.all_steppers(list(steppers.values()), delta_angles)
print(np.array(steps_taken)[order])

[steppers[key].release() for key in steppers.keys()]
