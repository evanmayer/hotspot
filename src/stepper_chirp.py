# This file defines the interface used to command the hardware. It also provides
# "dummy" versions of the interfaces for use in test environments without the
# hardware.

from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import constants as const
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import time
import hardware as hw


kit = MotorKit(steppers_microsteps=const.MICROSTEP_NUM)

kit.stepper1.release()

deg_tot = const.DEG_PER_STEP * 50.
steps = 0
deg_per_sec = 0.
domega = 5. # deg per sec
direction = stepper.FORWARD
style = const.STEPPER_STYLE

deg_sec_cmd = []
deg_sec_est = []
try:
    start = time.time()
    while deg_per_sec < 110.:
        # fudge more accurate steps/sec by subtracting off execution time of non-sleeping part of loop.
        loop_start = time.time()
        kit.stepper1.onestep(style=style, direction=direction)
        if not steps % (deg_tot / const.DEG_PER_STEP):
            end = time.time() # completion time of a set of steps at an angular rate
            deg_sec_cmd.append(deg_per_sec)
            deg_sec_est.append(deg_tot / (end - start)) # estimate average steps done per second
            deg_per_sec += domega # done with this angular rate, so increase to test next angular rate
            steps = 0
            start = time.time() # begin measuring time again
        steps += 1
        loop_end = time.time()
        delta_time_loop = loop_end - loop_start
        # sleep for the remaining time to keep issuing steps at proper rate.
        time_rem = 1./(deg_per_sec / const.DEG_PER_STEP) - delta_time_loop
        if time_rem <= 0.:
            time_rem = 1e-31
        time.sleep(time_rem)
except KeyboardInterrupt as e:
    print('\nkbd interrupt caught, releasing steppers\n')
finally:
    kit.stepper1.release()
    
cmd = np.array(deg_sec_cmd[1:])
est = np.array(deg_sec_est[1:])

ax = plt.axes()
ax.plot(cmd, 100. * (cmd - est) / cmd, label='% Error: 100 * (Cmd. - Est.) / Cmd.')
ax.set_xlabel('Commanded Angular Speed (deg/sec)')
ax.set_ylabel('Angular Speed Relative Error (%)')
ax.set_title('Speed Control')
ax.legend(loc='best')
ax.grid(True)
plt.savefig('../doc/cmd_vs_est.png')

np.savetxt('../doc/cmd_vs_est.csv', np.array([deg_sec_cmd[1:], deg_sec_est[1:]]).T)
