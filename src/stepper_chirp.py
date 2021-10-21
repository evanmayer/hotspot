# This file defines the interface used to command the hardware. It also provides
# "dummy" versions of the interfaces for use in test environments without the
# hardware.

from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import constants as const
import matplotlib.pyplot as plt
import numpy as np
import time


micro_mod = 2
kit = MotorKit(steppers_microsteps=micro_mod)

deg_per_step = 360. / 200. / micro_mod
deg_tot = deg_per_step * 100.
steps = 0
deg_per_sec = 0.
domega = 5. # deg per sec
direction = stepper.FORWARD
style = stepper.MICROSTEP

deg_sec_cmd = []
deg_sec_est = []
try:
    start = time.time()
    while deg_per_sec < 100.:
        kit.stepper1.onestep(style=style, direction=direction)
        if not steps % (deg_tot / deg_per_step):
            end = time.time()
            deg_sec_cmd.append(deg_per_sec)
            deg_sec_est.append(deg_tot / (end - start))
            print(deg_sec_est)
            if direction == stepper.FORWARD:
                direction = stepper.BACKWARD
            else:
                direction = stepper.FORWARD
            deg_per_sec += domega
            steps = 0
            start = time.time()
        steps += 1
        time.sleep(1./(deg_per_sec / deg_per_step))
except KeyboardInterrupt as e:
    print('\nkbd interrupt caught, releasing steppers\n')
finally:
    kit.stepper1.release()
    

ax = plt.axes()
ax.plot(deg_sec_cmd[1:], deg_sec_est[1:], label='Meas.')
ax.plot(deg_sec_cmd[1:], deg_sec_cmd[1:], label='Est.')
ax.set_xlabel('Commanded Angular Speed (deg/sec)')
ax.set_ylabel('Estimated Angular Speed (deg/sec)')
ax.set_title('Speed Control')
ax.legend(loc='best')
ax.grid(True)
plt.show()

np.savetxt('../doc/cmd_vs_est.csv', np.array([deg_sec_cmd[1:], deg_sec_est[1:]]).T)
