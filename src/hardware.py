# This file defines the interface used to command the hardware. It also provides
# "dummy" versions of the interfaces for use in test environments without the
# hardware.

from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import time

kit = MotorKit()

steps = 0
direction = stepper.FORWARD
style = stepper.INTERLEAVE

try:
    # make line taut: some microsteps away from each other
    for i in range(200):
        kit.stepper1.onestep(style=stepper.MICROSTEP, direction=stepper.FORWARD)
        kit.stepper2.onestep(style=stepper.MICROSTEP, direction=stepper.BACKWARD)
        time.sleep(.01)
    
    while True:
        kit.stepper1.onestep(style=style, direction=direction)
        kit.stepper2.onestep(style=style, direction=direction)
        if not steps % 200:
            steps = 0
            if direction == stepper.FORWARD:
                direction = stepper.BACKWARD
            else:
                direction = stepper.FORWARD
        steps += 1
        #time.sleep(0.001)
except KeyboardInterrupt as e:
    kit.stepper1.release()
    kit.stepper2.release()
    print('\nkbd interrupt caught, releasing steppers\n')
    raise e
    
