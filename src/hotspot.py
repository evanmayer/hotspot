# This file houses the argument parsing, program setup, the state machine,
# calling the control algorithm, handling user/telemetry I/O, and load balancing
import executive as ex
import constants as const
import logging
import os

logging.basicConfig()

if __name__ == "__main__":
    executive = ex.Executive('geometry.txt')
    executive.run(os.path.join(const.TOPLEVEL_DIR, 'data', 'input', 'box.csv'))
