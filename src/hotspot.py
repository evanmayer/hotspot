# This file houses the argument parsing, program setup, the state machine,
# calling the control algorithm, handling user/telemetry I/O, and load balancing
import constants as const
import executive as ex
import logging
import multiprocessing as mp
import os

logging.basicConfig()

if __name__ == "__main__":
    geometry_file = os.path.join(const.TOPLEVEL_DIR, 'data', 'input', 'geometry', 'lab_bench.csv')
    command_profile = os.path.join(const.TOPLEVEL_DIR, 'data', 'input', 'profiles', 'raster.csv')

    # Start up executive
    executive = ex.Executive(geometry_file)
    executive.run(command_profile)
    