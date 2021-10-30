# This file houses the argument parsing, program setup, the state machine,
# calling the control algorithm, handling user/telemetry I/O, and load balancing
import constants as const
import executive as ex
import logging
import multiprocessing as mp
import os
import telemetry as tm
import threading

logging.basicConfig()

if __name__ == "__main__":
    geometry_file = os.path.join(const.TOPLEVEL_DIR, 'data', 'input', 'geometry', 'test_surface.csv')
    command_profile = os.path.join(const.TOPLEVEL_DIR, 'data', 'input', 'profiles', 'box.csv')

    # Being awaiting telemetry
    tm_queue = mp.Queue(const.MAX_QLEN)
    router = tm.DataRouter(tm_queue)
    thread = threading.Thread(target=router.process_tm, daemon=True)
    thread.start()

    # Start up executive
    executive = ex.Executive(geometry_file, tm_queue)
    executive.run(command_profile)
    