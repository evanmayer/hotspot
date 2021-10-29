# This file houses the argument parsing, program setup, the state machine,
# calling the control algorithm, handling user/telemetry I/O, and load balancing
import constants as const
import executive as ex
import logging
import os
import vis

logging.basicConfig()

if __name__ == "__main__":
    visualizer = vis.Visualizer()
    print(vars(visualizer))
    executive = ex.Executive(os.path.join(const.TOPLEVEL_DIR, 'data', 'input', 'geometry', 'test_surface.csv'))
    executive.run(os.path.join(const.TOPLEVEL_DIR, 'data', 'input', 'profiles', 'box.csv'))
    