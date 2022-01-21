# This file houses the code to process inputs, send commands to motors and IR
# sources, and log/display telemetry

import logging
import multiprocessing as mp
import numpy as np
import os
import sys
import threading
import time

from hotspot.hw_context import MotorKit
from hotspot.hw_context import stepper
import hotspot.algorithm as alg
import hotspot.constants as const
import hotspot.hardware as hw
import hotspot.telemetry as tm


logger = logging.getLogger(__name__)

MODES = {'c': 'CAL_HOME', 'h': 'HOME', 's': 'SEQ', 'w': 'WAIT'}
HR = '-' * 80
MENU_STR = HR + f'\nListening for user input for mode changes. Type a mode char and press enter:\n{MODES}\n' + HR


class Executive:
    '''
    Handles control flow with a state machine, initializes the algorithm's
    Robot class, ingests command files to process and dispatch tasks to
    hardware, and outputs telemetry.

    Parameters
    ----------
    geometry_file
        Text file with 10 cols specifying the geometry of the robot for a given
        surface. This file defines 4 pairs of points describing cable endpoints
        where they are attached to the mirror, in the mirror coordinate frame,
        and the rectangular geometry of the central effector as a width and
        height.
    '''
    def __init__(self, geometry_file: str, plot_enable=False):
        logger.debug('Executive init')
        # Set defaults
        self.mode = 'WAIT'
        self.last_mode = 'WAIT'
        self.kbd_queue = mp.Queue(1)
        self.cmd_queue = mp.Queue(const.MAX_QLEN)
        self.tm_queue = mp.Queue(const.MAX_QLEN)
        self.sequence_len = 0.

        # Handle telemetry output
        self.router = tm.DataRouter(self.tm_queue)
        self.plot_enable = plot_enable

        # Read in positions of cable endpoints and raft dimensions
        (sw_0, sw_1,
         nw_0, nw_1,
         se_0, se_1,
         ne_0, ne_1,
         w, h) = np.loadtxt(
            geometry_file,
            dtype=float,
            delimiter=',',
            skiprows=1,
            )

        sw = (sw_0,sw_1)
        nw = (nw_0,nw_1)
        se = (se_0,se_1)
        ne = (ne_0,ne_1)
        surf = alg.TestSurface(sw=sw, se=se, nw=nw, ne=ne)
        # Initialize the raft to 0,0 until homed.
        raft = alg.Raft((0,0), w, h)
        self.robot = alg.Robot(surf, raft, self.tm_queue)

        if const.STEPPER_STYLE in [stepper.SINGLE, stepper.DOUBLE]:
            assert const.MICROSTEP_NUM == 1, 'const.MICROSTEP_NUM multiplier must be 1 for single- or double-stepping mode.'
            kit0 = MotorKit(address=const.HAT_0_ADDR, pwm_frequency=const.PWM_FREQ)
            kit1 = MotorKit(address=const.HAT_1_ADDR, pwm_frequency=const.PWM_FREQ)
        else:
            if const.STEPPER_STYLE == stepper.INTERLEAVE:
                assert const.MICROSTEP_NUM == 2, 'const.MICROSTEP_NUM multiplier must be 2 for interleaved stepping mode.'
            kit0 = MotorKit(address=const.HAT_0_ADDR, steppers_microsteps=const.MICROSTEP_NUM, pwm_frequency=const.PWM_FREQ)
            kit1 = MotorKit(address=const.HAT_1_ADDR, steppers_microsteps=const.MICROSTEP_NUM, pwm_frequency=const.PWM_FREQ)
        # This mapping should match the physical setup. kit0 is closest to the
        # parent board, kit1 is on top. stepper1 is terminals M1+M2, stepper2
        # is terminals M3+M4
        self.steppers = {
            'sw': kit1.stepper2,
            'ne': kit0.stepper1,
            'nw': kit0.stepper2,
            'se': kit1.stepper1
        }

        # Keep track of total steps taken post-homing. Closed loop paths
        # should maintain ~0 net steps.
        self.stepper_net_steps = np.array([0] * 4)
        # Keep track of rounding errors in steps to correct them as they
        # accumulate
        self.net_step_error = np.array([0.] * 4)

        self.lj_instance = hw.try_open(hw.MODEL_NAME, hw.MODE)
        hw.spawn_all_threads_off(self.lj_instance)

        return


    def _get_kbd(self, queue: mp.Queue):
        '''
        Helper function to run in a separate thread and add user input chars 
        to a buffer.

        Parameters
        ----------
        queue
            Threadsafe structure for handling user keyboard input for mode
            switching
        '''
        while True:
            queue.put(sys.stdin.read(1))


    def add_cmds(self, fname: str):
        '''
        Read command input file and add commands to the command queue.

        Parameters
        ----------
        fname
            .csv-formatted file containing a sequence of commands, one per col.
            Data spec described in docstring of sequence() function.
        '''
        logger.info(f'Parsing command sequence: {fname}')
        rows = np.genfromtxt(
            fname,
            dtype=None,
            delimiter=',',
            skip_header=1,
            encoding='utf8'
            )
        rows = np.atleast_1d(rows)
        self.sequence_len = len(rows)

        if (self.sequence_len > const.MAX_QLEN):
            logger.warn(f'Input command number {len(rows)} exceeds command'
                + ' queue length {const.MAX_QLEN}. Increase'
                + ' constants.MAX_QLEN.')

        for i in range(self.sequence_len):
            cmd = {}
            cmd['flasher_cmds'] = [int(item) for item in rows[i][0].split()]
            cmd['pos_cmd']      = (rows[i][1], rows[i][2])
            self.cmd_queue.put(cmd)
        return


    def empty_queue(self, queue: mp.Queue):
        '''
        Completely empty the given Queue object.

        Parameters
        ----------
        queue
            Any queue
        '''
        while not queue.empty():
            queue.get()
        return


    def run(self, fname: str):
        '''
        Main run function, including processing human input to switch between
        states.

        Parameters
        ----------
        fname
            .csv-formatted file containing a sequence of commands, one per col.
            Data spec described in docstring of sequence() function.
        '''
        # Allow user input to change the mode.
        input_thread = threading.Thread(
            target=self._get_kbd,
            args=(self.kbd_queue,),
            daemon=True
        )
        input_thread.start()
        print(MENU_STR)

        # Start off in WAIT mode to await user input.
        self.mode = 'WAIT'
        self.last_mode = self.mode
        running = True
        while running:
            try:
                # Check for user input
                if not self.kbd_queue.empty():
                    kbd_in = self.kbd_queue.get_nowait()
                    if 'c' == kbd_in:
                        logger.info('Home calibration requested.')
                        self.mode = 'CAL_HOME'
                    elif 'i' == kbd_in:
                        logger.info('Home position override requested.')
                        self.mode = 'INPUT_HOME'
                    elif 'h' == kbd_in:
                        logger.info('Moving to home requested.')
                        self.mode = 'HOME'
                    elif 's' == kbd_in:
                        logger.info('Sequence run requested.')
                        self.mode = 'SEQ'
                    elif 'w' == kbd_in:
                        logger.info('Wait mode requested.')
                        self.mode = 'WAIT'
                    else:
                        continue

                if self.mode == 'CAL_HOME':
                    self.cal_home_auto()
                    self.mode = 'WAIT'
                    print(MENU_STR)
                elif self.mode == 'INPUT_HOME':
                    self.cal_home()
                    self.mode = 'WAIT'
                    print(MENU_STR)
                elif self.mode == 'HOME':
                    self.go_home()
                    self.mode = 'WAIT'
                    print(MENU_STR)
                elif self.mode == 'SEQ':
                    self.sequence(fname)
                elif self.mode == 'WAIT':
                    self.wait()
                else:
                    pass
                self.last_mode = self.mode

            except KeyboardInterrupt:
                print('\nCaught KeyboardInterrupt, shutting down.')
                running = False
        self.close()
        return


    def cal_home(self):
        '''
        Ask the user to input the current position of the center of the raft to
        store it as the home position.
        '''
        print('Where is home? Input the current position, in meters, of'
            + ' the raft centroid relative to the mirror origin, then press'
            + ' ENTER to store it as home.')
        x = float(input('x-coord: '))
        y = float(input('y-coord: '))
        pos = (x, y)
        self.robot.home = pos
        self.robot.raft.position = pos
        logger.info(f'Home position set: {self.robot.home}')
        return


    def cal_home_auto(self):
        '''
        Moves motors to limits to calibrate the stepper positions.

        First moves to NW limit, then retracts cable from each other stepper
        until we can be sure all lines are taut. Updates the raft centroid
        position according to offsets in constants.py which depend on the
        (repeatable) position the raft ends up in when it reaches the NW limit.

        Assumptions:
        - All cables are slack, but without excessive cable played out.
        - Hawkeye signaling wires protrude from the S side of the raft,
            preventing us from choosing SW or SE as limit finding locations.
        - Corners form a rectangle.
        - Stepper torque is sufficient to hold the raft once at NW while SW,
            SE, NE retract.
        '''
        # All motors must be released before starting, to NE, SE, SW don't
        # inhibit motion
        [self.steppers[key].release() for key in self.steppers.keys()]

        # The total available cable length should be sized so that the raft
        # can just reach all corners of the workspace at the maximum eyelet
        # separation of 0.6177 m (accommodates ~25.5" mirror K2).
        max_length = np.linalg.norm(
            [
                self.robot.surf.sw + np.array([0., 0.6197]),
                self.robot.surf.se
            ]
        )
        # Worst case, about how far would we have to move before hitting NW?
        max_radians = max_length / const.PULLEY_RADIUS
        # valid for single or double steps
        max_steps = np.round(np.abs(max_radians) * const.DEG_PER_RAD / (360. / 200. / 1)).astype(int)
        num_steps = max(1, max_steps // 1)
        logger.info('Homing to NW')
        report_interval = 100
        i = num_steps
        while i > 0:
            self.steppers['nw'].onestep(style=stepper.DOUBLE, direction=stepper.BACKWARD)
            time.sleep(const.STEP_WAIT)
            i -= 1
            if not i % report_interval:
                progress = 100. * (num_steps - i) / num_steps
                logger.info(f'Progress: {progress:.2f} %')

        logger.info('Retracting cables to tension NE, SE, SW')
        i = num_steps
        while i > 0:
            self.steppers['ne'].onestep(style=stepper.MICROSTEP, direction=stepper.BACKWARD)
            time.sleep(const.STEP_WAIT)
            self.steppers['se'].onestep(style=stepper.MICROSTEP, direction=stepper.BACKWARD)
            time.sleep(const.STEP_WAIT)
            self.steppers['sw'].onestep(style=stepper.MICROSTEP, direction=stepper.BACKWARD)
            time.sleep(const.STEP_WAIT)
            i -= 1
            if not i % report_interval:
                progress = 100. * (num_steps - i) / num_steps
                logger.info(f'Progress: {progress:.2f} %')

        # Adjust home position
        pos = self.robot.surf.nw + np.array((const.HOMING_OFFSET_X, const.HOMING_OFFSET_Y))
        self.robot.raft.position = pos
        self.robot.home = pos
        # Calculate the amount of cable wound onto each spool when at home.
        lengths_home = np.linalg.norm(self.robot.raft.corners - self.robot.surf.corners, axis=-1)
        lengths_on_spool = (max_length * np.ones_like(lengths_home)) - lengths_home
        print(lengths_on_spool)
        # Calculate the initial angular positions for use in compensating for
        # the effects of spooling on cable
        if abs(const.RADIUS_M_PER_RAD) > np.finfo(float).eps:
            self.robot.spool_angles = (
                -const.PULLEY_RADIUS + np.sqrt(
                    2. * const.RADIUS_M_PER_RAD * lengths_on_spool + const.PULLEY_RADIUS ** 2.
                )
            ) / const.RADIUS_M_PER_RAD
        logger.debug(f'Starting spool angles: {self.robot.spool_angles}')

        logger.info(f'Raft is homed with centroid position {self.robot.raft.position}')
        logger.warning('Verify that the raft has been driven to one of its limits and all cables are taut. If not, request CAL_HOME again.')

        packet = {'algorithm':
            {
                'Local Time (s)': time.time(),
                'Position Command (m)' : pos,
                'Motor Delta Angle Command (rad)' : np.array([0.] * len(self.steppers.keys())),
                'Angle Correction Due to Spool Radius (rad)' : np.array([0.] * len(self.steppers.keys()))
            }
        }
        self.tm_queue.put(packet)
        self.router.process_tm(plot_enable=self.plot_enable)
        return


    def go_home(self):
        '''
        Clear all commands in the queue and drive to the home position.
        '''
        # If not already home, go there.
        eps = np.finfo(float).eps
        if not all(abs(a - b) < eps for a,b in zip(self.robot.raft.position, self.robot.home)):
            logger.info(f'Clearing command queue and commanding home: {self.robot.home}')
            self.empty_queue(self.cmd_queue)
            cmd = {}
            cmd['flasher_cmd'] = []
            cmd['pos_cmd']     = self.robot.home
            self.do_motor_tasks(cmd)
            self.router.process_tm(plot_enable=self.plot_enable)
            logger.info(f'Home.')
        else:
            logger.info('Already home, nothing to do.')
        return


    def sequence(self, fname: str):
        '''
        On each call, pop a new command off of the command queue and dispatch
        it to motors/LabJack.

        Parameters
        ----------
        fname
            .csv-formatted file containing a sequence of commands, one per col:
            - flasher_cmds: 1 or 0 depending on whether that address is enabled
            - pos_cmd_0s: 0th element of position command coordinate
            - pos_cmd_1s: 1st element of position command coordinate
        '''
        # If we are changing to sequence from another mode, ensure we start
        # fresh
        if self.mode != self.last_mode:
            logger.info('Beginning command sequence.')
            self.empty_queue(self.cmd_queue)
            self.add_cmds(fname)

        num_remaining = self.cmd_queue.qsize()

        if num_remaining < 1:
            self.mode = 'WAIT'
            return
        else:
            cmd = self.cmd_queue.get()

        progress = 100. * (1 + self.sequence_len - num_remaining) / self.sequence_len
        # Do motor tasks, then LJ tasks in serial, so IR source tasks happen at the end of each move.
        self.do_motor_tasks(cmd)
        self.do_labjack_tasks(cmd)
        logger.info(f'Raft centroid: {self.robot.raft.position}')
        logger.info(f'Command completed. Sequence progress: {progress:.2f} %')
        self.router.process_tm(plot_enable=self.plot_enable)
        return


    def wait(self):
        if self.plot_enable:
            self.router.run_gui_event_loop()
        return


    def do_motor_tasks(self, cmd: dict) -> list:
        '''
        Transform the move command into motor commands

        Parameters
        ----------
        cmd:
            Command packet dictionary with keys for position commands to pass
            to control algorithm
        '''
        def send_pos_cmd(cmd_tuple: tuple):
            '''
            Helper to send position commands to control algorithm and collect
            results
            '''
            motor_cmds = self.robot.process_input(cmd_tuple) 
            # bootleg OrderedDict
            keys = ['sw', 'nw', 'ne', 'se']
            angs = [cmd for cmd in [motor_cmds[key] for key in keys]]
            order, steps_taken, err = hw.all_steppers([self.steppers[key] for key in keys], angs)
            
            # Perform corrections for rounding errors
            self.stepper_net_steps += steps_taken
            self.net_step_error += err
            logger.debug(f'Net steps: {self.stepper_net_steps}')
            logger.debug(f'Net step error incurred by rounding: {self.net_step_error}')
            for i, step_err in enumerate(self.net_step_error):
                # If off by more than a step, fix it.
                if np.abs(step_err) >= 0.5:
                    steps_to_go = np.round(step_err).astype(int)
                    logger.debug(f'Correcting {steps_to_go} steps on {keys[i]}')
                    # correct in the opposite direction of error
                    self.stepper_net_steps[i] += -steps_to_go
                    self.net_step_error[i] += -steps_to_go
                    stepper_dir = stepper.BACKWARD
                    direction = np.sign(steps_to_go)
                    if direction == -1:
                        stepper_dir = stepper.FORWARD
                    for _ in range(abs(steps_to_go)):
                        self.steppers[keys[i]].onestep(style=const.STEPPER_STYLE, direction=stepper_dir)
                        time.sleep(const.STEP_WAIT)

            # HACK FIXME truly despicable: force all motors to skip after each move to equalize tension.
            n_tens = 10
            for _ in range(n_tens):
                self.steppers[keys[i]].onestep(style=const.STEPPER_STYLE, direction=stepper.BACKWARD)
            
        logger.debug(f'Move cmd: {cmd}')
        pos_after = cmd['pos_cmd']
        send_pos_cmd(pos_after)

        return


    def do_labjack_tasks(self, cmd: dict):
        packet = {'LabJack Cmd':
            {
                'Local Time (s)': time.time(),
                'Addresses Turned On' : np.where(np.array(cmd['flasher_cmds']) > 0)[0],
            }
        }
        self.tm_queue.put(packet)
        
        freq = 10. # switching freq, Hz, i.e. flashing freq is 1/2 this
        num_blinks = 10
        flipflop = 0
        while num_blinks > 0:
            start_time = time.time()
            if flipflop:
                hw.spawn_all_threads(self.lj_instance, cmd['flasher_cmds'])
                num_blinks -= 1
            else:
                hw.spawn_all_threads_off(self.lj_instance)
            flipflop = 1 - flipflop
            # sleep off remaining time to fudge actions at frequency freq
            time.sleep((1. / freq) - (time.time()-start_time))
        hw.spawn_all_threads_off(self.lj_instance)
        
        return


    def close(self):
        [self.steppers[key].release() for key in self.steppers.keys()]
        return
