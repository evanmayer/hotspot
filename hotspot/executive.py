# This file houses the code to coordinate starting, running, and stopping
# the various processes/threads needed.

import logging
import multiprocessing as mp
import numpy as np
import os
import serial
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
        self.mode = 'CAL_HOME'
        self.last_mode = 'CAL_HOME'
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
        # should be ~0 for closed loop shapes
        self.cumulative_steps = np.array([0.] * len(self.steppers))

        self.lj_instance = hw.try_open(hw.MODEL_NAME, hw.MODE)
        hw.spawn_all_threads_off(self.lj_instance)

        #self.ser = serial.Serial(const.SERIAL_PORT, const.SERIAL_BAUD)
        # time.sleep(2)
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
        # First, require that a home position be set. This also tells the 
        # robot where the raft starts at.
        self.cal_home()

        # Allow user input to change the mode.
        input_thread = threading.Thread(target=self._get_kbd,
            args=(self.kbd_queue,),
            daemon=True
        )
        input_thread.start()
        print(HR)
        print(f'Listening for user input for mode changes. Type a mode char and press enter:\n{MODES}')
        print(HR)

        # Start off in STARE mode to await user input.
        self.mode = 'STARE'
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
                    self.cal_home()
                    self.mode = 'WAIT'
                elif self.mode == 'HOME':
                    self.home()
                    self.mode = 'WAIT'
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
        - We do not know where the raft starts out, so we must move the maximum
            possible number of steps to get to NW, given the surface geometry.
        - Corners form a rectangle.
        - Stepper torque is sufficient to hold the raft once at NW while SW,
            SE, NE retract.
        '''
        # All motors must be released before starting, to NE, SE, SW don't
        # inhibit motion
        [self.steppers[key].release() for key in self.steppers.keys()]

        # Worst case, about how far would we have to move before hitting NW?
        max_distance = self.robot.surf.nw - self.robot.surf.se
        max_radians = max_distance / const.PULLEY_RADIUS
        max_steps = np.round(np.abs(max_radians) * const.DEG_PER_RAD / const.DEG_PER_STEP).astype(int)

        logger.info('Homing to NW')
        i = max_steps
        while i > 0:
            self.steppers['nw'].onestep(style=stepper.MICROSTEP, direction=stepper.BACKWARD)
            time.sleep(1e-4)
            i -= 1

        self.robot.raft.position = self.robot.surf.nw + np.array((const.HOMING_OFFSET_X, HOMING_OFFSET_Y))
        logger.info(f'Homed raft centroid is: {self.robot.raft.position}')

        # With a little more work, the number of steps here could be reduced,
        # but doing the same # of steps as before should always work.
        logger.info('Retracting cables to home NE, SE, SW')
        i = max_steps
        while i > 0:
            self.steppers['ne'].onestep(style=stepper.MICROSTEP, direction=stepper.BACKWARD)
            self.steppers['se'].onestep(style=stepper.MICROSTEP, direction=stepper.BACKWARD)
            self.steppers['sw'].onestep(style=stepper.MICROSTEP, direction=stepper.BACKWARD)
            time.sleep(1e-4)
            i -= 1
        return


    def home(self):
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
            - flasher_cmds: TODO:  not sure how LabJack wants command info
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
            return
        else:
            cmd = self.cmd_queue.get()

        progress = 100. * (1 + self.sequence_len - num_remaining) / self.sequence_len
        # Do motor tasks, then LJ tasks in serial, so IR source tasks happen at the end of each move.
        self.do_motor_tasks(cmd)
        self.do_labjack_tasks(cmd)
        logger.info(f'Command completed. Sequence progress: {progress:.2f} %')
        # take time to log TM and update display before doing next cmd
        self.router.process_tm(plot_enable=self.plot_enable)
        return


    def wait(self):
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
        logger.debug(f'Move cmd: {cmd}')
        motor_cmds = self.robot.process_input(cmd['pos_cmd'])
        # bootleg OrderedDict
        angs = [cmd for cmd in [motor_cmds[key] for key in ['sw', 'nw', 'ne', 'se']]]
        steps_taken = hw.all_steppers([self.steppers[key] for key in ['sw', 'nw', 'ne', 'se']], angs)
        #steps_taken = hw.all_steppers_serial(self.ser, angs)

        self.cumulative_steps += np.array(steps_taken)
        logger.info(f'Cumulative steps:{self.cumulative_steps}')

        packet = {'Motor Cmd':
            {
                'Time UTC (s)': time.time(),
                'Motor Steps Taken' : steps_taken
            }
        }
        self.tm_queue.put(packet)
        return


    def do_labjack_tasks(self, cmd: dict):
        freq = 10. # Hz
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
        packet = {'LabJack Cmd':
            {
                'Time UTC (s)': time.time(),
                'Addresses Turned On' : 1 + np.where(np.array(cmd['flasher_cmds']) > 0)[0],
            }
        }
        self.tm_queue.put(packet)
        return


    def close(self):
        [self.steppers[key].release() for key in self.steppers.keys()]
        #self.ser.close()
        return
