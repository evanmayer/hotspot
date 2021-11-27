# This file houses the code to coordinate starting, running, and stopping
# the various processes/threads needed.

from hw_context import MotorKit
from hw_context import stepper
import algorithm as alg
import constants as const
import hardware as hw
import logging
import multiprocessing as mp
import numpy as np
import os
import sys
import telemetry as tm
import threading
import time


logger = logging.getLogger(__name__)

MODES = {'c': 'CAL_HOME', 'h': 'HOME', 's': 'SEQ', 'w': 'WAIT'}
HR = '-' * 80

class Executive(object):
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
    def __init__(self, geometry_file: str):
        # Set defaults
        self.mode = 'CAL_HOME'
        self.last_mode = 'CAL_HOME'
        self.kbd_queue = mp.Queue(1)
        self.cmd_queue = mp.Queue(const.MAX_QLEN)
        self.tm_queue = mp.Queue(const.MAX_QLEN)
        self.sequence_len = 0.

        # Handle telemetry output and display
        self.router = tm.DataRouter(self.tm_queue)

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
        # kit0 = MotorKit(address=const.HAT_0_ADDR, pwm_frequency=const.PWM_FREQ)
        # kit1 = MotorKit(address=const.HAT_1_ADDR, pwm_frequency=const.PWM_FREQ)
        self.steppers = {
            'sw': kit0.stepper1,
            'ne': kit1.stepper2,
            'nw': kit1.stepper1,
            'se': kit0.stepper2
        }

        # tension steppers
        # for _ in range(1):
            # [stepper_n.onestep(style=stepper.DOUBLE, direction=stepper.BACKWARD) for stepper_n in self.steppers.values()]
        self.cumulative_steps = np.array([0.] * len(self.steppers))
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
            cmd['flasher_mode'] = rows[i][0] # TODO: ECM: NotImplemented
            cmd['flasher_cmd']  = rows[i][1] # TODO: ECM: NotImplemented
            cmd['move_mode']    = rows[i][2]
            cmd['pos_cmd']      = (rows[i][3], rows[i][4])
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
            cmd['flasher_mode'] = None # TODO: ECM: NotImplemented
            cmd['flasher_cmd']  = None # TODO: ECM: NotImplemented
            cmd['move_mode']    = 'move'
            cmd['pos_cmd']      = self.robot.home
            self.do_motor_tasks(cmd)
            self.router.process_tm()
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
            - flasher_modes: TODO: not sure how LabJack wants command info
            - flasher_cmds: TODO:  not sure how LabJack wants command info
            - move_modes: if 'move', issue motor commands to achieve move
                spec'd by pos_cmds and speed_cmds
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
        self.router.process_tm()

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
        if cmd['move_mode'] == 'move':
            logger.debug(f'Move cmd: {cmd}')
            motor_cmds = self.robot.process_input(cmd['pos_cmd'])
            angs = [cmd for cmd in motor_cmds.values()]
            steps_taken = hw.all_steppers(self.steppers.values(), angs)

            self.cumulative_steps += np.array(steps_taken)
            logger.info(f'Cumulative steps:{self.cumulative_steps}')

            packet = {'hardware':
                {
                    'Time UTC (s)': time.time(),
                    'Motor Steps Taken' : steps_taken
                }
            }
            self.tm_queue.put(packet)


    def do_labjack_tasks(self, cmd: dict):
        # time.sleep(.5)
        return []


    def close(self):
        [self.steppers[key].release() for key in self.steppers.keys()]
        return 