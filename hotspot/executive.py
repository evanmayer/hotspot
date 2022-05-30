# This file houses the code to process inputs, send commands to motors and IR
# sources, and log/display telemetry

import logging
import multiprocessing as mp
import numpy as np
import sys
import threading
import time

from hotspot.hw_context import ser
import hotspot.algorithm as alg
import hotspot.constants as const
import hotspot.hardware as hw
import hotspot.telemetry as tm


logger = logging.getLogger(__name__)

MODES = {'c': 'CAL_HOME', 'h': 'HOME', 's': 'SEQ', 'w': 'WAIT', 'b': 'BLINK'}
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

        # Talk to EZSteppers over RS485.
        self.ser = ser
        # This mapping should match the physical setup. Match the corner eyelet
        # location to the address selector pot on the EZStepper driver board.
        self.steppers = {
            'sw': 1,
            'ne': 2,
            'nw': 3,
            'se': 4
        }
        for address in self.steppers.keys():
            # set stepping and hold current limits: 50% = 1 A
            resp = hw.ezstepper_write(self.ser, f'/{self.steppers[address]}m50R\r\n')
            resp = hw.ezstepper_write(self.ser, f'/{self.steppers[address]}h50R\r\n')
            # encoder ratio: 1000 * (usteps / rev) / (encoder ticks / rev)
            # 1280 = 1000 * (256 * 200) / 40000
            resp = hw.ezstepper_write(self.ser, f'/{self.steppers[address]}aE1280R\r\n')
            # zero out positions
            resp = hw.ezstepper_write(self.ser, f'/{self.steppers[address]}z0R\r\n')
            resp = hw.ezstepper_write(self.ser, f'/{self.steppers[address]}A0R\r\n')
            resp = hw.ezstepper_write(self.ser, f'/{self.steppers[address]}z0R\r\n')
            # enable encoder feedback mode
            resp = hw.ezstepper_write(self.ser, f'/{self.steppers[address]}n8R\r\n')

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
                    elif 'b' == kbd_in:
                        logger.info('Manual blink mode requested.')
                        self.mode = 'BLINK'
                    else:
                        continue

                if self.mode == 'CAL_HOME':
                    self.cal_home()
                    self.mode = 'WAIT'
                    print(MENU_STR)
                elif self.mode == 'INPUT_HOME':
                    self.cal_home_manual()
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
                elif self.mode == 'BLINK':
                    self.blink()
                    self.mode = 'WAIT'
                    print(MENU_STR)
                else:
                    pass
                self.last_mode = self.mode

            except KeyboardInterrupt:
                print('\nCaught KeyboardInterrupt, shutting down.')
                running = False
        self.close()
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


    def cal_home_manual(self):
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


    def cal_home(self):
        '''
        Moves motors to limits to calibrate the stepper positions.

        In turn, move each motor to its retracted limit and record the encoder position.

        Assumptions:
        - All cables are slack, but without excessive cable played out.
        - Hawkeye signaling wires protrude from the S side of the raft,
            preventing us from choosing SW or SE as limit finding locations.
        - Corners form a rectangle.
        - Stepper torque is sufficient to hold the raft once at NW while SW,
            SE, NE retract.
        '''
        # Worst case, about how far would we have to move before hitting an 
        # opposite corner?
        # The total available cable length should be sized so that the raft
        # can just reach all corners of the workspace at the maximum eyelet
        # separation of 0.6177 m (accommodates ~25.5" mirror K2).
        # In position correction mode, the EZStepper will attempt this move a 
        # set number of times before giving up, which is what we want upon
        # hitting a hard stop.
        max_length = np.linalg.norm(
            [
                self.robot.surf.sw + np.array([0., 0.715]), # meas. fully extended cable
                self.robot.surf.se
            ]
        )
        max_radians = max_length / const.PULLEY_RADIUS
        max_ticks = np.round(np.abs(max_radians) * const.DEG_PER_RAD / const.DEG_PER_STEP / const.STEP_PER_TICK).astype(int)
        t_move = max(max_ticks / const.MAX_SPEED_TICKS, 1e-9)
        num_ticks = max(1, max_ticks)

        # Home each axis
        axes = ['ne', 'se', 'sw', 'nw']
        for current_axis in axes:
            logger.info(f'Homing to {current_axis}')
            # turn off current to axes not being homed
            resp = hw.ezstepper_write(self.ser, f'/_m1R\r\n')
            resp = hw.ezstepper_write(self.ser, f'/_h1R\r\n')
            resp = hw.ezstepper_write(self.ser, f'/{self.steppers[current_axis]}m50R\r\n')
            resp = hw.ezstepper_write(self.ser, f'/{self.steppers[current_axis]}h50R\r\n')
            # set speed for moving axis
            resp = hw.ezstepper_write(self.ser, f'/{self.steppers[current_axis]}V{const.MAX_SPEED_TICKS}R\r\n')
            # home axis, negative direction (D) retracts
            resp = hw.ezstepper_write(self.ser, f'/{self.steppers[current_axis]}D{num_ticks}R\r\n')
            logger.debug(f'Sleeping {t_move:.2f} sec for move')
            time.sleep(t_move + 1e-1)
            logger.debug(f'Post-homing move: {hw.get_encoder_pos(self.ser, self.steppers[current_axis])}')
            # zero out encoder position
            resp = hw.ezstepper_write(self.ser, f'/{self.steppers[current_axis]}z0R\r\n')
        # Restore proper current limits to all drivers
        resp = hw.ezstepper_write(self.ser, f'/_m50R\r\n')
        resp = hw.ezstepper_write(self.ser, f'/_h50R\r\n')

        # Adjust home position
        pos = self.robot.surf.nw + np.array((const.HOMING_OFFSET_X, const.HOMING_OFFSET_Y))
        # Lie. Take up slack by spoofing a slight offset to force 
        # recalculation/commanding of positions.
        self.robot.raft.position = pos + np.array((0.001, -0.001))
        self.robot.home = pos
        self.go_home()

        logger.info(f'Raft is homed with centroid position {self.robot.raft.position}')
        logger.warning('Verify that the raft has been driven to one of its limits and all cables are taut. If not, request CAL_HOME again.')

        packet = {'algorithm':
            {
                'Local Time (s)': time.time(),
                'Position Command (m)' : pos,
                'Motor Angle Command (rad)' : np.array([0.] * len(self.steppers.keys())),
            }
        }
        self.tm_queue.put(packet)
        self.router.process_tm(plot_enable=self.plot_enable)
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
        # Do motor tasks, then LJ tasks, so IR source tasks happen at the end of each move.
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


    def blink(self):
        cmd = {}
        cmd['flasher_cmds'] = 4 * [1] + 8 * [0]
        self.do_labjack_tasks(cmd)
        self.router.process_tm(plot_enable=self.plot_enable)
        logger.info('Blink complete.')
        return


    def do_motor_tasks(self, cmd: dict):
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
            ticks_to_go, err = hw.all_steppers_ez(self.ser, [self.steppers[key] for key in keys], angs)

        # HACK: Linear approximation only holds for small distances, so
        # chunk up big moves into tiny bits. ECM: This should really probably
        # happen inside the control algorithm itself, or even better, ditch
        # the linear approximation and do the math to find out how each motor
        # should move at each point in the move.
        MAX_DIST = .015
        pos_before = self.robot.raft.position
        pos_after = cmd['pos_cmd']
        dist_to_go = np.linalg.norm(np.array(pos_after) - np.array(pos_before))
        while dist_to_go > MAX_DIST:
            logger.debug(f'Dist. to go in this move: {dist_to_go}')
            # determine a position MAX_DIST away from the starting pos along
            # the line of travel
            v = np.array(pos_after) - np.array(pos_before)
            u = v / np.linalg.norm(v) # the unit vector pointing along the line
            logger.debug(f'Direction vector: {v}, Unit vector: {u}')
            pos_cmd = pos_before + MAX_DIST * u
            logger.debug(f'Intermediate move: {pos_cmd}')
            send_pos_cmd(pos_cmd) # alg updates raft position
            # calculate new dist to go
            pos_before = self.robot.raft.position
            dist_to_go = np.linalg.norm(np.array(pos_after) - np.array(pos_before))
        logger.debug(f'Final move: {pos_after}')
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
        # Terminate all running commands
        hw.ezstepper_write(self.ser, f'/_T\r\n')
        # Release torque
        hw.ezstepper_write(self.ser, f'/_m0R\r\n')
        hw.ezstepper_write(self.ser, f'/_h0R\r\n')
        # Zero out position
        resp = hw.ezstepper_write(self.ser, f'/_z0R\r\n')

        self.ser.close()
        return
