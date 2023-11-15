# This file houses the code to process inputs, send commands to motors and IR
# sources, and log/display telemetry

import logging
import multiprocessing as mp
import numpy as np
import os
import subprocess
import sys
import threading
import time

from hotspot.hw_context import hawkeye_serial_instance, stepper_serial_instance, StepperSerial
import hotspot.algorithm as alg
import hotspot.constants as const
import hotspot.hardware as hw
import hotspot.telemetry as tm


logger = logging.getLogger(__name__)

MODES = {'c': 'CAL_HOME', 'h': 'HOME', 'r': 'RUN', 's': 'SEL', 'w': 'WAIT', 'b': 'BLINK'}
MODES_VERBOSE = {
    'c': 'Calibrate axes',
    'h': 'Home goto',
    'r': 'Run profile',
    's': 'Select profile',
    'w': 'Wait',
    'b': 'Blink all Hawkeyes'
}
HR = '-' * 80
MENU_STR = (
    HR +
    f'\nListening for mode changes. Type a mode char and press enter:\n' +
    f'{MODES_VERBOSE}\n' +
    HR
)

STEPPER_ORDER = ['sw', 'nw', 'ne', 'se'] # just for convenience


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
        self.positions_visited = 0

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

        # Talk to Hawkeyes over USB.
        self.hawkeye_ser = hawkeye_serial_instance
        # Talk to EZSteppers over RS485.
        self.stepper_ser = stepper_serial_instance
        # This mapping should match the physical setup. Match the corner eyelet
        # location to the address selector pot on the EZStepper driver board.
        self.steppers = {
            'nw': 2,
            'ne': 1,
            'se': 4,
            'sw': 3
        }

        # terminate all running commands
        resp = hw.ezstepper_write(self.stepper_ser, '_', 'T\r\n')
        # Swap to faster baud
        self.stepper_ser.write(f'/_b{const.SERIAL_BAUD_FAST}R\r\n'.encode())
        self.stepper_ser.close()
        self.stepper_ser = StepperSerial(const.STEPPER_SERIAL_PORT, const.SERIAL_BAUD_FAST, timeout=const.SERIAL_TIMEOUT)
        for address in self.steppers.keys():
            # initialize driver settings
            resp = hw.ezstepper_write(
                self.stepper_ser,
                self.steppers[address],
                (
                    'n0' + # clear any special modes
                    f'm{const.MOVE_CURRENT_PCT}' +
                    f'h{const.HOLD_CURRENT_PCT}' +
                    f'L{const.MAX_ACCEL_TICKS}' + # acceleration factor
                    'aC200' + # encoder coarse correction deadband, ticks
                    'ac5' + # encoder fine correction deadband, ticks
                    # encoder ratio: 1000 * (usteps / rev) / (encoder ticks / rev)
                    'aE1280' + # 1280 = 1000 * (256 * 200) / 40000
                    'au1' + # number of retries allowed under stall condition
                    'z400000' + # set encoder zero point
                    'n8' + # enable encoder feedback mode
                    'R\r\n'
                )
            )
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
            queue.put(input())


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
                + f' queue length {const.MAX_QLEN}. Increase'
                + ' constants.MAX_QLEN.')

        for i in range(self.sequence_len):
            cmd = {}
            cmd['flasher_cmds'] = int(rows[i][0])
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
                    elif 'h' == kbd_in:
                        logger.info('Moving to home requested.')
                        self.mode = 'HOME'
                    elif 'r' == kbd_in:
                        logger.info('Sequence run requested.')
                        self.mode = 'RUN'
                    elif 's' == kbd_in:
                        logger.info('Select sequence requested.')
                        self.mode = 'SEL'
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
                elif self.mode == 'HOME':
                    self.go_home()
                    self.mode = 'WAIT'
                    print(MENU_STR)
                elif self.mode == 'RUN':
                    self.sequence(fname)
                elif self.mode == 'SEL':
                    fname = self.get_new_sequence()
                    self.mode = 'WAIT'
                    print(MENU_STR)
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
            cmd['flasher_cmd'] = 0
            cmd['pos_cmd']     = self.robot.home
            self.do_motor_tasks(cmd)
            self.router.process_tm(plot_enable=self.plot_enable)
            logger.info(f'Home.')
        else:
            logger.info('Already home, nothing to do.')
        return


    def cal_home(self):
        '''
        Moves motors to limits to calibrate the stepper positions.

        In turn, move each motor to its hard stop and record the encoder position.

        Assumptions:
        - All cables are slack, but without excessive cable played out.
        - Corners form a rectangle.
        '''
        # ought to be smaller than distance stepper bounced back after hitting hard stop
        coarse_ticks = 5000
        coarse_homing_speed = 80000
        fine_ticks = 500
        fine_homing_speed = coarse_homing_speed
        # sets the final zero-point length error for each axis
        ultra_fine_ticks = 5
        ultra_fine_homing_speed = coarse_homing_speed

        # Home each axis
        axes = ['nw', 'ne', 'se', 'sw']
        for current_axis in axes:
            logger.info(f'Homing to {current_axis}')
            # turn off current to axes not being homed
            resp = hw.ezstepper_write(self.stepper_ser, '_', 'm0R\r\n')
            resp = hw.ezstepper_write(self.stepper_ser, '_', 'h0R\r\n')

            logger.info('Coarse homing against hard stop.')
            hw.bump_hard_stop(
                self.stepper_ser,
                self.steppers[current_axis],
                coarse_ticks,
                coarse_homing_speed,
                move_current=50,
                hold_current=50
            )
            logger.info('Fine homing against hard stop.')
            hw.bump_hard_stop(
                self.stepper_ser,
                self.steppers[current_axis],
                fine_ticks,
                fine_homing_speed,
                move_current=50,
                hold_current=10
            )
            logger.info('Ultra-fine homing against hard stop.')
            hw.bump_hard_stop(
                self.stepper_ser,
                self.steppers[current_axis],
                ultra_fine_ticks,
                ultra_fine_homing_speed,
                move_current=100,
                hold_current=10
            )
            # run to position of hard stop
            resp = hw.ezstepper_write(
                self.stepper_ser,
                self.steppers[current_axis],
                (
                    'm50' +
                    'h50' +
                    f'V{fine_homing_speed}' +
                    f'D{fine_ticks}' +
                    'R\r\n'
                )
            )
            # set cable length zero point
            time.sleep(1)
            hw.wait_for_ready(self.stepper_ser, self.steppers[current_axis])
            resp = hw.ezstepper_write(self.stepper_ser, self.steppers[current_axis], 'z0R\r\n')
            time.sleep(1)
            # Check to see that homing operation succeeeded.
            if hw.get_encoder_pos(self.stepper_ser, self.steppers[current_axis]) > 20:
                logger.critical('Encoder failed to set zero point during homing. Aborting.')
                sys.exit(1)

        resp = hw.ezstepper_write(self.stepper_ser, '_', f'm{const.MOVE_CURRENT_PCT}R\r\n')
        resp = hw.ezstepper_write(self.stepper_ser, '_', f'h{const.HOLD_CURRENT_PCT}R\r\n')

        # Update current encoder position (used for velocity calc in move to
        # home pos)
        for ax in axes:
            address = self.steppers[ax]
            hw.wait_for_ready(self.stepper_ser, address)
            actual_ticks = hw.get_encoder_pos(self.stepper_ser, address)
            hw.ezstepper_write(self.stepper_ser, address, f'z{actual_ticks}R\r\n')
            hw.wait_for_ready(self.stepper_ser, address)
            hw.ezstepper_write(self.stepper_ser, address, f'A{actual_ticks}R\r\n')

        self.robot.raft.position = self.robot.surf.sw + np.array([0.1, 0.1])
        home = (
            np.mean([self.robot.surf.ne[0], self.robot.surf.sw[0]]),
            np.mean([self.robot.surf.ne[1], self.robot.surf.sw[1]])
        )
        self.robot.home = home
        self.go_home()

        logger.info(f'Raft is homed with centroid position {self.robot.raft.position}')
        logger.warning('Verify that the raft has been driven to the center of the workspace and all cables are taut. If not, request CAL_HOME again.')

        packet = {'algorithm':
            {
                'Local Time (s)': time.time(),
                'Position Command (m)' : home,
                'Motor Angle Command (rad)' : np.array([0.] * len(self.steppers.keys())),
            }
        }
        self.tm_queue.put(packet)
        self.router.process_tm(plot_enable=self.plot_enable)
        return


    def sequence(self, fname: str):
        '''
        On each call, pop a new command off of the command queue and dispatch
        it to motors/Hawkeyes.

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
            self.positions_visited = 0
            # Make a new timestamped logfile
            self.router = tm.DataRouter(self.tm_queue)

        num_remaining = self.cmd_queue.qsize()
        if num_remaining < 1:
            self.mode = 'WAIT'
            return
        else:
            cmd = self.cmd_queue.get()

        progress = 100. * (1 + self.sequence_len - num_remaining) / self.sequence_len
        # Do motor tasks, then LJ tasks, so IR source tasks happen at the end of each move.
        self.do_motor_tasks(cmd)
        self.do_hawkeye_tasks(cmd)
        logger.info(f'Raft centroid: {self.robot.raft.position}')
        logger.info(f'Command completed. Sequence progress: {progress:.2f} %')
        self.router.process_tm(plot_enable=self.plot_enable)
        return


    def get_new_sequence(self):
        '''
        Prompt the user to input the path to a sequence .csv file to run.

        Returns
        -------
        fname
            the filename of the new scan profile to run on next 'r' command
        '''
        profile_dir = os.path.join(const.TOPLEVEL_DIR, 'data', 'input', 'profiles')
        print(
            'Please input the path to a new profile .csv file to run. ' +
            f'Profile sequence files are stored in {profile_dir}.\nHit Enter' +
            ' for the prompt.'
        )
        input_str = input('Type a file path, or type \'l\' for a list: ')
        if input_str == 'l':
            print('Options are:')
            [print(file) for file in os.listdir(profile_dir)]
            fname = self.get_new_sequence()
        else:
            fname = os.path.abspath(os.path.join(profile_dir, input_str))
            if not os.path.exists(fname):
                print(f'File {fname} does not exist.')
                fname = self.get_new_sequence()
            else:
                print(f'The next run command (r) will execute this profile: \n{fname}')
        return fname


    def wait(self):
        if self.plot_enable:
            self.router.run_gui_event_loop()
        return


    def blink(self):
        '''
        Blink all sources.
        '''
        cmd = {}
        cmd['flasher_cmds'] = 7 # center, inner ring, outer ring. binary: 111.
        self.do_hawkeye_tasks(cmd)
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
        def send_pos_cmd(cmd_tuple: tuple, run=True):
            '''
            Helper to send position commands to control algorithm and collect
            results
            '''
            motor_cmds = self.robot.process_input(cmd_tuple)
            # bootleg OrderedDict
            keys = STEPPER_ORDER
            angs = [cmd for cmd in [motor_cmds[key] for key in keys]]
            ticks_to_go, err = hw.all_steppers_ez(
                self.stepper_ser,
                [self.steppers[key] for key in keys],
                angs,
                run=run
            )

        # Linear approximation only holds for small distances, so
        # chunk up big moves into tiny bits. This ensures all cables stay taut,
        # even when moving to opposite corners.
        # CHUNK_DIST is a trade-off between smoothly approximating the needed
        # stepping profiles and the time taken to send many commands over
        # serial before starting the move.
        pos_before = self.robot.raft.position
        pos_after = cmd['pos_cmd']
        dist_to_go = np.linalg.norm(np.array(pos_after) - np.array(pos_before))
        wait_time = dist_to_go * const.ENCODER_TICKS_PER_REV / const.LENGTH_PER_REV / const.MAX_SPEED_TICKS
        while dist_to_go > const.CHUNK_DIST:
            logger.debug(f'Dist. to go in this move: {dist_to_go}')
            # determine a position CHUNK_DIST away from the starting pos along
            # the direction of travel
            v = np.array(pos_after) - np.array(pos_before)
            u = v / np.linalg.norm(v) # the unit vector pointing along the line
            logger.debug(f'Direction vector: {v}, Unit vector: {u}')
            pos_cmd = pos_before + const.CHUNK_DIST * u
            logger.debug(f'Intermediate move: {pos_cmd}')
            send_pos_cmd(pos_cmd, run=True) # alg updates raft position
            # calculate new dist to go
            pos_before = self.robot.raft.position
            dist_to_go = np.linalg.norm(np.array(pos_after) - np.array(pos_before))
            # temporary until streaming commands is fixed
            wait_time = dist_to_go * const.ENCODER_TICKS_PER_REV / const.LENGTH_PER_REV / const.MAX_SPEED_TICKS
        # Signal ezsteppers to run once we've sent all commands for this move
        logger.debug(f'Final move: {pos_after}')
        send_pos_cmd(pos_after, run=True)
        time.sleep(wait_time)
        return


    def take_image(self):
        time.sleep(1)
        self.positions_visited += 1
        cmd = ['bash', '-c', f'. takeimg.sh {self.positions_visited}']
        ret = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if ret.returncode == 0:
            print(ret.stdout.decode())
        else:
            if ret.stdout:
                print(ret.stdout.decode())
            if ret.stderr:
                print(ret.stderr.decode())
        return


    def do_hawkeye_tasks(self, cmd: dict):
        packet = {'Hawkeye Cmd':
            {
                'Local Time (s)': time.time(),
                'Hawkeye Cmd Byte' : np.array(cmd['flasher_cmds']),
            }
        }
        self.tm_queue.put(packet)

        time.sleep(const.SETTLE_TIME)

        # self.take_image()

        freq = 10. # switching freq, Hz, i.e. flashing freq is 1/2 this
        num_blinks = 10
        flipflop = 0
        while num_blinks > 0:
            start_time = time.time()
            if flipflop:
                hw.send_hawkeye_byte(self.hawkeye_ser, cmd['flasher_cmds'])
                num_blinks -= 1
            else:
                hw.send_hawkeye_byte(self.hawkeye_ser, 0)
            flipflop = 1 - flipflop
            # sleep off remaining time to fudge actions at frequency freq
            time.sleep((1. / freq) - (time.time() - start_time))
        hw.send_hawkeye_byte(self.hawkeye_ser, 0)
        return


    def close(self):
        # ensure all hawkeyes are left in off state
        hw.send_hawkeye_byte(self.hawkeye_ser, 0)
        self.hawkeye_ser.close()
        # Terminate all running commands
        time.sleep(.1)
        hw.ezstepper_write(self.stepper_ser, '_', 'TR\r\n')
        # Release torque
        time.sleep(.1)
        hw.ezstepper_write(self.stepper_ser, '_', 'm0R\r\n')
        hw.ezstepper_write(self.stepper_ser, '_', 'h0R\r\n')
        # Restore default baud
        hw.ezstepper_write(self.stepper_ser, '_', 'b9600R\r\n')
        self.stepper_ser.close()
        return
