# This file houses the code to coordinate starting, running, and stopping
# the various processes/threads needed.

from adafruit_motorkit import MotorKit
import algorithm as alg
import constants as const
import hardware as hw
import logging
import multiprocessing as mp
import os
import sys
import threading
import numpy as np


logger = logging.getLogger(__name__)
logger.setLevel(getattr(logging, 'DEBUG'))

MODES = {'c': 'CAL_HOME', 'h': 'HOME', 's': 'SEQ', 'j': 'JOG', 't': 'STARE'}

class Executive(object):
    '''Handles control flow with a state machine.'''
    def __init__(self, geometry_file: str):
        self.mode = 'CAL_HOME'
        self.last_mode = 'CAL_HOME'
        self.kbd_queue = mp.Queue(1)
        self.cmd_queue = mp.Queue(const.MAX_COMMANDS)

        # HACK TODO FIXME ECM: hard coded geometry until input file coded
        sw = (0,0)
        nw = (0,1)
        ne = (1,1)
        se = (1,0)
        surf = alg.TestSurface(sw=sw, se=se, nw=nw, ne=ne)
        w = .1
        h = .1
        pos = (0.5,0.5)
        raft = alg.Raft(pos, w, h)
        self.robot = alg.Robot(surf, raft)

        kit0 = MotorKit(address=const.HAT_0_ADDR, steppers_microsteps=const.MICROSTEP_NUM)
        # kit1 = MotorKit(address=const.HAT_1_ADDR, steppers_microsteps=const.MICROSTEP_NUM)
        # HACK TODO FIXME ECM: Need to dupe some motors until second HAT connected
        self.steppers = {
            'sw': kit0.stepper1,
            'ne': kit0.stepper2,
            'nw': kit0.stepper1,
            'se': kit0.stepper2
        }


    def _get_kbd(self, queue: mp.Queue):
        '''
        Helper function to run in a separate thread and add user input chars 
        to a buffer.
        '''
        while True:
            queue.put(sys.stdin.read(1))


    def add_cmds(self, fname: str):
        '''
        Read command input file and add commands to the command queue.
        '''
        logger.info(f'Parsing command sequence: {fname}')
        rows = np.genfromtxt(
            fname,
            dtype=None,
            delimiter=',',
            skip_header=1,
            encoding='utf8'
            )

        num_cmds = len(rows)
        assert (num_cmds <= const.MAX_COMMANDS), ('Input command number exceeds'
            + ' command queue length. Increase const.MAX_COMMANDS.')

        for i in range(num_cmds):
            cmd = {}
            cmd['flasher_mode'] = rows[i][0] # TODO: ECM: NotImplemented
            cmd['flasher_cmd']  = rows[i][1] # TODO: ECM: NotImplemented
            cmd['move_mode']    = rows[i][2]
            cmd['pos_cmd']      = (rows[i][3], rows[i][4])
            cmd['speed_cmd']    = rows[i][5]
            self.cmd_queue.put(cmd)


    def empty_queue(self, queue: mp.Queue):
        while not queue.empty():
            queue.get()


    def run(self, fname: str):
        '''
        Main run function, including processing human input to switch between
        states.
        '''
        input_thread = threading.Thread(target=self._get_kbd,
            args=(self.kbd_queue,),
            daemon=True
        )
        input_thread.start()
        print(f'Listening for user input for mode changes. Type a mode char and press enter: {MODES}')

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
                    elif 'j' == kbd_in:
                        logger.info('Jog mode requested.')
                        self.mode = 'JOG'
                    elif 't' == kbd_in:
                        logger.info('Stare mode requested.')
                        self.mode = 'STARE'
                    else:
                        continue

                if self.mode == 'CAL_HOME':
                    self.cal_home()
                elif self.mode == 'HOME':
                    self.home()
                elif self.mode == 'SEQ':
                    self.sequence(fname)
                elif self.mode == 'JOG':
                    self.jog()
                elif self.mode == 'STARE':
                    self.stare()
                else:
                    pass
                self.last_mode = self.mode

            except KeyboardInterrupt:
                print('\nCaught KeyboardInterrupt, shutting down.')
                running = False
        self.close()


    def cal_home(self):
        pass


    def home(self):
        pass


    def sequence(self, fname: str):
        # If we are changing to sequence from another mode, ensure we start
        # fresh
        if self.mode != self.last_mode:
            self.empty_queue(self.cmd_queue)
            self.add_cmds(fname)

        if self.cmd_queue.qsize() < 1:
            return
        else:
            cmd = self.cmd_queue.get(timeout=1)
        # TODO: Process LabJack commands

        # Process move commands if necessary
        motor_threads = []
        if cmd['move_mode'] == 'move':
            motor_cmds = self.robot.process_input(cmd['pos_cmd'], cmd['speed_cmd'])
            # Dish out motor commands to each motor
            for key in motor_cmds.keys():
                # HACK TODO FIXME ECM: Need to skip some keys until both HATs connected
                if key in ['nw', 'se']:
                    continue
                stepper = self.steppers[key]
                print(*motor_cmds[key])
                thread = threading.Thread(target=hw.move_motor, args=(stepper, *motor_cmds[key]), daemon=True)
                thread.start()
                motor_threads.append(thread)

        # If move mode is dwell, just do whatever the LabJack needs to do.

        # Join all threads
        [thread.join() for thread in motor_threads]


    def jog(self):
        pass


    def stare(self):
        pass


    def close(self):
        # HACK TODO FIXME ECM: When all steppers addressable, release all of them.
        for key in self.steppers.keys():
            if key in ['nw', 'se']:
                continue
            self.steppers[key].release()