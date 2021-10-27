# This file houses the code to coordinate starting, running, and stopping
# the various processes/threads needed.

from adafruit_motorkit import MotorKit
import algorithm as alg
import constants as const
import hardware as hw
import logging
import multiprocessing as mp
import numpy as np


MODES = ('SET_HOME', 'HOME', 'SEQ', 'JOG', 'STARE')

class Executive(object):
    '''Handles control flow with a state machine.'''
    def __init__(self):
        self.state = 'SET_HOME'
        self.kbd_queue = mp.Queue(1)
        self.cmd_queue = mp.Queue(const.MAX_COMMANDS)


    def _get_kbd(self, queue):
        '''
        Helper function to run in a separate thread and add user input chars 
        to a buffer.
        '''
        while True:
            queue.put(sys.stdin.read(1))


    def parse_cmds(self, fname: str):
        '''
        Read command input file and add commands to the command queue.
        '''
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
            cmd['dwell_cmd']    = rows[i][6]
            self.cmd_queue.put(cmd)


    def empty_queue(self, queue: mp.Queue):
        while not queue.empty():
            queue.get()


    def run(self, fname: str):
        '''
        Main run function, including processing human input to switch between
        states and processing input commands from file.
        '''
        self.parse_cmds(fname)


    def close(self):
        pass