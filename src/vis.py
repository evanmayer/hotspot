# This file defines functions for outputting telemetry (TM) and visualization.

import constants as const
import h5py
import logging
import matplotlib.pyplot as plt
import multiprocessing as mp
import os
import time


logger = logging.getLogger(__name__)
logger.setLevel(getattr(logging, 'DEBUG'))

SOURCES = ['executive', 'algorithm', 'hardware']


class Visualizer(object):
    def __init__(self):
        # map output data to the correct file
        self.output_dir = os.path.join(const.TOPLEVEL_DIR, 'data', 'output')
        time_str = time.strftime('_%Y%m%d-%H%M%S')
        self.output_files = {source: os.path.join(self.output_dir, source + time_str + '.hd5') for source in SOURCES}


    def get_tm(self, queue: mp.Queue):
        '''
        Pop a packet off of the queue and route it to the right file/visualizer.

        Packet format:
        {
            'source' :
                'time' : value
                'variable0' : value,
                'variable1' : value
        }
        Times are referenced to UNIX epoch UTC.
        '''
        packet = queue.get()

        if not any(item in packet.keys() for item in SOURCES):
            logger.warning(f'Received unrecognized packet: {packet}')
            return

        
    
