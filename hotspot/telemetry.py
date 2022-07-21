# This file defines functions for outputting telemetry (TM) and visualization.

import logging
import matplotlib.pyplot as plt
import multiprocessing as mp
import numpy as np
import os
import time

import h5py

import hotspot.constants as const


logger = logging.getLogger(__name__)

class Visualizer:
    '''
    Keeps track of plotting objects to enable updating plots for a given source
    '''
    def __init__(self, file_handle: h5py.File, source: str):
        logger.debug('Visualiser init')
        self.file_handle = file_handle
        self.source = source

        keys = [key for key in file_handle[source].keys() if 'local time' not in key.lower()]
        fig, axes = plt.subplots(nrows=len(keys), squeeze=False)
        fig.suptitle(f'Packet: {source}')
        self.fig = fig

        self.var_axes = {key: axes.flatten()[i] for i, key in enumerate(keys)}

        plt.ion()
        plt.show()


    
    def update_subplot(self, file_handle: h5py.File, varname: str, first_time: bool):
        ax = self.var_axes[varname]
        fig = ax.get_figure()

        if first_time:
            ax.set_title(varname)
            ax.grid(True)
        
        time = np.atleast_1d(file_handle[self.source]['Local Time (s)'])
        data = np.atleast_1d(file_handle[self.source][varname])
        dim = len(data.shape)
        if dim < 2: # 1D data
            if first_time:
                ax.plot(time, data)
            else:
                l = ax.lines[0]
                l.set_data(time, data)
        elif dim < 3:
            # scatter plot for positions
            if 'position' in varname.lower():
                if first_time:
                    ax.plot(data[:,0], data[:,1], color='k', marker='o', linestyle='None')
                else:
                    l = ax.lines[0]
                    l.set_data(data[:,0], data[:,1])
            else: # line plot for timeseries
                if first_time:
                    ax.plot(time[:], data, marker='o', alpha=0.3)
                else:
                    for j in range(data.shape[1]):
                      ax.lines[j].set_data(time[:], data[:,j])
        else:
            logger.warn(f'[{varname}] Plotting data of shape {data.shape} not implemented.')
        ax.relim()
        ax.autoscale()
        fig.tight_layout()
        fig.canvas.draw()
        plt.pause(1e-9)
        return


class DataRouter:
    def __init__(self, tm_queue: mp.Queue):
        logger.debug('DataRouter init')
        # map output data to the correct file
        self.output_dir = os.path.join(const.TOPLEVEL_DIR, 'data', 'output')
        self.time_str = time.strftime('_%Y%m%d-%H%M%S')
        self.fname = os.path.join(self.output_dir, 'hotspot' + self.time_str + '.hdf5')
        self.tm_queue = tm_queue
        self.visualizers = {}
        return


    def process_packet(self, packet: dict):
        '''
        Route a TM packet to the hdf5 group.

        Packet format:
        {
            'source' :
                'Local Time (s)' : value (all packets should have this key)
                'variable0' : value,
                'variable1' : value
        }
        Times are referenced to UNIX epoch UTC. All packets should have the
        key 'Local Time (s)'.
        Values can be 1- or 2-D.
        '''
        # Otherwise, create or add to output file on disk
        with h5py.File(self.fname, 'a') as f:
            for source in packet.keys():
                # Get a group, making it if it doesn't exist
                if source not in f.keys():
                    f.create_group(source)
                group = f[source]
                for var in packet[source].keys():
                    data = packet[source][var]
                    # Get a dataset, making it if it doesn't exist
                    if var in group.keys():
                        dset = group[var]
                        # extend row axis by one
                        dset.resize((dset.shape[0] + 1, *dset.shape[1:]))
                        dset[-1] = data
                    else:
                        data = np.array([packet[source][var]])
                        maxshape = (None, *data.shape[1:])
                        group.create_dataset(var, maxshape=maxshape, data=data, chunks=True) # Need chunked to enable resizing
        return


    def process_tm(self, plot_enable=False):
        '''
        Function to continuously log TM to file and update plots as TM comes in.
        '''
        while not self.tm_queue.empty():
            self.process_packet(self.tm_queue.get())
            if plot_enable:
                self.update_display()
        return


    def run_gui_event_loop(self):
        plt.pause(1)
        return


    def update_display(self):
        '''
        Function to update plotters.
        '''
        with h5py.File(self.fname) as f:
            for source in f.keys():
                first_time = False
                # Make a new visualizer if needed
                if source not in self.visualizers.keys():
                    self.visualizers[source] = Visualizer(f, source)
                    first_time = True
                for var in f[source].keys():
                    if 'local time' in var.lower():
                        continue
                    self.visualizers[source].update_subplot(f, var, first_time)
        return