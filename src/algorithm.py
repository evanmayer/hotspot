# This file houses the algorithms necessary for calculating the control
# quantities.
# MKS units only.
import numpy as np


def rot(theta):
    '''2D rotation matrix in the plane'''
    # rotations are done like pts_mtx @ rot_mtx
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta),  np.cos(theta)]])


class TestSurface(object):
    '''
    Contains the mirror-dependent geometry of attachment points, 
    as-constructed/measured, relative to the mirror origin.
    '''
    def __init__(self, nw: tuple, ne: tuple, se: tuple, sw=(0.,0.)):
        # Define the sw corner as the origin, so default (0,0).
        for inp in [nw, ne, se, sw]:
            assert(len(inp) == 2,
                'This module is 2D planar only, so points should be 2-vectors.')
        # Corner points are expressed as x,y offsets in mirror coordinate frame
        self.corners = np.array([[sw, nw], [se, ne]])
        self.sw = self.corners[0,0]
        self.se = self.corners[1,0]
        self.nw = self.corners[0,1]
        self.ne = self.corners[1,1]
        self.origin = self.sw


class Raft(object):
    '''
    Contains the payload-dependent geometry of attachment points,
    as-constructed/measured, relative to the payload origin.
    '''
    def __init__(self, width: float, height: float, position: tuple):
        assert(len(position) == 2,
                'This module is 2D planar only, so points should be 2-vectors.')
        # locations of attachment points in raft coordinate frame
        self.corners = np.array(
            [[(-width / 2., -height / 2), (-width / 2.,  height / 2)],
             [( width / 2., -height / 2), ( width / 2.,  height / 2)]]
        )
        # position of the raft origin in mirror coordinate frame
        self.position = position

    @property
    def sw(self):
        return self.corners[0,0] + self.position
    @property
    def se(self):
        return self.corners[1,0] + self.position
    @property
    def nw(self):
        return self.corners[0,1] + self.position
    @property
    def ne(self):
        return self.corners[1,1] + self.position


class Robot(object):
    '''
    Contains all information and functions needed to translate a sequence of 
    commands into a set of cable length deltas and velocities.
    '''
    def __init__(self, surf: TestSurface, raft: Raft, cmd_sequence: list):
        self.surf = surf
        self.raft = raft
        self.dt = 1.
        self.sequence_start_time = 0.
        self.sequence_start_elapsed = 0.
        self.move_start_time = 0.
        self.move_time_elapsed = 0.
        self.cmd_sequence = cmd_sequence


    def calc_lengths(self):
        '''
        Calculates the final lengths of each leg given the commanded
        position.
        '''
        return


    def calc_length_rates(self):
        '''
        Calculates the rates of change of each leg length given the commanded
        position and allowed time.
        '''
        return


    def calc_move_progress(self):
        '''
        Estimates the progress through the current move, given the allowed time
        and time elapsed since start.
        '''
        return


    def calc_sequence_progress(self):
        '''
        Estimates the progress through the current sequence, given the allowed time
        and time elapsed since start.
        '''
        return
