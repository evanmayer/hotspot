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
    def __init__(self, sw=(0.,0.), se=(1.,0.), nw=(0.,1.), ne=(1.,1.)):
        # Define the sw corner as the origin, should always be (0,0)
        for inp in [nw, ne, se, sw]:
            assert len(inp) == 2, \
                f'This module is 2D planar only, so points should be 2-vectors instead of {inp}.'
        # Corner points are expressed as x,y offsets in mirror coordinate frame
        self.corners = np.array([[sw, nw], [se, ne]])
        self.sw = self.corners[0,0]
        self.se = self.corners[1,0]
        self.nw = self.corners[0,1]
        self.ne = self.corners[1,1]
        self.origin = self.sw
    

    def is_inbounds(self, pos: tuple):
        '''
        Interior/exterior test: sum angles between pos and successive pairs of
        vertices. If they sum to 360, you're inside the shape. Do not allow 
        positions close to edges (angle ~= 180 deg).
        '''
        # ECM: Might be able to exploit matrix math to make faster
        result = True
        ang_tot_rad = 0.
        vertex_seq = [self.sw, self.nw, self.ne, self.se, self.sw]
        for i in range(len(vertex_seq) - 1):
            if i < len(vertex_seq):
                v0_hat = (vertex_seq[i]   - pos) / np.linalg.norm(vertex_seq[i]   - pos)
                v1_hat = (vertex_seq[i+1] - pos) / np.linalg.norm(vertex_seq[i+1] - pos)
                ang = np.arccos(np.dot(v0_hat, v1_hat))
                if np.abs(ang - np.pi) < np.finfo(float).eps:
                    result = False
                    break
                ang_tot_rad += ang
                
        if np.abs(ang_tot_rad - 2. * np.pi) > np.finfo(float).eps:
            result = False

        return result



class Raft(object):
    '''
    Contains the payload-dependent geometry of attachment points,
    as-constructed/measured, relative to the payload origin.
    '''
    def __init__(self, position: tuple, width: float, height: float):
        assert len(position) == 2, \
            f'This module is 2D planar only, so points should be 2-vectors instead of {position}.'
        # locations of attachment points in raft coordinate frame
        self.corners = np.array(
            [[(-width / 2., -height / 2.), (-width / 2.,  height / 2.)],
             [( width / 2., -height / 2.), ( width / 2.,  height / 2.)]]
        )
        # position of the raft origin in mirror coordinate frame
        self._position = position

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, new_pos):
        '''Reject position input if outside convex hull of surface vertices'''


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
