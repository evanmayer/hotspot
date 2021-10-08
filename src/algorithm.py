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
        corner_list = [nw, ne, se, sw]
        for inp in corner_list:
            assert len(inp) == 2, \
                f'This module is 2D planar only, so points should be 2-vectors instead of {inp}.'
        # Corners should not share locations
        assert len({elem for elem in corner_list}) == len(corner_list), f'Vertices should be unique: {corner_list}'

        # Corner points are expressed as x,y offsets in mirror coordinate frame
        self.corners = np.array([[sw, nw], [se, ne]])
        self.sw = self.corners[0,0]
        self.se = self.corners[1,0]
        self.nw = self.corners[0,1]
        self.ne = self.corners[1,1]


    def is_inbounds(self, pos: tuple):
        '''
        Interior/exterior test: sum angles between pos and successive pairs of
        vertices. If they sum to 360, you're inside the shape. Do not allow 
        positions close to edges (angle ~= 180 deg).
        '''
        # ECM: Might be able to exploit matrix math to make faster
        # e.g. make two arrays of vertices and vectorize the for loop
        eps = np.finfo(float).eps
        result = True
        ang_tot_rad = 0.
        vertex_seq = [self.sw, self.nw, self.ne, self.se, self.sw]
        for i in range(len(vertex_seq) - 1):
            disp0 = vertex_seq[i] - pos
            disp1 = vertex_seq[i+1] - pos
            mag0 = np.linalg.norm(disp0)
            mag1 = np.linalg.norm(disp1)
            # pos is too close to a vertex (protect against divide by 0)
            if (mag0 < eps or mag1 < eps):
                result = False
                break

            v0_hat = disp0 / mag0
            v1_hat = disp1 / mag1
            ang = np.arccos(np.dot(v0_hat, v1_hat))
            if np.abs(ang - np.pi) < eps:
                result = False
                break
            ang_tot_rad += ang
                
        if np.abs(ang_tot_rad - 2. * np.pi) > eps:
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
        # Geometry
        self.surf = surf
        self.raft = raft
        # Map surface attachmentpoints to raft attachmentpoints
        self.surf_to_raft = {
            'sw' : raft.sw,
            'nw' : raft.nw,
            'ne' : raft.ne,
            'se' : raft.se
        }
        # Control algorithm settings
        self.dt = 1.
        self.sequence_start_time = -1.
        self.sequence_start_elapsed = -1.
        self.move_start_time = -1.
        self.move_time_elapsed = -1.
        self.cmd_sequence = cmd_sequence
        # Init home to an invalid position until we are homed
        self._home = (-np.inf, -np.inf)
        self.current_pos = self._home
        # Start off pos_cmd in an error state, hoping an error will occur
        # If we attempt to move before issuing a real pos_cmd
        self._pos_cmd = self._home


    @property
    def home(self):
        return self._home

    @home.setter
    def home(self, home_pos):
        inbounds = self.surf.is_inbounds(home_pos)
        if inbounds:
            self._home = home_pos
        else:
            raise ValueError(f'Home location {home_pos} is outside of bounds for surface {self.surf}')

    @property
    def pos_cmd(self):
        return self._pos_cmd


    @pos_cmd.setter
    def pos_cmd(self, new_pos):
        inbounds = self.surf.is_inbounds(new_pos)
        if inbounds:
            self._pos_cmd = new_pos
        else:
            raise ValueError(f'Position command {new_pos} is outside of bounds for surface {self.surf}')


    def calc_lengths(self):
        '''
        Calculates the final lengths of each leg given the current commanded
        position.
        '''
        return


    def calc_length_rates(self):
        '''
        Calculates the rates of change of each leg length given the curernt
        commanded position and allowed time.
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
    

    def run():
        '''
        The function that runs the algorithm in the overall state machine.
        '''
        pass
