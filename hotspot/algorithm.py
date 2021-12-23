# This file houses the algorithms necessary for calculating the control
# quantities.
# MKS units only.

import logging
import multiprocessing as mp
import numpy as np
import time

import hotspot.constants as const


logger = logging.getLogger(__name__)
# logger.setLevel(getattr(logging, const.LOGLEVEL))

class TestSurface:
    '''
    Contains the mirror-dependent geometry of attachment points, 
    as-constructed/measured, relative to the mirror origin.
    '''
    def __init__(self, sw=(0.,0.), se=(1.,0.), nw=(0.,1.), ne=(1.,1.)):
        logger.debug('TestSurface init')
        corner_list = [nw, ne, se, sw]
        for inp in corner_list:
            assert len(inp) == 2, \
                f'This module is 2D planar only, so points should be 2-vectors instead of {inp}.'
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
        vertices. If they sum to 360, you're inside the shape.
        '''
        # ECM: Might be able to exploit matrix math to make faster
        # e.g. make two arrays of vertices and vectorize the for loop
        eps = 1e-9
        result = True
        ang_tot_rad = 0.
        vertex_seq = [self.sw, self.nw, self.ne, self.se, self.sw]
        reason = None
        for i in range(len(vertex_seq) - 1):
            disp0 = vertex_seq[i] - pos
            disp1 = vertex_seq[i+1] - pos
            mag0 = np.linalg.norm(disp0)
            mag1 = np.linalg.norm(disp1)
            # is pos too close to a vertex (protect against divide by 0)
            if (mag0 < eps or mag1 < eps):
                result = False
                reason = 'Too close to corner'
                break
            v0_hat = disp0 / mag0
            v1_hat = disp1 / mag1
            ang = np.arccos(np.dot(v0_hat, v1_hat))
            # is pos too close to an edge
            if np.abs(ang - np.pi) < eps:
                result = False
                reason = 'Too close to edge'
                break
            ang_tot_rad += ang
        # is pos inside shape
        if np.abs(ang_tot_rad - 2. * np.pi) > eps:
            result = False
            reason = f'Outside shape: {np.abs(ang_tot_rad - 2. * np.pi)}'

        if reason:
            logger.debug(f'Bounds check failed: {reason}')

        return result


class Raft:
    '''
    Contains the payload-dependent geometry of attachment points,
    as-constructed/measured.
    '''
    def __init__(self, position: tuple, width: float, height: float):
        logger.debug('Raft init')
        assert len(position) == 2, \
            f'This module is 2D planar only, so points should be 2-vectors instead of {position}.'
        # Private attribute maintains coords in raft frame
        # origin implicit at centroid of rectangle
        self._corners_priv = np.array(
            [[(-width / 2., -height / 2.), (-width / 2.,  height / 2.)],
             [( width / 2., -height / 2.), ( width / 2.,  height / 2.)]]
        )
        # Public attribute maintains coords in mirror frame
        self.corners = self._corners_priv + position
        # position of the raft origin in mirror coordinate frame
        self._position = position
        
        self.sw = self.corners[0,0]
        self.se = self.corners[1,0]
        self.nw = self.corners[0,1]
        self.ne = self.corners[1,1]


    @property
    def position(self):
        '''Accessor for raft origin position in mirror coordinate frame'''
        return self._position

    @position.setter
    def position(self, new_pos):
        '''
        Setter for raft origin position ensures the raft corners follow when
        position is updated
        '''
        self.corners = self._corners_priv + new_pos
        self.sw = self.corners[0,0]
        self.se = self.corners[1,0]
        self.nw = self.corners[0,1]
        self.ne = self.corners[1,1]
        self._position = new_pos



class Robot:
    '''
    Contains all information and functions needed to translate a sequence of 
    commands into a set of cable length deltas and velocities, and then into
    a set of motor commands.
    '''
    def __init__(self, surf: TestSurface, raft: Raft, tm_queue: mp.Queue):
        logger.debug('Robot init')
        # Geometry
        self.surf = surf
        self.raft = raft

        # Init home to an invalid position until we are homed
        self._home = (-np.inf, -np.inf)
        # Start off pos_cmd in an error state, hoping an error will occur
        # If we attempt to move before issuing a real pos_cmd
        self._pos_cmd = self._home

        # A handle to the queue for outputting TM packets for vis and logging
        self.tm_queue = tm_queue


    @property
    def home(self):
        '''
        The location in mirror coordinate frame the robot will guide to
        when commanded home
        '''
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
        '''
        The location in mirror coordinate frame the robot will guide to for the 
        next move
        '''
        return self._pos_cmd

    @pos_cmd.setter
    def pos_cmd(self, new_pos):
        inbounds = self.surf.is_inbounds(new_pos)
        if inbounds:
            self._pos_cmd = new_pos
        else:
            raise ValueError(f'Position command {new_pos} is outside of bounds for surface {self.surf}')


    def process_input(self, pos_cmd: tuple):
        '''
        Translate a move command into 4 motor commands. Motors should be
        attached to cables such that increasing the length of the cable played
        out requires a positive-valued rotation (motor shaft spins clockwise)
        from rear.

        Parameters
        ----------
        pos_cmd
            the position command in the frame of the surface
        
        Returns
        -------
        motor_cmds : dict of tuples
            dict containing pairs of (radians, rad_per_sec) motor commands. 
        '''
        motor_cmds = {
            'sw': (0., 0.),
            'nw': (0., 0.),
            'ne': (0., 0.),
            'se': (0., 0.)
        }

        # Input checking, this is just in case of malformed inputs.
        eps = np.finfo(float).eps
        distance = np.linalg.norm(np.array(pos_cmd) - self.raft.position)
        if distance <= eps:
            logger.warning(f'Position command malformed: distance: {distance}')
            return motor_cmds

        lengths_before = np.linalg.norm(self.raft.corners - self.surf.corners, axis=-1)
        # update the commanded position
        self.pos_cmd = pos_cmd
        # update the raft's position to move its corners
        self.raft.position = pos_cmd
        lengths_after = np.linalg.norm(self.raft.corners - self.surf.corners, axis=-1)

        delta_lengths = lengths_after - lengths_before
        delta_angles = delta_lengths / const.PULLEY_RADIUS

        motor_cmds['sw'] = delta_angles[0, 0]
        motor_cmds['se'] = delta_angles[1, 0]
        motor_cmds['nw'] = delta_angles[0, 1]
        motor_cmds['ne'] = delta_angles[1, 1]

        logger.debug(f'Motor commands: {motor_cmds}\nDelta lengths:{delta_lengths}')
        
        packet = {'algorithm':
            {
                'Time UTC (s)': time.time(),
                'Position Command (m)' : pos_cmd,
                'Motor Delta Angle Command (rad)' : delta_angles.flatten(),
            }
        }
        self.tm_queue.put(packet)

        return motor_cmds
