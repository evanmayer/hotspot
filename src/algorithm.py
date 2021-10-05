# This file houses the algorithms necessary for calculating the control
# quantities.
# MKS units only.


class Point(object):
    def __init__(self, u, v):
        self.u = u
        self.v = v


class TestSurface(object):
    '''
    Contains the mirror-dependent geometry of attachment points, 
    as-constructed/measured, relative to the mirror origin.
    '''
    def __init__(self, nw: Point, ne: Point, se: Point):
        # Define the sw corner as the origin, so always (0,0).
        # Other corner points in the plane are expressed as u,v offsets
        self.sw = Point(0., 0.)
        self.nw = Point(nw.u, nw.v)
        self.ne = Point(ne.u, ne.v)
        self.se = Point(se.u, se.v)
        self.origin = self.sw


class Raft(object):
    '''
    Contains the payload-dependent geometry of attachment points,
    as-constructed/measured, relative to the payload origin.
    '''
    def __init__(self, width: float, height: float, position: Point):
        self.sw = Point(-1. * width / 2., -1. * height / 2)
        self.nw = Point(-1. * width / 2.,       height / 2)
        self.ne = Point(      width / 2.,       height / 2)
        self.se = Point(-1. * width / 2., -1. * height / 2)
        # location of payload origin in mirror coordinate frame
        self.origin = position


class Robot(object):
    '''
    Contains all information and functions needed to translate a sequence of 
    commands into a set of cable length deltas and velocities.
    '''
    def __init__(self, surf: TestSurface, raft: Raft, sequence: list):
        self.surf = surf
        self.raft = raft
        self.dt = 1.
        self.sequence_start_time = 0.
        self.sequence_start_elapsed = 0.
        self.move_start_time = 0.
        self.move_time_elapsed = 0.
        self.sequence = sequence


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
