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
    def __init__(self, Point: nw, Point: ne, Point: se):
        # Define the sw corner as the origin, so always (0,0).
        # Other corner points in the plane are expressed as u,v offsets
        self.sw = Point(0., 0.)
        self.nw = Point(nw.u, nw.v)
        self.ne = Point(ne.u, ne.v)
        self.se = Point(se.u, se.v)


class Raft(object):
    '''
    Contains the payload-dependent geometry of attachment points,
    as-constructed/measured, relative to the payload origin.
    '''
    def __init__(self, width, height, position):
        self.sw = Point(-1. * width / 2., -1. * height / 2)
        self.nw = Point(-1. * width / 2.,       height / 2)
        self.ne = Point(      width / 2.,       height / 2)
        self.se = Point(-1. * width / 2., -1. * height / 2)
        # location of payload origin in mirror coordinate frame
        self.position = 0.0 


class Robot(object):
    '''
    Contains all information and functions needed to translate a commanded
    position into a set of motor commands via cable length deltas and 
    velocities.
    '''
    def __init__(self, TestSurface: surf, Raft: raft):
        self.surf = surf
        self.raft = raft
