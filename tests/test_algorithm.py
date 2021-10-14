import numpy as np
import pytest
from context import algorithm as alg
from context import constants as const


# ------------------------------------------------------------------------------
# Low-level object testing
# ------------------------------------------------------------------------------
def test_TestSurface_init():
    sw = (0,0)
    nw = (0,1)
    ne = (1,1)
    se = (1,0)
    surf = alg.TestSurface(sw=sw, se=se, nw=nw, ne=ne)

    # test corner assignment
    corner_list = surf.corners.reshape(4,2)
    accessor_list = [surf.sw, surf.nw, surf.se, surf.ne]
    for i, coord in enumerate([sw, nw, se, ne]):
        # underlying matrix
        # This pattern tests for agreement in both elements of the tuple
        assert all(a == b for a,b in zip(coord, corner_list[i])), \
            f'{coord} != {corner_list[i]}'
        # public-facing accessors
        assert all(a == b for a,b in zip(coord, accessor_list[i])), \
            f'{coord} != {accessor_list[i]}'


def test_TestSurface_is_inbounds_good():
    sw = (0,0)
    nw = (0,1)
    ne = (1,1)
    se = (1,0)
    surf = alg.TestSurface(sw=sw, se=se, nw=nw, ne=ne)
    assert surf.is_inbounds((0.5, 0.5))
    assert surf.is_inbounds((0.1, 0.1))
    assert not surf.is_inbounds((0.0, 0.5))
    assert not surf.is_inbounds((1.0, 0.5))
    assert not surf.is_inbounds((1.5, 1.5))


def test_Raft_init():
    w = 2
    h = 3
    pos = (0,0)
    raft = alg.Raft(pos, w, h)

    input_list = [(-w / 2., -h / 2.), (-w / 2.,  h / 2.),
                  ( w / 2., -h / 2.), ( w / 2.,  h / 2.)]
    # test corner assignment
    corner_list = raft.corners.reshape(4,2)
    accessor_list = [raft.sw, raft.nw, raft.se, raft.ne]
    for i, coord in enumerate(input_list):
        # underlying matrix
        assert all(a == b for a,b in zip(coord, corner_list[i])), \
            f'{coord} != {corner_list[i]}'
        # public-facing accessors
        assert all(a == b for a,b in zip(coord, accessor_list[i])), \
            f'{coord} != {accessor_list[i]}'


# ------------------------------------------------------------------------------
# Fixtures
# ------------------------------------------------------------------------------
@pytest.fixture(scope='class')
def robot():
    # Class scope cuts down on time spent init-ing
    # Used by any test function that needs a default robot instance
    sw = (0,0)
    nw = (0,1)
    ne = (1,1)
    se = (1,0)
    surf = alg.TestSurface(sw=sw, se=se, nw=nw, ne=ne)

    w = 2
    h = 3
    pos = (0,0)
    raft = alg.Raft(pos, w, h)
    # TODO: update with a default cmd sequence when that stuff is ready
    robot = alg.Robot(surf, raft, [])
    yield robot


# ------------------------------------------------------------------------------
# Default test class: reuses one default-init robot instance for maintainability
# ------------------------------------------------------------------------------
@ pytest.mark.usefixtures('robot')
class TestDefault(object):
    def test_Robot_init(self, robot):
        assert all(np.isinf(robot.pos_cmd))
        assert all(np.isinf(robot.home))


    @pytest.mark.parametrize('pos_cmd', [(.1,.1), (.9,.1), (.1,.9), (.5,.5), (.9,.9)])
    def test_Robot_bounds_check_good(self, robot, pos_cmd):
        robot.pos_cmd = pos_cmd


    @pytest.mark.parametrize('pos_cmd',
        [( 0.0, 0.0), ( 0.0, 1.0), ( 1.0, 1.0), ( 1.0, 0.0), # edges
         (-1.0,-1.0), (-1.0, 0.5), (-1.0, 1.5), ( 0.5, 1.5), # outside quadrants
         ( 1.5, 1.5), ( 1.5, 0.5), ( 1.5,-1.0), ( 0.5,-1.0)])
    def test_Robot_bounds_check_bad(self, robot, pos_cmd):
        with pytest.raises(ValueError):
            robot.pos_cmd = pos_cmd
    

    @pytest.mark.parametrize('length, expected', 
        [ # stepper has 200 steps per full rotation
            (-2. * np.pi * const.PULLEY_RADIUS, -200.),
            (-np.pi * const.PULLEY_RADIUS, -100.),
            (0., 0.),
            (np.pi * const.PULLEY_RADIUS, 100.)
        ]
    )
    def test_Robot_length_to_steps(self, robot, length, expected):
        eps = np.finfo(float).eps
        assert np.abs(expected - robot.length_to_steps(length)) < eps
        