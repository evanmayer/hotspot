import multiprocessing as mp
import numpy as np
import pytest

from context import algorithm as alg
from context import constants as const


EPS = 1e-9
# hand calcs in this module are already a bear. make them easier by assuming an ideal spool.
const.RADIUS_M_PER_RAD = 0.
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
    w = .2
    h = .3
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


def multiple_robot_process_input(this_robot: alg.Robot, pos_delta: tuple):
    '''
    Helper function for exercising move cmds of any robot fixture
    '''
    this_robot.raft.position = (
        (this_robot.surf.ne[0] - this_robot.surf.nw[0]) / 2,
        (this_robot.surf.ne[1] - this_robot.surf.se[1]) / 2
    )
    pos_delta = np.array(pos_delta)
    pos_cmd = tuple(this_robot.raft.position + pos_delta)

    sw_d_before = np.linalg.norm(this_robot.surf.sw - this_robot.raft.sw)
    nw_d_before = np.linalg.norm(this_robot.surf.nw - this_robot.raft.nw)
    ne_d_before = np.linalg.norm(this_robot.surf.ne - this_robot.raft.ne)
    se_d_before = np.linalg.norm(this_robot.surf.se - this_robot.raft.se)

    sw_d_after = np.linalg.norm(this_robot.surf.sw - (this_robot.raft.sw + pos_delta))
    nw_d_after = np.linalg.norm(this_robot.surf.nw - (this_robot.raft.nw + pos_delta))
    ne_d_after = np.linalg.norm(this_robot.surf.ne - (this_robot.raft.ne + pos_delta))
    se_d_after = np.linalg.norm(this_robot.surf.se - (this_robot.raft.se + pos_delta))

    delta_sw = sw_d_after - sw_d_before
    delta_nw = nw_d_after - nw_d_before
    delta_ne = ne_d_after - ne_d_before
    delta_se = se_d_after - se_d_before

    length_per_rev = np.sqrt(const.DRUM_PITCH ** 2. + (np.pi * 2. * const.PULLEY_RADIUS) ** 2.)
    length_per_rad = length_per_rev / 2. / np.pi

    rad_sw = delta_sw / length_per_rad
    rad_nw = delta_nw / length_per_rad
    rad_ne = delta_ne / length_per_rad
    rad_se = delta_se / length_per_rad

    cmds = this_robot.process_input(pos_cmd)

    assert (abs(cmds['sw'] - rad_sw) < EPS), f"Incorrect value calculated for SW in movement toward N: {cmds['sw']}"
    assert (abs(cmds['nw'] - rad_nw) < EPS), f"Incorrect value calculated for NW in movement toward N: {cmds['nw']}"
    assert (abs(cmds['ne'] - rad_ne) < EPS), f"Incorrect value calculated for NE in movement toward N: {cmds['ne']}"
    assert (abs(cmds['se'] - rad_se) < EPS), f"Incorrect value calculated for SE in movement toward N: {cmds['se']}"

    return
# ------------------------------------------------------------------------------
# Fixtures
# ------------------------------------------------------------------------------
@pytest.fixture(scope='class')
def robot():
    '''
    Class scope cuts down on time spent init-ing
    Used by any test function that needs a default robot instance
    '''
    sw = (0,0)
    nw = (0,1)
    ne = (1,1)
    se = (1,0)
    surf = alg.TestSurface(sw=sw, se=se, nw=nw, ne=ne)

    w = .1
    h = .1
    pos = (
        (surf.ne[0] - surf.nw[0]) / 2,
        (surf.ne[1] - surf.se[1]) / 2
    )
    raft = alg.Raft(pos, w, h)
    robot = alg.Robot(surf, raft, mp.Queue())
    yield robot


@pytest.fixture(scope='class')
def robot_surf_rect():
    '''
    Class scope cuts down on time spent init-ing
    Used by any test function that needs a rectangular surf instance instead
    of a square one
    '''
    sw = (0,0)
    nw = (0,2)
    ne = (1,2)
    se = (1,0)
    surf = alg.TestSurface(sw=sw, se=se, nw=nw, ne=ne)

    w = .1
    h = .1
    pos = (
        (surf.ne[0] - surf.nw[0]) / 2,
        (surf.ne[1] - surf.se[1]) / 2
    )
    raft = alg.Raft(pos, w, h)
    robot = alg.Robot(surf, raft, mp.Queue())
    yield robot


@pytest.fixture(scope='class')
def robot_raft_rect():
    '''
    Class scope cuts down on time spent init-ing
    Used by any test function that needs a rectangular raft instance instead
    of a square one
    '''
    sw = (0,0)
    nw = (0,1)
    ne = (1,1)
    se = (1,0)
    surf = alg.TestSurface(sw=sw, se=se, nw=nw, ne=ne)

    w = .2
    h = .1
    pos = (
        (surf.ne[0] - surf.nw[0]) / 2,
        (surf.ne[1] - surf.se[1]) / 2
    )
    raft = alg.Raft(pos, w, h)
    robot = alg.Robot(surf, raft, mp.Queue())
    yield robot


# ------------------------------------------------------------------------------
# Test class: reuses robot instances for maintainability/speed
# ------------------------------------------------------------------------------
@pytest.mark.usefixtures('robot', 'robot_surf_rect', 'robot_raft_rect')
class TestDefault(object):
    def test_Robot_init(self, robot):
        assert all(np.isinf(robot.pos_cmd))
        assert all(np.isinf(robot.home))


    @pytest.mark.parametrize('pos_cmd', [(.1,.1), (.9,.1), (.1,.9), (.5,.5), (.9,.9)])
    def test_default_Robot_bounds_check_good(self, robot, pos_cmd):
        robot.pos_cmd = pos_cmd


    @pytest.mark.parametrize('pos_cmd', [(.1,.1), (.9,.1), (.1,1.9), (.5,.5), (.9,1.9)])
    def test_surf_rect_Robot_bounds_check_good(self, robot_surf_rect, pos_cmd):
        robot_surf_rect.pos_cmd = pos_cmd


    @pytest.mark.parametrize('pos_cmd', [(.1,.1), (.9,.1), (.1,.9), (.5,.5), (.9,.9)])
    def test_raft_rect_Robot_bounds_check_good(self, robot_raft_rect, pos_cmd):
        robot_raft_rect.pos_cmd = pos_cmd


    @pytest.mark.parametrize('pos_cmd',
        [( 0.0, 0.0), ( 0.0, 1.0), ( 1.0, 1.0), ( 1.0, 0.0), # edges
         (-1.0,-1.0), (-1.0, 0.5), (-1.0, 1.5), ( 0.5, 1.5), # outside quadrants
         ( 1.5, 1.5), ( 1.5, 0.5), ( 1.5,-1.0), ( 0.5,-1.0)])
    def test_default_Robot_bounds_check_bad(self, robot, pos_cmd):
        with pytest.raises(ValueError):
            robot.pos_cmd = pos_cmd


    @pytest.mark.parametrize('pos_cmd',
        [( 0.0, 0.0), ( 0.0, 2.0), ( 1.0, 2.0), ( 1.0, 0.0), # edges
         (-1.0,-1.0), (-1.0, 0.5), (-1.0, 1.5), ( 0.5, 2.5), # outside quadrants
         ( 1.5, 1.5), ( 1.5, 0.5), ( 1.5,-1.0), ( 0.5,-1.0)])
    def test_surf_rect_Robot_bounds_check_bad(self, robot_surf_rect, pos_cmd):
        with pytest.raises(ValueError):
            robot_surf_rect.pos_cmd = pos_cmd


    @pytest.mark.parametrize('pos_cmd',
        [( 0.0, 0.0), ( 0.0, 1.0), ( 1.0, 1.0), ( 1.0, 0.0), # edges
         (-1.0,-1.0), (-1.0, 0.5), (-1.0, 1.5), ( 0.5, 1.5), # outside quadrants
         ( 1.5, 1.5), ( 1.5, 0.5), ( 1.5,-1.0), ( 0.5,-1.0)])
    def test_raft_rect_Robot_bounds_check_bad(self, robot_raft_rect, pos_cmd):
        with pytest.raises(ValueError):
            robot_raft_rect.pos_cmd = pos_cmd


    all_pos_cmds = [
        (-0.1, -0.1), # to SW
        (-0.1,  0.1), # to NW
        ( 0.1,  0.1), # to NE
        ( 0.1, -0.1), # to SE
        ( 0.0,  0.1), # to N
        ( 0.0,  0.2),
        ( 0.0, -0.1), # to S
        ( 0.0, -0.2),
        ( 0.1,  0.0), # to E
        ( 0.2,  0.0),
        (-0.1,  0.0), # to W
        (-0.2,  0.0),
    ]

    @pytest.mark.parametrize('pos_delta', all_pos_cmds)
    def test_default_Robot_process_input(self, robot, pos_delta: tuple):
        multiple_robot_process_input(robot, pos_delta)


    @pytest.mark.parametrize('pos_delta', all_pos_cmds)
    def test_surf_rect_Robot_process_input(self, robot_surf_rect, pos_delta: tuple):
        multiple_robot_process_input(robot_surf_rect, pos_delta)


    @pytest.mark.parametrize('pos_delta', all_pos_cmds)
    def test_raft_rect_Robot_process_input(self, robot_raft_rect, pos_delta: tuple):
        multiple_robot_process_input(robot_raft_rect, pos_delta)
