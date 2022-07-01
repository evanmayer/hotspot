import multiprocessing as mp
import os

from context import executive as ex
from context import constants as const


class TestDefault(object):
    def test_Executive_init(self):
        geometry_file = os.path.join(const.TOPLEVEL_DIR, 'data', 'input', 'geometry', 'test_surface.csv')
        tm_queue = mp.Queue(const.MAX_QLEN)
        executive = ex.Executive(geometry_file)