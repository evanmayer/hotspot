import multiprocessing as mp
import numpy as np
import os
import pytest

from context import executive as ex
from context import hardware as hw
from context import constants as const


@pytest.mark.skipif(hw.try_open(hw.MODEL_NAME, hw.MODE, retry=False) == -1,
    reason='LabJack not found via USB. Skipping test.')
class TestDefault(object):
    def test_Executive_init(self):
        geometry_file = os.path.join(const.TOPLEVEL_DIR, 'data', 'input', 'geometry', 'test_surface.csv')
        tm_queue = mp.Queue(const.MAX_QLEN)
        executive = ex.Executive(geometry_file)