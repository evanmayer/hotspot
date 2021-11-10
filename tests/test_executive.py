import multiprocessing as mp
import numpy as np
import os
import pytest

from context import executive as ex
from context import constants as const


# ------------------------------------------------------------------------------
# Fixtures
# ------------------------------------------------------------------------------
@pytest.fixture(scope='class')
def executive():
    # Class scope cuts down on time spent init-ing
    # Used by any test function that needs a default motorkit instance
    geometry_file = os.path.join(const.TOPLEVEL_DIR, 'data', 'input', 'geometry', 'test_surface.csv')
    tm_queue = mp.Queue(const.MAX_QLEN)
    executive = ex.Executive(geometry_file)
    yield executive


# -----------------------------------------------------------------------------
# Default test class: reuses one default-init motorkit instance
# -----------------------------------------------------------------------------
@ pytest.mark.usefixtures('executive')
class TestDefault(object):
    def test_Executive_init(self, executive):
        pass
