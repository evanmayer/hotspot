import os
import numpy as np
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
    executive = ex.Executive()
    yield executive


# -----------------------------------------------------------------------------
# Default test class: reuses one default-init motorkit instance
# -----------------------------------------------------------------------------
@ pytest.mark.usefixtures('executive')
class TestDefault(object):
    def test_Executive_init(self, executive):
        executive.run(os.path.join(const.TOPLEVEL_DIR, 'data', 'input', 'test.csv'))
