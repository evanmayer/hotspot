import os
import sys
# https://docs.python-guide.org/writing/structure/
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import hotspot.algorithm as algorithm
import hotspot.constants as constants
import hotspot.executive as executive
import hotspot.hardware as hardware
import hotspot.hw_context as hw_context
