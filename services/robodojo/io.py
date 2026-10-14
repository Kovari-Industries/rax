"""Moved to services.simulator.io, shared by every benchmark; this alias keeps old imports working."""
import sys

from services.simulator import io

sys.modules[__name__] = io
