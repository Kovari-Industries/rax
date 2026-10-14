"""Moved to services.simulator.protocol, shared by every benchmark; this alias keeps old imports working."""
import sys

from services.simulator import protocol

sys.modules[__name__] = protocol
