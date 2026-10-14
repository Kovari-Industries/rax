"""Moved to services.simulator.rpc, shared by every benchmark; this alias keeps old imports working."""
import sys

from services.simulator import rpc

sys.modules[__name__] = rpc
