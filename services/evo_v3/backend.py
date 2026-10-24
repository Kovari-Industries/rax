"""Evo v3 robot registration using the shared Evo protocol."""
from services.evo.backend import (
    BY_NAME, CLIENT_TOOLS, DEFAULTS, ELEVATOR_TOOLS, MAX_RESPONSE, MOTION, TOOLS,
    EvoBackend, EvoRefused, validate_value,
)


class EvoV3Backend(EvoBackend):
    """Rax adapter selected by the evo-v3 robot profile."""
