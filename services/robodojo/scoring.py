"""Moved to services.scoring, shared by every benchmark; this alias keeps old imports working."""
import sys

from services import scoring

sys.modules[__name__] = scoring
