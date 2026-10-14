"""BEHAVIOR-1K (OmniGibson, R1Pro) simulator backend for the auto-research harness.

Host-side modules import only the standard library, msgpack and numpy. OmniGibson is
imported solely by ``server.py`` and ``session.py``, which run in the separate
``runtime/envs/behavior`` interpreter.
"""
