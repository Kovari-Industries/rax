"""Service-side wrapper around Evo's existing PrimitiveClient.

Evo owns every journey decision and control limit. The runtime supplies its
existing client; constructing this wrapper does not open a robot or arm it.
This is not advertised as an agent tool until a qualified native provider exists.
"""


class EvoJourneyClient:
    def __init__(self, primitive_client):
        self.client = primitive_client

    @staticmethod
    def _destination(destination):
        if not isinstance(destination, str) or not destination or destination != destination.strip():
            raise ValueError("Destination must be an exact nonempty floor label")

    def start(self, destination, *, feedback=None):
        self._destination(destination)
        # Return Evo's own handle: callers use its done/outcome/cancel interface.
        return self.client.send("elevator_journey", destination, feedback)

    def run(self, destination, *, cancelled=lambda: False, feedback=None):
        self._destination(destination)
        return self.client.run("elevator_journey", destination, feedback=feedback,
                               timeout_sec=360., cancelled=cancelled)
