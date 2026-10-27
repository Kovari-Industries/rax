from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from services.evo.journey import EvoJourneyClient


def test_start_returns_native_handle_without_reimplementing_stages():
    native = Mock()
    handle = SimpleNamespace(done=False, outcome=None, cancel=Mock())
    native.send.return_value = handle
    feedback = Mock()
    client = EvoJourneyClient(native)
    assert client.start("2", feedback=feedback) is handle
    native.send.assert_called_once_with("elevator_journey", "2", feedback)
    handle.cancel()
    handle.cancel.assert_called_once_with()


def test_run_preserves_cancellation_feedback_and_native_outcome():
    native = Mock()
    cancelled, feedback = Mock(), Mock()
    client = EvoJourneyClient(native)
    assert client.run("4", cancelled=cancelled, feedback=feedback) is native.run.return_value
    native.run.assert_called_once_with("elevator_journey", "4", feedback=feedback,
                                      timeout_sec=360., cancelled=cancelled)


@pytest.mark.parametrize("destination", [None, "", " 2", "2 ", 2, {"floor": "2"}])
def test_invalid_request_cannot_dispatch(destination):
    native = Mock()
    client = EvoJourneyClient(native)
    with pytest.raises(ValueError):
        client.start(destination)
    with pytest.raises(ValueError):
        client.run(destination)
    assert native.mock_calls == []
