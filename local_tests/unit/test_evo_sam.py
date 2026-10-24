import base64
import copy
import hashlib
import json
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np
import pytest

from auto_research_agent.api.cloth_trial import TrialContext, ReplayContext
from auto_research_agent.api.runtime import Context
from auto_research_agent.api.sam import Sam
from test_evo_v3_backend import frontend_socket, service


def result():
    ok, data = cv2.imencode(".png", np.zeros((32, 48), np.uint8))
    assert ok
    artifact = dict(
        mime_type="image/png",
        sha256=hashlib.sha256(data).hexdigest(),
        data=base64.b64encode(data).decode(),
    )
    return dict(
        job_id="sam-test",
        state="succeeded",
        frame_count=1,
        result={"profile": "image-854x480"},
        frame=dict(
            observation_id="capture",
            capture_us=42,
            camera="base",
            source_sha256="a" * 64,
            transform=dict(
                width=48, height=32, coordinate_frame="captured_camera_pixels"
            ),
            objects=[
                dict(
                    object_id=0,
                    score=0.9,
                    box_xyxy=[0, 0, 1, 1],
                    mask=copy.deepcopy(artifact),
                )
            ],
            overlay=artifact,
        ),
    )


def test_python_and_interactive_sam_use_same_frontend_and_save_artifacts(tmp_path):
    def handler(r):
        if r["method"] == "sam_segment":
            return dict(job_id="sam-test", state="running")
        value = result()
        if "frame_index" not in r["params"]:
            value.pop("frame")
        return value

    with service(tmp_path, handler) as (client, received), frontend_socket(
        client, tmp_path
    ) as socket:
        with Context(str(socket)) as context:
            sam = Sam(context)
            accepted = context.value(
                "evo_sam_segment", observation_id="capture", camera="base"
            )
            value = sam.wait(accepted)
            frame = value["frames"][0]
            assert frame["observation_id"] == "capture" and frame["capture_us"] == 42
            for artifact in [frame["overlay"], frame["objects"][0]["mask"]]:
                data = (tmp_path / "agent-workspace" / artifact["path"]).read_bytes()
                assert hashlib.sha256(data).hexdigest() == artifact["sha256"]
                assert "data" not in artifact
            assert not client.session_id  # SAM has no actuator authority.
            assert [r["method"] for r in received] == [
                "sam_segment",
                "sam_result",
                "sam_result",
            ]


@pytest.mark.parametrize("tool", ["evo_cancel", "evo_hold", "evo_sam_cancel"])
def test_cancel_and_hold_cross_frontend_while_sam_result_is_blocked(tmp_path, tool):
    entered, release = threading.Event(), threading.Event()

    def handler(r):
        if r["method"] == "sam_result":
            entered.set()
            assert release.wait(3)
            return dict(job_id="sam-test", state="cancelled")
        return dict(state="cancelled" if r["method"] == "sam_cancel" else "stopping")

    with service(tmp_path, handler) as (client, received), frontend_socket(
        client, tmp_path
    ) as socket:
        client._ensure_session()
        with Context(str(socket)) as ordinary, Context(
            str(socket)
        ) as urgent, ThreadPoolExecutor() as pool:
            pending = pool.submit(ordinary.value, "evo_sam_result", job_id="sam-test")
            try:
                assert entered.wait(1)
                urgent._socket.settimeout(1)
                args = (
                    {"action_id": "a"}
                    if tool == "evo_cancel"
                    else ({"job_id": "sam-test"} if tool == "evo_sam_cancel" else {})
                )
                urgent.value(tool, **args)
                assert not pending.done()
                assert any(r["method"] == tool.removeprefix("evo_") for r in received)
                assert not client._stop.is_set()
            finally:
                release.set()
            pending.result(timeout=2)


@pytest.mark.parametrize(
    "change",
    [
        lambda r: r["frame"]["overlay"].update(sha256="f" * 64),
        lambda r: r["frame"]["transform"].update(width=49),
        lambda r: r.update(job_id="another-job"),
    ],
)
def test_invalid_artifacts_and_job_identity_fail_closed(tmp_path, change):
    value = result()
    change(value)
    with service(tmp_path, lambda _: value) as (client, _):
        with pytest.raises(ValueError):
            client.call("evo_sam_result", dict(job_id="sam-test", frame_index=0))


@pytest.mark.parametrize("index", [True, 0.5, -1, 64])
def test_frame_index_requires_a_bounded_integer(tmp_path, index):
    with service(tmp_path, lambda _: {}) as (client, received):
        with pytest.raises(ValueError):
            client.call("evo_sam_result", dict(job_id="j", frame_index=index))
        assert not received


def test_trial_records_sam_inputs_outputs_and_replay_checks_both(tmp_path):
    source = tmp_path / "source.jpg"
    source.write_bytes(b"jpeg")
    mask = tmp_path / "mask.png"
    mask.write_bytes(b"png")

    class Recorded:
        def value(self, tool, **args):
            if tool == "evo_history":
                return dict(
                    observation_id="capture", cameras={"base": {"path": "source.jpg"}}
                )
            if tool == "evo_sam_segment":
                return dict(job_id="sam-test", state="running")
            value = result()
            for a in [value["frame"]["overlay"], value["frame"]["objects"][0]["mask"]]:
                a.pop("data")
                a.update(path="mask.png", sha256=hashlib.sha256(b"png").hexdigest())
            return value

    trial = TrialContext(Recorded(), tmp_path / "trial", workspace=tmp_path)
    trial.value("evo_sam_segment", observation_id="capture", camera="base")
    trial.value("evo_sam_result", job_id="sam-test", frame_index=0)
    trial.close()
    replay = ReplayContext(tmp_path / "trial")
    replay.value("evo_sam_segment", observation_id="capture", camera="base")
    page = replay.value("evo_sam_result", job_id="sam-test", frame_index=0)
    assert Path(page["frame"]["overlay"]["path"]).read_bytes() == b"png"
    events = [
        json.loads(line)
        for line in (tmp_path / "trial/events.jsonl").read_text().splitlines()
    ]
    path = events[1]["model_inputs"][0]["cameras"]["base"]["path"]
    (tmp_path / "trial" / path).write_bytes(b"changed")
    with pytest.raises(ValueError, match="changed"):
        ReplayContext(tmp_path / "trial").value(
            "evo_sam_segment", observation_id="capture", camera="base"
        )


def test_wait_timeout_cancels_job():
    calls = []

    class Waiting:
        def value(self, tool, **kw):
            calls.append(tool)
            return dict(state="running")

    with pytest.raises(TimeoutError):
        Sam(Waiting()).wait({"job_id": "job"}, timeout_s=0.001)
    assert calls[-1] == "evo_sam_cancel"


def test_shared_sam_protocol_requests():
    fixtures = json.loads(Path("services/evo/protocol_fixtures.json").read_text())
    from services.evo_v3.backend import BY_NAME, validate_value

    for request in fixtures["sam_requests"]:
        params = {k: v for k, v in request.items() if k not in ("operation", "frames")}
        params["camera"] = request["frames"][0]["camera"]
        if request["operation"] == "segment":
            params["observation_id"] = request["frames"][0]["observation_id"]
        else:
            params["observation_ids"] = [f["observation_id"] for f in request["frames"]]
        validate_value(
            params, BY_NAME["evo_sam_" + request["operation"]]["inputSchema"]
        )


@pytest.mark.parametrize("blocked_at", ["status", "audit"])
def test_sam_cancel_bypasses_priority_status_and_audit(tmp_path, blocked_at):
    entered, release = threading.Event(), threading.Event()

    def block():
        entered.set()
        assert release.wait(3)

    def audit(event):
        if blocked_at == "audit":
            block()

    def handler(r):
        if r["method"] == "status" and blocked_at == "status":
            block()
        return dict(job_id="sam-test", state="cancelled")

    with service(tmp_path, handler) as (client, received), frontend_socket(
        client, tmp_path, audit=audit
    ) as path:
        with Context(str(path)) as ordinary, Context(
            str(path)
        ) as urgent, ThreadPoolExecutor() as pool:
            pending = pool.submit(ordinary.value, "evo_status")
            try:
                assert entered.wait(1)
                urgent._socket.settimeout(1)
                assert (
                    Sam(urgent).cancel({"job_id": "sam-test"})["state"] == "cancelled"
                )
                assert not pending.done()
                assert any(r["method"] == "sam_cancel" for r in received)
                assert not client.session_id and not client._stop.is_set()
            finally:
                release.set()
            pending.result(timeout=2)


@pytest.mark.parametrize("expire_at", [None, 0, 1])
def test_wait_deadline_covers_status_and_frame_retrieval_through_frontend(
    tmp_path, monkeypatch, expire_at
):
    from types import SimpleNamespace
    from auto_research_agent.api import sam as sam_module

    clock = [0.0]
    monkeypatch.setattr(sam_module, "time", SimpleNamespace(monotonic=lambda: clock[0]))

    def handler(r):
        if r["method"] == "sam_cancel":
            return dict(job_id="sam-test", state="succeeded")
        index = r["params"].get("frame_index")
        if index == expire_at:
            clock[0] = 2.0
        value = result()
        value["frame_count"] = 2
        if index is None:
            value.pop("frame")
        return value

    with service(tmp_path, handler) as (client, received), frontend_socket(
        client, tmp_path
    ) as path:
        with Context(str(path)) as ctx:
            with pytest.raises(TimeoutError, match="cancellation requested"):
                Sam(ctx).wait({"job_id": "sam-test"}, timeout_s=1)
        assert received[-1]["method"] == "sam_cancel"
        assert sum(r["method"] == "sam_result" for r in received) == (
            1 if expire_at is None else expire_at + 2
        )


def test_wait_reports_timeout_when_cancellation_fails(monkeypatch):
    from types import SimpleNamespace
    from auto_research_agent.api import sam as sam_module

    clock = [0.0]
    monkeypatch.setattr(sam_module, "time", SimpleNamespace(monotonic=lambda: clock[0]))

    class Waiting:
        def value(self, tool, **kw):
            if tool == "evo_sam_cancel":
                raise ConnectionError("socket closed")
            clock[0] = 2.0
            return dict(state="running")

    with pytest.raises(TimeoutError, match="cancellation failed") as error:
        Sam(Waiting()).wait({"job_id": "job"}, timeout_s=1)
    assert isinstance(error.value.__cause__, ConnectionError)


@pytest.mark.parametrize("tool", ["evo_cancel", "evo_hold"])
def test_blocked_sam_cancellations_do_not_consume_robot_stop_slots(tmp_path, tool):
    from contextlib import ExitStack

    entered, release = threading.Event(), threading.Event()
    count = [0]
    lock = threading.Lock()

    def handler(r):
        if r["method"] == "sam_cancel":
            with lock:
                count[0] += 1
                if count[0] == 4:
                    entered.set()
            assert release.wait(3)
            return dict(job_id="sam-test", state="cancelled")
        return dict(state="stopping")

    with service(tmp_path, handler) as (client, received), frontend_socket(
        client, tmp_path
    ) as path:
        client._ensure_session()
        with ExitStack() as stack, ThreadPoolExecutor(max_workers=4) as pool:
            contexts = [stack.enter_context(Context(str(path))) for _ in range(5)]
            pending = [
                pool.submit(Sam(ctx).cancel, {"job_id": "sam-test"})
                for ctx in contexts[:4]
            ]
            try:
                assert entered.wait(1)
                urgent = contexts[-1]
                urgent._socket.settimeout(1)
                args = {"action_id": "a"} if tool == "evo_cancel" else {}
                assert urgent.value(tool, **args)["state"] == "stopping"
                assert all(not job.done() for job in pending)
                assert not client._stop.is_set()
            finally:
                release.set()
            for job in pending:
                job.result(timeout=2)
