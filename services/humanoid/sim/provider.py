"""semantic40 extension of the pinned Arena SONIC action provider; no upstream edits.

The agent's 50 Hz semantic40 frames replace only the high-level LeRobot VLA model: upstream's
converter, the frozen SONIC encoder/decoder, PD gains, action scales and hand mapping are kept.
Upstream catches model errors and returns a default pose; that fallback is refused here.
"""
import numpy as np


class CheckedSession:
    """ONNX session wrapper that counts calls and rejects nonfinite outputs."""

    def __init__(self, session):
        self.session = session
        self.calls, self.failed = 0, False

    def get_inputs(self):
        return self.session.get_inputs()

    def run(self, outputs, inputs):
        self.failed = False
        try:
            result = self.session.run(outputs, inputs)
            if any(not np.isfinite(value).all() for value in result):
                raise ValueError("Nonfinite frozen model output")
            self.calls += 1
            return result
        except Exception:
            self.failed = True
            raise


def create_provider(env, args):
    from action_provider.action_provider_sonic import (
        SonicActionProvider, quat_mul_wxyz, rot6d_to_quat_wxyz_with_layout, quat_delta_deg_wxyz,
    )

    class Provider(SonicActionProvider):
        def _make_session(self, path):
            # Explicit CUDA execution avoids unavailable TensorRT and host-wide pools.
            import onnxruntime as ort
            options = ort.SessionOptions()
            options.intra_op_num_threads = 2
            options.inter_op_num_threads = 1
            device = self._resolve_ort_device_id()
            if device is None or "CUDAExecutionProvider" not in ort.get_available_providers():
                raise RuntimeError("Frozen controller requires CUDA ONNX execution")
            session = ort.InferenceSession(path, sess_options=options,
                providers=[("CUDAExecutionProvider", {"device_id": device}), "CPUExecutionProvider"])
            if "CUDAExecutionProvider" not in session.get_providers():
                raise RuntimeError("CUDA ONNX provider failed to initialize")
            return session

        def _setup_lerobot_vla(self, args_cli):
            pass  # The agent's frames replace the high-level model only.

        def _setup_hand_dds(self, args_cli):
            self._dex3_dds = None  # Dex3 targets come only from the current command.

        def _recording_enabled_for_current_mode(self):
            return False  # The session writes its own evidence.

        def _should_refresh_lerobot_visuals_next_step(self):
            return False  # The session renders explicitly without stepping physics.

        def _infer_lerobot_semantic_action(self):
            if self.command is None:
                raise RuntimeError("No accepted agent command")
            return np.asarray(self.command[self.segment_step], dtype=np.float32)

        def _apply_lerobot_semantic_action(self, action):
            super()._apply_lerobot_semantic_action(action)
            runtime = self._lerobot_vla_runtime
            requested = quat_mul_wxyz(runtime._episode_ref_to_world_heading_quat_wxyz,
                rot6d_to_quat_wxyz_with_layout(action[3:9], layout="row").reshape(4))
            self._latest_root_projection_deg = quat_delta_deg_wxyz(
                requested, runtime._prev_target_root_quat_wxyz)

        def _run_gear_sonic(self):
            before = self._decoder.calls
            self._encoder.failed = self._decoder.failed = False
            target = super()._run_gear_sonic()
            if self._encoder.failed or self._decoder.failed or self._decoder.calls != before+1:
                raise RuntimeError("SONIC inference failed; default-pose fallback rejected")
            return target

        def reset_reference(self):
            self.on_env_reset()
            self._should_start_recording_on_first_call = False
            self.command = None

        def execute_tick(self):
            before = self._decoder.calls
            result = self.get_action(env)
            if result is None or self._decoder.calls != before+1:
                raise RuntimeError("Arena control tick failed")
            self.segment_step += 1

    provider = Provider(env, args)
    provider.command, provider.segment_step = None, 0
    if provider._encoder.get_inputs()[0].shape[-1] != 1762 or provider._decoder.get_inputs()[0].shape[-1] != 994:
        provider.cleanup()
        raise ValueError("The Arena runtime requires the released SONIC encoder (1762) and decoder (994)")
    provider._encoder = CheckedSession(provider._encoder)
    provider._decoder = CheckedSession(provider._decoder)
    provider._should_start_recording_on_first_call = False
    return provider
