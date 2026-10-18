"""Reset-only support for Arena's unmanaged inference-profile door articulation.

The pinned inference profile spawns the door as AssetBaseCfg. IsaacLab's scene
reset therefore resets the robot, but does not restore the physical door joints.
This adapter preserves the initial physical state and applies it only at episode
boundaries. It never runs during a robot action and never changes drive settings.
"""
import numpy as np

from .quat import inverse, multiply, rotate


class DoorEpisodeReset:
    def __init__(self, env, cfg):
        import torch
        if env.num_envs != 1:
            raise ValueError("Door reset adapter currently requires one environment")
        handles = cfg._get_open_door_tensor_proxy_handles(env)
        if not handles or handles["leaf_id"] is None:
            raise RuntimeError("Cannot establish physical door reset state")
        self.view = handles["view"]
        self.sim = env.sim
        self.indices = torch.arange(self.view.count, device=env.device, dtype=torch.int32)
        self.joint_positions = self.view.get_dof_positions().clone()
        self.joint_velocities = torch.zeros_like(self.joint_positions)
        self.root_velocities = torch.zeros_like(self.view.get_root_velocities())
        self.initial_physical_root = self.view.get_root_transforms().clone()
        # Calibrate against the authored container (PhysX parsed the door from it): the
        # physical root then follows each seeded container pose, including the first one. The
        # container keeps upstream's composed pose (authored position plus the sample).
        from .prim_reset import authored_world_pose
        container_p, container_q = (np.asarray(value, dtype=np.float64)
                                    for value in authored_world_pose("/World/envs/env_0/Door"))
        initial = self.initial_physical_root[0].cpu().numpy()
        initial_q = np.asarray([initial[6], *initial[3:6]])
        self.root_offset_p = rotate(inverse(container_q), initial[:3] - container_p)
        self.root_offset_q = multiply(inverse(container_q), initial_q)

    @staticmethod
    def container_pose():
        import omni.usd
        from pxr import UsdGeom
        prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/envs/env_0/Door")
        transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        q = transform.RemoveScaleShear().ExtractRotationQuat()
        return (np.asarray(transform.ExtractTranslation(), dtype=np.float64),
                np.asarray([q.GetReal(), *q.GetImaginary()], dtype=np.float64))

    def restore(self):
        import torch
        container_p, container_q = self.container_pose()
        p = container_p + rotate(container_q, self.root_offset_p)
        q = multiply(container_q, self.root_offset_q)
        root = self.view.get_root_transforms().clone()
        root[0] = torch.as_tensor([*p, *q[1:], q[0]], device=root.device, dtype=root.dtype)
        self.view.set_root_transforms(root, indices=self.indices)
        self.view.set_root_velocities(self.root_velocities, indices=self.indices)
        self.view.set_dof_positions(self.joint_positions, indices=self.indices)
        self.view.set_dof_velocities(self.joint_velocities, indices=self.indices)
        # Isaac's writes are queued; update kinematics without a physics step
        # before reading back or capturing latch references.
        before = (float(self.sim.current_time), int(self.sim.current_time_step_index))
        self.sim.forward()
        if before != (float(self.sim.current_time), int(self.sim.current_time_step_index)):
            raise RuntimeError("Door reset synchronization advanced physics")
        actual_root = self.view.get_root_transforms()
        p_error = float(torch.linalg.vector_norm(actual_root[:, :3] - root[:, :3]))
        q_error = float(torch.minimum(torch.linalg.vector_norm(actual_root[:, 3:] - root[:, 3:]),
                                     torch.linalg.vector_norm(actual_root[:, 3:] + root[:, 3:])))
        self.root_position_residual_m = p_error
        # Bound root placement to 1 mm; same-layout repeatability is separately
        # checked at 1e-5 m. PhysX's fixed-root readback is not bit-exact.
        if p_error > 0.001 or q_error > 1e-5:
            raise RuntimeError(f"Physical door root reset mismatch: position={p_error}m quaternion={q_error}")
        actual = self.view.get_dof_positions()
        if not bool(torch.allclose(actual, self.joint_positions, atol=1e-6, rtol=0)):
            raise RuntimeError("Physical door joint reset did not take effect")

    def fingerprint_state(self):
        """Owner-only state for layout identity; never an agent observation."""
        return {"root_pose_world_m_xyzw": self.view.get_root_transforms()[0].tolist(),
                "joint_positions_rad": self.view.get_dof_positions()[0].tolist(),
                "joint_velocities_rad_s": self.view.get_dof_velocities()[0].tolist()}
