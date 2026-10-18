"""Isaac Sim 6.1 physics sync for upstream's seeded prim-pose resets (outside the upstream sources).

Upstream ``common_env_objects._write_prim_world_pose`` writes each sampled pose into an
``xformOp:transform`` appended after the prim's authored translate/orient/scale ops, so USD
composes it with them. On the DoubleDesk basket and the navigation sign (authored in
centimetres, scale 0.01) the sampled translation therefore shrinks a hundredfold while the
sampled yaw applies in full; the PnP shelf and the OpenDoor container (scale 1) land at their
authored position plus the sample. This is upstream's realized official layout: the official
expert recordings show the same poses, every success predicate reads the same composed pose,
and it is independent of the Isaac Sim version. The backend keeps it exactly, following the
official benchmark.

What does differ in Isaac Sim 6.1 is physics: PhysX builds a static collider when it parses
the stage and does not follow later USD edits. When upstream's reset has written a static
collider, the backend therefore restarts physics and replays the identical seeded reset, so
the colliders sit where the prims are rendered. Simulated bodies keep their PhysX state; the
OpenDoor door follows its composed container through its own adapter (door_reset.py).
"""
import math

TRS = ("xformOp:translate", "xformOp:orient", "xformOp:scale")
APPENDED = "xformOp:transform"


def _quat_wxyz(rotation):
    quat = rotation.GetQuat() if hasattr(rotation, "GetQuat") else rotation
    return [float(quat.GetReal()), *[float(v) for v in quat.GetImaginary()]]


def _yaw_deg(wxyz):
    w, x, y, z = wxyz
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _authored_local(values, Gf):
    """The prim's authored translate/orient/scale transform, without the appended matrix."""
    translate, orient, scale = values
    transform = Gf.Transform()
    transform.SetScale(Gf.Vec3d(*scale))
    transform.SetRotation(Gf.Rotation(Gf.Quatd(float(orient.GetReal()), Gf.Vec3d(*orient.GetImaginary()))))
    transform.SetTranslation(Gf.Vec3d(*translate))
    return transform.GetMatrix()


def authored_world_pose(path):
    """World position and wxyz rotation of a prim's authored TRS ops, ignoring an appended op."""
    import omni.usd
    from pxr import Gf, Usd, UsdGeom
    prim = omni.usd.get_context().get_stage().GetPrimAtPath(path)
    ops = {op.GetOpName(): op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
    parent = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim.GetParent())
    if all(name in ops for name in TRS):
        world = _authored_local(tuple(ops[name].Get() for name in TRS), Gf) * parent
    else:
        world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return ([float(v) for v in world.ExtractTranslation()],
            _quat_wxyz(world.RemoveScaleShear().ExtractRotationQuat()))


def world_pose(prim):
    """Composed world position and scale-free wxyz rotation of a prim (operator diagnostics)."""
    from pxr import Usd, UsdGeom
    world = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    return ([float(v) for v in world.ExtractTranslation()],
            _quat_wxyz(world.RemoveScaleShear().ExtractRotationQuat()))


def _static_collider(prim, Usd, UsdPhysics):
    """No rigid body or articulation in the subtree: PhysX built a static actor at parse time."""
    return not any(node.HasAPI(UsdPhysics.RigidBodyAPI) or node.HasAPI(UsdPhysics.ArticulationRootAPI)
                   for node in Usd.PrimRange(prim))


def upstream_prim_resets(env, cfg):
    """Report the poses upstream's reset wrote into USD; nothing is rewritten.

    Each entry names the record, its prim, whether it is a static collider, whether upstream
    wrote an appended transform this episode, and the composed world pose that results.
    """
    specs = [spec for spec in (getattr(cfg, "deterministic_object_resets", None) or [])
             if isinstance(spec, dict) and spec.get("prim_paths") and spec.get("pose_range")]
    episode_seed = getattr(cfg, "_current_episode_object_seed", None)
    if not specs or episode_seed is None:
        return []
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics
    import common_env_objects as objects

    stage = omni.usd.get_context().get_stage()
    report = []
    for spec in objects.get_recordable_env_object_specs(cfg):
        name, pose_range = spec["record_name"], spec.get("pose_range") or {}
        if not spec.get("prim_paths") or not pose_range:
            continue
        if not objects._should_prefer_prim_pose_write(spec):
            key = objects.resolve_env_object_scene_key(env, cfg, name)
            if key is not None and hasattr(env.scene[key], "data"):
                continue  # upstream wrote this asset's physics root state directly
        fallback = bool(spec.get("allow_prroot_fallback", True))
        for env_index in range(env.num_envs):
            prim = None
            for template in spec["prim_paths"]:
                for path in objects._candidate_prim_paths(str(template), env_index, allow_prroot_fallback=fallback):
                    candidate = stage.GetPrimAtPath(path)
                    if candidate and candidate.IsValid() and candidate.IsActive() and UsdGeom.Xformable(candidate):
                        prim = candidate
                        break
                if prim is not None:
                    break
            if prim is None:
                continue
            order = [op.GetOpName() for op in UsdGeom.Xformable(prim).GetOrderedXformOps()]
            position, orientation = world_pose(prim)
            report.append({"record": name, "prim": str(prim.GetPath()),
                           "static_collider": _static_collider(prim, Usd, UsdPhysics),
                           "upstream_written": APPENDED in order, "xform_op_order": order,
                           "episode_object_seed": int(episode_seed),
                           "realized_position_m": position, "realized_orientation_wxyz": orientation,
                           "realized_yaw_deg": _yaw_deg(orientation)})
    return report


def measure_static_footprints(report, half_width=0.45, spacing=0.05):
    """Locate each written static collider in PhysX with downward rays around its rendered pose.

    Records the centre of the ray hits on the prim's own colliders (a hollow object's open
    interior lets rays through, so this estimates its footprint centre) and fails if PhysX
    has no collider of that prim near where it is rendered.
    """
    import carb
    from omni.physx import get_physx_scene_query_interface
    query = get_physx_scene_query_interface()
    steps = int(round(2 * half_width / spacing)) + 1
    for item in report:
        if not (item.get("upstream_written") and item.get("static_collider")):
            continue
        x0, y0, _ = item["realized_position_m"]
        prefix, hits = item["prim"] + "/", []
        for i in range(steps):
            for j in range(steps):
                x, y = x0 - half_width + i * spacing, y0 - half_width + j * spacing
                hit = query.raycast_closest(carb.Float3(x, y, 3.0), carb.Float3(0.0, 0.0, -1.0), 6.0)
                if hit.get("hit") and str(hit.get("collision", "")).startswith(prefix):
                    hits.append((x, y))
        if not hits:
            raise RuntimeError(f"PhysX has no collider of {item['prim']} near its rendered pose")
        center = [sum(h[0] for h in hits) / len(hits), sum(h[1] for h in hits) / len(hits)]
        item["physics_footprint"] = {"ray_hits": len(hits), "center_xy_m": center,
                                     "offset_from_rendered_m": math.hypot(center[0] - x0, center[1] - y0)}
    return report
