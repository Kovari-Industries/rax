"""Football goal collider: the goal net collides as its exact triangle mesh (always on for football_single).

Upstream's goal asset (football_goal/football_goal_physics.usd) collides through convexDecomposition.
Isaac Sim 6.1's PhysX cooks one of its hulls across the goal mouth: an invisible flat step 5.4 cm above
the turf and 15 cm deep, while the drawn ground bar lies below the turf. The same decomposition puts a
low ceiling inside the goal and makes the posts wider than drawn and asymmetric. The HumanoidArena
paper (arXiv 2606.17833) describes a kinematic goal net and no step. Whether Isaac Sim 5.0, upstream's
version, cooks the same hull is unverified.

Here the goal mesh collides as the exact triangle mesh
(approximation "none", which PhysX supports on the kinematic body upstream declares). Upstream files
stay unchanged: the instanced goal geometry becomes ordinary prims of the live stage, where the
approximation is authored, and PhysX reads it when the simulation is reset (parsed again). Pose, body
flags, contact offsets, materials, the ball, the rendering and the success predicate stay upstream's.
"""
GOAL_ASSET_DIRECTORY = "/football_scene/interaction_obj/football_goal/"
TASKS = frozenset({"football", "football_single"})
ENV_REGEX_NS = "/World/envs/env_.*"


def goal_entities(cfg):
    """(scene entity, prim path expression, asset) of every entity spawned from upstream's goal asset."""
    found = []
    for name, value in vars(cfg.scene).items():
        asset = str(getattr(getattr(value, "spawn", None), "usd_path", "") or "")
        if GOAL_ASSET_DIRECTORY in asset:
            found.append((name, value.prim_path, asset))
    return sorted(found)


def use_exact_goal_mesh(stage, cfg, env_namespace="/World/envs/env_0"):
    """Author triangle-mesh collision on the goal meshes of the live stage and return what changed.

    Call before the simulation is reset: PhysX reads the change only when it parses the stage again.
    """
    entities = goal_entities(cfg)
    if not entities:
        raise RuntimeError("flat_goal_mouth: the scene has no upstream football goal")
    from pxr import Usd, UsdGeom, UsdPhysics
    records = []
    for name, expression, asset in entities:
        path = expression.replace("{ENV_REGEX_NS}", env_namespace).replace(ENV_REGEX_NS, env_namespace)
        root = stage.GetPrimAtPath(path)
        if not root.IsValid():
            raise RuntimeError(f"flat_goal_mouth: goal prim missing at {path}")
        if not (root.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(root).GetKinematicEnabledAttr().Get()):
            # PhysX would replace a triangle mesh on a dynamic body with one convex hull.
            raise RuntimeError(f"flat_goal_mouth: {path} is not the kinematic body upstream declares")
        # Instance proxies are read-only: make the instanced geometry ordinary prims of this stage.
        cleared = []
        while True:
            instances = [prim for prim in Usd.PrimRange(root) if prim.IsInstance()]
            if not instances:
                break
            for prim in instances:
                prim.SetInstanceable(False)
                cleared.append(str(prim.GetPath()))
        meshes = []
        for prim in Usd.PrimRange(root):
            if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                attribute = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr()
                before = str(attribute.Get())
                attribute.Set("none")
                meshes.append({"prim": str(prim.GetPath()), "approximation_upstream": before,
                               "approximation": str(attribute.Get())})
        if not meshes:
            raise RuntimeError(f"flat_goal_mouth: no collision mesh under {path}")
        records.append({"patch": "flat_goal_mouth", "entity": name, "prim": path, "asset": asset,
                        "instanceable_cleared": cleared, "meshes": meshes,
                        "collision": "exact triangle mesh on the kinematic goal body",
                        "unchanged": "pose, body flags, contact offsets, materials, ball, rendering, success predicate",
                        "reason": "upstream convexDecomposition cooked by Isaac Sim 6.1 adds an invisible 5.4 cm "
                                  "step across the goal mouth, a low interior ceiling and wide asymmetric posts"})
    return records
