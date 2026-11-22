"""hoopkit: a closed spring-steel hoop (pop-up sunshade frame) for yam_arena task files, built on cablekit.

The hoop is a ring of rigid capsule segments joined by D6 joints that bend (two axes) and twist, each with a
spring-damper drive back to the hoop's rest shape: a flat circle. The chain is a PhysX articulation from seg_0 round
to seg_{n-1}; one extra joint (excluded from the articulation, same drives) closes the ring from seg_{n-1} to seg_0.

It reuses cablekit's scene hook, articulation object and CableState readout: a Hoop is a cablekit Cable whose spawner
lays the segments on a circle instead of a straight line.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import cablekit
from cablekit import Cable


@dataclass(frozen=True)
class Hoop(Cable):
    centre: tuple[float, float, float] = (0.0, 0.0, 0.0)  # hoop centre (m), lying flat at spawn
    hoop_radius: float = 0.22  # m, to the wire centreline
    twist_stiffness: float = 0.02  # N·m/rad per joint
    max_twist: float = 1.2  # rad per joint (limit)

    @property
    def seg_len(self) -> float:  # chord
        return 2 * self.hoop_radius * math.sin(math.pi / self.segments)

    @property
    def dtheta(self) -> float:
        return 2 * math.pi / self.segments

    def _check(self) -> None:
        pass

    def seg_pose(self, i: int):
        """Spawn pose of segment i: centre (3,) and heading angle (rad about z). seg_0 sits at the hoop's -y side
        (well, at angle -pi/2 + yaw from the centre), tangent along +x rotated by yaw."""
        a = self.hoop_radius * math.cos(math.pi / self.segments)  # apothem: chord midpoints
        ang = self.yaw + i * self.dtheta
        c = np.asarray(self.centre, float)
        p = c + a * np.array([math.sin(ang), -math.cos(ang), 0.0])
        return p, ang

    def point_at(self, s: float) -> np.ndarray:  # used by cablekit for the articulation root pose (s = seg_len/2)
        return self.seg_pose(0)[0]

    def start_points(self) -> np.ndarray:
        return np.stack([self.seg_pose(i)[0] for i in range(self.segments)])


def hoop(name: str, centre, *, hoop_radius=0.22, segments=48, radius=0.005, yaw=0.0, color=(0.1, 0.1, 0.1), **kw) -> Hoop:
    c = np.asarray(centre, float)
    lo = c - np.array([hoop_radius + radius, hoop_radius + radius, radius])
    hi = c + np.array([hoop_radius + radius, hoop_radius + radius, radius])
    return Hoop(name=name, pos=tuple(((lo + hi) / 2).tolist()), size=tuple((hi - lo).tolist()), color=color, yaw=yaw,
                start=tuple(c.tolist()), length=2 * math.pi * hoop_radius, radius=radius, segments=segments,
                centre=tuple(float(v) for v in c), hoop_radius=hoop_radius, collision_sides=0, **kw)


def _quat_z(a: float):
    from pxr import Gf

    return Gf.Quatf(math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


def _spawn_hoop(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Hoop = cablekit._SPECS[cfg.cable_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    vis = f"{prim_path}/Looks/hoop"
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.5)
    look.func(vis, look)
    phys = f"{prim_path}/Looks/hoop_physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    pm.func(phys, pm)

    n, L, r = spec.segments, spec.seg_len, spec.radius
    seg_mass = spec.cable_mass / n
    p0, a0 = spec.seg_pose(0)
    # local frame = seg_0 frame (root Xform is placed at seg_0's pose by cablekit's initial pose, yaw = a0)
    R0 = np.array([[math.cos(a0), -math.sin(a0), 0], [math.sin(a0), math.cos(a0), 0], [0, 0, 1.0]])

    seg_paths = []
    for i in range(n):
        p, a = spec.seg_pose(i)
        loc = R0.T @ (p - p0)
        path = f"{prim_path}/seg_{i}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*loc.tolist()))
        q = _quat_z(a - a0)
        xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(q.GetReal(), *q.GetImaginary()))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(seg_mass)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateLinearDampingAttr(0.5)
        rb.CreateAngularDampingAttr(2.0)
        cap = UsdGeom.Capsule.Define(stage, f"{path}/geom")
        cap.CreateAxisAttr("X")
        cap.CreateRadiusAttr(r)
        cap.CreateHeightAttr(max(L - 2 * r, 1e-4) + 2 * r * 0.9)
        UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
        cp = PhysxSchema.PhysxCollisionAPI.Apply(cap.GetPrim())
        cp.CreateContactOffsetAttr(0.002)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(cap.GetPath()), phys, stage=stage)
        bind_visual_material(str(cap.GetPath()), vis, stage=stage)
        seg_paths.append(path)

    d = spec.dtheta

    def joint(parent, child, name, excluded=False):
        j = UsdPhysics.Joint.Define(stage, f"{child}/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        j.CreateLocalPos0Attr(Gf.Vec3f(L / 2, 0, 0))
        j.CreateLocalRot0Attr(_quat_z(d / 2))
        j.CreateLocalPos1Attr(Gf.Vec3f(-L / 2, 0, 0))
        j.CreateLocalRot1Attr(_quat_z(-d / 2))
        if excluded:
            j.CreateExcludeFromArticulationAttr(True)
        prim = j.GetPrim()
        for dof in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(prim, dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        for dof, mx, k in (("rotX", spec.max_twist, spec.twist_stiffness), ("rotY", spec.max_bend, spec.bend_stiffness),
                           ("rotZ", spec.max_bend, spec.bend_stiffness)):
            if not excluded:
                lim = UsdPhysics.LimitAPI.Apply(prim, dof)
                lim.CreateLowAttr(-math.degrees(mx))
                lim.CreateHighAttr(math.degrees(mx))
            drv = UsdPhysics.DriveAPI.Apply(prim, dof)
            drv.CreateTypeAttr("force")
            drv.CreateStiffnessAttr(k * math.pi / 180)
            drv.CreateDampingAttr(spec.bend_damping * math.pi / 180)
            drv.CreateTargetPositionAttr(0.0)

    for i in range(1, n):
        joint(seg_paths[i - 1], seg_paths[i], f"bend_{i}")
    joint(seg_paths[-1], seg_paths[0], "close", excluded=True)

    root_link = stage.GetPrimAtPath(seg_paths[0])
    UsdPhysics.ArticulationRootAPI.Apply(root_link)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root_link)
    art.CreateEnabledSelfCollisionsAttr(spec.self_collision)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


_orig_spawn = cablekit._spawn_cable


def _spawn_dispatch(prim_path, cfg, translation=None, orientation=None, **kwargs):
    if isinstance(cablekit._SPECS.get(cfg.cable_name), Hoop):
        return _spawn_hoop(prim_path, cfg, translation, orientation, **kwargs)
    return _orig_spawn(prim_path, cfg, translation, orientation, **kwargs)


if not getattr(cablekit._spawn_cable, "_hoop", False):
    _spawn_dispatch._hoop = True
    cablekit._spawn_cable = _spawn_dispatch
