"""Bakery bench: lift a slab cake out of its deep square tin by the two ends of its parchment sling and set it on the
cooling rack.

Scene (metres from the arm-base midpoint, x forward, y left; heights above the table). A dark-grey square tin stands on
the table at (0.32, 0): inner 12.8 x 16.3 cm (x by y), 5 cm deep (floor top 0.3 cm, rim 5.3 cm), 4 mm walls, a
non-stick inside (friction 0.05). In it a brown cake slab (11.6 x 14.0 x 4.0 cm, 400 g) rests on a white parchment
sling: a strip 9 cm wide and 2 mm thick (34 pieces of 1 cm, hinged across the strip, 34 cm long) that lies on the tin
floor under the cake, runs up the +y and -y walls in the 11.5 mm gaps beside the cake and stands 4 cm up out of each
gap above the rim. The cake is wider than the gripper opens (9.4 cm) in every direction, so it can only be lifted by
the sling. A pale wire cooling rack (14 x 18 cm, 2 cm tall) stands in front of the tin at (0.16, 0).

The sling is not tied to anything. Lifted by one end only, it runs: the strip slides out from under the cake (which
stays on the slick floor) and the other end is drawn down into the tin. Only both ends lifted together form a basket
that carries the cake; letting one go mid-carry spills it. So two hands must lift, carry and lower together, level.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants: SLING_DEMO=correct (default) | none | hold | one_left | one_right | one_then_other | gather |
uneven | tilt | drop | push | drag | tier03 | tier06.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the sling below is simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("A brown cake slab sits in the deep grey tin on a white parchment sling whose two ends stand up out of "
               "the tin. Lift the cake out by the sling, set it flat on the wire rack and let go.")

TIN_COL = (0.24, 0.25, 0.27)
CAKE_COL = (0.55, 0.34, 0.17)
PAPER_COL = (0.96, 0.95, 0.90)
RACK_COL = (0.80, 0.80, 0.78)

# -- sling geometry: every straight run is a whole number of 1 cm pieces, so each 90 deg crease sits on a hinge -------
SEG = 0.01
H = 0.001  # half thickness (2 mm strip)
SW = 0.09  # strip width (along x)
N_FLOOR, N_WALL, N_UP = 16, 5, 4  # pieces: across the floor, up each wall inside the tin, standing above the rim
N_SEG = N_FLOOR + 2 * (N_WALL + N_UP)
SLING_LEN = N_SEG * SEG
LIFT = 0.001  # written shape raised 1 mm so no piece starts inside the floor

# -- tin (fixed) ----------------------------------------------------------------------------------------------------------
TX = 0.32
WALL = 0.004
FLOOR_T = 0.003
ZF = TOP + FLOOR_T  # floor top
DEPTH = N_WALL * SEG  # 5 cm
RIM = ZF + DEPTH  # rim (wall top)
A_IN = N_FLOOR * SEG / 2 + H + 0.0005  # inner half width along y (sling legs ride the +y and -y walls)
B_IN = 0.064  # inner half length along x
TIN_MU = 0.05  # non-stick tin (min-combined: anything on it slides at 0.05)

_wz = (RIM - TOP) / 2 + TOP
TIN = [
    Box("tin_floor", pos=(TX, 0.0, TOP + FLOOR_T / 2), size=(2 * B_IN, 2 * A_IN, FLOOR_T), color=TIN_COL),
    Box("tin_wall_left", pos=(TX, A_IN + WALL / 2, _wz), size=(2 * B_IN + 2 * WALL, WALL, RIM - TOP), color=TIN_COL),
    Box("tin_wall_right", pos=(TX, -A_IN - WALL / 2, _wz), size=(2 * B_IN + 2 * WALL, WALL, RIM - TOP), color=TIN_COL),
    Box("tin_wall_far", pos=(TX + B_IN + WALL / 2, 0.0, _wz), size=(WALL, 2 * A_IN, RIM - TOP), color=TIN_COL),
    Box("tin_wall_near", pos=(TX - B_IN - WALL / 2, 0.0, _wz), size=(WALL, 2 * A_IN, RIM - TOP), color=TIN_COL),
]
TIN_OUT = (B_IN + WALL, A_IN + WALL)  # outer half sizes (x, y)

# -- cake (free) ------------------------------------------------------------------------------------------------------------
CAKE_SIZE = (0.116, 0.140, 0.040)
CAKE_M = 0.40
CAKE_MU = 0.15  # baked crust on silicone-coated parchment (min-combined: the cake slides on the sling at 0.15)
CAKE_Z0 = ZF + 2 * H + LIFT + CAKE_SIZE[2] / 2 + 0.0005
CAKE = Box("cake", pos=(TX, 0.0, CAKE_Z0), size=CAKE_SIZE, mass=CAKE_M, color=CAKE_COL)
CAKE_HALF = np.array(CAKE_SIZE) / 2

# -- cooling rack (fixed): four feet, two side rails and eight cross wires ----------------------------------------------------
RX, RY = 0.16, 0.0
RACK_LX, RACK_LY, RACK_H = 0.14, 0.18, 0.02
RACK_TOP = TOP + RACK_H
_BAR_T = 0.004
RACK = [Box(f"rack_foot_{i}", pos=(RX + sx * (RACK_LX / 2 - 0.006), RY + sy * (RACK_LY / 2 - 0.006),
                                   TOP + (RACK_H - _BAR_T) / 2), size=(0.010, 0.010, RACK_H - _BAR_T), color=RACK_COL)
        for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1)))]
RACK += [Box(f"rack_rail_{i}", pos=(RX, RY + s * (RACK_LY / 2 - 0.003), RACK_TOP - _BAR_T / 2),
             size=(RACK_LX, 0.006, _BAR_T), color=RACK_COL) for i, s in enumerate((1, -1))]
RACK += [Box(f"rack_wire_{i}", pos=(RX - RACK_LX / 2 + 0.005 + i * (RACK_LX - 0.010) / 7, RY, RACK_TOP - _BAR_T / 2),
             size=(0.004, RACK_LY, _BAR_T), color=RACK_COL) for i in range(8)]

# -- sling: a flat strip; spawned straight on the table out of the way, laid into the tin at every reset ----------------
SLING_K, SLING_C = 0.01, 0.001  # N m/rad per hinge (bending through the strip's thickness), rest shape straight
INPLANE_K, INPLANE_C, INPLANE_MAX = 0.3, 0.01, 0.05  # a 9 cm wide strip does not bend edgewise
SLING_MU = 0.35
SLING = cable("sling", start=(0.56, 0.18, TOP + H + 0.0005), yaw=-math.pi / 2, length=SLING_LEN, radius=H,
              segment=SEG, cable_mass=0.018, bend_stiffness=SLING_K, bend_damping=SLING_C, friction=SLING_MU,
              max_bend=1.65, color=PAPER_COL)
LEG_Y = A_IN - 0.0005 - H  # leg centreline |y|
_Z0 = ZF + H  # centreline on the floor
_TOPZ = _Z0 + (N_WALL + N_UP) * SEG
SLING_PATH = np.array([
    [TX, LEG_Y + 0.0005, _TOPZ],  # +y leg top (a hair outboard so seg_0's width axis is set along x)
    [TX, LEG_Y, _Z0],
    [TX, -LEG_Y, _Z0],
    [TX, -LEG_Y, _TOPZ],
])

FIXED = [*TIN, *RACK]
OBJECTS = [*FIXED, CAKE, SLING]

TWIN_EXTRA = [
    {"name": "sling_creases", "kind": "joint", "real": "silicone-coated baking parchment, folded up the two sides of the tin",
     "note": "sim: 34 rigid 1 x 9 cm pieces, 2 mm thick, hinged across the strip with weak straightening springs"},
]


def _install_spawn_hook() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_sling_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != SLING.name:
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        # flat pieces: a 90 x 2 mm box collider and a flat visual per piece; stiff edgewise
        L, hw, hh = SLING.seg_len, SW / 2, H
        pts = []
        for x, sw, sh in ((L / 2 - 0.3 * hh, hw, hh), (L / 2 + 0.6 * hh, 0.97 * hw, 0.55 * hh)):
            for sx in (-x, x):
                for cy, cz in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    pts.append(Gf.Vec3f(sx, cy * sw, cz * sh))
        for i in range(SLING.segments):
            seg = f"{prim_path}/seg_{i}"
            col = stage.GetPrimAtPath(f"{seg}/collider")
            if col and col.IsValid():
                m = UsdGeom.Mesh(col)
                m.GetPointsAttr().Set(pts)
                m.GetFaceVertexCountsAttr().Set([4, 4, 4, 4])
                m.GetFaceVertexIndicesAttr().Set(list(range(16)))
            cap = stage.GetPrimAtPath(f"{seg}/geom")
            if cap and cap.IsValid():
                UsdGeom.Imageable(cap).MakeInvisible()
            if i > 0:
                jp = stage.GetPrimAtPath(f"{seg}/bend_{i}")
                if jp and jp.IsValid():
                    UsdPhysics.DriveAPI(jp, "rotZ").GetStiffnessAttr().Set(INPLANE_K * math.pi / 180)
                    UsdPhysics.DriveAPI(jp, "rotZ").GetDampingAttr().Set(INPLANE_C * math.pi / 180)
                    lim = UsdPhysics.LimitAPI(jp, "rotZ")
                    lim.GetLowAttr().Set(-math.degrees(INPLANE_MAX))
                    lim.GetHighAttr().Set(math.degrees(INPLANE_MAX))
            cube = UsdGeom.Cube.Define(stage, f"{seg}/slab")
            cube.CreateSizeAttr(1.0)
            cube.AddScaleOp().Set(Gf.Vec3f(L + 0.0015, SW, 2 * H))
            bind_visual_material(str(cube.GetPath()), f"{prim_path}/Looks/cable", stage=stage)
        # non-stick tin: min-combined low friction on every tin part
        mat = f"{parent}/tin_nonstick_material"
        mc = sim_utils.RigidBodyMaterialCfg(static_friction=TIN_MU, dynamic_friction=TIN_MU, friction_combine_mode="min")
        mc.func(mat, mc)
        for nm in [o.name for o in TIN]:
            for p in Usd.PrimRange(stage.GetPrimAtPath(f"{parent}/{nm}")):
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    bind_physics_material(str(p.GetPath()), mat, stage=stage)
        cmat = f"{parent}/cake_material"
        cc = sim_utils.RigidBodyMaterialCfg(static_friction=CAKE_MU, dynamic_friction=CAKE_MU, friction_combine_mode="min")
        cc.func(cmat, cc)
        cake_prim = stage.GetPrimAtPath(f"{parent}/{CAKE.name}")
        nb = 0
        for p in Usd.PrimRange(cake_prim):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                bind_physics_material(str(p.GetPath()), cmat, stage=stage)
                nb += 1
        print(f"[sling] cake material bound to {nb} collider(s)", flush=True)
        if cake_prim.IsValid():
            PhysxSchema.PhysxRigidBodyAPI.Apply(cake_prim).CreateAngularDampingAttr(1.0)
        return prim

    spawn._sling_hook = True
    cablekit._spawn_cable = spawn


_install_spawn_hook()


# -- start layout: cake set on the floor, sling laid under it and up both walls ---------------------------------------
def _shape(emb) -> None:
    import torch

    env = getattr(emb, "_env", emb)
    base = env.unwrapped
    dev = base.device
    origin = getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0]
    o = base.scene[CAKE.name]
    ids = torch.tensor([0], device=dev)
    pose = torch.tensor([[*CAKE.pos, 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32, device=dev)
    pose[0, :3] += origin
    try:
        o.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=dev), env_ids=ids)
    except Exception as e:  # pragma: no cover
        print(f"[sling] cake pose write failed: {e!r}", flush=True)
    cableshape.preshape(emb, SLING, SLING_PATH, settle_s=0, hold="ramp", lift=LIFT, max_angle=1.6)
    cableshape.settle(emb, 2.0)


cableshape.on_reset(_shape)

# -- monitors: cake pose/velocity and robot-cake contact every physics substep (240 Hz) ----------------------------------
TOUCH_SEP = 0.001
DROP_VZ = 0.35
TILT_MAX = 35.0
_ROBOT_PRIMS = ("/LeftArm/", "/RightArm/")
_MON: dict = {"view": None, "body": None, "polls": 0, "err": None}
_LATCH: dict = {}


def _reset_latches() -> None:
    _LATCH.clear()
    _LATCH.update(nan=False, robot_touch=False, near=False, drop=False, tilt=False, vzmax=0.0,
                  tiltmax=0.0)


_reset_latches()


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def _quat_mat(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tilt_deg(rot) -> float:
    return math.degrees(math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))))


def _check_pose(c, rot, v) -> None:
    vz = abs(float(v[2]))
    _LATCH["vzmax"] = max(_LATCH["vzmax"], vz)
    tl = tilt_deg(rot)
    _LATCH["tiltmax"] = max(_LATCH["tiltmax"], tl)
    if -float(v[2]) > DROP_VZ and not _LATCH["drop"]:  # falling only: a brisk upward lift is not a failure
        _LATCH["drop"] = True
        print(f"[sling] cake dropped: falling at {vz:.2f} m/s at {np.round(c - [0, 0, TOP], 3)}", flush=True)
    if tl > TILT_MAX and not _LATCH["tilt"]:
        _LATCH["tilt"] = True
        print(f"[sling] cake tilted {tl:.1f} deg at {np.round(c - [0, 0, TOP], 3)}", flush=True)


def _enable_contact_processing() -> None:
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # pragma: no cover
        pass


def _make_views(mgr) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    sv = mgr.get_physics_sim_view()
    if sv is None:
        return
    stage = get_current_stage()
    sensors, cake = [], None
    for prim in stage.Traverse():
        p = prim.GetPath().pathString
        if "/env_0/" not in p:
            continue
        if any(r in p for r in _ROBOT_PRIMS) and "PhysxContactReportAPI" in prim.GetAppliedSchemas():
            sensors.append(p)
        elif p.endswith(f"/{CAKE.name}") and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            cake = p
    if not sensors or cake is None:
        raise RuntimeError(f"cake monitor: {len(sensors)} robot links, cake={cake}")
    _MON["view"] = sv.create_rigid_contact_view(sensors, filter_patterns=[[cake]] * len(sensors),
                                                max_contact_data_count=64 * len(sensors))
    _MON["body"] = sv.create_rigid_body_view(cake)
    _MON["stage"] = stage
    print(f"[sling] monitors: {len(sensors)} robot links x {cake}", flush=True)


def _poll(mgr) -> None:
    if not _LATCH.get("armed", False):
        return
    try:
        if _MON["view"] is None or _MON.get("stage") is None:
            _make_views(mgr)
        if _MON["view"] is None:
            return
        _, _, _, sep, counts, starts = _MON["view"].get_contact_data(dt=mgr.get_physics_dt())
        tf = _np(_MON["body"].get_transforms()).reshape(-1)[:7].astype(np.float64)
        vel = _np(_MON["body"].get_velocities()).reshape(-1)[:6].astype(np.float64)
    except Exception as e:  # pragma: no cover
        if _MON["err"] is None:
            _MON["err"] = repr(e)
            print(f"[sling] monitor error: {e!r}", flush=True)
        return
    _MON["polls"] += 1
    counts = _np(counts).reshape(-1)
    if counts.any() and not _LATCH["robot_touch"]:
        starts, sep = _np(starts).reshape(-1), _np(sep).reshape(-1)
        idx = np.concatenate([np.arange(s, s + c) for s, c in zip(starts[counts > 0], counts[counts > 0])])
        if float(sep[idx].min()) <= TOUCH_SEP:
            _LATCH["robot_touch"] = True
            print("[sling] a robot part touched the cake", flush=True)
    if not (np.all(np.isfinite(tf)) and np.all(np.isfinite(vel))):
        _LATCH["nan"] = True
        return
    origin = _MON.get("origin", np.zeros(3))
    _check_pose(tf[:3] - origin, _quat_mat(tf[3:7]), vel[:3])


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_sling_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__
    base_step = mgr.step.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        _enable_contact_processing()

    def step(cls) -> None:
        base_step(cls)
        _poll(cls)

    step._sling_hooked = True
    mgr._configure_physics = classmethod(_configure_physics)
    mgr.step = classmethod(step)


class _MgrFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != _PHYSX_MGR:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_manager(module)

        spec.loader.exec_module = exec_module
        return spec


_ENV: dict = {"env": None}


def _install_hooks() -> None:
    if _PHYSX_MGR in sys.modules:
        _patch_manager(sys.modules[_PHYSX_MGR])
    elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _MgrFinder())
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_sling_hooked", False):
        base_reset = cls.reset  # cableshape's reset (shaping + settle) runs inside this call

        def reset(self, *a, **k):
            _MON["view"] = None
            _LATCH["armed"] = False
            out = base_reset(self, *a, **k)
            try:
                _MON["origin"] = _np(self._env.unwrapped.scene.env_origins[0]).astype(np.float64)
            except Exception:  # pragma: no cover
                pass
            _reset_latches()
            _LATCH["armed"] = True
            return out

        reset._sling_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_sling_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._sling_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[tuple[np.ndarray, np.ndarray]]:
    """Per arm (left, right): (TCP, pad tips (2,3)); [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import pad_tips_world, tcp_world

    base = env.unwrapped
    origin = _np(base.scene.env_origins[0]).astype(np.float64)
    return [(np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin,
             np.asarray(pad_tips_world(base.scene[k]), dtype=np.float64).reshape(-1, 3) - origin)
            for k in ("left_robot", "right_robot")]


# -- geometry on states --------------------------------------------------------------------------------------------------
def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * (k / n) for k in range(1, n + 1)]
    return np.asarray(out)


def cake_pose(objects):
    s = objects[CAKE.name]
    return np.asarray(s.pos, dtype=float), _quat_mat(np.asarray(s.quat_xyzw, dtype=float))


def cake_corners(c, rot) -> np.ndarray:
    s = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
    return c + (s * CAKE_HALF) @ rot.T


def dist_to_cake(p, c, rot) -> float:
    q = rot.T @ (np.asarray(p) - c)
    return float(np.linalg.norm(np.maximum(np.abs(q) - CAKE_HALF, 0.0)))


def sling_pts(objects) -> np.ndarray:
    return _dense(objects[SLING.name].polyline())


def _finite(objects) -> bool:
    return bool(np.isfinite(objects[SLING.name].points).all() and np.isfinite(objects[CAKE.name].pos).all()
                and np.isfinite(objects[CAKE.name].quat_xyzw).all())


NEAR_CAKE = 0.015  # logged only (not scored): a gripper (TCP or pad tip) within 1.5 cm of the cake
RELEASE = 0.04  # let go: every TCP and pad tip >= 4 cm from the cake and from every point of the sling
REST_V = 0.02


def _track(objects) -> None:
    if not _finite(objects):
        _LATCH["nan"] = True
        return
    g = grippers()
    if not g:
        return
    c, rot = cake_pose(objects)
    for tcp, tips in g:
        for p in np.vstack([tcp[None], tips]):
            if dist_to_cake(p, c, rot) < NEAR_CAKE and not _LATCH["near"]:
                _LATCH["near"] = True
                print(f"[sling] a gripper came within {NEAR_CAKE * 100:.1f} cm of the cake", flush=True)


def failure(objects) -> str | None:
    _track(objects)
    if _LATCH["nan"] or not _finite(objects):
        return "nan"
    for k in ("robot_touch", "drop", "tilt"):
        if _LATCH[k]:
            return k
    return None


def sling_pieces() -> list[tuple[np.ndarray, np.ndarray]]:
    """(centre, rotation) of every 1 x 9 cm sling piece, from the live sim; [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    art = base.scene[SLING.name]
    origin = _np(base.scene.env_origins[0]).astype(np.float64)
    names = list(art.body_names)
    pos = _np(art.data.body_pos_w)[0].astype(np.float64)
    quat = _np(art.data.body_quat_w)[0].astype(np.float64)
    idx = [names.index(f"seg_{i}") for i in range(SLING.segments)]
    ctr = pos[idx] - origin
    tang = np.gradient(ctr, axis=0)
    out = []
    for j, k in enumerate(idx):
        # the stored quaternion order (w first or last) differs across Isaac Lab versions: take the reading whose
        # piece x axis lies along the strip, as it does by construction
        q = quat[k]
        rots = [_quat_mat(q), _quat_mat((q[1], q[2], q[3], q[0]))]
        r = max(rots, key=lambda m: abs(float(m[:, 0] @ tang[j])))
        out.append((ctr[j], r))
    return out


PIECE_HALF = np.array([SEG / 2, SW / 2, H])  # piece frame: x along the strip, y across its 9 cm width, z thickness


def dist_to_sling(p, objects, pieces) -> float:
    """Distance from p to the strip's full 9 cm wide surface (every piece as a box), or to its centreline offline."""
    if pieces:
        return min(float(np.linalg.norm(np.maximum(np.abs(r.T @ (np.asarray(p) - c)) - PIECE_HALF, 0.0)))
                   for c, r in pieces)
    return float(np.linalg.norm(sling_pts(objects) - p, axis=1).min())


def released(objects) -> bool:
    """Every TCP and pad tip is >= 4 cm from the cake and from every point of the sling, edges included."""
    c, rot = cake_pose(objects)
    pieces = sling_pieces()
    for tcp, tips in grippers():
        for p in np.vstack([tcp[None], tips]):
            if dist_to_sling(p, objects, pieces) < RELEASE or dist_to_cake(p, c, rot) < RELEASE:
                return False
    return True


def out_of_tin(objects) -> bool:
    """No corner of the cake is inside the tin: each corner is above the rim or outside the tin's outer outline."""
    c, rot = cake_pose(objects)
    for p in cake_corners(c, rot):
        inside_xy = abs(p[0] - TX) <= TIN_OUT[0] and abs(p[1]) <= TIN_OUT[1]
        if inside_xy and p[2] <= RIM:
            return False
    return True


def on_rack(objects) -> bool:
    """Cake bottom on the rack: lowest corner at most 8 mm above the wires (the sling lies between) and centre over
    the rack."""
    c, rot = cake_pose(objects)
    low = float(cake_corners(c, rot)[:, 2].min())
    return low <= RACK_TOP + 0.008 and abs(c[0] - RX) <= RACK_LX / 2 and abs(c[1] - RY) <= RACK_LY / 2


def score(objects) -> float:
    """0 if at any time: a robot part touched the cake; the cake fell faster than 0.35 m/s (dropped); it tilted more
    than 35 deg; or a state went
    non-finite. Also 0 unless, at the end, both grippers are 4 cm or more
    from the cake and from the sling (let go), the cake is still (< 2 cm/s) and wholly out of the tin (every corner
    above the rim or outside the tin's outline). Then: 1.0 if it rests on the rack, centre within 3 cm of the rack
    centre, tilt < 5 deg; 0.6 if it rests upright (tilt < 10 deg) anywhere else; else 0.3."""
    if failure(objects) or not released(objects) or not out_of_tin(objects):
        return 0.0
    if float(np.linalg.norm(objects[CAKE.name].lin_vel)) >= REST_V:
        return 0.0
    c, rot = cake_pose(objects)
    tl = tilt_deg(rot)
    if tl >= 10.0:
        return 0.3
    if on_rack(objects) and float(np.linalg.norm(c[:2] - [RX, RY])) <= 0.03 and tl < 5.0:
        return 1.0
    return 0.6


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and float(np.linalg.norm(objects[CAKE.name].lin_vel)) < 0.01)


def distance(objects) -> float:
    if failure(objects):
        return 1.0
    c, _ = cake_pose(objects)
    return float(np.linalg.norm(c[:2] - [RX, RY]))


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    if not _finite(objects):
        return {}
    c, rot = cake_pose(objects)
    low = float(cake_corners(c, rot)[:, 2].min())
    return {"lifted": float(low > ZF + 0.02), "cleared_rim": float(low > RIM), "out_of_tin": float(out_of_tin(objects)),
            "on_rack": float(on_rack(objects)), "released": float(released(objects)),
            "clean": float(failure(objects) is None)}


VARIATION = {"obj:cake": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}  # sits in a close-fitting tin on the laid sling: fixed


# -- demo ----------------------------------------------------------------------------------------------------------------
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402

GRASP_OPEN = -0.016
PINCH_Z = RIM + 0.025  # TCP height for the pinch (pad tips ~1.2 cm above the rim, ~3 cm above the cake)
CARRY_Z = TOP + 0.175
STOW = {1: (0.10, 0.40, TOP + 0.12), -1: (0.10, -0.40, TOP + 0.12)}


def _log(ctx, tag=""):
    st = ctx.states()
    if not _finite(st):
        print(f"[t={ctx.t:5.2f}] {tag}: NON-FINITE", flush=True)
        return
    c, rot = cake_pose(st)
    low = float(cake_corners(c, rot)[:, 2].min())
    p = st[SLING.name].points
    print(f"[t={ctx.t:5.2f}] {tag}: cake={np.round(c - [0, 0, TOP], 3)} low={low - TOP:.3f} tilt={tilt_deg(rot):.1f} "
          f"v={np.linalg.norm(st[CAKE.name].lin_vel):.3f} ends=({np.round(p[0] - [0, 0, TOP], 3)}, "
          f"{np.round(p[-1] - [0, 0, TOP], 3)}) L={np.round(ctx.left.tcp() - [0, 0, TOP], 3)} "
          f"R={np.round(ctx.right.tcp() - [0, 0, TOP], 3)} fail={failure(st)} rel={released(st)} "
          f"out={out_of_tin(st)} score={score(st):.2f} vz={_LATCH['vzmax']:.2f} tiltmax={_LATCH['tiltmax']:.1f} "
          f"polls={_MON['polls']}", flush=True)


def _leg_point(st, sign) -> np.ndarray:
    """Live point on a standing leg at the pinch height (sign +1: the +y leg, seg_0 end; -1: the -y leg)."""
    p = st[SLING.name].points
    leg = p[: N_WALL + N_UP] if sign > 0 else p[-(N_WALL + N_UP):]
    i = int(np.argmin(np.abs(leg[:, 2] - (PINCH_Z - 0.008))))
    return leg[i]


def _pinch(arm, ctx, sign):
    q = _leg_point(ctx.states(), sign)
    yield from arm.move((q[0], q[1], TOP + 0.16), yaw=0.0, finger=GRASP_OPEN, duration=1.6)
    q = _leg_point(ctx.states(), sign)
    yield from arm.move((q[0], q[1], PINCH_Z + 0.03), yaw=0.0, duration=0.7)
    yield from arm.move((q[0], q[1], PINCH_Z), yaw=0.0, duration=0.7)
    yield from arm.grip(FINGER_CLOSED, 0.6)


def _both_move(ctx, pl, pr, duration, steps=None):
    """Both hands together in small steps (straight lines, level)."""
    L, Rt = ctx.left, ctx.right
    a0, b0 = L.tcp(), Rt.tcp()
    n = steps or max(1, int(duration / 0.1))
    for k in range(1, n + 1):
        t = k / n
        yield from ctx.both(L.move(tuple(a0 + (np.asarray(pl) - a0) * t), yaw=0.0, duration=duration / n),
                            Rt.move(tuple(b0 + (np.asarray(pr) - b0) * t), yaw=0.0, duration=duration / n))


def _let_go(arm, sign):
    """Open, lift 4 cm straight up, then sweep back beside the arm's own base, out of the top camera's view."""
    yield from arm.grip(GRASP_OPEN, 0.4)
    p = arm.tcp()
    yield from arm.move((p[0], p[1], p[2] + 0.04), yaw=0.0, duration=0.4)
    yield from arm.move(STOW[sign], yaw=0.0, finger=FINGER_CLOSED, duration=0.8)


def _settle_cake(ctx, tmax=4.0):
    t = 0.0
    while t < tmax and float(np.linalg.norm(ctx.states()[CAKE.name].lin_vel)) > 0.005:
        yield from ctx.left.wait(0.25)
        t += 0.25


RUN_S = 30.0  # run limit: 600 control steps at 20 Hz


def _end(ctx):
    """Wait out the run (a successful demo ends earlier, 1 s after success holds), logging the final state."""
    yield from _settle_cake(ctx)
    while ctx.t < RUN_S - 1.5:
        yield from ctx.left.wait(0.5)
    for _ in range(2):
        yield from ctx.left.wait(0.5)
        _log(ctx, "end")


def demo_script(ctx):
    """SLING_DEMO=correct (default) | none | hold | one_left | one_right | one_then_other | gather | uneven | tilt |
    drop | push | drag | tier03 | tier06 (test-only variants; see README)."""
    mode = os.environ.get("SLING_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "none":
        yield from _end(ctx)
        return
    if mode == "push":  # a bare gripper prods the cake and tries to lever it up against the wall
        c = ctx.states()[CAKE.name].pos
        yield from L.move((c[0], c[1] + 0.03, TOP + 0.14), yaw=0.0, finger=FINGER_CLOSED, duration=1.5)
        yield from L.move((c[0], c[1] + 0.03, c[2] + CAKE_HALF[2] + 0.012), yaw=0.0, duration=1.0)
        yield from L.move((c[0] - 0.03, c[1] + 0.03, c[2] + CAKE_HALF[2] + 0.005), yaw=0.0, duration=1.0)
        _log(ctx, "pushed")
        yield from L.move(STOW[1], yaw=0.0, duration=1.0)
        yield from _end(ctx)
        return
    if mode in ("one_left", "one_right"):  # one hand lifts its end as high as it goes, holds, lets go
        sign, arm = (1, L) if mode == "one_left" else (-1, Rt)
        yield from _pinch(arm, ctx, sign)
        _log(ctx, "pinched")
        p = arm.tcp()
        for k, z in enumerate(np.linspace(p[2], TOP + 0.20, 24)):
            yield from arm.move((p[0], p[1], z), yaw=0.0, duration=0.25)
            if k % 3 == 2:
                _log(ctx, f"lifting z={z - TOP:.3f}")
        _log(ctx, "lifted high")
        for _ in range(6):
            yield from arm.wait(0.5)
            _log(ctx, "holding")
        yield from _let_go(arm, sign)
        yield from _end(ctx)
        return
    if mode == "one_then_other":  # lift the left end, let it go, then lift the right end
        for sign, arm in ((1, L), (-1, Rt)):
            yield from _pinch(arm, ctx, sign)
            p = arm.tcp()
            for z in np.linspace(p[2], TOP + 0.20, 16):
                yield from arm.move((p[0], p[1], z), yaw=0.0, duration=0.25)
            _log(ctx, f"lifted {sign:+d}")
            yield from arm.wait(1.0)
            yield from _let_go(arm, sign)
            _log(ctx, f"let go {sign:+d}")
        yield from _end(ctx)
        return
    if mode == "gather":  # one hand carries its end across towards the other end (to hold both in one hand)
        yield from _pinch(L, ctx, 1)
        p = L.tcp()
        yield from L.move((p[0], p[1], TOP + 0.16), yaw=0.0, duration=1.5)
        yield from L.move((p[0], 0.0, TOP + 0.16), yaw=0.0, duration=2.0)
        yield from L.move((p[0], -0.01, TOP + 0.12), yaw=0.0, duration=1.0)
        _log(ctx, "carried across")
        q = _leg_point(ctx.states(), -1)
        yield from Rt.move((q[0], q[1], TOP + 0.16), yaw=0.0, finger=GRASP_OPEN, duration=1.5)
        _log(ctx, "other end now")
        yield from L.wait(1.0)
        yield from _let_go(L, 1)
        yield from Rt.move(STOW[-1], yaw=0.0, duration=1.0)
        yield from _end(ctx)
        return

    yield from ctx.both(_pinch(L, ctx, 1), _pinch(Rt, ctx, -1))
    _log(ctx, "pinched")
    pl, pr = L.tcp(), Rt.tcp()
    if mode == "drag":  # both ends drawn towards the robot at pinch height: the cake is pulled at the near wall
        yield from _both_move(ctx, (RX, pl[1], PINCH_Z), (RX, pr[1], PINCH_Z), 5.0)
        _log(ctx, "dragged")
        yield from ctx.both(_let_go(L, 1), _let_go(Rt, -1))
        yield from _end(ctx)
        return
    if mode == "tilt":  # the left end lifted high, the right end only a little: the cake is tipped out sideways
        yield from _both_move(ctx, (TX, pl[1], TOP + 0.22), (TX, pr[1], PINCH_Z + 0.04), 5.0)
        _log(ctx, "tipped")
        yield from _both_move(ctx, (RX + 0.04, pl[1], TOP + 0.20), (RX + 0.04, pr[1], PINCH_Z + 0.04), 4.0)
        yield from ctx.both(_let_go(L, 1), _let_go(Rt, -1))
        yield from _end(ctx)
        return
    lead = 0.06 if mode == "uneven" else 0.0
    up = CARRY_Z
    yield from _both_move(ctx, (pl[0], pl[1], up + lead), (pr[0], pr[1], up), 6.0)
    _log(ctx, "lifted")
    drop_x = 0.5 * (TX + RX)
    if mode == "drop":
        yield from _both_move(ctx, (drop_x, pl[1], up), (drop_x, pr[1], up), 3.0)
        _log(ctx, "mid-carry")
        yield from L.grip(GRASP_OPEN, 0.3)
        _log(ctx, "left let go mid-carry")
        yield from L.move(STOW[1], yaw=0.0, duration=1.0)
        yield from _let_go(Rt, -1)
        yield from _end(ctx)
        return
    tx = RX
    # tier06: set down flat on the rack 4.5 cm right of its centre; tier03: 9.5 cm right, the cake's centre just past
    # the rack's right edge, so it rests tilted between the rack's edge and the table
    ty = {"tier06": -0.045, "tier03": -0.095}.get(mode, 0.0)
    yield from _both_move(ctx, (tx, pl[1] + ty, up + lead), (tx, pr[1] + ty, up), 6.0)
    _log(ctx, "over the rack")
    for _ in range(8):  # let the swing die out
        yield from L.wait(0.25)
        if float(np.linalg.norm(ctx.states()[CAKE.name].lin_vel)) < 0.01:
            break
    # lower until the cake sits on the rack (closed loop on the cake's lowest corner)
    for _ in range(80):
        st = ctx.states()
        c, rot = cake_pose(st)
        low = float(cake_corners(c, rot)[:, 2].min())
        gap = low - (RACK_TOP + 2 * H) if mode != "tier03" else low - (TOP + 2 * H)
        if gap < 0.002 and mode != "tier03":
            break
        if mode == "tier03" and (gap < 0.002 or tilt_deg(rot) > 8.0 or L.tcp()[2] < TOP + 0.06):
            break
        dz = -min(0.006, max(0.0015, 0.4 * gap))
        a, b = L.tcp(), Rt.tcp()
        yield from ctx.both(L.move((a[0], a[1], a[2] + dz), yaw=0.0, duration=0.1),
                            Rt.move((b[0], b[1], b[2] + dz), yaw=0.0, duration=0.1))
    a, b = L.tcp(), Rt.tcp()
    dz = 0.012 if mode != "tier03" else 0.0
    yield from ctx.both(L.move((a[0], a[1], a[2] - dz), yaw=0.0, duration=0.5),
                        Rt.move((b[0], b[1], b[2] - dz), yaw=0.0, duration=0.5))
    _log(ctx, "set down")
    if mode == "hold":  # never lets go
        yield from _end(ctx)
        return
    yield from ctx.both(_let_go(L, 1), _let_go(Rt, -1))
    yield from _end(ctx)
