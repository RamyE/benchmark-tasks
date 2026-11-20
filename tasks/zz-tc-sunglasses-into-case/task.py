"""Put sunglasses away: fold both temples, lay them in their clamshell case and snap it shut.

On the table in front of the arms a pair of Wayfarer-size sunglasses (50-22-150: 133 mm front, 41 mm lenses, 150 mm
temples) stands open on the lower rim of its front and the bent tips of its temples, lenses facing the robot. Each
temple turns on its hinge at the top corner of the front (hinge stiffness modelled as viscous damping) between its open stop
and folded flat behind the lenses; the first temple folded lies flat, the second comes to rest across the first one's
bent earpiece. Nearer the robot lies an open hard clamshell case (inside about 150 x 58 x 52 mm), its lid on a hinge
along the back edge with an over-centre spring: past about 40 deg it holds the lid open against its 100 deg stop,
below that it pulls the lid shut and keeps it shut. The case is free on the table (light, it slides when pushed).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "Fold the sunglasses and put them away in their case, then close it."

BLACK = (0.05, 0.05, 0.055)
LENS = (0.10, 0.12, 0.10)
CASE_OUT = (0.22, 0.13, 0.08)
CASE_IN = (0.55, 0.45, 0.30)

# -- sunglasses (Wayfarer 50-22-150 class) --------------------------------------------------------------------------
XF = 0.27  # x of the front's centre plane
FRONT_W, FRONT_T, FRONT_H = 0.133, 0.005, 0.050  # front: width (y), thickness (x), height (z; 41 mm lenses + brow bar)
HINGE_Y = 0.0615  # hinge points at y +-HINGE_Y
SETBACK = 0.010  # hinge axis this far behind the front's back face
HINGE_X = XF + FRONT_T / 2 + SETBACK
END_D = SETBACK + 0.002  # endpiece depth: to the hinge axis
TEMPLE_L, TEMPLE_T, TEMPLE_H = 0.105, 0.005, 0.009  # straight part of a temple (150 mm temple incl. the earpiece)
EAR_L, EAR_DEG = 0.045, 58.0  # earpiece: bent down this many degrees from the straight part
FRONT_MASS, END_MASS, TEMPLE_MASS, EAR_MASS = 0.030, 0.002, 0.005, 0.002
HINGE_DAMP = 0.005  # N m s / deg: the hinge's stiffness, as viscous damping (about 0.22 N m at 45 deg/s)
FOLD_LIM = 90.0  # folded flat behind the lenses
OPEN_LIM = 2.0  # open stop just past the spawn pose
FOLDED_DEG = 60.0  # a temple counts as folded when turned this far in (the second rests across the first, ~70 deg)

# hinge height: the earpiece tips stand on the table
HINGE_Z = TOP + EAR_L * math.sin(math.radians(EAR_DEG)) + (TEMPLE_H / 2) * math.cos(math.radians(EAR_DEG)) + 0.0005

ROT: dict = {}


def _quat_pitch(pitch: float) -> tuple[float, float, float, float]:
    """xyzw of a turn about +y (+ pitch dips the +x end up... standard right hand: +x end goes down for negative)."""
    return (0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2))


def _rotm(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


FRONT = Box("glasses_front", pos=(XF, 0.0, TOP + FRONT_H / 2 + 0.0005), size=(FRONT_T, FRONT_W, FRONT_H), mass=FRONT_MASS,
            color=LENS)
ENDS, TEMPLES, EARS = [], [], []
for _s, _n in ((1, "l"), (-1, "r")):
    ENDS.append(Box(f"endpiece_{_n}", pos=(XF + FRONT_T / 2 + END_D / 2, _s * (HINGE_Y + 0.001), HINGE_Z),
                    size=(END_D, 0.006, 0.011), mass=END_MASS, color=BLACK))
    TEMPLES.append(Box(f"temple_{_n}", pos=(HINGE_X + TEMPLE_L / 2 + 0.001, _s * HINGE_Y, HINGE_Z),
                       size=(TEMPLE_L, TEMPLE_T, TEMPLE_H), mass=TEMPLE_MASS, color=BLACK))
    _a = math.radians(EAR_DEG)
    _ex = HINGE_X + 0.001 + TEMPLE_L + (EAR_L / 2) * math.cos(_a) - 0.002
    _ez = HINGE_Z - (EAR_L / 2) * math.sin(_a)
    EARS.append(Box(f"earpiece_{_n}", pos=(_ex, _s * HINGE_Y, _ez), size=(EAR_L, TEMPLE_T, TEMPLE_H * 0.8),
                    mass=EAR_MASS, color=BLACK))
    ROT[f"earpiece_{_n}"] = _quat_pitch(_a)  # +pitch about y turns the +x end down
GLASSES = [FRONT, *ENDS, *TEMPLES, *EARS]

# -- clamshell case: long side along y, lid hinged along its back (+x) edge ---------------------------------------
CX, CY = 0.15, 0.0
C_L, C_W = 0.154, 0.062  # outside length (y), width (x)
WALL = 0.002
BASE_H, LID_H = 0.032, 0.024  # outside heights of the base and the lid shells (56 mm shut)
BASE_MASS, LID_MASS = 0.045, 0.030
LID_HINGE = np.array([CX + C_W / 2, CY, TOP + BASE_H])
LID_OPEN = 100.0  # open stop
LID0 = 99.0
OVER_C = 40.0  # over-centre point of the hinge spring
LID_K = 0.0006  # N m / deg
LID_DAMP = 0.0002
SHUT_DEG = 3.0
IN_LO = np.array([-C_W / 2 + WALL, -C_L / 2 + WALL, WALL])  # inside, case frame (origin: base centre at the table)
IN_HI = np.array([C_W / 2 - WALL, C_L / 2 - WALL, BASE_H + LID_H - WALL])


def _about_lid_hinge(p, deg: float):
    a = math.radians(deg)
    dx, dz = p[0] - LID_HINGE[0], p[2] - LID_HINGE[2]
    return (float(LID_HINGE[0] + dx * math.cos(a) + dz * math.sin(a)), float(p[1]),
            float(LID_HINGE[2] - dx * math.sin(a) + dz * math.cos(a)))


def _shell(prefix, z0, h, floor_at_bottom, color_out):
    """Open box shell (floor/top panel + 4 walls) with its panel at the bottom (base) or top (lid)."""
    zc = z0 + h / 2
    pz = z0 + WALL / 2 if floor_at_bottom else z0 + h - WALL / 2
    return [
        (f"{prefix}_panel", (CX, CY, pz), (C_W, C_L, WALL)),
        (f"{prefix}_near", (CX - C_W / 2 + WALL / 2, CY, zc), (WALL, C_L - 2 * WALL, h)),
        (f"{prefix}_far", (CX + C_W / 2 - WALL / 2, CY, zc), (WALL, C_L - 2 * WALL, h)),
        (f"{prefix}_endl", (CX, CY + C_L / 2 - WALL / 2, zc), (C_W, WALL, h)),
        (f"{prefix}_endr", (CX, CY - C_L / 2 + WALL / 2, zc), (C_W, WALL, h)),
    ]


BASE_PARTS = [Box(n, pos=c, size=s, mass=BASE_MASS / 5, color=CASE_OUT if "panel" in n or "near" in n else CASE_OUT)
              for n, c, s in _shell("case", TOP + 0.0002, BASE_H, True, CASE_OUT)]
LID_PARTS = []
for _n, _c, _s in _shell("lid", TOP + BASE_H + 0.0003, LID_H, False, CASE_OUT):
    LID_PARTS.append(Box(_n, pos=_about_lid_hinge(_c, LID0), size=_s, mass=LID_MASS / 5, color=CASE_OUT))
    ROT[_n] = _quat_pitch(math.radians(LID0))
CASE_BASE = BASE_PARTS[0]
LID = LID_PARTS[0]


class JointHost(Box):
    """A small fixed marker block under the table edge; its spawner authors all the joints."""


HOST = JointHost("joint_host", pos=(0.05, 0.40, TOP - 0.02), size=(0.01, 0.01, 0.01), color=BLACK)

OBJECTS = [*GLASSES, *BASE_PARTS, *LID_PARTS, HOST]

TWIN_EXTRA = [
    {"name": "temple_hinges", "kind": "joint", "real": "sunglasses temple hinges", "note": "revolute 0-90 deg, damped"},
    {"name": "case_lid", "kind": "joint", "real": "clamshell case spring hinge", "note": "revolute 0-100 deg; over-centre "
     "spring: above 40 deg pushes open, below pulls shut"},
]

_P: dict = {}


def _author_joints(stage, parent: str) -> None:
    from pxr import Gf, Sdf, UsdPhysics

    q0 = Gf.Quatf(1.0, 0.0, 0.0, 0.0)

    def v(p):
        return Gf.Vec3f(*(float(x) for x in p))

    def path(o):
        return Sdf.Path(f"{parent}/{o.name}")

    def local(o, p_world):
        d = np.asarray(p_world, float) - np.array(o.pos)
        q = ROT.get(o.name)
        return tuple(d) if q is None else tuple(_rotm(q).T @ d)

    def lrot(o):
        q = ROT.get(o.name)
        if q is None:
            return q0
        qi = (-q[0], -q[1], -q[2], q[3])  # body frame -> world-aligned joint frame
        return Gf.Quatf(float(qi[3]), float(qi[0]), float(qi[1]), float(qi[2]))

    def joint(kind, name, b0, b1, p_world, axis="Z", lo=None, hi=None):
        j = kind.Define(stage, f"{parent}/{HOST.name}/{name}")
        j.CreateBody0Rel().SetTargets([path(b0)])
        j.CreateLocalPos0Attr(v(local(b0, p_world)))
        j.CreateBody1Rel().SetTargets([path(b1)])
        j.CreateLocalPos1Attr(v(local(b1, p_world)))
        j.CreateLocalRot0Attr(lrot(b0))
        j.CreateLocalRot1Attr(lrot(b1))
        if kind is not UsdPhysics.FixedJoint:
            j.CreateAxisAttr(axis)
            if lo is not None:
                j.CreateLowerLimitAttr(lo)
                j.CreateUpperLimitAttr(hi)
        j.CreateExcludeFromArticulationAttr(True)
        return j

    def drive(j, kind, k, c, target=0.0):
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(k)
        d.CreateDampingAttr(c)
        d.CreateTargetPositionAttr(target)
        return d

    for e in ENDS:
        joint(UsdPhysics.FixedJoint, f"weld_{e.name}", FRONT, e, e.pos)
    for s, e, t, ear in zip((1, -1), ENDS, TEMPLES, EARS):
        lo, hi = (-FOLD_LIM, OPEN_LIM) if s > 0 else (-OPEN_LIM, FOLD_LIM)
        j = joint(UsdPhysics.RevoluteJoint, f"hinge_{t.name}", e, t, (HINGE_X, s * HINGE_Y, HINGE_Z), lo=lo, hi=hi)
        drive(j, "angular", 0.0, HINGE_DAMP)
        joint(UsdPhysics.FixedJoint, f"weld_{ear.name}", t, ear, ear.pos)
    for b in BASE_PARTS[1:]:
        joint(UsdPhysics.FixedJoint, f"weld_{b.name}", CASE_BASE, b, b.pos)
    for b in LID_PARTS[1:]:
        joint(UsdPhysics.FixedJoint, f"weld_{b.name}", LID, b, b.pos)
    # lid hinge: joint frames world-aligned at spawn, so the joint reads 0 at LID0; limits shifted accordingly.
    # +rotation about world y opens the lid; the joint angle a_j = -(lid opening - LID0) with axis Y? use axis Y and
    # read the lid angle from the bodies' poses instead (lid_angle), limits in joint terms.
    j = joint(UsdPhysics.RevoluteJoint, "hinge_lid", CASE_BASE, LID, LID_HINGE, axis="Y", lo=-LID0 - 1.0,
              hi=LID_OPEN - LID0)
    drive(j, "angular", LID_K, LID_DAMP, LID_OPEN - LID0 + 5.0)
    _P["hinge_lid"] = str(j.GetPath())


def _spawn_host(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_cuboid(prim_path, cfg, translation, orientation, **kwargs)
    _author_joints(get_current_stage(), str(prim_path).rsplit("/", 1)[0])
    return prim


def _host_object(spec: JointHost):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import CollisionPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.station import linear_rgb

    _register_module()

    @configclass
    class HostCfg(sim_utils.CuboidCfg):
        func: object = clone(_spawn_host)

    cfg = HostCfg(size=tuple(spec.size), collision_props=CollisionPropertiesCfg(collision_enabled=False),
                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.4))
    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _register_module():
    if __name__ not in sys.modules:
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _patch_scene(scene) -> None:
    if getattr(scene, "_glasses_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, JointHost):
            return _host_object(spec)
        q = ROT.get(spec.name)
        if q is None:
            return orig(spec)
        yaw_q = scene._yaw_xyzw
        scene._yaw_xyzw = lambda _yaw: q
        try:
            return orig(spec)
        finally:
            scene._yaw_xyzw = yaw_q

    scene.task_object = task_object
    scene._glasses_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _glasses = True

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
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
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


def _install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(getattr(f, "_glasses", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()

_ENV: dict = {"env": None}


class _Episode:
    def __init__(self):
        self.spring_open = None  # last over-centre spring state written
        self.lens_down = False  # the glasses have been knocked over on the table at some point (latched)
        self.open_in = False  # part of the glasses went into the case while a temple was not folded (latched)
        self.spin = []  # the front's angular speed (deg/s) over the last FALL_STEPS control steps
        self.was_flat = False  # the front was lying tipped over on the table at the previous step
        self.grip_hist = []  # gripper points at the last PARK_STEPS + 1 control steps


_EP = _Episode()


def _stage():
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage()


def reset_episode() -> None:
    global _EP
    _EP = _Episode()
    if not _P:
        return
    try:
        _set_spring(True)
    except Exception:  # noqa: BLE001
        pass


def _set_spring(open_: bool) -> None:
    """Over-centre spring: above OVER_C it pushes the lid onto its open stop, below it pulls the lid shut."""
    if _EP.spring_open == open_:
        return
    _EP.spring_open = open_
    d = _stage().GetPrimAtPath(_P["hinge_lid"])
    d.GetAttribute("drive:angular:physics:targetPosition").Set(
        float(LID_OPEN - LID0 + 5.0) if open_ else float(-LID0 - 5.0))


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_glasses_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._glasses_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_glasses_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            out = base(self, *a, **k)
            try:
                from yam_arena.scene import object_states

                st = object_states(self._env, [CASE_BASE.name, LID.name] + [g.name for g in GLASSES])
                a_ = lid_angle(st)
                if math.isfinite(a_) and _P:
                    _set_spring(a_ > OVER_C)
                w = st[FRONT.name].ang_vel
                _EP.spin = (_EP.spin + [math.degrees(float(np.linalg.norm(w))) if _finite(w) else 0.0])[-FALL_STEPS:]
                gp = gripper_points()
                if gp:
                    _EP.grip_hist = (_EP.grip_hist + [np.array(gp, dtype=np.float64)])[-(PARK_STEPS + 1):]
                flat = lens_on_table(st)
                if flat and not _EP.was_flat and max(_EP.spin) > FALL_RATE:
                    _EP.lens_down = True  # it fell over (tipped fast), not laid down
                _EP.was_flat = flat
                if not _EP.open_in and entering(st) and not folded(st):
                    _EP.open_in = True
            except Exception as e:  # noqa: BLE001
                print(f"[glasses] spring step failed: {e!r}", flush=True)
            return out

        _task_step._glasses_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- readouts ----------------------------------------------------------------------------------------------------------
def _finite(a) -> bool:
    return a is not None and bool(np.isfinite(np.asarray(a, dtype=float)).all())


def _rel(objects, a_name, b_name) -> np.ndarray | None:
    a, b = objects[a_name], objects[b_name]
    if not (_finite(a.quat_xyzw) and _finite(b.quat_xyzw)):
        return None
    ra = _rotm(a.quat_xyzw)
    rb = _rotm(b.quat_xyzw)
    qb0 = ROT.get(b_name)
    qa0 = ROT.get(a_name)
    rb = rb @ (_rotm(qb0).T if qb0 is not None else np.eye(3))  # undo the spawn tilt: frames world-aligned at spawn
    ra = ra @ (_rotm(qa0).T if qa0 is not None else np.eye(3))
    return ra.T @ rb


def temple_fold(objects, i: int) -> float:
    """How far (deg) temple i (0 left, 1 right) is turned in from open: 0 open, 90 folded flat behind the lenses."""
    m = _rel(objects, FRONT.name, TEMPLES[i].name)
    if m is None:
        return float("nan")
    yaw = math.degrees(math.atan2(m[1, 0], m[0, 0]))
    return -yaw if i == 0 else yaw


def lid_angle(objects) -> float:
    """Lid opening (deg): 0 shut, 100 at its open stop."""
    m = _rel(objects, CASE_BASE.name, LID.name)
    if m is None:
        return float("nan")
    return LID0 + math.degrees(math.atan2(m[0, 2], m[0, 0]))


def _corners(spec, state) -> np.ndarray | None:
    if not (_finite(state.pos) and _finite(state.quat_xyzw)):
        return None
    h = np.array(spec.size) / 2
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
    return np.asarray(state.pos, float) + (signs * h) @ _rotm(state.quat_xyzw).T


def _case_frame(objects):
    """(origin, rotation) of the case: origin at the base's centre on its underside."""
    b = objects[CASE_BASE.name]
    if not (_finite(b.pos) and _finite(b.quat_xyzw)):
        return None, None
    r = _rotm(b.quat_xyzw)
    o = np.asarray(b.pos, float) - r @ np.array([0.0, 0.0, WALL / 2])
    return o, r


TOL = 0.002


def glasses_inside(objects) -> bool:
    """Every corner of every part of the sunglasses inside the case's inner space (base and lid), 2 mm tolerance."""
    o, r = _case_frame(objects)
    if o is None:
        return False
    for spec in GLASSES:
        c = _corners(spec, objects[spec.name])
        if c is None:
            return False
        loc = (c - o) @ r
        if not (np.all(loc >= IN_LO - TOL) and np.all(loc <= IN_HI + TOL)):
            return False
    return True


def entering(objects) -> bool:
    """Some corner of some part of the sunglasses is in the case's base: within its inner footprint and below its rim."""
    o, r = _case_frame(objects)
    if o is None:
        return False
    for spec in GLASSES:
        c = _corners(spec, objects[spec.name])
        if c is None:
            continue
        loc = (c - o) @ r
        m = (np.abs(loc[:, 0]) <= C_W / 2 - WALL) & (np.abs(loc[:, 1]) <= C_L / 2 - WALL) & (loc[:, 2] <= BASE_H)
        if bool(m.any()):
            return True
    return False


LENS_TILT = 60.0  # deg from upright


FALL_RATE = 150.0  # deg/s: the front tipping faster than this on its way down = it fell (a hand lays it down slower)
FALL_STEPS = 10  # look back 0.5 s (20 Hz) for the fall speed when the front comes to lie tipped over


def lens_on_table(objects) -> bool:
    """The front lies tipped over on the table: more than 60 deg from upright (onto its lenses or onto
    its back), low on the table and outside the case's footprint."""
    f = objects[FRONT.name]
    if not (_finite(f.pos) and _finite(f.quat_xyzw)):
        return False
    n = -_rotm(f.quat_xyzw)[:, 0]  # outward normal of the lenses
    if abs(float(n[2])) < math.sin(math.radians(LENS_TILT)) or float(f.pos[2]) > TOP + 0.015:
        return False
    o, r = _case_frame(objects)
    if o is not None:
        loc = (np.asarray(f.pos, float) - o) @ r
        if abs(loc[0]) <= C_W / 2 and abs(loc[1]) <= C_L / 2:
            return False
    return True


def folded(objects) -> bool:
    return all(temple_fold(objects, i) >= FOLDED_DEG for i in range(2))


def shut(objects) -> bool:
    a = lid_angle(objects)
    return bool(math.isfinite(a) and a <= SHUT_DEG)


def on_table(objects) -> bool:
    """Neither the sunglasses nor the case has fallen off the table."""
    for n in (FRONT.name, CASE_BASE.name):
        p = objects[n].pos
        if not _finite(p) or float(p[2]) < TOP - 0.01:
            return False
    return True


def _robots():
    env = _ENV["env"]
    if env is None:
        return None, None
    base = env.unwrapped
    return base, base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)


def gripper_points() -> list[np.ndarray]:
    base, origin = _robots()
    if base is None:
        return []
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    pts = []
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        pts += [np.asarray(pad_tips_world(rob), dtype=np.float64) - origin,
                np.asarray(tcp_world(rob), dtype=np.float64) - origin]
        lp = link_poses(rob)
        for n in ("tip_left", "tip_right"):
            if n in lp:
                pts.append(np.asarray(lp[n][:3, 3], dtype=np.float64) - origin)
    return pts


CLEAR = 0.05
PARK = 0.10  # parked: every gripper point at least 10 cm from the glasses' and the case's boxes ...
PARK_SPEED = 0.02  # ... moving under 2 cm/s ...
PARK_STEPS = 10  # ... over the last 0.5 s (10 control steps at 20 Hz)
CONTROL_HZ = 20.0


def _boxes(objects):
    """(lo, hi) world bounding boxes of the sunglasses and of the case (base and lid), or None."""
    boxes = []
    for group in (GLASSES, [*BASE_PARTS, *LID_PARTS]):
        cs = []
        for spec in group:
            c = _corners(spec, objects[spec.name])
            if c is None:
                return None
            cs.append(c)
        allc = np.vstack(cs)
        boxes.append((allc.min(axis=0), allc.max(axis=0)))
    return boxes


def parked(objects) -> bool:
    """Both arms parked: over the last 0.5 s every gripper point stayed 10 cm or more from the glasses' box and the
    case's box and moved under 2 cm/s."""
    hist = _EP.grip_hist
    boxes = _boxes(objects)
    if len(hist) < PARK_STEPS + 1 or boxes is None:
        return False
    for k, frame in enumerate(hist):
        if not _finite(frame):
            return False
        for p in frame:
            for lo, hi in boxes:
                if float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0))) < PARK:
                    return False
        if k and float(np.linalg.norm(frame - hist[k - 1], axis=1).max()) * CONTROL_HZ >= PARK_SPEED:
            return False
    return True


def released(objects) -> bool:
    """Every gripper point 5 cm or more from the sunglasses' bounding box and from the case's (base and lid) bounding
    box, each taken separately."""
    pts = gripper_points()
    if not pts:
        return False
    boxes = []
    for group in (GLASSES, [*BASE_PARTS, *LID_PARTS]):
        cs = []
        for spec in group:
            c = _corners(spec, objects[spec.name])
            if c is None:
                return False
            cs.append(c)
        allc = np.vstack(cs)
        boxes.append((allc.min(axis=0), allc.max(axis=0)))
    for p in pts:
        if not _finite(p):
            return False
        for lo, hi in boxes:
            if float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0))) < CLEAR:
                return False
    return True


def score(objects) -> float:
    """0 if the sunglasses were knocked over on the table at any time, or went into the case with a temple not
    folded; 0 unless both the sunglasses and the case are
    on the table and every gripper point is 5 cm clear of them;
    then 1.0 both temples folded, the glasses wholly inside the case, the lid shut and both arms parked (10 cm clear,
    under 2 cm/s for 0.5 s); 0.5 folded and inside, but the lid not shut or the arms not parked; 0.25 folded, not
    inside; otherwise 0."""
    if _EP.lens_down or _EP.open_in or not on_table(objects) or not released(objects):
        return 0.0
    if not folded(objects):
        return 0.0
    if not glasses_inside(objects):
        return 0.25
    return 1.0 if shut(objects) and parked(objects) else 0.5


def success(objects) -> bool:
    return bool(score(objects) == 1.0)


def distance(objects) -> float:
    f = [temple_fold(objects, i) for i in range(2)]
    a = lid_angle(objects)
    if not all(math.isfinite(x) for x in (*f, a)):
        return 1.0
    return float(sum(max(0.0, FOLDED_DEG - x) for x in f) / 90.0 + 0.5 * (not glasses_inside(objects)) + a / 100.0)


def progress(objects) -> dict:
    return {"fold_l": temple_fold(objects, 0), "fold_r": temple_fold(objects, 1), "lid_deg": lid_angle(objects),
            "inside": float(glasses_inside(objects)), "lens_down": float(_EP.lens_down), "open_in": float(_EP.open_in), "released": float(released(objects)), "parked": float(parked(objects))}


VARIATION: dict = {}
