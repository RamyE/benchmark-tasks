"""On the station table a cable drum (grey hub, 5 cm across, 7 cm tall, with a white bar on top that shows it
turning) stands on a vertical axle at the left, 29 cm in front of the robot. A red 6 mm multicore, 64 cm long, is
wound 2.5 turns round the hub (inner end fixed to the hub); 15 cm of it plus its 35 mm connector lies out on the
table, pointing toward the right along a yellow-taped cable run (two tape lines 8 cm apart). At the far end of the
run, in front of the right arm, is a white cradle the connector must rest in. The drum turns freely with light drag,
so a fast pull spins it and it keeps turning for a moment after the pull stops. Only the left arm reaches the
connector at the start and only the right arm reaches the cradle, so the cable has to change hands once, near the
middle of the table, while the drum must never turn with the cable unheld.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Pay the red cable off the drum along the yellow-taped run and rest its connector in the white "
               "cradle, then let go.")

# -- layout (m; x forward from the robot, y to the robot's left) ------------------------------------------------------
GREY, WHITE, YELLOW, RED = (0.45, 0.47, 0.50), (0.93, 0.93, 0.90), (0.95, 0.80, 0.10), (0.78, 0.08, 0.06)
HUB_XY = (0.29, 0.31)
HUB_R, HUB_H = 0.025, 0.07
HUB_MASS = float(os.environ.get("DRUM_MASS", "0.4"))
DRUM_DAMP = float(os.environ.get("DRUM_DAMP", "2e-5"))  # axle drag, N m s per degree
DRUM = Cylinder("drum", pos=(*HUB_XY, TOP + HUB_H / 2 + 0.001), radius=HUB_R, height=HUB_H, color=GREY,
                mass=HUB_MASS)
FL_R, FL_H = 0.06, 0.006  # top flange: gives the drum the inertia of a real cable drum (it coasts when let go)
FLANGE_MASS = float(os.environ.get("DRUM_FLANGE_MASS", "0.3"))
FLANGE = Cylinder("drum_flange", pos=(*HUB_XY, TOP + HUB_H + 0.001 + 0.0005 + FL_H / 2), radius=FL_R, height=FL_H,
                  color=GREY, mass=FLANGE_MASS)
MARK_SIZE = (0.11, 0.012, 0.003)
DRUM_MARK = Box("drum_mark", pos=(HUB_XY[0], HUB_XY[1], FLANGE.pos[2] + FL_H / 2 + MARK_SIZE[2] / 2 + 0.0005),
                size=MARK_SIZE, color=WHITE, mass=0.004)

R = 0.003  # 6 mm multicore
TURNS = 2.5
CABLE_LEN = 0.64
CONN = Plug(size=(0.035, 0.016, 0.012), mass=0.015, color=(0.12, 0.12, 0.12))
HEAD = -1.4  # heading of the cable run (rad): mostly toward the right arm, slightly away from the robot
U = np.array([math.cos(HEAD), math.sin(HEAD)])
NV = np.array([-U[1], U[0]])  # left-hand normal of the run
THETA_T = math.atan2(U[0], -U[1])  # hub angle where a clockwise-wound cable leaves along U
_Z = TOP + R + 0.0005
_proto = cable("multicore", start=(0.20, 0.20, _Z), yaw=0.0, length=CABLE_LEN, radius=R)
WIND = dict(start_angle=THETA_T + math.pi, sense=-1)  # clockwise from above; ends at THETA_T, tail along U
WPATH = cableshape.wind_path(_proto, (*HUB_XY, HUB_R), TURNS, **WIND)
SP = cableshape.spawn_for(_proto, WPATH)
CABLE = cable("multicore", start=SP["start"], yaw=SP["yaw"], length=CABLE_LEN, radius=R, end_plug=CONN,
              color=RED, bend_stiffness=0.003, bend_damping=0.0005)
WIND_R = float(np.hypot(*(WPATH[0, :2] - np.array(HUB_XY))))  # centreline radius of the wound turns
T0 = np.array(HUB_XY) + WIND_R * np.array([math.cos(THETA_T), math.sin(THETA_T)])  # where the run leaves the hub


def run_pt(s: float, off: float = 0.0) -> np.ndarray:
    """Point on the run centreline s m from the hub's leaving point, off m to its left."""
    return T0 + s * U + off * NV


# yellow tape: two 10 mm lines, inner edges 4 cm either side of the run centreline
TAPE_S0, TAPE_S1 = 0.07, 0.52
CORR_HALF = 0.04
TAPE = [Box(f"tape_{k}", pos=(*run_pt((TAPE_S0 + TAPE_S1) / 2, sg * (CORR_HALF + 0.005)), TOP + 0.0005),
            size=(TAPE_S1 - TAPE_S0, 0.010, 0.001), yaw=HEAD, color=YELLOW) for k, sg in (("l", 1), ("r", -1))]

# cradle: open-top seat for the connector at the far end of the run (axis along the run). Low corner posts leave the
# middle of each side open for the fingers; the near wall has a 10 mm slot for the cable (the 16 mm connector can't
# pass it, so it must be lowered in from above).
CR_S = 0.555
CR_C = run_pt(CR_S)
IN_HALF_U, IN_HALF_V, WALL_H, WT = 0.024, 0.011, 0.012, 0.006


def _cr(name, u, v, su, sv):
    p = CR_C + u * U + v * NV
    return Box(name, pos=(p[0], p[1], TOP + WALL_H / 2), size=(su, sv, WALL_H), yaw=HEAD, color=WHITE)


CRADLE = [
    _cr("cradle_end", IN_HALF_U + WT / 2, 0.0, WT, 2 * (IN_HALF_V + WT)),
    _cr("cradle_slot_l", -IN_HALF_U - WT / 2, (0.005 + IN_HALF_V + WT) / 2, WT, IN_HALF_V + WT - 0.005),
    _cr("cradle_slot_r", -IN_HALF_U - WT / 2, -(0.005 + IN_HALF_V + WT) / 2, WT, IN_HALF_V + WT - 0.005),
    *[_cr(f"cradle_post_{a}{b}", su * (IN_HALF_U - 0.0045), sv * (IN_HALF_V + WT / 2), 0.009, WT)
      for a, su in (("f", 1), ("n", -1)) for b, sv in (("l", 1), ("r", -1))],
]

OBJECTS = [DRUM, FLANGE, DRUM_MARK, *TAPE, *CRADLE, CABLE]

TWIN_EXTRA = [
    {"name": "drum_axle", "kind": "joint", "real": "608ZZ bearing on an M8 bolt in a printed base",
     "note": "drum turns about the vertical with light drag (~1e-3 N m s/rad); the hub weighs 0.4 kg"},
    {"name": "hub_anchor", "kind": "joint", "real": "inner cable end clamped in a slot on the hub top",
     "note": "the cable's inner end is fixed to the hub"},
    {"name": "drum_flange", "kind": "body", "size_mm": [120, 120, 6], "colour": "grey", "pose": [0.29, 0.31, 0.075],
     "note": "steel or printed disc fixed on the hub top, 0.3 kg (drum inertia)"},
    {"name": "drum_mark", "kind": "body", "size_mm": [110, 12, 3], "colour": "white", "pose": [0.29, 0.31, 0.08],
     "note": "bar glued across the flange, shows the drum turning"},
]


# -- the drum: revolute z axle world->hub, the cable's inner end and the mark bar fixed to the hub ------------------
def _install_drum() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_payout_drum", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CABLE.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        hub = f"{root}/{DRUM.name}"
        w = UsdGeom.Xformable(stage.GetPrimAtPath(root)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        h = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/drum_axle")
        h.CreateBody1Rel().SetTargets([Sdf.Path(hub)])
        h.CreateAxisAttr("Z")
        h.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + DRUM.pos[0]), float(w[1] + DRUM.pos[1]),
                                       float(w[2] + DRUM.pos[2])))
        h.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        h.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        h.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        h.CreateExcludeFromArticulationAttr(True)
        drv = UsdPhysics.DriveAPI.Apply(h.GetPrim(), "angular")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(0.0)
        drv.CreateDampingAttr(DRUM_DAMP)
        j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/hub_weld")
        j.CreateBody0Rel().SetTargets([Sdf.Path(hub)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        off = np.asarray(SP["seg0_pos"]) - np.asarray(DRUM.pos)
        x, y, z, qw = (float(v) for v in SP["seg0_quat_xyzw"])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, off)))
        j.CreateLocalRot0Attr(Gf.Quatf(qw, x, y, z))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        for part in (FLANGE, DRUM_MARK):
            m = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/{part.name}_weld")
            m.CreateBody0Rel().SetTargets([Sdf.Path(hub)])
            m.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{part.name}")])
            m.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, float(part.pos[2] - DRUM.pos[2])))
            m.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            m.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            m.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            m.CreateExcludeFromArticulationAttr(True)
        return prim

    spawn._payout_drum = True
    cablekit._spawn_cable = spawn


_install_drum()

# -- pre-wound start (coil set while on the drum) ---------------------------------------------------------------------
_SH: dict = {}


def _shape(emb):
    _SH["sh"] = cableshape.prewind(emb, CABLE, (*HUB_XY, HUB_R), TURNS, hold="set", settle_s=0, **WIND)
    _SH["relaxed"] = None
    cableshape.settle(emb, 2.0)


cableshape.on_reset(_shape)

OFF_DRUM = WIND_R + 0.015  # a bend joint whose two segments both lie this far from the axle has left the drum


def _relax_off_drum(env) -> None:
    """Coil set only while on the drum: once both segments at a bend joint have left the drum, that joint's rest
    angle goes to straight (latched), so the paid-out cable lies straight on the table like a real multicore."""
    sh = _SH.get("sh")
    if sh is None or os.environ.get("DRUM_RELAX", "1") == "0":
        return
    import torch
    from cablekit import _cable_state

    pts = _cable_state(env, CABLE).points
    if not np.isfinite(pts).all():
        return
    d = np.hypot(pts[:, 0] - HUB_XY[0], pts[:, 1] - HUB_XY[1])
    off = (d[:-1] > OFF_DRUM) & (d[1:] > OFF_DRUM)  # joint i+1 between seg i and seg i+1
    rel = _SH["relaxed"]
    rel = off.copy() if rel is None else (rel | off)
    if _SH["relaxed"] is not None and np.array_equal(rel, _SH["relaxed"]):
        return
    _SH["relaxed"] = rel
    base = env.unwrapped
    art = base.scene[CABLE.name]
    i1, i2 = cableshape._dof_index(art)
    q = torch.zeros_like(cableshape._t(art.data.joint_pos))
    keep = (~rel).astype(np.float64)[:, None] * sh.q
    q[0, i1] = torch.tensor(keep[:, 0], dtype=q.dtype, device=base.device)
    q[0, i2] = torch.tensor(keep[:, 1], dtype=q.dtype, device=base.device)
    art.set_joint_position_target_index(target=q)


# -- env handle (gripper TCPs and finger openings), per-episode memory --------------------------------------------
_ENV = {"env": None, "step": 0}
_MON = {"violated": "", "last_step": -1, "max_rate": 0.0, "max_loose": 0.0, "both_held": False, "handover": False}


def _reset_episode() -> None:
    _MON.update(violated="", last_step=-1, max_rate=0.0, max_loose=0.0, both_held=False, handover=False)
    _ENV["step"] = 0


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline import)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_drum_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            _ENV["step"] += 1
            _relax_off_drum(self._env)
            return base(self, *a, **k)

        _task_step._drum_hooked = True
        cls._task_step = _task_step
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._task_reset_hooks = []
        cls.reset = reset
    if _reset_episode not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_episode)


_install_hooks()

RELEASE_DIST = 0.030  # at the end every TCP at least 3 cm from the cable and the connector
MAX_RATE = 2 * math.pi  # rad/s: one drum turn per second
FREE_RATE = math.radians(20.0)  # rad/s: the drum counts as turning above 20 deg/s
STILL_RATE = math.radians(3.0)  # rad/s: the drum counts as still below 3 deg/s
LOOSE_MAX = 0.02  # m: a wound turn may not stand further than this off its wound radius

# A grasp is a pinch, not proximity: the fingers must be stopped by the part. The gap between the finger pads
# (from the two finger joints) must sit at the part's width across the closing axis (within GAP_UNDER below it,
# fingers pressing in, and GAP_OVER above it) AND the part must lie between the pads (pad box in the gripper frame).
# A gripper closed on nothing closes to ~0 mm; a gripper merely hovering near the cable has no cable between its pads.
PAD_UP, PAD_DOWN = 0.015, 0.013  # pad extent above / below the TCP along the approach axis
PAD_HALF_W = 0.010  # half pad width (along the axis normal to both approach and closing)
GAP_UNDER, GAP_OVER = 0.004, 0.002
GAP_PAD = 0.0033  # finger-joint gap minus pad-to-pad distance (measured: 19.3 mm on the 16 mm connector)
GAP_MIN = 0.002  # below this the fingers are shut on nothing
MID_TOL = 0.010  # the pinched connector centre within 10 mm of the tool point across the fingers (the two fingers
                 # can stop off-centre: measured up to 7 mm on a 16 mm connector pinched at 16.0 mm)


def _t(x):
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy()


def hands() -> list[tuple]:
    """Per arm (left, right): (tcp, approach axis, closing axis, width axis, finger gap m) in env coordinates;
    [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.analysis import pose7_to_matrix
    from yam_arena.constants import TCP_OFFSET

    base = env.unwrapped
    origin = _t(base.scene.env_origins)[0].astype(np.float64)
    out = []
    for k in ("left_robot", "right_robot"):
        art = base.scene[k]
        names = list(art.body_names)
        poses = _t(art.data.body_link_pose_w)[0].astype(np.float64)
        g = pose7_to_matrix(poses[names.index("gripper")])
        tcp = g[:3, :3] @ np.array(TCP_OFFSET) + g[:3, 3] - origin
        a = g[:3, 2]
        # the tip_left/tip_right body origins are offset along the pad width; the fingers close across that offset
        # (measured: a 16 mm wide connector pinched across reads a 16.1 mm gap along this axis)
        t = poses[names.index("tip_left")][:3] - poses[names.index("tip_right")][:3]
        t = t - a * (t @ a)
        t = t / (np.linalg.norm(t) + 1e-12)
        c = np.cross(a, t)
        jn = list(art.joint_names)
        q = _t(art.data.joint_pos)[0]
        gap = -float(q[jn.index("joint7")] + q[jn.index("joint8")]) - GAP_PAD
        out.append((tcp, a, c, t, gap))
    return out


def grippers() -> list[tuple[np.ndarray, float]]:
    """(TCP position in env frame, pad-to-pad gap m) per arm; [] offline."""
    return [(h[0], h[4]) for h in hands()]


def _dense(objects, step: float = 0.002) -> np.ndarray:
    poly = objects[CABLE.name].polyline()
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(math.ceil(float(np.linalg.norm(b - a)) / step)))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.asarray(out)


def _conn_axes(objects) -> np.ndarray | None:
    """Connector body axes (rows: along the cable, across, up) from the connector body's own pose in the sim (it is
    welded to the last cable segment, so it can sit at an angle to the cable further back); None if unreadable."""
    env = _ENV["env"]
    if env is None:
        return None
    art = env.unwrapped.scene[CABLE.name]
    names = list(art.body_names)
    if "end_plug" not in names:
        return None
    w, x, y, z = (float(v) for v in _t(art.data.body_quat_w)[0][names.index("end_plug")])
    m = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return m.T if np.isfinite(m).all() else None


def grasp(objects, h) -> str:
    """'connector', 'cable' or '' for one hand h = (tcp, a, c, w, gap): what the closed fingers are pinching."""
    tcp, a, cax, w, gap = h
    if not (np.isfinite(tcp).all() and np.isfinite(gap)) or gap < GAP_MIN:
        return ""
    c = objects[CABLE.name]
    ax = _conn_axes(objects)
    if ax is not None and c.end_plug is not None and np.isfinite(c.end_plug).all():
        half = np.asarray(CONN.size) / 2
        width = float(2 * np.sum(np.abs(ax @ cax) * half))  # connector extent across the closing axis
        d = np.asarray(c.end_plug) - tcp
        ext_a, ext_w = float(np.sum(np.abs(ax @ a) * half)), float(np.sum(np.abs(ax @ w) * half))
        if os.environ.get("DRUM_GDEBUG") and gap < 0.05:
            print(f"[grasp] conn width={width*1000:.1f} gap={gap*1000:.1f} dc={d @ cax*1000:.1f} da={d @ a*1000:.1f} "
                  f"(ext {ext_a*1000:.1f}) dw={d @ w*1000:.1f} (ext {ext_w*1000:.1f}) width_w={2*ext_w*1000:.1f}", flush=True)
        if (width - GAP_UNDER <= gap <= width + GAP_OVER and abs(d @ cax) <= MID_TOL
                and -PAD_UP - ext_a <= d @ a <= PAD_DOWN + ext_a and abs(d @ w) <= PAD_HALF_W + ext_w):
            return "connector"
    if 2 * R - GAP_UNDER <= gap <= 2 * R + GAP_OVER:
        d = _dense(objects) - tcp
        da, dc, dw = d @ a, d @ cax, d @ w
        if os.environ.get("DRUM_GDEBUG"):
            m = (da >= -PAD_UP) & (da <= PAD_DOWN) & (np.abs(dw) <= PAD_HALF_W)
            print(f"[grasp] cable gap={gap*1000:.1f} inbox={int(m.sum())} min|dc|={np.abs(dc[m]).min()*1000 if m.any() else -1:.1f}", flush=True)
        if np.any((da >= -PAD_UP) & (da <= PAD_DOWN) & (np.abs(dc) <= gap / 2 + 0.001) & (np.abs(dw) <= PAD_HALF_W)):
            return "cable"
    return ""


def holders(objects) -> list[bool]:
    """Per arm: the fingers are pinching the cable or the connector (see grasp())."""
    return [bool(grasp(objects, h)) for h in hands()]


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _cable_dist(objects, g) -> float:
    c = objects[CABLE.name]
    poly = c.polyline()
    d = min(_seg_dist(g, u, v) for u, v in zip(poly[:-1], poly[1:]))
    if c.end_plug is not None:
        p, a = np.asarray(c.end_plug), np.asarray(c.end_dir)
        d = min(d, _seg_dist(g, p - a * CONN.size[0] / 2, p + a * CONN.size[0] / 2))
    return d


def released(objects) -> bool:
    return all(_cable_dist(objects, g) >= RELEASE_DIST for g, _ in grippers())


def drum_rate(objects) -> float:
    return float(abs(objects[DRUM.name].ang_vel[2]))


def loose_turn(objects) -> float:
    """How far (m) the cable stands off the wound radius anywhere before its last point on the hub: a loop thrown off
    by an overrunning drum, with cable further along still wound on."""
    pts = objects[CABLE.name].points
    d = np.hypot(pts[:, 0] - HUB_XY[0], pts[:, 1] - HUB_XY[1])
    on = np.where(d < WIND_R + 0.01)[0]
    return float(d[: on.max() + 1].max() - WIND_R) if len(on) else 0.0


def _finite(objects) -> bool:
    c = objects[CABLE.name]
    return bool(np.isfinite(c.points).all() and c.end_plug is not None and np.isfinite(c.end_plug).all())


def _local(p) -> tuple[float, float]:
    d = np.asarray(p[:2]) - CR_C
    return float(d @ U), float(d @ NV)


def in_cradle(objects) -> bool:
    """Connector centre between the cradle's walls (inside its 48 x 22 mm seat), lying on the table."""
    c = objects[CABLE.name]
    p = np.asarray(c.end_plug)
    u, v = _local(p)
    return bool(abs(u) <= IN_HALF_U - CONN.size[0] / 2 + 0.004 and abs(v) <= IN_HALF_V - CONN.size[1] / 2 + 0.003
                and p[2] <= TOP + CONN.size[2] / 2 + 0.006)


def in_corridor(objects) -> bool:
    """Every cable point more than 6 cm from the drum axle lies within 4 cm of the run centreline (between the tapes)
    and before the cradle's far wall."""
    pts = objects[CABLE.name].points
    d = np.hypot(pts[:, 0] - HUB_XY[0], pts[:, 1] - HUB_XY[1])
    q = pts[d > 0.06, :2] - T0
    return bool(len(q) == 0 or (np.all(np.abs(q @ NV) <= CORR_HALF) and np.all(q @ U <= CR_S + IN_HALF_U)))


def _track(objects) -> None:
    """Once per control step: latch overspeed (> 1 turn/s), a loose turn, and a free-turning drum (> 20 deg/s while no
    gripper is pinching the cable or its connector, see grasp())."""
    if not _finite(objects):
        _MON["violated"] = _MON["violated"] or "nan"
        return
    step = _ENV["step"]
    if step == _MON["last_step"] or _ENV["env"] is None:
        return
    _MON["last_step"] = step
    w = drum_rate(objects)
    _MON["max_rate"] = max(_MON["max_rate"], w)
    if os.environ.get("DRUM_DEBUG") and step % 5 == 0:
        c = objects[CABLE.name]
        print(f"[dbg] step {step} w={math.degrees(objects[DRUM.name].ang_vel[2]):.0f} plug={np.round(c.end_plug, 3).tolist()} "
              f"relaxed={None if _SH.get('relaxed') is None else int(_SH['relaxed'].sum())} "
              f"grasp={[(round(x[4] * 1000, 1), grasp(objects, x)) for x in hands()]}", flush=True)
    h = holders(objects)
    if len(h) == 2 and all(h):
        _MON["both_held"] = True
    lo = loose_turn(objects)
    _MON["max_loose"] = max(_MON["max_loose"], lo)
    if lo > LOOSE_MAX and not _MON["violated"]:
        _MON["violated"] = f"loose turn {lo * 1000:.0f} mm at step {step}"
    if w > MAX_RATE and not _MON["violated"]:
        _MON["violated"] = f"overspeed {math.degrees(w):.0f} deg/s at step {step}"
    elif w > FREE_RATE and not any(h) and not _MON["violated"]:
        _MON["violated"] = f"unheld while turning {math.degrees(w):.0f} deg/s at step {step}"


def score(objects) -> float:
    """0 after any overspeed, loose-turn moment or moment the drum turns over 20 deg/s while no gripper pinches the
    cable or connector (grasp()), if a gripper is within 3 cm of the cable at the end, or if the connector is not in
    the cradle. 0.5 connector in the cradle but the cable off the taped run (or the drum
    still turning); 1.0 connector in the cradle, paid-out cable inside the taped run, drum still."""
    _track(objects)
    if _MON["violated"] or not _finite(objects):
        return 0.0
    if not released(objects) or not in_cradle(objects):
        return 0.0
    return 1.0 if in_corridor(objects) and drum_rate(objects) < STILL_RATE else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[CABLE.name].resting(max_speed=0.02)


def distance(objects) -> float:
    _track(objects)
    if _MON["violated"] or not _finite(objects):
        return 3.0
    return float(np.linalg.norm(np.asarray(objects[CABLE.name].end_plug)[:2] - CR_C))


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    _track(objects)
    ok = _finite(objects)
    plug = np.asarray(objects[CABLE.name].end_plug) if ok else np.zeros(3)
    start = run_pt(0.165)
    paid = float(np.clip((plug[:2] - start) @ U / (CR_S - 0.165), 0, 1)) if ok else 0.0
    return {"paid_out": round(paid, 2), "past_midline": int(ok and plug[1] < -0.02),
            "in_cradle": int(ok and in_cradle(objects)), "in_corridor": int(ok and in_corridor(objects)),
            "rate_kept": int(not _MON["violated"].startswith("overspeed")),
            "never_free": int(not _MON["violated"].startswith("unheld")),
            "coil_tight": int(not _MON["violated"].startswith("loose")),
            "released": int(ok and released(objects))}


VARIATION = {"obj:drum": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}, "cable:multicore": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------
# The demo chooses its motions from the top camera image (vision.py), the arms' own TCP poses and the fixed layout
# above only. ctx.states() is read in _log() for the printed diagnostics and never steers a motion.
HAND_S = 0.335  # connector centre at the hand-over, m along the run
YAW = HEAD  # fingers close across the run; wrist cameras look along the run, toward the cradle
PINCH = -0.030
CONN_START = (0.325, 0.145)  # connector centre in the set-up layout (README "setup")
CONN_TOP = TOP + CONN.size[2]  # top face of the connector lying on the table
BAR_TOP = DRUM_MARK.pos[2] + MARK_SIZE[2] / 2


def _log(ctx, tag):
    st = ctx.states()
    c = st[CABLE.name]
    p = np.asarray(c.end_plug)
    u, v = _local(p)
    prox = [f < 0.030 * 2 and _cable_dist(st, g) <= 0.03 for g, f in grippers()]  # the old proximity rule, for comparison
    print(f"[demo] {tag}: plug=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) cr_uv=({u:.3f},{v:.3f}) rate={math.degrees(drum_rate(st)):.0f} "
          f"max={math.degrees(_MON['max_rate']):.0f} loose={_MON['max_loose'] * 1000:.0f}mm "
          f"grasp={[(round(h[4] * 1000, 1), grasp(st, h)) for h in hands()]} prox={prox} viol='{_MON['violated']}' "
          f"cradle={in_cradle(st)} corr={in_corridor(st)} score={score(st)}", flush=True)


def _see_conn(ctx, expect, r=0.06):
    """Connector (x, y) and grip yaw from the top camera: the near-black pixels within r of where it is expected,
    back-projected onto its top face. None if it is not in view."""
    import vision

    img = vision.top_image(ctx)
    mask = img.max(axis=-1) < 60
    pts = vision.top_camera(img.shape).to_plane(mask, CONN_TOP)
    pts = pts[np.linalg.norm(pts - np.asarray(expect)[:2], axis=1) < r]
    if len(pts) < 5:
        return None
    xy = np.median(pts, axis=0)
    pts = pts[np.linalg.norm(pts - xy, axis=1) < 0.03]
    yaw = YAW
    if len(pts) >= 5:
        ev, evec = np.linalg.eigh(np.cov((pts - pts.mean(axis=0)).T))
        if ev[1] > 2.0 * ev[0]:
            ax = evec[:, 1]
            y = (math.atan2(ax[1], ax[0]) + math.pi / 2 - YAW + math.pi / 2) % math.pi - math.pi / 2 + YAW
            if abs(y - YAW) < math.radians(25):
                yaw = y
    if os.environ.get("DRUM_VDEBUG"):
        p = np.asarray(ctx.states()[CABLE.name].end_plug)
        print(f"[vis] conn est=({xy[0]:.3f},{xy[1]:.3f}) yaw={math.degrees(yaw):.0f} n={len(pts)} true=({p[0]:.3f},{p[1]:.3f}) "
              f"err={np.linalg.norm(xy - p[:2]) * 1000:.0f}mm", flush=True)
    return xy, yaw


def _locate(ctx, arm, expect):
    """Look for the connector (a few tries); fall back to the expected place."""
    for _ in range(4):
        got = _see_conn(ctx, expect)
        if got is not None:
            return got
        yield from arm.wait(0.15)
    return np.asarray(expect[:2], dtype=float), YAW


_ROI: dict = {}


def _drum_roi(shape) -> np.ndarray:
    """Top-image pixels that look at the drum's top flange (fixed layout)."""
    key = tuple(shape[:2])
    if key not in _ROI:
        import vision

        cam = vision.top_camera(shape)
        xy = cam.to_plane(np.ones(shape[:2], bool), BAR_TOP)  # row-major order, same as np.nonzero
        d = np.linalg.norm(xy - np.asarray(HUB_XY), axis=1).reshape(shape[:2])
        _ROI[key] = d < FL_R
    return _ROI[key]


def _drum_motion(prev, img) -> int:
    """Pixels on the drum top that changed by more than 30 levels between two top images."""
    roi = _drum_roi(img.shape)
    return int(((np.abs(img - prev).max(axis=-1) > 30) & roi).sum())


def _wait_still(ctx, arm, limit=4.0, dt=0.4):
    """Hold still until the drum top (white bar on the grey flange) looks unchanged in the top camera over two
    consecutive 0.4 s intervals. The arms are not moving meanwhile, so a change there is the drum turning."""
    import vision

    t, calm, prev = 0.0, 0, vision.top_image(ctx)
    while t < limit:
        yield from arm.wait(dt)
        t += dt
        img = vision.top_image(ctx)
        n = _drum_motion(prev, img)
        calm = calm + 1 if n <= 1 else 0
        if os.environ.get("DRUM_VDEBUG"):
            print(f"[vis] drum changed px={n} calm={calm} true_rate={math.degrees(drum_rate(ctx.states())):.1f}", flush=True)
        prev = img
        if calm >= 2:
            return


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("DRUM_CHEAT", "")
    gz = TOP + CONN.size[2] / 2 + 0.010
    lift = gz + 0.006
    carry = gz + 0.018  # connector bottom ~15 mm up: clears the 12 mm cradle walls
    hi = TOP + 0.08
    _log(ctx, "start")
    if cheat == "nothing":
        yield from L.wait(4.0)
        _log(ctx, "end")
        return

    # 1) left arm takes the connector (fingers across it, wrist camera toward the run)
    p, yaw = yield from _locate(ctx, L, CONN_START)
    yield from L.move((p[0], p[1], hi), yaw=yaw, finger=PINCH, duration=1.6)
    yield from L.move((p[0], p[1], gz), yaw=yaw, duration=1.0)
    yield from L.grip(FINGER_CLOSED, 0.6)
    yield from L.move((p[0], p[1], lift), yaw=yaw, duration=0.4)
    _log(ctx, "left has connector")

    # 2) left pays the cable out along the run to the hand-over point near the middle of the table
    h = run_pt(HAND_S)
    if cheat == "yank":
        yield from L.move((h[0], h[1], lift), yaw=YAW, duration=0.5)
    elif cheat == "overrun":  # stop dead at peak pull speed, still holding: the drum runs on and throws cable off
        pull = L.move((h[0], h[1], lift), yaw=YAW, duration=1.6)
        for _ in range(16):
            next(pull)
            yield
        pull.close()
        g = L.tcp()
        yield from L.move(g, yaw=YAW, duration=0.05)
        for k in range(4):
            yield from L.wait(0.25)
            _log(ctx, f"overrun {k}")
        yield from L.move((h[0], h[1], lift), yaw=YAW, duration=3.0)
    elif cheat in ("letgo", "hover"):
        # letgo: open the fingers at peak pull speed (under one turn per second) and step clear while the drum coasts.
        # hover: same, but the fingers then close fully just above the connector (TCP ~1.5 cm from it) while the drum
        # coasts: a closed gripper next to the cable that is not pinching it. Both then re-take it and finish normally.
        pull = L.move((h[0], h[1], lift), yaw=YAW, duration=2.0)
        for _ in range(20):
            next(pull)
            yield
        pull.close()
        _log(ctx, f"{cheat} opens")
        g = L.tcp()
        yield from L.move(g, yaw=YAW, finger=FINGER_OPEN, duration=0.15)
        if cheat == "hover":
            yield from L.move((g[0], g[1], g[2] + 0.012), yaw=YAW, finger=FINGER_CLOSED, duration=0.25)
        for k in range(6):
            yield from L.wait(0.25)
            _log(ctx, f"{cheat} coast {k}")
        yield from L.move((g[0], g[1], hi), yaw=YAW, finger=PINCH, duration=0.6)
        g = L.tcp()
        yield from L.move((0.25, 0.30, hi), yaw=0.0, duration=0.8)  # out of the camera's view of the connector
        p, yaw = yield from _locate(ctx, L, g)
        yield from L.move((p[0], p[1], hi), yaw=yaw, duration=0.8)
        yield from L.move((p[0], p[1], gz), yaw=yaw, duration=0.8)
        yield from L.grip(FINGER_CLOSED, 0.6)
        yield from L.move((p[0], p[1], lift), yaw=yaw, duration=0.4)
        yield from L.move((h[0], h[1], lift), yaw=YAW, duration=3.0)
    else:
        yield from L.move((h[0], h[1], lift), yaw=YAW, duration=3.5)
    yield from L.move((h[0], h[1], gz), yaw=YAW, duration=0.4)
    yield from _wait_still(ctx, L)
    _log(ctx, "left at hand-over")
    if cheat == "onearm":  # left arm alone: push the connector as far toward the cradle as it reaches, let go
        far = run_pt(HAND_S + 0.10)
        yield from L.move((far[0], far[1], gz), yaw=YAW, duration=2.0)
        yield from _wait_still(ctx, L)
        yield from L.grip(FINGER_OPEN, 0.4)
        yield from L.move((far[0], far[1], hi), yaw=YAW, duration=0.8)
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    yield from L.grip(FINGER_OPEN, 0.4)
    g = L.tcp()
    yield from L.move((g[0], g[1], hi), yaw=YAW, duration=0.6)
    yield from L.move((0.25, 0.20, hi), yaw=0.0, duration=1.2)

    # 3) right arm takes the connector where the left set it down (seen by the top camera) and pays out the rest
    p, yaw = yield from _locate(ctx, Rt, g)
    yield from Rt.move((p[0], p[1], hi), yaw=yaw, finger=PINCH, duration=1.6)
    yield from Rt.move((p[0], p[1], gz), yaw=yaw, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED, 0.6)
    yield from Rt.move((p[0], p[1], carry), yaw=yaw, duration=0.5)
    _log(ctx, "right has connector")
    yield from Rt.move((CR_C[0], CR_C[1], carry), yaw=YAW, duration=3.5)
    yield from Rt.move((CR_C[0], CR_C[1], gz + 0.001), yaw=YAW, duration=0.8)
    yield from _wait_still(ctx, Rt)
    _log(ctx, "right at cradle")
    if cheat == "hold":
        yield from Rt.wait(3.0)
        _log(ctx, "end")
        return
    if cheat == "tier05":  # right arm keeps the connector seated while the left arm drags the middle of the run out over the tape: 0.5
        s = float((0.10 - T0[1]) / U[1])  # the run point at y = 0.10 (layout), where the paid-out cable lies
        q = run_pt(s)
        off = q - 0.09 * NV
        z = TOP + R
        yield from L.move((q[0], q[1], hi), yaw=YAW, finger=-0.014, duration=1.2)
        yield from L.move((q[0], q[1], z + 0.010), yaw=YAW, duration=0.8)
        yield from L.grip(FINGER_CLOSED, 0.6)
        yield from L.move((off[0], off[1], z + 0.014), yaw=YAW, duration=3.0)
        yield from _wait_still(ctx, L)
        yield from L.grip(-0.014, 0.5)
        yield from L.move((off[0], off[1], hi), yaw=YAW, duration=0.8)
        yield from L.move((0.25, 0.20, hi), yaw=0.0, duration=1.0)
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move((CR_C[0], CR_C[1], hi), yaw=YAW, duration=0.8)
    yield from Rt.move((0.25, -0.20, hi), yaw=0.0, duration=1.0)
    yield from Rt.wait(1.5)
    _log(ctx, "end")
