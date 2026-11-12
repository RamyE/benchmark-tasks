"""Tension a cable on its gauge and lock it in a toggle clamp.

A cable-assembly fixture on the station table. A black 6 mm cable, 46 cm long, is hooked at one end to the carriage of
a spring tension gauge on the left (a yellow carriage on a grey rail, pulled back by a spring; a green band on the rail
marks the target pull). From the gauge the cable runs along the fixture line (x 28 cm, along -y) through a hold-down
toggle clamp (red hinged bar with a black rubber pad, standing open, sprung over centre) and ends in an orange plug on
the right. The cable lies slack in curves: a loop bulges away from the robot between the gauge and the clamp, and the
tail curls towards the robot.

Goal: leave the cable locked in the clamp under the gauge's target tension, on the fixture line, with both hands off.
Taking up the slack and drawing the gauge carriage into the green band needs one hand on the plug; the clamp is shut
by the other while the tension is held. A cable clamped slack, or tensioned and then let go before the clamp shuts,
springs back to the gauge's zero.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
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
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Lock the black cable in the red toggle clamp with the tension gauge's yellow carriage held in the "
               "green band, the cable straight along the fixture line, then let go of everything.")

GREY = (0.55, 0.57, 0.60)
DARK_GREY = (0.30, 0.31, 0.33)
BLACK = (0.03, 0.03, 0.03)
RED = (0.80, 0.08, 0.06)
YELLOW = (0.95, 0.80, 0.05)
GREEN = (0.10, 0.65, 0.20)
ORANGE = (0.95, 0.40, 0.05)
WHITE = (0.92, 0.92, 0.90)

LX = 0.28  # fixture line: x = LX, cable runs along -y
R = 0.003  # 6 mm cable
CORD_LEN = 0.46

# -- spring tension gauge: carriage on a rail, spring pulls it towards +y -------------------------------------------
CAR = (0.030, 0.024, 0.016)  # carriage size (x, y, z)
CAR_Y0 = 0.245  # carriage centre at the gauge's zero (rest stop)
CAR_Z = TOP + 0.005 + CAR[2] / 2  # floats 5 mm above the table between the rails (it rides on the prismatic joint)
GAUGE_K = 25.0  # N/m
GAUGE_PRELOAD = 0.04  # m: 1.0 N at zero, 1.6 N at 2.5 cm, 2.1 N at 4.5 cm
GAUGE_TRAVEL = 0.06  # end stop
BAND = (0.025, 0.045)  # target draw (green band), m
LOW_DRAW = 0.010  # below this the cable is not held under tension at all
CARRIAGE = Box("gauge_carriage", pos=(LX, CAR_Y0, CAR_Z), size=CAR, mass=0.04, color=YELLOW)
RAIL_Y = (0.17, 0.30)
RAIL_W, RAIL_H, RAIL_OFF = 0.006, 0.006, 0.024
GAUGE = [Box(f"gauge_rail_{s}", pos=(LX + sg * RAIL_OFF, sum(RAIL_Y) / 2, TOP + RAIL_H / 2),
             size=(RAIL_W, RAIL_Y[1] - RAIL_Y[0], RAIL_H), color=GREY) for s, sg in (("near", -1), ("far", 1))]
GAUGE.append(Box("gauge_back", pos=(LX, RAIL_Y[1] + 0.005, TOP + 0.012), size=(0.054, 0.010, 0.024), color=GREY))
# scale on the far rail: the carriage's front (-y) face over the green band = draw within BAND
_FACE0 = CAR_Y0 - CAR[1] / 2
GAUGE.append(Box("gauge_band", pos=(LX + RAIL_OFF + 0.006, _FACE0 - sum(BAND) / 2, TOP + 0.0005),
                 size=(0.012, BAND[1] - BAND[0], 0.001), color=GREEN))
GAUGE.append(Box("gauge_zero", pos=(LX + RAIL_OFF + 0.006, _FACE0, TOP + 0.0005), size=(0.012, 0.003, 0.001),
                 color=BLACK))

# -- toggle clamp: red bar hinged about y at O, black pad 5 cm along it, over-centre spring -------------------------
CY = 0.06  # clamp across the line at y = CY
HX, HZ = LX - 0.05, TOP + 0.018  # hinge
LEV_L, LEV_T, LEV_W = 0.10, 0.008, 0.012
PAD = (0.022, 0.030, 0.008)  # along the bar, across (y), thick
PAD_AT = 0.05  # pad centre this far along the bar from the hinge (over the line when shut)
# spawned standing open: bar along +z from the hinge, pad on its +x face
LEVER = Box("clamp_lever", pos=(HX, CY, HZ + LEV_L / 2), size=(LEV_T, LEV_W, LEV_L), mass=0.04, color=RED)
PAD_BODY = Box("clamp_pad", pos=(HX + LEV_T / 2 + PAD[2] / 2, CY, HZ + PAD_AT), size=(PAD[2], PAD[1], PAD[0]),
               mass=0.01, color=BLACK)
BASE = Box("clamp_base", pos=(HX - 0.002, CY, TOP + 0.006), size=(0.030, 0.034, 0.012), color=DARK_GREY)
SIDE_Y = 0.024  # toggle side plate (spring anchor) beside the bar, clear of the pad's sweep
TOG_A, TOG_H, TOG_TILT = 0.03, 0.06, math.radians(6.0)  # spring pin on the bar, anchor height, dead centre offset
TOG_REST = 0.08  # spring free length (always compressed), stiffness below
TOG_K, TOG_C = 700.0, 6.0
TOG_Q = np.array([HX, CY + SIDE_Y, HZ + TOG_A])  # strut pin on the bar (spawn pose, standing open)
TOG_P = np.array([HX + TOG_H * math.sin(TOG_TILT), CY + SIDE_Y, HZ + TOG_H * math.cos(TOG_TILT)])  # top pin
STRUT_A = Box("strut_top", pos=tuple(float(v) for v in TOG_P), size=(0.008, 0.006, 0.008), mass=0.01, color=DARK_GREY)
STRUT_B = Box("strut_bar", pos=tuple(float(v) for v in TOG_Q), size=(0.008, 0.006, 0.008), mass=0.01, color=DARK_GREY)
SIDE = Box("clamp_side_plate", pos=(HX, CY + SIDE_Y + 0.010, TOP + 0.04), size=(0.010, 0.006, 0.08), color=DARK_GREY)
HINGE_DAMP = 0.002  # N m s / deg on the hinge
SHUT_DEG = 3.0

FIXTURE = [*GAUGE, BASE, SIDE]

# -- the cable ----------------------------------------------------------------------------------------------------------
START = (LX, _FACE0 - 0.006, TOP + R + 0.0005)
PLUG = Plug(size=(0.030, 0.014, 0.012), mass=0.012, color=ORANGE)
CORD = cable("cord", start=START, yaw=-math.pi / 2, length=CORD_LEN, radius=R, anchor_start=True, end_plug=PLUG,
             color=BLACK, bend_stiffness=0.003, bend_damping=0.0005)


def lay_path() -> np.ndarray:
    """Start layout (x, y) centreline: 2 cm straight out of the gauge, a 4.5 cm loop away from the robot, straight
    through the open clamp, then the tail curling towards the robot."""
    y0 = START[1]
    pts = [np.array([LX, y0 - 0.02 * t]) for t in np.linspace(0, 1, 5)]
    ya, yb, amp = y0 - 0.02, CY + 0.035, 0.036
    for t in np.linspace(0, 1, 60)[1:]:
        y = ya + (yb - ya) * t
        pts.append(np.array([LX + amp * math.sin(math.pi * t) ** 2, y]))
    for y in np.linspace(yb, CY - 0.06, 12)[1:]:
        pts.append(np.array([LX, y]))
    c = np.array([LX - 0.10, CY - 0.06])  # tail: arc of radius 10 cm towards the robot, then straight
    for a in np.linspace(0, math.radians(50), 20)[1:]:
        pts.append(c + 0.10 * np.array([math.cos(a), -math.sin(a)]))
    d = np.array([-math.sin(math.radians(50)), -math.cos(math.radians(50))])
    for s in np.linspace(0, 0.25, 20)[1:]:
        pts.append(pts[-1] + d * (0.25 / 19))
    return np.array(pts)


LAY = lay_path()


def _shape(emb) -> None:
    cableshape.preshape(emb, CORD, LAY, hold="ramp", settle_s=0)
    cableshape.settle(emb, 2.0)


cableshape.on_reset(_shape)


def _install_spawn_hook() -> None:
    """Gauge prismatic + spring, cable start pinned to the carriage, clamp hinge + damper, pad weld, over-centre
    spring (a distance joint whose minimum length is a compression spring)."""
    orig = cablekit._spawn_cable
    if getattr(orig, "_toggle_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORD.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        W = np.array([w[0], w[1], w[2]], dtype=float)
        car, lev, pad = (f"{parent}/{o.name}" for o in (CARRIAGE, LEVER, PAD_BODY))
        q0 = Gf.Quatf(1.0, 0.0, 0.0, 0.0)

        def v(p):
            return Gf.Vec3f(*(float(x) for x in p))

        # gauge: carriage slides along y, spring drive towards +y (zero stop at joint pos 0)
        g = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/gauge_slide")
        g.CreateBody1Rel().SetTargets([Sdf.Path(car)])
        g.CreateAxisAttr("Y")
        g.CreateLocalPos0Attr(v(W + np.array(CARRIAGE.pos)))
        g.CreateLocalRot0Attr(q0)
        g.CreateLocalPos1Attr(v((0, 0, 0)))
        g.CreateLocalRot1Attr(q0)
        g.CreateLowerLimitAttr(-GAUGE_TRAVEL)
        g.CreateUpperLimitAttr(0.0)
        g.CreateExcludeFromArticulationAttr(True)
        d = UsdPhysics.DriveAPI.Apply(g.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(GAUGE_K)
        d.CreateDampingAttr(1.0)
        d.CreateTargetPositionAttr(GAUGE_PRELOAD)
        # cable start: replace the kit's world weld by a vertical pin on the carriage's hook
        stage.RemovePrim(f"{prim_path}/anchor")
        p = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/gauge_hook")
        p.CreateBody0Rel().SetTargets([Sdf.Path(car)])
        p.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        p.CreateLocalPos0Attr(v(np.array(START) - np.array(CARRIAGE.pos)))
        p.CreateLocalRot0Attr(Gf.Quatf(math.cos(-math.pi / 4), 0.0, 0.0, math.sin(-math.pi / 4)))
        p.CreateLocalPos1Attr(v((-CORD.seg_len / 2, 0, 0)))
        p.CreateLocalRot1Attr(q0)
        p.CreateAxisAttr("Z")
        p.CreateExcludeFromArticulationAttr(True)
        # clamp hinge about y at O; joint 0 = standing open (stop), +90 deg = shut
        h = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/clamp_hinge")
        h.CreateBody1Rel().SetTargets([Sdf.Path(lev)])
        h.CreateAxisAttr("Y")
        h.CreateLocalPos0Attr(v(W + np.array([HX, CY, HZ])))
        h.CreateLocalRot0Attr(q0)
        h.CreateLocalPos1Attr(v((0, 0, -LEV_L / 2)))
        h.CreateLocalRot1Attr(q0)
        h.CreateLowerLimitAttr(0.0)
        h.CreateUpperLimitAttr(97.5)
        h.CreateExcludeFromArticulationAttr(True)
        hd = UsdPhysics.DriveAPI.Apply(h.GetPrim(), "angular")
        hd.CreateTypeAttr("force")
        hd.CreateStiffnessAttr(0.0)
        hd.CreateDampingAttr(HINGE_DAMP)
        hd.CreateTargetVelocityAttr(0.0)
        f = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/pad_weld")
        f.CreateBody0Rel().SetTargets([Sdf.Path(lev)])
        f.CreateBody1Rel().SetTargets([Sdf.Path(pad)])
        f.CreateLocalPos0Attr(v(np.array(PAD_BODY.pos) - np.array(LEVER.pos)))
        f.CreateLocalRot0Attr(q0)
        f.CreateLocalPos1Attr(v((0, 0, 0)))
        f.CreateLocalRot1Attr(q0)
        f.CreateExcludeFromArticulationAttr(True)
        # over-centre spring strut: stub A pinned to the side plate at P, stub B pinned to the bar at Q, a
        # compression-spring slide between them (dead centre 6 deg past the open stop)
        a_, b_ = (f"{parent}/{o.name}" for o in (STRUT_A, STRUT_B))
        for jn, b0, p0, b1 in (("strut_top_pin", None, W + TOG_P, a_),
                               ("strut_bar_pin", lev, TOG_Q - np.array(LEVER.pos), b_)):
            j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/{jn}")
            if b0 is not None:
                j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
            j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
            j.CreateAxisAttr("Y")
            j.CreateLocalPos0Attr(v(p0))
            j.CreateLocalRot0Attr(q0)
            j.CreateLocalPos1Attr(v((0, 0, 0)))
            j.CreateLocalRot1Attr(q0)
            j.CreateExcludeFromArticulationAttr(True)
        u = (TOG_P - TOG_Q) / np.linalg.norm(TOG_P - TOG_Q)
        beta = math.atan2(u[0], u[2])  # rotation about y taking +z onto the strut axis
        qs = Gf.Quatf(math.cos(beta / 2), 0.0, math.sin(beta / 2), 0.0)
        s = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/toggle_spring")
        s.CreateBody0Rel().SetTargets([Sdf.Path(a_)])
        s.CreateBody1Rel().SetTargets([Sdf.Path(b_)])
        s.CreateAxisAttr("Z")
        s.CreateLocalPos0Attr(v((0, 0, 0)))
        s.CreateLocalRot0Attr(qs)
        s.CreateLocalPos1Attr(v((0, 0, 0)))
        s.CreateLocalRot1Attr(qs)
        s.CreateLowerLimitAttr(-0.2)
        s.CreateUpperLimitAttr(-0.012)
        s.CreateExcludeFromArticulationAttr(True)
        sd = UsdPhysics.DriveAPI.Apply(s.GetPrim(), "linear")
        sd.CreateTypeAttr("force")
        sd.CreateStiffnessAttr(TOG_K)
        sd.CreateDampingAttr(TOG_C)
        sd.CreateTargetPositionAttr(-TOG_REST)
        if os.environ.get("TOGGLE_DEBUG"):
            print(f"[toggle] joints authored under {prim_path}; env origin {W}", flush=True)
        return prim

    spawn._toggle_hook = True
    cablekit._spawn_cable = spawn


_install_spawn_hook()

OBJECTS = [*FIXTURE, CARRIAGE, LEVER, PAD_BODY, STRUT_A, STRUT_B, CORD]

TWIN_EXTRA = [
    {"name": "gauge_slide", "kind": "joint", "real": "linear rail + 25 N/m extension spring, 1 N preload",
     "note": "carriage slides along y, 6 cm travel"},
    {"name": "gauge_hook", "kind": "joint", "real": "swivel hook", "note": "cable end turns about the vertical"},
    {"name": "clamp_hinge", "kind": "joint", "real": "M4 shoulder bolt", "note": "bar turns 0-97 deg about y"},
    {"name": "toggle_spring", "kind": "joint", "real": "compression spring strut, side plate to bar",
     "note": "over-centre: holds the bar open past dead centre and shut below it"},
]

# -- grippers ------------------------------------------------------------------------------------------------------------
_ENV: dict = {"env": None}
_EP: dict = {"dragged": False, "s_ref": None, "t": 0, "prev": None}


def _reset_episode() -> None:
    _EP.update(dragged=False, s_ref=None, t=0, prev=None)


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_toggle_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._toggle_hooked = True
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


def grippers() -> list[np.ndarray] | None:
    """Both TCPs (env frame); None if the env is not known (a missing reading fails the release check)."""
    env = _ENV["env"]
    if env is None:
        return None
    try:
        from yam_arena.runtime import tcp_world
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
        out = [np.asarray(tcp_world(base.scene[k]), dtype=np.float64).reshape(-1)[:3] - origin
               for k in ("left_robot", "right_robot")]
    except Exception:  # noqa: BLE001
        return None
    return out if all(np.isfinite(g).all() for g in out) else None


# -- geometry -----------------------------------------------------------------------------------------------------------
RELEASE_DIST = 0.03
LINE_TOL = 0.012  # every cable point from the gauge to 8 cm past the clamp within 12 mm of the line
LINE_PAST = 0.08
STRAIGHT_TOL = 0.006  # gauge-to-clamp span: max sideways deviation for full credit (no sag or kink)
DRAG_TOL = 0.010  # cable pulled more than 1 cm through the shut clamp = dragged


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    poly = np.asarray(poly, dtype=float)
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = min(1000, max(1, int(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def lever_deg(objects) -> float:
    """Clamp bar angle above horizontal (deg): 0 = shut on the cable, 96 = standing open; nan if unknown."""
    q = objects[LEVER.name].quat_xyzw
    if not np.isfinite(q).all():
        return float("nan")
    u = _rot(q) @ np.array([0.0, 0.0, 1.0])
    return float(math.degrees(math.atan2(u[2], u[0])))


def draw(objects) -> float:
    """How far the gauge carriage is drawn out of its zero (m); nan if unknown."""
    p = objects[CARRIAGE.name].pos
    return float(CAR_Y0 - p[1]) if np.isfinite(p).all() else float("nan")


def _finite(objects) -> bool:
    c = objects[CORD.name]
    return all(np.isfinite(a).all() for a in (c.points, objects[LEVER.name].pos, objects[PAD_BODY.name].pos,
                                              objects[CARRIAGE.name].pos))


def _arclen(pts: np.ndarray) -> np.ndarray:
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]


def under_pad(objects) -> tuple[bool, float | None]:
    """(cable under the pad and the pad down on it, arclength of the cable at the pad centre)."""
    pts = _dense(objects[CORD.name].polyline(), 0.002)
    ps = objects[PAD_BODY.name]
    Rp = _rot(ps.quat_xyzw)
    loc = (pts - ps.pos) @ Rp  # pad frame: x thick (down when shut), y across, z along the bar
    half = np.array([PAD[2], PAD[1], PAD[0]]) / 2
    inside = (np.abs(loc[:, 1]) <= half[1] - 0.002) & (np.abs(loc[:, 2]) <= half[2] - 0.002)
    if not inside.any():
        return False, None
    gap = float(np.abs(np.abs(loc[inside, 0]) - half[0] - R).min())  # pad face to the cable surface
    s = _arclen(pts)
    k = int(np.argmin(np.linalg.norm(pts[:, :2] - ps.pos[:2], axis=1)))
    return gap <= 0.002, float(s[k])


def line_dev(objects) -> tuple[float, float]:
    """(max sideways distance from the line, gauge to LINE_PAST past the clamp; same, gauge to clamp only)."""
    pts = _dense(objects[CORD.name].polyline())
    sel = pts[pts[:, 1] >= CY - LINE_PAST]
    if pts[:, 1].min() > CY - LINE_PAST:
        return float("inf"), float("inf")
    span = pts[(pts[:, 1] >= CY) & (pts[:, 1] <= START[1] - 0.01)]
    return float(np.abs(sel[:, 0] - LX).max()), float(np.abs(span[:, 0] - LX).max()) if len(span) else float("inf")


def _track(objects) -> None:
    """Latch: cable dragged through the clamp while shut (by more than DRAG_TOL along the cable)."""
    if not _finite(objects):
        return
    shut = lever_deg(objects) <= SHUT_DEG
    on, s = under_pad(objects)
    if shut and on and s is not None:
        if _EP["s_ref"] is None:
            _EP["s_ref"] = s
        elif abs(s - _EP["s_ref"]) > DRAG_TOL:
            _EP["dragged"] = True
    elif lever_deg(objects) > 20.0:
        _EP["s_ref"] = None  # clamp opened again: a new closing starts a new reference


def released(objects) -> bool:
    g = grippers()
    if g is None:
        return False
    c = objects[CORD.name]
    pts = np.vstack([_dense(c.polyline())] + ([c.end_plug[None]] if c.end_plug is not None else []))
    boxes = [objects[n] for n in (LEVER.name, PAD_BODY.name, CARRIAGE.name)]
    for t in g:
        if float(np.linalg.norm(pts - t, axis=1).min()) < RELEASE_DIST:
            return False
        for b in boxes:
            if float(np.linalg.norm(np.maximum(np.maximum(b.low - t, t - b.high), 0.0))) < RELEASE_DIST:
                return False
    return True


def score(objects) -> float:
    """0 if: any state unreadable; cable dragged through the shut clamp; a gripper within 3 cm of the cable, plug,
    clamp bar/pad or gauge carriage; clamp bar not shut (more than 3 deg up); cable not under the pad with the pad on
    it; any cable point from the gauge to 8 cm past the clamp more than 12 mm off the line; gauge drawn less than 1 cm
    (no tension held). Else 1.0 if the draw is in the green band (2.5-4.5 cm) and the gauge-to-clamp span is straight
    (within 6 mm of the line); otherwise 0.5 (clamped and held, but under- or over-tensioned or not straight)."""
    if not _finite(objects):
        return 0.0
    _track(objects)
    if _EP["dragged"] or not released(objects):
        return 0.0
    if not lever_deg(objects) <= SHUT_DEG:
        return 0.0
    on, _ = under_pad(objects)
    if not on:
        return 0.0
    dev, span = line_dev(objects)
    if not dev <= LINE_TOL:
        return 0.0
    d = draw(objects)
    if not d >= LOW_DRAW:
        return 0.0
    if BAND[0] <= d <= BAND[1] and span <= STRAIGHT_TOL:
        return 1.0
    return 0.5


def _still(objects) -> bool:
    """Cable points and carriage moved less than 1 mm / 0.5 mm since the previous control step (about 2 cm/s)."""
    pts = np.vstack([objects[CORD.name].points, objects[CARRIAGE.name].pos[None]])
    prev, _EP["prev"] = _EP.get("prev"), pts
    if prev is None or prev.shape != pts.shape:
        return False
    mv = np.linalg.norm(pts - prev, axis=1)
    return bool(mv[:-1].max() < 0.001 and mv[-1] < 0.0005)


def success(objects) -> bool:
    still = _still(objects)
    return score(objects) >= 1.0 and still


def distance(objects) -> float:
    if not _finite(objects):
        return 3.0
    d = draw(objects)
    return float(min(3.0, abs(d - sum(BAND) / 2) + max(0.0, lever_deg(objects)) / 90.0))


def progress(objects) -> dict:
    if not _finite(objects):
        return {"tensioned": 0, "shut": 0, "under_pad": 0, "on_line": 0, "released": 0, "in_band": 0}
    d = draw(objects)
    dev, _ = line_dev(objects)
    return {"tensioned": float(d >= LOW_DRAW), "shut": float(lever_deg(objects) <= SHUT_DEG),
            "under_pad": float(under_pad(objects)[0]), "on_line": float(dev <= LINE_TOL),
            "released": float(released(objects)), "in_band": float(BAND[0] <= d <= BAND[1])}


VARIATION = {"cable:cord": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}
