"""Recover a buried corner iron from its two witness ties and stand the marker stake on it.

A survey mock-up on the station table (cm from the midpoint between the arm bases, x forward, y left). A tan mat
(2 mm, x 22-46, y -12..+14) is the "sand" over a buried corner iron. The iron has no physical object: its position
C = (27, +3) exists only in the score. Two black witness nails (12 mm round, 40 mm tall) stand far out at the table
sides, in front of each arm: nail 1 at (45.6, +41.3), nail 2 at (45.5, -41.3). Each holds the zero end of a white tie
tape (6 mm round marked cord) through a short tension spring (150 N/m, swivels freely); the zero points are at
(45, +40) and (45, -40). Each tape carries one red band (2.2 cm) whose centre lies at the iron's distance from that
zero point when the tape is drawn straight and the spring is out 5 mm:
- tape 1 (left nail, 46.1 cm): band centre 40.7 cm along it (41.2 cm from the zero point when drawn);
- tape 2 (right nail, 51.5 cm): band centre 46.1 cm along it (46.6 cm when drawn).
Each tape lies in a U outboard of its nail (out along +x, bend outboard, back along -x), ending in a 20 x 20 x 10 mm
white tab. A free orange marker stake (26 mm round, 60 mm tall, 50 g) stands on the mat at (36, -2).

The robot must swing each tape out taut from its nail, find where the two red bands meet, stand the stake there and
let go. The two arcs cross twice: at the iron (on the mat) and at (63, +3), off the mat's far edge.

Two arms: each tape lies wholly beyond the other arm's reach (position-only IK misses every point by >= 72 mm), so the
two tapes can only be drawn with both arms.

Score (end state; violations latch for the run): 0 if a tape was never drawn taut (its spring out >= 3 mm and its
red band within 1.5 cm of the iron's distance from the zero point, at the same step), if a spring was ever drawn out
more than 20 mm (over-pull), if the stake ever tilted more than 20 deg, or if at the end a gripper is within 4 cm of a tape, its tab or the stake, or the stake is not standing upright (tilt < 10 deg,
base on the mat or table). Otherwise by the stake centre's distance from C: 1.0 within 1.2 cm, 0.6 within 2.5 cm,
0.3 within 4 cm, else 0.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("The corner iron is buried under the sand. The two tapes are tied to its witness nails and each red mark "
               "is the iron's distance from that nail. Find where the iron is, stand the orange stake on it, then let go.")

WHITE = (0.93, 0.93, 0.90)
RED = (0.85, 0.05, 0.05)
BLACK = (0.06, 0.06, 0.07)
SAND = (0.55, 0.43, 0.27)
ORANGE = (0.98, 0.45, 0.05)

# -- geometry (world m, x forward, y left) -------------------------------------------------------------------------
C = np.array([0.27, 0.03])  # the buried corner iron (score only)
A1 = np.array([0.45, 0.40])  # tape 1 zero point (spring hook at the left nail)
A2 = np.array([0.45, -0.40])  # tape 2 zero point (right nail)
E0 = 0.005  # bands are set for a light taut draw: spring drawn out 5 mm
R = 0.003  # 6 mm tape
Z0 = TOP + R + 0.0005

MAT_X, MAT_Y, MAT_T = (0.22, 0.46), (-0.12, 0.14), 0.002
MAT = Box("sand_mat", pos=(sum(MAT_X) / 2, sum(MAT_Y) / 2, TOP + MAT_T / 2),
          size=(MAT_X[1] - MAT_X[0], MAT_Y[1] - MAT_Y[0], MAT_T), color=SAND)
MAT_TOP = TOP + MAT_T


def _tape_dims(dist: float, band_i: int, n: int):
    """Segment length so that segment band_i's centre lies dist - E0 from the zero tip; total length n segments."""
    seg = (dist - E0) / (band_i + 0.5)
    return seg, n * seg


TAPES = {}
for _name, _A, _bi, _side in (("tape_1", A1, 18, 1.0), ("tape_2", A2, 21, -1.0)):
    _d = float(np.linalg.norm(C - _A))
    _n = _bi + 3
    _seg, _len = _tape_dims(_d, _bi, _n)
    TAPES[_name] = dict(A=_A, band=_bi, n=_n, seg=_seg, length=_len, dist=_d, side=_side, u=(C - _A) / _d)

TAB = Plug(size=(0.020, 0.020, 0.010), mass=0.006, color=WHITE)
SPRING_K = float(os.environ.get("TIE_SPRING_K", 150.0))  # N/m, tension spring between the nail and the tape's zero
SPRING_D = 1.0
U_R = 0.035  # radius of the U bend in the laid-up tape


def _u_path(t):
    """Start layout: from the zero point straight away from the robot (+x), a U bend outboard, then back along -x."""
    A, s, L = t["A"], t["side"], t["length"]
    leg1 = 0.15 if t["n"] < 22 else 0.18
    p0 = np.array(A)
    p1 = p0 + np.array([leg1, 0.0])
    cc = p1 + np.array([0.0, s * U_R])
    th = np.linspace(0.0, math.pi, 40)
    arc = np.c_[cc[0] + U_R * np.sin(th), cc[1] - s * U_R * np.cos(th)]
    leg2 = L - leg1 - math.pi * U_R + 0.05
    p3 = arc[-1] - np.array([leg2, 0.0])
    return np.vstack([np.linspace(p0, p1, 30), arc[1:], np.linspace(arc[-1], p3, 40)[1:]])


for _name, _t in TAPES.items():
    _t["cable"] = cable(_name, start=(float(_t["A"][0]), float(_t["A"][1]), Z0), yaw=0.0, length=_t["length"],
                        radius=R, segment=_t["seg"], end_plug=TAB, anchor_start=True, cable_mass=0.002 * _t["n"],
                        bend_stiffness=0.003, bend_damping=0.0005, color=WHITE)
    _t["path"] = _u_path(_t)

TAPE1, TAPE2 = TAPES["tape_1"]["cable"], TAPES["tape_2"]["cable"]

NAIL_R, NAIL_H, NAIL_OFF = 0.006, 0.040, 0.014
NAILS = []
for _k, (_name, _t) in enumerate(TAPES.items()):
    _p = _t["A"] - NAIL_OFF * _t["u"]  # the nail stands just behind the zero point, away from the pull
    _t["nail"] = _p
    NAILS.append(Cylinder(f"nail_{_k + 1}", pos=(float(_p[0]), float(_p[1]), TOP + NAIL_H / 2), radius=NAIL_R,
                          height=NAIL_H, color=BLACK))

STAKE_R, STAKE_H, STAKE_M = 0.013, 0.060, 0.050
STAKE_XY = np.array([0.36, -0.02])
STAKE = Cylinder("stake", pos=(float(STAKE_XY[0]), float(STAKE_XY[1]), TOP + STAKE_H / 2 + 0.0005), radius=STAKE_R,
                 height=STAKE_H, mass=STAKE_M, color=ORANGE)

OBJECTS = [MAT, *NAILS, TAPE1, TAPE2, STAKE]


def _install_spawn() -> None:
    """Replace each tape's world weld by the nail's tension spring (planar spring at the zero point, free to swivel),
    and colour the band segment red."""
    orig = cablekit._spawn_cable
    if getattr(orig, "_tie_spring", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        name = getattr(cfg, "cable_name", "")
        if name not in TAPES:
            return prim
        t = TAPES[name]
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdPhysics
        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        stage.RemovePrim(f"{prim_path}/anchor")
        j = UsdPhysics.Joint.Define(stage, f"{prim_path}/tie_spring")
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        j.CreateLocalPos0Attr(Gf.Vec3f(float(t["A"][0]), float(t["A"][1]), float(Z0)))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))  # the spawn heading (+x)
        j.CreateLocalPos1Attr(Gf.Vec3f(-t["cable"].seg_len / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateExcludeFromArticulationAttr(True)
        for ax in ("transZ", "rotX", "rotY"):  # locked (low > high); rotZ free: the hook swivels
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), ax)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        for ax in ("transX", "transY"):  # the spring: pulls the zero point back to the nail from any direction
            drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), ax)
            drv.CreateTypeAttr("force")
            drv.CreateStiffnessAttr(SPRING_K)
            drv.CreateDampingAttr(SPRING_D)
            drv.CreateTargetPositionAttr(0.0)
        look = f"{prim_path}/Looks/band"
        mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(RED), roughness=0.5)
        mat.func(look, mat)
        bind_visual_material(f"{prim_path}/seg_{t['band']}/geom", look, stage=stage)
        return prim

    spawn._tie_spring = True
    cablekit._spawn_cable = spawn


_install_spawn()

# -- episode state ---------------------------------------------------------------------------------------------------
_ST: dict = {"env": None}


def _reset_episode() -> None:
    _ST.update(taut={n: False for n in TAPES}, emax={n: 0.0 for n in TAPES}, over=False, tipped=False, nan=False,
               tiltmax=0.0, armed=False)


_reset_episode()


def _shape(emb):
    _ST["env"] = emb._env
    for n, t in TAPES.items():
        cableshape.preshape(emb, t["cable"], t["path"], hold="ramp", settle_s=0)
    cableshape.settle(emb, 2.0)
    _reset_episode()
    _ST["armed"] = True


cableshape.on_reset(_shape)


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_ties_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ST["env"] = self._env
        return base(self, *a, **k)

    _task_step._ties_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    env = _ST["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- scoring geometry ------------------------------------------------------------------------------------------------
TAUT_E = 0.003  # a tape counts as drawn taut once its spring is drawn out 3 mm ...
STRAIGHT_TOL = 0.015  # ... and its red band is within 1.5 cm of the iron's distance from the zero point (tape straight)
OVER_E = 0.020  # over-pull: spring drawn out more than 20 mm
TIP_DEG = 20.0  # stake knocked over (latched)
UPRIGHT_DEG = 10.0
RELEASE = 0.04
TIERS = ((0.012, 1.0), (0.025, 0.6), (0.040, 0.3))


def _finite(objects) -> bool:
    return all(np.isfinite(objects[n].points).all() for n in TAPES) and bool(np.isfinite(objects["stake"].pos).all())


def extension(objects, name) -> float:
    """How far the tape's zero end is drawn out from its nail hook (m, top view)."""
    return float(np.linalg.norm(objects[name].start_tip[:2] - TAPES[name]["A"]))


def band_xy(objects, name) -> np.ndarray:
    return objects[name].points[TAPES[name]["band"], :2]


def tilt_deg(objects) -> float:
    x, y, z, w = objects["stake"].quat_xyzw
    zz = 1.0 - 2.0 * (x * x + y * y)  # world z of the stake's axis
    return math.degrees(math.acos(float(np.clip(zz, -1.0, 1.0))))


def stake_xy(objects) -> np.ndarray:
    return np.asarray(objects["stake"].pos[:2], dtype=float)


def stake_error(objects) -> float:
    return float(np.linalg.norm(stake_xy(objects) - C))


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def released(objects) -> bool:
    """Both TCPs at least 4 cm from every point of both tapes, their tabs and the stake."""
    pts = [_dense(objects[n].polyline()) for n in TAPES]
    pts += [objects[n].end_plug[None] for n in TAPES if objects[n].end_plug is not None]
    pts = np.vstack(pts)
    s = objects["stake"]
    for g in grippers():
        if float(np.linalg.norm(pts - g, axis=1).min()) < RELEASE:
            return False
        if float(np.linalg.norm(g - np.clip(g, s.low, s.high))) < RELEASE:
            return False
    return True


def standing(objects) -> bool:
    """Stake upright (tilt < 10 deg) with its base on the mat or table (not perched on a tape)."""
    s = objects["stake"]
    return tilt_deg(objects) < UPRIGHT_DEG and float(s.low[2]) < MAT_TOP + 0.004


def _track(objects) -> None:
    if not _ST.get("armed"):
        return
    if not _finite(objects):
        _ST["nan"] = True
        return
    for n in TAPES:
        e = extension(objects, n)
        _ST["emax"][n] = max(_ST["emax"][n], e)
        rb = float(np.linalg.norm(band_xy(objects, n) - TAPES[n]["A"]))
        if e >= TAUT_E and rb >= TAPES[n]["dist"] - STRAIGHT_TOL:
            if not _ST["taut"][n] and os.environ.get("DEMO_DEBUG"):
                print(f"[track] taut latched on {n}: e={e * 1000:.1f} mm band r={rb * 100:.2f} cm "
                      f"(needs >= {(TAPES[n]['dist'] - STRAIGHT_TOL) * 100:.2f})", flush=True)
            _ST["taut"][n] = True
        if e > OVER_E:
            if not _ST["over"] and os.environ.get("DEMO_DEBUG"):
                print(f"[track] over-pull latched on {n}: e={e * 1000:.1f} mm tab={np.round(objects[n].end_plug, 3)}",
                      flush=True)
            _ST["over"] = True
    t = tilt_deg(objects)
    _ST["tiltmax"] = max(_ST["tiltmax"], t)
    if t > TIP_DEG:
        _ST["tipped"] = True


def score(objects) -> float:
    """0 unless both tapes were drawn taut at some step (spring out >= 3 mm with the band within 1.5 cm of the iron's
    distance from the zero point), no spring was ever drawn out more than 20 mm and the stake never tilted past 20 deg
    (all latched), and at the end both grippers are 4 cm clear of both tapes, their tabs and the stake, and the stake stands upright (< 10 deg) on the mat. Then 1.0 / 0.6 / 0.3 for the
    stake centre within 1.2 / 2.5 / 4 cm of the buried iron."""
    _track(objects)
    if _ST["nan"] or _ST["over"] or _ST["tipped"] or not all(_ST["taut"].values()):
        return 0.0
    if not _finite(objects) or not standing(objects) or not released(objects):
        return 0.0
    d = stake_error(objects)
    for lim, val in TIERS:
        if d <= lim:
            return val
    return 0.0


def success(objects) -> bool:
    return (score(objects) >= 1.0 and objects["stake"].resting(max_speed=0.01)
            and all(objects[n].resting(max_speed=0.02) for n in TAPES))


def distance(objects) -> float:
    if not _finite(objects):
        return 1.0
    return stake_error(objects)


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    if not _finite(objects):
        return {"nan": 1}
    return {
        "tape_1_taut": int(_ST["taut"]["tape_1"]), "tape_2_taut": int(_ST["taut"]["tape_2"]),
        "stake_moved": int(np.linalg.norm(stake_xy(objects) - STAKE_XY) > 0.03),
        "stake_within_4cm": int(stake_error(objects) <= 0.04), "stake_within_1_2cm": int(stake_error(objects) <= 0.012),
        "standing": int(standing(objects)), "released": int(released(objects)),
        "no_violation": int(not (_ST["over"] or _ST["tipped"])),
    }


VARIATION = {"cable:tape_1": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}, "cable:tape_2": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},
             "obj:stake": {"dx": 0.02, "dy": 0.02, "yaw": 3.14}}

TWIN_EXTRA = [
    {"name": "tie_springs", "kind": "joint", "real": "small extension spring (about 150 N/m, 20 mm long) from each nail to "
     "the tape's zero ring", "note": "sim: planar spring at the zero point, swivels freely, no vertical travel"},
    {"name": "red_bands", "kind": "decal", "size_mm": [22, 6], "colour": "red",
     "note": "red tape wrapped round the tie tape: tape 1 at 407 mm, tape 2 at 461 mm from the zero (band centres)"},
]


# -- closed-loop scripted demo ---------------------------------------------------------------------------------------
Z_DRAW = TOP + 0.019  # tab held low (bottom ~4 mm above the mat) so the taut tape pulls it level, not out of the fingers
SWING_R = 0.10
LAY_R = 0.36  # the tab is laid down this far from the zero point after the reading
TURN_2 = math.radians(15.0)  # tape 2 is read along a line 15 deg towards the robot from the iron (clear of it)
Z_HI = TOP + 0.08


def circle_cross(a, ra, b, rb, far=True):
    """Crossing of circles (a, ra) and (b, rb) on the far side (+x) of the line a-b (or the near one)."""
    d = float(np.linalg.norm(b - a))
    x = (ra * ra - rb * rb + d * d) / (2 * d)
    h = math.sqrt(max(ra * ra - x * x, 0.0))
    u = (b - a) / d
    n = np.array([-u[1], u[0]])
    p1, p2 = a + x * u + h * n, a + x * u - h * n
    pf, pn = (p1, p2) if p1[0] > p2[0] else (p2, p1)
    return pf if far else pn


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def demo_script(ctx):
    """Left arm draws tape 1 taut from its nail towards the mat (spring out ~5 mm), reads the band's distance, and lays
    the tape back off the mat; the right arm draws tape 2 along a line 15 deg towards the robot from the iron (the band
    is on its arc there too), reads it and lets go of it there, clear of the iron. The crossing of the two measured
    arcs that lies on the mat is computed and the right arm stands the stake there with a closed-loop correction, lets
    go, and backs off.

    TIE_CHEAT (test only): none | hold (keeps holding the stake) | onearm (right arm only: tape 2 + stake at C) |
    mirror (towards the other crossing, as far as a downward grip reaches) | centre (mat centre) | mid (towards the
    midpoint of the nails) | swap (crossing with the two band distances swapped) | guess (stake at C without drawing
    the tapes) | over (tape 1 read, then hauled until its spring is out 30 mm) | tug (each tape tugged near its zero end
    until its spring is out ~5 mm, not straightened, then the stake on the iron's spot) | knock (stake shoved over
    while letting go) | t06 / t03 (stake set 1.9 / 3.2 cm from C: partial tiers)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("TIE_CHEAT", "")
    meas = {}

    def log(tag):
        st = ctx.states()
        if not _finite(st):
            print(f"[demo t={ctx.t:5.1f}] {tag}: NaN", flush=True)
            return
        ex = " ".join(f"{n}:e={extension(st, n) * 1000:.1f}mm band={np.round(band_xy(st, n), 3)} "
                      f"r={np.linalg.norm(band_xy(st, n) - TAPES[n]['A']) * 100:.2f}cm" for n in TAPES)
        print(f"[demo t={ctx.t:5.1f}] {tag}: {ex} stake={np.round(st['stake'].pos, 3)} tilt={tilt_deg(st):.1f} "
              f"err={stake_error(st) * 100:.2f}cm rel={released(st)} taut={_ST['taut']} over={_ST['over']} "
              f"tip={_ST['tipped']} score={score(st):.2f} prog={progress(st)}", flush=True)

    def draw(arm, name, extra=0.0, turn=0.0, lay=True):
        """Take the tab, swing the tape out over the mat along the nail->iron line turned by `turn` (rad, any line works:
        the band sweeps an arc) and draw it until the spring is out about 5 mm; measure the band's distance from the
        zero point. Then (extra > 0) haul on until the spring is out `extra`; then lay the tape off the mat towards the
        robot (lay=True) or let go of it where it lies (lay=False)."""
        t = TAPES[name]
        A, u = t["A"], t["u"]
        st = ctx.states()
        c = st[name]
        p = c.end_plug
        gy = _grip_yaw(math.atan2(c.end_dir[1], c.end_dir[0]))
        yield from arm.move((p[0], p[1], Z_HI), yaw=gy, finger=FINGER_OPEN, duration=1.5)
        p = ctx.states()[name].end_plug
        yield from arm.move((p[0], p[1], p[2] + 0.008), yaw=gy, duration=0.8)
        yield from arm.grip(FINGER_CLOSED)
        yield from arm.move((p[0], p[1], Z_DRAW + 0.01), yaw=gy, duration=0.5)
        # swing round the nail at a short radius (away from the nail side) onto the nail->iron line, then draw out
        a0 = math.atan2(p[1] - A[1], p[0] - A[0])
        a1 = math.atan2(u[1], u[0]) + turn
        # the tape winds outboard round its zero point; swing the tab the way that turns the zero end away from the
        # nail (tape 1 clockwise, tape 2 counter-clockwise seen from above)
        sgn = -t["side"]
        da = sgn * ((sgn * (a1 - a0)) % (2 * math.pi))
        r0 = float(np.linalg.norm(p[:2] - A))
        hy = _grip_yaw(a1)

        def at(a, r):
            return (A[0] + r * math.cos(a), A[1] + r * math.sin(a), Z_DRAW)

        for k in range(1, 13):
            f = k / 12
            yield from arm.move(at(a0 + da * f, r0 + (SWING_R - r0) * f), yaw=gy, duration=0.4)
            if os.environ.get("DEMO_DEBUG"):
                print(f"[dbg] swing {k}: e={extension(ctx.states(), name) * 1000:.1f} mm", flush=True)
        def dbg(tag):
            if os.environ.get("DEMO_DEBUG"):
                c2 = ctx.states()[name]
                q2 = arm.tcp()
                print(f"[dbg] {tag}: r={r * 100:.1f} e={extension(ctx.states(), name) * 1000:.1f}mm "
                      f"tab-tcp={np.linalg.norm(c2.end_plug[:2] - q2[:2]) * 1000:.0f}mm tabz={c2.end_plug[2]:.3f} tcpz={q2[2]:.3f}", flush=True)

        r = SWING_R
        dbg("swung")
        # draw out steadily (a fast pull drags the tab out of the fingers) in 1 cm steps to nearly the tape's length
        # (drag on the mat can stretch the spring a little before the tape is straight, so that alone is no sign)
        while r < t["length"] - 0.035 and extension(ctx.states(), name) < 0.004:
            r += 0.01
            yield from arm.move(at(a1, r), yaw=hy, duration=0.25)
            dbg("draw")
        for _ in range(60):  # then 1.5 mm steps, each let settle, until the spring stays out ~5 mm
            yield from arm.wait(0.2)
            if extension(ctx.states(), name) >= E0 - 0.0005:
                break
            r += 0.0015
            yield from arm.move(at(a1, r), yaw=hy, duration=0.15)
        dbg("taut")
        for _ in range(15):  # settle to a light, steady pull for the reading (spring out 3.5-7 mm)
            yield from arm.wait(0.2)
            e = extension(ctx.states(), name)
            if e > E0 + 0.002:
                r -= 0.001
            elif e < E0 - 0.0015:
                r += 0.001
            else:
                break
            yield from arm.move(at(a1, r), yaw=hy, duration=0.2)
        yield from arm.wait(0.5)
        dbg("read")
        st = ctx.states()
        meas[name] = float(np.linalg.norm(band_xy(st, name) - A))
        log(f"{name} drawn")
        if extra > 0:  # over-pull test: haul on along a line turned 20 deg towards the robot (within reach)
            ah = a1 + sgn * math.radians(20.0)
            yield from arm.move(at(ah, r), yaw=hy, duration=0.6)
            for _ in range(30):
                if extension(ctx.states(), name) >= extra:
                    break
                r += 0.005
                yield from arm.move(at(ah, r), yaw=hy, duration=0.15)
            log(f"{name} hauled")
            a1 = ah
        if not lay:  # ease off and let go of the tab where it lies
            yield from arm.move(at(a1, r - 0.01), yaw=hy, duration=0.4)
            yield from arm.grip(FINGER_OPEN, 0.4)
            q = arm.tcp()
            yield from arm.move((q[0], q[1], Z_HI), yaw=hy, duration=0.6)
            yield from arm.move((0.18, math.copysign(0.24, arm.base[1]), Z_HI), yaw=0.0, duration=1.0)
            return
        # swing the drawn tape on round (same way, still nearly taut) off the mat to point back towards the robot past
        # the nail's side, ease it in a little and lay it down there
        a_end = -t["side"] * math.radians(160.0)
        db = sgn * ((sgn * (a_end - a1)) % (2 * math.pi))
        for k in range(1, 9):
            f = k / 8
            yield from arm.move(at(a1 + db * f, (r - 0.015) + (LAY_R - (r - 0.015)) * f), yaw=hy, duration=0.4)
            dbg(f"lay {k}")
        yield from arm.grip(FINGER_OPEN, 0.4)
        q = arm.tcp()
        yield from arm.move((q[0], q[1], Z_HI), yaw=hy, duration=0.6)
        for _ in range(2):  # check: if the tape still lies near the iron (tab slipped), take the tab again and re-lay
            st = ctx.states()
            if float(np.linalg.norm(st[name].points[:, :2] - C, axis=1).min()) > 0.07:
                break
            log(f"{name} still on the mat: re-laying")
            c = st[name]
            p = c.end_plug
            gy2 = _grip_yaw(math.atan2(c.end_dir[1], c.end_dir[0]))
            yield from arm.move((p[0], p[1], Z_HI), yaw=gy2, finger=FINGER_OPEN, duration=1.0)
            yield from arm.move((p[0], p[1], p[2] + 0.008), yaw=gy2, duration=0.8)
            yield from arm.grip(FINGER_CLOSED)
            yield from arm.move((p[0], p[1], Z_DRAW), yaw=gy2, duration=0.5)
            a_p = math.atan2(p[1] - A[1], p[0] - A[0])
            r_p = float(np.linalg.norm(p[:2] - A))
            dc = sgn * ((sgn * (a_end - a_p)) % (2 * math.pi))
            for k in range(1, 7):
                f = k / 6
                yield from arm.move(at(a_p + dc * f, r_p + (LAY_R - r_p) * f), yaw=gy2, duration=0.4)
            yield from arm.grip(FINGER_OPEN, 0.4)
            q = arm.tcp()
            yield from arm.move((q[0], q[1], Z_HI), yaw=gy2, duration=0.6)
        yield from arm.move((0.18, math.copysign(0.24, arm.base[1]), Z_HI), yaw=0.0, duration=1.0)

    def place(arm, target_xy, hold=False, knock=False):
        st = ctx.states()
        m = st["stake"].pos
        yaw = math.atan2(target_xy[1] - m[1], target_xy[0] - m[0])
        yaw = _grip_yaw(yaw)
        pad = STAKE_H / 2 - 0.012  # TCP 12 mm below the stake top
        yield from arm.move((m[0], m[1], Z_HI + 0.03), yaw=yaw, finger=FINGER_OPEN, duration=1.3)
        m = ctx.states()["stake"].pos
        yield from arm.move((m[0], m[1], m[2] + pad), yaw=yaw, duration=0.9)
        yield from arm.grip(FINGER_CLOSED)
        off = ctx.states()["stake"].pos[:2] - np.array(m[:2])
        yield from arm.move((m[0], m[1], Z_HI + 0.03), yaw=yaw, duration=0.7)
        tx, ty = target_xy[0] - off[0], target_xy[1] - off[1]
        zp = MAT_TOP + STAKE_H / 2 + pad + 0.002
        yield from arm.move((tx, ty, Z_HI + 0.03), yaw=yaw, duration=1.4)
        yield from arm.move((tx, ty, zp + 0.02), yaw=yaw, duration=0.8)
        st = ctx.states()
        err = np.asarray(target_xy) - stake_xy(st)
        tx, ty = tx + err[0], ty + err[1]
        yield from arm.move((tx, ty, zp), yaw=yaw, duration=0.8)
        if hold:
            return
        yield from arm.grip(FINGER_OPEN, 0.5)
        if knock:  # shove the stake over sideways while letting go (the gripper is still beside it, not yet released)
            yield from arm.move((tx + 0.06 * math.cos(yaw + math.pi / 2), ty + 0.06 * math.sin(yaw + math.pi / 2), zp),
                                yaw=yaw, duration=0.4)
        yield from arm.move((tx, ty, Z_HI + 0.04), yaw=yaw, duration=0.8)
        yield from arm.move((0.18, math.copysign(0.24, arm.base[1]), Z_HI), yaw=0.0, duration=1.0)

    log("start")
    if cheat == "none":
        yield from L.wait(4.0)
        log("end")
        return
    if cheat == "guess":  # stake straight onto the iron's spot, tapes never drawn
        yield from place(Rt, C)
        yield from Rt.wait(1.5)
        log("end")
        return
    if cheat == "tug":  # each tape tugged near its zero end until the spring is out ~5 mm, never straightened
        for arm, name in ((L, "tape_1"), (Rt, "tape_2")):
            t = TAPES[name]
            q = ctx.states()[name].points
            k = 2  # a point about 4 cm out from the zero end, on the first leg of the U
            d = q[k + 1, :2] - q[k - 1, :2]
            gy = _grip_yaw(math.atan2(d[1], d[0]))
            out = (q[k, :2] - t["A"]) / np.linalg.norm(q[k, :2] - t["A"])
            yield from arm.move((q[k, 0], q[k, 1], Z_HI), yaw=gy, finger=FINGER_OPEN, duration=1.5)
            yield from arm.move((q[k, 0], q[k, 1], q[k, 2] + 0.010), yaw=gy, duration=0.8)
            yield from arm.grip(FINGER_CLOSED)
            pull = 0.0
            while pull < 0.015 and extension(ctx.states(), name) < E0:
                pull += 0.001
                yield from arm.move((q[k, 0] + out[0] * pull, q[k, 1] + out[1] * pull, q[k, 2] + 0.010), yaw=gy,
                                    duration=0.15)
            yield from arm.wait(0.5)
            log(f"{name} tugged")
            yield from arm.grip(FINGER_OPEN, 0.4)
            p = arm.tcp()
            yield from arm.move((p[0], p[1], Z_HI), yaw=gy, duration=0.6)
            yield from arm.move((0.18, math.copysign(0.24, arm.base[1]), Z_HI), yaw=0.0, duration=1.0)
        yield from place(Rt, C)
        yield from Rt.wait(1.5)
        log("end")
        return
    if cheat == "onearm":  # right arm alone: its own tape, then the stake on the iron's spot
        yield from draw(Rt, "tape_2", turn=TURN_2, lay=False)
        yield from place(Rt, C)
        yield from Rt.wait(1.5)
        log("end")
        return
    yield from draw(L, "tape_1", extra=0.03 if cheat == "over" else 0.0)  # over: spring hauled out 30 mm
    yield from draw(Rt, "tape_2", turn=TURN_2, lay=False)
    r1, r2 = meas["tape_1"], meas["tape_2"]
    tgt = circle_cross(A1, r1, A2, r2, far=False)
    print(f"[demo] measured r1={r1 * 100:.2f} r2={r2 * 100:.2f} cm -> crossing on the mat {np.round(tgt, 4)} "
          f"(iron {C}), other crossing {np.round(circle_cross(A1, r1, A2, r2), 4)}", flush=True)
    if cheat == "mirror":  # towards the other crossing, as far out as a downward grip reaches
        tgt = np.array([0.37, float(tgt[1])])
    elif cheat == "centre":
        tgt = np.array([sum(MAT_X) / 2, sum(MAT_Y) / 2])
    elif cheat == "mid":  # towards the midpoint between the nails, as far out as a downward grip reaches
        tgt = np.array([0.37, 0.0])
    elif cheat == "swap":
        tgt = circle_cross(A1, r2, A2, r1, far=False)
    elif cheat == "t06":  # partial tiers: stake set 1.9 / 3.2 cm from the iron
        tgt = C + np.array([0.019, 0.0])
    elif cheat == "t03":
        tgt = C + np.array([0.032, 0.0])
    yield from place(Rt, tgt, hold=(cheat == "hold"), knock=(cheat == "knock"))
    yield from Rt.wait(1.5)
    log("end")
