"""An air-conditioner condensate drain hose must fall continuously from the unit to the drain.

On the station table, on the robot's left, stands a white indoor-unit block (12 x 10 x 20 cm). An 8 mm off-white
PVC condensate hose (5 mm bore), 56 cm long, leaves its right-hand face 14 cm above the table (outlet fixed) and lies
straight on the table towards the robot's right, ending in a grey 30 x 14 x 14 mm drain nozzle. Further from the
robot, two grey support blocks of decreasing height (12 and 9.5 cm) step down along a bent path from the unit to a
drain: a dark grey tundish (open box, inner 6.5 x 6.5 cm) standing on a 1.5 cm plinth, rim 3.5 cm above the table,
on the right. The nozzle must end inside the tundish with the hose falling all the way from the outlet (no trap):
carried straight across, the hose sags to the table and has to climb back up into the raised tundish, which forms a
water trap. Laid over the two supports, it falls all the way. Both grippers must let go.

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
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Route the air conditioner's white condensate hose into the drain tundish so that it slopes downhill "
               "all the way from the unit to the drain, with no dip that would trap water: lay it over the two grey "
               "steps and put the nozzle inside the tundish. Then let go of the hose.")

WHITE = (0.93, 0.93, 0.91)
GREY = (0.55, 0.57, 0.60)
DARK = (0.25, 0.26, 0.28)
HOSE_COLOR = (0.90, 0.92, 0.95)

# -- unit and hose -------------------------------------------------------------------------------------------------
R = 0.004  # 8 mm outside diameter hose
BORE = 0.005  # 5 mm inside diameter (scoring tolerance reference; the sim hose is a solid chain)
HOSE_LEN = 0.56
OUT_Z = 0.140  # outlet (hose centre) height above the table
OUT_XY = (0.14, 0.34)  # outlet position; the hose leaves towards -y
NOZZLE = Plug(size=(0.030, 0.014, 0.014), mass=0.012, color=(0.45, 0.46, 0.48))
HOSE_START = (OUT_XY[0], OUT_XY[1], TOP + OUT_Z)
UNIT_SIZE = (0.12, 0.10, 0.20)
UNIT = Box("ac_unit", pos=(OUT_XY[0], OUT_XY[1] + 0.006 + UNIT_SIZE[1] / 2, TOP + UNIT_SIZE[2] / 2), size=UNIT_SIZE,
           color=WHITE)
HOSE = cable("hose", start=HOSE_START, yaw=-math.pi / 2, length=HOSE_LEN, radius=R, anchor_start=True,
             end_plug=NOZZLE, cable_mass=0.03, color=HOSE_COLOR, bend_stiffness=0.01, bend_damping=0.001)

# -- drain: tundish (open box) on a plinth -------------------------------------------------------------------------
D_C = (0.22, -0.10)
PLINTH_H = 0.015
PLINTH = 0.11  # square
WELL_IN, WELL_WALL, WELL_H = 0.065, 0.008, 0.020
RIM = PLINTH_H + WELL_H  # 3.5 cm above the table
PLINTH_BOX = Box("drain_plinth", pos=(D_C[0], D_C[1], TOP + PLINTH_H / 2), size=(PLINTH, PLINTH, PLINTH_H),
                 color=GREY)
_wo = WELL_IN / 2 + WELL_WALL / 2
_wl = WELL_IN + 2 * WELL_WALL
_wz = TOP + PLINTH_H + WELL_H / 2
TUNDISH = [
    Box("tundish_near", pos=(D_C[0] - _wo, D_C[1], _wz), size=(WELL_WALL, _wl, WELL_H), color=DARK),
    Box("tundish_far", pos=(D_C[0] + _wo, D_C[1], _wz), size=(WELL_WALL, _wl, WELL_H), color=DARK),
    Box("tundish_left", pos=(D_C[0], D_C[1] + _wo, _wz), size=(WELL_IN, WELL_WALL, WELL_H), color=DARK),
    Box("tundish_right", pos=(D_C[0], D_C[1] - _wo, _wz), size=(WELL_IN, WELL_WALL, WELL_H), color=DARK),
]

# -- two supports along the bent path outlet -> S2 (corner) -> drain ---------------------------------------------
S2_C = (0.32, 0.14)
_a = math.atan2(S2_C[1] - OUT_XY[1], S2_C[0] - OUT_XY[0])  # leg A heading (outlet -> S2)
_b = math.atan2(D_C[1] - S2_C[1], D_C[0] - S2_C[0])  # leg B heading (S2 -> drain)
S1_C = ((OUT_XY[0] + S2_C[0]) / 2 + 0.01 * math.cos(_a), (OUT_XY[1] + S2_C[1]) / 2 + 0.01 * math.sin(_a))
S_H = (0.120, 0.095)  # support heights above the table
S1_SIZE = (0.04, 0.06)  # high support: 4 cm along the path (short level top), 6 cm across
S2_SIZE = (0.06, 0.06)  # low support at the bend, 6 x 6 cm
STEPS = [
    Box("step_high", pos=(S1_C[0], S1_C[1], TOP + S_H[0] / 2), size=(*S1_SIZE, S_H[0]), yaw=_a, color=GREY),
    Box("step_mid", pos=(S2_C[0], S2_C[1], TOP + S_H[1] / 2), size=(*S2_SIZE, S_H[1]), yaw=(_a + _b) / 2,
        color=GREY),
]

OBJECTS = [UNIT, HOSE, PLINTH_BOX, *TUNDISH, *STEPS]

# -- grippers (score requires both to have let go) ----------------------------------------------------------------
RELEASE_DIST = 0.035  # each gripper TCP (between the finger pads) at least this far from the hose and the nozzle
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_drain_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._drain_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads, 13 mm above the pad tips), [] if unknown."""
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


# -- scoring geometry -----------------------------------------------------------------------------------------------


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _line(objects) -> np.ndarray:
    """Hose centre line from the outlet to the nozzle's front face."""
    c = objects["hose"]
    poly = c.polyline()
    if c.end_plug is not None:
        poly = np.vstack([poly, c.end_tip + c.end_dir * NOZZLE.size[0]])
    return poly


def released(objects) -> bool:
    """Both gripper TCPs >= 3.5 cm from every point of the hose and nozzle. No gripper reading = not released."""
    g = grippers()
    if len(g) != 2:
        return False
    pts = _dense(_line(objects))
    return all(float(np.linalg.norm(pts - x, axis=1).min()) >= RELEASE_DIST for x in g)


def in_drain(objects) -> bool:
    """Nozzle centre inside the tundish's inner walls (6.5 x 6.5 cm opening) and below its rim."""
    p = objects["hose"].end_plug
    h = WELL_IN / 2
    return bool(p is not None and abs(p[0] - D_C[0]) <= h and abs(p[1] - D_C[1]) <= h and p[2] <= TOP + RIM)


# -- slope profile (the scored quantity) ---------------------------------------------------------------------------
# The hose centre line is sampled at the outlet, every 2 cm segment centre and the hose end (where it enters the
# nozzle), 30 points. Three numbers come from it:
#   max_rise:  the largest height by which any sample stands above any sample before it (0 for a line that never
#              climbs). A dip followed by a climb of h holds back water up to h deep, so this is the depth of the
#              deepest pocket. Full score needs <= 1.5 mm (RISE_TOL): under a third of the 5 mm bore, so any water held
#              is a film the air path above it stays open over; it cannot fill the bore and seal like a trap. 1.5 mm
#              is also about the reading error of a ruler or side photo, and above the sim's settling noise (<1 mm).
#   min_window_slope: over every stretch of hose whose horizontal run is at least 10 cm, fall / run; the smallest.
#              Full score needs >= 1/8 in per ft (1.04 %), the code minimum, so no 10 cm stretch is level or climbing.
#   mean_slope: total fall from the outlet to the hose end / total horizontal run. Full score needs >= 1.04 %.
RISE_TOL = 0.0015  # full score: no pocket deeper than 1.5 mm
RISE_HALF = 0.003  # 0.5: no pocket deeper than 3 mm (still under the 5 mm bore: a puddle, not a sealed trap)
MIN_SLOPE = 0.125 / 12.0  # 1/8 in per ft = 1.04 %
WINDOW = 0.10  # m of horizontal run per slope window


def _profile(objects):
    p = objects["hose"].polyline()
    z = p[:, 2]
    run = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))]
    return z, run


def max_rise(objects) -> float:
    """Largest height (m) any centre-line sample stands above the lowest sample before it."""
    z, _ = _profile(objects)
    low, worst = float(z[0]), 0.0
    for v in z[1:]:
        v = float(v)
        worst = max(worst, v - low)
        low = min(low, v)
    return worst


def min_window_slope(objects) -> float:
    """Smallest fall/run over any stretch of the centre line with >= 10 cm horizontal run (from each sample to the
    first sample at least 10 cm of run further on)."""
    z, run = _profile(objects)
    best = math.inf
    for i in range(len(z)):
        j = int(np.searchsorted(run, run[i] + WINDOW - 1e-9))
        if j >= len(z):
            break
        best = min(best, float((z[i] - z[j]) / (run[j] - run[i])))
    return best if math.isfinite(best) else -1.0


def mean_slope(objects) -> float:
    z, run = _profile(objects)
    return float((z[0] - z[-1]) / max(run[-1], 1e-6))


def trap_free(objects) -> bool:
    """The full-score profile rule: pocket <= 1.5 mm, every 10 cm stretch and the whole run fall >= 1.04 %."""
    return max_rise(objects) <= RISE_TOL and min_window_slope(objects) >= MIN_SLOPE and mean_slope(objects) >= MIN_SLOPE


def score(objects) -> float:
    """0 unless both grippers are >= 3.5 cm from the hose and nozzle, the nozzle centre is inside the tundish
    (inner walls, below the rim) and the hose rests on both supports (some hose segment inside each support's
    footprint, its underside at most 1.1 cm above the support top). Then by the centre-line profile: 1.0 if
    trap_free (deepest pocket <= 1.5 mm, every 10 cm stretch of run and the whole run falling >= 1/8 in per ft);
    0.5 if the deepest pocket is <= 3 mm; else 0."""
    if not in_drain(objects) or not released(objects) or not on_both_steps(objects):
        return 0.0
    if trap_free(objects):
        return 1.0
    return 0.5 if max_rise(objects) <= RISE_HALF else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["hose"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Nozzle centre to the tundish centre (m) plus the deepest pocket (m)."""
    p = objects["hose"].end_plug
    d = float(np.linalg.norm(p[:2] - np.array(D_C))) if p is not None else 1.0
    return d + max_rise(objects)


def _on_step(objects, i: int) -> bool:
    """Some hose segment rests on top of support i: its centre inside the support's footprint and 0-1.5 cm above the
    top (the 8 mm hose's underside touching or at most 1.1 cm above it)."""
    b = STEPS[i]
    pts = objects["hose"].points
    c, s = math.cos(b.yaw), math.sin(b.yaw)
    rel = pts[:, :2] - np.array(b.pos[:2])
    u = rel @ np.array([c, s])
    v = rel @ np.array([-s, c])
    dz = pts[:, 2] - (TOP + S_H[i])
    return bool(np.any((np.abs(u) <= b.size[0] / 2) & (np.abs(v) <= b.size[1] / 2) & (dz > 0) & (dz < 0.015)))


def on_both_steps(objects) -> bool:
    return _on_step(objects, 0) and _on_step(objects, 1)


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    c = objects["hose"]
    g = grippers()
    r = max_rise(objects)
    return {
        "end_grasped": int(c.end_plug is not None and any(float(np.linalg.norm(x - c.end_plug)) < 0.03 for x in g)),
        "on_step_high": int(_on_step(objects, 0)),
        "on_step_low": int(_on_step(objects, 1)),
        "end_in_drain": int(in_drain(objects)),
        "pocket_le_3mm": int(r <= RISE_HALF),
        "pocket_le_1_5mm": int(r <= RISE_TOL),
        "falls_1pct_every_10cm": int(min_window_slope(objects) >= MIN_SLOPE),
        "released": int(released(objects)),
    }


# Seeded reset variation for a later runner (the demo finds the nozzle and hose with the top camera). The outlet
# slides along the unit's face only (dy would push the welded start into the unit or away from it); the hose must
# leave the outlet square to the face (yaw 0). Every other part is fixed.
VARIATION = {"cable:hose": {"dx": 0.01, "dy": 0.0, "yaw": 0.0}}


# -- closed-loop scripted demo (cabledemo.py), camera only ---------------------------------------------------------
# Motions are chosen from the top camera image (vision.py), the arms' own TCPs and the fixed layout (unit, supports,
# drain, hose length). ctx.states() is read only by log() for the debug print.


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _see_hose(img):
    """Top-camera estimate of the hose lying on the table: nozzle centre (dark bluish-grey blob near the hose's far
    end, beyond the drain plinth) and the hose's line (median x of the unsaturated white pixels between y -3 and
    +12 cm, where only the hose lies). Windows are on the table plane and exclude the plinth, tundish and supports.
    Returns (nozzle_xy, hose_x), either None if not seen. Checked offline against the simulator: within 3 mm."""
    import vision

    cam = vision.top_camera(img.shape)
    f = img.astype(float)
    mx, mn = f.max(axis=2), f.min(axis=2)
    sat = (mx - mn) / (mx + 1e-9)
    hose_m = (mx > 150) & (sat < 0.05)  # white PVC; the oak table is saturated (~0.17)
    grey_m = (f[..., 2] >= f[..., 0]) & (mx < 120)  # grey nozzle (often in the arm's shadow); the table is r > b
    hp = cam.to_plane(hose_m, TOP + R)
    k = (hp[:, 0] > 0.06) & (hp[:, 0] < 0.24) & (hp[:, 1] > -0.03) & (hp[:, 1] < 0.12)
    hose_x = float(np.median(hp[k, 0])) if k.sum() >= 6 else None
    gp = cam.to_plane(grey_m, TOP + NOZZLE.size[2])
    k = (gp[:, 0] > 0.08) & (gp[:, 0] < 0.21) & (gp[:, 1] > -0.26) & (gp[:, 1] < -0.165)
    noz = np.median(gp[k], axis=0) if k.sum() >= 4 else None
    return noz, hose_x


def demo_script(ctx):
    """Both arms start from a top-camera look at the hose lying on the table. The left arm pinches the hose at the
    point that belongs on the low support (found along the seen hose line from the seen nozzle), the right arm pinches
    the nozzle; together they lift the hose, the left lays its point on the low support (the hose from the outlet
    drops onto the high support) while the right carries the nozzle, camera ahead, to the far side of the tundish
    (keeping the last span short of slack). The right lowers the nozzle to just above the tundish floor level with
    the rim and lets go, then the left lets go and both arms move clear.
    DRAIN_CHEAT=nothing | direct (right arm alone carries the nozzle into the tundish, the hose hanging free) |
    onestep (the left lays the hose on the high support only) | onestep_taut (the left picks the hose where it hangs
    near the outlet) | hold (full demo, right keeps holding the nozzle in the tundish) | sag (full demo but the
    left takes 6 mm less hose and the nozzle goes 5 mm short of the tundish centre, so the last span sags a little before
    the rim: passed the old 5 mm total-rise rule, must score below 1.0). DRAIN_CHEAT=settle only waits (debug)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    import vision

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("DRAIN_CHEAT", "")
    GRASP_OPEN = -0.014  # pads 28 mm apart round the 8 mm hose
    over = 0.010  # TCP above the hose / nozzle centre when grasping
    hi = TOP + 0.16

    def log(tag):
        st = ctx.states()
        c = st["hose"]
        print(f"[demo t={ctx.t:5.1f}] {tag}: nozzle={np.round(c.end_plug, 3)} pocket={max_rise(st)*1000:.1f}mm "
              f"winslope={min_window_slope(st)*100:.2f}% mean={mean_slope(st)*100:.2f}% "
              f"oldrise={_old_total_rise(st)*1000:.1f}mm drain={in_drain(st)} rel={released(st)} "
              f"score={score(st):.2f} prog={progress(st)}", flush=True)
        print("   z(mm):", np.round((c.polyline()[:, 2] - TOP) * 1000, 1).tolist(), flush=True)

    log("start")
    yield from L.wait(1.5)
    log("settled")
    if cheat in ("settle", "nothing"):
        yield from L.wait(3.0)
        log("end")
        return

    # -- look: nozzle blob and hose line from the top camera (arms are at READY, beside the table centre)
    noz, hose_x = None, None
    for _ in range(8 if np.asarray(ctx.obs.images["top_cam"]).shape[1] >= 200 else 0):
        n2, h2 = _see_hose(vision.top_image(ctx))
        noz = n2 if n2 is not None else noz
        hose_x = h2 if h2 is not None else hose_x
        if noz is not None and hose_x is not None:
            break
        yield from L.wait(0.2)
    if noz is None or hose_x is None:
        # not seen (e.g. low-resolution debug render): the reset position from the setup sheet
        print("[demo] camera: hose/nozzle not seen, using the reset layout", flush=True)
        noz = noz if noz is not None else np.array([OUT_XY[0], -0.194])
        hose_x = hose_x if hose_x is not None else OUT_XY[0]
    if os.environ.get("DRAIN_DEBUG"):
        d = pathlib.Path(os.environ.get("DRAIN_DEBUG"))
        d.mkdir(parents=True, exist_ok=True)
        np.save(d / "top_start.npy", np.asarray(ctx.obs.images["top_cam"]))
    st = ctx.states()  # debug comparison only (printed, never used for motion)
    print(f"[demo] camera: nozzle {np.round(noz, 3)} (sim {np.round(st['hose'].end_plug[:2], 3)}), hose x "
          f"{hose_x:.3f}", flush=True)

    out = np.array(HOSE_START)
    # where the left arm will put its hose point: on the low support, 1 cm before its centre along leg A
    put = np.array([S2_C[0] - 0.01 * math.cos(_a), S2_C[1] - 0.01 * math.sin(_a), TOP + S_H[1] + R])
    tgt_step = 0 if cheat.startswith("onestep") else 1
    if tgt_step == 0:
        put = np.array([S1_C[0] + 0.01 * math.cos(_a), S1_C[1] + 0.01 * math.sin(_a), TOP + S_H[0] + R])
    a_left = float(np.linalg.norm(put - out)) + float(os.environ.get("DRAIN_SLACK", "0.006"))
    if cheat == "sag":
        a_left -= float(os.environ.get("DRAIN_SAG_A", "0.006"))
    if cheat == "onestep":
        a_left = 0.25  # a point lying on the table; the extra length stays on the high support
    # the hose lies straight along x = hose_x from the outlet to the nozzle; the hose end is 15 mm on the outlet
    # side of the nozzle centre, so the point a_left from the outlet is (HOSE_LEN - a_left) from the hose end
    end_y = noz[1] + NOZZLE.size[0] / 2
    pl = np.array([hose_x, end_y + (HOSE_LEN - a_left), TOP + 0.009])
    yl = -math.pi / 2  # gripper yaw along the hose (it runs along -y): fingers close across it
    if cheat == "onestep_taut":
        pl = np.array([hose_x, OUT_XY[1] - 0.14, TOP + 0.03])  # where it hangs from the outlet, a few cm up
    pn = np.array([noz[0], noz[1], TOP + NOZZLE.size[2] / 2])
    yn = math.pi / 2  # same grasp as yaw -90, IK-friendly for the right arm
    y_carry = math.radians(40.0)  # wrist camera ahead, towards the tundish
    print(f"[demo] left grasps a={a_left:.3f} at {np.round(pl, 3)} yaw {math.degrees(yl):.0f}; nozzle {np.round(pn, 3)}"
          f" yaw {math.degrees(yn):.0f}", flush=True)

    def right_take():
        yield from Rt.move((pn[0], pn[1], TOP + 0.06), yaw=yn, finger=FINGER_OPEN, duration=1.5)
        yield from Rt.move((pn[0], pn[1], pn[2] + over), yaw=yn, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED, 0.5)

    def left_take():
        yield from L.move((pl[0], pl[1], TOP + 0.06), yaw=yl, finger=GRASP_OPEN, duration=1.5)
        yield from L.move((pl[0], pl[1], pl[2] + over), yaw=yl, duration=0.8)
        yield from L.grip(FINGER_CLOSED, 0.5)

    if cheat == "direct":
        yield from right_take()
    else:
        yield from ctx.both(right_take(), left_take())
    log("grasped")

    # nozzle goes 1 cm beyond the tundish centre along leg B (far half; the hose enters over the near wall), or
    # 5 mm short of the centre for the sag variant
    off = float(os.environ.get("DRAIN_SAG_OFF", "-0.005")) if cheat == "sag" else 0.010
    dc = (D_C[0] + off * math.cos(_b), D_C[1] + off * math.sin(_b))
    above_d = (dc[0], dc[1], hi)
    seat = (dc[0], dc[1], TOP + RIM + NOZZLE.size[2] / 2 + 0.010 + over)
    ys = _a if tgt_step == 0 else (_a + _b) / 2
    ys = _grip_yaw(ys)
    if cheat == "direct":
        yield from Rt.move((pn[0], pn[1], hi), yaw=yn, duration=0.8)
        yield from Rt.move(above_d, yaw=y_carry, duration=2.0)
    else:
        yield from ctx.both(Rt.move((pn[0] + 0.02, pn[1], hi), yaw=yn, duration=1.0),
                            L.move((pl[0], pl[1], hi), yaw=yl, duration=1.0))
        yield from ctx.both(Rt.move(above_d, yaw=y_carry, duration=2.0),
                            L.move((put[0], put[1], hi), yaw=ys, duration=2.0))
        log("lifted")
        yield from L.move((put[0], put[1], put[2] + over), yaw=ys, duration=1.5)
        log("left on step")
    yield from Rt.move(seat, yaw=y_carry, duration=1.2)
    log("nozzle over tundish")
    if cheat == "hold":
        yield from L.grip(GRASP_OPEN, 0.5)
        yield from L.move((put[0], put[1], hi), yaw=ys, duration=0.8)
        yield from L.move((0.15, 0.25, TOP + 0.22), yaw=0.0, duration=1.2)
        yield from Rt.move((dc[0], dc[1], TOP + PLINTH_H + 0.007 + over + 0.004), yaw=y_carry, duration=1.0)
        yield from Rt.wait(3.0)
        log("end (held)")
        return
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move(above_d, yaw=y_carry, duration=0.8)
    # out of the top camera's view: beside the right arm's base, low
    yield from Rt.move((0.08, -0.52, TOP + 0.10), yaw=y_carry, duration=1.0)
    log("right let go")
    if cheat != "direct":
        # let go, straight up 4 cm, then sweep out to the left past the far side of the unit and back beside the
        # left base, so the top camera sees the hose over both supports
        yield from L.grip(GRASP_OPEN, 0.5)
        yield from L.move((put[0], put[1], put[2] + over + 0.04), yaw=ys, duration=0.4)
        yield from L.move((0.30, 0.50, TOP + 0.15), yaw=ys, duration=0.5)
        yield from L.move((0.04, 0.58, TOP + 0.10), yaw=0.0, duration=0.6)
    log("released")
    yield from L.wait(1.5)
    log("end")


def _old_total_rise(objects) -> float:
    """The previous version's full-score measure (sum of climbs with a 2 mm dead band; full score was <= 5 mm). Kept
    only for the debug log, to show that the sag variant passed the old rule."""
    z = objects["hose"].polyline()[:, 2]
    low, rise = float(z[0]), 0.0
    for v in z[1:]:
        v = float(v)
        if v < low:
            low = v
        elif v - low > 0.002:
            rise += v - low
            low = v
    return rise
