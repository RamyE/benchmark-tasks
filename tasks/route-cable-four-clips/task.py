"""On the station table: a 6 mm black cable (52 cm long) comes out of the -y face of a grey anchor block fixed
at the left rear of the table and lies straight across the table towards the front right, ending in a dark
plug body. Four white open-top U-clips, numbered 1-4 from left to right, stand in a straight row 36 cm in
front of the arms, parallel to the arm baseline (clip centres at y = +17, +6.5, -4.5, -15 cm); each clip has a
10 mm wide, 14 mm tall slot whose axis runs left-right. A shallow open-top orange tray (8 x 9 cm inside) sits at
the right end of the row. The cable must be pressed down into clips 1, 2, 3, 4 in order, starting from the
anchor, and the plug laid in the tray.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "Starting at the grey block, press the cable down into the four white clips in order from left to right, then put the plug in the orange tray."
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

# -- route line: the four clips and the tray lie on the line x = ROUTE_X, the cable runs along -y ---------------
ROUTE_X = 0.36
CLIP_Y = (0.17, 0.065, -0.045, -0.15)
ROUTE_DIR = np.array([0.0, -1.0])
ROUTE_YAW = -math.pi / 2

# -- cable and anchor block --------------------------------------------------------------------------------
R = 0.003  # 6 mm cable
CORD_LEN = 0.52
SEG = 0.02
CORD_YAW = math.radians(-105)  # starting layout: straight from the anchor towards the front right
_D = np.array([math.cos(CORD_YAW), math.sin(CORD_YAW)])
# The anchor weld fixes the first 2 cm segment; place it so that segment ends on the route line.
SEG0_END_Y = 0.23
_A = np.array([ROUTE_X, SEG0_END_Y]) - SEG * _D  # cable start (exit point on the anchor face)
CORD_START = (float(_A[0]), float(_A[1]), TOP + R + 0.0005)
PLUG = Plug(size=(0.035, 0.016, 0.012), mass=0.015, color=(0.15, 0.15, 0.17))
CORD = cable("cord", start=CORD_START, yaw=CORD_YAW, length=CORD_LEN, radius=R, segment=SEG, anchor_start=True,
             end_plug=PLUG, color=(0.05, 0.05, 0.05), bend_stiffness=0.01, bend_damping=0.001)
ANCHOR_SIZE = (0.04, 0.04, 0.03)
_AC = _A - (ANCHOR_SIZE[0] / 2 + 0.003) * _D
ANCHOR = Box("anchor_block", pos=(float(_AC[0]), float(_AC[1]), TOP + ANCHOR_SIZE[2] / 2), size=ANCHOR_SIZE,
             yaw=CORD_YAW, color=(0.45, 0.47, 0.50))

# -- U-clips (3D-printed): two walls on a thin base -----------------------------------------------------------
GAP = 0.010  # slot width
WALL_T = 0.005
WALL_L = 0.020  # along the slot
WALL_H = 0.014  # above the table
BASE_H = 0.002  # base plate under the slot
CLIP_COLOR = (0.93, 0.93, 0.92)


def _clips() -> list[Box]:
    out = []
    for k, y in enumerate(CLIP_Y, start=1):
        out.append(Box(f"clip{k}_base", pos=(ROUTE_X, y, TOP + BASE_H / 2), size=(GAP + 2 * WALL_T, WALL_L, BASE_H),
                       color=CLIP_COLOR))
        for side, sgn in (("a", 1), ("b", -1)):
            out.append(Box(f"clip{k}_wall{side}", pos=(ROUTE_X + sgn * (GAP / 2 + WALL_T / 2), y, TOP + WALL_H / 2),
                           size=(WALL_T, WALL_L, WALL_H), color=CLIP_COLOR))
    return out


CLIPS = _clips()

# -- tray (open top) at the right end of the row -------------------------------------------------------------
TRAY_Y = -0.268
TRAY_IN = (0.08, 0.09)  # inside, x by y
TRAY_WALL_T = 0.004
TRAY_WALL_H = 0.012  # above the table
TRAY_FLOOR = 0.003
TRAY_COLOR = (0.95, 0.55, 0.15)


def _tray() -> list[Box]:
    ox, oy = TRAY_IN[0] + 2 * TRAY_WALL_T, TRAY_IN[1] + 2 * TRAY_WALL_T
    z = TOP + TRAY_WALL_H / 2
    return [
        Box("tray_floor", pos=(ROUTE_X, TRAY_Y, TOP + TRAY_FLOOR / 2), size=(ox, oy, TRAY_FLOOR), color=TRAY_COLOR),
        Box("tray_wall_n", pos=(ROUTE_X, TRAY_Y + oy / 2 - TRAY_WALL_T / 2, z), size=(ox, TRAY_WALL_T, TRAY_WALL_H),
            color=TRAY_COLOR),
        Box("tray_wall_s", pos=(ROUTE_X, TRAY_Y - oy / 2 + TRAY_WALL_T / 2, z), size=(ox, TRAY_WALL_T, TRAY_WALL_H),
            color=TRAY_COLOR),
        Box("tray_wall_f", pos=(ROUTE_X + ox / 2 - TRAY_WALL_T / 2, TRAY_Y, z), size=(TRAY_WALL_T, TRAY_IN[1], TRAY_WALL_H),
            color=TRAY_COLOR),
        Box("tray_wall_r", pos=(ROUTE_X - ox / 2 + TRAY_WALL_T / 2, TRAY_Y, z), size=(TRAY_WALL_T, TRAY_IN[1], TRAY_WALL_H),
            color=TRAY_COLOR),
    ]


OBJECTS = [ANCHOR, CORD, *CLIPS, *_tray()]

# -- scoring --------------------------------------------------------------------------------------------------
SEATED_Z = TOP + WALL_H - R  # cord centre at most this high: the cord's top is not above the clip walls
MIN_SPAN = 0.012  # the cord must run through at least 12 mm of the 20 mm slot


def _dense(poly: np.ndarray, step: float = 0.002) -> tuple[np.ndarray, np.ndarray]:
    """Points every ~2 mm along a polyline, with their arclength from the first point."""
    pts, arc = [poly[0]], [0.0]
    s = 0.0
    for a, b in zip(poly[:-1], poly[1:]):
        seg = float(np.linalg.norm(b - a))
        k = max(1, int(seg / step))
        for j in range(1, k + 1):
            pts.append(a + (b - a) * (j / k))
            arc.append(s + seg * j / k)
        s += seg
    return np.array(pts), np.array(arc)


def clip_arclengths(objects) -> list[float | None]:
    """Per clip: arclength (m from the anchor) where the cord runs through its slot, seated; None if not seated."""
    pts, arc = _dense(objects["cord"].polyline())
    out = []
    for y in CLIP_Y:
        lat = np.abs(pts[:, 0] - ROUTE_X)
        along = pts[:, 1] - y
        inside = (lat <= GAP / 2) & (np.abs(along) <= WALL_L / 2) & (pts[:, 2] <= SEATED_Z)
        if inside.any() and np.ptp(along[inside]) >= MIN_SPAN:
            out.append(float(arc[inside].mean()))
        else:
            out.append(None)
    return out


def clips_in_order(objects) -> int:
    """Number of clips seated in order: clips 1..n all seated, reached one after another along the cord."""
    n, last = 0, -1.0
    for s in clip_arclengths(objects):
        if s is None or s <= last:
            break
        n, last = n + 1, s
    return n


def plug_in_tray(objects) -> bool:
    p = objects["cord"].end_plug
    return bool(abs(p[0] - ROUTE_X) <= TRAY_IN[0] / 2 and abs(p[1] - TRAY_Y) <= TRAY_IN[1] / 2
                and p[2] <= TOP + TRAY_WALL_H)


def score(objects) -> float:
    """0.2 per clip seated in order (1, 1-2, 1-2-3, 1-2-3-4); +0.2 for the plug in the tray once all four are."""
    if not np.isfinite(objects["cord"].points).all():  # a simulation blow-up never scores
        return 0.0
    n = clips_in_order(objects)
    return 0.2 * n + (0.2 if n == 4 and plug_in_tray(objects) else 0.0)


def success(objects) -> bool:
    return score(objects) >= 0.999


def distance(objects) -> float:
    """Route length (m) not yet done: the clip row (10 cm per clip) plus 10 cm for the plug."""
    return float(round((1.0 - score(objects)) / 0.2) * 0.10)


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------


def _cord_at(objects, a: float) -> tuple[np.ndarray, float]:
    """Live cord point at arclength ``a`` from the anchor, and the cord's heading there."""
    poly = objects["cord"].polyline()
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    p = np.array([np.interp(a, s, poly[:, k]) for k in range(3)])
    i = min(max(int(np.searchsorted(s, a)), 1), len(poly) - 1)
    d = poly[i] - poly[i - 1]
    return p, math.atan2(d[1], d[0])


def _arc_near(objects, xy) -> float:
    """Arclength of the cord point nearest to ``xy``."""
    pts, arc = _dense(objects["cord"].polyline())
    return float(arc[np.argmin(np.linalg.norm(pts[:, :2] - np.asarray(xy), axis=1))])


def _cord_dx(objects, y: float, a_lo: float = 0.0, a_hi: float = 1e9) -> float:
    """Sideways offset (m, +x) of the cord section between arclengths a_lo..a_hi where it crosses ``y``."""
    pts, arc = _dense(objects["cord"].polyline())
    sel = (arc >= a_lo) & (arc <= a_hi)
    pts = pts[sel] if sel.any() else pts
    i = int(np.argmin(np.abs(pts[:, 1] - y) + np.abs(pts[:, 0] - ROUTE_X)))
    return float(pts[i, 0] - ROUTE_X)


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


DEMO = dict(lead=0.035, taut=0.004, carry=0.05, plug_slack=-0.002, debug=True)


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    over = 0.010  # TCP above the cord centre when grasping it
    hi = TOP + 0.07
    carry = TOP + DEMO["carry"]
    ry = _grip_yaw(ROUTE_YAW)  # fingers close across the route line

    def log(msg):
        if DEMO["debug"]:
            st = ctx.states()
            pts, _ = _dense(st["cord"].polyline())
            at = []
            for y in CLIP_Y:
                i = int(np.argmin(np.abs(pts[:, 1] - y) + np.abs(pts[:, 0] - ROUTE_X)))
                at.append(f"({(pts[i, 0] - ROUTE_X) * 1000:+.0f},{(pts[i, 2] - TOP) * 1000:.0f})")
            print(f"[demo t={ctx.t:5.1f}] {msg}  clips={clip_arclengths(st)} dx,z(mm)={' '.join(at)} "
                  f"plug={np.round(st['cord'].end_plug, 3)} score={score(st):.2f}", flush=True)

    def seat(arm, k, prev_xy, hold=False):
        """Take the cord a bit past clip k (pulled taut from prev_xy), lay it over the slot and down."""
        st = ctx.states()
        tgt = np.array([ROUTE_X, CLIP_Y[k] - DEMO["lead"]])
        a0 = _arc_near(st, prev_xy)
        a = a0 + float(np.linalg.norm(tgt - np.asarray(prev_xy))) - DEMO["taut"]
        for attempt in range(3):  # closed loop: re-grasp if the cord is not in the fingers after lifting
            st = ctx.states()
            p, h = _cord_at(st, a)
            gy = _grip_yaw(h)
            yield from arm.move((p[0], p[1], hi), yaw=gy, finger=FINGER_OPEN, duration=1.2)
            yield from arm.move((p[0], p[1], p[2] + over), yaw=gy, duration=0.8)
            yield from arm.grip(FINGER_CLOSED, 0.5)
            yield from arm.move((p[0], p[1], carry), yaw=gy, duration=0.6)
            q, _ = _cord_at(ctx.states(), a)
            miss = float(np.linalg.norm(q - (arm.tcp() - np.array([0.0, 0.0, over]))))
            log(f"  grasp attempt {attempt} miss={miss * 1000:.0f}mm")
            if miss < 0.02:
                break
            yield from arm.grip(FINGER_OPEN, 0.4)
        ty = math.copysign(math.pi / 2, gy)  # across the route, turning the wrist the short way from gy
        yield from arm.move((tgt[0], tgt[1], carry), yaw=ty, duration=1.2)
        # Closed loop: shift the hand sideways until the cord crosses the slot centre, then lower in steps.
        prev = np.asarray(prev_xy)
        lever = float(np.linalg.norm(tgt - prev) / max(np.linalg.norm(np.array([ROUTE_X, CLIP_Y[k]]) - prev), 1e-3))
        gx = tgt[0]
        for z in (carry, carry, TOP + 0.04, TOP + 0.04):
            yield from arm.move((gx, tgt[1], z), yaw=ty, duration=0.5)
            yield from arm.wait(0.2)
            off = _cord_dx(ctx.states(), CLIP_Y[k], a0 - 0.01, a + 0.005)
            gx -= float(np.clip(off * lever, -0.01, 0.01))
            log(f"  correct z={z - TOP:.3f} off={off * 1000:+.1f}mm -> gx={gx:.4f} tcp={np.round(arm.tcp(), 4)}")
        yield from arm.move((gx, tgt[1], TOP + BASE_H + R + over), yaw=ty, duration=0.8)
        yield from arm.wait(0.3)
        log(f"clip {k + 1} lowered (grip a={a:.3f})")
        if not hold:
            yield from arm.grip(FINGER_OPEN, 0.5)
            yield from arm.wait(0.3)
            yield from arm.move((gx, tgt[1], hi), yaw=ty, duration=1.0)

    def plug_to_tray():
        st = ctx.states()
        pp = st["cord"].end_plug
        ed = st["cord"].end_dir
        py = _grip_yaw(math.atan2(ed[1], ed[0]))
        pty = math.copysign(math.pi / 2, py)
        yield from Rt.move((pp[0], pp[1], hi), yaw=py, finger=FINGER_OPEN, duration=1.2)
        yield from Rt.move((pp[0], pp[1], pp[2] + 0.004), yaw=py, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED, 0.5)
        yield from Rt.move((pp[0], pp[1], TOP + 0.05), yaw=py, duration=0.6)
        yield from Rt.move((ROUTE_X, TRAY_Y, TOP + 0.05), yaw=pty, duration=1.2)
        yield from Rt.move((ROUTE_X, TRAY_Y, TOP + TRAY_WALL_H + 0.015), yaw=pty, duration=0.8)
        yield from Rt.grip(0.5 * FINGER_OPEN, 0.4)  # half open: the pads stay inside the tray walls
        yield from ctx.both(Rt.move((ROUTE_X, TRAY_Y, hi), yaw=pty, duration=0.8),
                            L.grip(FINGER_OPEN, 0.4))

    def last_clip_and_plug():
        """Right arm takes the plug and lays the cord from clip 3 over clip 4 into the tray (one straight pull)."""
        st = ctx.states()
        a3 = _arc_near(st, (ROUTE_X, CLIP_Y[2]))
        a_plug = CORD_LEN + PLUG.size[0] / 2
        y_p = CLIP_Y[2] - (a_plug - a3) + DEMO["plug_slack"]
        lever = (CLIP_Y[2] - y_p) / (CLIP_Y[2] - CLIP_Y[3])
        pp = st["cord"].end_plug
        ed = st["cord"].end_dir
        py = _grip_yaw(math.atan2(ed[1], ed[0]))
        pty = math.copysign(math.pi / 2, py)
        yield from Rt.move((pp[0], pp[1], hi), yaw=py, finger=FINGER_OPEN, duration=1.2)
        yield from Rt.move((pp[0], pp[1], pp[2] + 0.004), yaw=py, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED, 0.5)
        lift = TOP + 0.12  # high, so the cord clears clip 4's walls while it swings over the row
        yield from Rt.move((pp[0], pp[1], lift), yaw=py, duration=1.0)
        gx = ROUTE_X
        yield from Rt.move((gx, y_p, lift), yaw=pty, duration=1.4)
        for z in (lift, lift, TOP + 0.08, TOP + 0.06, TOP + 0.045):
            yield from Rt.move((gx, y_p, z), yaw=pty, duration=0.5)
            yield from Rt.wait(0.2)
            off = _cord_dx(ctx.states(), CLIP_Y[3], a3 + 0.01, CORD_LEN)
            gx -= float(np.clip(off * lever, -0.01, 0.01))
            gx = float(np.clip(gx, ROUTE_X - 0.02, ROUTE_X + 0.02))
            log(f"  plug correct z={z - TOP:.3f} off={off * 1000:+.1f}mm -> gx={gx:.4f}")
        yield from Rt.move((gx, y_p, TOP + TRAY_FLOOR + PLUG.size[2] / 2 + 0.006), yaw=pty, duration=1.0)
        yield from Rt.wait(0.3)
        log("clip 4 + plug lowered")
        yield from Rt.grip(0.5 * FINGER_OPEN, 0.5)  # half open: the pads stay inside the tray walls
        yield from Rt.wait(0.3)
        yield from Rt.move((gx, y_p, hi), yaw=pty, duration=1.0)

    start_xy = (ROUTE_X, SEG0_END_Y)
    cheat = os.environ.get("ROUTE_CHEAT", "")
    if cheat == "nothing":  # cheat 1: do nothing
        yield from L.wait(20.0)
        return
    if cheat == "plug_only":  # cheat 2: skip all clips, drag the plug straight into the tray
        yield from plug_to_tray()
        log("plug only")
        yield from Rt.move((0.25, -0.25, hi), yaw=0.0, duration=1.0)
        return
    if cheat == "skip_clip1":  # cheat 3: skip clip 1, route clips 2-4 and the plug
        yield from seat(L, 1, start_xy)
        log("clip 2 (clip 1 skipped)")
        yield from seat(Rt, 2, (ROUTE_X, CLIP_Y[1]))
        yield from last_clip_and_plug()
        yield from L.grip(FINGER_OPEN, 0.4)
        log("skip_clip1 done")
        return
    # Left arm: clips 1 and 2 (seated one after the other, each pulled taut from the previous one).
    yield from seat(L, 0, start_xy)
    log("clip 1")
    yield from seat(L, 1, (ROUTE_X, CLIP_Y[0]))
    log("clip 2")
    # Left arm pins the routed cord on the table just before clip 2 (clear of the right arm), so it stays seated
    # while the right arm pulls the rest of the cord along.
    pin = (ROUTE_X, CLIP_Y[1] + DEMO["lead"])
    yield from L.move((pin[0], pin[1], hi), yaw=ry, finger=FINGER_OPEN, duration=0.8)
    yield from L.move((pin[0], pin[1], TOP + R + over), yaw=ry, duration=0.8)
    yield from L.grip(FINGER_CLOSED, 0.5)
    log("left pins")
    # Right arm: clips 3 and 4, then the plug.
    yield from seat(Rt, 2, (ROUTE_X, CLIP_Y[1]))
    log("clip 3")
    yield from last_clip_and_plug()
    log("clip 4 + plug")
    yield from L.grip(FINGER_OPEN, 0.4)
    log("plug")

    def press(arm, ks):
        """Closed loop: press the cord flat beside any clip it has ridden up in."""
        for k in ks:
            if clip_arclengths(ctx.states())[k] is not None:
                continue
            for dy in (0.022, -0.022):
                y = CLIP_Y[k] + dy
                yield from arm.move((ROUTE_X, y, TOP + 0.04), yaw=0.0, finger=FINGER_CLOSED, duration=0.8)
                yield from arm.move((ROUTE_X, y, TOP + 2 * R + 0.012), yaw=0.0, duration=0.6)
                yield from arm.move((ROUTE_X, y, TOP + 0.04), yaw=0.0, duration=0.5)

    yield from ctx.both(press(L, (0, 1)), press(Rt, (2, 3)))
    log("pressed")
    yield from ctx.both(Rt.move((0.25, -0.25, hi), yaw=0.0, duration=1.0),
                        L.move((ROUTE_X, CLIP_Y[1] + DEMO["lead"], hi), yaw=ry, duration=1.0))
    log("done")
