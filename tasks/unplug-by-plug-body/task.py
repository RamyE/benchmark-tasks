"""On the station table: a white 3D-printed wall socket block (80 x 60 x 60 mm, fixed) stands at the right front of
the table with its socket opening facing left. A black 6 mm round cord, 35 cm long, comes out of a grey anchor block
(40 x 40 x 45 mm, fixed) at the left front of the table, runs straight to the right in front of both arms and ends in
a black rigid plug body (50 x 20 x 14 mm). Half of the plug (25 mm) is pushed into the socket's 21.5 x 15.5 mm
cavity, whose floor is 20 mm above the table; the other 25 mm of the plug sticks out of the socket face. The cord
hangs from the plug down to the table. Nothing latches the plug: it is held only by the cavity walls and friction.
In front of the right arm is a white open tray (inner 70 x 100 mm, the long side left-right, 20 mm walls), fixed to the table.
The plug must be unplugged by gripping the plug body (not the cord) and laid in the tray.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "Unplug the cable from the wall socket by gripping the plug body, not the cable, and lay the plug in the tray."
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

WHITE = (0.93, 0.93, 0.92)
GREY = (0.45, 0.47, 0.50)

# -- plug and cord ------------------------------------------------------------------------------------------
R = 0.003  # 6 mm cord
CORD_LEN = 0.35
PL, PW, PH = 0.055, 0.020, 0.014  # plug body: length along the cord, width, height
INSERT = 0.025  # how far the plug is pushed into the socket at reset
SX = 0.30  # socket axis x (the cord runs along y at this x)
FLOOR_Z = TOP + 0.020  # cavity floor
PLUG_Z = FLOOR_Z + PH / 2 + 0.0005  # plug (and cord) centre height at spawn
ANCHOR_Y = 0.21  # where the cord leaves the anchor block
CORD = cable("cord", start=(SX, ANCHOR_Y, PLUG_Z), yaw=-math.pi / 2, length=CORD_LEN, radius=R,
             anchor_start=True, end_plug=Plug(size=(PL, PW, PH), mass=0.015, color=(0.05, 0.05, 0.05)),
             color=(0.06, 0.06, 0.06), bend_stiffness=0.01, bend_damping=0.001)
PLUG_HOME = np.array([SX, ANCHOR_Y - CORD_LEN - PL / 2, PLUG_Z])  # plug centre at spawn
MOUTH_Y = PLUG_HOME[1] + PL / 2 - (PL - INSERT)  # socket face (the plug sticks out of it towards +y)

# -- wall socket (fixed, built from boxes): cavity 21.5 mm wide, 15.5 mm tall, 26 mm deep -------------------
CAV_W, CAV_H = PW + 0.0015, PH + 0.0015
CAV_BACK_Y = MOUTH_Y - INSERT - 0.001
SOCK_W, SOCK_BACK_Y, SOCK_TOP = 0.08, MOUTH_Y - 0.06, FLOOR_Z + CAV_H + 0.006
_cd = MOUTH_Y - CAV_BACK_Y  # cavity depth
_side_w = (SOCK_W - CAV_W) / 2
SOCKET = [
    Box("sock_base", pos=(SX, (MOUTH_Y + SOCK_BACK_Y) / 2, (TOP + FLOOR_Z) / 2), size=(SOCK_W, 0.06, FLOOR_Z - TOP),
        color=WHITE),
    Box("sock_back", pos=(SX, (CAV_BACK_Y + SOCK_BACK_Y) / 2, (FLOOR_Z + SOCK_TOP) / 2),
        size=(SOCK_W, CAV_BACK_Y - SOCK_BACK_Y, SOCK_TOP - FLOOR_Z), color=WHITE),
    Box("sock_side_a", pos=(SX - CAV_W / 2 - _side_w / 2, (MOUTH_Y + CAV_BACK_Y) / 2, (FLOOR_Z + SOCK_TOP) / 2),
        size=(_side_w, _cd, SOCK_TOP - FLOOR_Z), color=WHITE),
    Box("sock_side_b", pos=(SX + CAV_W / 2 + _side_w / 2, (MOUTH_Y + CAV_BACK_Y) / 2, (FLOOR_Z + SOCK_TOP) / 2),
        size=(_side_w, _cd, SOCK_TOP - FLOOR_Z), color=WHITE),
    Box("sock_top", pos=(SX, (MOUTH_Y + CAV_BACK_Y) / 2, (FLOOR_Z + CAV_H + SOCK_TOP) / 2),
        size=(CAV_W, _cd, SOCK_TOP - FLOOR_Z - CAV_H), color=WHITE),
]

# -- anchor block (the cord's other end is fixed in it) ----------------------------------------------------
ANCHOR = Box("anchor_block", pos=(SX, ANCHOR_Y + 0.002 + 0.02, TOP + 0.0225), size=(0.04, 0.04, 0.045), color=GREY)

# -- tray (fixed): inner 70 x 100 mm (x by y), 4 mm floor, 20 mm walls 3 mm thick -------------------------------------
TRAY_C = np.array([0.20, -0.06])
TRAY_IN = (0.07, 0.10)
TRAY_FLOOR, TRAY_WALL_H, TRAY_T = 0.004, 0.020, 0.003


def _tray() -> list[Box]:
    x, y = TRAY_C
    ix, iy = TRAY_IN
    ox, oy = ix + 2 * TRAY_T, iy + 2 * TRAY_T
    zc = TOP + TRAY_WALL_H / 2
    return [
        Box("tray_floor", pos=(x, y, TOP + TRAY_FLOOR / 2), size=(ox, oy, TRAY_FLOOR), color=WHITE),
        Box("tray_wall_n", pos=(x + ix / 2 + TRAY_T / 2, y, zc), size=(TRAY_T, oy, TRAY_WALL_H), color=WHITE),
        Box("tray_wall_s", pos=(x - ix / 2 - TRAY_T / 2, y, zc), size=(TRAY_T, oy, TRAY_WALL_H), color=WHITE),
        Box("tray_wall_e", pos=(x, y - iy / 2 - TRAY_T / 2, zc), size=(ix, TRAY_T, TRAY_WALL_H), color=WHITE),
        Box("tray_wall_w", pos=(x, y + iy / 2 + TRAY_T / 2, zc), size=(ix, TRAY_T, TRAY_WALL_H), color=WHITE),
    ]


TRAY = _tray()
OBJECTS = [CORD, *SOCKET, ANCHOR, *TRAY]

# -- scoring ------------------------------------------------------------------------------------------------
# What a real trial measures (top-camera video + tape marks on the cord 3 cm and 15 cm behind the plug):
#  * CORD PULL: while any part of the plug is still inside the socket, a point of the cord 3-15 cm behind the
#    plug moves more than 1.5 cm (plus however far the plug itself has moved) from where it rested at the start.
#    Gripping the plug body moves only the plug (the cord just follows it); pulling or lifting the cord moves the
#    cord before the plug comes out.
#  * DANGLE: once the plug is out of the socket and not in the tray, its body tilts more than 30 deg from
#    horizontal, i.e. it hangs from the cord instead of being held by its body.
#  Either one latches for the rest of the trial and the trial scores 0.
ZONE = (0.03, 0.15)  # cord arclength behind the plug watched for a cord pull (m)
CORD_MOVE_TOL = 0.015  # m
DANGLE_DEG = 30.0
DROP_Z = FLOOR_Z - 0.008  # plug centre below this outside socket and tray: it fell / was dragged on the table
IN_TRAY_Z = TOP + TRAY_FLOOR + 0.015  # plug centre at most this high: lying on the tray floor
ON_TRAY_Z = TOP + TRAY_WALL_H + 0.012  # plug centre at most this high: lying on the tray rim (32 mm)


class _Episode:
    """History for the current trial (reset with every embodiment reset, see _hook_reset)."""

    def __init__(self):
        self.ref = None  # cord segment centres at the first step
        self.plug_ref = None
        self.cord_pull = False
        self.dangle = False
        self.dropped = False
        self.last = None
        self.max_zone_move = 0.0
        self.max_tilt = 0.0


_EP = _Episode()


def reset_episode() -> None:
    global _EP
    _EP = _Episode()


def _hook_reset() -> None:
    """Clear the trial history whenever the yam_arena embodiment resets (score() only sees object states)."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (no embodiment: offline use)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_unplug_hooked", False):
        orig = cls.reset._orig
    else:
        orig = cls.reset

    def reset(self, *a, **k):
        reset_episode()
        return orig(self, *a, **k)

    reset._unplug_hooked = True
    reset._orig = orig
    cls.reset = reset


_hook_reset()


def plug_in_socket(plug: np.ndarray) -> bool:
    """Some of the plug is still inside the cavity (its tip behind the socket face)."""
    near = abs(plug[0] - SX) < 0.02 and plug[2] < FLOOR_Z + CAV_H + 0.01
    return bool(near and plug[1] - PL / 2 < MOUTH_Y - 0.002)


def plug_out(plug: np.ndarray) -> bool:
    """The plug tip is at least 5 mm clear of the socket face (or the plug is away from the socket)."""
    near = abs(plug[0] - SX) < 0.03 and plug[2] < SOCK_TOP + 0.01
    return bool((not near) or plug[1] - PL / 2 > MOUTH_Y + 0.005)


def plug_in_tray(plug: np.ndarray) -> bool:
    x, y = plug[:2] - TRAY_C
    return bool(abs(x) < TRAY_IN[0] / 2 and abs(y) < TRAY_IN[1] / 2 and plug[2] < IN_TRAY_Z)


def plug_on_tray(plug: np.ndarray) -> bool:
    """Plug centre over the tray's outer footprint (+1 cm) and low: lying on the rim / partly in the tray."""
    x, y = plug[:2] - TRAY_C
    return bool(abs(x) < TRAY_IN[0] / 2 + TRAY_T + 0.01 and abs(y) < TRAY_IN[1] / 2 + TRAY_T + 0.01
                and plug[2] < ON_TRAY_Z)


def _tilt_deg(c) -> float:
    d = c.end_plug - c.points[-1]
    return math.degrees(math.atan2(abs(d[2]), float(np.linalg.norm(d[:2])) + 1e-9))


def _update(objects) -> None:
    if objects is _EP.last:
        return
    _EP.last = objects
    c = objects["cord"]
    plug = c.end_plug
    if not (np.isfinite(c.points).all() and np.isfinite(plug).all()):  # simulation blew up (violent handling)
        _EP.cord_pull = True
        return
    if _EP.ref is None:
        _EP.ref, _EP.plug_ref = c.points.copy(), plug.copy()
        return
    from_plug = CORD_LEN - c.arclength()
    zone = (from_plug >= ZONE[0]) & (from_plug <= ZONE[1])
    if plug_in_socket(plug):
        move = float(np.linalg.norm(c.points[zone] - _EP.ref[zone], axis=1).max())
        rel = move - float(np.linalg.norm(plug - _EP.plug_ref))
        _EP.max_zone_move = max(_EP.max_zone_move, rel)
        if rel > CORD_MOVE_TOL:
            _EP.cord_pull = True
    elif plug_out(plug) and not (plug_in_tray(plug) or plug_on_tray(plug)):
        t = _tilt_deg(c)
        _EP.max_tilt = max(_EP.max_tilt, t)
        if t > DANGLE_DEG:
            _EP.dangle = True
        if plug[2] < DROP_Z:
            _EP.dropped = True


def mishandled() -> bool:
    return _EP.cord_pull or _EP.dangle or _EP.dropped


def success(objects) -> bool:
    _update(objects)
    c = objects["cord"]
    return bool(not mishandled() and plug_in_tray(c.end_plug) and c.resting(0.02, 1.0))


def score(objects) -> float:
    """0 if the cord was pulled/lifted while the plug was in the socket, or the plug dangled from the cord or fell
    onto the table.
    Otherwise 1.0 for the plug lying in the tray, 0.5 for the plug lying on the tray but not inside its walls
    (e.g. across the rim), else 0 (a plug still in the socket, or held in the air, scores nothing)."""
    _update(objects)
    if mishandled():
        return 0.0
    plug = objects["cord"].end_plug
    if plug_in_tray(plug):
        return 1.0
    return 0.5 if plug_on_tray(plug) else 0.0


def distance(objects) -> float:
    """Plug centre to the tray centre (m)."""
    p = objects["cord"].end_plug
    return float(np.linalg.norm(p - np.array([TRAY_C[0], TRAY_C[1], TOP + TRAY_FLOOR + PH / 2])))


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------


def _log(ctx, tag=""):
    st = ctx.states()
    c = st["cord"]
    print(f"[t={ctx.t:5.2f}] {tag} plug={np.round(c.end_plug, 4)} tilt={_tilt_deg(c):.1f} "
          f"in_sock={plug_in_socket(c.end_plug)} out={plug_out(c.end_plug)} tray={plug_in_tray(c.end_plug)} "
          f"zmove={_EP.max_zone_move:.4f} maxtilt={_EP.max_tilt:.1f} pull={_EP.cord_pull} dangle={_EP.dangle} "
          f"drop={_EP.dropped} "
          f"cordz_min={c.points[:, 2].min() - TOP:.4f}", flush=True)


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    mode = os.environ.get("UNPLUG_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    yaw = math.pi / 2  # fingers close along x, across the plug's width
    hi = TOP + 0.09
    _log(ctx, "start")
    if mode == "nothing":  # cheat: do nothing
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    if mode == "probe":
        for _ in range(8):
            yield from L.wait(0.5)
            _log(ctx)
        return
    if mode.startswith("yank"):
        # cheat: grab the cord ~8 cm behind the plug and pull it out by the cord
        c = ctx.states()["cord"]
        k = int(np.argmin(np.abs((CORD_LEN - c.arclength()) - float(os.environ.get("UNPLUG_YANK_AT", "0.12")))))
        p = c.points[k]
        yield from Rt.move((p[0], p[1], hi), yaw=yaw, finger=FINGER_OPEN, duration=1.5)
        yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=yaw, duration=1.0)
        yield from Rt.grip(FINGER_CLOSED)
        _log(ctx, "cord gripped")
        lift = 0.04 if mode == "yank_lift" else 0.0
        yield from Rt.move((p[0], p[1] + 0.06, p[2] + 0.010 + lift), yaw=yaw, duration=1.5)
        _log(ctx, "pulled")
        yield from Rt.move((TRAY_C[0], TRAY_C[1], TOP + 0.08), yaw=yaw, duration=1.5)
        yield from Rt.move((TRAY_C[0], TRAY_C[1], TOP + 0.04), yaw=yaw, duration=1.0)
        yield from Rt.grip(FINGER_OPEN)
        _log(ctx, "released")
        yield from Rt.move((TRAY_C[0], TRAY_C[1], hi), yaw=yaw, duration=1.0)
        _log(ctx, "end")
        return

    # Correct: grip the protruding plug body from above, pull it straight out along the socket axis, carry it
    # level to the tray and lay it down.
    p = ctx.states()["cord"].end_plug
    gy = MOUTH_Y + (PL - INSERT) / 2 + 0.004  # middle of the exposed part of the plug
    g = np.array([p[0], gy, p[2] + 0.004])
    yield from Rt.move((g[0], g[1], hi), yaw=yaw, finger=FINGER_OPEN, duration=1.5)
    yield from Rt.move(g, yaw=yaw, duration=1.2)
    yield from Rt.grip(FINGER_CLOSED, 0.8)
    _log(ctx, "plug gripped")
    out = g + np.array([0.0, PL + 0.02, 0.0])
    yield from Rt.move(out, yaw=yaw, duration=2.0)
    _log(ctx, "withdrawn")
    yield from Rt.move((out[0], out[1], TOP + 0.07), yaw=yaw, duration=0.8)
    # closed loop: where the plug sits relative to the TCP, so the plug centre lands on the tray centre
    off = ctx.states()["cord"].end_plug - Rt.tcp()
    place = np.array([TRAY_C[0] - off[0], TRAY_C[1] - off[1], TOP + 0.07])
    yield from Rt.move(place, yaw=yaw, duration=1.5)
    if mode == "hold":  # cheat: unplug correctly but keep holding the plug 7 cm above the tray
        yield from Rt.wait(3.0)
        _log(ctx, "holding")
        return
    place[2] = TOP + TRAY_FLOOR + PH / 2 + 0.003 - off[2]
    yield from Rt.move(place, yaw=yaw, duration=1.0)
    yield from Rt.grip(FINGER_OPEN, 0.6)
    _log(ctx, "released")
    yield from Rt.move((place[0], place[1], hi), yaw=yaw, duration=1.0)
    _log(ctx, "end")
