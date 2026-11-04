"""On the station table: a table lamp stands at the left, in front of the left arm (fixed base, 80 mm diameter, 20 mm
tall, with a short pole and a cream shade). Its black 6 mm power cord (52 cm) leaves the base on its right-hand side
and lies straight across the open "walkway" in front of the robot towards the right arm; a dark 50 x 16 x 12 mm plug
is on the free end. A soft rose rug (120 x 220 x 4 mm, 50 g, free) lies in the walkway just in front of the cord.
Along the back of the table runs a grey cord raceway: two fixed rails 4 mm wide and 10 mm tall with a 14 mm slot
between them, 32 cm long. Beyond its right end a white wall-socket block (fixed) has its 24 x 16 mm, 26 mm deep
socket opening facing the raceway. The cord must be tidied by pressing it into the raceway and plugging the plug
into the socket. Tucking the cord under the rug (a fire hazard), leaving it across the walkway, or moving the rug
scores 0.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var RUGCORD_DEMO): correct (default) | nothing | plug_only | hold.
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
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Tidy the lamp cord: press it into the grey raceway along the back of the table and push its plug "
               "into the socket at the raceway's right end, then let go. Never put the cord under the rug, leave it "
               "across the walkway in front, or move the rug.")

GREY = (0.66, 0.67, 0.69)
OFFWHITE = (0.93, 0.92, 0.88)
CORD_BLACK = (0.07, 0.07, 0.07)
ROSE = (0.86, 0.62, 0.62)

# -- raceway (two fixed rails along y) -----------------------------------------------------------------------------
XR = 0.36  # raceway centreline (x)
GAP, RAIL_T, RAIL_H = 0.014, 0.004, 0.010
RW_Y0, RW_Y1 = 0.195, -0.125  # raceway start (lamp end) and end (socket end), y
RW_LEN = RW_Y0 - RW_Y1  # 0.28
N_SECTIONS = 8  # 4 cm each


def _raceway() -> list[Box]:
    yc = (RW_Y0 + RW_Y1) / 2
    off = GAP / 2 + RAIL_T / 2
    return [Box(f"raceway_{n}", pos=(XR + s * off, yc, TOP + RAIL_H / 2), size=(RAIL_T, RW_LEN, RAIL_H), color=GREY)
            for n, s in (("front_rail", -1), ("back_rail", 1))]


# -- wall socket block (fixed): socket opening faces +y (towards the raceway), at table level ---------------------
FACE_Y = -0.19  # socket face
CAV_W, CAV_H, CAV_D = 0.024, 0.016, 0.026  # opening width (x), height (z), depth (along -y)
SOCK_W, SOCK_D, SOCK_H = 0.050, 0.050, 0.030  # block footprint (x, y) and height


def _socket() -> list[Box]:
    side = (SOCK_W - CAV_W) / 2
    yc = FACE_Y - CAV_D / 2
    return [
        Box("socket_back", pos=(XR, FACE_Y - CAV_D - (SOCK_D - CAV_D) / 2, TOP + SOCK_H / 2),
            size=(SOCK_W, SOCK_D - CAV_D, SOCK_H), color=OFFWHITE),
        Box("socket_roof", pos=(XR, yc, TOP + (CAV_H + SOCK_H) / 2), size=(SOCK_W, CAV_D, SOCK_H - CAV_H),
            color=OFFWHITE),
        Box("socket_wall_f", pos=(XR - CAV_W / 2 - side / 2, yc, TOP + CAV_H / 2), size=(side, CAV_D, CAV_H),
            color=OFFWHITE),
        Box("socket_wall_b", pos=(XR + CAV_W / 2 + side / 2, yc, TOP + CAV_H / 2), size=(side, CAV_D, CAV_H),
            color=OFFWHITE),
    ]


# -- table lamp (fixed) and its cord ---------------------------------------------------------------------------------
LAMP_XY = (0.27, 0.32)
BASE_R, BASE_H = 0.04, 0.02
LAMP = [
    Cylinder("lamp_base", pos=(LAMP_XY[0], LAMP_XY[1], TOP + BASE_H / 2), radius=BASE_R, height=BASE_H,
             color=(0.20, 0.21, 0.23)),
    Cylinder("lamp_pole", pos=(LAMP_XY[0], LAMP_XY[1], TOP + BASE_H + 0.045), radius=0.006, height=0.09,
             color=(0.70, 0.66, 0.55)),
    Cylinder("lamp_shade", pos=(LAMP_XY[0], LAMP_XY[1], TOP + BASE_H + 0.09 + 0.02), radius=0.04, height=0.04,
             color=(0.96, 0.92, 0.80)),
]
R = 0.003  # 6 mm cord
CORD_LEN = 0.52
CORD_YAW = math.radians(-97)
PL, PW, PH = 0.050, 0.016, 0.012
PLUG = Plug(size=(PL, PW, PH), mass=0.014, color=(0.22, 0.22, 0.24))
CORD_START = (LAMP_XY[0], LAMP_XY[1] - BASE_R - 0.006, TOP + R + 0.0005)  # 6 mm off the base's -y face
CORD = cable("cord", start=CORD_START, yaw=CORD_YAW, length=CORD_LEN, radius=R, anchor_start=True, end_plug=PLUG,
             color=CORD_BLACK, bend_stiffness=0.003, bend_damping=0.0005)

# -- rug (free) and walkway -----------------------------------------------------------------------------------------
RUG_XY = np.array([0.135, 0.0])
RUG_SIZE = (0.12, 0.22, 0.004)
RUG = Box("rug", pos=(float(RUG_XY[0]), float(RUG_XY[1]), TOP + RUG_SIZE[2] / 2), size=RUG_SIZE, mass=0.05,
          color=ROSE)
WALK_X = (0.05, 0.31)  # walkway: the open floor between the lamp and the socket, in front of the raceway
WALK_Y = (-0.11, 0.11)

OBJECTS = [*LAMP, CORD, RUG, *_raceway(), *_socket()]

# -- scoring ---------------------------------------------------------------------------------------------------------
SEATED_Z = TOP + RAIL_H - R + 0.002  # cord centre at most this high: its top no more than 2 mm above the rails
SEATED_MIN_SECTIONS = 2
PLUG_MIN = 0.015  # plug tip at least 15 mm inside the socket
RUG_MOVE = 0.03
RUG_TURN_DEG = 10.0
RELEASE_DIST = 0.04
SAMPLES = np.linspace(RW_Y0 - 0.004, RW_Y1 + 0.004, 8 * N_SECTIONS)  # y of the check points along the raceway
_ENV: dict = {"env": None}


def _install_hooks() -> None:
    """Keep a handle on the env so score() can see where the grippers are."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (no embodiment: offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_rugcord_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._rugcord_hooked = True
        cls._task_step = _task_step


_install_hooks()


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def cord_points(objects) -> np.ndarray:
    """Dense points along the cord (tips included) plus the plug centre."""
    c = objects[CORD.name]
    pts = _dense(c.polyline())
    return np.vstack([pts, c.end_plug[None]]) if c.end_plug is not None else pts


def seated_points(objects) -> np.ndarray:
    pts = _dense(objects[CORD.name].polyline())
    ok = np.zeros(len(SAMPLES), dtype=bool)
    for i, y in enumerate(SAMPLES):
        d = np.linalg.norm(pts[:, :2] - np.array([XR, y]), axis=1)
        near = d < GAP / 2
        ok[i] = bool(near.any() and pts[near, 2].min() <= SEATED_Z)
    return ok


def seated_sections(objects) -> np.ndarray:
    """Per 4 cm raceway section: True if the cord lies in the slot, below the rail tops, all along it."""
    return seated_points(objects).reshape(N_SECTIONS, -1).all(axis=1)


def plug_depth(objects) -> float:
    """How far (m) the plug tip is inside the socket; -1 if the plug isn't lined up with the opening."""
    c = objects[CORD.name]
    p = c.end_plug
    if p is None or not np.isfinite(p).all():
        return -1.0
    a = p - c.points[-1]
    a = a / (np.linalg.norm(a) + 1e-12)
    if abs(p[0] - XR) > 0.006 or p[2] > TOP + CAV_H + 0.004 or a[1] > -0.9:
        return -1.0
    return float(FACE_Y - (p[1] - PL / 2))


def _rug_yaw_deg(rug) -> float:
    x, y, z, w = rug.quat_xyzw
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def rug_moved(objects) -> bool:
    rug = objects[RUG.name]
    return bool(np.linalg.norm(rug.pos[:2] - RUG_XY) > RUG_MOVE or abs(_rug_yaw_deg(rug)) > RUG_TURN_DEG
                or rug.pos[2] > TOP + RUG_SIZE[2] / 2 + 0.01)


def cord_under_rug(objects) -> bool:
    rug = objects[RUG.name]
    th = math.radians(_rug_yaw_deg(rug))
    pts = cord_points(objects)
    d = pts[:, :2] - rug.pos[:2]
    lx = d[:, 0] * math.cos(th) + d[:, 1] * math.sin(th)
    ly = -d[:, 0] * math.sin(th) + d[:, 1] * math.cos(th)
    inside = (np.abs(lx) <= RUG_SIZE[0] / 2) & (np.abs(ly) <= RUG_SIZE[1] / 2)
    return bool((inside & (pts[:, 2] < rug.high[2])).any())


def cord_in_walkway(objects) -> bool:
    pts = cord_points(objects)
    return bool(((pts[:, 0] >= WALK_X[0]) & (pts[:, 0] <= WALK_X[1]) & (pts[:, 1] >= WALK_Y[0])
                 & (pts[:, 1] <= WALK_Y[1])).any())


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP; [] if unknown (offline)."""
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


def released(objects) -> bool:
    pts = cord_points(objects)
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def gates(objects) -> list[str]:
    """Reasons for a zero score (empty list = none)."""
    out = []
    if not np.isfinite(cord_points(objects)).all():
        return ["nan"]
    if cord_under_rug(objects):
        out.append("under_rug")
    if rug_moved(objects):
        out.append("rug_moved")
    if cord_in_walkway(objects):
        out.append("walkway")
    if not released(objects):
        out.append("held")
    if seated_sections(objects).sum() < SEATED_MIN_SECTIONS:
        out.append("not_routed")
    return out


def score(objects) -> float:
    """0 if any cord point lies under the rug, the rug moved > 3 cm / turned > 10 deg / was lifted, any cord point
    or the plug is over the walkway, a gripper is within 4 cm of the cord or plug, or fewer than 2 of the 7 raceway
    sections hold the cord. Otherwise 0.5 * (seated sections / 8) + 0.5 if the plug tip is >= 15 mm in the socket."""
    if gates(objects):
        return 0.0
    k = int(seated_sections(objects).sum())
    return 0.5 * k / N_SECTIONS + (0.5 if plug_depth(objects) >= PLUG_MIN else 0.0)


def success(objects) -> bool:
    return bool(score(objects) == 1.0 and objects[CORD.name].resting(0.02, 1.0))


def distance(objects) -> float:
    """Unseated raceway length plus missing plug depth (m)."""
    k = int(seated_sections(objects).sum())
    return float((N_SECTIONS - k) / N_SECTIONS * RW_LEN + max(0.0, PLUG_MIN - max(plug_depth(objects), 0.0)))


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
FINGER_PINCH = -0.020  # plug pinch opening (pads ~44 mm apart)
GRASP_OPEN = -0.014  # bare-cord grasp opening (pads ~28 mm apart)
GRASP_BACK = 0.010  # pinch the plug this far from its cord end
SEAT_TO = 0.0225  # target plug tip depth (cavity 26 mm)
PRE = 0.008  # staged plug tip this far before the socket face
PAD_TIP = 0.013
HI = TOP + 0.08


def _log(ctx, tag=""):
    st = ctx.states()
    print(f"[t={ctx.t:5.2f}] {tag}: seated={seated_sections(st).astype(int).tolist()} "
          f"depth={plug_depth(st)*1000:.1f}mm gates={gates(st)} score={score(st):.3f} "
          f"rug={np.round(st[RUG.name].pos, 3).tolist()} plug={np.round(st[CORD.name].end_plug, 3).tolist()}",
          flush=True)
    if os.environ.get("RUGCORD_DBG"):
        pts = st[CORD.name].points
        print("   cord(mm x,y,z-top):", " ".join(f"{p[0]*1000:.0f},{p[1]*1000:.0f},{(p[2]-TOP)*1000:.0f}" for p in pts),
              f"| L={np.round(ctx.left.tcp(), 3).tolist()} R={np.round(ctx.right.tcp(), 3).tolist()} "
              f"grip={[np.round(g, 3).tolist() for g in grippers()]}", flush=True)


def _cord_at(objects, a: float) -> tuple[np.ndarray, float]:
    """Live cord point at arclength ``a`` from the lamp, and the cord heading there."""
    poly = objects[CORD.name].polyline()
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    p = np.array([np.interp(a, s, poly[:, k]) for k in range(3)])
    i = min(max(int(np.searchsorted(s, a)), 1), len(poly) - 1)
    d = poly[i] - poly[i - 1]
    return p, math.atan2(d[1], d[0])


def _yaw_near(heading: float, ref: float) -> float:
    return heading + math.pi * round((ref - heading) / math.pi)


def _plug_pose(objects) -> tuple[np.ndarray, np.ndarray]:
    c = objects[CORD.name]
    p = np.asarray(c.end_plug, dtype=float)
    a = p - c.points[-1]
    a[2] = 0.0
    return p, a / (np.linalg.norm(a) + 1e-12)


def _pick_plug(Rt, st):
    from yam_arena.constants import FINGER_CLOSED

    p, a = _plug_pose(st)
    yaw = _yaw_near(math.atan2(a[1], a[0]), math.pi / 2)
    g = p - a * (PL / 2 - GRASP_BACK)
    gz = TOP + PH + 0.003
    yield from Rt.move((g[0], g[1], HI), yaw=yaw, finger=FINGER_PINCH, duration=1.5)
    yield from Rt.move((g[0], g[1], gz), yaw=yaw, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED, 0.6)
    yield from Rt.move((g[0], g[1], gz + 0.03), yaw=yaw, duration=0.6)


STAGE_GY = FACE_Y + PRE + PL - GRASP_BACK  # TCP y with the plug tip PRE before the face


def _stage_plug(Rt, z=0.03, duration=2.5):
    yield from Rt.move((XR, STAGE_GY, TOP + PH + 0.003 + z), yaw=math.pi / 2, duration=duration)


def _insert(Rt, ctx, release=True, x0=XR):
    """Lower the staged plug to the table, line it up with the opening (closed loop) and push it in. Returns the
    final TCP (x, y)."""
    gz = TOP + PH + 0.003
    yaw = math.pi / 2
    dx = x0 - XR
    yield from Rt.move((XR + dx, STAGE_GY, gz), yaw=yaw, duration=0.8)
    yield from Rt.wait(0.3)
    for _ in range(4):
        p, _ = _plug_pose(ctx.states())
        err = XR - p[0]
        if abs(err) < 0.001:
            break
        dx = float(np.clip(dx + err, -0.03, 0.03))
        yield from Rt.move((XR + dx, STAGE_GY, gz), yaw=yaw, duration=0.5)
        yield from Rt.wait(0.2)
    _log(ctx, "aligned")
    gy = STAGE_GY - (PRE + SEAT_TO)
    yield from Rt.move((XR + dx, gy, gz), yaw=yaw, duration=2.0)
    for _ in range(2):  # top up if the plug slipped back in the fingers
        d = plug_depth(ctx.states())
        if d >= SEAT_TO - 0.002:
            break
        gy -= SEAT_TO - max(d, 0.0)
        yield from Rt.move((XR + dx, gy, gz), yaw=yaw, duration=1.0)
    yield from Rt.wait(0.3)
    if release:
        yield from _release_plug(Rt, XR + dx, gy)
    return XR + dx, gy


def _release_plug(Rt, x, gy):
    yield from Rt.grip(FINGER_PINCH, 0.6)
    yield from Rt.move((x, gy + 0.003, HI), yaw=math.pi / 2, duration=1.0)


def _press(arm, y, yaw, x=XR):
    from yam_arena.constants import FINGER_CLOSED

    pz = TOP + 0.007 + PAD_TIP  # closed fingertips 7 mm above the table: 1 mm into a seated cord, 3 mm below the lip
    yield from arm.move((x, y, TOP + 0.035), yaw=yaw, finger=FINGER_CLOSED, duration=0.5)
    yield from arm.move((x, y, pz), yaw=yaw, duration=0.4)
    yield from arm.move((x, y, TOP + 0.035), yaw=yaw, duration=0.3)


def _press_run(arm, ys, yaw, ctx=None):
    """Press at each y; with ``ctx``, over the cord where it actually lies (within 3 mm of the slot centre)."""
    for y in ys:
        x = XR
        if ctx is not None:
            pts = _dense(ctx.states()[CORD.name].polyline())
            m = (np.abs(pts[:, 1] - y) < 0.004) & (np.abs(pts[:, 0] - XR) < 0.015)
            if m.any():
                x = float(np.clip(pts[m, 0].mean(), XR - 0.003, XR + 0.003))
        yield from _press(arm, float(y), yaw, x)


def _park(arm, side):
    yield from arm.move((0.24, side * 0.12, TOP + 0.10), yaw=-side * math.pi / 2, duration=0.8)


def _retreat(arm, side):
    yield from arm.move((0.22, side * 0.24, TOP + 0.14), yaw=0.0, duration=1.2)


LEFT_HOLD_Y = RW_Y0 + 0.005  # where the left arm holds the cord above the slot (TCP y)
HOLD_Z = TOP + RAIL_H + R + 0.003 + 0.010  # TCP z while holding the cord: its centre 3 mm above the rail tops
RIGHT_HOLD_Y = -0.105  # where the right arm holds the cord above the slot's socket end
SEATED_PLUG_BACK_Y = FACE_Y - SEAT_TO + PL  # y of the plug's cord end once seated
# cord arclengths the arms take: right so the rest just reaches the seated plug with ~1 cm slack, left so the cord
# between the two hands just spans the slot (3 mm slack)
S_RIGHT = CORD_LEN - (RIGHT_HOLD_Y - SEATED_PLUG_BACK_Y) - 0.005
S_LEFT = S_RIGHT - (LEFT_HOLD_Y - RIGHT_HOLD_Y) - 0.003


def demo_script(ctx):
    """Left arm takes the cord S_LEFT from the lamp while the right arm pinches the plug; together they hold the cord
    up straight over the raceway (left over its start, right with the plug staged just before the socket; closed loop
    on x), lower it so it settles into the slot, the left arm lets go and the right arm pushes the plug into the
    socket and lets go. Then both arms press the cord down along their half of the raceway, a closed-loop touch-up
    presses any check point still unseated (one arm at a time) and both retreat.
    RUGCORD_DEMO=nothing|plug_only|hold runs a shortcut instead (each must score 0)."""
    from yam_arena.constants import FINGER_CLOSED

    mode = os.environ.get("RUGCORD_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "nothing":
        for _ in range(5):
            yield from L.wait(1.0)
        _log(ctx, "end")
        return
    if mode == "plug_only":  # shortcut: plug in, cord left across the walkway
        yield from ctx.both(_pick_plug(Rt, ctx.states()), L.wait(0.5))
        yield from _stage_plug(Rt)
        yield from _insert(Rt, ctx)
        yield from _retreat(Rt, -1)
        yield from L.wait(1.0)
        _log(ctx, "end")
        return

    # 1) right arm: plug into the socket (the cord drags along the table behind it)
    yield from ctx.both(_pick_plug(Rt, ctx.states()), L.move((0.24, 0.10, TOP + 0.16), yaw=0.0, duration=2.0))
    yield from _stage_plug(Rt)
    yield from _insert(Rt, ctx)
    _log(ctx, "plugged")
    # 2) each arm takes the cord at the arclength that belongs at its end of the raceway (left near the lamp end,
    #    right near the socket end) and holds it up over the slot; closed loop on the held points' x
    hyaw = -math.pi / 2  # left hold/press yaw: fingers across the slot
    ryaw = math.pi / 2

    def pick(arm, s, ref):
        for attempt in range(3):  # closed loop: re-grasp if the cord point did not come up with the fingers
            p, h = _cord_at(ctx.states(), s)
            y = _yaw_near(h, ref)
            yield from arm.move((p[0], p[1], HI), yaw=y, finger=GRASP_OPEN, duration=1.2)
            p, _ = _cord_at(ctx.states(), s)
            yield from arm.move((p[0], p[1], p[2] + 0.009), yaw=y, duration=0.9)
            yield from arm.grip(FINGER_CLOSED, 0.6)
            yield from arm.move((p[0], p[1], TOP + 0.03), yaw=y, duration=0.6)
            q, _ = _cord_at(ctx.states(), s)
            print(f"[demo] {arm.name} grasp attempt {attempt} at s={s:.3f}: cord point z={(q[2] - TOP) * 1000:.0f} mm",
                  flush=True)
            if q[2] > TOP + 0.015:
                break
            yield from arm.grip(GRASP_OPEN, 0.4)

    # the right arm's point lies near the raceway end: take it a little further back where the open fingers clear the
    # rails, and hold it correspondingly further from the socket
    s_r = S_RIGHT
    for d in np.arange(0.0, 0.08, 0.005):
        q, _ = _cord_at(ctx.states(), S_RIGHT - d)
        if q[0] < XR - GAP / 2 - RAIL_T - 0.016 or q[1] < RW_Y1 - 0.02:
            s_r = S_RIGHT - d
            break
    ry_hold = RIGHT_HOLD_Y + (S_RIGHT - s_r)
    yield from ctx.both(pick(L, S_LEFT, hyaw), pick(Rt, s_r, ryaw))
    _log(ctx, "picked")
    yield from ctx.both(L.move((XR, LEFT_HOLD_Y, TOP + 0.03), yaw=hyaw, duration=2.0),
                        Rt.move((XR, ry_hold, TOP + 0.03), yaw=ryaw, duration=2.0))
    yield from L.wait(0.4)
    gl, gr, hy = XR, XR, LEFT_HOLD_Y
    for _ in range(8):  # closed loop: held points over the slot centre, cord between the hands drawn straight
        st = ctx.states()
        e_l = float(_cord_at(st, S_LEFT)[0][0] - XR)
        e_r = float(_cord_at(st, s_r)[0][0] - XR)
        mid = np.array([_cord_at(st, a)[0] for a in np.linspace(S_LEFT + 0.02, s_r - 0.02, 12)])
        dev = float(np.abs(mid[:, 0] - XR).max())
        print(f"[demo] align: held cord x err left {e_l*1000:+.1f} mm, right {e_r*1000:+.1f} mm, "
              f"between {dev*1000:.1f} mm (left hold y {hy:.3f})", flush=True)
        if abs(e_l) < 0.002 and abs(e_r) < 0.002 and dev < 0.003:
            break
        gl = float(np.clip(gl - e_l, XR - 0.025, XR + 0.025))
        gr = float(np.clip(gr - e_r, XR - 0.025, XR + 0.025))
        if dev >= 0.003 and hy < LEFT_HOLD_Y + 0.04:
            hy += min(0.008, max(0.003, dev / 3))
        yield from ctx.both(L.move((gl, hy, TOP + 0.03), yaw=hyaw, duration=0.5),
                            Rt.move((gr, ry_hold, TOP + 0.03), yaw=ryaw, duration=0.5))
        yield from L.wait(0.3)
    _log(ctx, "held up")
    # 3) lower both so the cord between them settles into the slot, then let go
    yield from ctx.both(L.move((gl, hy, HOLD_Z), yaw=hyaw, duration=1.0),
                        Rt.move((gr, ry_hold, HOLD_Z), yaw=ryaw, duration=1.0))
    yield from L.wait(0.3)
    _log(ctx, "lowered")
    yield from ctx.both(L.grip(-0.008, 0.4), Rt.grip(-0.008, 0.4))
    yield from ctx.both(L.move((gl, hy, TOP + 0.035), yaw=hyaw, duration=0.4),
                        Rt.move((gr, ry_hold, TOP + 0.035), yaw=ryaw, duration=0.4))
    _log(ctx, "released")
    # 4) both press their half of the raceway every 2 cm
    # both run towards +y, ~13 cm apart, so the arms never meet at the midline
    left_ys = np.r_[np.arange(0.006, RW_Y0 - 0.002, 0.02), RW_Y0 + 0.008]  # last: the cord's entry, just outside
    right_ys = np.arange(RW_Y1 + 0.008, 0.0, 0.02)
    if mode == "hold":
        # shortcut: done, but the left arm keeps the cord pressed in the raceway at the end
        yield from ctx.both(_press_run(L, left_ys, -math.pi / 2), _press_run(Rt, right_ys, math.pi / 2))
        yield from L.move((XR, 0.06, TOP + 0.007 + PAD_TIP), yaw=-math.pi / 2, duration=0.6)
        yield from ctx.both(L.wait(2.0), _retreat(Rt, -1))
        _log(ctx, "end")
        return
    yield from ctx.both(_press_run(L, left_ys, -math.pi / 2), _press_run(Rt, right_ys, math.pi / 2))
    _log(ctx, "pressed")
    # 5) closed-loop touch-up on unseated check points (left for y > 0, right otherwise)
    for _ in range(6):
        bad = [y for y, ok in zip(SAMPLES, seated_points(ctx.states())) if not ok]
        if not bad:
            break
        lb = [y for y in bad if y > 0.0]
        rb = [y for y in bad if y <= 0.0]
        # one arm at a time (the other parked at its side), so they never meet at the midline
        if lb:
            yield from ctx.both(_press_run(L, lb[:3], -math.pi / 2, ctx), _park(Rt, -1))
        if rb:
            yield from ctx.both(_press_run(Rt, rb[:3], math.pi / 2, ctx), _park(L, 1))
    yield from ctx.both(_retreat(L, 1), _retreat(Rt, -1))
    yield from L.wait(1.0)
    _log(ctx, "end")
