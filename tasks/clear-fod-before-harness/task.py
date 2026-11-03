"""On the station table: a white harness layout sheet (1 mm) lies in front of the robot with a dark-blue route line
drawn on it, running left to right (along -y) at x 32 cm, and a dark plug outline at the line's right end. A grey
connector block at the line's left end holds a black 6 mm cord (40 cm, black plug on its free end); the cord does
not lie on the route yet: it leaves the block angled 18 degrees towards the robot. Two loose steel discs (washer
stand-ins, 30 mm diameter, 6 mm thick) lie ON the route line (foreign object debris, FOD). A red open FOD bin stands
in front of the board on the midline. The discs must go into the bin first; then the cord is laid along the line
with its plug in the outline. The cord must never be laid over or pushed against a disc that is still on the board.

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
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("The two steel discs on the harness board are loose debris. First put both discs in the red bin, "
               "then lay the black cable along the blue line with its plug inside the plug outline at the end of "
               "the line. Never lay the cable over or against a disc that is still on the board.")

# -- harness board: 1 mm layout sheet with a route line and a plug outline -----------------------------------------
SHEET_T = 0.001
BT = TOP + SHEET_T  # board top
SHEET_X, SHEET_Y = (0.25, 0.39), (-0.23, 0.31)
WHITE = (0.95, 0.95, 0.93)
BLUE = (0.10, 0.20, 0.60)
INK_T = 0.0003  # printed line / outline thickness
RX = 0.32  # route line x
R = 0.003  # 6 mm cord
CORD_LEN = 0.40
ROUTE_Y0 = 0.245  # cord exit from the connector block (start of the route line)
ROUTE_Y1 = ROUTE_Y0 - CORD_LEN  # route end (cord end, where the plug starts)
PLUG = Plug(size=(0.035, 0.016, 0.012), mass=0.012, color=(0.06, 0.06, 0.06))
PLUG_GOAL = np.array([RX, ROUTE_Y1 - PLUG.size[0] / 2])
OUTLINE_IN = (0.036, 0.065)  # plug outline inside, x by y
OUTLINE_W = 0.003
LINE_W = 0.006

SHEET = Box("layout_sheet", pos=((SHEET_X[0] + SHEET_X[1]) / 2, (SHEET_Y[0] + SHEET_Y[1]) / 2, TOP + SHEET_T / 2),
            size=(SHEET_X[1] - SHEET_X[0], SHEET_Y[1] - SHEET_Y[0], SHEET_T), color=WHITE)
_LINE_Y0, _LINE_Y1 = ROUTE_Y0 + 0.004, ROUTE_Y1 + 0.004  # ink stops just short of the plug outline
LINE = Box("route_line", pos=(RX, (_LINE_Y0 + _LINE_Y1) / 2, BT + INK_T / 2), size=(LINE_W, _LINE_Y0 - _LINE_Y1, INK_T),
           color=BLUE)


def _outline() -> list[Box]:
    (x, y), (ix, iy), w, z = PLUG_GOAL, OUTLINE_IN, OUTLINE_W, BT + INK_T / 2
    return [
        Box("outline_n", pos=(x + ix / 2 + w / 2, y, z), size=(w, iy + 2 * w, INK_T), color=BLUE),
        Box("outline_s", pos=(x - ix / 2 - w / 2, y, z), size=(w, iy + 2 * w, INK_T), color=BLUE),
        Box("outline_e", pos=(x, y - iy / 2 - w / 2, z), size=(ix, w, INK_T), color=BLUE),
        Box("outline_w", pos=(x, y + iy / 2 + w / 2, z), size=(ix, w, INK_T), color=BLUE),
    ]


# -- connector block (fixed) and cord ---------------------------------------------------------------------------------
BLOCK_SIZE = (0.04, 0.03, 0.025)
BLOCK = Box("connector_block", pos=(RX, ROUTE_Y0 + 0.006 + BLOCK_SIZE[1] / 2, TOP + BLOCK_SIZE[2] / 2),
            size=BLOCK_SIZE, color=(0.45, 0.47, 0.50))
CORD_YAW = math.radians(-108)  # starts 18 deg off the route (towards the robot)
CORD_START = (RX, ROUTE_Y0, BT + R + 0.0005)
CORD = cable("cord", start=CORD_START, yaw=CORD_YAW, length=CORD_LEN, radius=R, anchor_start=True,
             end_plug=PLUG, color=(0.05, 0.05, 0.05), bend_stiffness=0.01, bend_damping=0.001)

# -- FOD: two loose steel discs on the route line -----------------------------------------------------------------------
DISC_R, DISC_H, DISC_MASS = 0.015, 0.006, 0.03
DISC_XY = {"disc_a": np.array([RX, 0.08]), "disc_b": np.array([RX, -0.08])}
STEEL = (0.42, 0.44, 0.48)
DISCS = [Cylinder(n, pos=(float(p[0]), float(p[1]), BT + INK_T + DISC_H / 2 + 0.0005), radius=DISC_R, height=DISC_H,
                  mass=DISC_MASS, color=STEEL) for n, p in DISC_XY.items()]

# -- FOD bin (fixed): inner 80 x 80 mm, 30 mm walls ---------------------------------------------------------------------
BIN_C = np.array([0.15, 0.0])
BIN_IN = 0.08
BIN_T, BIN_H, BIN_FLOOR = 0.003, 0.03, 0.003
RED = (0.80, 0.12, 0.10)


def _bin() -> list[Box]:
    x, y = BIN_C
    o = BIN_IN + 2 * BIN_T
    zc = TOP + BIN_H / 2
    return [
        Box("bin_floor", pos=(x, y, TOP + BIN_FLOOR / 2), size=(o, o, BIN_FLOOR), color=RED),
        Box("bin_wall_n", pos=(x + BIN_IN / 2 + BIN_T / 2, y, zc), size=(BIN_T, o, BIN_H), color=RED),
        Box("bin_wall_s", pos=(x - BIN_IN / 2 - BIN_T / 2, y, zc), size=(BIN_T, o, BIN_H), color=RED),
        Box("bin_wall_e", pos=(x, y - BIN_IN / 2 - BIN_T / 2, zc), size=(BIN_IN, BIN_T, BIN_H), color=RED),
        Box("bin_wall_w", pos=(x, y + BIN_IN / 2 + BIN_T / 2, zc), size=(BIN_IN, BIN_T, BIN_H), color=RED),
    ]


OBJECTS = [SHEET, LINE, *_outline(), BLOCK, CORD, *DISCS, *_bin()]
DISC_NAMES = tuple(DISC_XY)

# -- scoring ---------------------------------------------------------------------------------------------------------
TOUCH_GAP = 0.002  # cable surface within 2 mm of a disc edge (top view) = laid over / against it
LOW_GAP = 0.010  # ... and its underside less than 1 cm above the disc top (held higher = carried over it, allowed)
DISC_DOWN_Z = BT + DISC_H / 2 + 0.010  # disc centre below this = resting on the board/table, not lifted
ROUTE_TOL = 0.015  # every cable point within 1.5 cm of the route line
LYING_Z = BT + 0.015  # every cable point at most 1.5 cm above the board
RELEASE_DIST = 0.03  # every gripper (centre between the pads) at least this far from the cable, plug and discs


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def disc_in_bin(d) -> bool:
    """Disc centre over the bin's inner floor and below the wall tops."""
    p = d.pos
    return bool(np.all(np.abs(p[:2] - BIN_C) <= BIN_IN / 2) and p[2] <= TOP + BIN_H)


def cable_disc_gap(objects, name: str) -> float:
    """Top-view gap (m) between the disc edge and the parts of the cable/plug that are less than 1 cm above the disc
    top (negative = overlapping); inf if none is that low."""
    c = objects["cord"]
    d = objects[name].pos
    p, top = d[:2], d[2] + DISC_H / 2
    pts = _dense(c.polyline())
    low = pts[pts[:, 2] - R <= top + LOW_GAP]  # cable underside < 1 cm above the disc top: on / against it
    if not len(low):
        gap = float("inf")
    else:
        gap = float((np.linalg.norm(low[:, :2] - p, axis=1) - DISC_R - R).min())
    if c.end_plug is not None and c.end_plug[2] - PLUG.size[2] / 2 <= top + LOW_GAP:
        gap = min(gap, float(np.linalg.norm(c.end_plug[:2] - p) - DISC_R - PLUG.size[1] / 2))
    return gap


# Episode memory: the cable must never touch / lie over a disc resting on the board. Reset at every episode start
# (wrapped embodiment reset; fallback: the untouched starting layout also resets it).
_STATE = {"violated": False, "min_gap": float("inf")}


def _reset_episode() -> None:
    _STATE.update(violated=False, min_gap=float("inf"))


_ENV: dict = {"env": None}


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._task_reset_hooks = []
        cls.reset = reset
    cls.reset._task_reset_hooks.append(_reset_episode)
    if not getattr(cls._task_step, "_fod_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env  # so score() can check that the grippers have let go
            return base(self, *a, **k)

        _task_step._fod_hooked = True
        cls._task_step = _task_step


_install_hooks()
_START_PTS = CORD.start_points()


def _track(objects) -> None:
    c = objects["cord"]
    if np.abs(c.points - _START_PTS).max() < 0.003 and all(
            np.linalg.norm(objects[n].pos[:2] - DISC_XY[n]) < 0.002 for n in DISC_NAMES):
        _reset_episode()  # untouched starting layout => new episode
    for n in DISC_NAMES:
        d = objects[n]
        if d.pos[2] > DISC_DOWN_Z or disc_in_bin(d):
            continue  # lifted (carried) or already binned: not debris on the board
        gap = cable_disc_gap(objects, n)
        _STATE["min_gap"] = min(_STATE["min_gap"], gap)
        if gap < TOUCH_GAP:
            _STATE["violated"] = True


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


def released(objects) -> bool:
    c = objects["cord"]
    pts = np.vstack([_dense(c.polyline()), c.end_plug[None]] + [objects[n].pos[None] for n in DISC_NAMES])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def plug_in_outline(objects) -> bool:
    """The plug body's whole footprint inside the outline, lying on the board."""
    c = objects["cord"]
    p, d = c.end_plug, c.end_dir[:2] / (np.linalg.norm(c.end_dir[:2]) + 1e-12)
    n = np.array([-d[1], d[0]])
    corners = [p[:2] + a * PLUG.size[0] / 2 * d + b * PLUG.size[1] / 2 * n for a in (-1, 1) for b in (-1, 1)]
    inside = all(abs(q[0] - PLUG_GOAL[0]) <= OUTLINE_IN[0] / 2 and abs(q[1] - PLUG_GOAL[1]) <= OUTLINE_IN[1] / 2
                 for q in corners)
    return bool(inside and p[2] <= BT + PLUG.size[2] / 2 + 0.006)


def cable_on_line(objects) -> bool:
    """Every point of the cable (connector block to plug) within 1.5 cm of the route line and lying on the board."""
    pts = _dense(objects["cord"].polyline())
    return bool(np.abs(pts[:, 0] - RX).max() <= ROUTE_TOL and pts[:, 2].max() <= LYING_Z)


def score(objects) -> float:
    """0 if the cable ever touched / lay over a disc resting on the board, if a disc is not in the bin at the end, or
    if a gripper still holds the cable, plug or a disc. Else 0.5 (both discs binned) + 0.25 (plug inside the outline)
    + 0.25 (whole cable within 1.5 cm of the line, lying on the board)."""
    _track(objects)
    if _STATE["violated"]:
        return 0.0
    if not all(disc_in_bin(objects[n]) for n in DISC_NAMES):
        return 0.0
    if not released(objects):
        return 0.0
    return 0.5 + 0.25 * plug_in_outline(objects) + 0.25 * cable_on_line(objects)


def success(objects) -> bool:
    s = score(objects)
    return bool(s >= 1.0 and objects["cord"].resting(0.02, 1.0) and all(objects[n].resting() for n in DISC_NAMES))


def distance(objects) -> float:
    """Plug centre to the outline centre (m) once both discs are binned; 1 m otherwise or after a violation."""
    _track(objects)
    if _STATE["violated"] or not all(disc_in_bin(objects[n]) for n in DISC_NAMES):
        return 1.0
    return float(np.linalg.norm(objects["cord"].end_plug[:2] - PLUG_GOAL))


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------


def _log(ctx, tag=""):
    st = ctx.states()
    c = st["cord"]
    pts = _dense(c.polyline())
    ds = " ".join(f"{n}={np.round(st[n].pos, 3)} bin={disc_in_bin(st[n])}" for n in DISC_NAMES)
    print(f"[t={ctx.t:5.2f}] {tag}: {ds} plug={np.round(c.end_plug, 3)} dev={np.abs(pts[:, 0] - RX).max():.3f} "
          f"zmax={pts[:, 2].max() - BT:.3f} outline={plug_in_outline(st)} viol={_STATE['violated']} "
          f"min_gap={_STATE['min_gap']:.3f} rel={released(st)} score={score(st):.2f}", flush=True)


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _cord_at(objects, a: float) -> tuple[np.ndarray, float]:
    poly = objects["cord"].polyline()
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    p = np.array([np.interp(a, s, poly[:, k]) for k in range(3)])
    i = min(max(int(np.searchsorted(s, a)), 1), len(poly) - 1)
    d = poly[i] - poly[i - 1]
    return p, math.atan2(d[1], d[0])


HI = TOP + 0.08
BIN_DROP_Z = TOP + 0.065


def _bin_disc(ctx, arm, name, drop_y, f_open, f_closed, *, target=None):
    """Pick a disc and drop it into the bin (or at ``target`` xy for the sweep/drop cheats)."""
    p = ctx.states()[name].pos
    yield from arm.move((p[0], p[1], HI), yaw=0.0, finger=f_open, duration=1.4)
    p = ctx.states()[name].pos
    yield from arm.move((p[0], p[1], p[2] + 0.010), yaw=0.0, duration=1.0)
    yield from arm.grip(f_closed, 0.6)
    yield from arm.move((p[0], p[1], HI), yaw=0.0, duration=0.8)
    gx, gy = (BIN_C[0], drop_y) if target is None else target
    gz = BIN_DROP_Z if target is None else BT + DISC_H / 2 + 0.012
    yield from arm.move((gx, gy, HI), yaw=0.0, duration=1.2)
    yield from arm.move((gx, gy, gz), yaw=0.0, duration=0.6)
    yield from arm.grip(f_open, 0.5)
    yield from arm.move((gx, gy, HI), yaw=0.0, duration=0.5)


MID_S = 0.20  # left arm takes the cord this far from the block


def _route(ctx, L, Rt, f_open, f_closed):
    """Left arm takes the cord mid-way, right arm the plug; both carry their part onto the line and lay it down."""
    f_cable = -0.014  # pads ~28 mm apart for the bare cord
    st = ctx.states()
    pm, hm = _cord_at(st, MID_S)
    gm = _grip_yaw(hm)
    c = st["cord"]
    pp = c.end_plug
    gp = _grip_yaw(math.atan2(c.end_dir[1], c.end_dir[0]))
    up = BT + 0.04
    yield from ctx.both(L.move((pm[0], pm[1], up + 0.03), yaw=gm, finger=f_cable, duration=1.4),
                        Rt.move((pp[0], pp[1], up + 0.03), yaw=gp, finger=f_open, duration=1.4))
    st = ctx.states()
    pm, _ = _cord_at(st, MID_S)
    pp = st["cord"].end_plug
    yield from ctx.both(L.move((pm[0], pm[1], pm[2] + 0.010), yaw=gm, duration=1.0),
                        Rt.move((pp[0], pp[1], pp[2] + 0.010), yaw=gp, duration=1.0))
    yield from ctx.both(L.grip(f_closed, 0.6), Rt.grip(f_closed, 0.6))
    yield from ctx.both(L.move((pm[0], pm[1], up), yaw=gm, duration=0.7),
                        Rt.move((pp[0], pp[1], up), yaw=gp, duration=0.7))
    _log(ctx, "cord lifted")
    # carry: mid point onto the line, plug past its outline centre by a little so the cord ends up straight
    mid_goal = (RX, ROUTE_Y0 - MID_S)
    yr = -math.pi / 2 if gp < 0 else math.pi / 2
    plug_goal = (RX, PLUG_GOAL[1] + 0.004)
    yl = -math.pi / 2 if gm < 0 else math.pi / 2
    yield from ctx.both(L.move((mid_goal[0], mid_goal[1], up), yaw=yl, duration=1.6),
                        Rt.move((plug_goal[0], plug_goal[1], up), yaw=yr, duration=1.6))
    off = ctx.states()["cord"].end_plug[:2] - Rt.tcp()[:2]  # plug centre relative to the TCP
    gx, gy = PLUG_GOAL - off
    yield from Rt.move((gx, gy, up), yaw=yr, duration=0.5)
    _log(ctx, "over line")
    lay = BT + R + 0.010
    yield from ctx.both(L.move((mid_goal[0], mid_goal[1], lay), yaw=yl, duration=1.0),
                        Rt.move((gx, gy, BT + PLUG.size[2] / 2 + 0.010), yaw=yr, duration=1.0))
    yield from ctx.both(L.grip(f_cable, 0.5), Rt.grip(f_open, 0.5))
    yield from ctx.both(L.move((mid_goal[0], mid_goal[1], HI + 0.04), yaw=yl, duration=0.8),
                        Rt.move((gx, gy, HI + 0.04), yaw=yr, duration=0.8))
    yield from ctx.both(L.move((0.20, 0.25, HI + 0.04), yaw=0.0, duration=1.0),
                        Rt.move((0.20, -0.25, HI + 0.04), yaw=0.0, duration=1.0))


def demo_script(ctx):
    """FOD_DEMO=correct (default) | nothing | route (lay the cable, ignoring the discs) | sweep (push the discs off
    the board onto the table, then lay the cable) | drop (pick the discs but put them on the table, not in the bin,
    then lay the cable) | bin_only (bin the discs, leave the cable: 0.5)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    mode = os.environ.get("FOD_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "nothing":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    if mode == "sweep":
        # closed gripper pushes each disc off the board, away from the robot (+x), onto the table
        z = BT + 0.012
        for arm, n in ((L, "disc_a"), (Rt, "disc_b")):
            p = ctx.states()[n].pos
            yield from arm.move((p[0] - 0.04, p[1], z + 0.05), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
            yield from arm.move((p[0] - 0.04, p[1], z), yaw=0.0, duration=0.6)
            yield from arm.move((SHEET_X[1] + 0.03, p[1], z), yaw=0.0, duration=1.5)
            yield from arm.move((SHEET_X[1] + 0.03, p[1], z + 0.06), yaw=0.0, duration=0.5)
            yield from arm.move((0.20, 0.25 if arm is L else -0.25, HI), yaw=0.0, duration=1.0)
        _log(ctx, "swept")
    elif mode == "drop":
        yield from ctx.both(_bin_disc(ctx, L, "disc_a", 0, FINGER_OPEN, FINGER_CLOSED, target=(0.25, 0.20)),
                            _bin_disc(ctx, Rt, "disc_b", 0, FINGER_OPEN, FINGER_CLOSED, target=(0.25, -0.22)))
        _log(ctx, "dropped on table")
    elif mode != "route":
        # Correct: each arm picks the disc on its side; left drops first, then right (one bin between the arms).
        yield from ctx.both(_bin_disc(ctx, L, "disc_a", BIN_C[1] + 0.015, FINGER_OPEN, FINGER_CLOSED),
                            _pick_then_wait(ctx, Rt, "disc_b", FINGER_OPEN, FINGER_CLOSED))
        yield from ctx.both(L.move((0.22, 0.20, HI), yaw=0.0, duration=0.8),
                            _drop_held(Rt, BIN_C[1] - 0.015, FINGER_OPEN, delay=0.6))
        _log(ctx, "discs binned")
        if mode == "bin_only":
            yield from Rt.move((0.20, -0.25, HI + 0.04), yaw=0.0, duration=1.0)
            yield from L.wait(1.0)
            _log(ctx, "end")
            return
    yield from _route(ctx, L, Rt, FINGER_OPEN, FINGER_CLOSED)
    _log(ctx, "routed")
    yield from L.wait(1.5)
    _log(ctx, "end")


def _pick_then_wait(ctx, arm, name, f_open, f_closed):
    p = ctx.states()[name].pos
    yield from arm.move((p[0], p[1], HI), yaw=0.0, finger=f_open, duration=1.4)
    p = ctx.states()[name].pos
    yield from arm.move((p[0], p[1], p[2] + 0.010), yaw=0.0, duration=1.0)
    yield from arm.grip(f_closed, 0.6)
    yield from arm.move((p[0], p[1], HI), yaw=0.0, duration=0.8)
    yield from arm.move((0.24, -0.10, HI), yaw=0.0, duration=1.0)


def _drop_held(arm, drop_y, f_open, delay=0.0):
    if delay:
        yield from arm.wait(delay)
    yield from arm.move((BIN_C[0], drop_y, HI), yaw=0.0, duration=1.0)
    yield from arm.move((BIN_C[0], drop_y, BIN_DROP_Z), yaw=0.0, duration=0.6)
    yield from arm.grip(f_open, 0.5)
    yield from arm.move((BIN_C[0], drop_y, HI), yaw=0.0, duration=0.5)
    yield from arm.move((0.22, -0.20, HI), yaw=0.0, duration=0.8)
