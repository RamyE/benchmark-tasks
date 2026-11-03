"""On the station table, in front of the left arm, a grey weatherproof enclosure (80 x 120 x 80 mm box) stands on a
40 x 40 mm post so that its bottom is 6 cm above the table. A black cable gland (14 x 18 x 18 mm, angled 23 deg towards the robot) sits on the
enclosure's right-hand face (facing the right arm), its centre 10 cm above the table; a black 6 mm cable, 50 cm long,
leaves the gland horizontally, droops to the table and lies on it towards the front right, ending in a small black
plug (35 x 16 x 12 mm). In front of the right arm, 18 cm to the right of the gland and in line with it, stands a
grey support hook: a 20 mm round post topped by an open U cradle whose floor is 14 cm above the table (4 cm above
the gland). The cable must be laid over the hook so that between the hook and the gland it hangs in a drip loop:
its lowest point at least 3 cm (but no more than 7 cm) below the centre of the gland, so water running along the
cable drips off the low point instead of running into the gland. Then the robot lets go.

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

INSTRUCTION = ("The black cable comes out of the gland on the side of the grey enclosure. Lay the cable over the "
               "support hook so that, between the hook and the gland, it hangs down in a drip loop whose lowest point "
               "is 3 to 7 cm below the centre of the gland, then rises into the gland. The rest of the cable hangs "
               "down on the far side of the hook. Then let go of the cable.")

GREY = (0.62, 0.64, 0.66)
DARK = (0.10, 0.10, 0.11)

# -- enclosure on a stand, gland on its right-hand (-y) face -----------------------------------------------------
BOX_X, BOX_Y0 = 0.27, 0.14  # enclosure centred on x 0.27; its right face at y 0.14
BOX_SIZE = (0.08, 0.12, 0.08)
BOX_BOTTOM = TOP + 0.06
GLAND_Z = TOP + 0.10  # gland axis = cable centre where it leaves the enclosure
GLAND_SIZE = (0.014, 0.018, 0.018)  # gland nut (fixed), long side along the cable
ENCLOSURE = [
    Box("enclosure", pos=(BOX_X, BOX_Y0 + BOX_SIZE[1] / 2, BOX_BOTTOM + BOX_SIZE[2] / 2), size=BOX_SIZE,
        color=GREY),
    Box("enclosure_stand", pos=(BOX_X, BOX_Y0 + BOX_SIZE[1] / 2, TOP + (BOX_BOTTOM - TOP) / 2 - 0.0005),
        size=(0.04, 0.04, BOX_BOTTOM - TOP - 0.001), color=GREY),
]

# -- support hook: post + open-top U cradle running along y ------------------------------------------------------
HOOK_X, HOOK_Y = 0.27, -0.05
LINE_Z = TOP + 0.14  # top of the cradle floor
POST_R = 0.010
FLOOR_T = 0.008
CRADLE_LX, CRADLE_LY = 0.032, 0.025
SLOT_W = 0.024
WALL_T, WALL_H = 0.004, 0.012


def _hook() -> list:
    post_h = LINE_Z - FLOOR_T - TOP
    parts = [
        Cylinder("hook_post", pos=(HOOK_X, HOOK_Y, TOP + post_h / 2), radius=POST_R, height=post_h, color=GREY),
        Box("hook_floor", pos=(HOOK_X, HOOK_Y, LINE_Z - FLOOR_T / 2), size=(CRADLE_LX, CRADLE_LY, FLOOR_T),
            color=GREY),
    ]
    for s, sg in (("front", -1), ("back", 1)):
        parts.append(Box(f"hook_wall_{s}", pos=(HOOK_X + sg * (SLOT_W / 2 + WALL_T / 2), HOOK_Y,
                                               LINE_Z - FLOOR_T + (FLOOR_T + WALL_H) / 2),
                         size=(WALL_T, CRADLE_LY, FLOOR_T + WALL_H), color=GREY))
    return parts


# -- the cable: start (gland) welded to the world, leaving the gland horizontally, angled 23 deg towards the robot
R = 0.003
CORD_LEN = 0.50
CORD_YAW = -math.pi / 2 - 0.40
_D = np.array([math.cos(CORD_YAW), math.sin(CORD_YAW), 0.0])
_GLAND_C = np.array([BOX_X, BOX_Y0 - 0.004, GLAND_Z]) + _D * 0.003  # gland centre (its back sunk into the face)
CORD_START = tuple((_GLAND_C + _D * (GLAND_SIZE[0] / 2 + 0.006)).tolist())  # cable start 6 mm off the gland face
GLAND = Box("gland", pos=tuple(_GLAND_C.tolist()), size=GLAND_SIZE, yaw=CORD_YAW, color=DARK)
PLUG = Plug(size=(0.035, 0.016, 0.012), mass=0.012, color=DARK)
CORD = cable("cord", start=CORD_START, yaw=CORD_YAW, length=CORD_LEN, radius=R, anchor_start=True,
             end_plug=PLUG, color=(0.05, 0.05, 0.05), bend_stiffness=0.01, bend_damping=0.001)

OBJECTS = [*ENCLOSURE, GLAND, *_hook(), CORD]

# -- start-of-episode settling -----------------------------------------------------------------------------------
# The kit spawns a cable straight; this one spawns sticking out of the gland and drops onto the table. After every
# reset the scene is stepped (arms at READY) for SETTLE_EXTRA s more so each trial starts with the cable at rest,
# hanging from the gland and lying on the table.
SETTLE_EXTRA = 2.5
_ENV: dict = {"env": None}


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_drip_hooked", False):
        base_step = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base_step(self, *a, **k)

        _task_step._drip_hooked = True
        cls._task_step = _task_step
    if not getattr(cls.reset, "_drip_hooked", False):
        base_reset = cls.reset

        def reset(self, *a, **k):
            base_reset(self, *a, **k)
            _ENV["env"] = self._env
            b = self._base
            for _ in range(max(1, round(SETTLE_EXTRA / b.step_dt))):
                self._env.step(self._ready)
            for _ in range(4):
                b.sim.render()
            self._t = 0.0
            return self._observation()

        reset._drip_hooked = True
        cls.reset = reset


_install_hooks()

# -- scoring -----------------------------------------------------------------------------------------------------
DRIP_MIN, DRIP_MAX = 0.03, 0.07  # full credit: lowest point 3-7 cm below the gland centre
DRIP_ZERO = 0.015  # less than this: level / taut / sloping down into the gland = no drip loop -> 0
TAIL_PAST = 0.03  # the cable end (plug) must be at least this far past the hook, below the hook line
RELEASE_DIST = 0.03


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    poly = poly[np.isfinite(poly).all(axis=1)]
    if len(poly) == 0:
        return np.full((1, 3), -1.0)
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _in_slot(pts: np.ndarray) -> np.ndarray:
    return ((np.abs(pts[:, 0] - HOOK_X) <= SLOT_W / 2) & (np.abs(pts[:, 1] - HOOK_Y) <= CRADLE_LY / 2)
            & (pts[:, 2] >= LINE_Z) & (pts[:, 2] <= LINE_Z + WALL_H))


def on_hook(objects) -> bool:
    """The cable lies in the hook's cradle and runs on past it: its end plug is beyond the hook (farther from the
    gland) by at least 3 cm and below the hook line."""
    c = objects["cord"]
    pts = _dense(c.polyline())
    p = c.end_plug if c.end_plug is not None else c.end_tip
    return bool(_in_slot(pts).any() and p[1] <= HOOK_Y - TAIL_PAST and p[2] < LINE_Z)


def drip(objects) -> float:
    """Gland centre height minus the lowest point of the cable's underside between the gland and the cradle (m)."""
    pts = _dense(objects["cord"].polyline())
    ins = np.flatnonzero(_in_slot(pts))
    if len(ins) == 0:
        return -1.0
    span = pts[: ins[0] + 1]
    return float(GLAND_Z - (span[:, 2].min() - R))


def grippers() -> list[np.ndarray]:
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
    pts = np.vstack([_dense(c.polyline())] + [c.end_plug[None]])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """0 unless the cable is released and lies over the hook (in the cradle, its end hanging past the hook).
    Then, with drip = gland centre minus the lowest point of the cable between gland and hook:
    drip < 1.5 cm (level, taut or sloping down into the gland) 0; 1.5-3 cm 0.4; 3-7 cm 1.0; > 7 cm 0.4."""
    if not released(objects) or not on_hook(objects):
        return 0.0
    d = drip(objects)
    if d < DRIP_ZERO:
        return 0.0
    if DRIP_MIN <= d <= DRIP_MAX:
        return 1.0
    return 0.4


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["cord"].resting(0.02, 1.0)


def distance(objects) -> float:
    if not on_hook(objects):
        return 1.0
    return abs(drip(objects) - 0.05)


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------

GRASP_S = float(os.environ.get("DRIP_GRASP_S", "0.24"))  # arclength (m from the gland) the right arm takes
PULL_TARGET = float(os.environ.get("DRIP_PULL", "0.060"))  # drip set while the left arm holds the loop


def _point_at(c, s: float) -> np.ndarray:
    poly = c.polyline()
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    acc = np.r_[0.0, np.cumsum(seg)]
    i = int(np.clip(np.searchsorted(acc, s) - 1, 0, len(seg) - 1))
    t = (s - acc[i]) / max(seg[i], 1e-9)
    return poly[i] + t * (poly[i + 1] - poly[i]), poly[i + 1] - poly[i]


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _span_mid(objects):
    """Middle (by length) of the cable between the gland and the hook's cradle, and the local heading there."""
    c = objects["cord"]
    pts = _dense(c.polyline())
    ins = np.flatnonzero(_in_slot(pts))
    n = ins[0] if len(ins) else len(pts) // 2
    seg = np.linalg.norm(np.diff(pts[: n + 1], axis=0), axis=1)
    return _point_at(c, 0.5 * float(seg.sum()))


def demo_script(ctx):
    """DRIP_DEMO = correct (default) | none (do nothing) | level (lay the cable over the hook pulled straight, no
    drip loop) | hold (form the loop but keep holding it down).
    Right arm: takes the cable 24 cm from the gland, carries it over the hook and lays it in the cradle, the tail
    falling on the far side. Left arm: takes the cable at the low point between the gland and the hook and pulls it
    down (drawing cable over the hook from the tail) until the loop's low point is ~6.5 cm below the gland, lets go.
    """
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    mode = os.environ.get("DRIP_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    half = (FINGER_OPEN + FINGER_CLOSED) / 2 if FINGER_OPEN > FINGER_CLOSED else FINGER_OPEN
    f_open = FINGER_OPEN

    def log(tag):
        st = ctx.states()
        c = st["cord"]
        print(f"[demo t={ctx.t:5.1f}] {tag}: on_hook={on_hook(st)} drip={drip(st):.3f} rel={released(st)} "
              f"score={score(st):.2f} plug={np.round(c.end_plug, 3)} v={np.linalg.norm(c.lin_vel):.3f} "
              f"w={np.linalg.norm(c.ang_vel):.2f} succ={success(st)}", flush=True)

    log("start")
    if mode == "none":
        yield from L.wait(3.0)
        log("end")
        return
    # 1) right arm takes the cable at GRASP_S
    st = ctx.states()
    p, h = _point_at(st["cord"], GRASP_S)
    ry = _grip_yaw(math.atan2(h[1], h[0]))
    yield from Rt.move((p[0], p[1], TOP + 0.06), yaw=ry, finger=f_open, duration=1.5)
    p, h = _point_at(ctx.states()["cord"], GRASP_S)
    yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=ry, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED)
    log("right grasped")
    # 2) lift, carry over the cradle (gland side first), on past the hook, and down on the far side
    carry = LINE_Z + 0.04
    yield from Rt.move((p[0], p[1], carry), yaw=ry, duration=1.0)
    yield from Rt.move((HOOK_X, HOOK_Y + 0.03, carry), yaw=math.pi / 2, duration=1.5)
    yield from Rt.move((HOOK_X, HOOK_Y - 0.05, carry), yaw=math.pi / 2, duration=1.0)
    yield from Rt.move((HOOK_X, HOOK_Y - 0.05, LINE_Z - 0.03), yaw=math.pi / 2, duration=1.0)
    yield from Rt.wait(0.5)
    log("laid over hook")
    if mode == "level":  # pull the tail down so the span from the gland runs straight up to the hook, let go
        yield from Rt.move((HOOK_X, HOOK_Y - 0.07, TOP + 0.05), yaw=math.pi / 2, duration=1.5)
        log("pulled straight")
    yield from Rt.grip(f_open)
    yield from Rt.move((HOOK_X, HOOK_Y - 0.05, LINE_Z + 0.03), yaw=math.pi / 2, duration=0.6)
    yield from Rt.move((0.12, -0.30, TOP + 0.14), yaw=0.0, duration=1.2)  # park beside the right base
    yield from Rt.wait(1.0)
    log("right released")
    if mode == "level":
        yield from Rt.wait(1.5)
        log("end")
        return
    # 3) left arm takes the cable at the loop's low point and pulls it down to the target drip (closed loop)
    lp, lh = _span_mid(ctx.states())
    ly = _grip_yaw(math.atan2(lh[1], lh[0]))  # fingers close across the cable
    yield from L.move((lp[0], lp[1], lp[2] + 0.06), yaw=ly, finger=f_open, duration=1.5)
    lp, _ = _span_mid(ctx.states())
    yield from L.move((lp[0], lp[1], lp[2] + 0.010), yaw=ly, duration=1.0)
    yield from L.grip(FINGER_CLOSED)
    log("left grasped")
    z = lp[2] + 0.010
    for _ in range(5):
        d = drip(ctx.states())
        if d >= PULL_TARGET - 0.004:
            break
        if z <= TOP + 0.035 + 1e-6:
            break
        z -= max(0.01, PULL_TARGET - d)
        z = max(z, TOP + 0.035)
        yield from L.move((lp[0], lp[1], z), yaw=ly, duration=1.0)
        yield from L.wait(0.3)
        log(f"pulled z={z - TOP:.3f}")
    if mode == "hold":
        yield from L.wait(3.0)
        log("end (held)")
        return
    yield from L.grip(f_open)
    yield from L.move((lp[0], lp[1], z + 0.06), yaw=ly, duration=0.8)
    yield from L.move((lp[0] - 0.10, lp[1], TOP + 0.16), yaw=ly, duration=0.8)
    yield from L.move((0.12, 0.30, TOP + 0.14), yaw=0.0, duration=1.2)  # park beside the left base
    log("left released")
    for _ in range(6):
        yield from L.wait(1.0)
        log("settling")
    log("end")
