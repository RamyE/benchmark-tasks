"""On the station table, two grey device boxes stand at the back (front faces ~50 cm in front of the robot), one
left of the midline and one right of it. A red 6 mm cable, 32 cm long, leaves the left device and a blue 6 mm
cable, 32 cm long, leaves the right device; each has a small plug (35 x 16 x 12 mm, 10 g) on its free end. The two
cables lie straight and cross each other once in the middle of the table, the blue one resting on top of the red
one: the red plug ends up in front of the right arm and the blue plug in front of the left arm. At the front centre
stands a grey two-port charging dock: two U-shaped ports side by side, open towards the devices and at the top, a
red label over the left port and a blue label over the right port. The cables must be untangled so that they no
longer cross (in top view), with the red plug in the red (left) port and the blue plug in the blue (right) port,
and the robot must let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("The red and blue cables are crossed. Untangle them so that they no longer cross each other, then put "
               "the red cable's plug into the red (left) port and the blue cable's plug into the blue (right) port "
               "of the charging dock, and let go of both plugs.")

GREY = (0.45, 0.47, 0.50)
LIGHT = (0.85, 0.85, 0.83)
RED = (0.85, 0.07, 0.05)
BLUE = (0.05, 0.25, 0.85)

# -- devices at the back (the cables are fixed where they leave them) ------------------------------------------
ANCHOR_X = 0.50
ANCHOR_Y = 0.10  # red leaves the left device at y +ANCHOR_Y, blue the right device at y -ANCHOR_Y
DEV_SIZE = (0.10, 0.12, 0.06)
DEVICES = [Box(f"device_{t}", pos=(ANCHOR_X + 0.006 + DEV_SIZE[0] / 2, sg * ANCHOR_Y, TOP + DEV_SIZE[2] / 2),
               size=DEV_SIZE, color=GREY) for t, sg in (("red", 1), ("blue", -1))]

# -- dock with two ports (U slots along x, open at +x (towards the devices) and at the top) ---------------------
SLOT_L, SLOT_W, SLOT_H, WALL_T = 0.045, 0.024, 0.008, 0.004
SLOT_X = 0.205  # slot centre x
SLOT_X0, SLOT_X1 = SLOT_X - SLOT_L / 2, SLOT_X + SLOT_L / 2  # closed (dock) end, open end
PORT_Y = {"red": (SLOT_W + WALL_T) / 2, "blue": -(SLOT_W + WALL_T) / 2}  # +-14 mm
DOCK_SIZE = (0.05, 2 * SLOT_W + 3 * WALL_T, 0.03)
_wall_y = [PORT_Y["red"] + (SLOT_W + WALL_T) / 2, 0.0, PORT_Y["blue"] - (SLOT_W + WALL_T) / 2]
DOCK = [
    *[Box(f"port_wall_{k}", pos=(SLOT_X, y, TOP + SLOT_H / 2), size=(SLOT_L, WALL_T, SLOT_H), color=LIGHT)
      for k, y in enumerate(_wall_y)],
    Box("dock_body", pos=(SLOT_X0 - DOCK_SIZE[0] / 2, 0.0, TOP + DOCK_SIZE[2] / 2), size=DOCK_SIZE, color=GREY),
    *[Box(f"label_{t}", pos=(SLOT_X0 - DOCK_SIZE[0] / 2, y, TOP + DOCK_SIZE[2] + 0.0005),
          size=(0.03, SLOT_W, 0.001), color=c) for t, y, c in (("red", PORT_Y["red"], RED),
                                                                ("blue", PORT_Y["blue"], BLUE))],
]

# -- the two cables: straight, crossing once in the middle, blue on top ----------------------------------------
R = 0.003  # 6 mm cable
CORD_LEN = 0.32
PLUG_SIZE = (0.035, 0.016, 0.012)
END_Y = 0.12  # each cable's free end lies on the far side of the midline (red at y -END_Y, blue at +END_Y)
_dx = -math.sqrt(CORD_LEN ** 2 - (2 * ANCHOR_Y + 0.02) ** 2)
YAW_RED = math.atan2(-(ANCHOR_Y + END_Y), _dx)
YAW_BLUE = math.atan2(ANCHOR_Y + END_Y, _dx)
Z_RED = TOP + PLUG_SIZE[2] / 2 + 0.0005  # plugs lie flat on the table; the cable settles 3 mm
Z_BLUE = Z_RED + 2 * R + 0.001  # blue spawns just above red so it rests on top at the crossing


def _cord(name, y0, yaw, z, color):
    return cable(name, start=(ANCHOR_X, y0, z), yaw=yaw, length=CORD_LEN, radius=R, anchor_start=True,
                 end_plug=Plug(size=PLUG_SIZE, mass=0.010, color=color), color=color,
                 bend_stiffness=0.01, bend_damping=0.001)


RED_CORD = _cord("red_cable", ANCHOR_Y, YAW_RED, Z_RED, RED)
BLUE_CORD = _cord("blue_cable", -ANCHOR_Y, YAW_BLUE, Z_BLUE, BLUE)
CORDS = {"red": "red_cable", "blue": "blue_cable"}

OBJECTS = [*DEVICES, RED_CORD, BLUE_CORD, *DOCK]

# -- scoring ---------------------------------------------------------------------------------------------------------
RELEASE_DIST = 0.03  # every gripper (TCP) at least this far from both cables and plugs


def _poly2(c) -> np.ndarray:
    """Top-view centre line of a cable: start tip, segment centres, end tip, plug far end."""
    pts = c.polyline()
    if c.end_plug is not None:
        pts = np.vstack([pts, c.end_plug + c.end_dir * PLUG_SIZE[0] / 2])
    return pts[:, :2]


def _cross2(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def crossings(objects) -> int:
    """Number of places where the red and blue centre lines cross in top view (height ignored)."""
    p, q = _poly2(objects["red_cable"]), _poly2(objects["blue_cable"])
    a0, a1 = p[:-1, None], p[1:, None]
    b0, b1 = q[None, :-1], q[None, 1:]
    da, db = a1 - a0, b1 - b0
    d1 = _cross2(db, a0 - b0)
    d2 = _cross2(db, a1 - b0)
    d3 = _cross2(da, b0 - a0)
    d4 = _cross2(da, b1 - a0)
    return int(((d1 * d2 < 0) & (d3 * d4 < 0)).sum())


def port_of(objects, colour: str) -> str | None:
    """Which port ("red"/"blue") the plug of the ``colour`` cable is seated in: plug centre between that port's
    walls, inside the slot (sticking out of the open end by at most 1 cm), lying on the table."""
    p = objects[CORDS[colour]].end_plug
    if p is None:
        return None
    half = PLUG_SIZE[0] / 2
    if not (SLOT_X0 + half - 0.003 <= p[0] <= SLOT_X1 - half + 0.010 and p[2] <= TOP + PLUG_SIZE[2] / 2 + 0.004):
        return None
    for port, y in PORT_Y.items():
        if abs(p[1] - y) <= (SLOT_W - PLUG_SIZE[1]) / 2 + 0.002:
            return port
    return None


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_uncross_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._uncross_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads), [] if unknown."""
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
    pts = []
    for name in CORDS.values():
        c = objects[name]
        pts.append(c.polyline())
        if c.end_plug is not None:
            pts.append(c.end_plug[None])
    pts = np.vstack(pts)
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """0 unless both plugs are released, the cables do not cross anywhere in top view and no plug sits in the
    other cable's port. Then 0.2 + 0.4 for each plug seated in its own port (both = 1.0)."""
    if not all(np.isfinite(objects[n].points).all() for n in CORDS.values()):
        return 0.0
    if not released(objects) or crossings(objects) > 0:
        return 0.0
    ports = {c: port_of(objects, c) for c in CORDS}
    if any(p is not None and p != c for c, p in ports.items()):
        return 0.0
    return 0.2 + 0.4 * sum(p == c for c, p in ports.items())


def success(objects) -> bool:
    return score(objects) >= 1.0 and all(objects[n].resting(max_speed=0.02) for n in CORDS.values())


def distance(objects) -> float:
    """Sum of the two plugs' distances to their own ports (m), +1 m while the cables cross."""
    d = sum(float(np.linalg.norm(objects[n].end_plug[:2] - np.array([SLOT_X, PORT_Y[c]]))) for c, n in CORDS.items())
    return d + (1.0 if crossings(objects) > 0 else 0.0)


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _plug(st, name):
    c = st[name]
    return c.end_plug, _grip_yaw(math.atan2(c.end_dir[1], c.end_dir[0]))


def demo_script(ctx):
    """Both arms take the plugs. The left arm lifts the blue (top) cable's plug high and holds it, which lifts the
    blue cable off the crossing; the right arm pulls the red (bottom) cable out from under it, low over the table,
    and drops its plug into the red port; then the left arm lowers the blue plug into the blue port. Both let go."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("UNCROSS_CHEAT", "")  # test only: "none" | "drag" | "swap" | "hold"
    over = 0.010  # TCP above the plug centre when grasping
    hi = TOP + 0.06
    drop = TOP + SLOT_H + 0.013 + 0.003  # fingertips 3 mm above the port walls

    def log(tag):
        st = ctx.states()
        print(f"[demo t={ctx.t:5.1f}] {tag}: cross={crossings(st)} ports={ {c: port_of(st, c) for c in CORDS} } "
              f"rel={released(st)} score={score(st):.2f} red={np.round(st['red_cable'].end_plug, 3)} "
              f"blue={np.round(st['blue_cable'].end_plug, 3)}", flush=True)

    yield from L.wait(0.5)  # let the cables settle
    log("start")
    if cheat == "none":
        yield from L.wait(3.0)
        log("end")
        return
    st = ctx.states()
    pb, yb = _plug(st, "blue_cable")
    pr, yr = _plug(st, "red_cable")
    # 1) take both plugs
    yield from ctx.both(L.move((pb[0], pb[1], hi), yaw=yb, finger=FINGER_OPEN, duration=1.5),
                        Rt.move((pr[0], pr[1], hi), yaw=yr, finger=FINGER_OPEN, duration=1.5))
    st = ctx.states()
    pb, _ = _plug(st, "blue_cable")
    pr, _ = _plug(st, "red_cable")
    yield from ctx.both(L.move((pb[0], pb[1], pb[2] + over), yaw=yb, duration=1.0),
                        Rt.move((pr[0], pr[1], pr[2] + over), yaw=yr, duration=1.0))
    yield from ctx.both(L.grip(FINGER_CLOSED), Rt.grip(FINGER_CLOSED))
    log("grasped")
    port_r = {"red": PORT_Y["red"], "blue": PORT_Y["blue"]}
    if cheat == "swap":  # each arm drops its plug into the nearer, wrong-colour port (the cables stay crossed)
        port_r = {"red": PORT_Y["blue"], "blue": PORT_Y["red"]}
    if cheat in ("drag", "swap"):  # drag both plugs over the table into the ports, nothing lifted
        low = pr[2] + over
        yield from Rt.move((SLOT_X + 0.05, -0.06, low), yaw=0.0, duration=1.5)
        log("red dragged near")
        yield from Rt.move((SLOT_X + 0.05, port_r["red"], low), yaw=0.0, duration=1.5)
        yield from Rt.move((SLOT_X + 0.05, port_r["red"], drop), yaw=0.0, duration=0.5)
        yield from Rt.move((SLOT_X, port_r["red"], drop), yaw=0.0, duration=1.0)
        yield from Rt.grip(FINGER_OPEN)
        yield from Rt.move((SLOT_X, port_r["red"], hi), yaw=0.0, duration=0.8)
        yield from Rt.move((0.25, -0.18, hi), yaw=0.0, duration=1.0)
        yield from L.move((SLOT_X + 0.05, 0.06, low), yaw=0.0, duration=1.5)
        log("blue dragged near")
        yield from L.move((SLOT_X + 0.05, port_r["blue"], low), yaw=0.0, duration=1.5)
        yield from L.move((SLOT_X + 0.05, port_r["blue"], drop), yaw=0.0, duration=0.5)
        yield from L.move((SLOT_X, port_r["blue"], drop), yaw=0.0, duration=1.0)
        yield from L.grip(FINGER_OPEN)
        yield from L.move((SLOT_X, port_r["blue"], hi), yaw=0.0, duration=0.8)
        yield from L.move((0.25, 0.18, hi), yaw=0.0, duration=1.0)
        log("dropped")
        yield from L.wait(2.0)
        log("end")
        return
    # 2) left arm lifts the blue plug high: the blue cable comes up off the red one at the crossing
    hold = (0.37, 0.12, TOP + 0.12)
    yield from L.move((pb[0], pb[1], TOP + 0.10), yaw=yb, duration=1.0)
    yield from L.move(hold, yaw=0.0, duration=1.5)
    log("blue lifted")
    # 3) right arm pulls the red cable out from under it, low over the table, into the red port
    low = TOP + 0.04
    yield from Rt.move((pr[0], pr[1], low), yaw=yr, duration=0.6)
    log("red up")
    yield from Rt.move((SLOT_X + 0.05, PORT_Y["red"] - 0.02, low), yaw=0.0, duration=2.0)
    log("red near")
    yield from Rt.move((SLOT_X, PORT_Y["red"], low), yaw=0.0, duration=1.0)
    log("red above port")
    yield from Rt.move((SLOT_X, PORT_Y["red"], drop), yaw=0.0, duration=1.0)
    yield from Rt.grip(FINGER_OPEN)
    yield from Rt.move((SLOT_X, PORT_Y["red"], hi), yaw=0.0, duration=0.8)
    yield from Rt.move((0.25, -0.18, hi), yaw=0.0, duration=1.0)
    log("red in port")
    # 4) left arm lowers the blue plug into the blue port, right of the red cable
    yield from L.move((SLOT_X + 0.05, PORT_Y["blue"], TOP + 0.08), yaw=0.0, duration=1.5)
    yield from L.move((SLOT_X, PORT_Y["blue"], TOP + 0.05), yaw=0.0, duration=1.0)
    yield from L.move((SLOT_X, PORT_Y["blue"], drop), yaw=0.0, duration=1.0)
    if cheat == "hold":  # keep holding the blue plug in its port
        log("holding")
        yield from L.wait(3.0)
        log("end (held)")
        return
    yield from L.grip(FINGER_OPEN)
    yield from L.move((SLOT_X, PORT_Y["blue"], hi), yaw=0.0, duration=0.8)
    yield from L.move((0.25, 0.18, hi), yaw=0.0, duration=1.0)
    log("blue in port")
    yield from L.wait(2.0)
    log("end")
