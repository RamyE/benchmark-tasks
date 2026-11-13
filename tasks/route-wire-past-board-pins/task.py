"""Wire-harness formboard on the station table: a white 1 mm layout sheet carries a slalom row of five steel routing
pins (15 mm diameter, 40 mm tall, 9 cm apart across the board), a 44 mm steel turn post to the right of the row, and
a 6 mm blue printed route line. An orange 7 mm wire (95 cm) leaves a fixed grey start fixture at the board's
front-left and lies straight across the front of the board towards the right arm, ending in a black
55 x 16 x 12 mm connector plug. A black mating-connector block stands just past the turn post's right side, its cavity
facing the robot. The printed route weaves through the pin row (behind pin 1, in front of pin 2, behind pin 3, in front of
pin 4, behind pin 5), turns sharply round the front and right side of the turn post and runs into the cavity. The wire must
pass every pin and the post on the same side as the printed line, and the plug must be pushed into the cavity.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var PINS_DEMO): correct (default) | nothing | shortest | sweep | onearm | hold |
half | noplug.
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

INSTRUCTION = ("Route the orange wire on the harness board along the blue printed line: the wire must pass every steel "
               "pin on the same side as the blue line. Then push the wire's plug into the black connector at the end "
               "of the line and let go.")

# -- formboard: 1 mm white layout sheet ------------------------------------------------------------------------------
SHEET_T = 0.001
BT = TOP + SHEET_T  # board top
SHEET_X, SHEET_Y = (0.05, 0.50), (-0.65, 0.42)
WHITE = (0.95, 0.95, 0.93)
BLUE = (0.10, 0.25, 0.75)
STEEL = (0.38, 0.40, 0.44)
INK_T = 0.0003
LINE_W = 0.006
SHEET = Box("formboard", pos=((SHEET_X[0] + SHEET_X[1]) / 2, (SHEET_Y[0] + SHEET_Y[1]) / 2, TOP + SHEET_T / 2),
            size=(SHEET_X[1] - SHEET_X[0], SHEET_Y[1] - SHEET_Y[0], SHEET_T), color=WHITE)

# -- routing pins: a slalom row across the board, and a turn post --------------------------------------------------
PIN_R, PIN_H = 0.0075, 0.040
ROW_X = 0.28
ROW_Y = (0.18, 0.09, 0.00, -0.09, -0.18)
POST = np.array([0.31, -0.28])  # turn post centre
POST_R = 0.022  # 44 mm post: the wire's centreline bends at >= 25 mm radius round it
PINS = np.array([(ROW_X, y) for y in ROW_Y] + [tuple(POST)])  # route order: pin 1..5 (left to right), then the post
RADII = np.array([PIN_R] * len(ROW_Y) + [POST_R])
NAMES = [f"pin {k + 1}" for k in range(len(ROW_Y))] + ["turn post"]
# side of each pin the printed line passes (unit vector from the pin centre towards the line) and the line's
# direction of travel there: behind / in front alternately, then round the post's right side
SIDE = np.array([(1, 0), (-1, 0), (1, 0), (-1, 0), (1, 0), (0, -1)], dtype=float)
TRAVEL = np.array([(0, -1)] * len(ROW_Y) + [(1, 0)], dtype=float)
PIN_OBJS = [Cylinder(f"pin_{k + 1}", pos=(ROW_X, float(y), BT + PIN_H / 2), radius=PIN_R, height=PIN_H, color=STEEL)
            for k, y in enumerate(ROW_Y)]
POST_OBJ = Cylinder("turn_post", pos=(float(POST[0]), float(POST[1]), BT + PIN_H / 2), radius=POST_R, height=PIN_H,
                    color=STEEL)

# -- printed route: start fixture exit -> slalom through the row -> sharp turn round the post -> connector ----------
START = np.array([0.15, 0.35])  # wire exit from the start fixture
MOUTH = np.array([0.40, -0.32])  # connector cavity opening (insertion along +x), just past the post's right side
AMP = 0.03  # the line passes 3 cm from each pin centre
_arc = [(POST[0] - 0.04 * math.cos(a), POST[1] - 0.04 * math.sin(a)) for a in np.linspace(0.0, math.pi, 7)]
ROUTE = np.array([START, (START[0], START[1] - 0.05), *[(ROW_X + AMP * SIDE[k][0], y) for k, y in enumerate(ROW_Y)],
                  *_arc[:4], MOUTH])  # round the post's front to its right side, then straight into the mouth


def _route_lines() -> list[Box]:
    out = []
    for i, (a, b) in enumerate(zip(ROUTE[:-1], ROUTE[1:])):
        c, d = (a + b) / 2, b - a
        n = float(np.linalg.norm(d))
        out.append(Box(f"route_{i}", pos=(float(c[0]), float(c[1]), BT + INK_T / 2), size=(n + LINE_W, LINE_W, INK_T),
                       yaw=float(math.atan2(d[1], d[0])), color=BLUE))
    return out


# -- start-connector fixture and wire -----------------------------------------------------------------------------------
FIX_SIZE = (0.04, 0.03, 0.025)
FIXTURE = Box("start_fixture", pos=(float(START[0]), float(START[1] + 0.006 + FIX_SIZE[1] / 2), BT + FIX_SIZE[2] / 2),
              size=FIX_SIZE, color=(0.45, 0.47, 0.50))
R = 0.0035  # 7 mm wire
WIRE_LEN = 0.95
PL, PW, PH = 0.055, 0.016, 0.012  # plug: length along the wire, width, height
PLUG = Plug(size=(PL, PW, PH), mass=0.015, color=(0.06, 0.06, 0.06))
WIRE_YAW = math.radians(-88.0)
WIRE = cable("wire", start=(float(START[0]), float(START[1]), BT + R + 0.0005), yaw=WIRE_YAW, length=WIRE_LEN,
             radius=R, anchor_start=True, end_plug=PLUG, color=(0.95, 0.45, 0.08), bend_stiffness=0.005,
             bend_damping=0.001, friction=0.4)  # 2 cm segments; PVC insulation on smooth steel pins / laminate
PLUG_START = START + (WIRE_LEN + PL / 2) * np.array([math.cos(WIRE_YAW), math.sin(WIRE_YAW)])  # nominal plug centre

# -- mating connector: fixed block just past the post's right side, cavity 22 x 15 mm, 25 mm deep, opening towards -x
CAV_W, CAV_H, CAV_D = PW + 0.006, PH + 0.003, 0.025
MOD_D, MOD_W, MOD_H = 0.06, 0.07, 0.03
DARK = (0.10, 0.10, 0.11)


def _connector() -> list[Box]:
    x0, y0 = MOUTH

    def box(tag, u0, u1, v0, v1, z0, z1):
        return Box(f"connector_{tag}", pos=(float(x0 + (u0 + u1) / 2), float(y0 + (v0 + v1) / 2), BT + (z0 + z1) / 2),
                   size=(u1 - u0, v1 - v0, z1 - z0), color=DARK)

    hw, cw = MOD_W / 2, CAV_W / 2
    return [box("back", CAV_D, MOD_D, -hw, hw, 0.0, MOD_H), box("roof", 0.0, CAV_D, -hw, hw, CAV_H, MOD_H),
            box("wall_l", 0.0, CAV_D, cw, hw, 0.0, CAV_H), box("wall_r", 0.0, CAV_D, -hw, -cw, 0.0, CAV_H)]


OBJECTS = [SHEET, *_route_lines(), *PIN_OBJS, POST_OBJ, FIXTURE, WIRE, *_connector()]

# -- scoring ----------------------------------------------------------------------------------------------------------
SEATED = 0.012  # plug tip at least 12 mm inside the cavity
LAT_MAX = 0.006  # plug centre within 6 mm of the cavity axis sideways
NEAR = 0.04  # plug centre within 4 cm of the mouth, in front of it
LOW_Z = BT + 0.025  # every wire point at most 25 mm above the board (pins are 40 mm tall)
RELEASE_DIST = 0.03  # both gripper TCPs at least 3 cm from every wire point and from the plug centre
GATE = np.array([0.035] * len(ROW_Y) + [0.05])  # side-gate length from each pin centre
PIN_STEP = 0.1  # credit per pin passed on the drawn side, in route order
_ENV: dict = {"env": None}


def _install_hooks() -> None:
    """Keep a handle on the env so score() can see where the grippers are."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_pins_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._pins_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[np.ndarray] | None:
    """World positions of both grippers' TCP; None if they cannot be read (then nothing counts as released)."""
    env = _ENV["env"]
    if env is None:
        return None
    try:
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
        return out if all(np.isfinite(g).all() for g in out) else None
    except Exception:  # noqa: BLE001
        return None


def plug_pose(objects) -> tuple[np.ndarray, np.ndarray]:
    """(plug centre, unit axis pointing out through its front end)."""
    st = objects["wire"]
    p = np.asarray(st.end_plug if st.end_plug is not None else np.full(3, np.nan), dtype=float)
    a = p - np.asarray(st.points[-1], dtype=float)
    return p, a / (np.linalg.norm(a) + 1e-12)


def tip_depth(objects) -> float:
    """How far the plug's front end reaches past the cavity mouth (m); -1 if the plug is not in line with the cavity
    (centre more than 6 mm off its axis sideways, lifted above it, or pointing more than 30 deg off the axis)."""
    p, a = plug_pose(objects)
    if not np.isfinite(p).all():
        return -1.0
    if abs(p[1] - MOUTH[1]) > LAT_MAX or p[2] > BT + CAV_H + 0.003 or a[0] < math.cos(math.radians(30)):
        return -1.0
    return float(p[0] + PL / 2 - MOUTH[0])


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def wire_xy(objects) -> np.ndarray:
    """The wire in top view: start fixture exit, segment centres, end tip, plug centre."""
    st = objects["wire"]
    p, _a = plug_pose(objects)
    return np.vstack([st.polyline()[:, :2], p[None, :2]])


def _crossings(path: np.ndarray, a: np.ndarray, b: np.ndarray, travel: np.ndarray) -> int:
    """Signed number of times ``path`` crosses segment a-b (+1 when crossing along ``travel``)."""
    p, q = path[:-1], path[1:]
    d, e = q - p, b - a

    def cross(u, v):
        return u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]

    den = cross(d, e)
    ok = np.abs(den) > 1e-12
    t = np.where(ok, cross(a - p, e) / np.where(ok, den, 1.0), -1.0)
    s = np.where(ok, cross(a - p, d) / np.where(ok, den, 1.0), -1.0)
    hit = ok & (t >= 0.0) & (t < 1.0) & (s >= 0.0) & (s <= 1.0)
    return int(np.sign(d[hit] @ travel).sum())


def pin_sides(objects) -> list[int]:
    """Per route pin: +1 the wire passes it on the drawn side (crosses the drawn-side gate once, along the route,
    and never the opposite gate), -1 on the wrong side (crosses the opposite gate, or the drawn-side gate backwards
    or twice), 0 not passed yet. Gates run from the pin centre 3.5 cm (post: 5 cm) to either side of the line."""
    w = wire_xy(objects)
    out = []
    for c, s, t, g in zip(PINS, SIDE, TRAVEL, GATE):
        good = _crossings(w, c, c + s * g, t)
        bad = _crossings(w, c, c - s * g, t)
        out.append(-1 if (bad != 0 or good not in (0, 1)) else int(good))
    return out


def windings(objects) -> np.ndarray:
    """Winding number around each route pin of the closed loop: wire (start fixture -> plug centre), straight on to
    the connector mouth, then back along the printed line to the start. 0 = the wire passes that pin on the same side
    as the printed line. Only meaningful once the plug is at the connector (the closing piece is then short)."""
    loop = np.vstack([START[None], wire_xy(objects), MOUTH[None], ROUTE[::-1][1:]])
    w = []
    for c in PINS:
        v = loop - c
        ang = np.arctan2(v[:, 1], v[:, 0])
        d = np.diff(ang)
        d = (d + math.pi) % (2 * math.pi) - math.pi
        w.append(d.sum() / (2 * math.pi))
    return np.round(np.array(w), 3)


def released(objects) -> bool:
    gs = grippers()
    if not gs:
        return False
    st = objects["wire"]
    p, _a = plug_pose(objects)
    pts = np.vstack([_dense(st.polyline()), p[None]])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in gs)


def analyse(objects) -> dict:
    st = objects["wire"]
    p, _a = plug_pose(objects)
    if not (np.isfinite(st.points).all() and np.isfinite(p).all()):  # simulation blew up: no credit
        return {"over": list(range(len(PINS))), "sides": [-1] * len(PINS), "wind": [1.0] * len(PINS), "k": 0,
                "depth": -1.0, "near": False, "released": False, "low": False}
    pts = _dense(st.polyline())
    over = [k for k, c in enumerate(PINS) if (np.linalg.norm(pts[:, :2] - c, axis=1) < RADII[k]).any()]
    sides = pin_sides(objects)
    dep = tip_depth(objects)
    near = bool(np.linalg.norm(p[:2] - MOUTH) <= NEAR and p[0] <= MOUTH[0] + PL / 2)
    wind = windings(objects)
    if near or dep >= 0.0:  # plug at the connector: the closed-loop test also applies to every pin
        sides = [-1 if abs(wind[k]) > 0.5 else s for k, s in enumerate(sides)]
    k = 0
    while k < len(PINS) and sides[k] == 1:
        k += 1
    low = bool(st.points[:, 2].max() <= LOW_Z)
    return {"over": over, "sides": sides, "wind": wind.tolist(), "k": k, "depth": dep, "near": near,
            "released": released(objects), "low": low}


def score(objects) -> float:
    """0 if: the wire passes any pin or the post on the other side from the printed line (or lies over one), any part
    of the wire is more than 25 mm above the board, or a gripper is within 3 cm of the wire or plug. Otherwise
    0.1 per pin passed on the drawn side, counted in route order from the start fixture up to the first pin not yet
    passed (pin 1..5, then the post: up to 0.6); with all six: plug lying within 4 cm in front of the mouth 0.7, plug
    tip inside the cavity 0.8, tip >= 12 mm into the cavity 1.0."""
    r = analyse(objects)
    if r["over"] or (-1 in r["sides"]) or not r["low"] or not r["released"]:
        return 0.0
    if r["k"] < len(PINS):
        return round(PIN_STEP * r["k"], 3)
    if r["depth"] >= SEATED:
        return 1.0
    if r["depth"] >= 0.0:
        return 0.8
    if r["near"]:
        return 0.7
    return 0.6


def success(objects) -> bool:
    """Full score, wire at rest, and both grippers moved at least 10 cm clear of the plug (so the last frames show the
    finished routing unobstructed)."""
    p, _a = plug_pose(objects)
    gs = grippers()
    clear = bool(gs) and all(float(np.linalg.norm(p - g)) >= 0.10 for g in gs)
    return bool(score(objects) >= 0.999 and np.linalg.norm(objects["wire"].lin_vel) < 0.02 and clear)


def distance(objects) -> float:
    """Plug centre to its seated position (m); 1 m while the wire passes a pin on the wrong side."""
    r = analyse(objects)
    if r["over"] or -1 in r["sides"]:
        return 1.0
    p, _a = plug_pose(objects)
    seat = np.array([MOUTH[0] + 0.015 - PL / 2, MOUTH[1]])
    return float(np.linalg.norm(p[:2] - seat)) if np.isfinite(p).all() else 1.0


def progress(objects) -> dict:
    """Stage diagnostic (not the score): fraction of route pins passed on the drawn side in order, any pin on the
    wrong side, wire on the board, plug near / in the cavity (0..1 up to 12 mm), grippers released."""
    r = analyse(objects)
    return {"pins_in_order": r["k"] / len(PINS), "pin_wrong_side": float(-1 in r["sides"] or bool(r["over"])),
            "wire_on_board": float(r["low"]), "plug_near_mouth": float(r["near"] or r["depth"] >= 0.0),
            "plug_seated": float(np.clip(r["depth"] / SEATED, 0.0, 1.0)) if r["depth"] > -1 else 0.0,
            "released": float(r["released"])}


# Seeded reset variation for a later runner (the demo runs on the nominal layout): the wire's spawn heading varies by
# +-3 deg (it stays clear of the pins by >= 4 cm); the pins, post, printed line and connector are one fixed board and
# do not move relative to each other.
VARIATION = {"cable:wire": {"dx": 0.0, "dy": 0.0, "yaw": 0.05}}


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
# Camera-only: the demo's motions use the top camera image (vision.py: the black plug's blob and axis), the arms' own
# joint state and the fixed board layout (pins, post, line, connector); it never reads ctx.states().
# Right arm: picks the plug at the front-right and lays it down in front of the row near the middle (the left arm
# cannot reach the plug's start, the right arm cannot reach the left half of the row). Left arm: picks it up, leads it
# round the left of pin 1, behind pin 1, forward through the gap, in front of pin 2, back through the next gap and
# lays it down behind pin 3 and parks. Right arm: picks the plug up again and leads it in front of pin 4, behind
# pin 5 and round the front and right side of the turn post. The right arm
# lays the plug down in front of the connector, looks at it again, re-grips it square and pushes it in, then both let
# go and park.

FINGER_PINCH = -0.022
GRASP_BACK = 0.022  # pinch this far from the plug's wire end for the push into the connector
LEAD_DUR = 1.8  # s per lead waypoint (slow, so the trailing wire follows)
LEAD_BACK = 0.024  # ... and here (mid-plug) while leading it: the wire leaving the plug then cannot pry it out of the fingers
GZ = BT + PH + 0.002  # TCP height for pinching / leading the plug
CARRY = BT + 0.030
INSERT_TO = 0.016
PARK = {"left": (0.12, 0.42, TOP + 0.12), "right": (0.12, -0.42, TOP + 0.12)}
LEAD_YAW = 0.0  # plug heading (pointing away from the robot) while it is led through the board
H1_YAW = math.pi / 3  # plug heading when laid at H1: pointing back-left, its wire trailing back to the right
DARKV = 100  # top-camera brightness below which a pixel belongs to a black part (steel pins and shadows stay above ~110)
H1 = np.array([0.24, 0.04])  # right arm lays the plug here for the left arm (between the wire's lay and the row)
H2 = np.array([ROW_X - 0.05, -0.06])  # left arm lays the plug here (in front of the pin 3-4 gap) for the right arm
H2_YAW = -60 * math.pi / 180  # plug heading there (the right arm cannot grip it square to the y axis so near the middle)
H2_SWEEP = np.array([ROW_X + 0.05, -0.045])  # (test variant 'sweep' only: laid behind the gap instead)
_D = math.pi / 180
# plug centre (x, y) and heading along the way: the plug points where it goes, so its wire trails behind it
FAR, NEAR_X = ROW_X + 0.04, ROW_X - 0.04  # plug centre beside a pin: the fingers stay >= 1 cm off it, little slack used
def _side(x, y, h_in, h_out):
    """Plug past a pin's side at (x, y): turning from the heading it came in on to the one it leaves on."""
    return [(x, y + 0.01, h_in), (x, y, (h_in + h_out) / 2), (x, y - 0.01, h_out)]


_BF, _FB = -138 * _D, -42 * _D  # diagonal headings: back -> front of the row (pin 1 -> 2, 3 -> 4) and front -> back
LEFT_PATH = [(0.24, 0.13, 60 * _D), (0.24, 0.22, 75 * _D), (0.27, 0.28, 45 * _D), (0.34, 0.25, -45 * _D),
             (FAR, 0.20, -75 * _D), *_side(FAR, 0.18, -90 * _D, -120 * _D), (ROW_X, 0.135, _BF),
             *_side(NEAR_X, 0.09, -120 * _D, -60 * _D), (ROW_X, 0.045, _FB),
             *_side(FAR, 0.0, -60 * _D, -120 * _D), (ROW_X, -0.045, _BF),
             (H2[0], H2[1], -100 * _D), (H2[0], H2[1], H2_YAW)]  # gaps crossed on the diagonal, the plug along it
RIGHT_PATH = [(NEAR_X, -0.09, -75 * _D), (NEAR_X, -0.10, -60 * _D), (ROW_X, -0.135, _FB),
              *_side(FAR, -0.18, -60 * _D, -90 * _D),
              (0.30, -0.215, -100 * _D), (0.255, -0.25, -100 * _D), (0.255, -0.29, -90 * _D),  # in front of the post
              (0.27, -0.325, -45 * _D), (0.31, -0.34, 0.0)]  # round the post's front to its right side, close in
PRE = np.array([MOUTH[0] - PL / 2 - 0.02, MOUTH[1]])  # plug centre laid down in front of the mouth
VAR = os.environ.get("PINS_DEMO", "correct")
DBG = bool(os.environ.get("PINS_DBG", ""))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _log(ctx, tag=""):
    """Progress line from the demo's own view: time and both measured TCPs (no simulator object state)."""
    try:
        tl, tr = meas_tcp(ctx, "left"), meas_tcp(ctx, "right")
        print(f"[t={ctx.t:5.2f}] {tag} tcp_left={np.round(tl, 3).tolist()} tcp_right={np.round(tr, 3).tolist()}", flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"[log] {tag} error {e!r}", flush=True)


def _tcp_for(c, yaw: float, back: float = LEAD_BACK) -> np.ndarray:
    """TCP xy that puts the plug centre at ``c`` when the plug points along ``yaw`` (pinched ``back`` from its rear)."""
    u = np.array([math.cos(yaw), math.sin(yaw)])
    return np.asarray(c, dtype=float) - u * (PL / 2 - back)


def see_plug(ctx, guess, heading, win=0.045, xmax=MOUTH[0] - 0.012):
    """Black plug seen by the top camera near ``guess``: (centre xy, heading) with the heading's sign taken from the
    expected ``heading``; (None, None) when the image shows no plug there (or is too small to tell)."""
    import vision

    img = vision.top_image(ctx)
    if img.shape[0] < 100:  # --no-record debug frames (64x36) carry no usable detail
        return None, None
    m = img.max(axis=-1) < DARKV  # the plug's top face is 3-4 px wide: no erosion
    if m.sum() < 3:
        return None, None
    pts = vision.top_camera(img.shape).to_plane(m, BT + PH)
    sel = pts[(np.linalg.norm(pts - np.asarray(guess), axis=1) < win) & (pts[:, 0] < xmax)]
    if len(sel) < 4:
        return None, None
    c = sel.mean(axis=0)
    _w, V = np.linalg.eigh(np.cov((sel - c).T))
    ax = V[:, 1]
    h = math.atan2(ax[1], ax[0])
    if abs(_wrap(h - heading)) > math.pi / 2:
        h = _wrap(h + math.pi)
    return c, h


def _look(ctx, arm, guess, heading, tag):
    """Up to three looks at the plug (the arms are out of the camera's way); falls back to ``guess``."""
    for _ in range(3):
        yield from arm.wait(0.3)
        c, h = see_plug(ctx, guess, heading)
        if c is not None:
            break
    if DBG:
        print(f"[see] {tag}: seen {None if c is None else np.round(c, 4).tolist()} head "
              f"{None if h is None else round(math.degrees(h), 1)}", flush=True)
    if c is None:
        c, h = np.asarray(guess, dtype=float), heading
    return c, h


def _pick(arm, c, h, dur=1.4, back=LEAD_BACK):
    from yam_arena.constants import FINGER_CLOSED

    g = _tcp_for(c, h, back)
    yield from arm.move((g[0], g[1], CARRY + 0.03), yaw=_gy(h), finger=FINGER_PINCH, duration=dur)
    yield from arm.move((g[0], g[1], GZ), yaw=_gy(h), duration=0.8)
    yield from arm.grip(FINGER_CLOSED, 0.5)


def _gy(h: float) -> float:
    """Gripper yaw for a held plug heading ``h`` (the demo always grips with the fingers' zero along the plug)."""
    return float(_wrap(h))


def _lead(arm, centres, yaw=0.0, z=GZ, dur=1.0):
    """Move the held plug's centre through ``centres``: (x, y) at heading ``yaw``, or (x, y, heading)."""
    for c in centres:
        h = c[2] if len(c) > 2 else yaw
        t = _tcp_for(np.asarray(c[:2]), h)
        yield from arm.move((t[0], t[1], z), yaw=_gy(h), duration=dur)


def _let_go(arm, yaw, back=(0.0, 0.0), up=0.05):
    yield from arm.grip(FINGER_PINCH, 0.5)
    t = arm.tcp()
    yield from arm.move((t[0] + back[0], t[1] + back[1], CARRY + up), yaw=_gy(yaw), duration=0.7)


def _park(arm, side, dur=1.2):
    yield from arm.move(PARK[side], yaw=0.0, duration=dur)


def meas_tcp(ctx, side):
    """TCP from the measured joint angles (robot state), not the command."""
    from yam_arena import ik

    q = np.asarray(ctx.obs.state["joint_pos"], dtype=float).reshape(-1)
    arm = ctx.left if side == "left" else ctx.right
    qa = q[0:6] if side == "left" else q[7:13]
    return ik.tcp_pose(qa, arm.base)[0]


def _insert(ctx, arm):
    """Plug lying in front of the mouth: look at it, re-grip it square (yaw 0) on the cavity axis, push it in; if the
    measured TCP stalls short of the push (plug tip on the rim), back off and retry a little to the side."""
    from yam_arena.constants import FINGER_CLOSED

    c, h = yield from _look(ctx, arm, PRE, 0.0, "pre-insert")
    yield from _pick(arm, c, h, dur=1.2, back=GRASP_BACK)
    t = arm.tcp()
    yield from arm.move((t[0], t[1], GZ + 0.002), yaw=_gy(h), duration=0.4)
    # turn square in place: the plug turns about the TCP
    yield from arm.move((t[0], t[1], GZ + 0.002), yaw=0.0, duration=0.8)
    base = np.array([MOUTH[0] - PL / 2 - 0.012 - (PL / 2 - GRASP_BACK), MOUTH[1]])  # TCP: tip 12 mm short of mouth
    yield from arm.move((base[0], base[1], GZ), yaw=0.0, duration=0.8)
    full = MOUTH[0] + INSERT_TO - PL + GRASP_BACK  # TCP x with the tip INSERT_TO inside
    inside = MOUTH[0] + 0.002 - PL + GRASP_BACK  # TCP x with the tip 2 mm past the mouth
    for k, dy in enumerate((0.0, 0.003, -0.003, 0.006, -0.006, 0.009, -0.009)):
        yield from arm.move((base[0], base[1] + dy, GZ), yaw=0.0, duration=0.4)
        yield from arm.move((full + 0.012, base[1] + dy, GZ), yaw=0.0, duration=1.4)
        yield from arm.wait(0.3)
        mx = meas_tcp(ctx, "right")[0]
        if DBG:
            _log(ctx, f"push {k} dy={dy * 1000:.0f}mm measured TCP x {mx:.4f} (target {full:.4f})")
        if mx >= full - 0.003:
            break
        if mx > inside:  # tip in but the wire drags: re-grip 12 mm further back (fingers clear of the face), push firmly
            y = base[1] + dy
            yield from arm.grip(FINGER_PINCH, 0.4)
            yield from arm.move((mx - 0.012, y, GZ), yaw=0.0, duration=0.5)
            yield from arm.grip(FINGER_CLOSED, 0.4)
            yield from arm.move((full - 0.012 + 0.040, y, GZ), yaw=0.0, duration=2.0)  # overshoot: push firmly
            yield from arm.wait(0.5)
            if DBG:
                _log(ctx, f"push {k} re-gripped: measured TCP x {meas_tcp(ctx, 'right')[0]:.4f} (target {full - 0.012:.4f})")
            break
    yield from arm.grip(FINGER_CLOSED, 0.2)


def demo_script(ctx):
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if VAR == "nothing":
        for _ in range(4):
            yield from L.wait(1.0)
        _log(ctx, "end")
        return

    # 1) right arm: plug from the front-right to the middle of the board, in front of the row
    yield from Rt.move((0.10, -0.30, TOP + 0.08), yaw=0.0, duration=1.0)  # out of the camera's view of the plug
    c, h = yield from _look(ctx, Rt, PLUG_START, WIRE_YAW, "start")
    yield from _pick(Rt, c, h)
    yield from Rt.move(tuple(_tcp_for(c, h)) + (CARRY,), yaw=_gy(h), duration=0.5)

    if VAR in ("shortest", "onearm"):
        if VAR == "shortest":  # straight to the connector, in front of the whole row and the post
            way = [(0.22, -0.35), (0.30, -0.36)]
        else:  # one arm: the right half of the drawn path only (pin 4, pin 5, the post)
            way = [(0.22, -0.30), (0.22, -0.045, -60 * _D), *RIGHT_PATH]
        yield from _lead(Rt, [way[0]], h, z=CARRY, dur=1.4)
        yield from Rt.move(tuple(_tcp_for(way[0], LEAD_YAW)) + (CARRY,), yaw=LEAD_YAW, duration=0.8)
        yield from _lead(Rt, way[1:], LEAD_YAW, z=GZ, dur=LEAD_DUR)
        yield from _lead(Rt, [PRE], 0.0, z=GZ, dur=1.0)
        yield from _let_go(Rt, 0.0, back=(-0.05, -0.08))
        yield from _park(Rt, "right")
        yield from _insert(ctx, Rt)
        yield from _let_go(Rt, 0.0, back=(-0.04, 0.0))
        yield from _park(Rt, "right")
        for _ in range(3):
            yield from L.wait(1.0)
        _log(ctx, "end")
        return

    # carry it back along the gap between the wire's lay and the row, so the doubled-back wire lies behind the lay
    # (the left arm can then lead the plug round pin 1 without crossing the wire over itself)
    yield from Rt.move(tuple(_tcp_for((0.24, -0.55), h)) + (CARRY,), yaw=_gy(h), duration=1.0)
    yield from Rt.move(tuple(_tcp_for((0.24, -0.55), H1_YAW)) + (CARRY,), yaw=_gy(H1_YAW), duration=1.2)
    yield from Rt.move(tuple(_tcp_for((0.24, -0.25), H1_YAW)) + (CARRY,), yaw=_gy(H1_YAW), duration=1.5)
    yield from Rt.move(tuple(_tcp_for(H1, H1_YAW)) + (CARRY,), yaw=_gy(H1_YAW), duration=1.0)
    yield from _lead(Rt, [H1], H1_YAW, dur=0.5)
    yield from _let_go(Rt, H1_YAW, back=(-0.03, -0.06))
    yield from _park(Rt, "right")
    _log(ctx, "plug at H1")

    # 2) left arm: round pin 1 and through the left half of the slalom
    c, h = yield from _look(ctx, L, H1, H1_YAW, "H1")
    yield from _pick(L, c, h)
    yield from _lead(L, LEFT_PATH[:3], h, dur=LEAD_DUR)  # round the left of pin 1 to behind it, then turn the plug
    if VAR == "sweep":  # one sweep behind the whole row (never weaving), then round the post and in
        yield from _lead(L, LEFT_PATH[2:4] + [(FAR, 0.09, -90 * _D), (FAR, 0.0, -90 * _D), (*H2_SWEEP, H2_YAW)], dur=LEAD_DUR)
    else:
        yield from _lead(L, LEFT_PATH[2:], dur=LEAD_DUR)
    _log(ctx, "left half led")
    yield from _let_go(L, H2_YAW, back=(-0.02, 0.04))
    if VAR == "half":
        yield from _park(L, "left")
        for _ in range(3):
            yield from L.wait(1.0)
        _log(ctx, "end")
        return
    yield from _park(L, "left")

    # 3) right arm: in front of pin 4, behind pin 5, round the front and right side of the post
    c, h = yield from _look(ctx, Rt, H2_SWEEP if VAR == "sweep" else H2, H2_YAW, "H2")
    yield from _pick(Rt, c, h)
    if VAR == "sweep":
        yield from _lead(Rt, [(FAR, -0.09, -75 * _D), (FAR, -0.18, -90 * _D)] + RIGHT_PATH[4:], dur=LEAD_DUR)
    else:
        yield from _lead(Rt, RIGHT_PATH, dur=LEAD_DUR)
    _log(ctx, "right half led")
    yield from _lead(Rt, [PRE], 0.0, dur=1.2)
    yield from _let_go(Rt, 0.0, back=(-0.05, -0.08))
    yield from _park(Rt, "right")
    _log(ctx, "plug laid at the connector")
    if VAR == "noplug":
        for _ in range(3):
            yield from L.wait(1.0)
        _log(ctx, "end")
        return

    # 4) look again, re-grip square, push in, let go, park
    yield from _insert(ctx, Rt)
    _log(ctx, "inserted")
    if VAR == "hold":
        yield from L.wait(2.0)
        _log(ctx, "end (holding)")
        return
    yield from _let_go(Rt, 0.0, back=(-0.04, 0.0))
    yield from _park(Rt, "right")
    for _ in range(2):
        yield from L.wait(1.0)
    _log(ctx, "end")
