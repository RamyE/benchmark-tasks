"""Swap a failed QSFP28 optic in a top-of-rack switch: unplug its fibre first by the push-pull tab, keep the ferrule end
face clear of every surface, swing the bail open and extract the module, seat the spare until it latches, and reconnect.

Scene (metres from the arm-base midpoint, x forward, y left; heights above the table). An unpowered 1U 32-port QSFP28
switch (43.8 x 26.0 x 4.36 cm, NVIDIA/Mellanox MSN2700 form factor) is mounted in the top unit (U2) of a two-post 19-inch open-frame rack
standing on the table: U1 below it holds a 1U LC fibre patch panel with eight live patch cords, and a vertical finger cable manager stands beside each post. Faceplate at x = 0.370 facing the robot
(-x), upper port row centred at z = TOP + 0.0753, horizontal port pitch 21.5 mm, cage opening 20.4 x 10.5 mm and
insertion depth 45.0 mm (SFF-8663). Port 5 (y = 0) holds a failed 100GBASE-LR4 QSFP28 transceiver (18.35 x 8.5 x 89.4
mm, Finisar FTLC1154RDPL / SFF-8661, red wire bail closed upright at 0 deg) latched in its cage. Plugged into its duplex
LC receptacle is a blue LC uniboot connector with a sliding push-pull release tab on a 31 cm yellow single-mode patch
cord routed down onto the table and taped at (0.10, 0.06). Ports 1 (y = +0.043) and 9 (y = -0.043) carry live modules
whose 4 mm cords run outward into the side managers; the remaining ports hold black QSFP dust plugs. An antistatic ESD
tray (14 x 10 x 0.8 cm inner) sits at (0.235, -0.190) holding the spare module (silver wire bail closed upright) at
(0.235, -0.213), pointing +x.

Simulation mechanisms (documented in README and task.json fidelity):
- Grasp is the shared force-limited pad grip (grip_limited.LimitedGrip, no rigid weld): it engages only on PhysX contact
  on both finger pads while the jaws squeeze on a part's grip band, is capped at 50 N pull / 0.5 N m twist / 1.5 N m tip
  (the part slips above a cap), and releases when the jaws open.
- The LC push-pull release tab rides a spring-returned prismatic joint (-4.5 to 0 mm, 120 N/m) along the LC body; the
  LC latch releases only when the tab is pulled back >= 2.5 mm relative to the LC body, and re-latches when the LC is
  pushed home into a receptacle with the tab at rest. The cord attaches to the inner LC body, so pulling the cord or
  gripping the front body without pulling the tab does not unlatch it.
- Each module's wire bail turns on a revolute hinge (0 to -90 deg) with an over-centre spring; while the LC is plugged
  in, the bail crossbar strikes the LC latch housing near -69 deg, short of the -80 deg cage-unlatch angle.
- Live neighbour modules (ports 1 and 9) are held by welds that break above 15 N or 0.6 N m.
- End-face contamination is checked geometrically every control step across 9 points on the LC's front ferrule face
  against all scene surfaces, modules, bails, cord links, and all collision boxes of both robot grippers.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

import grip_limited as gl  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = (
    "The optic in port 5 has failed. Replace it with the spare from the tray, put the failed one in the tray, "
    "and reconnect its fibre."
)
RUN_LIMIT_STEPS = 1800  # 90 s at 20 Hz (step count of the scripted demo: see README)

# -- colours ---------------------------------------------------------------------------------------------------------
CHASSIS = (0.16, 0.17, 0.19)
BEZEL_C = (0.26, 0.27, 0.30)
STEEL = (0.70, 0.71, 0.73)
MOD_C = (0.78, 0.79, 0.81)
BAIL_C = (0.86, 0.87, 0.89)
RED = (0.82, 0.10, 0.08)
AMBER = (0.95, 0.55, 0.05)
GREEN = (0.12, 0.80, 0.22)
YELLOW = (0.95, 0.80, 0.10)
LCBODY = (0.18, 0.44, 0.86)
LCTAB_C = (0.10, 0.30, 0.76)
WHITE = (0.96, 0.96, 0.96)
PLUG_C = (0.06, 0.06, 0.07)
TAPE_C = (0.10, 0.10, 0.11)
TRAY_C = (0.56, 0.12, 0.14)
SHELF_C = (0.36, 0.37, 0.39)

# -- switch geometry (1U 32-port QSFP28, SFF-8663 45 mm cage depth) ---------------------------------------------------
FP = 0.370
RU = 0.04445  # EIA-310 rack unit (1.75 in)
PZ = TOP + RU + 0.0436 - 0.01275  # switch sits in the top unit (U2) of a 2U two-post rack, above a 1U fibre patch panel in U1
PITCH = 0.0215
SW_W, SW_D = 0.438, 0.260
SW_TOP = PZ + 0.01275
SW_BOT = SW_TOP - 0.0436
CW, CH, CD = 0.0204, 0.0105, 0.0450
PORT_Y = {1: 2 * PITCH, 3: PITCH, 5: 0.0, 7: -PITCH, 9: -2 * PITCH}
LIVE = (1, 9)

# -- QSFP28 module (SFF-8661 / Finisar FTLC1154RDPL: 18.35 x 8.5 x 89.4 mm, 35 g) ------------------------------------
MW, MH, ML = 0.01835, 0.0085, 0.0894
RD, RW, RH = 0.0120, 0.0120, 0.0070  # duplex LC receptacle: 12 mm deep, 12.0 x 7.0 mm opening (0.5 mm clearance/side)
M_FRONT = (ML - RD) / 2  # +0.0387 m from solid-body centre
M_REAR = -(ML - RD) / 2 - RD  # -0.0507 m from solid-body centre
MOD_MASS = 0.035
SEAT_X = FP + CD - 0.0005  # 0.4145 m: module front face when seated against cage back stop
MOD0_C = (SEAT_X - M_FRONT, 0.0, PZ)  # (0.3758, 0.0, PZ); rear face at x = 0.3251 m (44.9 mm proud of faceplate)
REAR0 = MOD0_C[0] + M_REAR

# wire bail: hinged at module sides 1.5 mm inside rear face; 0 deg = closed upright, -90 deg = open horizontal (-x)
BAIL_PIV = M_REAR + 0.0015
BAIL_ARM = 0.0130
BAIL_Y = MW / 2 + 0.00115
BAIL_T = 0.0015
BAIL_MASS = 0.002
BAIL_K = float(os.environ.get("QS_BAIL_K", "0.0006"))
UNLATCH_DEG = 80.0

# -- Duplex LC uniboot connector + sliding push-pull release tab ------------------------------------------------------
LC_BODY = (0.020, 0.0124, 0.0068)
LC_HOUS = (0.012, 0.0110, 0.0060)  # TIA-604-10 duplex front shroud (two 1.25 mm ferrules at 6.25 mm pitch)
LC_MASS = 0.005
LC_FRONT = LC_BODY[0] / 2 + LC_HOUS[0]  # +0.0220 m
LC_SEAT_REL = M_REAR + RD - LC_FRONT - 0.0003  # -0.0610 m in module frame when fully home
LC0_X = MOD0_C[0] + LC_SEAT_REL  # 0.3148 m

TAB_SIZE = (0.026, 0.0124, 0.0074)
TAB_MASS = 0.005
TAB_REL_X = -LC_BODY[0] / 2 - TAB_SIZE[0] / 2 - 0.0005  # -0.0235 m relative to LC centre at rest
TAB_MAX_PULL = 0.0045  # 4.5 mm travel along -x
TAB_UNLATCH = 0.0025  # >= 2.5 mm pull releases the LC latch
TAB_K, TAB_D = 120.0, 3.0  # N/m, N s/m return spring on push-pull tab
TAB0_X = LC0_X + TAB_REL_X  # 0.2913 m

# -- 30 cm yellow fibre patch cord (15 capsule links, ball-jointed, taped on bench at (0.10, 0.06)) -------------------
CORD_R = 0.0020
CORD_KGM = 0.045
CORD_BEND_K, CORD_BEND_D = 0.0025, 0.0006
N_CORD = 15
TAPE_XY = (0.10, 0.06)
CORD_ATTACH_LC_X = TAB_REL_X - TAB_SIZE[0] / 2 - 0.002  # -0.0385 m in LC frame (exits rear of pull-tab boot)
CORD_PATH = [
    np.array([LC0_X + CORD_ATTACH_LC_X, 0.0, PZ - TOP]),
    np.array([0.240, 0.025, 0.018]),
    np.array([0.226, 0.106, CORD_R + 0.001]),
    np.array([0.155, 0.126, CORD_R + 0.001]),
    np.array([TAPE_XY[0], TAPE_XY[1], CORD_R + 0.001]),
]


def _interp_chain(waypoints: list[np.ndarray], n: int) -> list[np.ndarray]:
    segs = [float(np.linalg.norm(b - a)) for a, b in zip(waypoints[:-1], waypoints[1:])]
    tot = sum(segs)
    pts = []
    for i in range(n + 1):
        s = tot * i / n
        for k, L in enumerate(segs):
            if s <= L + 1e-9 or k == len(segs) - 1:
                t = min(1.0, s / max(L, 1e-9))
                pts.append(waypoints[k] + t * (waypoints[k + 1] - waypoints[k]))
                break
            s -= L
    return pts


def _x_quat_xyzw(a: np.ndarray, b: np.ndarray) -> tuple[float, float, float, float]:
    d = (b - a) / max(float(np.linalg.norm(b - a)), 1e-12)
    x = np.array([1.0, 0.0, 0.0])
    c = float(np.dot(x, d))
    ax = np.cross(x, d)
    s = float(np.linalg.norm(ax))
    if s < 1e-9:
        return (0.0, 0.0, 0.0, 1.0) if c > 0 else (0.0, 0.0, 1.0, 0.0)
    ax /= s
    ang = math.atan2(s, c)
    return (*(ax * math.sin(ang / 2)).tolist(), math.cos(ang / 2))


CORD_PTS = _interp_chain(CORD_PATH, N_CORD)
CORD_LEN = sum(float(np.linalg.norm(b - a)) for a, b in zip(CORD_PTS[:-1], CORD_PTS[1:]))
LINK_L = CORD_LEN / N_CORD
CORD_C0 = [(CORD_PTS[i] + CORD_PTS[i + 1]) / 2 for i in range(N_CORD)]
CORD_Q0 = [_x_quat_xyzw(CORD_PTS[i], CORD_PTS[i + 1]) for i in range(N_CORD)]

# -- ESD tray (14 x 10 cm inner, 0.8 cm walls, at (0.235, -0.190)) ---------------------------------------------------
TRAY_C0 = (0.235, -0.190)
TRAY_IN = (0.140, 0.100)
TRAY_WALL, TRAY_H = 0.004, 0.008
SPARE_C = (TRAY_C0[0], TRAY_C0[1] - 0.023, TOP + 0.002 + MH / 2 + 0.0004)

# -- scene objects ----------------------------------------------------------------------------------------------------
STATIC: list[Box] = []


def _sbox(name: str, lo, hi, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    b = Box(name, pos=tuple(((lo + hi) / 2).tolist()), size=tuple((hi - lo).tolist()), color=color)
    STATIC.append(b)
    return b


X0, X1 = FP, FP + SW_D
_sbox("sw_top", (X0, -SW_W / 2, PZ + CH / 2), (X1, SW_W / 2, SW_TOP), CHASSIS)
_sbox("sw_bottom", (X0, -SW_W / 2, SW_BOT), (X1, SW_W / 2, PZ - CH / 2), CHASSIS)
SW_BACK = Box(
    "sw_back",
    pos=((X0 + CD + X1) / 2, 0.0, PZ),
    size=(X1 - (X0 + CD), SW_W, CH),
    color=CHASSIS,
    mass=4.2,
)
_cav = sorted(PORT_Y[p] for p in (1, 5, 9))
_edges = [-SW_W / 2] + [v for y in _cav for v in (y - CW / 2, y + CW / 2)] + [SW_W / 2]
for _k in range(0, len(_edges), 2):
    _sbox(f"sw_band{_k // 2}", (X0, _edges[_k], PZ - CH / 2), (X0 + CD, _edges[_k + 1], PZ + CH / 2), CHASSIS)
# 2U two-post EIA-310 open-frame rack (19 in, 450 mm rail opening): U1 1U LC fibre patch panel, U2 the switch at the top
# of the rack (top-of-rack position, open above); vertical finger cable managers on both sides take the live patch cords.
RACK_TOP = TOP + 2 * RU
POST_Y0, POST_Y1, POST_D = 0.2253, 0.2603, 0.076
_sbox("u1_patch_panel", (X0, -SW_W / 2, TOP), (X0 + 0.200, SW_W / 2, TOP + 0.0436), CHASSIS)
for _s, _nm in ((1, "l"), (-1, "r")):
    _sbox(f"rack_post_{_nm}", (X0, min(_s * POST_Y0, _s * POST_Y1), TOP), (X0 + POST_D, max(_s * POST_Y0, _s * POST_Y1),
          RACK_TOP + 0.010), STEEL)
    # vertical finger cable manager (10 cm deep fingers, 7.6 cm channel) beside each post
    _y0, _y1 = sorted((_s * POST_Y1, _s * (POST_Y1 + 0.076)))
    _sbox(f"vcm_{_nm}_back", (X0 + 0.040, _y0, TOP), (X0 + POST_D, _y1, RACK_TOP + 0.010), BEZEL_C)
    for _k, _zc in enumerate((0.008, *((_j + 0.5) * RU for _j in range(1, 2)))):
        for _e, (_a, _b) in enumerate(((POST_Y1, POST_Y1 + 0.005), (POST_Y1 + 0.071, POST_Y1 + 0.076))):
            _ya, _yb = sorted((_s * _a, _s * _b))
            _sbox(f"vcm_{_nm}_f{_k}{_e}", (FP - 0.070, _ya, TOP + _zc - 0.006), (X0 + 0.040, _yb, TOP + _zc + 0.006),
                  BEZEL_C)
for _u in (0,):  # rack ears of U1
    for _s, _nm in ((1, "l"), (-1, "r")):
        _ya, _yb = sorted((_s * SW_W / 2, _s * (SW_W / 2 + 0.022)))
        _sbox(f"ear_u{_u + 1}_{_nm}", (FP - 0.004, _ya, TOP + _u * RU), (FP, _yb, TOP + _u * RU + 0.0436), STEEL)
_sbox("rack_top_bar", (X0 + 0.040, -POST_Y1, RACK_TOP + 0.010), (X0 + POST_D, POST_Y1, RACK_TOP + 0.030), STEEL)
# live duplex LC uniboot plugs in the patch panel (U1), seated in its front adapters at z = TOP + 22 mm
PP_Z = TOP + 0.022
PP_Y = (0.085, 0.105, 0.125, 0.145, -0.085, -0.105, -0.125, -0.145)
for _i, _py in enumerate(PP_Y):
    _sbox(f"pp_lc{_i}", (FP - 0.020, _py - LC_BODY[1] / 2, PP_Z - LC_BODY[2] / 2), (FP, _py + LC_BODY[1] / 2,
          PP_Z + LC_BODY[2] / 2), LCBODY)
    _sbox(f"pp_tab{_i}", (FP - 0.0205 - 0.026, _py - TAB_SIZE[1] / 2, PP_Z - TAB_SIZE[2] / 2), (FP - 0.0205, _py
          + TAB_SIZE[1] / 2, PP_Z + TAB_SIZE[2] / 2), LCTAB_C)


def _catmull(waypoints, per: int = 4) -> np.ndarray:
    """Smooth polyline through the waypoints (centripetal-free uniform Catmull-Rom), per samples per span."""
    w = [np.asarray(v, float) for v in waypoints]
    w = [w[0]] + w + [w[-1]]
    out = [w[1]]
    for i in range(1, len(w) - 2):
        p0, p1, p2, p3 = w[i - 1], w[i], w[i + 1], w[i + 2]
        for k in range(1, per + 1):
            t = k / per
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t
                              + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
    return np.asarray(out)


# Fixed live patch cords (same 4.0 mm yellow OS2 duplex uniboot cord as port 5), dressed into the side managers:
# ports 1 and 9 leave their LC boot, drop 2.5 cm, angle back to x = 0.33 and run outward 5 cm above the table into the
# left/right manager; the eight U1 patch-panel cords turn at once and run outward at x = 0.312, 2.5-3 cm above the table.
NB_TAB_REAR = MOD0_C[0] + LC_SEAT_REL + TAB_REL_X - TAB_SIZE[0] / 2  # 0.2783 m
STATIC_CORDS: list[np.ndarray] = []
for _p in LIVE:
    _y0 = PORT_Y[_p]
    _s = 1.0 if _y0 > 0 else -1.0
    _bx = NB_TAB_REAR
    STATIC_CORDS.append(_catmull([
        (_bx - 0.0035, _y0, PZ), (_bx - 0.014, _y0, PZ - 0.004), (_bx - 0.018, _y0 + _s * 0.004, PZ - 0.016),
        (_bx - 0.008, _y0 + _s * 0.025, TOP + 0.050), (FP - 0.040, _s * 0.120, TOP + 0.050),
        (FP - 0.040, _s * 0.230, TOP + 0.048), (FP - 0.028, _s * 0.300, TOP + 0.055),
        (FP - 0.025, _s * 0.300, RACK_TOP)]))
for _k, _py in enumerate(PP_Y):
    _s = 1.0 if _py > 0 else -1.0
    _ye = _s * (0.282 + 0.0045 * (_k % 4))
    _bx = FP - 0.0205 - 0.026
    STATIC_CORDS.append(_catmull([
        (_bx - 0.002, _py, PP_Z), (_bx - 0.007, _py + _s * 0.003, PP_Z + 0.001),
        (_bx - 0.009, _py + _s * 0.015, PP_Z + 0.003), (_bx - 0.009, _s * 0.230, TOP + 0.027),
        (FP - 0.022 + 0.0045 * (_k % 4), _ye, TOP + 0.036), (FP - 0.016 + 0.0045 * (_k % 4), _ye, RACK_TOP)]))
STATIC_SEGS = np.asarray([(c[i], c[i + 1]) for c in STATIC_CORDS for i in range(len(c) - 1)])  # (n, 2, 3)


def _seg_dist(p: np.ndarray, segs: np.ndarray = STATIC_SEGS) -> float:
    a, b = segs[:, 0], segs[:, 1]
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", p - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(a + t[:, None] * ab - p, axis=1).min())
for _i in range(16):  # 32-port 2x16 QSFP28 layout: dust plugs in unused upper columns (incl. ports 3, 7) and lower row
    _col = 7 - _i  # +7 .. -8;ports 1 (+2), 3 (+1), 5 (0), 7 (-1), 9 (-2) sit near the switch centre
    _py = _col * PITCH
    if _col not in (2, 0, -2):
        _sbox(f"dust_u{_i}", (FP - 0.003, _py - MW / 2, PZ - MH / 2), (FP, _py + MW / 2, PZ + MH / 2), PLUG_C)
    _lz = PZ - CH / 2 - 0.002 - MH / 2
    _sbox(f"dust_l{_i}", (FP - 0.003, _py - MW / 2, _lz - MH / 2), (FP, _py + MW / 2, _lz + MH / 2), PLUG_C)
_sbox("rack_ear_l", (FP - 0.004, SW_W / 2, SW_BOT), (FP, SW_W / 2 + 0.022, SW_TOP), STEEL)
_sbox("rack_ear_r", (FP - 0.004, -SW_W / 2 - 0.022, SW_BOT), (FP, -SW_W / 2, SW_TOP), STEEL)
_sbox("cord_tape", (TAPE_XY[0] - 0.012, TAPE_XY[1] - 0.020, TOP), (TAPE_XY[0] + 0.012, TAPE_XY[1] + 0.020, TOP + 0.005),
      TAPE_C)

_tx, _ty = TRAY_C0
_lx, _ly = TRAY_IN[0] / 2, TRAY_IN[1] / 2
_w = TRAY_WALL
_sbox("tray_floor", (_tx - _lx - _w, _ty - _ly - _w, TOP), (_tx + _lx + _w, _ty + _ly + _w, TOP + 0.002), TRAY_C)
_sbox("tray_wall_n", (_tx - _lx - _w, _ty - _ly - _w, TOP + 0.002), (_tx - _lx, _ty + _ly + _w, TOP + TRAY_H), TRAY_C)
_sbox("tray_wall_f", (_tx + _lx, _ty - _ly - _w, TOP + 0.002), (_tx + _lx + _w, _ty + _ly + _w, TOP + TRAY_H), TRAY_C)
_sbox("tray_wall_l", (_tx - _lx, _ty + _ly, TOP + 0.002), (_tx + _lx, _ty + _ly + _w, TOP + TRAY_H), TRAY_C)
_sbox("tray_wall_r", (_tx - _lx, _ty - _ly - _w, TOP + 0.002), (_tx + _lx, _ty - _ly, TOP + TRAY_H), TRAY_C)

_MBASE = (ML - RD, MW, MH)
OLD = Box("module_failed", pos=MOD0_C, size=_MBASE, color=MOD_C, mass=MOD_MASS)
NEW = Box("module_spare", pos=(SPARE_C[0] - (M_FRONT + M_REAR) / 2, SPARE_C[1], SPARE_C[2]), size=_MBASE, color=MOD_C,
          mass=MOD_MASS)


def _bail_pos(mc):
    return (mc[0] + BAIL_PIV, mc[1], mc[2] + BAIL_ARM)


BAIL_OLD = Box("bail_failed", pos=_bail_pos(OLD.pos), size=(BAIL_T, 2 * BAIL_Y + BAIL_T, BAIL_T), color=RED,
               mass=BAIL_MASS)
BAIL_NEW = Box("bail_spare", pos=_bail_pos(NEW.pos), size=(BAIL_T, 2 * BAIL_Y + BAIL_T, BAIL_T), color=BAIL_C,
               mass=BAIL_MASS)
NEIGH = {p: Box(f"module_port{p}", pos=(MOD0_C[0], PORT_Y[p], PZ), size=_MBASE, color=MOD_C, mass=MOD_MASS)
         for p in LIVE}
NEIGH_BREAK_F, NEIGH_BREAK_T = 15.0, 0.6
LC = Box("lc_connector", pos=(LC0_X, 0.0, PZ), size=LC_BODY, color=LCBODY, mass=LC_MASS)
LC_TAB = Box("lc_pull_tab", pos=(TAB0_X, 0.0, PZ), size=TAB_SIZE, color=LCTAB_C, mass=TAB_MASS)

MODULES = {"old": OLD, "new": NEW}
BAILS = {"old": BAIL_OLD, "new": BAIL_NEW}
STATIC_COLLIDERS: list[Box] = [*STATIC, SW_BACK]

OBJECTS = [*STATIC, SW_BACK, OLD, NEW, BAIL_OLD, BAIL_NEW, *NEIGH.values(), LC, LC_TAB]

TWIN_EXTRA = [
    {"name": "qsfp28_module", "kind": "body", "size_mm": [89.4, 18.35, 8.5], "colour": "silver",
     "note": "x4: 100GBASE-LR4 QSFP28 (Finisar FTLC1154RDPL), duplex LC receptacle 12 x 7 x 12 mm; failed has red bail"},
    {"name": "bail_hinge", "kind": "joint", "real": "QSFP28 wire bail clasp", "note": "0 to -90 deg about y, over-centre"},
    {"name": "lc_push_pull_tab", "kind": "joint", "real": "LC uniboot push-pull tab",
     "note": "prismatic -4.5..0 mm along x on lc_connector, 120 N/m return spring; >=2.5 mm pull releases LC latch"},
    {"name": "fibre_cord", "kind": "body", "size_mm": [300, 4, 4], "colour": "yellow",
     "note": "15 ball-jointed capsule links from lc_connector rear to bench tape at (0.10, 0.06)"},
]

# -- small quaternion / matrix helpers --------------------------------------------------------------------------------
_ST: dict = {"env": None, "root": None}


def _quat_wxyz(R: np.ndarray) -> tuple[float, float, float, float]:
    from scipy.spatial.transform import Rotation

    x, y, z, w = Rotation.from_matrix(np.asarray(R, float)).as_quat()
    return float(w), float(x), float(y), float(z)


def _R(q) -> np.ndarray:
    q = np.asarray(q, float)
    n = float(np.linalg.norm(q))
    x, y, z, w = (q / n) if n > 1e-12 else (0.0, 0.0, 0.0, 1.0)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def _qconj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


# -- stage authoring hook ---------------------------------------------------------------------------------------------
def _author(root: str) -> None:
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    _ST["root"] = root
    env_off = np.array(UsdGeom.Xformable(stage.GetPrimAtPath(root)).ComputeLocalToWorldTransform(0).ExtractTranslation())
    looks = f"{root}/QSLooks"
    mats = {}
    for key, col in (
        ("mod", MOD_C), ("lc", LCBODY), ("tab", LCTAB_C), ("bail_old", RED), ("bail_new", BAIL_C),
        ("yellow", YELLOW), ("white", WHITE), ("amber", AMBER), ("green", GREEN), ("bezel", BEZEL_C),
    ):
        p = f"{looks}/{key}"
        m = sim_utils.PreviewSurfaceCfg(
            diffuse_color=linear_rgb(col), roughness=0.38, metallic=0.35 if key in ("mod", "bezel") else 0.0
        )
        m.func(p, m)
        mats[key] = p
    phys = f"{looks}/metal_phys"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=0.45, dynamic_friction=0.38)
    pm.func(phys, pm)
    cphys = f"{looks}/cord_phys"
    cpm = sim_utils.RigidBodyMaterialCfg(static_friction=0.8, dynamic_friction=0.7)
    cpm.func(cphys, cpm)

    def cube(path, c, s, look, collide=True):
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
        g.AddScaleOp().Set(Gf.Vec3f(*map(float, s)))
        if collide:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.001)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(g.GetPath()), phys, stage=stage)
        bind_visual_material(str(g.GetPath()), look, stage=stage)

    def cyl(path, c, r, h, axis, look):
        g = UsdGeom.Cylinder.Define(stage, path)
        g.CreateRadiusAttr(float(r))
        g.CreateHeightAttr(float(h))
        g.CreateAxisAttr(axis)
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
        bind_visual_material(str(g.GetPath()), look, stage=stage)

    def filt(p0, p1):
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(p0)).CreateFilteredPairsRel().AddTarget(Sdf.Path(p1))

    def xf(path):
        return np.array(UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)).T

    def fixed(path, body0, body1, enabled=True, brk=None):
        try:
            T1 = xf(body1)
            rel = (np.linalg.inv(xf(body0)) @ T1) if body0 else T1
        except Exception:  # noqa: BLE001
            rel = np.eye(4)
        j = UsdPhysics.FixedJoint.Define(stage, path)
        if body0:
            j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, rel[:3, 3])))
        w, x, y, z = _quat_wxyz(rel[:3, :3])
        j.CreateLocalRot0Attr(Gf.Quatf(w, x, y, z))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateJointEnabledAttr(bool(enabled))
        if brk:
            j.CreateBreakForceAttr(float(brk[0]))
            j.CreateBreakTorqueAttr(float(brk[1]))
        return j

    def calm(path, iters=24):
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(path))
        api.CreateSolverPositionIterationCountAttr(iters)
        api.CreateSolverVelocityIterationCountAttr(4)
        api.CreateMaxDepenetrationVelocityAttr(0.5)

    def receptacle(mp, look):
        xc = M_REAR + RD / 2
        tw = (MH - RH) / 2
        sw = (MW - RW) / 2
        cube(f"{mp}/rec_top", (xc, 0, RH / 2 + tw / 2), (RD, MW, tw), look)
        cube(f"{mp}/rec_bot", (xc, 0, -RH / 2 - tw / 2), (RD, MW, tw), look)
        cube(f"{mp}/rec_l", (xc, RW / 2 + sw / 2, 0), (RD, sw, RH), look)
        cube(f"{mp}/rec_r", (xc, -RW / 2 - sw / 2, 0), (RD, sw, RH), look)
        cube(f"{mp}/label", (-0.012, 0.0, MH / 2 + 0.0002), (0.036, 0.014, 0.0003), mats["white"], collide=False)

    # visual switch faceplate details: port LEDs, live-port witness tape marks, spare outline in tray
    vis_scope = f"{root}/qs_decor"
    UsdGeom.Xform.Define(stage, vis_scope)
    cube(f"{vis_scope}/led_p5", (FP - 0.0006, 0.0, PZ + CH / 2 + 0.003), (0.0012, 0.004, 0.0025), mats["amber"], False)
    for p in LIVE:
        cube(f"{vis_scope}/led_p{p}", (FP - 0.0006, PORT_Y[p], PZ + CH / 2 + 0.003), (0.0012, 0.004, 0.0025),
             mats["green"], False)
        cube(f"{vis_scope}/mark_face_p{p}", (FP - 0.0006, PORT_Y[p] + MW / 2 + 0.002, PZ), (0.0012, 0.003, 0.008),
             mats["white"], False)
    cube(f"{vis_scope}/spare_mark", (SPARE_C[0], SPARE_C[1], TOP + 0.0022), (ML, MW + 0.003, 0.0003), mats["white"], False)

    jroot = f"{root}/qs_joints"
    UsdGeom.Scope.Define(stage, jroot)
    swbp = f"{root}/{SW_BACK.name}"
    calm(swbp, 24)
    fixed(f"{jroot}/sw_mount", None, swbp, enabled=True)

    # LC connector + push-pull release tab
    lcp = f"{root}/{LC.name}"
    tabp = f"{root}/{LC_TAB.name}"
    calm(lcp, 24)
    calm(tabp, 24)
    cube(f"{lcp}/housing", (LC_BODY[0] / 2 + LC_HOUS[0] / 2, 0, 0), LC_HOUS, mats["lc"])
    cube(f"{lcp}/latch_clip", (0.001, 0, LC_BODY[2] / 2 + 0.0005), (0.012, 0.0050, 0.0010), mats["lc"])
    for sgn, nm in ((1, "ferrule_l"), (-1, "ferrule_r")):
        cyl(f"{lcp}/{nm}", (LC_FRONT - 0.0002, sgn * 0.003125, 0.0), 0.000625, 0.0008, "X", mats["white"])
    cube(f"{tabp}/tongue", (TAB_SIZE[0] / 2 + 0.009, 0, TAB_SIZE[2] / 2 + 0.0004), (0.018, 0.0055, 0.0010),
         mats["tab"], collide=False)
    cube(f"{tabp}/stripe", (-0.002, 0, TAB_SIZE[2] / 2 + 0.0003), (0.014, 0.006, 0.0004), mats["white"], collide=False)
    filt(lcp, tabp)

    pj = UsdPhysics.PrismaticJoint.Define(stage, f"{jroot}/tab_slide")
    pj.CreateBody0Rel().SetTargets([Sdf.Path(lcp)])
    pj.CreateBody1Rel().SetTargets([Sdf.Path(tabp)])
    pj.CreateLocalPos0Attr(Gf.Vec3f(float(TAB_REL_X), 0.0, 0.0))
    pj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    pj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    pj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    pj.CreateAxisAttr("X")
    pj.CreateLowerLimitAttr(float(-TAB_MAX_PULL))
    pj.CreateUpperLimitAttr(0.0)
    pj.CreateExcludeFromArticulationAttr(True)
    pdrv = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear")
    pdrv.CreateTypeAttr("force")
    pdrv.CreateStiffnessAttr(float(TAB_K))
    pdrv.CreateDampingAttr(float(TAB_D))
    pdrv.CreateTargetPositionAttr(0.0)

    # 15-link yellow fibre cord attached to inner LC body (so pulling the cord does not pull the release tab)
    croot = f"{root}/qs_cord"
    UsdGeom.Xform.Define(stage, croot)
    cpaths = [f"{croot}/cord_{i:02d}" for i in range(N_CORD)]
    _ST["cord_paths"] = cpaths

    def q4(q):
        return Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2]))

    for i, cp_path in enumerate(cpaths):
        xf_c = UsdGeom.Xform.Define(stage, cp_path)
        c0, q0 = CORD_C0[i], CORD_Q0[i]
        xop = UsdGeom.Xformable(xf_c)
        xop.AddTranslateOp().Set(Gf.Vec3d(float(c0[0]), float(c0[1]), float(TOP + c0[2])))
        xop.AddOrientOp().Set(q4(q0))
        cap = UsdGeom.Capsule.Define(stage, f"{cp_path}/shape")
        cap.CreateRadiusAttr(CORD_R)
        cap.CreateHeightAttr(max(0.001, LINK_L - 2 * CORD_R))
        cap.CreateAxisAttr("X")
        bind_visual_material(f"{cp_path}/shape", mats["yellow"], stage=stage)
        UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
        pcol = PhysxSchema.PhysxCollisionAPI.Apply(cap.GetPrim())
        pcol.CreateContactOffsetAttr(0.0015)
        pcol.CreateRestOffsetAttr(0.0)
        bind_physics_material(f"{cp_path}/shape", cphys, stage=stage)
        UsdPhysics.RigidBodyAPI.Apply(xf_c.GetPrim())
        UsdPhysics.MassAPI.Apply(xf_c.GetPrim()).CreateMassAttr(float(CORD_KGM * LINK_L))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(xf_c.GetPrim())
        rb.CreateSolverPositionIterationCountAttr(20)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateLinearDampingAttr(0.6)
        rb.CreateAngularDampingAttr(2.5)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)

    def ball(name, b0, b1, p0, p1, r0, r1):
        j = UsdPhysics.Joint.Define(stage, f"{jroot}/{name}")
        if b0:
            j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, p0)))
        j.CreateLocalRot0Attr(r0)
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, p1)))
        j.CreateLocalRot1Attr(r1)
        for dof in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        for dof in ("rotX", "rotY", "rotZ"):
            d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), dof)
            d.CreateTypeAttr("force")
            d.CreateStiffnessAttr(CORD_BEND_K)
            d.CreateDampingAttr(CORD_BEND_D)
            d.CreateTargetPositionAttr(0.0)
        j.CreateExcludeFromArticulationAttr(True)

    q_id = np.array([0.0, 0.0, 0.0, 1.0])
    for i in range(N_CORD + 1):
        if i == 0:
            rel = _qmul(_qconj(np.array(CORD_Q0[0])), q_id)
            ball("cord_to_lc", lcp, cpaths[0], (CORD_ATTACH_LC_X, 0, 0), (-LINK_L / 2, 0, 0), Gf.Quatf(1, 0, 0, 0),
                 q4(rel))
        elif i < N_CORD:
            rel = _qmul(_qconj(np.array(CORD_Q0[i])), np.array(CORD_Q0[i - 1]))
            ball(f"cord_{i:02d}_j", cpaths[i - 1], cpaths[i], (LINK_L / 2, 0, 0), (-LINK_L / 2, 0, 0),
                 Gf.Quatf(1, 0, 0, 0), q4(rel))
        else:
            pw = env_off + np.array([CORD_PTS[-1][0], CORD_PTS[-1][1], TOP + CORD_PTS[-1][2]])
            ball("cord_to_tape", None, cpaths[-1], pw, (LINK_L / 2, 0, 0), q4(np.array(CORD_Q0[-1])),
                 Gf.Quatf(1, 0, 0, 0))
    filt(lcp, cpaths[0])
    filt(tabp, cpaths[0])
    filt(tabp, cpaths[1])
    filt(cpaths[-1], f"{root}/cord_tape")
    filt(cpaths[-2], f"{root}/cord_tape")
    for i in range(N_CORD - 1):
        filt(cpaths[i], cpaths[i + 1])

    # failed and spare QSFP28 modules + hinged wire bails + cage/LC latches
    for side, key in (("old", "bail_old"), ("new", "bail_new")):
        mp = f"{root}/{MODULES[side].name}"
        bp = f"{root}/{BAILS[side].name}"
        receptacle(mp, mats["mod"])
        calm(mp, 24)
        calm(bp, 16)
        for sgn, nm in ((1, "arm_l"), (-1, "arm_r")):
            cube(f"{bp}/{nm}", (0, sgn * BAIL_Y, -BAIL_ARM / 2), (BAIL_T, BAIL_T, BAIL_ARM), mats[key])
        filt(bp, mp)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{mp}/bail_hinge")
        j.CreateBody0Rel().SetTargets([Sdf.Path(mp)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(bp)])
        j.CreateLocalPos0Attr(Gf.Vec3f(float(BAIL_PIV), 0.0, 0.0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, float(-BAIL_ARM)))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateAxisAttr("Y")
        j.CreateLowerLimitAttr(-90.0)
        j.CreateUpperLimitAttr(0.0)
        j.CreateExcludeFromArticulationAttr(True)
        drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(BAIL_K)
        drv.CreateDampingAttr(BAIL_K * 0.25)
        drv.CreateTargetPositionAttr(0.0)
        _ST[f"hinge_{side}"] = str(j.GetPath())
        fixed(f"{jroot}/cage_{side}", swbp, mp, enabled=(side == "old"))
        fixed(f"{jroot}/lc_{side}", mp, lcp, enabled=(side == "old"))

    # fixed live patch cords (static capsule colliders) dressed from ports 1, 9 and the U1 patch panel into the managers
    sroot = f"{root}/qs_live_cords"
    UsdGeom.Xform.Define(stage, sroot)
    for ci, cpts in enumerate(STATIC_CORDS):
        for si in range(len(cpts) - 1):
            a, b = cpts[si], cpts[si + 1]
            L = float(np.linalg.norm(b - a))
            sp = f"{sroot}/c{ci:02d}_{si:02d}"
            xs = UsdGeom.Xform.Define(stage, sp)
            xo = UsdGeom.Xformable(xs)
            xo.AddTranslateOp().Set(Gf.Vec3d(*map(float, (a + b) / 2)))
            xo.AddOrientOp().Set(q4(np.array(_x_quat_xyzw(a, b))))
            cap = UsdGeom.Capsule.Define(stage, f"{sp}/shape")
            cap.CreateRadiusAttr(CORD_R)
            cap.CreateHeightAttr(max(0.0005, L))
            cap.CreateAxisAttr("X")
            bind_visual_material(f"{sp}/shape", mats["yellow"], stage=stage)
            UsdPhysics.CollisionAPI.Apply(cap.GetPrim())

    # live neighbour modules (ports 1 and 9): module + LC + cord stub + witness stripe, held by 15 N breakable weld
    for p, nb in NEIGH.items():
        mp = f"{root}/{nb.name}"
        receptacle(mp, mats["mod"])
        lx = LC_SEAT_REL
        cube(f"{mp}/lc_body", (lx, 0, 0), LC_BODY, mats["lc"])
        cube(f"{mp}/lc_housing", (lx + LC_BODY[0] / 2 + LC_HOUS[0] / 2, 0, 0), LC_HOUS, mats["lc"])
        cube(f"{mp}/lc_tab", (lx + TAB_REL_X, 0, 0), TAB_SIZE, mats["tab"])
        for sgn, nm in ((1, "bail_l"), (-1, "bail_r")):
            cube(f"{mp}/{nm}", (BAIL_PIV, sgn * BAIL_Y, BAIL_ARM / 2), (BAIL_T, BAIL_T, BAIL_ARM), mats["bail_new"])
        cube(f"{mp}/bail_bar", (BAIL_PIV, 0, BAIL_ARM), (BAIL_T, 2 * BAIL_Y + BAIL_T, BAIL_T), mats["bail_new"])
        cube(f"{mp}/witness_mark", (FP - MOD0_C[0] - 0.002, MW / 2 + 0.0003, 0), (0.005, 0.0006, 0.008), mats["white"],
             collide=False)
        calm(mp, 20)
        fixed(f"{jroot}/live_{p}", swbp, mp, enabled=True, brk=(NEIGH_BREAK_F, NEIGH_BREAK_T))

    _ST["cord_view"] = None
    _ST["gpaths"] = {}
    print(f"[qs] authored switch, 4 modules, prismatic LC pull-tab, {N_CORD}-link cord ({CORD_LEN:.2f} m)", flush=True)


def _wrap_last_spawner(obj):
    from isaaclab.sim.utils import clone

    cfg = obj.spawner_cfg
    orig = cfg.func
    base = getattr(orig, "__wrapped__", orig)

    def spawn(prim_path, c, *a, **k):
        prim = base(prim_path, c, *a, **k)
        _author(str(prim_path).rsplit("/", 1)[0])
        return prim

    cfg.func = clone(spawn)
    obj.object_cfg = obj._init_object_cfg()
    return obj


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_qs_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        obj = orig(spec)
        if getattr(spec, "name", None) == OBJECTS[-1].name:
            _wrap_last_spawner(obj)
        return obj

    task_object._qs_patched = True
    scene.task_object = task_object


class _SceneFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _SceneFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _SceneFinder())


# -- robot gripper collision boxes (for end-face contact & parked clearance checks) -----------------------------------
_GRIPPER_BOXES_LOCAL: dict | None = None
_BOX_GRID = np.array([[i, j, k] for i in (0.0, 0.5, 1.0) for j in (0.0, 0.5, 1.0) for k in (0.0, 0.5, 1.0)])


def _load_gripper_boxes() -> dict:
    global _GRIPPER_BOXES_LOCAL
    if _GRIPPER_BOXES_LOCAL is not None:
        return _GRIPPER_BOXES_LOCAL
    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    text = pathlib.Path(YAM_URDF).read_text()
    uroot = pathlib.Path(YAM_URDF).parent

    def stl(p):
        b = p.read_bytes()
        if b[:5] == b"solid" and b"facet" in b[:300]:
            return np.array(re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", b), float)
        n = int.from_bytes(b[80:84], "little")
        a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
        return a["v"].reshape(-1, 3).astype(float)

    out = {}
    for link, split in (("gripper", None), ("tip_left", -0.044), ("tip_right", 0.044)):
        block = re.search(r'<link name="%s".*?</link>' % link, text, re.S).group(0)
        groups: dict[int, list] = {}
        for c in re.findall(r"<collision>.*?</collision>", block, re.S):
            xyz = np.array(re.search(r'xyz="([^"]+)"', c).group(1).split(), float)
            rpy = np.array(re.search(r'rpy="([^"]+)"', c).group(1).split(), float)
            v = stl(uroot / re.search(r'filename="([^"]+)"', c).group(1)) @ rpy_matrix(rpy).T + xyz
            key = 0 if split is None else int((v[:, 1].mean() < split) if split < 0 else (v[:, 1].mean() > split))
            groups.setdefault(key, []).append(v)
        out[link] = [(np.min(np.concatenate(g), 0), np.max(np.concatenate(g), 0)) for g in groups.values()]
    _GRIPPER_BOXES_LOCAL = out
    return out


# -- pure-geometry kinematics, grasp verification, latches & scoring --------------------------------------------------
LC_HOME_TOL = 0.0015
MOD_HOME_TOL = 0.0015
TOUCH_TOL = 0.0010
MOD_TOUCH_TOL = 0.0004
NEIGH_MOVE = 0.0020
PULL_EARLY = 0.0020
OLD_OUT_DIST = 0.050
CLEAR = 0.10
STILL_V = 0.02
PARK_S = 0.5

GRIP_SPECS = (
    (OLD.name, "old", M_REAR + 0.005, M_REAR + 0.038, MW, MH / 2),
    (NEW.name, "new", M_REAR + 0.005, M_REAR + 0.038, MW, MH / 2),
    (LC_TAB.name, "tab", -TAB_SIZE[0] / 2, TAB_SIZE[0] / 2, TAB_SIZE[1], TAB_SIZE[2] / 2),
    (LC.name, "lc", -LC_BODY[0] / 2, LC_BODY[0] / 2, LC_BODY[1], LC_BODY[2] / 2),
)


def _grip_bodies() -> list[str]:
    root = _ST.get("root")
    return [f"{root}/{nm}" for nm, *_ in GRIP_SPECS] if root else []


def _grip_band_ok(side: str, mid: np.ndarray, body: str) -> bool:
    """Pads must close on the part's grip band (within its length, centred across its width)."""
    nm = body.rsplit("/", 1)[-1]
    for name, key, x0, x1, w, _hz in GRIP_SPECS:
        if name != nm:
            continue
        T = _pose_obj(name)
        q = T[:3, :3].T @ (np.asarray(mid, float) - _origin() - T[:3, 3])
        return bool(x0 - 0.004 <= q[0] <= x1 + 0.004 and abs(q[1]) <= w / 2 + 0.006)
    return False


GRIP = gl.LimitedGrip(_grip_bodies, label="optic/LC", name="pad_grip", engage_ok=_grip_band_ok,
                      joint_root=lambda: f"{_ST['root']}/qs_joints" if _ST.get("root") else None, log_prefix="[qs]")


def bail_deg(Tm: np.ndarray, Tb: np.ndarray) -> float:
    """Bail swing angle (deg): 0 = upright closed, -90 = horizontal open (-x)."""
    d = Tm[:3, :3].T @ (Tb[:3, :3] @ np.array([0.0, 0.0, 1.0]))
    return math.degrees(math.atan2(float(d[0]), float(d[2])))


def tab_slide(Tl: np.ndarray, Tt: np.ndarray) -> float:
    """Signed displacement (m) of lc_pull_tab relative to its rest stop along LC +x (negative = pulled back)."""
    return float(Tl[:3, 0] @ (Tt[:3, 3] - Tl[:3, 3])) - TAB_REL_X


def lc_front_rel(Tm: np.ndarray, Tl: np.ndarray) -> np.ndarray:
    """Position of LC front ferrule face centre in module body frame."""
    return Tm[:3, :3].T @ (Tl[:3, :3] @ np.array([LC_FRONT, 0.0, 0.0]) + Tl[:3, 3] - Tm[:3, 3])


def lc_in_module(Tm: np.ndarray, Tl: np.ndarray) -> float:
    """Depth (m) of LC front face inside module receptacle (negative = outside), or -1 if misaligned."""
    R = Tm[:3, :3].T @ Tl[:3, :3]
    if float(R[0, 0]) < math.cos(math.radians(6.0)):
        return -1.0
    f = lc_front_rel(Tm, Tl)
    if math.hypot(float(f[1]), float(f[2])) > 0.0022:
        return -1.0
    return float(f[0] - M_REAR)


def module_depth(Tm: np.ndarray, port: int = 5) -> tuple[float, float]:
    """(front_x - SEAT_X [0 = home, negative = out], radial offset from port axis in y-z)."""
    f = Tm[:3, :3] @ np.array([M_FRONT, 0.0, 0.0]) + Tm[:3, 3]
    return float(f[0] - SEAT_X), math.hypot(float(f[1] - PORT_Y[port]), float(f[2] - PZ))


def _static_hit(p: np.ndarray) -> str:
    if p[2] <= TOP + TOUCH_TOL:
        return "table"
    if _seg_dist(p) <= CORD_R + TOUCH_TOL:
        return "live_patch_cord"
    for b in STATIC_COLLIDERS:
        c, s = np.asarray(b.pos), np.asarray(b.size) / 2 + TOUCH_TOL
        if np.all(np.abs(p - c) <= s):
            return b.name
    return ""


def _box_hit(p: np.ndarray, T: np.ndarray, half, tol: float = TOUCH_TOL) -> bool:
    q = T[:3, :3].T @ (p - T[:3, 3])
    return bool(np.all(np.abs(q) <= np.asarray(half) + tol))


def _module_hit(p: np.ndarray, T: np.ndarray) -> bool:
    q = T[:3, :3].T @ (p - T[:3, 3])
    t = TOUCH_TOL
    if not (M_REAR - t <= q[0] <= M_FRONT + t and abs(q[1]) <= MW / 2 + t and abs(q[2]) <= MH / 2 + t):
        return False
    in_mouth = (
        abs(q[1]) <= RW / 2 + 0.0015
        and abs(q[2]) <= RH / 2 + 0.0015
        and M_REAR - t - 0.001 <= q[0] <= M_REAR + RD + t
    )
    return not in_mouth


def lc_face_points(Tl: np.ndarray) -> list[np.ndarray]:
    hx = LC_FRONT
    hy, hz = LC_HOUS[1] / 2, LC_HOUS[2] / 2
    loc = [(hx, a * hy, b * hz) for a in (-1, 0, 1) for b in (-1, 0, 1)]
    return [Tl[:3, :3] @ np.array(v) + Tl[:3, 3] for v in loc]


def lc_touch(P: dict, gboxes: list | None = None, cord_pts: np.ndarray | None = None) -> str:
    """Surface name within TOUCH_TOL of any of the 9 points on the LC front ferrule face (receptacle mouths excepted),
    including both robot grippers and finger pads; '' if clean."""
    for p in lc_face_points(P["lc"]):
        h = _static_hit(p)
        if h:
            return h
        for side in ("old", "new"):
            if _module_hit(p, P[side]):
                return f"module_{side}"
        for k in NEIGH:
            if _box_hit(p, P[f"n{k}"], (0.09, MW / 2, MH / 2 + 0.004)):
                return f"port{k}"
        for side in ("old", "new"):
            if _box_hit(p, P[f"bail_{side}"], (BAIL_T / 2, BAIL_Y + BAIL_T, BAIL_T / 2)):
                return f"bail_{side}"
        if gboxes:
            for T_lk, lo, hi, gname in gboxes:
                ql = T_lk[:3, :3].T @ (p - T_lk[:3, 3])
                if np.all(ql >= lo - TOUCH_TOL) and np.all(ql <= hi + TOUCH_TOL):
                    return gname
        if cord_pts is not None and len(cord_pts):
            if float(np.linalg.norm(cord_pts - p, axis=1).min()) <= CORD_R + TOUCH_TOL:
                return "cord"
    return ""


def in_port(T: np.ndarray) -> int:
    f = T[:3, :3] @ np.array([M_FRONT, 0.0, 0.0]) + T[:3, 3]
    if f[0] < FP + 0.002:
        return 0
    for p, y in PORT_Y.items():
        if abs(f[1] - y) < CW / 2 and abs(f[2] - PZ) < CH / 2:
            return p
    return 0


def in_tray(T: np.ndarray, held_names=()) -> bool:
    if OLD.name in held_names:
        return False
    c = T[:3, 3]
    return bool(
        abs(c[0] - TRAY_C0[0]) <= TRAY_IN[0] / 2
        and abs(c[1] - TRAY_C0[1]) <= TRAY_IN[1] / 2
        and c[2] <= TOP + 0.002 + MH / 2 + 0.003
        and abs(float(T[2, 2])) >= 0.95
    )


def _part_points(P: dict) -> np.ndarray:
    pts = []
    for side in ("old", "new"):
        T = P[side]
        for x in (M_REAR, M_REAR + 0.020, 0.0, M_FRONT):
            pts.append(T[:3, :3] @ np.array([x, 0.0, 0.0]) + T[:3, 3])
        pts.append(P[f"bail_{side}"][:3, 3])
    for key, xs in (("lc", (-LC_BODY[0] / 2, 0.0, LC_FRONT)), ("tab", (-TAB_SIZE[0] / 2, 0.0, TAB_SIZE[0] / 2))):
        T = P[key]
        for x in xs:
            pts.append(T[:3, :3] @ np.array([x, 0.0, 0.0]) + T[:3, 3])
    return np.asarray(pts)


# -- live simulator state & runtime hooks -----------------------------------------------------------------------------
_H: dict = {}
_EP: dict = {}


def _stage():
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        return get_current_stage()
    except Exception:  # noqa: BLE001
        return None


def _joint(name: str):
    st = _stage()
    if st is None or _ST.get("root") is None:
        return None
    p = st.GetPrimAtPath(f"{_ST['root']}/qs_joints/{name}")
    return p if p.IsValid() else None


def _set(name: str, on: bool, T0=None, T1=None) -> bool:
    from pxr import Gf

    p = _joint(name)
    if p is None:
        return False
    if on and T1 is not None:
        rel = np.linalg.inv(T0) @ T1 if T0 is not None else T1
        p.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, rel[:3, 3])))
        w, x, y, z = _quat_wxyz(rel[:3, :3])
        p.GetAttribute("physics:localRot0").Set(Gf.Quatf(w, x, y, z))
    p.GetAttribute("physics:jointEnabled").Set(bool(on))
    return True


def _is_on(name: str) -> bool:
    p = _joint(name)
    return bool(p is not None and p.GetAttribute("physics:jointEnabled").Get())


def _origin() -> np.ndarray:
    base = _ST["env"].unwrapped
    return base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)


def _pose_obj(name: str) -> np.ndarray:
    base = _ST["env"].unwrapped
    p = base.arena_world.get_pose_w(name)[0].detach().cpu().numpy().astype(np.float64)
    T = np.eye(4)
    T[:3, :3] = _R(p[3:7])
    T[:3, 3] = p[:3] - _origin()
    return T


def _w(T: np.ndarray) -> np.ndarray:
    o = np.eye(4)
    o[:3, 3] = _origin()
    return o @ T


def live_poses() -> dict:
    P = {
        "sw_back": _pose_obj(SW_BACK.name),
        "old": _pose_obj(OLD.name),
        "new": _pose_obj(NEW.name),
        "bail_old": _pose_obj(BAIL_OLD.name),
        "bail_new": _pose_obj(BAIL_NEW.name),
        "lc": _pose_obj(LC.name),
        "tab": _pose_obj(LC_TAB.name),
    }
    for p, nb in NEIGH.items():
        P[f"n{p}"] = _pose_obj(nb.name)
    return P


def _cord_centres() -> np.ndarray | None:
    st = _stage()
    cpaths = _ST.get("cord_paths")
    if st is None or not cpaths:
        return None
    try:
        from pxr import UsdGeom

        o = _origin()
        return np.array([
            np.array(UsdGeom.Xformable(st.GetPrimAtPath(p)).ComputeLocalToWorldTransform(0).ExtractTranslation()) - o
            for p in cpaths
        ])
    except Exception:  # noqa: BLE001
        return None


def _hand_states():
    from yam_arena.runtime import link_poses, tcp_world

    base = _ST["env"].unwrapped
    o = _origin()
    blocal = _load_gripper_boxes()
    hands = []
    gboxes = []
    allpts = []
    tcps = []
    aps = []
    for side in ("left", "right"):
        rob = base.scene.articulations[f"{side}_robot"]
        names = list(rob.joint_names)
        j7, j8 = names.index("joint7"), names.index("joint8")
        q = getattr(rob.data.joint_pos, "torch", rob.data.joint_pos)[0].detach().cpu().numpy()
        tgt = getattr(rob.data.joint_pos_target, "torch", rob.data.joint_pos_target)[0].detach().cpu().numpy()
        ap = abs(float(q[j7])) + abs(float(q[j8]))
        cmd = abs(float(tgt[j7])) + abs(float(tgt[j8]))
        lp = link_poses(rob)
        G = lp["gripper"].copy()
        G[:3, 3] -= o
        tcp = np.asarray(tcp_world(rob), float) - o
        pts = []
        for lk, bl in blocal.items():
            T_lk = lp[lk].copy()
            T_lk[:3, 3] -= o
            for lo, hi in bl:
                gboxes.append((T_lk, lo, hi, f"{side}_gripper"))
                w = (_BOX_GRID * (hi - lo) + lo) @ T_lk[:3, :3].T + T_lk[:3, 3]
                pts.append(w)
        hands.append((side, ap, cmd, G))
        allpts.append(np.concatenate(pts))
        tcps.append(tcp)
        aps.append(ap)
    return hands, gboxes, np.concatenate(allpts), tcps, aps


def _hacks() -> None:
    if _ST.get("root") is None or _ST.get("env") is None:
        return
    _H["n"] = _H.get("n", 0) + 1
    P = live_poses()
    hands, gboxes, gpts, tcps, aps = _hand_states()
    _H.update(P=P, gboxes=gboxes, gpts=gpts, tcps=tcps, aps=aps)

    # 1. Bail over-centre spring & cage latches
    st = _stage()
    for side in ("old", "new"):
        ang = bail_deg(P[side], P[f"bail_{side}"])
        _H[f"bail_{side}"] = ang
        hp = st.GetPrimAtPath(_ST[f"hinge_{side}"])
        tgt = -90.0 if ang < -45.0 else 0.0
        if hp.IsValid() and hp.GetAttribute("drive:angular:physics:targetPosition").Get() != tgt:
            hp.GetAttribute("drive:angular:physics:targetPosition").Set(tgt)
        latched = _is_on(f"cage_{side}")
        if latched and ang < -UNLATCH_DEG:
            _set(f"cage_{side}", False)
            print(f"[qs] {side} module unlatched (bail {ang:.1f} deg)", flush=True)
        elif not latched and abs(ang) <= 10.0:
            d, off = module_depth(P[side])
            if abs(d) <= MOD_HOME_TOL and off <= 0.0015 and float(P[side][0, 0]) > math.cos(math.radians(4.0)):
                _set(f"cage_{side}", True, _w(P["sw_back"]), _w(P[side]))
                print(f"[qs] {side} module latched in port 5 (d={d * 1000:.2f} mm, off={off * 1000:.2f} mm)", flush=True)

    # 2. LC push-pull tab unlatch (requires pulling lc_pull_tab >= 2.5 mm back) & push-home latch
    ts = tab_slide(P["lc"], P["tab"])
    _H["tab_slide"] = ts
    if ts <= -TAB_UNLATCH:
        for side in ("old", "new"):
            if _is_on(f"lc_{side}"):
                _set(f"lc_{side}", False)
                print(f"[qs] LC unlatched from {side} module via push-pull tab ({ts * 1000:.2f} mm)", flush=True)
    for side in ("old", "new"):
        home = lc_in_module(P[side], P["lc"]) >= RD - LC_HOME_TOL
        was = _H.get(f"lc_home_{side}", side == "old")
        _H[f"lc_home_{side}"] = home
        if home and not was and ts > -0.0015 and not _is_on(f"lc_{side}"):
            _set(f"lc_{side}", True, _w(P[side]), _w(P["lc"]))
            print(f"[qs] LC latched into {side} module", flush=True)

    # 3. Force-limited two-pad grip (grip_limited.LimitedGrip): engages only on PhysX contact on BOTH finger pads while
    #    the jaws squeeze, on the grip band of a part (module nose, LC pull tab, LC body); capped pull/twist/tip, slips above.
    GRIP.update(_ST["env"])
    _H["held"] = {s: GRIP.held_body(s).rsplit("/", 1)[-1] for s in ("left", "right") if GRIP.holding(s)}


def _reset_hacks() -> None:
    GRIP.reset()
    if _ST.get("root") is not None:
        P = live_poses()
        _set("cage_old", True, _w(P["sw_back"]), _w(P["old"]))
        _set("cage_new", False)
        _set("lc_old", True, _w(P["old"]), _w(P["lc"]))
        _set("lc_new", False)
    _H.clear()


def _reset_episode() -> None:
    _EP.clear()
    _EP.update(violation="", old_out=False, lc_out=False, n0=None, hist=[], gpts=None, aps=[0.09, 0.09], t=0, nan=False)
    if _ST.get("root") is not None and _ST.get("env") is not None:
        try:
            _reset_hacks()
        except Exception:  # noqa: BLE001
            _H.clear()


_reset_episode()


def track(P: dict, gboxes=None, gpts=None, tcps=None, aps=None, cord_pts=None) -> None:
    for T in P.values():
        if not np.isfinite(T).all():
            _EP["nan"] = True
            return
    _EP["t"] += 1
    if _EP["n0"] is None:
        _EP["n0"] = {k: P[f"n{k}"][:3, 3].copy() for k in NEIGH}
    v = _EP["violation"]
    lc_q_old = lc_front_rel(P["old"], P["lc"])
    lc_in_old = (
        (lc_q_old[0] > M_REAR + 0.002 and abs(lc_q_old[1]) <= RW / 2 + 0.002 and abs(lc_q_old[2]) <= RH / 2 + 0.002)
        or _is_on("lc_old")
    )
    d_old, _ = module_depth(P["old"])
    if not v and lc_in_old and d_old < -PULL_EARLY:
        v = f"module moved {-d_old * 1000:.1f} mm out with the LC still in it"
    if not _EP["lc_out"] and lc_q_old[0] < M_REAR - 0.003 and not _is_on("lc_old"):
        _EP["lc_out"] = True
    if _EP["lc_out"] and not v:
        hit = lc_touch(P, gboxes=gboxes, cord_pts=cord_pts)
        if hit:
            v = f"LC end face touched {hit}"
    for k in NEIGH:
        if not v and float(np.linalg.norm(P[f"n{k}"][:3, 3] - _EP["n0"][k])) > NEIGH_MOVE:
            v = f"live port {k} moved more than 2 mm"
    if not _EP["old_out"] and d_old < -OLD_OUT_DIST:
        _EP["old_out"] = True
    if _EP["old_out"] and not v and in_port(P["old"]):
        v = f"failed module re-entered port {in_port(P['old'])}"
    if v and not _EP["violation"]:
        _EP["violation"] = v
        print(f"[qs] score gate: {v}", flush=True)
    _EP["P"] = P
    if gpts is not None:
        _EP["gpts"] = gpts
    if aps is not None:
        _EP["aps"] = aps
    if tcps is not None:
        _EP["hist"].append(np.concatenate(tcps))
        _EP["hist"] = _EP["hist"][-int(PARK_S * 20) - 1:]


def spare_seated(P: dict, latched: bool | None = None, held_names=()) -> bool:
    if NEW.name in held_names:
        return False
    is_lat = _is_on("cage_new") if latched is None else latched
    d, off = module_depth(P["new"])
    return bool(is_lat and abs(d) <= MOD_HOME_TOL and off <= 0.0015 and abs(bail_deg(P["new"], P["bail_new"])) <= 10.0)


def lc_home_in_spare(P: dict, latched: bool | None = None, held_names=()) -> bool:
    if LC.name in held_names or LC_TAB.name in held_names:
        return False
    is_lat = _is_on("lc_new") if latched is None else latched
    return bool(is_lat and lc_in_module(P["new"], P["lc"]) >= RD - LC_HOME_TOL)


def parked(P: dict, gpts: np.ndarray | None = None, hist: list | None = None, aps=None, held_names=()) -> bool:
    if held_names:
        return False
    aps = _EP.get("aps", [0.09, 0.09]) if aps is None else aps
    if any(a < 0.020 for a in aps):
        return False
    n = int(PARK_S * 20) + 1
    h = _EP["hist"] if hist is None else hist
    if len(h) < n:
        return False
    pts = _part_points(P)
    gp = _EP.get("gpts") if gpts is None else gpts
    if gp is None or not len(gp):
        last = h[-1].reshape(2, 3)
        if any(float(np.linalg.norm(pts - x, axis=1).min()) < CLEAR for x in last):
            return False
    else:
        dmin = float(np.linalg.norm(gp[:, None, :] - pts[None, :, :], axis=2).min())
        if dmin < CLEAR:
            return False
    arr = np.asarray(h[-n:]).reshape(n, 2, 3)
    return bool(np.linalg.norm(np.diff(arr, axis=0), axis=2).max() * 20.0 < STILL_V)


def ladder(P: dict, cage_new_lat: bool | None = None, lc_new_lat: bool | None = None, held_names=None) -> float:
    if _EP["nan"] or _EP["violation"]:
        return 0.0
    if not _EP["old_out"] or in_port(P["old"]):
        return 0.0
    held_names = tuple(_H.get("held", {}).values()) if held_names is None else tuple(held_names)
    if not spare_seated(P, latched=cage_new_lat, held_names=held_names):
        return 0.25
    if not lc_home_in_spare(P, latched=lc_new_lat, held_names=held_names):
        return 0.50
    if not (in_tray(P["old"], held_names=held_names) and parked(P, held_names=held_names)):
        return 0.75
    return 1.0


def score(objects=None) -> float:
    """0 if at any time: port 5's module moved more than 2 mm out while the LC was still in it; after the LC came out,
    its front face (ferrule end) came within 1 mm of any surface except the inside of a module's LC receptacle (table,
    rack, managers, live patch cords, switch, dust plugs, tray, cord, a module's exterior, a bail, or either robot gripper); port 1 or 9 moved
    more than 2 mm; or the failed module re-entered a port. Otherwise (with parts released at each placed stage): 0
    failed module still in port 5 or less than 50 mm out; 0.25 failed module out (>50 mm); 0.50 also spare home and latched in port 5 with
    bail closed (within 10 deg); 0.75 also LC home and latched in the spare; 1.0 also failed module resting in the tray
    and both grippers open, >=10 cm clear of every module, bail and LC part, moving under 2 cm/s for 0.5 s."""
    if _ST.get("env") is None:
        return 0.0
    try:
        n = _H.get("n", 0)
        if _EP.get("last_n") != n:
            _EP["last_n"] = n
            P = _H.get("P") or live_poses()
            gboxes = _H.get("gboxes")
            gpts = _H.get("gpts")
            tcps = _H.get("tcps")
            aps = _H.get("aps")
            if gboxes is None:
                _, gboxes, gpts, tcps, aps = _hand_states()
            cpts = _cord_centres() if _EP.get("lc_out") else None
            track(P, gboxes=gboxes, gpts=gpts, tcps=tcps, aps=aps, cord_pts=cpts)
        P = _EP.get("P") or _H.get("P") or live_poses()
        return float(ladder(P))
    except Exception as e:  # noqa: BLE001
        print(f"[qs] score failed: {e!r}", flush=True)
        return 0.0


def success(objects=None) -> bool:
    return bool(score(objects) >= 1.0 - 1e-6)


def distance(objects=None) -> float:
    return 1.0 - score(objects)


def progress(objects=None) -> dict:
    P = _EP.get("P")
    if P is None:
        return {}
    return {
        "lc_out": float(_EP["lc_out"]),
        "old_out": float(_EP["old_out"]),
        "spare_home": float(spare_seated(P)),
        "lc_home": float(lc_home_in_spare(P)),
        "old_in_tray": float(in_tray(P["old"])),
        "violation": _EP["violation"],
    }


VARIATION: dict = {
    "obj:module_spare": {"dx": 0.008, "dy": 0.006, "yaw": 0.08},
}


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_qs_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ST["env"] = self._env
            try:
                _hacks()
            except Exception as e:  # noqa: BLE001
                if not _ST.get("hack_err"):
                    _ST["hack_err"] = repr(e)
                    import traceback

                    traceback.print_exc()
                    print(f"[qs] hack error: {e!r}", flush=True)
            out = base(self, *a, **k)
            try:
                if getattr(getattr(self, "task", None), "module", None) is sys.modules[__name__]:
                    score()
            except Exception:  # noqa: BLE001
                pass
            return out

        _task_step._qs_hooked = True
        cls._task_step = _task_step
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            out = base_reset(self, *a, **k)
            _ST["env"] = self._env
            return out

        reset._task_reset_hooks = []
        cls.reset = reset
    if _reset_episode not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_episode)


_install_hooks()
