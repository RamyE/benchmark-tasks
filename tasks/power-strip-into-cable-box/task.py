"""Home desk mock-up on the station table: an open cable-management box (a dark grey box, 23 x 11 cm inside, 6 cm
tall, 1 cm walls, with a 4 mm floor) is fixed to the table in front of the robot, its long side along the robot's
left-right axis. Each end wall has a cord slot: a U notch 14 mm wide, cut from the rim down to 15 mm above the table,
centred on the wall. Its free lid (a 25 x 13 cm, 6 mm thick plate of the same grey with a small lift handle on top)
lies flat on the table behind the box, on the left. In front of the box, nearest the robot, lies an unplugged white power strip
(180 x 50 x 35 mm, 150 g) along the left-right axis. Its black 7 mm power cord (32 cm) leaves its right end face and
lies straight on the table to the right, ending in a black mains plug; a blue 5 mm USB cable (30 cm) leaves the USB
port in its left end face and lies straight to the left, ending in a grey USB-C plug. The robot must put the strip
into the box, lead the power cord out through the right-hand slot and the USB cable out through the left-hand slot,
put the lid on flat and let go. A cord left over a wall instead of in its slot lifts the lid (a pinched cord).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants: CABLEBOX_DEMO=correct (default) | nothing | drape | no_lid | hold.
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

INSTRUCTION = ("Hide the unplugged power strip in the cable box: put the strip inside, lead its black power cord out "
               "through the slot in the box's right end and the blue USB cable out through the slot in its left end, "
               "then put the lid on flat and let go. No cord may lie over the box's rim.")

BOX_GREY = (0.22, 0.23, 0.25)
LID_GREY = (0.30, 0.31, 0.33)
WHITE = (0.93, 0.93, 0.91)
BLACK = (0.05, 0.05, 0.05)
BLUE = (0.15, 0.35, 0.85)
PLUG_GREY = (0.55, 0.56, 0.58)

# -- cable box (fixed): open top, slot in each end wall ------------------------------------------------------------
BC = np.array([0.30, 0.0])  # box centre (x, y)
IX, IY = 0.11, 0.23  # inside size (x = depth, y = length)
WT, WH = 0.010, 0.060  # wall thickness, wall height (from the table)
FLOOR_T = 0.004
RIM = TOP + WH
FLOOR = TOP + FLOOR_T
X_IN0, X_IN1 = BC[0] - IX / 2, BC[0] + IX / 2  # 0.245 .. 0.355
Y_IN0, Y_IN1 = BC[1] - IY / 2, BC[1] + IY / 2  # -0.115 .. 0.115
X_OUT0, X_OUT1 = X_IN0 - WT, X_IN1 + WT
Y_OUT0, Y_OUT1 = Y_IN0 - WT, Y_IN1 + WT
SLOT_X, SLOT_W = float(BC[0]), 0.014  # slot centre (x) and width, same in both end walls
SILL = TOP + 0.015  # slot bottom


def _box() -> list[Box]:
    zc = TOP + WH / 2
    ox = IX + 2 * WT
    out = [
        Box("box_floor", pos=(float(BC[0]), float(BC[1]), TOP + FLOOR_T / 2), size=(IX, IY, FLOOR_T), color=BOX_GREY),
        Box("box_wall_near", pos=(X_IN0 - WT / 2, float(BC[1]), zc), size=(WT, IY + 2 * WT, WH), color=BOX_GREY),
        Box("box_wall_far", pos=(X_IN1 + WT / 2, float(BC[1]), zc), size=(WT, IY + 2 * WT, WH), color=BOX_GREY),
    ]
    s0, s1 = SLOT_X - SLOT_W / 2, SLOT_X + SLOT_W / 2
    for tag, yc in (("left", Y_IN1 + WT / 2), ("right", Y_IN0 - WT / 2)):
        out += [
            Box(f"box_end_{tag}_near", pos=((X_IN0 + s0) / 2, yc, zc), size=(s0 - X_IN0, WT, WH), color=BOX_GREY),
            Box(f"box_end_{tag}_far", pos=((s1 + X_IN1) / 2, yc, zc), size=(X_IN1 - s1, WT, WH), color=BOX_GREY),
            Box(f"box_end_{tag}_sill", pos=(SLOT_X, yc, TOP + (SILL - TOP) / 2), size=(SLOT_W, WT, SILL - TOP),
                color=BOX_GREY),
        ]
    del ox
    return out


# -- lid (free) with its lift handle welded on top -----------------------------------------------------------------
LID_X, LID_Y, LID_T = IX + 2 * WT, IY + 2 * WT, 0.006  # 13 x 25 cm, same footprint as the box
LID_MASS, HANDLE_MASS = 0.08, 0.01
HDL = (0.014, 0.050, 0.020)  # handle bar along y
LID_XY0 = np.array([0.46, 0.22])  # behind the box on the left, clear of where the USB cable is laid
LID = Box("box_lid", pos=(float(LID_XY0[0]), float(LID_XY0[1]), TOP + LID_T / 2 + 0.0005), size=(LID_X, LID_Y, LID_T),
          mass=LID_MASS, color=LID_GREY)
HANDLE_DZ = LID_T / 2 + HDL[2] / 2
HANDLE = Box("lid_handle", pos=(float(LID_XY0[0]), float(LID_XY0[1]), LID.pos[2] + HANDLE_DZ), size=HDL,
             mass=HANDLE_MASS, color=(0.45, 0.46, 0.48))

# -- power strip (free) and its two cords (start ends welded into its end faces) -----------------------------------
SL, SW, SH = 0.18, 0.05, 0.035  # strip length (y), width (x), height
STRIP_MASS = 0.15
STRIP_XY0 = np.array([0.14, 0.0])
STRIP = Box("power_strip", pos=(float(STRIP_XY0[0]), float(STRIP_XY0[1]), TOP + SH / 2 + 0.0005), size=(SW, SL, SH),
            mass=STRIP_MASS, color=WHITE)
CORD_Z0 = STRIP.pos[2]  # cords leave the end faces at mid-height
R_PWR, R_USB = 0.0035, 0.0025
PWR = cable("power_cord", start=(float(STRIP_XY0[0]), float(STRIP_XY0[1] - SL / 2), CORD_Z0), yaw=-math.pi / 2,
            length=0.32, radius=R_PWR, anchor_start=False,
            end_plug=Plug(size=(0.035, 0.025, 0.018), mass=0.015, color=BLACK),
            bend_stiffness=0.01, bend_damping=0.001, color=BLACK)
USB = cable("usb_cable", start=(float(STRIP_XY0[0]), float(STRIP_XY0[1] + SL / 2), CORD_Z0), yaw=math.pi / 2,
            length=0.30, radius=R_USB, anchor_start=False, cable_mass=0.02,
            end_plug=Plug(size=(0.022, 0.012, 0.007), mass=0.004, color=PLUG_GREY),
            bend_stiffness=0.01, bend_damping=0.001, color=BLUE)
CORDS = (PWR, USB)
SLOT_OF = {PWR.name: "right", USB.name: "left"}  # required slot per cord

OBJECTS = [*_box(), LID, HANDLE, STRIP, PWR, USB]


# -- welds: each cord's first segment into the strip, the handle onto the lid --------------------------------------
# cablekit can only weld a cable end to the world. Its spawn function is wrapped so that, right after each cable is
# authored, fixed joints tie the cable's first segment to the (free) strip; the lid handle weld is added with the
# first cable. The joints are excluded from the cable articulations, so strip and lid stay ordinary rigid objects.


def _install_welds() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_cablebox_welds", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        name = getattr(cfg, "cable_name", "")
        spec = {c.name: c for c in CORDS}.get(name)
        if spec is None:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdPhysics

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        yaw = -math.pi / 2 if spec is PWR else math.pi / 2
        j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/strip_weld")
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{STRIP.name}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        off = np.asarray(spec.point_at(0.0)) - np.asarray(STRIP.pos)  # cord start tip in the strip frame (yaw 0)
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, off)))
        j.CreateLocalRot0Attr(Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
        j.CreateLocalPos1Attr(Gf.Vec3f(-spec.seg_len / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateExcludeFromArticulationAttr(True)
        if spec is PWR:
            h = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/handle_weld")
            h.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{LID.name}")])
            h.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{HANDLE.name}")])
            h.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, float(HANDLE_DZ)))
            h.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            h.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            h.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            h.CreateExcludeFromArticulationAttr(True)
        return prim

    spawn._cablebox_welds = True
    cablekit._spawn_cable = spawn


_install_welds()

# -- grippers (score() needs them for the release check) ----------------------------------------------------------
_ENV: dict = {"env": None}


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (no embodiment: offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_cablebox_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._cablebox_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (13 mm above the finger pad tips); [] if unknown (offline)."""
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


# -- scoring -------------------------------------------------------------------------------------------------------
LIFT_MAX = 0.003  # every lid underside corner within 3 mm of the rim
CENTRE_TOL = 0.010  # lid centre within 1 cm (horizontally) of the box centre
YAW_TOL = 5.0  # lid edges within 5 deg of the box's
IN_TOL = 0.002  # strip footprint tolerance (m)
RELEASE_DIST = 0.035  # both TCPs at least 3.5 cm from the strip, every cord point, both plugs, the lid and its handle
SLOT_TOL = 0.001


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _dense(poly: np.ndarray, step: float = 0.002) -> np.ndarray:
    if not np.isfinite(poly).all():
        return np.asarray(poly)
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * (k / n) for k in range(1, n + 1)]
    return np.asarray(out)


def lid_corner_lift(objects) -> float:
    """Largest |height of a lid underside corner above the rim| (m)."""
    c = objects[LID.name]
    if not np.isfinite(c.pos).all():
        return 1.0
    Rm = _rot(c.quat_xyzw)
    lifts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            p = c.pos + Rm @ np.array([sx * LID_X / 2, sy * LID_Y / 2, -LID_T / 2])
            lifts.append(abs(p[2] - RIM))
    return float(max(lifts))


def lid_yaw_deg(objects) -> float:
    Rm = _rot(objects[LID.name].quat_xyzw)
    a = math.degrees(math.atan2(Rm[1, 0], Rm[0, 0]))
    a = (a + 90.0) % 180.0 - 90.0  # the lid is symmetric under a half turn
    return abs(a)


def lid_shift(objects) -> float:
    return float(np.linalg.norm(objects[LID.name].pos[:2] - BC))


def lid_seated(objects) -> bool:
    return (lid_corner_lift(objects) <= LIFT_MAX and lid_shift(objects) <= CENTRE_TOL
            and lid_yaw_deg(objects) <= YAW_TOL)


def strip_inside(objects) -> bool:
    s = objects[STRIP.name]
    if not (np.isfinite(s.low).all() and np.isfinite(s.high).all()):
        return False
    return bool(s.low[0] >= X_IN0 - IN_TOL and s.high[0] <= X_IN1 + IN_TOL and s.low[1] >= Y_IN0 - IN_TOL
                and s.high[1] <= Y_IN1 + IN_TOL and s.high[2] <= RIM and s.low[2] >= TOP - 0.005)


def _labels(pts: np.ndarray) -> list[str]:
    """Per point: 'I' inside the box (below the rim), 'L'/'R' in the left/right slot opening, 'O' outside the box's
    outer footprint, 'X' anywhere else (over a wall or rim, above the box)."""
    out = []
    for x, y, z in pts:
        in_x = X_IN0 <= x <= X_IN1
        in_y = Y_IN0 <= y <= Y_IN1
        if in_x and in_y and z < RIM:
            out.append("I")
        elif x < X_OUT0 or x > X_OUT1 or y < Y_OUT0 or y > Y_OUT1:
            out.append("O")
        elif abs(x - SLOT_X) <= SLOT_W / 2 + SLOT_TOL and SILL - 0.002 <= z < RIM and Y_IN1 < y <= Y_OUT1:
            out.append("L")
        elif abs(x - SLOT_X) <= SLOT_W / 2 + SLOT_TOL and SILL - 0.002 <= z < RIM and Y_OUT0 <= y < Y_IN0:
            out.append("R")
        else:
            out.append("X")
    return out


def cord_exit(objects, name: str) -> str | None:
    """'left' / 'right' if the cord runs strip -> inside the box -> one slot -> outside (and stays outside, plug
    included), never crossing a wall or the rim anywhere else; None otherwise."""
    c = objects[name]
    poly = c.polyline()
    if not np.isfinite(poly).all():
        return None
    pts = _dense(poly)
    if c.end_plug is not None:
        pts = np.vstack([pts, np.asarray(c.end_plug)[None]])
    lab = "".join(_labels(pts))
    i = 0
    while i < len(lab) and lab[i] == "I":
        i += 1
    if i == 0 or i == len(lab) or lab[i] not in "LR":
        return None
    slot = lab[i]
    j = i
    while j < len(lab) and lab[j] == slot:
        j += 1
    if j == len(lab) or set(lab[j:]) != {"O"}:
        return None
    return "left" if slot == "L" else "right"


def released(objects) -> bool:
    pts = [_dense(objects[c.name].polyline()) for c in CORDS]
    plugs = [objects[c.name].end_plug for c in CORDS]
    allp = np.vstack(pts + [np.asarray(p)[None] for p in plugs if p is not None])
    for g in grippers():
        if float(np.min(np.linalg.norm(allp - g, axis=1))) < RELEASE_DIST:
            return False
        for name in (STRIP.name, LID.name, HANDLE.name):
            st = objects[name]
            if float(np.linalg.norm(np.maximum(np.maximum(st.low - g, g - st.high), 0.0))) < RELEASE_DIST:
                return False
    return True


def _finite(objects) -> bool:
    return all(np.isfinite(objects[n].pos).all() for n in (STRIP.name, LID.name)) and all(
        np.isfinite(objects[c.name].polyline()).all() for c in CORDS)


def score(objects) -> float:
    """0 unless, at the end: the strip lies wholly inside the box below the rim; the lid sits flat on the rim (every
    underside corner within 3 mm), centred within 1 cm and square within 5 deg; both grippers are at least 3.5 cm from
    the strip, cords, plugs and lid; and each cord runs from the strip inside the box out through one end slot and
    then stays outside the box (no cord over a wall or the rim). Then 1.0 if the power cord leaves through the right
    slot and the USB cable through the left slot; 0.5 if they leave through different slots but swapped; 0 if both
    use the same slot."""
    if not _finite(objects):
        return 0.0
    if not (strip_inside(objects) and lid_seated(objects) and released(objects)):
        return 0.0
    ex = {c.name: cord_exit(objects, c.name) for c in CORDS}
    if None in ex.values() or ex[PWR.name] == ex[USB.name]:
        return 0.0
    return 1.0 if all(ex[k] == SLOT_OF[k] for k in ex) else 0.5


def success(objects) -> bool:
    return bool(score(objects) == 1.0 and objects[LID.name].resting() and objects[STRIP.name].resting()
                and all(float(np.linalg.norm(objects[c.name].lin_vel)) < 0.02 for c in CORDS))


def distance(objects) -> float:
    """Strip centre to the box floor centre (m), plus 0.1 per cord not yet out through its own slot."""
    s = objects[STRIP.name].pos
    d = float(np.linalg.norm(s - np.array([BC[0], BC[1], FLOOR + SH / 2])))
    return d + 0.1 * sum(cord_exit(objects, c.name) != SLOT_OF[c.name] for c in CORDS)


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    return {
        "strip_inside": float(strip_inside(objects)),
        "power_cord_in_right_slot": float(cord_exit(objects, PWR.name) == "right"),
        "usb_cable_in_left_slot": float(cord_exit(objects, USB.name) == "left"),
        "lid_seated": float(lid_seated(objects)),
        "released": float(released(objects)),
    }


# Seeded variation for a later paper runner only; official trials use object_jitter=0, so nothing (cables included) is jittered.
# Strip and both cords move together (the welds are authored at the nominal relative layout): same dx/dy for all three.
VARIATION = {
    "obj:power_strip": {"dx": 0.015, "dy": 0.015, "yaw": 0.0},
    "cable:power_cord": {"dx": 0.015, "dy": 0.015, "yaw": 0.0},
    "cable:usb_cable": {"dx": 0.015, "dy": 0.015, "yaw": 0.0},
    "obj:box_lid": {"dx": 0.015, "dy": 0.015, "yaw": 0.1},
}

# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------------
# Right arm: grip the strip across its width at its middle, lift it over the near wall and lower it onto the box
# floor, centred; let go.
# Then both arms at once (each end slot is reachable only by the arm on its side): each picks its cord up from the
# table a little way out from the box, carries it to just outside its end slot with the wrist camera facing the slot,
# and sets it down there, so the cord runs strip -> slot -> table; let go.
# Left arm: pick the lid up by its handle, carry it over the box and lower it onto the rim; let go; both arms clear.

FINGER_CORD = -0.014  # pads ~28 mm apart: round a cord lying on the table
FINGER_WIDE = -0.040  # pads ~80 mm apart: around the 50 mm strip / the handle
HI = TOP + 0.14
PARK = {"left": (0.16, 0.26, TOP + 0.16), "right": (0.16, -0.26, TOP + 0.16)}


def _log(ctx, tag=""):
    st = ctx.states()
    s = st[STRIP.name]
    print(f"[t={ctx.t:5.2f}] {tag} strip={np.round(s.pos, 3)} inside={strip_inside(st)} "
          f"exit pwr={cord_exit(st, PWR.name)} usb={cord_exit(st, USB.name)} lid lift={lid_corner_lift(st) * 1000:.1f}mm "
          f"shift={lid_shift(st) * 1000:.1f}mm yaw={lid_yaw_deg(st):.1f} rel={released(st)} score={score(st)}",
          flush=True)
    for c in CORDS:
        lab = "".join(_labels(_dense(st[c.name].polyline())))
        print(f"    {c.name}: {lab[:90]}", flush=True)


def _place_strip(ctx):
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    s = ctx.states()[STRIP.name].pos
    yaw = math.pi / 2  # fingers close across the strip's width (x); wrist camera looks along +y, over the box
    g = np.array([s[0], s[1], s[2] + 0.010])
    yield from Rt.move((g[0], g[1], g[2] + 0.06), yaw=yaw, finger=FINGER_WIDE, duration=1.5)
    yield from Rt.move(g, yaw=yaw, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED, 0.6)
    off = g - ctx.states()[STRIP.name].pos  # TCP relative to the strip centre in the grip
    yield from Rt.move((g[0], g[1], RIM + 0.05 + off[2] + SH / 2), yaw=yaw, duration=1.2)
    tgt = np.array([BC[0], BC[1], RIM + 0.05 + off[2] + SH / 2])
    yield from Rt.move(tgt, yaw=yaw, duration=1.8)
    yield from Rt.wait(0.3)
    p = ctx.states()[STRIP.name].pos  # closed loop: centre the strip over the box floor
    tgt[:2] += BC - p[:2]
    yield from Rt.move(tgt, yaw=yaw, duration=0.6)
    off = tgt - ctx.states()[STRIP.name].pos
    yield from Rt.move((tgt[0], tgt[1], FLOOR + SH / 2 + off[2] + 0.002), yaw=yaw, duration=1.6)
    yield from Rt.wait(0.3)
    yield from Rt.grip(FINGER_WIDE, 0.5)
    yield from Rt.move((tgt[0], tgt[1], HI), yaw=yaw, duration=1.0)
    _log(ctx, "strip in")


def _outside(p, margin) -> bool:
    return bool(p[0] < X_OUT0 - margin or p[0] > X_OUT1 + margin or p[1] < Y_OUT0 - margin or p[1] > Y_OUT1 + margin)


def _seat_cord(ctx, arm, spec, *, drape=False):
    """Pick ``spec`` up off the table a little way out from the box and set it down just outside its end slot (or,
    ``drape``: just outside the near wall, over the rim)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    A = ctx.left if arm == "left" else ctx.right
    sgn = 1.0 if arm == "left" else -1.0
    poly = ctx.states()[spec.name].polyline()
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    k = int(np.searchsorted(s, 0.11))
    k = min(max(k, 1), len(poly) - 2)
    while k < len(poly) - 2 and not (_outside(poly[k], 0.035) and poly[k][2] < TOP + 0.02):
        k += 1
    p = poly[k]
    u = poly[k - 1] - poly[k + 1]  # along the cord towards the strip
    yaw0 = math.atan2(u[1], u[0])  # strip side of the cord leaves the fingers on the wrist camera's side
    yaw1 = -sgn * math.pi / 2  # at the slot: strip side (and camera) towards the box
    sk = float(s[k])
    if drape:
        tgt = np.array([X_OUT0 - 0.05, sgn * (Y_IN1 - 0.03)])
        yaw1 = 0.0
    else:
        y0 = float(poly[0][1])  # cord's start tip (strip end face), measured
        tgt = np.array([SLOT_X, y0 + sgn * min(0.92 * sk, 0.15)])
    print(f"[demo] {spec.name}: grasp s={sk:.3f} p={np.round(p, 4)} yaw {yaw0:.2f} -> {np.round(tgt, 3)} yaw {yaw1:.2f}",
          flush=True)
    yield from A.move((p[0], p[1], TOP + 0.08), yaw=yaw0, finger=FINGER_CORD, duration=1.4)
    yield from A.move((p[0], p[1], p[2] + 0.010), yaw=yaw0, duration=1.0)
    yield from A.grip(FINGER_CLOSED, 0.5)
    yield from A.move((p[0], p[1], RIM + 0.03), yaw=yaw0, duration=1.0)
    yield from A.move((tgt[0], tgt[1], RIM + 0.03), yaw=yaw1, duration=1.8)
    yield from A.move((tgt[0], tgt[1], TOP + 0.012), yaw=yaw1, duration=1.4)
    yield from A.wait(0.3)
    yield from A.grip(FINGER_OPEN, 0.4)
    yield from A.move((tgt[0], tgt[1], TOP + 0.10), yaw=yaw1, duration=0.8)
    _log(ctx, f"{spec.name} set")


def _place_lid(ctx, *, keep_hold=False):
    from yam_arena.constants import FINGER_CLOSED

    L = ctx.left
    yaw = -math.pi / 2
    h = ctx.states()[HANDLE.name].pos
    g = np.array([h[0], h[1], h[2] + HDL[2] / 2 + 0.002])
    yield from L.move((g[0], g[1], g[2] + 0.05), yaw=yaw, finger=FINGER_WIDE, duration=1.4)
    yield from L.move(g, yaw=yaw, duration=1.0)
    yield from L.grip(FINGER_CLOSED, 0.6)
    yield from L.move((g[0], g[1], TOP + 0.105), yaw=yaw, duration=1.2)
    above = np.array([BC[0], BC[1], HI])
    _log(ctx, "lid lifted")
    yield from L.move(above, yaw=yaw, duration=1.8)
    yield from L.wait(0.3)
    _log(ctx, "lid above")
    c = ctx.states()[LID.name].pos
    tcp = above.copy()
    tcp[:2] += BC - c[:2]
    dz = c[2] - (RIM + LID_T / 2)
    yield from L.move(tcp, yaw=yaw, duration=0.6)
    yield from L.move((tcp[0], tcp[1], tcp[2] - dz + 0.004), yaw=yaw, duration=2.0)
    yield from L.wait(0.3)
    _log(ctx, "lid down")
    if keep_hold:
        _log(ctx, "lid on, still held")
        return
    yield from L.grip(FINGER_WIDE, 0.5)
    yield from L.move((tcp[0], tcp[1], HI), yaw=yaw, duration=1.0)
    _log(ctx, "lid on")


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED

    mode = os.environ.get("CABLEBOX_DEMO", "correct")  # test only: correct | nothing | drape | no_lid | hold
    _log(ctx, "start")
    if mode == "nothing":
        for _ in range(5):
            yield from ctx.left.wait(1.0)
        _log(ctx, "end")
        return
    yield from _place_strip(ctx)

    def right():
        yield from _seat_cord(ctx, "right", PWR, drape=(mode == "drape"))
        yield from ctx.right.move(PARK["right"], yaw=0.0, finger=FINGER_CLOSED, duration=1.0)

    def left():
        yield from ctx.left.wait(0.5)
        yield from _seat_cord(ctx, "left", USB, drape=(mode == "drape"))

    yield from ctx.both(left(), right())
    if mode != "no_lid":
        yield from _place_lid(ctx, keep_hold=(mode == "hold"))
    if mode != "hold":
        yield from ctx.left.move(PARK["left"], yaw=0.0, finger=FINGER_CLOSED, duration=1.0)
    yield from ctx.left.wait(1.5)
    _log(ctx, "end")
