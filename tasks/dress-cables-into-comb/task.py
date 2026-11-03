"""On the station table: three 6 mm round cables (red, blue, yellow PVC jackets, 38 cm long) come out of a grey
cable-entry block fixed at the left of the table. The block is turned so its exit face points to the front right
(55 deg right of straight ahead); the three cables leave it 3.5 cm apart, red nearest the robot, then blue, then
yellow. Blue and yellow lie straight and parallel towards the front right; red leaves its hole 20 deg further
to the left and lies crossed over both of them, so at the free ends the order is reversed. In front of the robot,
on the midline, a white comb is fixed to the table: a 9 cm x 2 cm x 4 mm base with four teeth 12 mm tall, leaving
three open-top slots 9 mm wide whose axis runs left to right. A coloured tab at the right-hand mouth of each slot
marks its cable: red slot nearest the robot, then blue, then yellow (the same order as the holes). Each cable
must be pressed down into its own slot, with no two cables crossing between the block and the comb.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "Press the red, blue and yellow cables into the cable comb so each sits in its matching coloured slot, without crossing each other."
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import numpy as np  # noqa: E402
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- cables and entry block ----------------------------------------------------------------------------------
R = 0.003  # 6 mm cable
CABLE_LEN = 0.38
NAMES = ("red", "blue", "yellow")
COLORS = {"red": (0.80, 0.08, 0.06), "blue": (0.08, 0.22, 0.80), "yellow": (0.95, 0.80, 0.08)}
NORMAL = math.radians(-55)  # exit direction of the block face
TANGENT = np.array([math.cos(NORMAL + math.pi / 2), math.sin(NORMAL + math.pi / 2)])
FACE_C = np.array([0.22, 0.15])  # middle (blue) exit, xy
PITCH_EXIT = 0.035
EXITS = {n: FACE_C + (i - 1) * PITCH_EXIT * TANGENT for i, n in enumerate(NAMES)}
YAWS = {"red": NORMAL + math.radians(20), "blue": NORMAL, "yellow": NORMAL}
Z_LOW = TOP + R + 0.0005
Z_HIGH = Z_LOW + 2 * R + 0.001  # red lies over blue and yellow
ZS = {"red": Z_HIGH, "blue": Z_LOW, "yellow": Z_LOW}
CABLES = {n: cable(n, start=(float(EXITS[n][0]), float(EXITS[n][1]), ZS[n]), yaw=YAWS[n], length=CABLE_LEN,
                   radius=R, anchor_start=True, color=COLORS[n], bend_stiffness=0.01, bend_damping=0.001)
          for n in NAMES}

BLOCK_DEPTH, BLOCK_W, BLOCK_H = 0.04, 0.10, 0.03
_bc = FACE_C - (BLOCK_DEPTH / 2 + 0.004) * np.array([math.cos(NORMAL), math.sin(NORMAL)])
BLOCK = Box("entry_block", pos=(float(_bc[0]), float(_bc[1]), TOP + BLOCK_H / 2), size=(BLOCK_DEPTH, BLOCK_W, BLOCK_H),
            yaw=NORMAL, color=(0.45, 0.47, 0.50))

# -- comb (slots run along -y, i.e. left to right) ---------------------------------------------------------------
SLOT_X = {"red": 0.21, "blue": 0.24, "yellow": 0.27}
SLOT_W = 0.009
COMB_Y = 0.0  # centre of the comb along the cable direction
COMB_D = 0.02  # comb depth along the slots
BASE_H = 0.004
TOOTH_H = 0.012
COMB_X0 = SLOT_X["red"] - SLOT_W / 2 - 0.0105
COMB_X1 = SLOT_X["yellow"] + SLOT_W / 2 + 0.0105
FLOOR_Z = TOP + BASE_H
LIP_Z = TOP + BASE_H + TOOTH_H
WHITE = (0.93, 0.93, 0.92)


def _comb() -> list[Box]:
    parts = [Box("comb_base", pos=((COMB_X0 + COMB_X1) / 2, COMB_Y, TOP + BASE_H / 2),
                 size=(COMB_X1 - COMB_X0, COMB_D, BASE_H), color=WHITE)]
    edges = [COMB_X0] + [v for n in NAMES for v in (SLOT_X[n] - SLOT_W / 2, SLOT_X[n] + SLOT_W / 2)] + [COMB_X1]
    for k in range(4):
        a, b = edges[2 * k], edges[2 * k + 1]
        parts.append(Box(f"comb_tooth{k}", pos=((a + b) / 2, COMB_Y, TOP + BASE_H + TOOTH_H / 2),
                         size=(b - a, COMB_D, TOOTH_H), color=WHITE))
    for n in NAMES:  # coloured tab at the right-hand (exit) mouth of each slot
        parts.append(Box(f"tab_{n}", pos=(SLOT_X[n], COMB_Y - COMB_D / 2 - 0.0075, TOP + 0.001),
                         size=(SLOT_W, 0.015, 0.002), color=COLORS[n]))
    return parts


COMB = _comb()
OBJECTS = [BLOCK, *CABLES.values(), *COMB]

# -- scoring --------------------------------------------------------------------------------------------------
SEATED_Z = LIP_Z - R + 0.001  # cable centre: its top no more than 1 mm above the tooth tops
PER_CABLE = 0.3
NO_CROSS_BONUS = 0.1


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def slot_of(objects, name: str) -> str | None:
    """Which slot the cable is seated in (passes through the comb inside one slot, below the lip, and comes out
    at least 2 cm past the comb's far face), or None."""
    if not np.isfinite(objects[name].points).all():
        return None
    pts = _dense(objects[name].polyline())
    inside = (pts[:, 0] > COMB_X0) & (pts[:, 0] < COMB_X1) & (np.abs(pts[:, 1] - COMB_Y) < COMB_D / 2)
    if not inside.any():
        return None
    p = pts[inside]
    if p[:, 2].max() > SEATED_Z:
        return None
    if not (pts[:, 1] < COMB_Y - COMB_D / 2 - 0.02).any() or not (pts[:, 1] > COMB_Y + COMB_D / 2 + 0.02).any():
        return None
    for n in NAMES:
        if np.all(np.abs(p[:, 0] - SLOT_X[n]) < SLOT_W / 2):
            return n
    return None


def _lead(objects, name: str) -> np.ndarray:
    """The cable's polyline (xy) from the block up to where it first reaches the comb's near face."""
    poly = objects[name].polyline()[:, :2]
    near = COMB_Y + COMB_D / 2
    for i in range(1, len(poly)):
        if poly[i, 1] <= near:
            a, b = poly[i - 1], poly[i]
            t = (a[1] - near) / max(a[1] - b[1], 1e-9)
            return np.vstack([poly[:i], a + t * (b - a)])
    return poly


def _seg_cross(p1, p2, q1, q2) -> bool:
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    d1, d2 = orient(q1, q2, p1), orient(q1, q2, p2)
    d3, d4 = orient(p1, p2, q1), orient(p1, p2, q2)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def crossing_pairs(objects) -> set[tuple[str, str]]:
    """Cable pairs whose top-view paths cross between the block and the comb."""
    if not all(np.isfinite(objects[n].points).all() for n in NAMES):
        return set()
    leads = {n: _lead(objects, n) for n in NAMES}
    pairs = set()
    for i, a in enumerate(NAMES):
        for b in NAMES[i + 1:]:
            pa, pb = leads[a], leads[b]
            if any(_seg_cross(pa[k], pa[k + 1], pb[m], pb[m + 1])
                   for k in range(len(pa) - 1) for m in range(len(pb) - 1)):
                pairs.add((a, b))
    return pairs


def crossings(objects) -> int:
    return len(crossing_pairs(objects))


def dressed(objects) -> list[str]:
    """Cables seated in the slot of their own colour; none count while any two cables cross between the block
    and the comb (untangling comes first)."""
    if not all(np.isfinite(objects[n].points).all() for n in NAMES):
        return []  # a diverged (NaN) simulation never scores
    if crossing_pairs(objects):
        return []
    return [n for n in NAMES if slot_of(objects, n) == n]


def success(objects) -> bool:
    return len(dressed(objects)) == 3


def score(objects) -> float:
    """0 while any two cables cross between the block and the comb; otherwise 0.3 per cable seated in its own slot
    (wrong slot or not seated: 0 for that cable) and +0.1 when all three are."""
    n = len(dressed(objects))
    return round(PER_CABLE * n + (NO_CROSS_BONUS if n == 3 else 0.0), 6)


def distance(objects) -> float:
    """Cables not yet dressed (0..3)."""
    return float(3 - len(dressed(objects)))


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------


def _at(objects, name: str, a: float) -> tuple[np.ndarray, float]:
    """Live point at arclength ``a`` from the block, and the cable's heading there."""
    poly = objects[name].polyline()
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    p = np.array([np.interp(a, s, poly[:, k]) for k in range(3)])
    i = min(max(int(np.searchsorted(s, a)), 1), len(poly) - 1)
    d = poly[i] - poly[i - 1]
    return p, math.atan2(d[1], d[0])


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


HOLD_L_Y = COMB_Y + 0.045  # left hand holds the cable this far before the comb (anchor side)
HOLD_R_Y = COMB_Y - 0.12  # right hand this far past it (well apart from the left hand)


def _arc_to_left_hold(name: str) -> float:
    """Arclength from the block to the left-hand hold point on the seated route (with ~1 cm slack)."""
    spec = CABLES[name]
    e = np.asarray(spec.start[:2])
    s0 = e + spec.seg_len * np.array([math.cos(spec.yaw), math.sin(spec.yaw)])
    hold = np.array([SLOT_X[name], HOLD_L_Y])
    return spec.seg_len + float(np.linalg.norm(hold - s0)) + 0.01


def _debug(ctx, tag):
    import os

    if os.environ.get("DEMO_DEBUG"):
        st = ctx.states()
        print("DEBUG", tag, {n: slot_of(st, n) for n in NAMES}, "crossings", sorted(crossing_pairs(st)), "score", score(st))
        for n in NAMES:
            print("   ", n, np.round(st[n].points, 3).tolist())


DEMO_GRASP_L = {"red": 0.15}
DEMO_GAP = {"red": 0.155}  # red's right-hand grasp would otherwise be out of reach
GRASP_OPEN = 0.22  # grasp with the pads ~21 mm apart (fully open: 94 mm) so a neighbouring cable isn't caught
DEMO_ORDER = ("red", "blue", "yellow")  # top cable first: red lies over the others


def demo_script(ctx):
    """Full demo. COMB_CHEAT=nothing|skip_red|hover runs an unintended strategy instead (each must score 0):
    nothing - arms stay still; skip_red - dress blue and yellow the normal way but leave red crossed over them;
    hover - hold red stretched over its slot (above the teeth) to the end instead of pressing it in."""
    import os

    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    cheat = os.environ.get("COMB_CHEAT", "")

    L, Rt = ctx.left, ctx.right
    tcp_over = 0.010
    hi = TOP + 0.06
    carry = LIP_Z + R + tcp_over + 0.012  # carried cable centre ~1.2 cm above the tooth tops
    place = FLOOR_Z + R + tcp_over + 0.001  # cable centre on the slot floor

    yield from ctx.both(L.wait(0.5), Rt.wait(0.5))  # let the cables settle
    _debug(ctx, "start")
    if cheat == "nothing":
        yield from ctx.both(L.wait(30.0), Rt.wait(30.0))
        _debug(ctx, "end")
        return
    order = {"skip_red": ("blue", "yellow"), "hover": ("red",)}.get(cheat, DEMO_ORDER)
    for name in order:
        st = ctx.states()
        # left grasp: at the hold point's arclength, except red, which is gripped midway between its crossings
        # over blue (~9 cm) and yellow (~17 cm) so the fingers can't catch a second cable
        aL = DEMO_GRASP_L.get(name, _arc_to_left_hold(name))
        gap = DEMO_GAP.get(name, HOLD_L_Y - HOLD_R_Y + 0.004)  # cable length between the two hands
        aR = aL + gap
        hold_r = HOLD_L_Y - (gap - 0.004)
        pL, hL = _at(st, name, aL)
        pR, hR = _at(st, name, aR)
        x = SLOT_X[name]

        def pick(arm, p, h, up=hi):
            gy = _grip_yaw(h)
            yield from arm.move((p[0], p[1], hi), yaw=gy, finger=GRASP_OPEN * FINGER_OPEN, duration=1.2)
            yield from arm.move((p[0], p[1], p[2] + tcp_over), yaw=gy, duration=0.8)
            yield from arm.grip(FINGER_CLOSED)
            yield from arm.move((p[0], p[1], up), yaw=gy, duration=0.8)

        # one hand after the other (both grasp points are near the midline and the arms would meet): the right hand
        # first takes the far point and lifts it a little, then the left hand takes the near point
        yield from pick(Rt, pR, hR, TOP + 0.03)
        pL, hL = _at(ctx.states(), name, aL)
        yield from pick(L, pL, hL)
        _debug(ctx, f"{name} picked: L tcp {np.round(L.tcp(), 3)} cable {np.round(_at(ctx.states(), name, aL)[0], 3)}; "
                    f"R tcp {np.round(Rt.tcp(), 3)} cable {np.round(_at(ctx.states(), name, aR)[0], 3)}")

        def both_to(z, dx, dur):
            yield from ctx.both(L.move((x + dx, HOLD_L_Y, z), yaw=-math.pi / 2, duration=dur),
                                Rt.move((x + dx, hold_r, z), yaw=math.pi / 2, duration=dur))

        def offset():
            """Slot x minus the cable's x over the comb (the span between the hands)."""
            cs = ctx.states()[name]
            pts, arc = cs.points, cs.arclength()
            near = pts[(np.abs(pts[:, 1] - COMB_Y) < 0.03) & (arc > aL) & (arc < aR)]
            return float(x - near[:, 0].mean()) if len(near) else 0.0

        def press(arm, y, yaw):  # closed fingertips press the cable flat just outside the comb
            yield from arm.move((x, y, hi), yaw=yaw, duration=0.6)
            yield from arm.move((x, y, TOP + 2 * R + 0.013 - 0.001), yaw=yaw, duration=0.6)
            yield from arm.move((x, y, hi), yaw=yaw, duration=0.5)

        # both grips close across the -y lane; the arms reach it from opposite sides (left -90 deg, right +90 deg)
        yield from both_to(hi, 0.0, 1.2)
        # closed loop: the cable sits off-centre in the fingers (and rolls a little as they move), so measure where
        # it hangs over the comb and shift both hands until it is right over the slot, then lower it in
        dx = 0.0
        pre = LIP_Z + 0.006  # TCP height: the held cable just above the tooth tops
        low = TOP + 0.014  # fingertips 1 mm above the table; the cable is held close to the TCP
        for z in (carry, pre, pre):
            dx = float(np.clip(dx + offset(), -0.03, 0.03))
            yield from both_to(z, dx, 0.6)
        _debug(ctx, f"{name} over slot, dx {dx:.4f}, residual {offset():.4f}")
        if cheat == "hover":  # hold it above the slot instead of pressing it in
            yield from ctx.both(L.wait(20.0), Rt.wait(20.0))
        else:
            for attempt in range(3):
                yield from both_to(low, dx, 0.8)
                _debug(ctx, f"{name} lowered, slot {slot_of(ctx.states(), name)}")
                if slot_of(ctx.states(), name) == name:
                    break
                yield from both_to(pre, dx, 0.6)  # missed: lift above the teeth, re-centre, try again
                dx = float(np.clip(dx + offset(), -0.03, 0.03))
                yield from both_to(pre, dx, 0.5)
            yield from ctx.both(L.grip(0.3 * FINGER_OPEN, 0.4), Rt.grip(0.3 * FINGER_OPEN, 0.4))  # pads ~28 mm apart
            yield from both_to(hi, dx, 0.6)
        _debug(ctx, f"{name} laid")
        if cheat != "hover" and slot_of(ctx.states(), name) != name:  # not in yet: press it down beside the comb
            yield from press(L, COMB_Y + COMB_D / 2 + 0.012, -math.pi / 2)
            yield from ctx.both(L.move((0.20, 0.20, hi), yaw=0.0, duration=0.8),
                                press(Rt, COMB_Y - COMB_D / 2 - 0.030, math.pi / 2))
        else:
            yield from ctx.both(L.move((0.20, 0.20, hi), yaw=0.0, duration=0.8),
                                Rt.move((0.20, -0.20, hi), yaw=0.0, duration=0.8))
        # tidy: excess cable between the block and the comb bulges towards the next cable's lane (+x); pick the
        # bulge up and lay it back on the near side so the next cable cannot end up lying over it
        cs = ctx.states()[name]
        lead_idx = np.where((cs.points[:, 1] > COMB_Y + COMB_D / 2 + 0.015) & (cs.arclength() > 0.03))[0]
        if cheat != "hover" and len(lead_idx):
            k = lead_idx[np.argmax(cs.points[lead_idx, 0])]
            apex = cs.points[k]
            if apex[0] > x + 0.012:
                _, ha = _at(ctx.states(), name, float(cs.arclength()[k]))
                ga = _grip_yaw(ha)
                tgt = (x - 0.008, apex[1])
                yield from L.move((apex[0], apex[1], hi), yaw=ga, finger=GRASP_OPEN * FINGER_OPEN, duration=1.0)
                yield from L.move((apex[0], apex[1], apex[2] + tcp_over), yaw=ga, duration=0.7)
                yield from L.grip(FINGER_CLOSED, 0.5)
                yield from L.move((apex[0], apex[1], TOP + 0.03), yaw=ga, duration=0.4)
                yield from L.move((tgt[0], tgt[1], TOP + 0.03), yaw=ga, duration=0.8)
                yield from L.move((tgt[0], tgt[1], TOP + 0.014), yaw=ga, duration=0.5)
                yield from L.grip(GRASP_OPEN * FINGER_OPEN, 0.4)
                yield from L.move((tgt[0], tgt[1], hi), yaw=ga, finger=FINGER_CLOSED, duration=0.6)
                _debug(ctx, f"{name} tidied (apex x {apex[0]:.3f})")
        _debug(ctx, name)
    if cheat:
        yield from ctx.both(L.wait(2.0), Rt.wait(2.0))
        _debug(ctx, "end")
        return

    yield from ctx.both(L.move((0.25, 0.25, hi), yaw=0.0, duration=1.0), Rt.move((0.25, -0.25, hi), yaw=0.0, duration=1.0))
