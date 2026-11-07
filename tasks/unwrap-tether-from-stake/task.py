"""Unwrap a grazing tether from a post by carrying its collar back round against the wrap.

A red tether runs from a fixed ground ring (on a short shock spring) to an 18 cm wooden post, goes about 210 deg
round it counter-clockwise seen from above, crosses over its own standing part and runs out to a blue collar
block on the right. A white circle (the grazing spot) is far out on the left. The post is too tall to lift the
turn off over the top, so the collar must be carried back round the post clockwise (the near side). Going the
short way across the far side winds more of the tether on, and it runs out before the circle. Once unwound, the
standing part still lies against the post and has to be laid clear for full score.

Reach forces a hand-over: the collar start is right-arm only and the circle is left-arm only.

Score (end state, 1.0 must hold 1 s): 0 if the tether ever went round twice, went over the post top, was
hauled on (ring spring drawn out > 2 cm) or the collar was towed by its tether (moved > 3 cm with no gripper
near); 0 unless at the end both grippers are clear, the tether no longer goes round the post and the collar
centre is in the circle; then 0.5, and 1.0 if every part of the tether and collar is at least 1 cm off the post.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "Unwrap the red tether from the wooden post and leave its blue collar in the white circle."

# -- post ---------------------------------------------------------------------------------------------------------
PC = np.array([0.36, 0.0])  # post axis (x, y)
RP = 0.035  # 70 mm round post
POST_H = 0.18
POST = Cylinder("post", pos=(float(PC[0]), float(PC[1]), TOP + POST_H / 2), radius=RP, height=POST_H,
                color=(0.45, 0.30, 0.16))

# -- tether: ground ring (fixed end) -> one turn round the post -> collar (free end) -----------------------------
R = 0.004  # 8 mm tether
TETHER_LEN = 0.80
COLLAR = Plug(size=(0.040, 0.024, 0.020), mass=0.020, color=(0.10, 0.30, 0.85))
TH_A = math.radians(-85.0)  # bearing of the ground ring from the post axis
DA = 0.25  # ring distance from the post axis
WRAP = math.radians(210.0)  # contact angle of the starting turn (counter-clockwise from the ring, seen from above)
_R0 = RP + R / math.cos(math.pi / 8) + 0.002
RW = _R0 + 0.02 ** 2 / (8 * _R0)  # centre-line radius of the turn (2 cm chord sagitta added)
A = PC + DA * np.array([math.cos(TH_A), math.sin(TH_A)])
PHI1 = TH_A + math.acos(RW / DA)  # where the standing part meets the post
T1 = PC + RW * np.array([math.cos(PHI1), math.sin(PHI1)])
_U = (T1 - A) / np.linalg.norm(T1 - A)
YAW_A = math.atan2(_U[1], _U[0])
Z0 = TOP + R + 0.0005
TETHER = cable("tether", start=(float(A[0]), float(A[1]), Z0), yaw=YAW_A, length=TETHER_LEN, radius=R,
               anchor_start=True, end_plug=COLLAR, cable_mass=0.075, color=(0.80, 0.10, 0.08),
               bend_stiffness=0.005, bend_damping=0.001, collision_sides=0)
SPRING_K = float(os.environ.get("TETHER_SPRING_K", 20.0))  # tie-out shock spring at the ring (N/m)


def _install_ring_spring() -> None:
    """Replace the kit's rigid world weld of the tether's ring end by the tie-out's swivel and shock spring: the end
    turns freely about the vertical and can be drawn out along the tether's lead by a soft spring (20 N/m). A hard
    pull stretches the spring instead of building unbounded tension in a line held at both ends (the scoring treats a
    stretch of more than 2 cm as a pull on the tether)."""
    orig = cablekit._spawn_cable
    if getattr(orig, "_ring_spring", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != TETHER.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdPhysics

        stage = get_current_stage()
        stage.RemovePrim(f"{prim_path}/anchor")
        j = UsdPhysics.Joint.Define(stage, f"{prim_path}/ring_spring")
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, TETHER.start)))
        j.CreateLocalRot0Attr(Gf.Quatf(math.cos(YAW_A / 2), 0.0, 0.0, math.sin(YAW_A / 2)))
        j.CreateLocalPos1Attr(Gf.Vec3f(-TETHER.seg_len / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateExcludeFromArticulationAttr(True)
        for ax in ("transY", "transZ", "rotX", "rotY"):  # locked (low > high)
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), ax)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "transX")  # spring along the lead; rotZ left free (swivel)
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(SPRING_K)
        drv.CreateDampingAttr(2.0)
        drv.CreateTargetPositionAttr(0.0)
        return prim

    spawn._ring_spring = True
    cablekit._spawn_cable = spawn


RING_SIZE = (0.030, 0.030, 0.014)
_rc = A - _U * (RING_SIZE[0] / 2 + 0.006)
RING = Box("ground_ring", pos=(float(_rc[0]), float(_rc[1]), TOP + RING_SIZE[2] / 2), size=RING_SIZE, yaw=YAW_A,
           color=(0.25, 0.25, 0.27))

# -- white circle (painted ring of thin strips, 1 mm tall) ----------------------------------------------------------
FC = np.array([0.50, 0.40])  # circle centre
TH_F = math.atan2(FC[1] - PC[1], FC[0] - PC[0])
DF = float(np.linalg.norm(FC - PC))
CIRC_R = 0.04  # inner radius of the painted ring
_NS = 16
CIRCLE = [Box(f"circle_{i}", pos=(float(FC[0] + (CIRC_R + 0.004) * math.cos(2 * math.pi * i / _NS)),
                                  float(FC[1] + (CIRC_R + 0.004) * math.sin(2 * math.pi * i / _NS)), TOP + 0.0005),
              size=(0.008, 2 * (CIRC_R + 0.008) * math.tan(math.pi / _NS) + 0.002, 0.001),
              yaw=2 * math.pi * i / _NS, color=(0.95, 0.95, 0.95)) for i in range(_NS)]

OBJECTS = [POST, RING, TETHER, *CIRCLE]
_install_ring_spring()


def _path() -> np.ndarray:
    st = np.linspace(A + _U * 0.03, T1, 14)
    angs = np.linspace(PHI1, PHI1 + WRAP, max(8, int(WRAP * RW / 0.003)))
    arc = PC + RW * np.c_[np.cos(angs), np.sin(angs)]
    t = np.array([-math.sin(angs[-1]), math.cos(angs[-1])])  # counter-clockwise tangent
    tail = arc[-1] + np.outer(np.linspace(0.01, 0.60, 60), t)
    rest = cableshape.over(np.vstack([arc, tail]), st, r_self=R, r_under=R, z_under=Z0,
                           where=lambda q: np.linalg.norm(q[:2] - PC) > RW + 0.010)
    return np.vstack([np.c_[st, np.full(len(st), Z0)], rest])


SHAPE_PATH = _path()

# -- episode state (latched violations) -----------------------------------------------------------------------------
_ST: dict = {"env": None, "wrong": False, "over": False, "snap": False, "drag": False, "nan": False, "kmax": 0,
             "ref": None}


def _reset_state() -> None:
    _ST.update(wrong=False, over=False, snap=False, drag=False, nan=False, kmax=0, ref=None)


def _shape(emb):
    _reset_state()
    _ST["env"] = emb._env
    cableshape.preshape(emb, TETHER, SHAPE_PATH, hold="ramp", settle_s=0)
    cableshape.settle(emb, 2.0)


cableshape.on_reset(_shape)


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_tether_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ST["env"] = self._env
        return base(self, *a, **k)

    _task_step._tether_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    env = _ST["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- scoring geometry ------------------------------------------------------------------------------------------------
RELEASE_DIST = 0.035  # each gripper TCP at least this far from every point of the tether and collar
CLEAR = 0.010  # "touches the post nowhere": every tether/collar point >= 1 cm from the post surface
HELD_DIST = 0.05  # a gripper TCP within 5 cm (top view) of the collar centre counts as handling the collar
DRAG_DIST = 0.03  # the collar moving more than 3 cm with no gripper at it = towed by its tether
SNAP_DIST = 0.02  # the tether's ring end drawn more than 2 cm out on its spring = the tether was hauled on
OVER_Z = TOP + POST_H - 0.01  # a tether point this high within 3 cm of the post = lifted over the top


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _line(objects) -> np.ndarray:
    """Tether centre line from the ring to the collar centre."""
    c = objects["tether"]
    pts = c.polyline()
    if c.end_plug is not None:
        pts = np.vstack([pts, c.end_plug[None, :]])
    return pts


def turns(objects) -> int:
    """Signed number of times the tether goes round the post (top view): the angle its centre line sweeps about the
    post axis from the ring to the collar, minus the angle the straight ring-to-collar line sweeps, in whole turns.
    + = counter-clockwise from the ring. The start layout is +1."""
    p = _line(objects)[:, :2] - PC
    ang = np.arctan2(p[:, 1], p[:, 0])
    sweep = float(((np.diff(ang) + math.pi) % (2 * math.pi) - math.pi).sum())
    chord = float((ang[-1] - ang[0] + math.pi) % (2 * math.pi) - math.pi)
    return int(round((sweep - chord) / (2 * math.pi)))


def post_gap(objects) -> float:
    """Smallest gap (m) between the post surface and the tether or collar surface."""
    pts = _dense(_line(objects))
    g = float((np.linalg.norm(pts[:, :2] - PC, axis=1) - RP - R).min())
    c = objects["tether"]
    if c.end_plug is not None:
        d = c.end_dir[:2] / (np.linalg.norm(c.end_dir[:2]) + 1e-12)
        n = np.array([-d[1], d[0]])
        hx, hy = COLLAR.size[0] / 2, COLLAR.size[1] / 2
        corners = [c.end_plug[:2] + sx * hx * d + sy * hy * n for sx in (-1, 1) for sy in (-1, 1)]
        corners += [c.end_plug[:2] + s * hx * d for s in (-1, 1)]
        g = min(g, float((np.linalg.norm(np.array(corners) - PC, axis=1) - RP).min()))
    return g


def released(objects) -> bool:
    pts = _dense(_line(objects))
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def in_circle(objects) -> bool:
    """Collar centre inside the white circle (within 4 cm of its centre) and lying on the table."""
    p = objects["tether"].end_plug
    return bool(p is not None and np.linalg.norm(p[:2] - FC) <= CIRC_R and p[2] <= TOP + 0.03)


def _finite(objects) -> bool:
    return bool(np.isfinite(objects["tether"].points).all())


def _track(objects) -> None:
    if not _finite(objects):
        _ST["nan"] = True
        return
    _ST["n"] = _ST.get("n", 0) + 1
    k = turns(objects)
    _ST["kmax"] = max(_ST["kmax"], abs(k))
    if abs(k) >= 2:
        _ST["wrong"] = True  # carried round the wrong way: a second turn
    if float(np.linalg.norm(objects["tether"].start_tip[:2] - A)) > SNAP_DIST:
        if not _ST["snap"] and os.environ.get("DEMO_DEBUG"):
            print(f"[track] snap latched at track call {_ST.get('n', 0)}", flush=True)
        _ST["snap"] = True  # the tie-out spring was stretched more than 2 cm: the tether was hauled on
    # the collar (the animal's end) may only be moved by a gripper at it, never towed along by its tether
    p = objects["tether"].end_plug[:2]
    if _ST["ref"] is None or any(float(np.linalg.norm(g[:2] - p)) <= HELD_DIST for g in grippers()):
        _ST["ref"] = p.copy()
    elif float(np.linalg.norm(p - _ST["ref"])) > DRAG_DIST:
        _ST["drag"] = True
    pts = _dense(_line(objects))
    if np.any((pts[:, 2] >= OVER_Z) & (np.linalg.norm(pts[:, :2] - PC, axis=1) <= RP + 0.03)):
        _ST["over"] = True  # lifted over the top of the post


def score(objects) -> float:
    """0 if at any time the tether went round the post twice or was lifted over its top (latched), if a gripper TCP is
    within 3.5 cm of the tether or collar at the end, if the tether still goes round the post, or if the collar centre
    is not inside the white circle. Otherwise 0.5, and 1.0 if every part of the tether and collar is >= 1 cm from the
    post."""
    _track(objects)
    if _ST["nan"] or _ST["wrong"] or _ST["over"] or _ST["snap"] or _ST["drag"]:
        return 0.0
    if not released(objects) or turns(objects) != 0 or not in_circle(objects):
        return 0.0
    return 1.0 if post_gap(objects) >= CLEAR else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["tether"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Turns left round the post plus the collar's distance from the circle centre (m)."""
    if not _finite(objects):
        return 1.0
    p = objects["tether"].end_plug
    d = float(np.linalg.norm(p[:2] - FC)) if p is not None else 1.0
    return abs(turns(objects)) + d


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    if not _finite(objects):
        return {"nan": 1}
    c = objects["tether"]
    g = grippers()
    return {
        "collar_grasped": int(c.end_plug is not None and any(float(np.linalg.norm(x - c.end_plug)) < 0.03 for x in g)),
        "unwound": int(turns(objects) == 0),
        "clear_of_post": int(post_gap(objects) >= CLEAR),
        "collar_in_circle": int(in_circle(objects)),
        "released": int(released(objects)),
        "no_violation": int(not (_ST["wrong"] or _ST["over"] or _ST["snap"] or _ST["drag"])),
    }


VARIATION = {"cable:tether": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}

TWIN_EXTRA = [
    {"name": "ring_spring", "kind": "joint", "real": "tie-out shock spring (about 20 N/m) between the ground ring and a swivel snap on the tether",
     "note": "sim: tether end slides along its lead on a 20 N/m spring and swivels freely about the vertical; other axes locked"},
    {"name": "collar_weld", "kind": "joint", "real": "tether end glued into a hole through the collar block"},
    {"name": "white_circle", "kind": "decal", "size_mm": [96, 96], "colour": "white",
     "note": "painted ring, inner radius 40 mm, 8 mm wide"},
]

# -- closed-loop scripted demo -------------------------------------------------------------------------------------
Z_CARRY = TOP + 0.035
RHO = 0.14
Z_HIGH = TOP + 0.08
_UF = (FC - A) / np.linalg.norm(FC - A)
STRAIGHT = tuple(FC + 0.03 * _UF)  # collar drawn out to here (inside the circle): the tether lies nearly straight from the ring


LAY_Y = (-0.07, 0.07)  # the right hand takes the tether here (just before and just past the post) ...
LAY_DX = 0.06  # ... and lays it this far further out from the post
SLACK_MIN = 0.03  # stop drawing out when the tether is this close to straight (ring to collar)


def slack(objects) -> float:
    """Tether length along its centre line (ring to collar centre) minus the straight ring-to-collar distance (m)."""
    p = _line(objects)
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum() - np.linalg.norm(p[-1] - p[0]))


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _arc(a0, a1, yaw_fn, r=RHO, r1=None, step=12.0, z=Z_CARRY):
    n = max(1, int(round(abs(a1 - a0) / step)))
    r1 = r if r1 is None else r1
    out = []
    for k, a in enumerate(np.linspace(a0, a1, n + 1)):
        rr = r + (r1 - r) * k / n
        out.append((PC[0] + rr * math.cos(math.radians(a)), PC[1] + rr * math.sin(math.radians(a)), z,
                    math.radians(yaw_fn(a))))
    return out


def demo_script(ctx):
    """Right arm takes the collar and carries it clockwise round the near side of the post (against the turn) to the
    middle; it sets it down there; the left arm picks it up, carries it on clockwise round the left side and lowers
    it into the circle; while the left hand holds it, the right hand lays the standing part clear of the post; both
    let go and move clear. TETHER_CHEAT = none | hold | wrong (short way across the far side) |
    right (right arm alone, as far round as it reaches) | pull (right arm hauls the collar straight out) | leftdrag
    (left arm alone: drags the tail into reach, then finishes) | lift (over the top of the post) | touch (no drawing
    out and no laying clear: the 0.5 tier)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("TETHER_CHEAT", "")

    def log(tag):
        st = ctx.states()
        c = st["tether"]
        if not _finite(st):
            print(f"[demo t={ctx.t:5.1f}] {tag}: NaN", flush=True)
            return
        print(f"[demo t={ctx.t:5.1f} n={_ST.get('n', 0)}] {tag}: collar={np.round(c.end_plug, 3)} turns={turns(st)} "
              f"gap={post_gap(st) * 100:.1f}cm circle={in_circle(st)} rel={released(st)} "
              f"latch={[k for k in ('wrong', 'over', 'snap', 'drag', 'nan') if _ST[k]]} score={score(st):.2f} "
              f"ringtip={np.linalg.norm(c.start_tip[:2] - A) * 1000:.0f}mm slack={slack(st) * 100:.0f}cm", flush=True)
        if os.environ.get("DEMO_DEBUG"):
            pc = np.round(c.points * 100).astype(int)
            print("   cm:", " ".join(f"{x},{y},{z - 75}" for x, y, z in pc), flush=True)

    def run(arm, path, dur=0.35):
        for x, y, z, yw in path:
            yield from arm.move((x, y, z), yaw=yw, duration=dur)

    def cw_yaw(a):  # clockwise travel heading a-90, wrist camera looking ahead
        return float(np.clip(((a - 90.0 + 180.0) % 360.0) - 180.0, -90.0, 90.0))

    def pick(arm):
        c = ctx.states()["tether"]
        p = c.end_plug
        gy = _grip_yaw(math.atan2(c.end_dir[1], c.end_dir[0]))
        yield from arm.move((p[0], p[1], TOP + 0.07), yaw=gy, finger=FINGER_OPEN, duration=1.5)
        p = ctx.states()["tether"].end_plug
        yield from arm.move((p[0], p[1], p[2] + 0.008), yaw=gy, duration=0.8)
        yield from arm.grip(FINGER_CLOSED)
        yield from arm.move((p[0], p[1], Z_CARRY), yaw=gy, duration=0.6)

    log("start")
    if cheat == "none":
        yield from L.wait(4.0)
        log("end")
        return

    def left_finish(straighten=True):
        yield from pick(L)
        log("left has collar")
        c1 = ctx.states()["tether"].end_plug
        a1 = math.degrees(math.atan2(c1[1] - PC[1], c1[0] - PC[0])) % 360.0
        yield from run(L, _arc(a1, math.degrees(TH_F), cw_yaw, r=RHO, r1=DF))
        log("left carried")
        if straighten:
            # draw the tether out from the ring until it is nearly straight (so it comes off the post), in small
            # steps watching the slack; the collar is then put down where this ends (inside the circle)
            start = np.array(L.tcp()[:2])
            for f in np.linspace(0.1, 1.0, 10):
                tgt = start + f * (np.array(STRAIGHT) - start)
                yield from L.move((tgt[0], tgt[1], Z_CARRY + 0.01), yaw=0.0, duration=0.3)
                st = ctx.states()
                if slack(st) < SLACK_MIN or post_gap(st) >= 0.025:
                    break
            yield from L.wait(0.3)
            log(f"drawn straight (slack {slack(ctx.states()) * 100:.1f} cm)")
            p = np.array(L.tcp()[:2])
            d = p - FC
            if np.linalg.norm(d) > 0.025:
                p = FC + d * 0.025 / np.linalg.norm(d)
            return p
        return FC

    if cheat == "leftdrag":
        # left arm alone: pinch the tail where it can reach it, drag the collar round into reach, then finish
        pts = ctx.states()["tether"].points
        rho = np.linalg.norm(pts[:, :2] - PC, axis=1)
        idx = [i for i in range(len(pts) - 6, len(pts)) if rho[i] >= 0.065]
        i = idx[0] if idx else len(pts) - 4
        p = pts[i]
        d = pts[min(i + 1, len(pts) - 1)] - pts[i - 1]
        gy = _grip_yaw(math.atan2(d[1], d[0]))
        log(f"leftdrag pinch seg {i} at {np.round(p, 3)}")
        yield from L.move((p[0], p[1], TOP + 0.06), yaw=gy, finger=FINGER_OPEN, duration=1.5)
        yield from L.move((p[0], p[1], p[2] + 0.010), yaw=gy, duration=0.8)
        yield from L.grip(FINGER_CLOSED)
        yield from L.move((0.26, -0.08, TOP + 0.02), yaw=gy, duration=1.5)
        yield from L.move((0.17, -0.10, TOP + 0.02), yaw=gy, duration=1.5)
        log("dragged")
        yield from L.grip(FINGER_OPEN, duration=0.5)
        yield from L.move((0.17, -0.10, TOP + 0.10), yaw=gy, duration=0.6)
        put = yield from left_finish()
    else:
        yield from pick(Rt)
        log("right has collar")
    if cheat == "lift":
        # try to take the turn off over the top of the post
        yield from Rt.move((PC[0] + 0.08, -0.06, TOP + 0.20), yaw=0.0, duration=2.0)
        yield from Rt.move((PC[0], 0.0, TOP + 0.22), yaw=0.0, duration=1.5)
        yield from Rt.move((PC[0] - 0.10, -0.04, TOP + 0.20), yaw=0.0, duration=1.5)
        log("lifted")
        yield from Rt.move((PC[0] - 0.12, -0.06, TOP + 0.03), yaw=0.0, duration=1.5)
        yield from Rt.grip(FINGER_OPEN, duration=0.5)
        yield from Rt.move((0.20, -0.33, TOP + 0.15), yaw=0.0, duration=1.0)
        yield from Rt.wait(1.5)
        log("end")
        return
    if cheat == "pull":
        yield from Rt.move((0.30, -0.30, Z_CARRY), yaw=0.0, duration=2.0)
        yield from Rt.move((0.10, -0.55, Z_CARRY + 0.03), yaw=0.0, duration=2.0)
        log("pulled")
        yield from Rt.grip(FINGER_OPEN, duration=0.5)
        yield from Rt.move((0.20, -0.33, TOP + 0.15), yaw=0.0, duration=1.0)
        yield from Rt.wait(1.5)
        log("end")
        return

    c0 = ctx.states()["tether"].end_plug
    a0 = math.degrees(math.atan2(c0[1] - PC[1], c0[0] - PC[0]))
    if cheat == "leftdrag":
        pass
    elif cheat == "wrong":
        path = _arc(a0, 0.0, lambda a: 0.0, r=0.20, r1=0.09)
        yield from run(Rt, path)
        hand = path[-1]
    else:
        path = _arc(a0, -180.0, cw_yaw, r=0.20, r1=RHO, z=Z_HIGH)  # high: the tail lifts clear of the standing part
        if cheat == "right":  # right arm alone: on round the left side as far as it reaches
            path += _arc(180.0, 100.0, cw_yaw, r=RHO)[1:]
        yield from Rt.move(path[0][:3], yaw=path[0][3], duration=0.8)
        yield from run(Rt, path[1:], dur=0.45)
        hand = path[-1]
    if cheat != "leftdrag":
        log("right carried")
    if cheat != "leftdrag":
        yield from Rt.move((hand[0], hand[1], TOP + 0.020), yaw=hand[3], duration=0.6)
        yield from Rt.grip(FINGER_OPEN, duration=0.5)
        yield from Rt.move((hand[0], hand[1], TOP + 0.10), yaw=hand[3], duration=0.6)
        yield from Rt.move((0.20, -0.33, TOP + 0.15), yaw=0.0, duration=1.0)
        log("set down")
    if cheat == "right":
        yield from Rt.wait(1.5)
        log("end")
        return
    if cheat == "wrong":
        yield from pick(L)
        log("left has collar")
        # on towards the circle the short way: the tether is now wound about 340 deg on, and reaching the circle this
        # way would need about 91 cm of it (it is 80 cm), so go as far as it allows and put the collar down there
        path = _arc(0.0, 40.0, lambda a: 0.0, r=0.09, r1=0.10)
        yield from run(L, path)
        log("left carried (tether ran out)")
        put = np.array(path[-1][:2])
    elif cheat != "leftdrag":
        put = yield from left_finish(straighten=cheat != "touch")
    yield from L.move((put[0], put[1], TOP + 0.04), yaw=0.0, duration=1.2)
    yield from L.move((put[0], put[1], TOP + 0.020), yaw=0.0, duration=0.6)
    log("in circle (held)")
    if cheat not in ("touch", "wrong", "leftdrag"):
        # the standing part still lies against the far side of the post: while the left hand holds the collar in
        # the circle, the right hand takes the tether just before and just past the post and lays it up to 6 cm further out
        for ly in LAY_Y:
            pts = ctx.states()["tether"].points
            cand = [i for i in range(2, len(pts) - 2) if pts[i, 0] > PC[0] and abs(pts[i, 1] - ly) < 0.03]
            i = min(cand, key=lambda j: abs(pts[j, 1] - ly)) if cand else int(np.argmin(np.abs(pts[:, 1] - ly)))
            q = pts[i]
            d = pts[i + 1] - pts[i - 1]
            gy = _grip_yaw(math.atan2(d[1], d[0]))
            yield from Rt.move((q[0], q[1], TOP + 0.06), yaw=gy, finger=FINGER_OPEN, duration=1.0)
            yield from Rt.move((q[0], q[1], q[2] + 0.010), yaw=gy, duration=0.7)
            yield from Rt.grip(FINGER_CLOSED)
            dx = 0.0
            while dx < LAY_DX - 1e-6:  # out in 1 cm steps, stopping before the ring spring is drawn out
                dx += 0.01
                yield from Rt.move((q[0] + dx, q[1], TOP + 0.015), yaw=gy, duration=0.25)
                if float(np.linalg.norm(ctx.states()["tether"].start_tip[:2] - A)) > 0.010:
                    break
            yield from Rt.grip(FINGER_OPEN, duration=0.4)
            yield from Rt.move((q[0] + dx, q[1], TOP + 0.08), yaw=gy, duration=0.5)
            log(f"laid out {dx * 100:.0f} cm at y {ly * 100:.0f}")
        yield from Rt.move((0.20, -0.33, TOP + 0.15), yaw=0.0, duration=1.0)
        log("laid clear of the post")
    if cheat == "hold":
        yield from L.wait(3.0)
        log("end (held)")
        return
    yield from L.grip(FINGER_OPEN, duration=0.5)
    yield from L.move((put[0], put[1], TOP + 0.12), yaw=0.0, duration=0.8)
    yield from L.move((0.20, 0.33, TOP + 0.15), yaw=0.0, duration=1.0)
    yield from L.wait(1.5)
    log("end")
