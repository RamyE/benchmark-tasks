"""Physics-lab sonometer: stand two bridges under a weight-tensioned cord at the marks.

A white sonometer box (6 x 66 x 8.5 cm, long side along y) stands in front of the arms at x 30 cm, with a dark
board just behind it so the lifted cord shows from the top camera. An orange 8 mm cord is tied to a peg at its
left end (y +31.7 cm), lies along the centre line of the box top, and runs over a smooth rounded steel lip (2 cm
diameter) at the right end (centre y -35 cm) down to a 40 g blue weight that hangs free about 1 cm above the table.
The hanging weight keeps the cord taut and flat on the bare top. Two blue lines across the top mark y +8 and y -8 cm.
Two free black bridges (7 x 1 x 3.8 cm, 15 g, with a 14 mm wide, 3 mm deep notch across the middle of the top) stand
on the table in front of the box at x 20, y +12 and y -12 cm. Set right, they hold the cord underside about 3.5 cm
above the box top between them.

The cord falls flat whenever it is let go, and a bridge pushed against the lying cord only bows it sideways (vertical
faces, nothing to ride up), so one hand must hold the cord up while the other stands a bridge under it. The box top
(6 cm) is narrower than a bridge is long (7 cm), so a bridge can only stand on it across the cord line.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants: SONO_DEMO=correct (default) | nothing | hold | onearm | drop | one.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the cord below is simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Set up the sonometer: stand the two black bridges on the box under the orange cord, one at each blue "
               "mark, so the cord rests in the notches of both bridges and the weight still hangs free. Then let go.")

WHITE = (0.92, 0.92, 0.90)
ORANGE = (1.00, 0.42, 0.02)
BLACK = (0.06, 0.06, 0.07)
BLUE = (0.02, 0.12, 0.75)
STEEL = (0.62, 0.63, 0.66)
DARKBLUE = (0.10, 0.18, 0.55)

AX = 0.30  # cord line (x)
D = 0.085  # box top above the table
BOX_X = (0.27, 0.33)
BOX_Y = (-0.34, 0.32)
DECK = Box("sono_box", pos=(AX, 0.0, TOP + D / 2), size=(BOX_X[1] - BOX_X[0], BOX_Y[1] - BOX_Y[0], D), color=WHITE)
BOARD_H = 0.04
BOARD = Box("scale_board", pos=(BOX_X[1] + 0.017, (BOX_Y[0] + BOX_Y[1]) / 2, TOP + (D + BOARD_H) / 2),
            size=(0.006, BOX_Y[1] - BOX_Y[0], D + BOARD_H), color=(0.10, 0.10, 0.12))  # dark board just behind the box, clear of the bridge ends (x <= 0.335)
PEG = Box("tie_peg", pos=(AX, 0.317, TOP + D + 0.008), size=(0.012, 0.008, 0.016), color=STEEL)
LIP_R = 0.010
LIP_Y = -0.350
LIP_Z = D - LIP_R  # lip centre above the table (lip top flush with the box top)
MARKS_Y = (0.08, -0.08)
DECK_FRICTION = 0.3
LIP_FRICTION = 0.05

R = 0.004
CORD_LEN = 0.73
CORD_START = np.array([AX, 0.302, TOP + D + R + 0.0005])
CORD = cable("cord", start=tuple(CORD_START.tolist()), yaw=-math.pi / 2, length=CORD_LEN, radius=R, segment=0.02,
             color=ORANGE, cable_mass=0.026, bend_stiffness=0.003, bend_damping=0.0005)
BAG_W, BAG_H, BAG_M = 0.020, 0.012, 0.04
SWIVEL_DZ = BAG_H / 2 + R + 0.0005
_END = CORD_START + CORD_LEN * np.array([0.0, -1.0, 0.0])
BAG = Box("weight", pos=(AX, float(_END[1]), float(_END[2] - SWIVEL_DZ)), size=(BAG_W, BAG_W, BAG_H), mass=BAG_M,
          color=DARKBLUE)

# bridges: body + two cheeks (welded) leaving a notch across the middle of the top
BR_L, BR_T, BR_H = 0.070, 0.010, 0.035  # body (x, y, z)
NOTCH_W, NOTCH_D = 0.014, 0.003
CHEEK_L = (BR_L - NOTCH_W) / 2
CHEEK_DX = NOTCH_W / 2 + CHEEK_L / 2
BR_TOP = BR_H + NOTCH_D  # 3.8 cm overall
BR_START = {"bridge_a": (0.20, 0.12), "bridge_b": (0.20, -0.12)}
BRIDGES = []
CHEEKS = {}
for _n, (_x, _y) in BR_START.items():
    BRIDGES.append(Box(_n, pos=(_x, _y, TOP + BR_H / 2 + 0.001), size=(BR_L, BR_T, BR_H), mass=0.013, color=BLACK))
    CHEEKS[_n] = [Box(f"{_n}_cheek_{s}", pos=(_x + sg * CHEEK_DX, _y, TOP + BR_H + NOTCH_D / 2 + 0.001),
                      size=(CHEEK_L, BR_T, NOTCH_D), mass=0.001, color=BLACK) for s, sg in (("f", -1), ("b", 1))]
BRIDGE_NAMES = [b.name for b in BRIDGES]

OBJECTS = [DECK, BOARD, PEG, *BRIDGES, *[c for v in CHEEKS.values() for c in v], BAG, CORD]

TWIN_EXTRA = [
    {"name": "lip", "kind": "body", "size_mm": [60, 20, 20], "colour": "steel", "pose": [AX, LIP_Y, LIP_Z],
     "real": "20 mm polished steel rod along x, fixed across the box's right end, top flush with the box top",
     "note": "sim: fixed smooth cylinder (friction 0.05) standing in for the sonometer's pulley"},
    {"name": "marks", "kind": "decal", "size_mm": [60, 5], "colour": "blue", "pose": [AX, 0.08, D]},
    {"name": "marks", "kind": "decal", "size_mm": [60, 5], "colour": "blue", "pose": [AX, -0.08, D]},
    {"name": "cord_tie", "kind": "joint", "real": "cord knotted to the steel peg at the box's left end",
     "note": "sim: ball joint at the peg"},
    {"name": "hanger_swivel", "kind": "joint", "real": "S-hook on the weight hanger's top loop"},
    {"name": "bridge_cheeks", "kind": "body", "real": "each bridge is one 3D-printed PLA part, notch cut in the top",
     "note": "sim: body + two welded cheeks"},
]


def _install_spawn_hook() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_sono_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORD.name:
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.schemas import CollisionPropertiesCfg
        from isaaclab.sim.utils import bind_physics_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        q0 = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        qy = Gf.Quatf(math.cos(-math.pi / 4), 0.0, 0.0, math.sin(-math.pi / 4))

        def ball(path, body0, body1, p0, r0, p1):
            j = UsdPhysics.Joint.Define(stage, path)
            if body0:
                j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
            j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
            j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, p0)))
            j.CreateLocalRot0Attr(r0)
            j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, p1)))
            j.CreateLocalRot1Attr(q0)
            for dof in ("transX", "transY", "transZ"):
                lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
                lim.CreateLowAttr(1.0)
                lim.CreateHighAttr(-1.0)
            j.CreateExcludeFromArticulationAttr(True)

        ball(f"{prim_path}/tie", None, f"{prim_path}/seg_0", [w[i] + CORD_START[i] for i in range(3)], qy,
             (-CORD.seg_len / 2, 0.0, 0.0))
        bag_path = f"{parent}/{BAG.name}"
        ball(f"{prim_path}/swivel", bag_path, f"{prim_path}/seg_{CORD.segments - 1}", (0.0, 0.0, SWIVEL_DZ), q0,
             (CORD.seg_len / 2, 0.0, 0.0))
        bag_prim = stage.GetPrimAtPath(bag_path)
        if bag_prim.IsValid():
            rb = PhysxSchema.PhysxRigidBodyAPI.Apply(bag_prim)
            rb.CreateAngularDampingAttr(10.0)
            rb.CreateLinearDampingAttr(1.0)  # swing dies out in a few seconds
        # bridge cheeks welded to their bodies
        for bn, cheeks in CHEEKS.items():
            for c in cheeks:
                j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/weld_{c.name}")
                j.CreateBody0Rel().SetTargets([Sdf.Path(f"{parent}/{bn}")])
                j.CreateBody1Rel().SetTargets([Sdf.Path(f"{parent}/{c.name}")])
                b = next(x for x in BRIDGES if x.name == bn)
                j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, np.asarray(c.pos) - np.asarray(b.pos))))
                j.CreateLocalRot0Attr(q0)
                j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
                j.CreateLocalRot1Attr(q0)
                j.CreateExcludeFromArticulationAttr(True)
                fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"{parent}/{c.name}"))
                fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(f"{parent}/{bn}"))
        # deck friction
        mat = f"{parent}/sono_deck_material"
        mc = sim_utils.RigidBodyMaterialCfg(static_friction=DECK_FRICTION, dynamic_friction=DECK_FRICTION)
        mc.func(mat, mc)
        dprim = stage.GetPrimAtPath(f"{parent}/{DECK.name}")
        nb = 0
        for p in Usd.PrimRange(dprim):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                bind_physics_material(str(p.GetPath()), mat, stage=stage)
                nb += 1
        print(f"[sono] deck valid={dprim.IsValid()} colliders bound={nb}", flush=True)
        # lip: smooth steel rod along x
        lmat = f"{parent}/sono_lip_material"
        lc = sim_utils.RigidBodyMaterialCfg(static_friction=LIP_FRICTION, dynamic_friction=LIP_FRICTION,
                                            friction_combine_mode="min")
        lc.func(lmat, lc)
        c = sim_utils.CylinderCfg(radius=LIP_R, height=BOX_X[1] - BOX_X[0], axis="X",
                                  collision_props=CollisionPropertiesCfg(),
                                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=STEEL, roughness=0.3))
        c.func(f"{parent}/sono_lip", c, translation=(AX, LIP_Y, TOP + LIP_Z))
        for p in Usd.PrimRange(stage.GetPrimAtPath(f"{parent}/sono_lip")):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                bind_physics_material(str(p.GetPath()), lmat, stage=stage)
        # painted marks (visual only)
        for i, my in enumerate(MARKS_Y):
            m = sim_utils.CuboidCfg(size=(BOX_X[1] - BOX_X[0], 0.005, 0.0004),
                                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=BLUE, roughness=0.6))
            m.func(f"{parent}/sono_mark_{i}", m, translation=(AX, my, TOP + D + 0.0002))
        return prim

    spawn._sono_hook = True
    cablekit._spawn_cable = spawn


_install_spawn_hook()


def _settle(emb):  # let the hanger drop over the lip and come to rest before the first observation
    cableshape.settle(emb, 3.0)


cableshape.on_reset(_settle)

# -- env handle (grippers) ---------------------------------------------------------------------------------------------
_EP: dict = {"env": None}


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_sono_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _EP["env"] = self._env
            return base(self, *a, **k)

        _task_step._sono_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[np.ndarray]:
    """TCP and pad-tip positions of both grippers (yam_arena.runtime); [] offline."""
    env = _EP["env"]
    if env is None:
        return []
    from yam_arena.runtime import pad_tips_world, tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    out = []
    for k in ("left_robot", "right_robot"):
        art = base.scene[k]
        out.append(np.asarray(tcp_world(art), dtype=np.float64) - origin)
        out.append(np.asarray(pad_tips_world(art), dtype=np.float64).reshape(-1, 3) - origin)
    return out


# -- geometry ---------------------------------------------------------------------------------------------------------
def _dense(poly: np.ndarray, step: float = 0.002) -> np.ndarray:
    pts = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(float(np.linalg.norm(b - a)) / step))
        pts += [a + (b - a) * (j / n) for j in range(1, n + 1)]
    return np.array(pts)


def _finite(objects) -> bool:
    ok = np.isfinite(objects[CORD.name].points).all() and np.isfinite(objects[BAG.name].pos).all()
    return bool(ok and all(np.isfinite(objects[n].pos).all() for n in BRIDGE_NAMES))


def _zaxis(q) -> np.ndarray:
    x, y, z, w = (float(t) for t in q)
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


MARK_TOL = 0.010  # bridge centre within 1 cm of its mark (y)
LINE_TOL = 0.006  # bridge centre within 6 mm of the cord line (x)
TILT_MAX = 10.0  # deg
NOTCH_X = NOTCH_W / 2  # cord centre within the notch (x)
RELEASE_DIST = 0.04
STILL_V = 0.01
SPAN = (0.145, 0.175)
LIFT_MIN = 0.030  # cord's lowest point between the bridges at least 3 cm above the box top (cord bottom)
CLEAR = 0.003


def bridge_tilt(objects, n) -> float:
    zc = _zaxis(objects[n].quat_xyzw)
    return float(math.degrees(math.acos(max(-1.0, min(1.0, zc[2])))))


def seated(objects, n) -> bool:
    """Cord centre inside this bridge's notch: every cord point within 4 mm (y) of the bridge's centre plane is within
    5 mm (x) of the notch centre and its height is the notch floor + cord radius, -3/+4 mm."""
    p = np.asarray(objects[n].pos, dtype=float)
    floor = p[2] + BR_H / 2
    c = _dense(objects[CORD.name].polyline())
    m = np.abs(c[:, 1] - p[1]) <= 0.004
    if not m.any():
        return False
    c = c[m]
    return bool((np.abs(c[:, 0] - p[0]) <= NOTCH_X).all() and (c[:, 2] >= floor + R - 0.003).all()
                and (c[:, 2] <= floor + R + 0.004).all())


def bridge_ok(objects, n, mark) -> bool:
    p = np.asarray(objects[n].pos, dtype=float)
    on_deck = abs(p[2] - (TOP + D + BR_H / 2)) <= 0.004
    return bool(on_deck and abs(p[1] - mark) <= MARK_TOL and abs(p[0] - AX) <= LINE_TOL
                and bridge_tilt(objects, n) < TILT_MAX and seated(objects, n))


def _assign(objects):
    """Which bridge stands at which mark (either bridge may go at either mark): [(name, mark)] for bridges that are
    within 1 cm of a mark, nearest first, each mark used once."""
    out, used = [], set()
    for n in BRIDGE_NAMES:
        p = np.asarray(objects[n].pos, dtype=float)
        for my in MARKS_Y:
            if my not in used and abs(p[1] - my) <= MARK_TOL:
                out.append((n, my))
                used.add(my)
                break
    return out


def n_bridges_ok(objects) -> int:
    return sum(bridge_ok(objects, n, my) for n, my in _assign(objects))


def _box_gap(lo, hi, c, s) -> float:
    c, s = np.asarray(c), np.asarray(s) / 2
    d = np.maximum(np.maximum((c - s) - hi, lo - (c + s)), 0.0)
    return float(np.linalg.norm(d))


def bag_clearance(objects) -> float:
    s = objects[BAG.name]
    lo, hi = np.asarray(s.low, dtype=float), np.asarray(s.high, dtype=float)
    return min(float(lo[2] - TOP), _box_gap(lo, hi, DECK.pos, DECK.size),
               _box_gap(lo, hi, (AX, LIP_Y, TOP + LIP_Z), (BOX_X[1] - BOX_X[0], 2 * LIP_R, 2 * LIP_R)))


def released(objects, dist: float = RELEASE_DIST) -> bool:
    """Every gripper TCP and pad tip at least ``dist`` (4 cm) from the cord and from both bridges."""
    pts = _dense(objects[CORD.name].polyline())
    boxes = [(np.asarray(objects[n].low, dtype=float), np.asarray(objects[n].high, dtype=float)) for n in BRIDGE_NAMES]
    for g in grippers():
        for q in np.atleast_2d(g):
            if float(np.linalg.norm(pts - q, axis=1).min()) < dist:
                return False
            for lo, hi in boxes:
                if float(np.linalg.norm(np.maximum(np.maximum(lo - q, q - hi), 0.0))) < dist:
                    return False
    return True


def still(objects) -> bool:
    v = [float(np.linalg.norm(objects[n].lin_vel)) for n in [*BRIDGE_NAMES, BAG.name]]
    return bool(max(v) < STILL_V)


def span_lift(objects) -> tuple[float, float]:
    """(distance between the two bridges' centres along y, lowest cord-bottom height above the box top between them)."""
    a = _assign(objects)
    if len(a) < 2:
        return 0.0, 0.0
    ys = sorted(float(objects[n].pos[1]) for n, _ in a)
    c = _dense(objects[CORD.name].polyline())
    m = (c[:, 1] > ys[0] + 0.01) & (c[:, 1] < ys[1] - 0.01)
    low = float(c[m, 2].min() - R - (TOP + D)) if m.any() else 0.0
    return ys[1] - ys[0], low


def score(objects) -> float:
    """0 if the state is non-finite, any gripper TCP or pad tip is within 4 cm of the cord or a bridge, or a bridge or
    the weight still moves (1 cm/s). A bridge counts when it stands on the box top (tilt < 10 deg) within 1 cm of a
    mark and 6 mm of the cord line, with the cord centre inside its notch. 0.5: one bridge counts. 1.0: both count, 16
    +- 1.5 cm apart, the cord's underside at least 3 cm above the box top all the way between them, and the weight
    hangs free (3 mm clear of the table, box and lip)."""
    if not _finite(objects) or not released(objects) or not still(objects):
        return 0.0
    k = n_bridges_ok(objects)
    if k == 0:
        return 0.0
    if k == 1:
        return 0.5
    span, low = span_lift(objects)
    if SPAN[0] <= span <= SPAN[1] and low >= LIFT_MIN and bag_clearance(objects) >= CLEAR:
        return 1.0
    return 0.5


def success(objects) -> bool:
    """Full score with both hands moved well clear (10 cm), so the episode ends with the set-up in view."""
    return bool(score(objects) >= 0.999 and released(objects, 0.10))


def distance(objects) -> float:
    if not _finite(objects):
        return 1.0
    d = 0.0
    for my in MARKS_Y:
        d += min(float(np.linalg.norm(np.asarray(objects[n].pos)[:2] - (AX, my))) for n in BRIDGE_NAMES)
    return d + 0.5 * (2 - n_bridges_ok(objects)) * 0.1


def progress(objects) -> dict:
    if not _finite(objects):
        return {}
    a = _assign(objects)
    span, low = span_lift(objects)
    return {
        "bridge_at_mark": len(a) / 2.0,
        "bridges_upright": float(np.mean([bridge_tilt(objects, n) < TILT_MAX for n in BRIDGE_NAMES])),
        "cord_seated": n_bridges_ok(objects) / 2.0,
        "cord_lifted": float(low >= LIFT_MIN),
        "weight_free": float(bag_clearance(objects) >= CLEAR),
        "released": float(released(objects)),
    }


VARIATION = {
    "obj:bridge_a": {"dx": 0.015, "dy": 0.015, "yaw": 0.1},
    "obj:bridge_b": {"dx": 0.015, "dy": 0.015, "yaw": 0.1},
    "cable:cord": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},  # tied at the peg and tensioned by the weight
}

# -- closed-loop scripted demo (cabledemo.py) ---------------------------------------------------------------------------
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402

GRASP_OPEN = -0.014  # pads ~28 mm apart
GRIP_DX = 0.030  # bridge grasp: TCP this far in front of the bridge centre (front cheek), fingers across its faces
CORD_LIFT = 0.101  # hand (TCP) this far above the box top while holding the cord up for bridge A
CORD_LIFT_B = 0.090  # for bridge B (left hand turned so its wrist camera faces away from the right arm)
STOW = {"left": (0.12, 0.40, TOP + 0.20), "right": (0.12, -0.40, TOP + 0.20)}
_DBG = bool(os.environ.get("SONO_DEBUG"))


def _log(ctx, tag=""):
    st = ctx.states()
    if not _finite(st):
        bad = [n for n in [*BRIDGE_NAMES, BAG.name] if not np.isfinite(st[n].pos).all()]
        nc = int((~np.isfinite(st[CORD.name].points).all(axis=1)).sum())
        print(f"[t={ctx.t:5.2f}] {tag}: NON-FINITE state (bodies {bad}, cord segs {nc})", flush=True)
        return
    br = []
    for n in BRIDGE_NAMES:
        p = np.asarray(st[n].pos)
        br.append(f"{n[-1]}=({p[0]:.3f},{p[1]:+.3f},{p[2] - TOP:.3f}) tilt={bridge_tilt(st, n):.0f} seat={int(seated(st, n))}")
    b = np.asarray(st[BAG.name].pos)
    span, low = span_lift(st)
    c = st[CORD.name].points
    print(f"[t={ctx.t:5.2f}] {tag}: {' '.join(br)} bag_z={b[2] - TOP:.3f} clear={bag_clearance(st):.3f} "
          f"span={span:.3f} low={low:.3f} cordmaxz={c[:, 2].max() - TOP - D:.3f} cordx=[{c[:, 0].min():.3f},"
          f"{c[:, 0].max():.3f}] v={max(float(np.linalg.norm(st[n].lin_vel)) for n in [*BRIDGE_NAMES, BAG.name]):.4f} rel={int(released(st))} still={int(still(st))} score={score(st):.2f}", flush=True)
    if _DBG:
        print("    cord (y, z-deck):", [(round(float(q[1]), 3), round(float(q[2] - TOP - D), 3)) for q in c[::2]],
              flush=True)


def _cord_at(st, y):
    c = st[CORD.name].points
    i = int(np.argmin(np.abs(c[:, 1] - y) + 10 * (c[:, 2] < TOP + D - 0.01)))
    return c[i]


def _lift_cord(arm, ctx, y, yaw, lift=CORD_LIFT):
    st = ctx.states()
    p = _cord_at(st, y)
    yield from arm.move((p[0], p[1], TOP + D + 0.06), yaw=yaw, finger=GRASP_OPEN, duration=1.5)
    p = _cord_at(ctx.states(), y)
    yield from arm.move((p[0], p[1], p[2] + 0.02), yaw=yaw, duration=0.6)
    p = _cord_at(ctx.states(), y)
    yield from arm.move((p[0], p[1], p[2] + 0.001), yaw=yaw, duration=0.8)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    yield from arm.move((AX, p[1], TOP + D + lift), yaw=yaw, duration=2.0)
    d = float(np.linalg.norm(ctx.states()[CORD.name].points - arm.tcp(), axis=1).min())
    _log(ctx, f"{arm.name} lifted cord (cord-tcp {d:.3f})")


def _cord_x_err(ctx, names):
    """Mean x offset of the cord from each standing bridge's notch centre, at that bridge's plane (for logs)."""
    st = ctx.states()
    c = _dense(st[CORD.name].polyline())
    e = []
    for n in names:
        b = np.asarray(st[n].pos)
        if abs(b[2] - (TOP + D + BR_H / 2)) > 0.01:
            continue
        m = np.abs(c[:, 1] - b[1]) <= 0.004
        if m.any():
            e.append(float(c[m, 0].mean() - b[0]))
    return float(np.mean(e)) if e else 0.0


def _hand_dx(ctx, names, hand_y):
    """Hand x shift that best puts the cord over every standing bridge's notch. The cord at a bridge moves k times
    as far as the hand, k = the bridge's distance from the fixed end on its side over the hand's (peg or lip)."""
    st = ctx.states()
    c = _dense(st[CORD.name].polyline())
    num = den = 0.0
    for n in names:
        b = np.asarray(st[n].pos)
        if abs(b[2] - (TOP + D + BR_H / 2)) > 0.01:
            continue
        m = np.abs(c[:, 1] - b[1]) <= 0.004
        if not m.any():
            continue
        e = float(c[m, 0].mean() - b[0])
        end = CORD_START[1] if b[1] > hand_y else LIP_Y
        k = abs(b[1] - end) / max(abs(hand_y - end), 1e-3)
        num += k * e
        den += k * k
    return -num / den if den > 0 else 0.0


def _lower_cord(arm, ctx, yaw, names=(), release=True):
    """Lower the held cord in stages; before each stage, shift the hand along x so the cord sits over the notch
    centre of the bridges ``names`` (the cord at a bridge moves about half as far as the hand)."""
    for _ in range(3):  # centre the cord over the notches before it touches them
        p = arm.tcp()
        dx = _hand_dx(ctx, names, p[1])
        yield from arm.move((p[0] + float(np.clip(dx, -0.01, 0.01)), p[1], p[2]), yaw=yaw, duration=0.5)
    if _DBG:
        print(f"    centred: cord x err={_cord_x_err(ctx, names):+.4f} hand x={arm.tcp()[0]:.3f}", flush=True)
    x0 = arm.tcp()[0]
    h0 = arm.tcp()[2] - TOP - D
    for z in (h0 - 0.002, h0 - 0.006, h0 - 0.010):
        for _ in range(2):
            p = arm.tcp()
            x = float(np.clip(p[0] + 0.5 * _hand_dx(ctx, names, p[1]), x0 - 0.008, x0 + 0.008))
            yield from arm.move((x, p[1], TOP + D + z), yaw=yaw, duration=0.4)
        if _DBG:
            st = ctx.states()
            c = _dense(st[CORD.name].polyline())
            g = [round(float(min(np.linalg.norm(c - q, axis=1).min() for q in np.atleast_2d(gg))), 3) for gg in grippers()]
            print(f"    lower z={z:.3f} cord x err={_cord_x_err(ctx, names):+.4f} hand x={arm.tcp()[0]:.3f} "
                  f"cord-gripper dists={g}", flush=True)
    p = arm.tcp()  # stop with the cord level with the bridge tops: lowering it further would pull a bridge over
    yield from arm.move((p[0], p[1], TOP + D + BR_TOP + R + 0.001), yaw=yaw, duration=1.2)
    if not release:
        return
    yield from arm.grip(GRASP_OPEN, 0.5)
    p = arm.tcp()
    yield from arm.move((p[0], p[1], TOP + D + 0.07), yaw=yaw, duration=0.8)


def _pick_bridge(arm, ctx, n):
    st = ctx.states()
    p = np.asarray(st[n].pos)
    gx = p[0] - GRIP_DX
    top = p[2] + BR_H / 2 + NOTCH_D
    yield from arm.move((gx, p[1], top + 0.06), yaw=0.0, finger=GRASP_OPEN, duration=1.5)
    yield from arm.move((gx, p[1], top - 0.002), yaw=0.0, duration=0.8)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    yield from arm.move((gx, p[1], top + 0.03), yaw=0.0, duration=0.6)
    st = ctx.states()
    off = arm.tcp() - np.asarray(st[n].pos)  # measured hold offset (TCP - bridge centre)
    if _DBG:
        print(f"    {n} hold offset {np.round(off, 4)}", flush=True)
    return off


def _carry_in(arm, ctx, n, off, mark, push_only=False):
    """Bring the bridge in front of the box at the mark, its underside 4 mm above the top, then slide it in along x
    until its centre is on the cord line, and set it down."""
    zc = TOP + D + BR_H / 2 + 0.003  # bridge centre height while sliding in
    if _DBG:
        _log(ctx, f"  {n} picked")
    yield from arm.move((0.20 + off[0], mark + off[1], zc + off[2] + 0.02), yaw=0.0, duration=1.5)
    if _DBG:
        _log(ctx, f"  {n} at the box front")
    yield from arm.move((0.21 + off[0], mark + off[1], zc + off[2]), yaw=0.0, duration=0.6)
    tx = 0.21 + off[0]
    for _ in range(6):
        p = np.asarray(ctx.states()[n].pos)
        cy = mark - p[1]  # closed loop on y
        tx += (AX - 0.21) / 6
        yield from arm.move((tx, arm.tcp()[1] + 0.5 * cy, zc + off[2]), yaw=0.0, duration=0.4)
        if _DBG:
            _log(ctx, f"  slide x={tx:.3f}")
    for _ in range(2):  # closed loop: bridge centre onto the cord line and the mark
        p = np.asarray(ctx.states()[n].pos)
        t = arm.tcp()
        yield from arm.move((t[0] + (AX - p[0]), t[1] + (mark - p[1]), zc + off[2]), yaw=0.0, duration=0.4)
    t = arm.tcp()
    yield from arm.move((t[0], t[1], zc + off[2] - 0.0035), yaw=0.0, duration=0.5)
    for _ in range(3):  # set it down: press until the bridge stands on the box top
        dz = float(ctx.states()[n].pos[2]) - (TOP + D + BR_H / 2)
        if dz <= 0.001:
            break
        t = arm.tcp()
        yield from arm.move((t[0], t[1], t[2] - dz - 0.001), yaw=0.0, duration=0.4)
    for _ in range(2):  # nudge the standing bridge onto the line and the mark
        p = np.asarray(ctx.states()[n].pos)
        ex, ey = AX - p[0], mark - p[1]
        if max(abs(ex), abs(ey)) < 0.0015:
            break
        t = arm.tcp()
        yield from arm.move((t[0] + ex, t[1] + ey, t[2]), yaw=0.0, duration=0.4)
    _log(ctx, f"{n} in place")
    if _DBG:
        st = ctx.states()
        c = _dense(st[CORD.name].polyline())
        for gg in grippers():
            q = np.atleast_2d(gg)
            print("    gripper pts", np.round(q - [0, 0, TOP], 3).tolist(), "min cord dist",
                  round(float(min(np.linalg.norm(c - x, axis=1).min() for x in q)), 3), flush=True)


def _let_go_bridge(arm, ctx):
    yield from arm.grip(GRASP_OPEN, 0.5)
    p = arm.tcp()
    yield from arm.move((p[0] - 0.01, p[1], p[2] + 0.06), yaw=0.0, duration=0.8)


def _set_bridge(ctx, holder, lifter, n, mark, lift_y, lift_yaw, mode, lift=CORD_LIFT):
    off = None

    def hold_side():
        nonlocal off
        off = yield from _pick_bridge(holder, ctx, n)

    yield from ctx.both(hold_side(), _lift_cord(lifter, ctx, lift_y, lift_yaw, lift))
    yield from _carry_in(holder, ctx, n, off, mark)
    # centre on the bridge already set (it stands free; the held one sits on the line under a straight cord), or on
    # the held one if it is the first
    others = [m for m in BRIDGE_NAMES if m != n and abs(float(ctx.states()[m].pos[2]) - (TOP + D + BR_H / 2)) < 0.01]
    yield from _lower_cord(lifter, ctx, lift_yaw, names=others or [n])
    yield from holder.wait(0.5)
    _log(ctx, f"{n} cord lowered")
    if mode == "hold" and n == "bridge_b":
        return
    yield from _let_go_bridge(holder, ctx)
    _log(ctx, f"{n} let go")


def demo_script(ctx):
    """SONO_DEMO=correct (default) | nothing | hold (both set, the right hand keeps bridge B) | onearm (the left arm
    alone pushes bridge A along the box top into the lying cord at the +8 mark and lets go) | drop (the right arm
    lifts the cord, lets it fall, then the left arm slides bridge A in) | one (only bridge A is set: 0.5)."""
    mode = os.environ.get("SONO_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    yield from L.wait(0.5)
    _log(ctx, "settled")
    if mode == "nothing":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    if mode == "probe_drop":
        yield from _lift_cord(Rt, ctx, -0.07, math.pi / 2)
        yield from Rt.grip(GRASP_OPEN, 0.4)
        p = Rt.tcp()
        yield from Rt.move((p[0], p[1], p[2] + 0.05), yaw=math.pi / 2, duration=0.6)
        for _ in range(4):
            yield from Rt.wait(0.5)
            _log(ctx, "after drop")
        yield from Rt.move(STOW["right"], yaw=0.0, duration=1.0)
    elif mode == "onearm":
        off = yield from _pick_bridge(L, ctx, "bridge_a")
        yield from _carry_in(L, ctx, "bridge_a", off, MARKS_Y[0])
        yield from _let_go_bridge(L, ctx)
    elif mode == "drop":
        yield from _lift_cord(Rt, ctx, -0.07, math.pi / 2)
        yield from Rt.grip(GRASP_OPEN, 0.4)
        p = Rt.tcp()
        yield from Rt.move((p[0], p[1], p[2] + 0.05), yaw=math.pi / 2, duration=0.6)
        off = yield from _pick_bridge(L, ctx, "bridge_a")
        yield from _carry_in(L, ctx, "bridge_a", off, MARKS_Y[0])
        yield from _let_go_bridge(L, ctx)
    else:
        yield from _set_bridge(ctx, L, Rt, "bridge_a", MARKS_Y[0], -0.07, math.pi / 2, mode)
        yield from L.move(STOW["left"], yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
        if mode not in ("one", "probe"):
            yield from Rt.move((0.22, -0.10, TOP + D + 0.07), yaw=0.0, duration=0.8)
            yield from _set_bridge(ctx, Rt, L, "bridge_b", MARKS_Y[1], 0.055, 1.3, mode, CORD_LIFT_B)
            yield from L.move(STOW["left"], yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
    if mode != "hold":
        yield from Rt.move(STOW["right"], yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
    for _ in range(6):
        yield from ctx.both(L.wait(0.5), Rt.wait(0.5))
        if _DBG:
            _log(ctx, "  wait")
    _log(ctx, "end")
