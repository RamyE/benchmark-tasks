"""Everyday chore on the station table: free a cord pinned under a stack of cases. Two hard equipment cases stand stacked on the left, in front
of the left arm: a black base case (14 x 16 x 8 cm, 600 g) with a dark grey bar handle (9 cm, held 4 cm off the face by two standoffs) on
its front face, and a grey top case (10 x 13 x 9 cm, 300 g) on it (centred 1 cm behind its centre). An orange 6 mm power cord (42 cm, 20 g) runs
left-right across the table and passes under the base case, 3.5 cm behind its front face, so the stack rests on the
cord (tilted back 3.3 deg, front edge about 8 mm up) and on its back edge. Its bare tail sticks out 3 cm left of the stack; the rest runs to the
right and ends in a black plug (35 x 16 x 12 mm). A grey open bin (10 x 10 cm inside, 4 cm walls) stands at the
right. The cord must come out from under the stack and its plug go in the bin, without the cases being dragged or
tipped: pulled alone, the cord drags the whole stack (the stack's weight is mostly on the cord). The demo lifts the
base case's handle 2 cm with the left arm (the stack tilts back about 7.5 deg on its back edge) while
the right arm draws the cord out by its plug and drops the plug in the bin; then the case is set down and let go, the open left gripper rises above the stack's right front corner and
the open right gripper stays 12 cm above the bin, so the two wrist cameras show the freed cord tail and the plug in the bin.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants: CASES_DEMO=correct (default) | nothing | yank | hold | tip.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Pull the orange cord out from under the stacked cases without sliding or tipping them, and put its plug in the grey bin. Let go of "
               "everything at the end.")

BLACK = (0.06, 0.06, 0.07)
GREY = (0.55, 0.57, 0.60)
ORANGE = (0.95, 0.45, 0.05)
BIN_C = (0.45, 0.47, 0.50)

# -- cord: straight along -y at x 0.235, bare tail on the left, plug on the right -------------------------------------
R = 0.003
CORD_X = 0.235
TAIL_Y = 0.24
CORD_LEN = 0.42
PL = Plug(size=(0.035, 0.016, 0.012), mass=0.015, color=BLACK)
CORD = cable("cord", start=(CORD_X, TAIL_Y, TOP + R + 0.0005), yaw=-math.pi / 2, length=CORD_LEN, radius=R,
             cable_mass=0.02, end_plug=PL, bend_stiffness=0.01, bend_damping=0.001, color=ORANGE)

# -- stacked cases (free) ---------------------------------------------------------------------------------------------
BC_SIZE = (0.14, 0.16, 0.08)
BC_XY = (0.27, 0.13)  # front face x 0.20, back x 0.34, y 0.05..0.21
BC_Z0 = TOP + 2 * R + 0.001 + BC_SIZE[2] / 2  # spawned flat just above the cord; settles back onto its back edge
BASE = Box("base_case", pos=(BC_XY[0], BC_XY[1], BC_Z0), size=BC_SIZE, mass=0.6, color=BLACK)
TC_SIZE = (0.10, 0.13, 0.09)
TC_XY = (0.28, 0.13)
TC_Z0 = BC_Z0 + BC_SIZE[2] / 2 + 0.001 + TC_SIZE[2] / 2
TOPC = Box("top_case", pos=(TC_XY[0], TC_XY[1], TC_Z0), size=TC_SIZE, mass=0.3, color=GREY)
# bar handle on the front face: a 12 x 90 x 15 mm bar held 40 mm off the face by two 10 mm standoffs (a finger fits
# behind the bar, so it is pinched front-to-back and the wrist camera sits beside the stack, not against it)
HD_C = (0.20, 0.20, 0.22)
HD_SIZE = (0.012, 0.09, 0.015)
HD_REL = np.array([-BC_SIZE[0] / 2 - 0.04 - HD_SIZE[0] / 2, 0.0, 0.005])
POST_SIZE = (0.039, 0.01, 0.015)
_bc0 = np.array([BC_XY[0], BC_XY[1], BC_Z0])
HANDLE = Box("case_handle", pos=tuple(map(float, _bc0 + HD_REL)), size=HD_SIZE, mass=0.015, color=HD_C)
POST_REL = {f"handle_post_{k}": np.array([-BC_SIZE[0] / 2 - POST_SIZE[0] / 2, sy * 0.04, 0.005])
            for k, sy in (("l", 1), ("r", -1))}
POSTS = [Box(n, pos=tuple(map(float, _bc0 + r)), size=POST_SIZE, mass=0.005, color=HD_C) for n, r in POST_REL.items()]
WELDS = {HANDLE.name: HD_REL, **POST_REL}
CASES = (BASE, TOPC)

# -- bin (fixed): floor + 4 walls -------------------------------------------------------------------------------------
BIN_XY = (0.24, -0.41)
BIN_IN = 0.10
BIN_T = 0.005
BIN_H = 0.04
_bx, _by = BIN_XY
_h = BIN_IN / 2 + BIN_T / 2
BIN = [
    Box("bin_floor", pos=(_bx, _by, TOP + 0.0015), size=(BIN_IN + 2 * BIN_T, BIN_IN + 2 * BIN_T, 0.003), color=BIN_C),
    Box("bin_wall_near", pos=(_bx - _h, _by, TOP + BIN_H / 2), size=(BIN_T, BIN_IN + 2 * BIN_T, BIN_H), color=BIN_C),
    Box("bin_wall_far", pos=(_bx + _h, _by, TOP + BIN_H / 2), size=(BIN_T, BIN_IN + 2 * BIN_T, BIN_H), color=BIN_C),
    Box("bin_wall_left", pos=(_bx, _by + _h, TOP + BIN_H / 2), size=(BIN_IN, BIN_T, BIN_H), color=BIN_C),
    Box("bin_wall_right", pos=(_bx, _by - _h, TOP + BIN_H / 2), size=(BIN_IN, BIN_T, BIN_H), color=BIN_C),
]

OBJECTS = [BASE, TOPC, HANDLE, *POSTS, *BIN, CORD]

TWIN_EXTRA = [
    {"name": "handle_mount", "kind": "joint", "real": "bar handle and standoffs screwed/glued to the base case",
     "note": "bar handle (and its two standoffs) rigidly fixed to the base case front face"},
]

# -- handle weld, authored right after the cable (the kit's only spawn hook) ---------------------------------------


def _install_joints() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_cases_joints", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORD.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdPhysics

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        for name, rel in WELDS.items():
            j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/weld_{name}")
            j.CreateBody0Rel().SetTargets([Sdf.Path(f"{parent}/{BASE.name}")])
            j.CreateBody1Rel().SetTargets([Sdf.Path(f"{parent}/{name}")])
            j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, rel)))
            j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateExcludeFromArticulationAttr(True)
        return prim

    spawn._cases_joints = True
    cablekit._spawn_cable = spawn


_install_joints()

# -- geometry helpers -----------------------------------------------------------------------------------------------


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def bottom_corners(st, size) -> np.ndarray:
    """(4, 2) top-view positions of a case's four bottom corners."""
    Rm = _rot(st.quat_xyzw)
    hx, hy, hz = (s / 2 for s in size)
    loc = np.array([[sx * hx, sy * hy, -hz] for sx in (-1, 1) for sy in (-1, 1)])
    return (np.asarray(st.pos) + loc @ Rm.T)[:, :2]


def tilt_deg(st) -> float:
    return math.degrees(math.acos(float(np.clip(_rot(st.quat_xyzw)[2, 2], -1.0, 1.0))))


# starting marks: bottom corners of the cases in the design pose (after settling on the cord the base case tips back
# 3.3 deg onto its back edge, which shifts the top case's corners 4 mm from these marks; measured in a do-nothing run)
def _design_corners(size, xy) -> np.ndarray:
    hx, hy = size[0] / 2, size[1] / 2
    return np.array([[xy[0] + sx * hx, xy[1] + sy * hy] for sx in (-1, 1) for sy in (-1, 1)])


REF = {BASE.name: _design_corners(BC_SIZE, BC_XY), TOPC.name: _design_corners(TC_SIZE, TC_XY)}
MOVE_TOL = 0.03  # any bottom corner more than 3 cm from its mark = case moved (the demo's 2 cm handle lift shifts
# the corners at most ~0.9 cm; a one-arm yank drags the stack ~23 cm)
TIP_DEG = 10.0  # either case tilted more than 10 deg = tipped
RELEASE_DIST = 0.03


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def cord_pts(objects) -> np.ndarray:
    c = objects[CORD.name]
    return np.vstack([_dense(c.polyline()), np.asarray(c.end_plug)[None]])


def under_case(objects) -> int:
    """Number of cord sample points (4 mm apart) inside the base case's footprint in top view."""
    b = objects[BASE.name]
    Rm = _rot(b.quat_xyzw)
    loc = (cord_pts(objects) - np.asarray(b.pos)) @ Rm
    ok = (np.abs(loc[:, 0]) <= BC_SIZE[0] / 2) & (np.abs(loc[:, 1]) <= BC_SIZE[1] / 2)
    return int(ok.sum())


def plug_in_bin(objects) -> bool:
    p = np.asarray(objects[CORD.name].end_plug)
    return bool(abs(p[0] - BIN_XY[0]) <= BIN_IN / 2 and abs(p[1] - BIN_XY[1]) <= BIN_IN / 2
                and p[2] <= TOP + BIN_H)


def _finite(objects) -> bool:
    c = objects[CORD.name]
    ok = np.isfinite(c.points).all() and c.end_plug is not None and np.isfinite(c.end_plug).all()
    for k in CASES:
        ok = ok and np.isfinite(objects[k.name].pos).all() and np.isfinite(objects[k.name].quat_xyzw).all()
    return bool(ok)


# -- per-step monitor (latched until reset) and gripper TCPs ----------------------------------------------------------
_ENV: dict = {"env": None}
_MON: dict = {}


def _reset_monitor() -> None:
    _MON.update(moved=False, tipped=False, nan=False, max_move=0.0, max_tilt=0.0, steps=0, events=[])


_reset_monitor()


def case_move(objects, name: str) -> float:
    size = BC_SIZE if name == BASE.name else TC_SIZE
    return float(np.linalg.norm(bottom_corners(objects[name], size) - REF[name], axis=1).max())


def _monitor(objects) -> None:
    _MON["steps"] += 1
    if not _finite(objects):
        _MON["nan"] = True
        return
    for k in CASES:
        m, t = case_move(objects, k.name), tilt_deg(objects[k.name])
        _MON["max_move"] = max(_MON["max_move"], m)
        _MON["max_tilt"] = max(_MON["max_tilt"], t)
        if m > MOVE_TOL and not _MON["moved"]:
            _MON["moved"] = True
            _MON["events"].append((f"{k.name} moved {m * 100:.1f} cm", _MON["steps"]))
        if t > TIP_DEG and not _MON["tipped"]:
            _MON["tipped"] = True
            _MON["events"].append((f"{k.name} tilted {t:.1f} deg", _MON["steps"]))


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_cases_hooked", False):
        return
    base_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        try:
            from yam_arena.scene import object_states

            _monitor(object_states(self._env, [CORD.name, BASE.name, TOPC.name]))
        except Exception as e:  # pragma: no cover
            print(f"[cases] monitor error: {e!r}", flush=True)
        return base_step(self, *a, **k)

    def reset(self, *a, **k):
        out = base_reset(self, *a, **k)
        _reset_monitor()
        return out

    _task_step._cases_hooked = True
    reset._cases_hooked = True
    cls._task_step = _task_step
    cls.reset = reset


_install_hooks()


def grippers() -> list[np.ndarray]:
    """World TCPs (yam_arena.runtime.tcp_world) of the left and right gripper; [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = np.asarray(getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0].detach().cpu().numpy(),
                        dtype=np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


def _box_dist(p, st, size) -> float:
    loc = _rot(st.quat_xyzw).T @ (np.asarray(p) - np.asarray(st.pos))
    return float(np.linalg.norm(np.maximum(np.abs(loc) - np.asarray(size) / 2, 0.0)))


def released(objects) -> bool:
    pts = cord_pts(objects)
    for g in grippers():
        if float(np.linalg.norm(pts - g, axis=1).min()) < RELEASE_DIST:
            return False
        for name, size in ((BASE.name, BC_SIZE), (TOPC.name, TC_SIZE), (HANDLE.name, HD_SIZE),
                           *((n, POST_SIZE) for n in POST_REL)):
            if _box_dist(g, objects[name], size) < RELEASE_DIST:
                return False
    return True


def violations() -> list[str]:
    return [k for k in ("moved", "tipped", "nan") if _MON[k]]


def score(objects) -> float:
    """0 if at any step a bottom corner of either case was more than 3 cm (top view) from its starting mark, or
    either case tilted more than 10 deg, or any state went NaN; 0 if at the end a gripper TCP is within 3 cm of the
    cord, plug, either case or the handle. Otherwise 1.0 if no part of the cord or plug is inside the base case's
    footprint and the plug's centre is inside the bin (within its inner walls, below the rim); 0.5 if the cord is
    out from under the case but the plug is not in the bin; else 0."""
    if violations() or not _finite(objects):
        return 0.0
    if not released(objects):
        return 0.0
    if under_case(objects) > 0:
        return 0.0
    return 1.0 if plug_in_bin(objects) else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[CORD.name].resting(0.03, 1.0) and objects[BASE.name].resting()


def distance(objects) -> float:
    """Cord length (m) still under the base case plus plug distance to the bin centre; 1 m after a violation."""
    if violations() or not _finite(objects):
        return 1.0
    p = np.asarray(objects[CORD.name].end_plug)
    return under_case(objects) * 0.004 + float(np.linalg.norm(p[:2] - np.asarray(BIN_XY)))


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    ok = _finite(objects)
    n0 = max(1, round(BC_SIZE[1] / 0.004))
    return {
        "cord_drawn_out": float(ok and np.clip(1 - under_case(objects) / n0, 0, 1)),
        "plug_in_bin": float(ok and plug_in_bin(objects)),
        "stack_not_moved": float(not _MON["moved"]),
        "stack_not_tipped": float(not _MON["tipped"]),
        "released": float(ok and released(objects)),
    }


# the cord must stay under the case's front strip and the plug within reach of the bin; small shifts only
VARIATION = {
    "cable:cord": {"dx": 0.005, "dy": 0.01, "yaw": 0.03},
    "obj:base_case": {"dx": 0.005, "dy": 0.01, "yaw": 0.05},
    "obj:top_case": {"dx": 0.005, "dy": 0.005, "yaw": 0.05},
}

# -- closed-loop scripted demo (cabledemo.py) -------------------------------------------------------------------------
# Left arm: pinch the base case's handle (yaw -pi/2, fingers close along x, front-to-back across the bar) and lift the handle 2 cm; the stack pivots
# on its back edge and the cord is free. Right arm: pinch the plug across its width (yaw -pi/2: wrist camera looks
# along the pull towards the bin), raise it above the bin rim, draw the cord out to the right until its tail is clear
# of the case, lower the plug into the bin and let go. Then the left arm sets the case down and lets go.
LIFT = float(os.environ.get("CASES_LIFT", "0.02"))
HOVER = TOP + 0.20


def _log(ctx, tag=""):
    st = ctx.states()
    c = st[CORD.name]
    print(f"[t={ctx.t:5.2f}] {tag}: under={under_case(st)} tail={np.round(c.start_tip, 3).tolist()} "
          f"plug={np.round(c.end_plug, 3).tolist()} in_bin={plug_in_bin(st)} "
          f"move_b={case_move(st, BASE.name) * 100:.1f}cm move_t={case_move(st, TOPC.name) * 100:.1f}cm "
          f"tilt_b={tilt_deg(st[BASE.name]):.1f} tilt_t={tilt_deg(st[TOPC.name]):.1f} viol={violations()} "
          f"max_move={_MON['max_move'] * 100:.1f} max_tilt={_MON['max_tilt']:.1f} rel={released(st)} "
          f"score={score(st):.2f}", flush=True)


def _then(*gens):
    for g in gens:
        yield from g


LPARK = (0.15, 0.0, TOP + 0.18)  # end pose of the left TCP (see demo_script)
LY = -math.pi / 2  # left grip yaw: fingers close along x (front-to-back across the bar); wrist camera sits 8 cm to
# the right of the handle, in front of the stack, looking down at where the cord leaves the case


def _grab_handle(ctx, L, f_open, f_closed):
    h = np.asarray(ctx.states()[HANDLE.name].pos)
    yield from L.move((h[0], h[1], h[2] + 0.10), yaw=LY, finger=-0.022, duration=1.8)
    h = np.asarray(ctx.states()[HANDLE.name].pos)
    yield from L.move((h[0], h[1], h[2] + 0.006), yaw=LY, duration=1.2)
    yield from L.grip(f_closed, 0.6)
    _log(ctx, "handle gripped")


def _lift_edge(ctx, L, lift):
    t = L.tcp()
    yield from L.move((t[0], t[1], t[2] + lift), yaw=LY, duration=1.5)
    yield from L.wait(0.5)
    _log(ctx, "edge lifted")


def _set_down(ctx, L, lift, f_open, release=True):
    t = L.tcp()
    yield from L.move((t[0], t[1], t[2] - lift - 0.002), yaw=LY, duration=1.5)
    yield from L.wait(0.4)
    _log(ctx, "case set down")
    if not release:
        return
    yield from L.grip(-0.022, 0.6)
    t = L.tcp()
    yield from L.move((t[0], t[1], t[2] + 0.04), yaw=LY, duration=0.4)  # let go cleanly: straight up off the bar


PY = -math.pi / 2  # right grip yaw: fingers close along x, across the plug; wrist camera faces -y (the bin)


def _pull_cord(ctx, Rt, f_open, f_closed):
    p = np.asarray(ctx.states()[CORD.name].end_plug)
    yield from Rt.move((p[0], p[1], TOP + 0.08), yaw=PY, finger=f_open, duration=1.6)
    p = np.asarray(ctx.states()[CORD.name].end_plug)
    yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=PY, duration=1.0)
    yield from Rt.grip(f_closed, 0.6)
    _log(ctx, "plug gripped")
    yield from Rt.move((p[0], p[1], TOP + 0.065), yaw=PY, duration=0.8)
    y_end = BIN_XY[1] - 0.015
    yield from Rt.move((BIN_XY[0], y_end, TOP + 0.065), yaw=PY, duration=4.0)
    _log(ctx, "cord drawn")


def _bin_plug(ctx, Rt, f_open):
    t = Rt.tcp()
    yield from Rt.move((t[0], t[1], TOP + 0.030), yaw=PY, duration=0.8)
    yield from Rt.grip(-0.014, 0.6)
    t = Rt.tcp()
    yield from Rt.move((t[0], t[1], TOP + 0.12), yaw=PY, duration=0.8)
    _log(ctx, "plug binned")


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    mode = os.environ.get("CASES_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "nothing":
        yield from L.wait(6.0)
        _log(ctx, "end")
        print(f"[cases] events={_MON['events']}", flush=True)
        return
    if mode == "yank":  # one arm: draw the cord out by its plug with the stack still on it
        yield from _pull_cord(ctx, Rt, FINGER_OPEN, FINGER_CLOSED)
        yield from _bin_plug(ctx, Rt, FINGER_OPEN)
        yield from Rt.wait(1.5)
        _log(ctx, "end")
        print(f"[cases] events={_MON['events']}", flush=True)
        return
    lift = 0.07 if mode == "tip" else LIFT
    yield from ctx.both(_grab_handle(ctx, L, FINGER_OPEN, FINGER_CLOSED),
                        Rt.move((0.24, -0.20, TOP + 0.10), yaw=PY, finger=FINGER_OPEN, duration=1.8))
    yield from _lift_edge(ctx, L, lift)
    yield from _pull_cord(ctx, Rt, FINGER_OPEN, FINGER_CLOSED)
    yield from _bin_plug(ctx, Rt, FINGER_OPEN)
    yield from _set_down(ctx, L, lift, FINGER_OPEN, release=(mode != "hold"))
    if mode != "hold":  # the open left gripper rises to 18 cm above the table in front of the stack's right corner,
        # so its wrist camera looks at the case's right front corner and the freed cord tail beyond it; the open
        # right gripper stays 12 cm above the bin, so its wrist camera looks down at the plug lying in the bin
        yield from L.move(LPARK, yaw=LY, duration=1.0)
    yield from ctx.both(L.wait(2.5), Rt.wait(2.5))
    _log(ctx, "end")
    print(f"[cases] events={_MON['events']}", flush=True)
