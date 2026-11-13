"""Rod a closed duct to find where it is blocked, mark the spot on the duct lid, and withdraw the rod.

A grey square duct (20 mm bore, closed lid, capped far end) lies on the table along the robot's left-right axis, far
out in front of the arms. Its open mouth is on the left; inside, out of sight, a 7 cm block fills the bore. Where it
sits is drawn afresh at every reset (seeded): its face is 53-75 cm from the mouth. A yellow fibreglass duct rod with
black bands every 10 cm from its brass bullet tip has its tip just inside the mouth; the rest lies on the table in one
wide bend (13 cm radius). An orange marker block stands on a small grey stand on the far right. White ticks on the
lid mark every 10 cm from the mouth.

The left arm feeds the rod in, regripping in short strokes, until it stops (the bullet against the block); the length
of rod inside is the distance to the blockage. The right arm sets the marker on the lid over that point. The left arm
pulls the rod back out, and both let go. The mouth and the rod are out of the right arm's reach, the stand out of the
left arm's reach (position-only IK, any gripper orientation).

Score (end state; 1.0 must hold 1 s): 0 unless the rod tip reached the plug face during the trial (latched) before the
marker ever came onto the lid (latched), both grippers are clear of the rod and the marker, and the marker rests on
the lid. Marker centre within 2 cm (along the duct) of this trial's plug face: 1.0 with the whole rod out of the duct, 0.6 with
the rod still in it; 2-4 cm off: 0.3; farther: 0.
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
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("This duct is blocked somewhere along its length. Use the yellow rod to find where, put the orange "
               "marker on top of the duct right over the blockage, and pull the rod back out. Then let go.")

# -- duct (fixed) along y at x = DX: mouth at y = YM (left), capped end at y = YC (right) ---------------------------
DX = float(os.environ.get("RD_DX", 0.49))
YM = 0.34
YC = -0.48
BORE = float(os.environ.get("RD_BORE", 0.020))
WALL = 0.003
LID_W = 0.050
LID_T = 0.003
FZ = TOP + WALL  # bore floor
LIDZ = FZ + BORE + LID_T  # lid top
# The blockage: a 7 cm block filling the bore. Its face (the end towards the mouth) lies DEPTH m from the mouth, drawn
# afresh at every reset, uniformly in [DEPTH_MIN, DEPTH_MAX] (seeded: the episode seed, else RD_SEED + reset count).
BLK_L = 0.07
DEPTH_MIN, DEPTH_MAX = 0.53, 0.75  # spot y -0.18 .. -0.41: where the right arm can set the marker down
DEPTH_DEFAULT = 0.75  # spawn pose only; replaced at every reset
GREY = (0.55, 0.56, 0.57)
DARK = (0.30, 0.30, 0.31)
WHITE = (0.95, 0.95, 0.93)
_L = YM - YC
_cy = (YM + YC) / 2
BLK_Z = FZ + BORE / 2 - 0.0005
DUCT = [
    Box("duct_floor", pos=(DX, _cy, TOP + WALL / 2), size=(BORE + 2 * WALL, _L, WALL), color=GREY),
    Box("duct_wall_near", pos=(DX - BORE / 2 - WALL / 2, _cy, FZ + BORE / 2), size=(WALL, _L, BORE), color=GREY),
    Box("duct_wall_far", pos=(DX + BORE / 2 + WALL / 2, _cy, FZ + BORE / 2), size=(WALL, _L, BORE), color=GREY),
    Box("duct_lid", pos=(DX, _cy, LIDZ - LID_T / 2), size=(LID_W, _L, LID_T), color=GREY),
    Box("duct_cap", pos=(DX, YC - WALL / 2, FZ + BORE / 2), size=(BORE + 2 * WALL, WALL, BORE + 2 * WALL),
        color=GREY),
]
# kinematic (moved only by the reset, never by contact): a mass is given only so the kit makes it a movable body
BLOCKAGE = Box("blockage", pos=(DX, YM - DEPTH_DEFAULT - BLK_L / 2, BLK_Z), size=(BORE - 0.001, BLK_L, BORE - 0.001),
               mass=0.05, color=DARK)
# ticks every 10 cm from the mouth: two short white stubs at the lid edges (the middle of the lid stays flat)
TICK_YS = [YM - 0.10 * k for k in range(1, 9)]
TICKS = []
for _k, _y in enumerate(TICK_YS, start=1):
    for _s, _side in ((-1, "n"), (1, "f")):
        TICKS.append(Box(f"tick_{_k}{_side}", pos=(DX + _s * (LID_W / 2 - 0.006), _y, LIDZ + 0.0004),
                         size=(0.012, 0.004, 0.0008), color=WHITE))

# -- marker block on its stand (right) --------------------------------------------------------------------------------
MK = (0.025, 0.025, 0.010)
MK_MASS = 0.010
STAND_XY = (0.20, -0.52)
STAND_H = 0.040
STAND = Box("marker_stand", pos=(STAND_XY[0], STAND_XY[1], TOP + STAND_H / 2), size=(0.018, 0.018, STAND_H), color=GREY)
MARKER = Box("marker", pos=(STAND_XY[0], STAND_XY[1], TOP + STAND_H + MK[2] / 2 + 0.0005), size=MK,
             mass=MK_MASS, color=(0.98, 0.45, 0.05))

# -- the duct rod: start end = bullet tip end, inside the mouth --------------------------------------------------------
R = 0.003  # 6 mm fibreglass rod
ROD_LEN = 0.90
BULLET = Plug(size=(0.014, 0.006, 0.006), mass=0.006, color=(0.78, 0.60, 0.22))
Z_IN = FZ + R + 0.0005
Z_OUT = TOP + R + 0.0005
TIP_IN = 0.030  # bullet front 3 cm inside the mouth at the start
ROD_START = (DX, YM - TIP_IN + BULLET.size[0], Z_IN)
ROD = cable("rod", start=ROD_START, yaw=math.pi / 2, length=ROD_LEN, radius=R, start_plug=BULLET,
            cable_mass=0.04, color=(0.95, 0.80, 0.08), bend_stiffness=float(os.environ.get("RD_K", 0.02)),
            bend_damping=0.002, friction=float(os.environ.get("RD_FRIC", 0.5)))
# black bands: the segment whose centre is 10, 20, ... cm from the bullet tip
BAND_SEGS = [int(round((0.10 * j - BULLET.size[0]) / ROD.seg_len - 0.5)) for j in range(1, 9)]
BAND_SEGS = [k for k in BAND_SEGS if 0 <= k < ROD.segments]

OBJECTS = [*DUCT, BLOCKAGE, *TICKS, STAND, MARKER, ROD]


def _install_bands() -> None:
    """cablekit has no per-segment colour: paint the band segments black (visual only)."""
    orig = cablekit._spawn_cable
    if getattr(orig, "_rod_bands", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        try:
            import isaaclab.sim as sim_utils
            from isaaclab.sim.utils import bind_visual_material
            from isaaclab.sim.utils.stage import get_current_stage
            from yam_arena.station import linear_rgb

            if cfg.cable_name == ROD.name:
                look = f"{prim_path}/Looks/band"
                mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb((0.04, 0.04, 0.04)), roughness=0.5)
                mat.func(look, mat)
                for s in BAND_SEGS:
                    bind_visual_material(f"{prim_path}/seg_{s}/geom", look, stage=get_current_stage())
        except Exception as e:  # pragma: no cover - never break the scene for a colour
            print(f"[rod task] bands not applied: {e}", flush=True)
        return prim

    spawn._rod_bands = True
    cablekit._spawn_cable = spawn


_install_bands()


# -- starting lay of the rod (cableshape): straight out of the mouth for 11 cm, then one 180-degree bend of 13 cm
#    centreline radius away from the robot (no tighter than a 6 mm duct rod's rated minimum bend radius, 127 mm), then
#    straight back along -y on the far side of the table, out of the right arm's reach ------------------------------
Y_BEND = YM + 0.11  # the bend starts here
R_U = 0.13


def _arc(c, r, a0, a1, z, n=60):
    a = np.linspace(a0, a1, n)
    return np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a), np.full(n, z)]


def _path() -> np.ndarray:
    s0 = ROD_START[1]
    inside = np.c_[np.full(20, DX), np.linspace(s0, YM + 0.005, 20), np.full(20, Z_IN)]
    drop = np.c_[np.full(10, DX), np.linspace(YM + 0.01, YM + 0.05, 10), np.linspace(Z_IN, Z_OUT, 10)]
    lead = np.c_[np.full(20, DX), np.linspace(YM + 0.055, Y_BEND, 20), np.full(20, Z_OUT)]
    u = _arc((DX + R_U, Y_BEND), R_U, math.pi, 0.0, Z_OUT)
    tail = np.c_[np.full(40, DX + 2 * R_U), np.linspace(Y_BEND - 0.005, Y_BEND - 0.45, 40), np.full(40, Z_OUT)]
    return np.vstack([inside, drop, lead, u, tail])


SHAPE_PATH = _path()

# -- episode state (latched) ----------------------------------------------------------------------------------------
_ST: dict = {"env": None, "rodded": False, "early": False, "nan": False, "min_tip": 9.0, "seed": None, "resets": 0,
             "depth": DEPTH_DEFAULT}


def _reset_state() -> None:
    _ST.update(rodded=False, early=False, nan=False, min_tip=9.0)


def draw_depth(seed: int | None, n_reset: int) -> float:
    """Blockage face distance from the mouth for one trial: the episode seed if the runner gives one, else
    RD_SEED (default 0) plus the number of earlier resets in this process. Uniform in [DEPTH_MIN, DEPTH_MAX], to 1 mm."""
    s = int(seed) if seed is not None else int(os.environ.get("RD_SEED", "0")) + n_reset
    d = np.random.default_rng([s, 1101]).uniform(DEPTH_MIN, DEPTH_MAX)
    return round(float(d), 3)


def _place_blockage(emb, depth: float) -> None:
    """Teleport the kinematic blockage so its face is ``depth`` from the mouth."""
    base = emb._base
    obj = base.scene[BLOCKAGE.name]
    pose = obj.data.default_root_pose.torch.clone() if hasattr(obj.data.default_root_pose, "torch") \
        else obj.data.default_root_pose.clone()
    pose[:, :3] += base.scene.env_origins
    pose[:, 1] = base.scene.env_origins[:, 1] + (YM - depth - BLK_L / 2)
    if hasattr(obj, "write_root_pose_to_sim_index"):
        obj.write_root_pose_to_sim_index(root_pose=pose)
    else:  # pragma: no cover - older Isaac Lab
        obj.write_root_pose_to_sim(pose)


def _shape(emb):
    _reset_state()
    _ST["env"] = emb._env
    _ST["depth"] = draw_depth(_ST["seed"], _ST["resets"])
    _ST["resets"] += 1
    _place_blockage(emb, _ST["depth"])
    print(f"[rod task] blockage face {_ST['depth'] * 100:.1f} cm from the mouth", flush=True)
    cableshape.preshape(emb, ROD, SHAPE_PATH, hold="ramp", settle_s=0)
    cableshape.settle(emb, 2.0)


cableshape.on_reset(_shape)


def _install_seed_hook() -> None:
    """Keep the episode seed the runner passes to reset() for draw_depth (the reset hooks get only the embodiment)."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_rod_seed", False):
        return
    inner = cls.reset

    def reset(self, *a, **k):
        _ST["seed"] = k.get("seed")
        return inner(self, *a, **k)

    reset._rod_seed = True
    cls.reset = reset


_install_seed_hook()


def _patch_scene(ys) -> None:
    if getattr(ys.task_object, "_rod_kin", False):
        return
    base = ys.task_object

    def task_object(spec):
        if spec.name != BLOCKAGE.name:
            return base(spec)
        orig = ys.PhysxRigidBodyPropertiesCfg
        ys.PhysxRigidBodyPropertiesCfg = lambda **k: orig(**{**k, "kinematic_enabled": True})
        try:
            return base(spec)
        finally:
            ys.PhysxRigidBodyPropertiesCfg = orig

    task_object._rod_kin = True
    ys.task_object = task_object


def _install_kinematic_blockage() -> None:
    """The blockage is a kinematic body: the reset can move it, nothing in the trial can. yam_arena.scene imports the
    simulator, so it is patched when it is first imported (after the app starts), not here."""
    import importlib.abc
    import importlib.util

    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
        return
    if any(getattr(f, "_rod_kin", False) for f in sys.meta_path):
        return

    class _Finder(importlib.abc.MetaPathFinder):
        _rod_kin = True

        def find_spec(self, name, path, target=None):
            if name != "yam_arena.scene":
                return None
            sys.meta_path.remove(self)
            spec = importlib.util.find_spec(name)
            if spec is None or spec.loader is None:
                return spec
            run = spec.loader.exec_module

            def exec_module(module):
                run(module)
                _patch_scene(module)

            spec.loader.exec_module = exec_module
            return spec

    sys.meta_path.insert(0, _Finder())


_install_kinematic_blockage()


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_rod_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ST["env"] = self._env
        return base(self, *a, **k)

    _task_step._rod_hooked = True
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
REACH_TOL = 0.015  # bullet front within 1.5 cm of the plug face = the rod reached the blockage
MARK_TOL = 0.02  # full / 0.6 credit: marker centre within 2 cm of the plug face (along the duct)
MARK_TOL2 = 0.04  # 0.3 credit: within 4 cm
RELEASE_ROD = 0.03  # every gripper TCP at least this far from every point of the rod
RELEASE_MK = 0.04  # ... and from the marker centre
EARLY_DZ = 0.02  # marker centre less than 2 cm above the lid top, over the lid = "on the lid"


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def rod_points(objects) -> np.ndarray:
    c = objects["rod"]
    pts = c.polyline()
    if c.start_plug is not None:
        pts = np.vstack([c.start_plug[None, :], pts])
    return _dense(pts)


def tip_front(objects) -> float:
    """y of the bullet's front face (m)."""
    c = objects["rod"]
    return float(c.start_plug[1] - BULLET.size[0] / 2)


def in_duct(p) -> bool:
    return bool(abs(p[0] - DX) < BORE / 2 + 0.006 and YC <= p[1] < YM - 0.003 and p[2] < LIDZ)


def rod_out(objects) -> bool:
    """No point of the rod or bullet inside the duct."""
    return not any(in_duct(p) for p in rod_points(objects))


def inserted(objects) -> float:
    """Length of rod (bullet front included) inside the duct, measured from the mouth (m)."""
    return max(0.0, YM - tip_front(objects)) if in_duct(objects["rod"].start_plug) else 0.0


def over_lid(p) -> bool:
    return bool(abs(p[0] - DX) <= LID_W / 2 and YC <= p[1] <= YM)


def marker_on_lid(objects) -> bool:
    """Marker resting on the lid: centre over the lid, its bottom within 5 mm of the lid top, still."""
    m = objects["marker"]
    return bool(over_lid(m.pos) and m.low[2] <= LIDZ + 0.005 and m.pos[2] >= LIDZ and m.resting(0.02, 1.0))


def plug_face(objects) -> float:
    """y of this trial's blockage face (the end towards the mouth), read from the blockage body."""
    return float(objects[BLOCKAGE.name].pos[1]) + BLK_L / 2


def mark_error(objects) -> float:
    return abs(float(objects["marker"].pos[1]) - plug_face(objects))


def released(objects) -> bool:
    g = grippers()
    if not g:
        return True
    pts = rod_points(objects)
    m = objects["marker"].pos
    return all(float(np.linalg.norm(pts - x, axis=1).min()) >= RELEASE_ROD and float(np.linalg.norm(m - x)) >= RELEASE_MK
               for x in g)


def _finite(objects) -> bool:
    return bool(np.isfinite(objects["rod"].points).all() and np.isfinite(objects["marker"].pos).all())


def _track(objects) -> None:
    """Latches, evaluated every control step (score/success/distance are called each step)."""
    if not _finite(objects):
        _ST["nan"] = True
        return
    c = objects["rod"]
    if in_duct(c.start_plug):
        _ST["min_tip"] = min(_ST["min_tip"], tip_front(objects))
        if tip_front(objects) <= plug_face(objects) + REACH_TOL:
            _ST["rodded"] = True
    m = objects["marker"].pos
    if not _ST["rodded"] and over_lid(m) and m[2] <= LIDZ + EARLY_DZ:
        _ST["early"] = True  # marker brought onto the lid before the rod had found the blockage


def score(objects) -> float:
    """0 if the marker came onto the lid before the rod tip reached the plug face (latched), if the rod tip never
    reached the plug face, if a gripper is within 3 cm of the rod or 4 cm of the marker, or if the marker does not rest
    on the lid. Otherwise by the marker centre's distance along the duct from the plug face: <= 2 cm -> 1.0 if no part of
    the rod is left in the duct, else 0.6; <= 4 cm -> 0.3; farther -> 0."""
    _track(objects)
    if _ST["nan"] or _ST["early"] or not _ST["rodded"]:
        return 0.0
    if not released(objects) or not marker_on_lid(objects):
        return 0.0
    e = mark_error(objects)
    if e <= MARK_TOL:
        return 1.0 if rod_out(objects) else 0.6
    return 0.3 if e <= MARK_TOL2 else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Marker centre to the spot over the plug face (m)."""
    _track(objects)
    m = objects["marker"].pos
    return float(np.linalg.norm(np.array([m[0] - DX, m[1] - plug_face(objects), m[2] - LIDZ])))


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    if not _finite(objects):
        return {"nan": 1}
    return {
        "rod_depth": float(np.clip((YM - _ST["min_tip"]) / (YM - plug_face(objects)), 0.0, 1.0)),
        "rodded": int(_ST["rodded"]),
        "marker_on_lid": int(marker_on_lid(objects)),
        "mark_within_2cm": int(marker_on_lid(objects) and mark_error(objects) <= MARK_TOL),
        "rod_out": int(rod_out(objects)),
        "released": int(released(objects)),
        "no_early_mark": int(not _ST["early"]),
    }


VARIATION = {"cable:rod": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}, "obj:marker": {"dx": 0.005, "dy": 0.005, "yaw": 0.2}}

TWIN_EXTRA = [
    {"name": "bullet_tip", "kind": "body", "size_mm": [14, 6, 6], "colour": "brass",
     "note": "brass rodder bullet glued on the rod end"},
    {"name": "rod_bands", "kind": "decal", "size_mm": [20, 6], "colour": "black",
     "note": "black tape bands, centres every 10 cm from the bullet tip"},
]

# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
# Actions come only from the top camera image (vision.py), the arms' own TCPs and the fixed layout. ctx.states() is read
# only by _log() (printed for the run log, never used to choose a motion) and by the oracle test variant "early".
HI = TOP + 0.07
GRASP_OPEN = -0.014
YAW_L = float(os.environ.get("RD_YAWL", -math.pi / 2))  # left wrist camera looks along -y: into the mouth
NEAR = YM + 0.022  # closest left TCP y to the mouth
S_PINCH = 0.10  # pinch the rod this far (along the rod) from the mouth
PUSH = S_PINCH - (NEAR - YM)  # rod pushed in per full stroke (0.078 m)
PARK_L = (0.10, 0.44, TOP + 0.10)  # left arm out of the top camera's view of the rod while looking
ROD_HUE, ROD_HUE_TOL = float(os.environ.get("RD_HUE", 52)), float(os.environ.get("RD_HTOL", 12))
ROD_SAT = float(os.environ.get("RD_SAT", 0.45))
MK_HUE = 26
STALL = 0.035  # a stroke that takes in less than this (camera) did not go in freely: the rod has met the blockage


def _rod_mask(img):
    import vision

    return vision.hsv_mask(img, ROD_HUE, ROD_HUE_TOL, min_sat=ROD_SAT, min_val=90)


_SAVE = {"n": 0}


def _see_rod(ctx, tag=""):
    """Top camera: the yellow rod traced from the duct mouth outwards (vision.Polyline, arc length from the mouth)."""
    import vision

    img = vision.top_image(ctx)
    poly = vision.trace_cable(img, _rod_mask(img), Z_OUT, (DX, YM))
    d = os.environ.get("RD_SAVE")
    if d:  # debug only: raw frame + the simulator's rod for offline calibration
        pathlib.Path(d).mkdir(parents=True, exist_ok=True)
        st = ctx.states()
        n = _SAVE["n"] = _SAVE["n"] + 1
        np.savez(f"{d}/look_{n:02d}.npz", img=np.asarray(ctx.obs.images["top_cam"]), pts=st["rod"].points,
                 plug=st["rod"].start_plug, tag=tag)
    return poly


def _out_len(poly) -> float:
    """Rod length seen outside the mouth: the bead chain's length plus the stretch from its last bead to the farthest
    rod pixel straight ahead (the beads stop up to ~2 cm short of the rod's free end)."""
    b = poly.points
    d = b[-1] - b[-2]
    d = d / (np.linalg.norm(d) + 1e-12)
    v = poly.raw - b[-1]
    ahead = v @ d
    m = (np.linalg.norm(v, axis=1) < 0.05) & (ahead > 0) & (np.abs(v @ np.array([-d[1], d[0]])) < 0.012)
    return poly.length + (float(ahead[m].max()) if m.any() else 0.0)


def _look(ctx, L, tag=""):
    """Park the left arm out of the camera's line of sight, wait, and trace the rod (3 tries)."""
    p = L.tcp()
    if np.linalg.norm(p - np.array(PARK_L)) > 0.02:
        yield from L.move((p[0], p[1], max(p[2], TOP + 0.05)), yaw=L._yaw, duration=0.25)
        yield from L.move(PARK_L, yaw=0.0, duration=0.7)
    poly = None
    for _ in range(3):
        yield from L.wait(0.3)
        poly = _see_rod(ctx, tag)
        if poly is not None:
            break
    return poly


def _grip_yaws(th):
    """(yaw to pinch the rod with, yaw to hold it on the duct axis with), given ``th``, the rod's direction at the pinch
    towards the mouth (camera). The fingers close across the rod; the wrist turns the pinched rod the short way until
    the part towards the tip points along -y into the mouth (both yaws within +-90 deg)."""
    y0 = (th + math.pi / 2) % math.pi - math.pi / 2
    d = (-math.pi / 2 - th + math.pi) % (2 * math.pi) - math.pi
    return y0, y0 + d


def _pinch_at(ctx, L, fc, xy, gy):
    p = L.tcp()
    yield from L.move((xy[0], xy[1], max(p[2], TOP + 0.04)), yaw=gy, finger=GRASP_OPEN, duration=0.6)
    yield from L.move((xy[0], xy[1], Z_OUT + 0.010), yaw=gy, duration=0.4)
    yield from L.grip(fc, 0.35)


_FEED: dict = {}


def _feed(ctx, L, fc, *, stop_depth=None, max_strokes=14):
    """Left arm, stroke by stroke. Each stroke: look (top camera) at the rod outside the mouth, pinch it S_PINCH along
    the rod from the mouth, draw that length onto the duct axis and push it in until the fingers are 2.2 cm from the
    mouth; let go and look again. The rod length visible outside (traced from the mouth) shrinks by what went in. A
    stroke that takes in less than STALL means the bullet has met the blockage; one more stroke confirms it. Returns
    (in _FEED["depth"]) the length of rod inside: 3 cm at the start (the fixed start lay) plus how much shorter the rod
    seen outside the mouth is now than at the start."""
    poly = yield from _look(ctx, L, "start")
    l0 = l_prev = _out_len(poly) if poly is not None else None
    depth, stalls, log = TIP_IN, 0, []
    for n in range(max_strokes):
        if stop_depth is not None and depth >= stop_depth:
            break
        hit = poly.at(S_PINCH) if poly is not None else None
        if hit is None:
            print(f"[demo] stroke {n + 1}: rod not seen {S_PINCH} m out of the mouth", flush=True)
            poly = yield from _look(ctx, L, "retry")
            continue
        xy, th = hit
        gy, fy = _grip_yaws(th)
        yield from _pinch_at(ctx, L, fc, xy, gy)
        reach = YM + S_PINCH
        z1 = Z_IN + 0.010
        yield from L.move((DX, reach, z1), yaw=fy, duration=0.4)
        if fy > 0:
            # the rod lay the other way round, so the wrist camera now faces away from the mouth: set the straightened
            # rod down on the axis and pinch it again (where this hand just laid it) with the camera facing the mouth
            yield from L.grip(GRASP_OPEN, 0.25)
            yield from L.move((DX, reach, TOP + 0.04), yaw=0.0, duration=0.35)
            yield from L.move((DX, reach, TOP + 0.04), yaw=YAW_L, duration=0.35)
            yield from L.move((DX, reach, Z_OUT + 0.010), yaw=YAW_L, duration=0.35)
            yield from L.grip(fc, 0.3)
            yield from L.move((DX, reach, z1), yaw=YAW_L, duration=0.25)
            fy = YAW_L
        for i in range(4):
            yield from L.move((DX, reach + (NEAR - reach) * (i + 1) / 4, z1), yaw=fy, duration=0.25)
        yield from L.grip(GRASP_OPEN, 0.3)
        p = L.tcp()
        yield from L.move((p[0], p[1] + 0.03, TOP + 0.05), yaw=fy, duration=0.3)
        poly = yield from _look(ctx, L, f"stroke{n + 1}")
        l_now = _out_len(poly) if poly is not None else None
        adv = (l_prev - l_now) if (l_now is not None and l_prev is not None) else PUSH
        if l_now is not None and l0 is not None:
            depth = TIP_IN + (l0 - l_now)  # rod seen outside at the start minus rod seen outside now
            l_prev = l_now
        else:
            depth += PUSH  # rod not seen this time: count a full stroke, the next look corrects it
        log.append(round(adv, 3))
        _log(ctx, f"stroke {n + 1}: seen to go in {adv:.3f}, depth read {depth:.3f}")
        if adv < STALL:
            stalls += 1
            if stalls >= 2:
                break
        else:
            stalls = 0
    _FEED.update(depth=depth, log=log)


WIDE_OPEN = -0.030  # wider than GRASP_OPEN: withdrawal pinches land on a rod that lies at an angle to the axis


def _rod_at_mouth(ctx) -> bool:
    """Top camera: any rod pixel within 3 cm of the duct mouth (the rod still runs into the duct)."""
    import vision

    img = vision.top_image(ctx)
    p = vision.top_camera(img.shape).to_plane(_rod_mask(img), Z_OUT)
    return bool((np.linalg.norm(p - np.array([DX, YM + 0.012]), axis=1) < 0.03).any())


def _withdraw(ctx, L, fc, depth, keep=False):
    """Left arm pulls the rod back out: look, pinch the rod S_PINCH along it from the mouth (fingers opened wide), draw
    it 12 cm out along the axis and swing the pulled length aside towards the robot; repeat until the camera sees no
    rod at the mouth any more (the bullet is out and the pulled length lies aside)."""
    for n in range(14):
        poly = yield from _look(ctx, L, f"withdraw{n + 1}")
        if not _rod_at_mouth(ctx):
            _log(ctx, f"withdraw: no rod seen at the mouth after {n} strokes")
            return
        hit = poly.at(S_PINCH) if poly is not None else None
        if hit is None:
            continue
        xy, th = hit
        gy, fy = _grip_yaws(th)
        p = L.tcp()
        yield from L.move((xy[0], xy[1], max(p[2], TOP + 0.04)), yaw=gy, finger=WIDE_OPEN, duration=0.6)
        yield from L.move((xy[0], xy[1], Z_OUT + 0.010), yaw=gy, duration=0.4)
        yield from L.grip(fc, 0.45)
        z1 = Z_IN + 0.012
        for i in range(4):
            yield from L.move((DX, YM + S_PINCH + 0.12 * (i + 1) / 4, z1), yaw=fy, duration=0.25)
        q = L.tcp()
        for i in range(4):
            yield from L.move((q[0] - 0.12 * (i + 1) / 4, q[1] + 0.03 * (i + 1) / 4, z1 + 0.01), yaw=fy,
                              duration=0.25)
        if keep and n >= 1 and not _rod_at_mouth(ctx):
            _log(ctx, f"withdraw {n + 1}, rod kept in hand")
            return  # shortcut variant "hold": never let go of the withdrawn rod
        yield from L.grip(GRASP_OPEN, 0.3)
        _log(ctx, f"withdraw {n + 1}")


def _marker_xy(ctx):
    """Top camera: the orange marker's top face; the known stand position if it is not seen."""
    import vision

    img = vision.top_image(ctx)
    m = vision.hsv_mask(img, MK_HUE, 10, min_sat=0.6, min_val=90)
    xy = vision.blob_xy(img, m, TOP + STAND_H + MK[2], min_px=4)
    if xy is None or np.linalg.norm(np.asarray(xy) - np.array(STAND_XY)) > 0.03:
        return np.array(STAND_XY, dtype=float)
    return np.asarray(xy, dtype=float)


def _mark(ctx, Rt, fo, fc, y_mark):
    """Right arm: take the marker off its stand (camera / known stand) and set it on the lid at y_mark."""
    m = _marker_xy(ctx)
    zt = TOP + STAND_H + MK[2] / 2 + 0.0005
    yield from Rt.move((m[0], m[1], HI), yaw=0.0, finger=fo, duration=1.2)
    yield from Rt.move((m[0], m[1], zt + 0.004), yaw=0.0, duration=0.7)
    yield from Rt.grip(fc, 0.4)
    yield from Rt.move((m[0], m[1], HI), yaw=0.0, duration=0.6)
    yield from Rt.move((DX, y_mark, LIDZ + 0.04), yaw=0.0, duration=1.5)
    yield from Rt.move((DX, y_mark, LIDZ + MK[2] / 2 + 0.006), yaw=0.0, duration=0.8)
    yield from Rt.grip(fo, 0.4)
    p = Rt.tcp()
    yield from Rt.move((p[0], p[1], HI), yaw=0.0, duration=0.6)
    yield from Rt.move((0.15, -0.22, HI + 0.05), yaw=0.0, duration=1.0)  # park clear of the top camera's view of the lid


def _park_left(ctx, L):
    p = L.tcp()
    yield from L.move((p[0], p[1], HI), yaw=L._yaw, duration=0.5)
    yield from L.move((0.15, 0.22, HI + 0.05), yaw=0.0, duration=1.0)


def _log(ctx, tag=""):
    """Run-log line (simulator state printed for the log only; never used to choose a motion)."""
    st = ctx.states()
    m = st["marker"].pos
    print(f"[t={ctx.t:6.2f}] {tag}: TRUE inserted={inserted(st):.3f} face_depth={YM - plug_face(st):.3f} "
          f"rodded={_ST['rodded']} early={_ST['early']} marker={np.round(m, 3)} err={mark_error(st):.3f} "
          f"on_lid={marker_on_lid(st)} rod_out={rod_out(st)} rel={released(st)} score={score(st):.2f}", flush=True)


def demo_script(ctx):
    """RD_DEMO=correct (default) | nothing | hold (correct, but the left hand never lets go of the withdrawn rod) |
    onearm_left (rod and withdraw, no marker) | onearm_right (marker guessed at mid-duct, no rodding) | early (oracle
    test: marker set over the true blockage before rodding, then rod and withdraw) | partway (rod pushed 20 cm, marker
    at the 20 cm point) | fixed75 (rod to the stop, but the marker goes where the blockage was in the fixed-layout
    version, 75 cm from the mouth, whatever was read) | farend (rod to the stop, marker at the capped end) | tier06
    (correct, rod left in the duct) | tier03 (correct, marker 3 cm short of the spot read)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    mode = os.environ.get("RD_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, f"start mode={mode}")
    if mode == "nothing":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    if mode == "onearm_right":
        yield from _mark(ctx, Rt, FINGER_OPEN, FINGER_CLOSED, (YM + YC) / 2)
        yield from Rt.wait(2.0)
        _log(ctx, "end")
        return
    if mode == "early":  # oracle test variant: the simulator's blockage position, used to show a right guess placed
        yield from _mark(ctx, Rt, FINGER_OPEN, FINGER_CLOSED, plug_face(ctx.states()))  # first still scores 0
        _log(ctx, "marked first")
    yield from _feed(ctx, L, FINGER_CLOSED, stop_depth=0.20 if mode == "partway" else None)
    depth = _FEED["depth"]
    _log(ctx, f"rodded: depth read {depth:.3f} (strokes seen {_FEED['log']})")
    if mode == "onearm_left":
        yield from _withdraw(ctx, L, FINGER_CLOSED, depth)
        yield from _park_left(ctx, L)
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    y_mark = YM - depth
    if mode == "farend":
        y_mark = YC + 0.02
    elif mode == "fixed75":
        y_mark = YM - 0.75
    elif mode == "tier03":
        y_mark = YM - depth + 0.03
    if mode == "early":
        yield from _withdraw(ctx, L, FINGER_CLOSED, depth)
        yield from _park_left(ctx, L)
    elif mode == "tier06":
        yield from ctx.both(_park_left(ctx, L), _mark(ctx, Rt, FINGER_OPEN, FINGER_CLOSED, y_mark))
    else:
        yield from ctx.both(_withdraw(ctx, L, FINGER_CLOSED, depth, keep=mode == "hold"),
                            _mark(ctx, Rt, FINGER_OPEN, FINGER_CLOSED, y_mark))
        if mode == "hold":
            yield from L.wait(3.0)
        else:
            yield from _park_left(ctx, L)
    yield from L.wait(2.0)
    _log(ctx, "end")
