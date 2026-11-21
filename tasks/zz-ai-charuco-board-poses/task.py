"""A robot cell's camera calibration. A ChArUco calibration board (40 x 32 cm, 9 x 7 squares of 40 mm with
ArUco markers, printed on 3 mm aluminium composite, 0.45 kg) stands on its long edge in a slotted wooden stand in
front of the robot, its pattern facing a fixed Intel RealSense D455 on a short post near the far edge of the table,
which looks back at the robot. To calibrate the camera the board must be shown to it in still views covering what
calibration needs: straight on and tilted left, right, up and down, pushed to every edge of the image, and both near
and far, the whole pattern in frame each time. The board is too wide for one gripper: both hands carry it by its two
side edges as one rigid piece, then stand it back in its slot.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score \\
           --instruction "Calibrate the camera with the board, then put the board back in its stand."
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys
import tempfile
from collections import deque

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # 0.7475
SCORE_OVER = "end"

INSTRUCTION = "Calibrate the camera with the board, then put the board back in its stand."

# -- board (m; board frame: x = pattern-face normal, y = along the long edge (left), z = along the short edge (up)) ----
SQ, MK = 0.040, 0.030  # square and marker size
NX, NY = 9, 7  # squares along the long and short edge
MARGIN = 0.020
BW, BH, BT = NX * SQ + 2 * MARGIN, NY * SQ + 2 * MARGIN, 0.003  # 0.40 x 0.32 x 0.003
BOARD_MASS = 0.45
PAT_W, PAT_H = NX * SQ, NY * SQ  # printed pattern 0.36 x 0.28

# -- slotted stand at (30, 0): a 20 x 8 x 4 cm block with a 4 mm slot 3 cm deep, its mouth chamfered 6 mm at 45 deg -------------------------------------
STAND_X, STAND_Y = 0.30, 0.0
STAND_L, STAND_D, STAND_H = 0.20, 0.08, 0.040
SLOT_W, SLOT_DEPTH = 0.004, 0.030
SLOT_FLOOR = TOP + STAND_H - SLOT_DEPTH  # board's bottom edge rests here
WOOD = (0.62, 0.47, 0.30)
JAW_T = (STAND_D - SLOT_W) / 2
CHAMFER = 0.006  # lead-in at the slot mouth, 45 deg

# -- camera: D455 on a post at (62, 0), optical centre 25 cm up, looking back along -x, level ---------------------------
CAM_POS = np.array([0.62, 0.0, TOP + 0.25])
IMG_W, IMG_H = 1280, 800  # D455 colour stream (global shutter) used for calibration
FX = FY = 640.0  # 90 x 64 deg field of view at 1280 x 800
CX, CY = 640.0, 400.0
CAM_BODY = (0.026, 0.124, 0.029)  # D455: 124 x 26 x 29 mm, long side across the view
# OpenCV camera frame in the world: z forward (-x world), x right (+y world, the camera faces the robot), y down
CAM_R = np.column_stack([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]])

BOARD = Box("charuco_board", pos=(STAND_X, STAND_Y, SLOT_FLOOR + BH / 2), size=(BT, BW, BH), mass=BOARD_MASS,
            color=(0.93, 0.93, 0.92))
STAND_BASE = Box("stand_block", pos=(STAND_X, STAND_Y, TOP + (STAND_H - SLOT_DEPTH) / 2),
                 size=(STAND_D, STAND_L, STAND_H - SLOT_DEPTH), color=WOOD)
STAND_JAWS = [Box(f"stand_jaw_{s}", pos=(STAND_X + sgn * (SLOT_W / 2 + JAW_T / 2), STAND_Y, TOP + STAND_H - SLOT_DEPTH / 2),
                  size=(JAW_T, STAND_L, SLOT_DEPTH), color=WOOD) for s, sgn in (("front", 1), ("back", -1))]
CAM_POST = Cylinder("camera_post", pos=(CAM_POS[0] + 0.02, 0.0, TOP + (CAM_POS[2] - TOP - 0.0145) / 2), radius=0.010,
                    height=CAM_POS[2] - TOP - 0.0145, color=(0.15, 0.15, 0.16))


class LastPart(Box):
    """The camera body (fixed, last object); its spawner also prints the ChArUco pattern on the board."""


CAMERA = LastPart("calib_camera", pos=tuple(CAM_POS + np.array([0.013, 0.0, 0.0])), size=CAM_BODY, color=(0.12, 0.12, 0.13))

OBJECTS = [BOARD, STAND_BASE, *STAND_JAWS, CAM_POST, CAMERA]

TWIN_EXTRA = [
    {"name": "charuco_pattern", "kind": "decal", "size_mm": [360, 280, 0], "colour": "black/white",
     "note": "OpenCV CharucoBoard DICT_5X5_100, 9 x 7 squares of 40 mm, 30 mm markers, 20 mm white margin"},
]


# -- the printed pattern (OpenCV) -------------------------------------------------------------------------------------
def pattern_png() -> str | None:
    """Render the board's ChArUco pattern with OpenCV to a PNG (None if OpenCV's aruco module is missing)."""
    try:
        import cv2
        import cv2.aruco as aruco
    except Exception:  # noqa: BLE001
        return None
    d = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    board = aruco.CharucoBoard((NX, NY), SQ, MK, d)
    px = 5000  # px per m
    img = board.generateImage((int(round(BW * px)), int(round(BH * px))), marginSize=int(round(MARGIN * px)))
    path = os.path.join(tempfile.gettempdir(), "charuco_9x7_40mm.png")
    cv2.imwrite(path, img)
    return path


def _author(stage, parent: str) -> None:
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

    board = f"{parent}/{BOARD.name}"
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(board))
    rb.CreateSolverPositionIterationCountAttr(16)
    rb.CreateSolverVelocityIterationCountAttr(2)
    # aluminium composite: grippy rubber pads hold it by friction
    m = UsdShade.Material.Define(stage, f"{board}/board_phys")
    api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
    api.CreateStaticFrictionAttr(0.9)
    api.CreateDynamicFrictionAttr(0.8)
    api.CreateRestitutionAttr(0.0)
    from pxr import Usd

    for prim in Usd.PrimRange(stage.GetPrimAtPath(board)):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(m, UsdShade.Tokens.strongerThanDescendants, "physics")
    # the slot's 45 deg lead-in: one bevel strip on the inner top edge of each jaw (fixed colliders)
    for spec, sgn in ((STAND_JAWS[0], 1), (STAND_JAWS[1], -1)):
        jc = np.array(spec.pos)
        edge = np.array([STAND_X + sgn * SLOT_W / 2, STAND_Y, TOP + STAND_H])
        half = CHAMFER / 2 / math.sqrt(2)
        c = edge + np.array([sgn * half, 0.0, half]) + np.array([sgn * 0.001, 0.0, 0.001])  # clear of a board at rest
        g = UsdGeom.Cube.Define(stage, f"{parent}/{spec.name}/bevel")
        g.CreateSizeAttr(1.0)
        g.CreateDisplayColorAttr([Gf.Vec3f(*WOOD)])
        xf = UsdGeom.Xformable(g)
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, c - jc)))
        xf.AddRotateYOp().Set(float(-sgn * 45.0))
        xf.AddScaleOp().Set(Gf.Vec3f(0.002, float(STAND_L), float(CHAMFER)))
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    png = pattern_png()
    if png is None:
        return
    # a textured quad 0.1 mm in front of the board's +x face (visual only), in the board body frame
    mesh = UsdGeom.Mesh.Define(stage, f"{board}/pattern")
    x = BT / 2 + 0.0001
    hw, hh = BW / 2, BH / 2
    mesh.CreatePointsAttr([Gf.Vec3f(x, hw, -hh), Gf.Vec3f(x, -hw, -hh), Gf.Vec3f(x, -hw, hh), Gf.Vec3f(x, hw, hh)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(1, 0, 0)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateDoubleSidedAttr(False)
    # seen from the camera (looking along -x) the board's +y is on the image's right: u runs along -y... keep the
    # image upright and unmirrored as printed: left edge of the printed image at the board's +y side seen from the front
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    mat = UsdShade.Material.Define(stage, f"{board}/pattern_mat")
    sh = UsdShade.Shader.Define(stage, f"{board}/pattern_mat/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
    tex = UsdShade.Shader.Define(stage, f"{board}/pattern_mat/tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(png)
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    rd = UsdShade.Shader.Define(stage, f"{board}/pattern_mat/st")
    rd.CreateIdAttr("UsdPrimvarReader_float2")
    rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)


def _spawn_last(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_cuboid(prim_path, cfg, translation, orientation, **kwargs)
    _author(get_current_stage(), str(prim_path).rsplit("/", 1)[0])
    return prim


def _last_object(spec: LastPart):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import CollisionPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.station import linear_rgb

    _register_module()

    @configclass
    class LastCfg(sim_utils.CuboidCfg):
        func: object = clone(_spawn_last)

    cfg = LastCfg(size=tuple(spec.size), collision_props=CollisionPropertiesCfg(),
                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.5))
    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _register_module():
    if __name__ not in sys.modules:  # configclass needs this module registered (task files load unregistered)
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _patch_scene(scene) -> None:
    if getattr(scene, "_charuco_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, LastPart):
            return _last_object(spec)
        return orig(spec)

    scene.task_object = task_object
    scene._charuco_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _charuco = True

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


def _install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(getattr(f, "_charuco", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()

# -- env handle and per-episode state ---------------------------------------------------------------------------------
_ENV: dict = {"env": None}
HOLD_STEPS = 20  # 1 s at 20 Hz
PARK_STEPS = 10  # 0.5 s at 20 Hz
PARK_SPEED = 0.02  # m/s
_S: dict = {}


def _new_episode() -> None:
    _S.clear()
    _S.update(hist=deque(maxlen=HOLD_STEPS + 1), tcp_hist=deque(maxlen=PARK_STEPS + 1),
              views=set(), cover=set(), failed="", last_t=-1)


_new_episode()


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_charuco_hooked", False):
        return
    base_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        type(self)._charuco_env = self._env
        return base_step(self, *a, **k)

    def reset(self, *a, **k):
        _new_episode()
        return base_reset(self, *a, **k)

    _task_step._charuco_hooked = True
    cls._task_step = _task_step
    cls.reset = reset


_install_env_hook()


def _env():
    env = _ENV["env"]
    if env is None:
        try:
            from yam_arena import inspect_embodiment as ie

            env = getattr(ie.YamArenaEmbodiment, "_charuco_env", None)
        except Exception:  # noqa: BLE001
            env = None
    return env


def _rot(q_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in q_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _board(objects):
    o = objects[BOARD.name]
    return np.asarray(getattr(o, "root_pos", o.pos), float), _rot(o.quat_xyzw)


def grippers():
    """{arm: [tcp, pad tip]} in the env frame, or None outside the sim."""
    env = _env()
    if env is None:
        return None
    from yam_arena.runtime import pad_tips_world, tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    out = {}
    for arm, k in (("left", "left_robot"), ("right", "right_robot")):
        out[arm] = [np.asarray(tcp_world(base.scene[k]), float) - origin,
                    np.asarray(pad_tips_world(base.scene[k]), float) - origin]  # both on the line between the pads
    return out


# -- geometry of a view -----------------------------------------------------------------------------------------------
EDGE_GRIP = 0.012  # a pad tip within 1.2 cm of the plate counts as on it
COVER_Y = PAT_W / 2 - SQ  # 0.14: pads further out than this cover at most the outer column of squares
LIFT_CLEAR = 0.02  # board's lowest point at least 2 cm above the stand top and the table
IMG_MARGIN = 10  # px
STILL_POS, STILL_ROT = 0.002, math.radians(0.3)
STRAIGHT_MAX, TILT_MIN, TILT_MAX, OFF_AXIS_MAX = 8.0, 18.0, 50.0, 12.0  # deg
CAM_CLEAR = 0.03  # 3 cm keep-out around the D455 camera housing
_CAM_GRID = np.stack(np.meshgrid(
    CAMERA.pos[0] + np.linspace(-CAM_BODY[0] / 2, CAM_BODY[0] / 2, 3),
    CAMERA.pos[1] + np.linspace(-CAM_BODY[1] / 2, CAM_BODY[1] / 2, 13),
    CAMERA.pos[2] + np.linspace(-CAM_BODY[2] / 2, CAM_BODY[2] / 2, 3),
), axis=-1).reshape(-1, 3)
STOW_XY, STOW_Z, STOW_TILT = 0.010, 0.004, 8.0
CLEAR = 0.10


def project(pt) -> np.ndarray:
    """World point -> (u, v, depth) in the D455 colour image."""
    pc = CAM_R.T @ (np.asarray(pt, float) - CAM_POS)
    return np.array([FX * pc[0] / pc[2] + CX, FY * pc[1] / pc[2] + CY, pc[2]])


def pattern_corners(p, r):
    return [p + r @ np.array([0.0, sy * PAT_W / 2, sz * PAT_H / 2]) for sy in (-1, 1) for sz in (-1, 1)]


def tilts(r) -> tuple[float, float]:
    """(left-right, up-down) tilt of the board (deg) relative to the camera's image plane. A board facing the camera
    has its pattern normal along the camera's -z. Positive left-right: the board's right edge (seen from the camera)
    swung away from it; positive up-down: its top edge leaned away from it."""
    n = CAM_R.T @ r[:, 0]
    return math.degrees(math.atan2(n[0], -n[2])), math.degrees(math.atan2(-n[1], -n[2]))


def view_type(r) -> str:
    lr, ud = tilts(r)
    if abs(lr) < STRAIGHT_MAX and abs(ud) < STRAIGHT_MAX:
        return "straight"
    if TILT_MIN <= abs(lr) <= TILT_MAX and abs(ud) < OFF_AXIS_MAX:
        return "right" if lr > 0 else "left"
    if TILT_MIN <= abs(ud) <= TILT_MAX and abs(lr) < OFF_AXIS_MAX:
        return "up" if ud > 0 else "down"
    return ""


def in_frame(p, r) -> bool:
    for c in pattern_corners(p, r):
        u, v, d = project(c)
        if not (d > 0.1 and IMG_MARGIN <= u <= IMG_W - IMG_MARGIN and IMG_MARGIN <= v <= IMG_H - IMG_MARGIN):
            return False
    n = r[:, 0]
    return bool(n @ (CAM_POS - p) > 0)  # pattern side toward the camera


def _local(pt, p, r):
    return r.T @ (np.asarray(pt, float) - p)


def two_hand_edge_grip(p, r, gs) -> bool:
    """Both grippers on the board's side edges, one on each, their pads in the margin or the outer column."""
    sides = {}
    for arm, pts in gs.items():
        locs = [_local(t, p, r) for t in pts]
        on = all(abs(l[0]) <= BT / 2 + EDGE_GRIP and abs(l[1]) <= BW / 2 + EDGE_GRIP and abs(l[2]) <= BH / 2 + EDGE_GRIP
                 for l in locs)
        if not on or not all(abs(l[1]) >= COVER_Y for l in locs):
            return False
        sides[arm] = np.sign(np.mean([l[1] for l in locs]))
    return sides.get("left") != sides.get("right") and len(sides) == 2


EDGE_BAND_U, EDGE_BAND_V = 0.10 * IMG_W, 0.10 * IMG_H  # pattern reaches the outer 10% of the image on that side
NEAR_W, FAR_W = 0.60, 0.50  # pattern width as a fraction of the image width


def coverage(p, r) -> set:
    """Which image edges the pattern reaches in this view, and whether it is a near or far view (the X, Y and Size
    coverage of ROS camera_calibration)."""
    uv = np.array([project(c)[:2] for c in pattern_corners(p, r)])
    out = set()
    if uv[:, 0].min() <= EDGE_BAND_U:
        out.add("image left")
    if uv[:, 0].max() >= IMG_W - EDGE_BAND_U:
        out.add("image right")
    if uv[:, 1].min() <= EDGE_BAND_V:
        out.add("image top")
    if uv[:, 1].max() >= IMG_H - EDGE_BAND_V:
        out.add("image bottom")
    w = (uv[:, 0].max() - uv[:, 0].min()) / IMG_W
    if w >= NEAR_W:
        out.add("near")
    if w <= FAR_W:
        out.add("far")
    return out


def lowest_z(p, r) -> float:
    return min((p + r @ np.array([0.0, sy * BW / 2, sz * BH / 2]))[2] for sy in (-1, 1) for sz in (-1, 1))


def off_supports(p, r) -> bool:
    return lowest_z(p, r) >= TOP + STAND_H + LIFT_CLEAR


def _still(hist) -> bool:
    if len(hist) < HOLD_STEPS + 1:
        return False
    p0, r0 = hist[-1][0], hist[-1][1]
    for item in hist:
        p, r = item[0], item[1]
        if len(item) > 2 and not item[2]:
            return False
        if np.linalg.norm(p - p0) > STILL_POS:
            return False
        c = (np.trace(r0.T @ r) - 1) / 2
        if math.acos(max(-1.0, min(1.0, c))) > STILL_ROT:
            return False
    return True


def stowed(p, r) -> bool:
    up = math.degrees(math.acos(max(-1.0, min(1.0, r[2, 2]))))
    return bool(np.hypot(p[0] - BOARD.pos[0], p[1] - BOARD.pos[1]) <= STOW_XY and abs(lowest_z(p, r) - SLOT_FLOOR) <= STOW_Z
                and up <= STOW_TILT and r[0, 0] > 0.9)


def _board_dist(pt, p, r) -> float:
    l = _local(pt, p, r)
    return float(np.linalg.norm(np.maximum(np.abs(l) - np.array([BT / 2, BW / 2, BH / 2]), 0.0)))


def arms_clear(p, r, gs) -> bool:
    return all(_board_dist(t, p, r) >= CLEAR for pts in gs.values() for t in pts)


def arms_still(tcp_hist) -> bool:
    if len(tcp_hist) < PARK_STEPS + 1:
        return False
    t0 = tcp_hist[0]
    lim = PARK_SPEED * (PARK_STEPS / 20.0)
    return all(np.linalg.norm(cur[i] - t0[i]) <= lim for cur in tcp_hist for i in (0, 1))


def _cam_hit(p, r) -> bool:
    locs = (_CAM_GRID - p) @ r
    d = np.linalg.norm(np.maximum(np.abs(locs) - np.array([BT / 2, BW / 2, BH / 2]), 0.0), axis=1)
    return bool(d.min() < CAM_CLEAR)


def _update(objects) -> None:
    """Advance the per-episode record once per control step."""
    env = _env()
    t = int(env.unwrapped.episode_length_buf[0].item()) if env is not None and hasattr(env.unwrapped, "episode_length_buf") else None
    if t is not None:
        if t < _S["last_t"]:
            _new_episode()
        if t == _S["last_t"]:
            return
        _S["last_t"] = t
    p, r = _board(objects)
    if not (np.isfinite(p).all() and np.isfinite(r).all()):
        _S["failed"] = _S["failed"] or "board unreadable"
        return
    if _cam_hit(p, r):
        _S["failed"] = _S["failed"] or "board hit the camera"
    if lowest_z(p, r) < TOP + 0.004 and not stowed(p, r):
        _S["failed"] = _S["failed"] or "board dropped on the table"
    gs = grippers()
    if gs is None:
        _S["hist"].append((p.copy(), r.copy(), False, "", set()))
        return
    _S["tcp_hist"].append((gs["left"][0].copy(), gs["right"][0].copy()))
    v = view_type(r)
    _S["cur"] = v
    ok = bool(in_frame(p, r) and off_supports(p, r) and two_hand_edge_grip(p, r, gs))
    cov = coverage(p, r) if ok else set()
    _S["hist"].append((p.copy(), r.copy(), ok, v if ok else "", cov))
    if ok and _still(_S["hist"]):
        if v and all(h[3] == v for h in _S["hist"]):
            _S["views"].add(v)
        _S["cover"] |= set.intersection(*(h[4] for h in _S["hist"]))


VIEWS = ("straight", "left", "right", "up", "down")
COVER = ("image left", "image right", "image top", "image bottom", "near", "far")
PER_ITEM = 0.08


def score(objects) -> float:
    _update(objects)
    if _S["failed"]:
        return 0.0
    s = PER_ITEM * (len(_S["views"]) + len(_S["cover"]))
    if len(_S["views"]) == len(VIEWS) and len(_S["cover"]) == len(COVER):
        p, r = _board(objects)
        gs = grippers()
        if (gs is not None and stowed(p, r) and arms_clear(p, r, gs) and arms_still(_S["tcp_hist"])
                and objects[BOARD.name].resting(max_speed=0.01, max_spin=0.2)):
            s = 1.0
    return s


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[BOARD.name].resting(max_speed=0.01, max_spin=0.2)


def distance(objects) -> float:
    return 1.0 - score(objects)


def progress(objects) -> dict:
    p, r = _board(objects)
    lr, ud = tilts(r)
    gs = grippers()
    return {"views": sorted(_S["views"]), "cover": sorted(_S["cover"]), "failed": _S["failed"], "tilt_lr": round(lr, 1), "tilt_ud": round(ud, 1),
            "type": view_type(r), "in_frame": in_frame(p, r), "off": off_supports(p, r),
            "grip": two_hand_edge_grip(p, r, gs) if gs else None, "still": _still(_S["hist"]),
            "stowed": stowed(p, r), "clear": arms_clear(p, r, gs) if gs else None,
            "arms_still": arms_still(_S["tcp_hist"])}


VARIATION = {"obj:charuco_board": {"dx": 0.0, "dy": 0.02, "yaw": 0.0}}
