"""Morning resident hair care on the station table (metres from the arm-base midpoint, x forward, y left; heights above
the table top). An adult female care-training bust (Diane D150 head and shoulders) reclines at 35 deg on a foam
backrest wedge at the far side of the table (x 0.44-0.63 m). Her 4.0 kg head sits at (0.495, 0.035, 0.185) on a
compliant neck joint (0.15 N m/deg centring spring, +-45 deg stops) with a collar alignment stripe.

One 38 cm, 8.0 g dry Kanekalon synthetic hair weft (6 unbanded free-fibre sections S0..S5, 25 mm wide, 6.5 N magnetic
root breakaway clips) exits a PTFE scalp eyelet behind her right ear at (0.420, 0.0, 0.120), tied behind the slot to an
Ohaus 8003-MN 5 N tubular pull spring scale equipped with a sliding red peak-force telltale ring and a red 2.0 N limit
band. The lock drapes forward over her right shoulder (crest z 0.114) onto a white folded terry towel on her chest
(x 0.045-0.382, y -0.08 to +0.08, top z 0.055), with a 3-stage matted tangle 10-22 cm from the tips: C1 at s = 16 cm,
C2 at s = 22 cm, C3 at s = 28 cm. Green (s = 3 cm, x = 0.394) and blue (s = 21 cm, x = 0.246) stitched index marks sit
on the right edge of the towel. A Goody Super Comb wide-tooth detangling comb (19.6 cm long, 10 rounded teeth, 32 g)
rests flat in an instrument tray at (0.24, -0.24).

Caregiver method: grip the comb with one hand, pinch the hair lock firmly with the other between the scalp and the
active tangle (>= 2 cm nearer the scalp) so combing drag lands on the holding hand rather than her roots, comb the
distal tangle out first and step up through the middle and upper crossings, then run one continuous verification stroke
from within 3 cm of the scalp past the tips, return the comb to its tray, and park both hands clear.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --max-steps 1200 --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import dataclasses
import importlib.abc
import importlib.util
import math
import os
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Sphere  # noqa: E402

import hair as hr  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.5
RUN_LIMIT_STEPS = 1200

INSTRUCTION = (
    "Comb the tangle out of the lock of hair lying over her shoulder so the comb runs from scalp to tip "
    "without pulling on her scalp, then put the comb back on its tray."
)

# -- colours (sRGB) --------------------------------------------------------------------------------------------------
WEDGE_COL = (0.16, 0.25, 0.42)
GOWN_COL = (0.58, 0.76, 0.78)
SKIN_COL = (0.92, 0.75, 0.63)
HAIR_CAP_COL = (0.34, 0.17, 0.09)
SNARL_COL = (0.28, 0.13, 0.06)
TOWEL_COL = (0.95, 0.96, 0.97)
MARK_SCALP_COL = (0.15, 0.70, 0.28)
MARK_MID_COL = (0.18, 0.45, 0.85)
COMB_COL = (0.14, 0.38, 0.84)
TRAY_COL = (0.85, 0.86, 0.88)
SCALE_BODY_COL = (0.82, 0.84, 0.86)
BRASS_COL = (0.78, 0.62, 0.28)
RED_COL = (0.86, 0.14, 0.12)
WHITE_COL = (0.97, 0.97, 0.97)
DARK_COL = (0.14, 0.14, 0.16)

# -- manikin head & backrest rig -------------------------------------------------------------------------------------
HEAD_A = np.array([0.078, 0.093, 0.105])  # semi-axes (m): 15.6 x 18.6 x 21.0 cm adult female head
HEAD_C = np.array([0.495, 0.035, TOP + 0.185])
HEAD_MASS = 4.0
NECK_LIM_DEG = 45.0
NECK_K_DEG = 0.15    # N m / deg centring spring
NECK_D_DEG = 0.020   # N m s / deg damping
HEAD_DEG_MAX = 6.0   # deg: knocking/turning head past this scores 0
HEAD_DISP_MAX = 0.015  # m: shifting head centre past this scores 0

# -- wide-tooth comb & instrument tray -------------------------------------------------------------------------------
COMB_SIZE = (0.024, 0.196, 0.010)  # bounding box on tray: 2.4 cm handle width (x), 19.6 cm length (y), 1.0 cm thick (z)
COMB_MASS = 0.032
TRAY_C = (0.24, -0.24)
TRAY_IN = (0.124, 0.224)  # interior footprint (m)
TRAY_BASE, TRAY_RIM_H, TRAY_RIM_T = 0.006, 0.014, 0.008
COMB0 = (TRAY_C[0], TRAY_C[1] + 0.005, TOP + TRAY_BASE + COMB_SIZE[2] / 2 + 0.001)

# -- 5 N peak-hold root spring scale (Ohaus 8003-MN, 20 mm travel per N, red limit band at 2.0 N) --------------------
SCALE_X = 0.432
SCALE_Z = TOP + 0.122
SCALE_ZERO_Y = -0.075
SCALE_MM_PER_N = 0.020  # 0..5 N spans y = -0.075 .. -0.175; 2.0 N mark sits at y = -0.115

# -- scoring & grasp-weld constants ----------------------------------------------------------------------------------
PAD_CONTACT_MIN_N = 0.02  # N: minimum measured PhysX normal contact force on EACH finger pad for two-pad grasp weld
GRASP_CLOSE = 0.016   # commanded jaw opening (m) at or below which comb handle grasp-weld activates
GRASP_MEAS_MIN = 0.008  # minimum measured jaw opening (m) confirming pads have not closed empty
GRASP_MEAS_MAX = 0.027  # measured jaw opening (m) confirming pads physically closed onto the 2.2 cm handle
GRASP_OPEN = 0.030    # commanded jaw opening (m) above which comb handle grasp-weld releases
LOCK_NEAT_Y = 0.050   # m: untangled lock must lie within +-5.0 cm of towel centreline at end
CLEAR_DIST = 0.100    # m: both grippers at least 10 cm clear of head, hair, comb, and tray
STILL_SPEED = 0.020   # m/s: jaw speed threshold for parked end


class HeadSpec(Sphere):
    """Care manikin head on compliant neck joint."""


class CombSpec(Box):
    """Goody wide-tooth detangling comb with 10 rounded 3.4 cm teeth."""


def _pyf(o):
    return dataclasses.replace(o, pos=tuple(float(v) for v in o.pos))


def _rig_boxes() -> list[Box]:
    cx, cy = TRAY_C
    ix, iy = TRAY_IN
    rt, rh = TRAY_RIM_T, TRAY_RIM_H
    rz = TOP + TRAY_BASE + rh / 2
    return [
        # Foam backrest wedge & reclined upper torso (front at x = 0.410 so hair lock drapes visibly on top)
        Box("backrest_wedge", pos=(0.535, 0.030, TOP + 0.065), size=(0.190, 0.360, 0.130), color=WEDGE_COL),
        Box("manikin_torso", pos=(0.465, 0.030, TOP + 0.054), size=(0.110, 0.340, 0.108), color=GOWN_COL),
        Box("right_shoulder", pos=(0.396, 0.000, TOP + 0.038), size=(0.028, 0.160, 0.076), color=TOWEL_COL),
        # Chest bolster & folded white cotton terry towel (x = 0.045..0.382, top at TOP + 0.055)
        Box("chest_bolster", pos=(0.2135, 0.000, TOP + 0.024), size=(0.337, 0.156, 0.048), color=GOWN_COL),
        Box("chest_towel", pos=(0.2135, 0.000, TOP + 0.0515), size=(0.337, 0.160, 0.007), color=TOWEL_COL),
        # Stitched index marks along right (-y) towel border: green 3 cm scalp mark, blue 21 cm midpoint mark
        Box("towel_mark_scalp", pos=(0.394, -0.070, TOP + 0.0768), size=(0.008, 0.018, 0.0016), color=MARK_SCALP_COL),
        Box("towel_mark_mid", pos=(0.246, -0.070, TOP + 0.0558), size=(0.008, 0.018, 0.0016), color=MARK_MID_COL),
        # Spring-scale mounting post behind right shoulder
        Box("scale_mount", pos=(SCALE_X, -0.135, TOP + 0.060), size=(0.024, 0.140, 0.120), color=SCALE_BODY_COL),
        # Instrument tray at (0.24, -0.24)
        Box("tray_base", pos=(cx, cy, TOP + TRAY_BASE / 2), size=(ix + 2 * rt, iy + 2 * rt, TRAY_BASE), color=TRAY_COL),
        Box("tray_rim_n", pos=(cx - ix / 2 - rt / 2, cy, rz), size=(rt, iy + 2 * rt, rh), color=TRAY_COL),
        Box("tray_rim_f", pos=(cx + ix / 2 + rt / 2, cy, rz), size=(rt, iy + 2 * rt, rh), color=TRAY_COL),
        Box("tray_rim_r", pos=(cx, cy - iy / 2 - rt / 2, rz), size=(ix, rt, rh), color=TRAY_COL),
        Box("tray_rim_l", pos=(cx, cy + iy / 2 + rt / 2, rz), size=(ix, rt, rh), color=TRAY_COL),
    ]


HEAD = _pyf(HeadSpec("manikin_head", pos=tuple(HEAD_C), radius=0.02, mass=HEAD_MASS, color=SKIN_COL))
COMB = _pyf(CombSpec("detangling_comb", pos=COMB0, size=COMB_SIZE, mass=COMB_MASS, color=COMB_COL))
OBJECTS = [_pyf(o) for o in _rig_boxes()] + [HEAD, COMB]

TWIN_EXTRA = [
    {"name": "neck_joint", "kind": "joint", "real": "Diane D150 head-to-bust swivel collar + centring torsion springs",
     "note": "revolute about vertical M10 neck pivot, +-45 deg limits, 0.15 N m/deg centring spring, collar witness stripe"},
    {"name": "hair_lock", "kind": "body",
     "real": "38 cm dry Kanekalon synthetic hair weft (6 unbanded free-fibre sections, 8 g, 6.5 N magnetic root clips)",
     "size_mm": [380, 25, 6], "colour": "chestnut brown", "pose": [0.23, 0.0, round(TOP + 0.06, 4)]},
    {"name": "root_spring_scale", "kind": "body",
     "real": "Ohaus 8003-MN 5 N pull-type spring scale with sliding red O-ring peak telltale and 2.0 N limit stripe",
     "size_mm": [22, 130, 22], "colour": "silver/red", "pose": [SCALE_X, -0.130, round(SCALE_Z, 4)]},
]

# -- Isaac Sim USD authoring (head, comb teeth, spring scale, grasp-weld slots) -------------------------------------
_P: dict = {}
_H: dict = {"welds": {}, "held": {}, "was_open": [True, True]}
_ENV: dict = {"env": None}
_BOXES_LOCAL = None
_GRID = np.array([[i, j, k] for i in (0.0, 0.5, 1.0) for j in (0.0, 0.5, 1.0) for k in (0.0, 0.5, 1.0)])


def _head_uv_mesh(n_phi: int = 12, n_lam: int = 18):
    pts = []
    for i in range(n_phi + 1):
        ph = math.pi * i / n_phi
        for j in range(n_lam if 0 < i < n_phi else 1):
            la = 2.0 * math.pi * j / n_lam
            # Local frame: -x toward robot (face tilted forward-up), y across ears, z crown
            d = np.array([math.sin(ph) * math.cos(la), math.sin(ph) * math.sin(la), math.cos(ph)])
            pts.append(HEAD_A * d)
    faces = []
    ring = lambda i, j: 1 + (i - 1) * n_lam + (j % n_lam)  # noqa: E731
    last = 1 + (n_phi - 1) * n_lam
    for j in range(n_lam):
        faces.append((0, ring(1, j + 1), ring(1, j)))
        faces.append((last, ring(n_phi - 1, j), ring(n_phi - 1, j + 1)))
    for i in range(1, n_phi - 1):
        for j in range(n_lam):
            a, b, c, d = ring(i, j), ring(i, j + 1), ring(i + 1, j), ring(i + 1, j + 1)
            faces += [(a, b, d), (a, d, c)]
    return np.array(pts), faces


def _vis_prim(stage, path: str, kind: str, pos, size, color, rot_wxyz=None, collide: bool = False):
    from pxr import Gf, UsdGeom, UsdPhysics

    cls = UsdGeom.Sphere if kind == "sphere" else UsdGeom.Cylinder if kind == "cyl" else UsdGeom.Cube
    g = cls.Define(stage, path)
    if kind == "sphere":
        g.CreateRadiusAttr(1.0)
    elif kind == "cyl":
        g.CreateRadiusAttr(1.0)
        g.CreateHeightAttr(1.0)
        g.CreateAxisAttr("Z")
    else:
        g.CreateSizeAttr(1.0)
    g.CreateDisplayColorAttr([Gf.Vec3f(*map(float, color))])
    xf = UsdGeom.Xformable(g)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    if rot_wxyz is not None:
        xf.AddOrientOp().Set(Gf.Quatf(*map(float, rot_wxyz)))
    xf.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    if collide:
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    return g


def _author_head(stage, prim_path: str) -> None:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

    parent = prim_path.rsplit("/", 1)[0]
    w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    W = np.array([w[0], w[1], w[2]], dtype=float)
    for ch in stage.GetPrimAtPath(prim_path).GetChildren():
        if ch.GetName() == "geometry":
            UsdGeom.Imageable(ch).MakeInvisible()

    pts, faces = _head_uv_mesh()
    mesh = UsdGeom.Mesh.Define(stage, f"{prim_path}/skull")
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in pts]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for f in faces for i in f]))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*SKIN_COL)])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")

    # Recognizable facial features (looking forward-up toward -x, +z) & crown hair cap
    _vis_prim(stage, f"{prim_path}/hair_cap", "sphere", (0.012, 0.0, 0.015), (0.080, 0.095, 0.104), HAIR_CAP_COL)
    _vis_prim(stage, f"{prim_path}/nose", "cube", (-0.078, 0.0, 0.018), (0.022, 0.016, 0.032), SKIN_COL)
    _vis_prim(stage, f"{prim_path}/lips", "cube", (-0.072, 0.0, -0.022), (0.012, 0.036, 0.010), (0.76, 0.42, 0.42))
    for sgn, nm in ((-1.0, "right_eye"), (+1.0, "left_eye")):
        _vis_prim(stage, f"{prim_path}/{nm}", "cube", (-0.071, sgn * 0.028, 0.036), (0.008, 0.020, 0.005), DARK_COL)
    for sgn, nm in ((-1.0, "right_ear"), (+1.0, "left_ear")):
        _vis_prim(stage, f"{prim_path}/{nm}", "sphere", (-0.005, sgn * 0.092, -0.005), (0.012, 0.008, 0.024), SKIN_COL)
    # Head-side neck witness stripe
    _vis_prim(stage, f"{prim_path}/head_collar_mark", "cube", (-0.068, 0.0, -0.085), (0.010, 0.008, 0.022), WHITE_COL)

    # Filter internal contact between the neck-mounted head and the fixed bust/wedge supports
    fp_targets = []
    for sib in ("backrest_wedge", "manikin_torso", "right_shoulder"):
        sp = stage.GetPrimAtPath(f"{parent}/{sib}")
        if sp.IsValid():
            for sub in Usd.PrimRange(sp):
                fp_targets.append(sub.GetPath())
    if fp_targets:
        for hp in (stage.GetPrimAtPath(prim_path), mesh.GetPrim()):
            fp = UsdPhysics.FilteredPairsAPI.Apply(hp)
            fp.CreateFilteredPairsRel().SetTargets(fp_targets)

    # Compliant neck joint (world -> manikin_head)
    j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/neck_joint")
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, W + HEAD_C)))
    j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateBody1Rel().SetTargets([Sdf.Path(prim_path)])
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateAxisAttr("Z")
    j.CreateLowerLimitAttr(-NECK_LIM_DEG)
    j.CreateUpperLimitAttr(NECK_LIM_DEG)
    j.CreateExcludeFromArticulationAttr(True)
    drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drv.CreateTypeAttr("force")
    drv.CreateStiffnessAttr(NECK_K_DEG)
    drv.CreateDampingAttr(NECK_D_DEG)
    drv.CreateTargetPositionAttr(0.0)

    m = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(prim_path))
    m.CreateMassAttr(HEAD_MASS)
    m.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(0.018, 0.018, 0.016))


def _author_comb_and_scene(stage, comb_path: str) -> None:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    root = comb_path.rsplit("/", 1)[0]
    _P["root"] = root
    _P["comb"] = comb_path
    for ch in stage.GetPrimAtPath(comb_path).GetChildren():
        if ch.GetName() == "geometry":
            UsdGeom.Imageable(ch).MakeInvisible()
            for sub in Usd.PrimRange(ch):
                attr = sub.GetAttribute("physics:collisionEnabled")
                if attr and attr.IsValid():
                    attr.Set(False)

    # Author realistic Goody wide-tooth detangling comb: ergonomic handle, spine, grip band, and 10 rounded teeth
    _vis_prim(stage, f"{comb_path}/handle", "cube", (0.0, -0.053, 0.0), (0.022, 0.090, 0.009), COMB_COL, collide=True)
    _vis_prim(stage, f"{comb_path}/grip_band", "cube", (0.0, -0.074, 0.0), (0.0235, 0.036, 0.010), WHITE_COL)
    _vis_prim(stage, f"{comb_path}/spine", "cube", (0.010, 0.045, 0.0), (0.010, 0.106, 0.009), COMB_COL, collide=True)
    rot_tooth = (math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0)  # cylinder Z turned onto X
    t_len = hr.TOOTH_X_BASE - hr.TOOTH_X_TIP
    t_cx = 0.5 * (hr.TOOTH_X_BASE + hr.TOOTH_X_TIP)
    for k, ty in enumerate(hr.TOOTH_Y):
        _vis_prim(stage, f"{comb_path}/tooth_{k:02d}", "cyl", (t_cx, float(ty), 0.0),
                  (0.0019, 0.0019, t_len), COMB_COL, rot_wxyz=rot_tooth)
        _vis_prim(stage, f"{comb_path}/tip_{k:02d}", "sphere", (hr.TOOTH_X_TIP, float(ty), 0.0),
                  (0.0022, 0.0022, 0.0022), COMB_COL)

    # Author scalp eyelet, torso witness stripe, and Ohaus 8003-MN 5 N tubular peak-hold spring scale
    w = UsdGeom.Xformable(stage.GetPrimAtPath(root)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    W = np.array([w[0], w[1], w[2]], dtype=float)
    sroot = f"{root}/care_rig_vis"
    UsdGeom.Xform.Define(stage, sroot)
    _vis_prim(stage, f"{sroot}/torso_collar_mark", "cube",
              W + np.array([0.425, 0.035, TOP + 0.104]), (0.012, 0.008, 0.016), WHITE_COL)
    _vis_prim(stage, f"{sroot}/scalp_eyelet", "cube",
              W + np.array([hr.ROOT_X + 0.006, 0.0, TOP + hr.ROOT_DZ]), (0.012, 0.034, 0.012), DARK_COL)
    # Horizontal tubular spring scale barrel along -y: 0 N at y = -0.075, 2.0 N red band at y = -0.115, 5 N at -0.175
    rot_y_cyl = (math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0)
    _vis_prim(stage, f"{sroot}/scale_tube", "cyl",
              W + np.array([SCALE_X, -0.130, SCALE_Z]), (0.009, 0.009, 0.120), WHITE_COL, rot_wxyz=rot_y_cyl)
    _vis_prim(stage, f"{sroot}/scale_zero_tick", "cyl",
              W + np.array([SCALE_X, SCALE_ZERO_Y, SCALE_Z]), (0.0102, 0.0102, 0.0025), DARK_COL, rot_wxyz=rot_y_cyl)
    _vis_prim(stage, f"{sroot}/scale_2N_limit", "cyl",
              W + np.array([SCALE_X, SCALE_ZERO_Y - 2.0 * SCALE_MM_PER_N, SCALE_Z]),
              (0.0106, 0.0106, 0.0045), RED_COL, rot_wxyz=rot_y_cyl)
    _vis_prim(stage, f"{sroot}/scale_cord", "cyl",
              W + np.array([SCALE_X - 0.006, -0.038, SCALE_Z]), (0.0018, 0.0018, 0.075), BRASS_COL, rot_wxyz=rot_y_cyl)

    # Grasp-weld slots for the comb handle (disabled until a hand closes on the comb handle)
    UsdGeom.Xform.Define(stage, f"{root}/comb_joints")
    for side in ("Left", "Right"):
        g = f"{root}/{side}Arm/Geometry/base/link1/link2/link3/link4/link5/gripper"
        j = UsdPhysics.FixedJoint.Define(stage, f"{root}/comb_joints/grasp_{side.lower()}_comb")
        j.CreateBody0Rel().SetTargets([Sdf.Path(g)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(comb_path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateJointEnabledAttr(False)


def _register_module() -> None:
    if __name__ not in sys.modules:
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _spawn_head(prim_path, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_sphere
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_sphere(prim_path, cfg, translation, orientation, **kwargs)
    _author_head(get_current_stage(), str(prim_path))
    return prim


def _spawn_comb(prim_path, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_cuboid(prim_path, cfg, translation, orientation, **kwargs)
    _author_comb_and_scene(get_current_stage(), str(prim_path))
    return prim


def _custom_object(spec, scene_mod):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import MassPropertiesCfg, RigidBodyPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.station import linear_rgb

    _register_module()
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.4)
    if isinstance(spec, HeadSpec):
        @configclass
        class HeadCfg(sim_utils.SphereCfg):
            func: object = clone(_spawn_head)

        cfg = HeadCfg(radius=spec.radius, rigid_props=RigidBodyPropertiesCfg(),
                      mass_props=MassPropertiesCfg(mass=spec.mass), collision_props=None, visual_material=look)
    else:
        @configclass
        class CombCfg(sim_utils.CuboidCfg):
            func: object = clone(_spawn_comb)

        cfg = CombCfg(size=tuple(spec.size), rigid_props=RigidBodyPropertiesCfg(),
                      mass_props=MassPropertiesCfg(mass=spec.mass), collision_props=scene_mod._contact_props(),
                      physics_material=scene_mod._grippy(), visual_material=look)
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _patch_scene(scene) -> None:
    if getattr(scene, "_detangle_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, (HeadSpec, CombSpec)):
            return _custom_object(spec, scene)
        return orig(spec)

    scene.task_object = task_object
    scene._detangle_patched = True


def _patch_embodiment(emb_mod) -> None:
    cls = emb_mod.BimanualYamEmbodiment
    if getattr(cls.__init__, "_detangle_cs_patched", False):
        return
    orig_init = cls.__init__

    def __init__(self, *a, **kw):
        kw["contact_sensors"] = True
        orig_init(self, *a, **kw)

    __init__._detangle_cs_patched = True
    cls.__init__ = __init__


class _SceneFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname not in ("yam_arena.scene", "yam_arena.embodiment"):
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
            if fullname == "yam_arena.scene":
                _patch_scene(module)
            else:
                _patch_embodiment(module)

        spec.loader.exec_module = exec_module
        return spec


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
if "yam_arena.embodiment" in sys.modules:
    _patch_embodiment(sys.modules["yam_arena.embodiment"])
if not any(isinstance(f, _SceneFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _SceneFinder())


# -- runtime helpers & grasp-weld ------------------------------------------------------------------------------------
def _Rq(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _mat_to_wxyz(R: np.ndarray) -> tuple[float, float, float, float]:
    from yam_arena.ik import matrix_to_quat_xyzw

    qx, qy, qz, qw = matrix_to_quat_xyzw(R)
    return float(qw), float(qx), float(qy), float(qz)


def _pose_w(env, name: str) -> tuple[np.ndarray, np.ndarray]:
    """(pos_local_env, rot_3x3, T_world_4x4) for a rigid Arena object."""
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    p = base.arena_world.get_pose_w(name)[0].detach().cpu().numpy().astype(np.float64)
    Rm = _Rq(p[3:])
    Tw = np.eye(4)
    Tw[:3, :3] = Rm
    Tw[:3, 3] = p[:3]
    return p[:3] - origin, Rm, Tw


def _gripper_boxes_local():
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
        out[link] = [(np.min(np.concatenate(g), axis=0), np.max(np.concatenate(g), axis=0)) for g in groups.values()]
    return out


def _arm_state(env):
    global _BOXES_LOCAL
    from yam_arena.runtime import link_poses, tcp_world

    if _BOXES_LOCAL is None:
        _BOXES_LOCAL = _gripper_boxes_local()
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    tcp, rot, ap, cmd_ap, boxes, g_mats, allpts = [], [], [], [], [], [], []
    for i, side in enumerate(("left", "right")):
        rob = base.scene.articulations[f"{side}_robot"]
        lp = link_poses(rob)
        g_mats.append(lp["gripper"])
        t_env = tcp_world(rob) - origin
        tcp.append(t_env)
        rot.append(lp["gripper"][:3, :3])
        q = rob.data.joint_pos.torch[0].cpu().numpy()
        tgt = getattr(rob.data.joint_pos_target, "torch", rob.data.joint_pos_target)[0].cpu().numpy()
        names = list(rob.joint_names)
        j7, j8 = names.index("joint7"), names.index("joint8")
        ap.append(abs(float(q[j7])) + abs(float(q[j8])))
        cmd_ap.append(abs(float(tgt[j7])) + abs(float(tgt[j8])))
        pts = []
        for link, bl in _BOXES_LOCAL.items():
            Tw = lp[link].copy()
            Tw[:3, 3] -= origin
            for lo, hi in bl:
                boxes.append((Tw, lo, hi, i))
                pts.append((lo + _GRID * (hi - lo)) @ Tw[:3, :3].T + Tw[:3, 3])
        allpts.append(np.concatenate(pts))
    return np.array(tcp), np.array(rot), np.array(ap), np.array(cmd_ap), boxes, g_mats, allpts


def _pad_forces(env) -> list[tuple[float, float]]:
    """Read (tip_left_N, tip_right_N) normal contact forces from yam_arena's ContactSensor per arm."""
    out = [(0.0, 0.0), (0.0, 0.0)]
    if env is None or not hasattr(env, "unwrapped"):
        return out
    try:
        from yam_arena.runtime import contact_forces

        sensors = env.unwrapped.scene.sensors
        for i, side in enumerate(("left", "right")):
            key = f"{side}_contacts"
            if key in sensors:
                cf = contact_forces(sensors[key])
                out[i] = (float(cf.get("tip_left", 0.0)), float(cf.get("tip_right", 0.0)))
    except Exception:  # noqa: BLE001
        pass
    return out


def _drop_comb_weld(arm: int) -> None:
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        st = get_current_stage()
    except Exception:  # noqa: BLE001
        st = None
    path = _H["welds"].pop(arm, None)
    _H["held"].pop(arm, None)
    if st is not None and path:
        p = st.GetPrimAtPath(path)
        if p.IsValid():
            p.GetAttribute("physics:jointEnabled").Set(False)


def _update_comb_weld(env, tcp, rot, ap, cmd_ap, g_mats, pad_f) -> int | None:
    """Two-pad-contact grasp stand-in for elastomer pad friction on the 2.2 cm comb handle: engages ONLY when BOTH
    tip_left and tip_right have >= 0.02 N measured PhysX normal contact force while jaws close across opposite sides."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf

    if _P.get("root") is None:
        return None
    st = get_current_stage()
    c_pos, c_rot, c_Tw = _pose_w(env, COMB.name)
    was_open = _H.setdefault("was_open", [True, True])
    for arm, side in enumerate(("left", "right")):
        if ap[arm] >= 0.028 or cmd_ap[arm] >= GRASP_OPEN:
            was_open[arm] = True
        if arm in _H["held"]:
            if cmd_ap[arm] > GRASP_OPEN:
                _drop_comb_weld(arm)
            continue
        if COMB.name in _H["held"].values() or cmd_ap[arm] > GRASP_CLOSE or not was_open[arm]:
            continue
        fl, fr = pad_f[arm]
        if fl < PAD_CONTACT_MIN_N or fr < PAD_CONTACT_MIN_N or ap[arm] < GRASP_MEAS_MIN or ap[arm] > GRASP_MEAS_MAX:
            if ap[arm] < 0.005:
                was_open[arm] = False
            continue
        # Require the two contacting pads (along gripper jaw axis rot[arm][:, 0]) to straddle opposite sides of handle
        jaw_ax = rot[arm][:, 0]
        pa_loc = ((tcp[arm] - 0.5 * ap[arm] * jaw_ax) - c_pos) @ c_rot
        pb_loc = ((tcp[arm] + 0.5 * ap[arm] * jaw_ax) - c_pos) @ c_rot
        loc = (tcp[arm] - c_pos) @ c_rot
        straddles_x = float(pa_loc[0]) * float(pb_loc[0]) < 0.0
        if straddles_x and abs(float(loc[0])) <= 0.015 and -0.095 <= float(loc[1]) <= -0.020 and abs(float(loc[2])) <= 0.016:
            path = f"{_P['root']}/comb_joints/grasp_{side}_comb"
            prim = st.GetPrimAtPath(path)
            rel = np.linalg.inv(g_mats[arm]) @ c_Tw
            prim.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, rel[:3, 3])))
            qw, qx, qy, qz = _mat_to_wxyz(rel[:3, :3])
            prim.GetAttribute("physics:localRot0").Set(Gf.Quatf(qw, qx, qy, qz))
            prim.GetAttribute("physics:jointEnabled").Set(True)
            _H["welds"][arm] = path
            _H["held"][arm] = COMB.name
            was_open[arm] = False
            print(
                f"[detangle] {side} grasped comb handle on two-pad contact "
                f"(F_L={fl:.2f}N, F_R={fr:.2f}N, ap={ap[arm] * 1000:.1f}mm)",
                flush=True,
            )
    for arm in _H["held"]:
        return arm
    return None


def _in_tray(c_pos: np.ndarray) -> bool:
    cx, cy = TRAY_C
    ix, iy = TRAY_IN
    return bool(
        abs(c_pos[0] - cx) <= ix / 2
        and abs(c_pos[1] - cy) <= iy / 2
        and TOP - 0.002 <= c_pos[2] <= TOP + 0.035
    )


def _comb_flat_in_tray(c_pos: np.ndarray, c_rot: np.ndarray) -> bool:
    return bool(
        _in_tray(c_pos)
        and abs(float(c_rot[2, 2])) >= 0.90
        and float(c_pos[2]) <= TOP + TRAY_BASE + 0.016
    )


# -- episode tracker & USD hair/scale rendering ----------------------------------------------------------------------
class Tracker:
    def __init__(self):
        self.hair = hr.HairLock(TOP)
        self.mesh = None
        self.snarls = []
        self.peak_ring = None
        self.reset()

    def reset(self) -> None:
        for a in list(_H["welds"]):
            _drop_comb_weld(a)
        _H["welds"].clear()
        _H["held"].clear()
        _H["was_open"] = [True, True]
        self.hair.reset()
        self.steps = 0
        self.why = ""
        self._said: set[str] = set()
        self.comb_lifted = False
        self.comb_dropped = False
        self.head_knocked = False
        self.max_head_deg = 0.0
        self.max_head_disp = 0.0
        self.head_pos = HEAD_C.copy()
        self.comb_pos = np.array(COMB0, float)
        self.comb_rot = np.eye(3)
        self.comb_holder: int | None = None
        self.aperture = np.array([0.08, 0.08])
        self.allpts = [np.zeros((1, 3)) + [0.0, 0.0, 5.0]] * 2
        self.prev_pts = None
        self.speed = 0.0
        self.still_t = 0.0

    def fail(self, reason: str) -> None:
        if not self.why:
            self.why = reason
        if reason not in self._said:
            self._said.add(reason)
            print(f"[detangle] step {self.steps}: DISQUALIFIED ({reason})", flush=True)

    def step(self, env) -> None:
        tcp, rot, ap, cmd_ap, boxes, g_mats, allpts = _arm_state(env)
        pad_f = _pad_forces(env)
        comb_holder = _update_comb_weld(env, tcp, rot, ap, cmd_ap, g_mats, pad_f)
        c_pos, c_rot, _ = _pose_w(env, COMB.name)
        h_pos, h_rot, _ = _pose_w(env, HEAD.name)

        self.head_pos = h_pos
        self.comb_pos = c_pos
        self.comb_rot = c_rot
        self.comb_holder = comb_holder
        self.aperture = ap
        self.allpts = allpts

        # Filter gripper boxes near the hair lock
        lo = self.hair.x.min(axis=(0, 1)) - 0.12
        hi = self.hair.x.max(axis=(0, 1)) + 0.12
        near_boxes = [b for b in boxes if np.all((b[0][:3, 3] > lo) & (b[0][:3, 3] < hi))]

        self.hair.step(tcp, rot, ap, c_pos, c_rot, comb_holder, boxes=near_boxes)
        self.steps += 1

        # Head disturbance check
        head_disp = float(np.linalg.norm(h_pos - HEAD_C))
        cos_ang = max(-1.0, min(1.0, 0.5 * (float(np.trace(h_rot)) - 1.0)))
        head_deg = math.degrees(math.acos(cos_ang))
        self.max_head_disp = max(self.max_head_disp, head_disp)
        self.max_head_deg = max(self.max_head_deg, head_deg)
        if head_deg > HEAD_DEG_MAX or head_disp > HEAD_DISP_MAX:
            self.head_knocked = True
            self.fail(f"head knocked ({head_deg:.1f} deg, {head_disp * 100:.1f} cm)")

        # Comb drop check (latched over the whole trial whenever released outside the tray interior footprint)
        if comb_holder is not None or c_pos[2] > TOP + 0.035:
            self.comb_lifted = True
        if self.comb_lifted and comb_holder is None and not _in_tray(c_pos):
            self.comb_dropped = True
            self.fail("comb dropped outside its tray")

        # Hair physics disqualifiers
        if self.hair.root_overpull:
            self.fail(f"scalp root pull exceeded 2.0 N ({self.hair.max_root_pull_N:.2f} N)")
        if self.hair.strand_broken:
            self.fail("6.5 N magnetic root clip detached by raking into a jammed multi-stage mat")
        if self.hair.unheld_tangle_comb:
            self.fail("combed into an active tangle without holding the lock >=2 cm nearer the scalp")

        cat_pts = np.concatenate(allpts)
        if self.prev_pts is not None:
            self.speed = float(np.max(np.linalg.norm(cat_pts - self.prev_pts, axis=1))) * 20.0
        self.prev_pts = cat_pts
        self.still_t = (self.still_t + 0.05) if self.speed < STILL_SPEED else 0.0

        self.render(env)
        if os.environ.get("DETANGLE_LOG") and self.steps % 25 == 0:
            print(
                f"[detangle] step {self.steps:4d} u={self.hair.u.round(2)} "
                f"root_max={self.hair.max_root_pull_N:.2f}N pins={list(self.hair.pins.keys())} "
                f"comb_held={comb_holder} final_stroke={self.hair.final_clean_stroke} why='{self.why}'",
                flush=True,
            )

    def render(self, env) -> None:
        import omni.usd
        from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

        stage = omni.usd.get_context().get_stage()
        origin = env.unwrapped.scene.env_origins[0].detach().cpu().numpy().astype(np.float32)

        if self.mesh is None or not self.mesh.GetPrim().IsValid():
            m = UsdGeom.Mesh.Define(stage, "/World/HairLockMesh")
            m.CreateDoubleSidedAttr(True)
            m.CreateSubdivisionSchemeAttr("none")
            m.CreateFaceVertexCountsAttr([3] * len(self.hair.faces))
            m.CreateFaceVertexIndicesAttr(self.hair.faces.ravel().tolist())
            pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar(
                "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.vertex
            )
            pv.Set(Vt.Vec3fArray.FromNumpy(self.hair.vertex_colors))
            mat = UsdShade.Material.Define(stage, "/World/HairLockMat")
            sh = UsdShade.Shader.Define(stage, "/World/HairLockMat/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            vr = UsdShade.Shader.Define(stage, "/World/HairLockMat/PrimvarColor")
            vr.CreateIdAttr("UsdPrimvarReader_float3")
            vr.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
            vr.CreateOutput("result", Sdf.ValueTypeNames.Float3)
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
                vr.ConnectableAPI(), "result"
            )
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.55)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
            self.mesh = m

            # 3D snarl cluster markers at C1, C2, C3 (shrink to invisible as each crossing is combed out)
            self.snarls = []
            for k in range(len(hr.CROSSINGS)):
                sp = UsdGeom.Sphere.Define(stage, f"/World/HairSnarl_{k}")
                sp.CreateRadiusAttr(1.0)
                sp.CreateDisplayColorAttr([Gf.Vec3f(*SNARL_COL)])
                xf = UsdGeom.Xformable(sp)
                t_op = xf.AddTranslateOp()
                s_op = xf.AddScaleOp()
                self.snarls.append((sp, t_op, s_op))

            # Live sliding red peak-hold ring on the 5 N spring scale
            ring = UsdGeom.Cylinder.Define(stage, "/World/ScalePeakRing")
            ring.CreateRadiusAttr(0.0112)
            ring.CreateHeightAttr(0.0045)
            ring.CreateAxisAttr("Y")
            ring.CreateDisplayColorAttr([Gf.Vec3f(*RED_COL)])
            self.peak_ring = UsdGeom.Xformable(ring).AddTranslateOp()

        verts = self.hair.tube_vertices() + origin[None, :]
        self.mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(verts))

        for k, (sp, t_op, s_op) in enumerate(self.snarls):
            wk = float(1.0 - self.hair.u[k])
            if wk <= 0.02:
                UsdGeom.Imageable(sp).MakeInvisible()
            else:
                UsdGeom.Imageable(sp).MakeVisible()
                cp = self.hair.crossing_world_pos(k) + origin
                t_op.Set(Gf.Vec3d(float(cp[0]), float(cp[1]), float(cp[2] + 0.002 * wk)))
                s_op.Set(Gf.Vec3f(0.014 * wk, 0.018 * wk, 0.008 * wk))

        if self.peak_ring is not None:
            f_clamp = min(5.0, float(self.hair.max_root_pull_N))
            y_ring = SCALE_ZERO_Y - SCALE_MM_PER_N * f_clamp
            self.peak_ring.Set(Gf.Vec3d(float(origin[0] + SCALE_X), float(origin[1] + y_ring), float(origin[2] + SCALE_Z)))


TRACKER = Tracker()


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_detangle_hooked", False):
        orig_reset = cls.reset

        def reset(self, *a, **k):
            obs = orig_reset(self, *a, **k)
            if getattr(getattr(self, "task", None), "module", None) is None or \
                    getattr(self.task.module, "INSTRUCTION", None) != INSTRUCTION:
                return obs
            _ENV["env"] = self._env
            TRACKER.reset()
            TRACKER.render(self._env)
            for _ in range(4):
                self._base.sim.render()
            for _ in range(2):
                self._env.step(self._ready)
                self._base.sim.render()
            return self._observation()

        reset._detangle_hooked = True
        cls.reset = reset

    if not getattr(cls._task_step, "_detangle_hooked", False):
        orig_step = cls._task_step

        def _task_step(self, *a, **k):
            if getattr(self.task.module, "INSTRUCTION", None) == INSTRUCTION:
                _ENV["env"] = self._env
                TRACKER.step(self._env)
            return orig_step(self, *a, **k)

        _task_step._detangle_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- scoring ---------------------------------------------------------------------------------------------------------
def parts() -> dict:
    t = TRACKER
    h = t.hair
    if h is None or h.bad or not np.isfinite(h.x).all():
        return {"valid": False}

    c3_clear = bool(h.u[2] >= 1.0 - 1e-6)
    c2_clear = bool(h.u[1] >= 1.0 - 1e-6)
    c1_clear = bool(h.u[0] >= 1.0 - 1e-6)
    all_clear = c1_clear and c2_clear and c3_clear

    lock_neat = bool(
        np.max(np.abs(h.x[:, :, 1])) <= LOCK_NEAT_Y
        and float(h.x[:, -1, 0].mean()) <= 0.120
        and float(h.x[:, :, 2].min()) >= TOP + 0.040
    )
    comb_in_tray = bool(t.comb_holder is None and _comb_flat_in_tray(t.comb_pos, t.comb_rot))
    released = bool(t.comb_holder is None and not h.pins and np.all(t.aperture > hr.PINCH_OPEN))

    pts = np.concatenate(t.allpts)
    hair_pts = h.x.reshape(-1, 3)
    d_hair = float(np.min(np.linalg.norm(pts[:, None, :] - hair_pts[None, :, :], axis=2)))
    d_head = float(np.min((np.linalg.norm((pts - t.head_pos) / HEAD_A, axis=1) - 1.0) * float(np.min(HEAD_A))))
    comb_loc = np.abs((pts - t.comb_pos) @ t.comb_rot) - np.array([0.020, 0.098, 0.006])
    d_comb = float(np.min(np.linalg.norm(np.maximum(comb_loc, 0.0), axis=1)))
    cx, cy = TRAY_C
    ix, iy = TRAY_IN
    tray_lo = np.array([cx - ix / 2 - TRAY_RIM_T, cy - iy / 2 - TRAY_RIM_T, TOP])
    tray_hi = np.array([cx + ix / 2 + TRAY_RIM_T, cy + iy / 2 + TRAY_RIM_T, TOP + TRAY_BASE + TRAY_RIM_H])
    tray_gap = np.maximum(np.maximum(tray_lo - pts, pts - tray_hi), 0.0)
    d_tray = float(np.min(np.linalg.norm(tray_gap, axis=1)))
    clear = min(d_hair, d_head, d_comb, d_tray)

    disqualified = bool(
        t.why
        or h.root_overpull
        or h.strand_broken
        or h.unheld_tangle_comb
        or t.comb_dropped
        or t.head_knocked
    )
    return {
        "valid": True,
        "disqualified": disqualified,
        "why": t.why,
        "u": h.u.round(3).tolist(),
        "max_root_N": round(float(h.max_root_pull_N), 3),
        "supported_comb": bool(h.supported_tangle_comb),
        "c3_clear": bool(c3_clear and h.distal_clear_stroke),
        "half_clear": bool(c3_clear and c2_clear and h.mid_clear_stroke),
        "all_clear": all_clear,
        "final_stroke": bool(h.final_clean_stroke),
        "lock_neat": lock_neat,
        "comb_in_tray": comb_in_tray,
        "released": released,
        "clear": round(clear, 3),
        "still": round(t.still_t, 3),
    }


def score(objects=None) -> float:
    p = parts()
    if not p.get("valid") or p["disqualified"]:
        return 0.0
    if p["all_clear"] and p["final_stroke"]:
        if p["lock_neat"] and p["comb_in_tray"] and p["released"] and p["clear"] >= CLEAR_DIST and p["still"] >= SUCCESS_HOLD_S:
            return 1.0
        return 0.75
    if p["half_clear"]:
        return 0.50
    if p["supported_comb"] and p["c3_clear"]:
        return 0.25
    return 0.0


def success(objects=None) -> bool:
    return score(objects) >= 1.0


def progress(objects=None) -> dict:
    p = parts()
    if not p.get("valid"):
        return {"valid": 0}
    return {
        "c3_clear": int(p["c3_clear"]),
        "half_clear": int(p["half_clear"]),
        "all_clear": int(p["all_clear"]),
        "final_stroke": int(p["final_stroke"]),
        "comb_in_tray": int(p["comb_in_tray"]),
        "clear": p["clear"],
        "max_root_N": p["max_root_N"],
    }


VARIATION = {
    "obj:detangling_comb": {"dx": 0.008, "dy": 0.012, "yaw": 0.08},
}
