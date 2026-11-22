"""Shuck an ear of sweet corn: peel the husk back in two halves, then snap it off with the stalk end.

Scene (metres; origin on the table top under the arm-base midpoint; x forward, y left; heights above the table). An
OXO Good Grips Acacia Wood Utility Cutting Board (27.0 x 37.6 x 2.8 cm, SKU 11443000) is fixed at x 0.270, y 0.110 with
its 37.6 cm length along y and two white alignment marks on top (shank butt line at y 0.000, cob tip line at y 0.230).
On it lies a USDA U.S. Fancy fresh ear of yellow sweet corn in its husk along x 0.270 (28.5 cm overall, 0.340 kg):
trimmed stalk stub (shank, 4.5 cm long, 2.4 cm diameter, y 0.000..0.045) toward the right arm (-y), kernelled cob
(18.5 cm long, 4.4 cm diameter, y 0.045..0.230) between the marks, and two parted husk flag tips with solid silk tufts
(y 0.230..0.285) toward the left arm (+y). A Winco MXB-500Q 5-quart stainless steel mixing bowl (29.2 cm outer rim
diameter, 27.0 cm inner opening, 8.9 cm tall) stands in front of the right arm at x 0.290, y -0.265.

How it is modelled: the cob, the shank (with an internal 2-axis bend coupler) and six husk segments (two halves of three
curved shell segments each, carrying inward tip shoulders, gathered flag-leaf tabs and solid silk-tuft strands) are
rigid bodies. All husk/flag grasps are pure PhysX contact and pad friction (no grip joints); on the cylindrical shank
and exposed cob, the shared LimitedGrip model engages on two-pad contact and squeezing, with a compliant joint capped
at 50 N pull, 0.50 N m twist and 1.50 N m tip on the shank; the gentler cob grip is capped at 25 N, 0.25 N m and 0.75 N m. It slips above a cap and releases when the jaws open or contact is lost. Each husk half hinges from the shank base with wrap retention before opening, inelastic peel
set while peeling, and crease fold-back past 90 deg. The shank joins the cob through a 2-axis bend spring (5.5 N m/rad,
~3.07 N m at 32 deg) that snaps free past 32 deg of bend. Clamping the exposed bare cob above 35 N latches kernel
crushing (score 0).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import grip_limited as gl  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0
RUN_LIMIT_STEPS = 1200

INSTRUCTION = "Shuck the ear of corn and leave the bare cob on the board and the husk in the bowl."

# -- colours (sRGB) ------------------------------------------------------------------------------------------------------
WOOD = (0.48, 0.32, 0.18)
TPE_DARK = (0.14, 0.14, 0.15)
MARK_WHITE = (0.95, 0.95, 0.93)
STEEL = (0.74, 0.76, 0.78)
KERNEL = (0.98, 0.82, 0.22)
KERNEL_ROW = (0.93, 0.72, 0.14)
KERNEL_PALE = (1.00, 0.90, 0.42)
KERNEL_BRUISE = (0.58, 0.44, 0.14)
HUSK = (0.32, 0.58, 0.20)
HUSK_IN = (0.76, 0.83, 0.52)
SILK = (0.84, 0.70, 0.40)
SHANK_C = (0.52, 0.64, 0.30)

# -- geometry (m, kg) ----------------------------------------------------------------------------------------------------
BOARD_C = (0.270, 0.110)
BOARD_SIZE = (0.270, 0.376, 0.028)  # OXO Good Grips Acacia Wood Utility Cutting Board (SKU 11443000)
BOARD_TOP = BOARD_SIZE[2]

AX = 0.270  # ear axis x
Y_BUTT = 0.045  # cob base / husk attachment station y
COB_R, COB_L = 0.022, 0.185  # 4.4 cm diameter, 18.5 cm long (y 0.045..0.230)
SHANK_R, SHANK_L = 0.012, 0.045  # 2.4 cm diameter, 4.5 cm long (y 0.000..0.045)
RH, TH = 0.0265, 0.004  # husk shell mid-radius (2.5 mm radial clearance over cob) and wall thickness
SEG_L = (0.062, 0.062, 0.061)  # three shell segments per half along the 18.5 cm cob
FLAG_L, FLAG_X, FLAG_T, FLAG_H = 0.051, 0.019, 0.010, 0.032  # gathered flag-leaf tab past cob tip (y 0.234..0.285)
CUP_X, CUP_L, CUP_W, CUP_H = 0.015, 0.006, 0.016, 0.026  # inward tip shoulder cupping in front of cob end
SILK_X, SILK_R, SILK_L = 0.012, 0.0016, 0.046  # solid silk tuft strands per half
Z0 = BOARD_TOP + 0.030  # ear axis height above table top (0.8 mm clearance under bottom husk plate at spawn)
PLATE_W = 2.0 * RH * math.tan(math.radians(16.0)) + 0.0015
PHIS = (0.0, 32.0, -32.0, 64.0, -64.0)

COB_M = 0.250
COUPLER_M = 0.004
SHANK_M = 0.026  # shank + coupler = 0.030 kg
SEG_M = (0.009, 0.009, 0.012)  # 0.030 kg per half, 0.060 kg both halves -> 0.340 kg total ear

# Winco MXB-500Q 5-Quart Stainless Steel Mixing Bowl (29.2 cm OD, 27.0 cm ID, 8.9 cm tall)
BOWL_C = (0.290, -0.265)
BOWL_R_OUT = 0.146
BOWL_R_IN = 0.135
BOWL_WALL_T = BOWL_R_OUT - BOWL_R_IN
BOWL_R_MID = 0.5 * (BOWL_R_OUT + BOWL_R_IN)
BOWL_H = 0.089

# Mechanics constants
K_BEND = 5.50  # N m/rad on 2-axis shank joint (~3.07 N m at 32 deg snap)
SNAP_DEG = 32.0  # deg of bend between cob and shank axes to snap shank off
BEND_DAMP = 0.16  # N m s/rad
K_WRAP = 0.45  # N m/rad initial husk-wrap retention before half is parted past WRAP_RELEASE_DEG
WRAP_RELEASE_DEG = 18.0
K_PEEL_HOLD = 0.28  # N m/rad inelastic fibrous set holding peeled angle during regrip (18..90 deg)
CREASE_AT_DEG = 90.0  # once peeled past 90 deg, husk node creases back behind the shank
K_CREASE = 0.48  # N m/rad crease spring holding peeled husk folded back
CREASE_TARGET_DEG = 140.0  # deg folded back clear of shank fingers (mid-hinges gather 32 deg inward after snap)
HINGE_DAMP = 0.09  # N m s/rad at base husk hinges
BASE_FRICTION = 0.06  # N m dry joint friction on base husk hinges
MID_K, MID_DAMP = 0.70, 0.06  # N m/rad, N m s/rad between shell segments

# Scoring thresholds
CRUSH_FORCE_N = 35.0  # clamping force on bare cob above which milk-stage kernels rupture (latched)
CRUSH_STEPS = 2  # consecutive control steps > CRUSH_FORCE_N to latch (ignores single-step contact spike)
PEEL_DEG = 100.0  # base hinge angle (deg) required (along with segment clearance) for a peeled half
PEEL_BASE_MARGIN = 0.020  # m: all segments of a peeled half must lie behind (cob_base + 2.0 cm) along cob axis
DETACH_GAP = 0.030  # m: once snapped, all husk/shank parts must be >= 3.0 cm clear of cob surface for bare cob
BOARD_EDGE_INSET = 0.010  # m: cob endpoints inside board footprint
CLEAR = 0.10  # m: every gripper surface point >= 10 cm from ear parts and scrap bowl
OPEN_APERTURE = 0.050  # m: jaws open >= 5.0 cm at finish
STILL_STEPS = 20  # 1.0 s at 20 Hz
STILL_SPEED = 0.02  # m/s (and <= 2.0 cm drift over 1.0 s window)


z = lambda h: TOP + h  # noqa: E731


def _seg_y(k: int) -> tuple[float, float]:
    y0 = Y_BUTT + sum(SEG_L[:k])
    return y0, y0 + SEG_L[k]


BOARD = Box("cut_board", pos=(BOARD_C[0], BOARD_C[1], z(BOARD_TOP / 2)), size=BOARD_SIZE, color=WOOD)
COB = Cylinder("corn_cob", pos=(AX, Y_BUTT + COB_L / 2, z(Z0)), radius=COB_R, height=COB_L, mass=COB_M, color=KERNEL)
COUPLER = Sphere("shank_coupler", pos=(AX, Y_BUTT, z(Z0)), radius=0.006, mass=COUPLER_M, color=SHANK_C)
SHANK = Cylinder(
    "corn_shank", pos=(AX, Y_BUTT - SHANK_L / 2, z(Z0)), radius=SHANK_R, height=SHANK_L, mass=SHANK_M, color=SHANK_C
)


class HuskSeg(Box):
    """Root side-plate of one husk shell segment; curved plates, tip shoulder, flag and silk are child prims."""


def _husk(side: str, k: int) -> HuskSeg:
    s = 1.0 if side == "f" else -1.0
    y0, y1 = _seg_y(k)
    th = 0.006 if k in (1, 2) else TH
    return HuskSeg(
        f"husk_{side}{k}",
        pos=(AX + s * RH, 0.5 * (y0 + y1), z(Z0)),
        size=(th, SEG_L[k] - 0.001, PLATE_W),
        mass=SEG_M[k],
        color=HUSK,
    )


HUSKS = [_husk(sd, k) for sd in ("f", "n") for k in range(3)]

# Bowl: 8 rotated floor plates forming a gapless 16-sided disc + 16 cylindrical-wall staves
_BOWL_FLOOR_W = 2.0 * BOWL_R_IN * math.cos(math.pi / 16) + 0.004
_BOWL_FLOOR_D = 2.0 * BOWL_R_IN * math.sin(math.pi / 16) + 0.006
_BOWL = [
    Box(
        f"bowl_base_{_k}",
        pos=(BOWL_C[0], BOWL_C[1], z(0.003)),
        size=(_BOWL_FLOOR_W, _BOWL_FLOOR_D, 0.006),
        yaw=_k * math.pi / 8,
        color=STEEL,
    )
    for _k in range(8)
]
for _i in range(16):
    _a = 2.0 * math.pi * _i / 16
    _BOWL.append(
        Box(
            f"bowl_wall{_i}",
            pos=(BOWL_C[0] + BOWL_R_MID * math.cos(_a), BOWL_C[1] + BOWL_R_MID * math.sin(_a), z(BOWL_H / 2)),
            size=(BOWL_WALL_T, 2.0 * BOWL_R_MID * math.tan(math.pi / 16) + 0.004, BOWL_H),
            yaw=_a,
            color=STEEL,
        )
    )

OBJECTS = [BOARD, *_BOWL, COB, COUPLER, SHANK, *HUSKS]
HUSK_NAMES = [h.name for h in HUSKS]
WASTE_NAMES = [SHANK.name, *HUSK_NAMES]
EAR = [COB.name, SHANK.name, *HUSK_NAMES]
_AXIS_Y = {COB.name, SHANK.name}

TWIN_EXTRA = [
    {
        "name": "board_marks",
        "kind": "decal",
        "size_mm": [40, 5],
        "colour": "white",
        "pose": [AX, 0.0, z(BOARD_TOP)],
        "note": "two white tape alignment lines on the cutting board at y 0.000 (shank butt) and y 0.230 (cob tip)",
    },
    {
        "name": "husk_hinges",
        "kind": "joint",
        "real": "fresh husk leaves attached round the shank node, parted into two halves at the tip",
        "note": "sim: each half = 3 curved rigid shell segments + tip cup + flag tab + solid silk strands, hinged at shank",
    },
    {
        "name": "shank_snap",
        "kind": "joint",
        "real": "trimmed sweet-corn stalk stub snapped from cob base",
        "note": "sim: 2-axis revolute bend spring 5.5 N m/rad (~3.07 N m at 32 deg), disabled past 32 deg",
    },
    {
        "name": "pad_grip",
        "kind": "joint",
        "real": "food-grade textured nitrile finger sleeves clamping the shank (<=50 N) and bare cob (<35 N)",
        "note": "sim: two-pad-contact compliant grip capped at 50 N pull, 0.50 N m twist and 1.50 N m tip; slips above cap",
    },
]

_P: dict = {}
GRIP = gl.LimitedGrip(lambda: _P.get(SHANK.name), label="shank", name="shank_grip")
COB_GRIP = gl.LimitedGrip(lambda: _P.get(COB.name), label="cob", name="cob_grip", squeeze_n=25.0)


# -- USD authoring -------------------------------------------------------------------------------------------------------
def _look(stage, root: str, name: str, rgb: tuple[float, float, float], rough: float = 0.65, metallic: float = 0.0):
    import isaaclab.sim as sim_utils
    from yam_arena.station import linear_rgb

    path = f"{root}/CornLooks/{name}"
    if not stage.GetPrimAtPath(path).IsValid():
        cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(rgb), roughness=rough, metallic=metallic)
        cfg.func(path, cfg)
    return path


def _phys_mats(stage, root: str) -> dict[str, str]:
    import isaaclab.sim as sim_utils

    out = {}
    specs = (
        ("pad", 1.65, 1.45, "max"),
        ("corn", 1.50, 1.30, "average"),
        ("board", 0.22, 0.18, "min"),
        ("bowl", 0.45, 0.38, "average"),
    )
    for key, sf, df, mode in specs:
        p = f"{root}/CornLooks/phys_{key}"
        if not stage.GetPrimAtPath(p).IsValid():
            cfg = sim_utils.RigidBodyMaterialCfg(
                static_friction=sf, dynamic_friction=df, friction_combine_mode=mode, restitution=0.02
            )
            cfg.func(p, cfg)
        out[key] = p
    return out


def _apply_collider(prim, phys_path: str | None, stage, patch: float = 0.024) -> None:
    from isaaclab.sim.utils import bind_physics_material
    from pxr import PhysxSchema, UsdPhysics

    UsdPhysics.CollisionAPI.Apply(prim)
    cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    cp.CreateContactOffsetAttr(0.002)
    cp.CreateRestOffsetAttr(0.0)
    if patch > 0.0:
        cp.CreateTorsionalPatchRadiusAttr(float(patch))
        cp.CreateMinTorsionalPatchRadiusAttr(float(patch * 0.5))
    if phys_path:
        bind_physics_material(str(prim.GetPath()), phys_path, stage=stage)


def _child(
    stage,
    root: str,
    parent: str,
    name: str,
    kind: str,
    pos,
    size=None,
    radius=None,
    height=None,
    axis: str = "Z",
    rot_y: float = 0.0,
    rgb=HUSK,
    collide: bool = True,
    phys_path: str | None = None,
    patch: float = 0.024,
) -> str:
    from isaaclab.sim.utils import bind_visual_material
    from pxr import Gf, UsdGeom

    path = f"{parent}/{name}"
    if kind == "box":
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
    else:
        g = UsdGeom.Cylinder.Define(stage, path)
        g.CreateRadiusAttr(float(radius))
        g.CreateHeightAttr(float(height))
        g.CreateAxisAttr(axis)
    x = UsdGeom.Xformable(g)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    if rot_y:
        h = math.radians(rot_y) / 2.0
        x.AddOrientOp().Set(Gf.Quatf(math.cos(h), 0.0, math.sin(h), 0.0))
    if kind == "box":
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    tag = "c_" + "_".join(str(int(round(v * 255))) for v in rgb)
    bind_visual_material(path, _look(stage, root, tag, rgb), stage=stage)
    if collide:
        _apply_collider(g.GetPrim(), phys_path, stage, patch=patch)
    return path


def _author_seg(stage, root: str, spec: HuskSeg, pmats: dict[str, str]) -> None:
    side, k = spec.name[-2], int(spec.name[-1])
    s = 1.0 if side == "f" else -1.0
    path = f"{root}/{spec.name}"
    c0 = np.array(spec.pos)
    L = SEG_L[k] - 0.001
    for phi in PHIS[1:]:
        p = np.array([AX + s * RH * math.cos(math.radians(phi)), c0[1], z(Z0) + RH * math.sin(math.radians(phi))])
        rot = -phi if s > 0 else phi
        _child(
            stage,
            root,
            path,
            f"plate{int(phi) + 100}",
            "box",
            p - c0,
            size=(TH, L, PLATE_W),
            rot_y=rot,
            rgb=HUSK,
            collide=True,
            phys_path=pmats["corn"],
            patch=0.022,
        )
    # Paler inner husk face (visual) so peeled husk halves look unmistakably like real corn husks
    th0 = 0.006 if k in (1, 2) else TH
    _child(
        stage,
        root,
        path,
        "inner",
        "box",
        (-s * (th0 / 2 + 0.0004), 0.0, 0.0),
        size=(0.0006, L * 0.95, PLATE_W * 1.4),
        rgb=HUSK_IN,
        collide=False,
    )
    if k == 2:
        y1 = _seg_y(2)[1]  # 0.230 (cob tip)
        # Inward tip shoulder 3 mm past cob end: physically prevents unpeeled husk from sliding backward off cob
        cup_c = np.array([AX + s * CUP_X, y1 + 0.003 + CUP_L / 2, z(Z0)])
        _child(
            stage,
            root,
            path,
            "cup",
            "box",
            cup_c - c0,
            size=(CUP_W, CUP_L, CUP_H),
            rgb=HUSK,
            collide=True,
            phys_path=pmats["corn"],
            patch=0.020,
        )
        # Gathered fibrous flag-leaf tab (8 mm thick, 5.1 cm long, 3.2 cm tall)
        flag_c = np.array([AX + s * FLAG_X, y1 + 0.004 + FLAG_L / 2, z(Z0)])
        _child(
            stage,
            root,
            path,
            "flag",
            "box",
            flag_c - c0,
            size=(FLAG_T, FLAG_L, FLAG_H),
            rgb=HUSK,
            collide=True,
            phys_path=pmats["corn"],
            patch=0.028,
        )
        # Solid silk tuft strands protruding between the flag leaves (with collision, carried away when shucked)
        for i, dz in enumerate((-0.008, -0.004, 0.0, 0.004, 0.008)):
            sp = np.array([AX + s * SILK_X, y1 + 0.005 + SILK_L / 2, z(Z0) + dz])
            _child(
                stage,
                root,
                path,
                f"silk{i}",
                "cyl",
                sp - c0,
                radius=SILK_R,
                height=SILK_L,
                axis="Y",
                rgb=SILK,
                collide=True,
                phys_path=pmats["corn"],
                patch=0.015,
            )


def _bind_prim_tree(stage, root_path: str, phys_path: str, patch: float = 0.0) -> None:
    from isaaclab.sim.utils import bind_physics_material
    from pxr import PhysxSchema, Usd, UsdPhysics

    top = stage.GetPrimAtPath(root_path)
    if not top or not top.IsValid():
        return
    for p in Usd.PrimRange(top):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            bind_physics_material(str(p.GetPath()), phys_path, stage=stage)
            if patch > 0.0:
                cp = PhysxSchema.PhysxCollisionAPI.Apply(p)
                cp.CreateTorsionalPatchRadiusAttr(float(patch))
                cp.CreateMinTorsionalPatchRadiusAttr(float(patch * 0.5))


def _author_ear(root: str) -> None:
    from isaaclab.sim.utils import bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

    stage = get_current_stage()
    pmats = _phys_mats(stage, root)
    all_dyn = [COB.name, COUPLER.name, SHANK.name, *HUSK_NAMES]
    _P.update(root=root, **{n: f"{root}/{n}" for n in all_dyn})

    # Bind physics materials to board, bowl, ear root shapes, and robot finger pads (food-safe nitrile grip sleeves)
    _bind_prim_tree(stage, f"{root}/{BOARD.name}", pmats["board"], patch=0.0)
    for b in _BOWL:
        _bind_prim_tree(stage, f"{root}/{b.name}", pmats["bowl"], patch=0.0)
        bind_visual_material(
            f"{root}/{b.name}", _look(stage, root, "steel_bowl", STEEL, rough=0.30, metallic=0.65), stage=stage
        )
    for n in all_dyn:
        _bind_prim_tree(stage, _P[n], pmats["corn"], patch=0.026)

    # Disable collision on the internal gimbal sphere
    for p in Usd.PrimRange(stage.GetPrimAtPath(_P[COUPLER.name])):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)

    # Robot finger pads: high-friction nitrile finger covers + PhysX contact reporting on both pads & ear cylinders
    _enable_contact_processing()
    for cname in (SHANK.name, COB.name):
        cp_prim = stage.GetPrimAtPath(_P[cname])
        if cp_prim and cp_prim.IsValid():
            PhysxSchema.PhysxContactReportAPI.Apply(cp_prim).CreateThresholdAttr(0.0)
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        ps = prim.GetPath().pathString
        if ps.startswith(root + "/") and ("/LeftArm/" in ps or "/RightArm/" in ps):
            if ps.endswith(("/tip_left", "/tip_right")) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
            if ("/tip_left" in ps or "/tip_right" in ps) and prim.HasAPI(UsdPhysics.CollisionAPI):
                _bind_prim_tree(stage, ps, pmats["pad"], patch=0.028)

    # Cutting board visual details: OXO non-slip TPE end grips + white alignment marks at y=0.000 and y=0.230
    bd = f"{root}/{BOARD.name}"
    for sgn, tag in ((-1.0, "tpe_neg"), (1.0, "tpe_pos")):
        _child(
            stage,
            root,
            bd,
            tag,
            "box",
            (0.0, sgn * (BOARD_SIZE[1] / 2 - 0.006), 0.0),
            size=(BOARD_SIZE[0] + 0.002, 0.012, BOARD_SIZE[2] + 0.001),
            rgb=TPE_DARK,
            collide=False,
        )
    for y_mark, tag in ((Y_BUTT - SHANK_L, "mark_butt"), (Y_BUTT + COB_L, "mark_tip")):
        _child(
            stage,
            root,
            bd,
            tag,
            "box",
            (AX - BOARD_C[0], y_mark - BOARD_C[1], BOARD_TOP / 2 + 0.0005),
            size=(0.044, 0.005, 0.001),
            rgb=MARK_WHITE,
            collide=False,
        )

    # Shank fibrous flat-faceted grip prism so parallel pads clamp cleanly without cylinder roll
    _child(
        stage,
        root,
        _P[SHANK.name],
        "stalk_facets",
        "box",
        (0.0, 0.0, 0.0),
        size=(2.0 * SHANK_R, SHANK_L - 0.004, 2.0 * SHANK_R * 0.92),
        rgb=SHANK_C,
        collide=True,
        phys_path=pmats["corn"],
        patch=0.028,
    )

    # Cob longitudinal kernel ridges (physical anti-roll texture) + transverse visual kernel rings + bruise prims
    cob = _P[COB.name]
    for i in range(16):
        a = 2.0 * math.pi * i / 16
        _child(
            stage,
            root,
            cob,
            f"row{i}",
            "box",
            ((COB_R - 0.0005) * math.cos(a), 0.0, (COB_R - 0.0005) * math.sin(a)),
            size=(0.0024, COB_L * 0.94, 0.0048),
            rot_y=-math.degrees(a),
            rgb=KERNEL_ROW if i % 2 == 0 else KERNEL_PALE,
            collide=(i % 2 == 0),
            phys_path=pmats["corn"],
            patch=0.025,
        )
    for j in range(9):
        yy = -0.42 * COB_L + j * (0.84 * COB_L / 8.0)
        _child(
            stage,
            root,
            cob,
            f"ring{j}",
            "cyl",
            (0.0, yy, 0.0),
            radius=COB_R + 0.0005,
            height=0.003,
            axis="Y",
            rgb=KERNEL_PALE,
            collide=False,
        )
    _P["bruise"] = _child(
        stage,
        root,
        cob,
        "bruise_patch",
        "box",
        (0.0, 0.0, 0.0),
        size=(0.004, 0.028, 0.004),
        rgb=KERNEL_BRUISE,
        collide=False,
    )

    # Solver iterations & mild damping on all dynamic ear bodies for crisp articulated stability
    for n in all_dyn:
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(_P[n]))
        rb.CreateSolverPositionIterationCountAttr(16)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateLinearDampingAttr(0.35)
        rb.CreateAngularDampingAttr(0.75)

    def wpos(path: str) -> np.ndarray:
        m = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)
        return np.array(m.ExtractTranslation(), dtype=np.float64)

    jroot = f"{root}/corn_joints"
    UsdGeom.Xform.Define(stage, jroot)

    def joint(name: str, b0: str, b1: str, anchor, axis: str, lo: float, hi: float, k: float = 0.0, c: float = 0.0,
              jf: float = 0.0):
        J = UsdPhysics.RevoluteJoint.Define(stage, f"{jroot}/{name}")
        J.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
        J.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        J.CreateAxisAttr(axis)
        a = np.asarray(anchor, float)
        J.CreateLocalPos0Attr(Gf.Vec3f(*map(float, a - wpos(b0))))
        J.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        J.CreateLocalPos1Attr(Gf.Vec3f(*map(float, a - wpos(b1))))
        J.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        J.CreateLowerLimitAttr(float(lo))
        J.CreateUpperLimitAttr(float(hi))
        J.CreateExcludeFromArticulationAttr(True)
        if jf > 0.0:
            PhysxSchema.PhysxJointAPI.Apply(J.GetPrim()).CreateJointFrictionAttr(float(jf))
        d = UsdPhysics.DriveAPI.Apply(J.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(float(k * math.pi / 180.0))  # USD angular drive stiffness is N m / deg
        d.CreateDampingAttr(float(c * math.pi / 180.0))
        d.CreateTargetPositionAttr(0.0)
        _P[name] = str(J.GetPath())
        return J

    sh = _P[SHANK.name]
    cp_p = _P[COUPLER.name]
    joint("shank_snap_x", cob, cp_p, (AX, Y_BUTT, z(Z0)), "X", -48.0, 48.0, k=K_BEND, c=BEND_DAMP)
    joint("shank_snap_z", cp_p, sh, (AX, Y_BUTT, z(Z0)), "Z", -48.0, 48.0, k=K_BEND, c=BEND_DAMP)

    for sd, s in (("f", 1.0), ("n", -1.0)):
        lo, hi = (-172.0, 2.0) if s > 0 else (-2.0, 172.0)
        mlo, mhi = (-38.0, 6.0) if s > 0 else (-6.0, 38.0)
        segs = [_P[f"husk_{sd}{k}"] for k in range(3)]
        joint(f"base_{sd}", sh, segs[0], (AX + s * RH, Y_BUTT, z(Z0)), "Z", lo, hi, k=K_WRAP, c=HINGE_DAMP,
              jf=BASE_FRICTION)
        for k in (1, 2):
            joint(f"mid{k}_{sd}", segs[k - 1], segs[k], (AX + s * RH, _seg_y(k)[0], z(Z0)), "Z", mlo, mhi, k=MID_K,
                  c=MID_DAMP, jf=0.02)

    # Collision filtering:
    # - Husk pieces fold freely past each other and past the shank; base/mid shell segments pass free of cob-lip
    #   interference (standing in for leaf flex), while tip segment (seg2, carrying inward tip shoulder) collides with
    #   the cob so an unpeeled husk cannot slide off the cob unpeeled.
    husk_paths = [_P[h] for h in HUSK_NAMES]
    for h in husk_paths:
        rel = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(h)).CreateFilteredPairsRel()
        for o in husk_paths:
            if o != h:
                rel.AddTarget(Sdf.Path(o))
        rel.AddTarget(Sdf.Path(sh))
        rel.AddTarget(Sdf.Path(cp_p))
        if not h.endswith("2"):
            rel.AddTarget(Sdf.Path(cob))
    cob_rel = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(cob)).CreateFilteredPairsRel()
    cob_rel.AddTarget(Sdf.Path(sh))
    cob_rel.AddTarget(Sdf.Path(cp_p))
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(sh)).CreateFilteredPairsRel().AddTarget(Sdf.Path(cp_p))


def _wrap_cfg(spec, obj):
    from isaaclab.sim.utils import clone

    cfg = obj.spawner_cfg
    if spec.name in _AXIS_Y:
        cfg.axis = "Y"
    if isinstance(spec, HuskSeg) or spec.name == OBJECTS[-1].name:
        orig = cfg.func
        base = getattr(orig, "__wrapped__", orig)
        is_seg, last = isinstance(spec, HuskSeg), spec.name == OBJECTS[-1].name

        def spawn(prim_path, c, *a, **k):
            from isaaclab.sim.utils.stage import get_current_stage

            prim = base(prim_path, c, *a, **k)
            root = str(prim_path).rsplit("/", 1)[0]
            st = get_current_stage()
            if is_seg:
                _author_seg(st, root, spec, _phys_mats(st, root))
            if last:
                _author_ear(root)
            return prim

        cfg.func = clone(spawn)
    obj.object_cfg = obj._init_object_cfg()
    return obj


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_corn_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        obj = orig(spec)
        if getattr(spec, "name", None) in _AXIS_Y or isinstance(spec, HuskSeg):
            _wrap_cfg(spec, obj)
        return obj

    task_object._corn_patched = True
    scene.task_object = task_object


class _SceneFinder(importlib.abc.MetaPathFinder):
    _corn = True

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
elif not any(getattr(f, "_corn", False) for f in sys.meta_path):
    sys.meta_path.insert(0, _SceneFinder())


def _enable_contact_processing() -> None:
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # noqa: BLE001
        pass
    try:
        from isaaclab.app.settings_manager import get_settings_manager

        get_settings_manager().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # noqa: BLE001
        pass


def _patch_embodiment_mod(mod) -> None:
    cls = mod.BimanualYamEmbodiment
    if getattr(cls.__init__, "_corn_contacts", False):
        return
    orig_init = cls.__init__

    def __init__(self, *a, **kw):
        kw["contact_sensors"] = True
        return orig_init(self, *a, **kw)

    __init__._corn_contacts = True
    cls.__init__ = __init__


class _EmbodimentFinder(importlib.abc.MetaPathFinder):
    _corn_emb = True

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.embodiment":
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
            _patch_embodiment_mod(module)

        spec.loader.exec_module = exec_module
        return spec


if "yam_arena.embodiment" in sys.modules:
    _patch_embodiment_mod(sys.modules["yam_arena.embodiment"])
elif not any(getattr(f, "_corn_emb", False) for f in sys.meta_path):
    sys.meta_path.insert(0, _EmbodimentFinder())


# -- episode state & kinematics ------------------------------------------------------------------------------------------
_FRESH = dict(
    env=None,
    step=0,
    snapped=False,
    dropped=False,
    crushed=False,
    crush_cnt=0,
    peak_cob_force=0.0,
    nan=False,
    peel={"f": 0.0, "n": 0.0},
    peel_max={"f": 0.0, "n": 0.0},
    bend=0.0,
    pinched={"left": None, "right": None},
    anchor={"left": None, "right": None},
    was_open={"left": 0, "right": 0},
    slips=0,
    peak_grip_N={"left": 0.0, "right": 0.0},
    peak_grip_Nm={"left": 0.0, "right": 0.0},
    ghist=None,
)
_M: dict = {}


def _set_joint_enabled(name: str, on: bool) -> None:
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        p = get_current_stage().GetPrimAtPath(_P[name])
        if p and p.IsValid():
            p.GetAttribute("physics:jointEnabled").Set(bool(on))
    except Exception:  # noqa: BLE001
        pass


def _show_bruise(on: bool) -> None:
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf

        p = get_current_stage().GetPrimAtPath(_P.get("bruise", ""))
        if p and p.IsValid():
            p.GetAttribute("xformOp:scale").Set(Gf.Vec3f(0.046, 0.032, 0.046) if on else Gf.Vec3f(0.004, 0.028, 0.004))
    except Exception:  # noqa: BLE001
        pass


def _reset_model() -> None:
    if "root" in _P:
        _set_joint_enabled("shank_snap_x", True)
        _set_joint_enabled("shank_snap_z", True)
        GRIP.reset()
        COB_GRIP.reset()
        _show_bruise(False)
    env = _M.get("env")
    _M.clear()
    _M.update({k: (dict(v) if isinstance(v, dict) else v) for k, v in _FRESH.items()})
    _M["env"] = env


_reset_model()


def _np(t) -> np.ndarray:
    t = getattr(t, "torch", t)
    return np.asarray(t.detach().cpu().numpy() if hasattr(t, "detach") else t, dtype=np.float64)


def _qrot(q, v) -> np.ndarray:
    w, x, y, zz = q
    u = np.array([x, y, zz], dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def _R(q) -> np.ndarray:
    return np.stack([_qrot(q, [1, 0, 0]), _qrot(q, [0, 1, 0]), _qrot(q, [0, 0, 1])], axis=1)


def _body(objects, name: str) -> tuple[np.ndarray, np.ndarray]:
    """Body origin (env-local world m) and quaternion (w, x, y, z)."""
    if objects is not None and name in objects and _M.get("env") is None:
        o = objects[name]
        x, y, zz, w = np.asarray(o.quat_xyzw, float)
        return np.asarray(o.pos, float), np.array([w, x, y, zz], float)
    env = _M.get("env")
    if env is not None:
        base = env.unwrapped
        pose = base.arena_world.get_pose_w(name)[0].detach().cpu().numpy().astype(np.float64)
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
        x, y, zz, w = pose[3:7]
        return pose[:3] - origin, np.array([w, x, y, zz], float)
    o = objects[name]
    x, y, zz, w = np.asarray(o.quat_xyzw, float)
    return np.asarray(o.pos, float), np.array([w, x, y, zz], float)


def _body_vel(objects, name: str) -> float:
    if objects is not None and name in objects and hasattr(objects[name], "lin_vel"):
        return float(np.linalg.norm(np.asarray(objects[name].lin_vel, float)))
    env = _M.get("env")
    if env is not None:
        try:
            v = env.unwrapped.scene[name].data.root_lin_vel_w[0].detach().cpu().numpy()
            return float(np.linalg.norm(v))
        except Exception:  # noqa: BLE001
            return 0.0
    return 0.0


def _rel_angle(objects, a: str, b: str, axis: str) -> float:
    """Signed angle of body b relative to body a about a's local ``axis`` (rad)."""
    _, qa = _body(objects, a)
    _, qb = _body(objects, b)
    Rr = _R(qa).T @ _R(qb)
    i = "xyz".index(axis)
    j, k = (i + 1) % 3, (i + 2) % 3
    return math.atan2(Rr[k, j], Rr[j, j])


def geometry(objects) -> dict:
    peel = {}
    for sd, s in (("f", -1.0), ("n", 1.0)):
        a = math.degrees(s * _rel_angle(objects, SHANK.name, f"husk_{sd}0", "z"))
        peel[sd] = a + 360.0 if a < -60.0 else a
    _, q_cob = _body(objects, COB.name)
    _, q_sh = _body(objects, SHANK.name)
    ay_cob = _qrot(q_cob, [0.0, 1.0, 0.0])
    ay_sh = _qrot(q_sh, [0.0, 1.0, 0.0])
    dot = float(np.clip(np.dot(ay_cob, ay_sh), -1.0, 1.0))
    bend = math.degrees(math.acos(dot))
    return dict(peel=peel, bend=bend)


def _local_pts(name: str) -> list[tuple[np.ndarray, float]]:
    """Representative bounding points (body frame) and local half-thickness/radius for each ear part."""
    if name == COB.name:
        return [(np.array([0.0, y, 0.0]), COB_R) for y in np.linspace(-COB_L / 2, COB_L / 2, 13)]
    if name == SHANK.name:
        return [(np.array([0.0, y, 0.0]), SHANK_R) for y in np.linspace(-SHANK_L / 2, SHANK_L / 2, 5)]
    side, k = name[-2], int(name[-1])
    s = 1.0 if side == "f" else -1.0
    L = SEG_L[k]
    pts = []
    for phi in PHIS:
        for fy in (-0.42, 0.0, 0.42):
            pts.append(
                (
                    np.array([s * RH * (math.cos(math.radians(phi)) - 1.0), fy * L, RH * math.sin(math.radians(phi))]),
                    TH / 2,
                )
            )
    if k == 2:
        for fy in (0.15, 0.55, 0.95):
            pts.append((np.array([s * (FLAG_X - RH), L / 2 + 0.004 + fy * FLAG_L, 0.0]), FLAG_T / 2))
            pts.append((np.array([s * (SILK_X - RH), L / 2 + 0.005 + fy * SILK_L, 0.0]), SILK_R))
    return pts


_PTS = {n: _local_pts(n) for n in EAR}


def part_distance(p, name: str, objects=None) -> float:
    bp, bq = _body(objects, name)
    Rm = _R(bq)
    p = np.asarray(p, dtype=np.float64)
    if name in (COB.name, SHANK.name):
        rad, half = (COB_R, COB_L / 2) if name == COB.name else (SHANK_R, SHANK_L / 2)
        loc = Rm.T @ (p - bp)
        return float(math.hypot(max(0.0, abs(loc[1]) - half), max(0.0, math.hypot(loc[0], loc[2]) - rad)))
    return min(float(np.linalg.norm(bp + Rm @ lp - p)) - r for lp, r in _PTS[name])


def cob_to_part_gap(name: str, objects=None) -> float:
    """Minimum surface-to-surface clearance (m) between ``name`` and ``corn_cob``."""
    bp, bq = _body(objects, name)
    Rm = _R(bq)
    return min(part_distance(bp + Rm @ lp, COB.name, objects) - r for lp, r in _PTS[name])


def _arm_jaw_state(side: str):
    """Returns (tcp, gripper_R, aperture_m, target_aperture_m, pad_left, pad_right, surface_pts) in env-local frame."""
    env = _M.get("env")
    if env is None:
        return None
    from yam_arena.constants import GRIPPER
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    rob = base.scene[f"{side}_robot"]
    lp = link_poses(rob)
    tcp = np.asarray(tcp_world(rob), dtype=np.float64) - origin
    tip_mid = np.asarray(pad_tips_world(rob), dtype=np.float64) - origin
    g_pos = lp["gripper"][:3, 3] - origin
    g_R = np.asarray(lp["gripper"][:3, :3], dtype=np.float64)
    names = list(rob.joint_names)
    j7, j8 = names.index("joint7"), names.index("joint8")
    q = _np(rob.data.joint_pos[0]).reshape(-1)
    qt = _np(rob.data.joint_pos_target[0]).reshape(-1)
    ap = abs(float(q[j7])) + abs(float(q[j8]))
    ap_t = abs(float(qt[j7])) + abs(float(qt[j8]))
    closing_ax = g_R[:, 0]
    pad_a = tcp + 0.5 * ap * closing_ax
    pad_b = tcp - 0.5 * ap * closing_ax
    tip_a = tip_mid + 0.5 * ap * closing_ax
    tip_b = tip_mid - 0.5 * ap * closing_ax
    mid_h = 0.5 * (g_pos + tcp)
    surf = [g_pos, mid_h, tcp, tip_mid, pad_a, pad_b, tip_a, tip_b]
    f_pinch = min(float(GRIPPER["effort"]), max(0.0, float(GRIPPER["stiffness"]) * 0.5 * (ap - ap_t)))
    return tcp, g_R, ap, ap_t, pad_a, pad_b, surf, f_pinch


def _cob_pinch_force(objects=None) -> float:
    """Measured clamping force (N) exerted directly on exposed kernels of the bare cob."""
    if _M.get("env") is None:
        return 0.0
    cp, cq = _body(objects, COB.name)
    cR = _R(cq)
    best = 0.0
    for side in ("left", "right"):
        st = _arm_jaw_state(side)
        if st is None:
            continue
        _, _, ap, _, pad_a, pad_b, _, f_pinch = st
        if f_pinch <= 1.0 or ap > 2.0 * COB_R + 0.012:
            continue
        da = part_distance(pad_a, COB.name, objects)
        db = part_distance(pad_b, COB.name, objects)
        if da > 0.008 or db > 0.008:
            continue
        # Check whether an unpeeled husk shell sits between either pad and the cob
        shielded = False
        for h in HUSK_NAMES:
            if min(part_distance(pad_a, h, objects), part_distance(pad_b, h, objects)) < 0.005:
                # Only shields if that husk half is still wrapped over this cob station
                sd = h[-2]
                if _M["peel"].get(sd, 0.0) < 45.0:
                    shielded = True
                    break
        if not shielded:
            loc = cR.T @ (0.5 * (pad_a + pad_b) - cp)
            if abs(loc[1]) <= COB_L / 2 + 0.005:
                best = max(best, f_pinch)
    return best


def _update_husk_drives(g: dict) -> None:
    if _M.get("env") is None or "root" not in _P:
        return
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        stage = get_current_stage()
        for sd, s in (("f", -1.0), ("n", 1.0)):
            pm = _M["peel_max"][sd]
            if _M["snapped"] and pm >= CREASE_AT_DEG:
                # Once snapped free of the cob, mid-hinges gather 36 deg inward behind the shank into a compact bundle
                k, tgt, mtgt = K_CREASE, s * CREASE_TARGET_DEG, s * 36.0
            elif pm < WRAP_RELEASE_DEG:
                k, tgt, mtgt = K_WRAP, 0.0, 0.0
            elif pm < CREASE_AT_DEG:
                pm_q = round(pm / 3.0) * 3.0
                k, tgt, mtgt = K_PEEL_HOLD, s * pm_q, 0.0
            else:
                k, tgt, mtgt = K_CREASE, s * CREASE_TARGET_DEG, 0.0
            prev = _M.get(f"drv_{sd}")
            if prev != (k, tgt, mtgt):
                _M[f"drv_{sd}"] = (k, tgt, mtgt)
                prim = stage.GetPrimAtPath(_P[f"base_{sd}"])
                if prim and prim.IsValid():
                    prim.GetAttribute("drive:angular:physics:stiffness").Set(float(k * math.pi / 180.0))
                    prim.GetAttribute("drive:angular:physics:targetPosition").Set(float(tgt))
                for mk in (1, 2):
                    mprim = stage.GetPrimAtPath(_P[f"mid{mk}_{sd}"])
                    if mprim and mprim.IsValid():
                        mprim.GetAttribute("drive:angular:physics:targetPosition").Set(float(mtgt))
    except Exception:  # noqa: BLE001
        pass


def _track(objects=None) -> None:
    if _M.get("env") is None and objects is None:
        return
    try:
        g = geometry(objects)
        cp, _ = _body(objects, COB.name)
    except Exception:  # noqa: BLE001
        return
    if not np.isfinite(cp).all():
        _M["nan"] = True
        return
    for sd in ("f", "n"):
        _M["peel"][sd] = g["peel"][sd]
        _M["peel_max"][sd] = max(_M["peel_max"][sd], g["peel"][sd])
    _M["bend"] = g["bend"]
    _update_husk_drives(g)

    if _M.get("env") is not None:
        if objects is None:
            GRIP.update(_M["env"])
            COB_GRIP.update(_M["env"])
        if not _M["snapped"] and g["bend"] >= SNAP_DEG:
            _M["snapped"] = True
            _set_joint_enabled("shank_snap_x", False)
            _set_joint_enabled("shank_snap_z", False)
            print(f"[corn] shank snapped off at step {_M['step']} (bend {g['bend']:.1f} deg)", flush=True)

        f_cob = _cob_pinch_force(objects)
        _M["peak_cob_force"] = max(_M["peak_cob_force"], f_cob)
        if f_cob > CRUSH_FORCE_N:
            _M["crush_cnt"] += 1
            if _M["crush_cnt"] >= CRUSH_STEPS and not _M["crushed"]:
                _M["crushed"] = True
                _show_bruise(True)
                print(f"[corn] kernels crushed at step {_M['step']} ({f_cob:.1f} N > {CRUSH_FORCE_N} N)", flush=True)
        else:
            _M["crush_cnt"] = 0

    # Dropped check (latched): cob touches table off the board or falls below table, or any waste part falls off table
    on_board_xy = (
        abs(cp[0] - BOARD_C[0]) <= BOARD_SIZE[0] / 2 and abs(cp[1] - BOARD_C[1]) <= BOARD_SIZE[1] / 2
    )
    if cp[2] < TOP + 0.015 or (not on_board_xy and cp[2] < TOP + COB_R + 0.015):
        if not _M["dropped"]:
            print(f"[corn] cob off board / dropped at step {_M['step']}: {np.round(cp, 3)}", flush=True)
        _M["dropped"] = True
    for wn in WASTE_NAMES:
        wp, _ = _body(objects, wn)
        if not np.isfinite(wp).all():
            _M["nan"] = True
        elif wp[2] < TOP - 0.02:
            _M["dropped"] = True


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover (offline import)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._task_reset_hooks = []
        cls.reset = reset
    if _reset_model not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_model)
    if not getattr(cls._task_step, "_corn_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _M["env"] = self._env
            _M["step"] += 1
            out = base(self, *a, **k)
            try:
                _track()
            except Exception as e:  # noqa: BLE001
                if not _M.get("hook_err"):
                    _M["hook_err"] = True
                    print(f"[corn] step hook failed: {e!r}", flush=True)
            return out

        _task_step._corn_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- scoring -------------------------------------------------------------------------------------------------------------
def grippers() -> list[np.ndarray]:
    out = []
    for side in ("left", "right"):
        st = _arm_jaw_state(side)
        if st is None:
            return []
        out.append(st[0])
    return out


def half_peeled(objects, sd: str) -> bool:
    """True when husk half ``sd`` exposes its half of the cob along the cob's length:
    - Before shank snap: base hinge turned >= 100 deg AND all 3 shell segments lie behind (cob base + 2 cm).
    - After shank snap: all 3 shell segments of that half are detached >= 3 cm clear of the cob surface.
    """
    segs = [f"husk_{sd}{k}" for k in range(3)]
    if _M["snapped"]:
        return all(cob_to_part_gap(n, objects) >= DETACH_GAP for n in segs)
    if _M["peel"].get(sd, 0.0) < PEEL_DEG:
        return False
    cp, cq = _body(objects, COB.name)
    ay = _qrot(cq, [0.0, 1.0, 0.0])
    cob_base = cp - 0.5 * COB_L * ay
    for n in segs:
        sp, _ = _body(objects, n)
        if float(np.dot(sp - cob_base, ay)) > PEEL_BASE_MARGIN:
            return False
    return True


def husk_detached(objects) -> bool:
    """True when the shank has snapped off and the shank plus all husk segments are >= 3 cm clear of the cob."""
    if not _M["snapped"]:
        return False
    return all(cob_to_part_gap(n, objects) >= DETACH_GAP for n in WASTE_NAMES)


def in_bowl(objects, name: str) -> bool:
    """True when ``name`` and all its representative bounding points lie inside the mixing bowl's 27.0 cm inner opening
    below the 8.9 cm rim, at rest (< 5 cm/s)."""
    bp, bq = _body(objects, name)
    Rm = _R(bq)
    if math.hypot(bp[0] - BOWL_C[0], bp[1] - BOWL_C[1]) > BOWL_R_IN:
        return False
    if not (TOP <= bp[2] <= TOP + BOWL_H):
        return False
    for lp, r in _PTS[name]:
        wp = bp + Rm @ lp
        if math.hypot(wp[0] - BOWL_C[0], wp[1] - BOWL_C[1]) + r > BOWL_R_IN:
            return False
        if not (TOP <= wp[2] and wp[2] + r <= TOP + BOWL_H):
            return False
    return _body_vel(objects, name) < 0.05


def cob_on_board(objects) -> bool:
    """True when the bare cob lies flat and at rest inside the cutting board's top surface."""
    p, q = _body(objects, COB.name)
    ay = _qrot(q, [0.0, 1.0, 0.0])
    ends = (p - 0.5 * COB_L * ay, p + 0.5 * COB_L * ay)
    hx = BOARD_SIZE[0] / 2 - BOARD_EDGE_INSET
    hy = BOARD_SIZE[1] / 2 - BOARD_EDGE_INSET
    inside = all(abs(e[0] - BOARD_C[0]) <= hx and abs(e[1] - BOARD_C[1]) <= hy for e in ends)
    on_top = (TOP + BOARD_TOP + COB_R - 0.006) <= p[2] <= (TOP + BOARD_TOP + COB_R + 0.012)
    level = abs(float(ay[2])) < math.sin(math.radians(14.5))
    return bool(inside and on_top and level and _body_vel(objects, COB.name) < 0.03)


def _bowl_distance(p) -> float:
    p = np.asarray(p, dtype=np.float64)
    dr = max(0.0, math.hypot(p[0] - BOWL_C[0], p[1] - BOWL_C[1]) - BOWL_R_OUT)
    dz = max(0.0, p[2] - (TOP + BOWL_H), TOP - p[2])
    return math.hypot(dr, dz)


def hands_clear(objects) -> bool:
    if _M.get("env") is None:
        return bool(_M.get("offline_hands_clear", False))
    if GRIP.holders() or COB_GRIP.holders():
        return False
    tcps = []
    for side in ("left", "right"):
        st = _arm_jaw_state(side)
        if st is None:
            return False
        tcp, _, ap, _, _, _, surf, _ = st
        if ap < OPEN_APERTURE:
            return False
        for pt in surf:
            if _bowl_distance(pt) < CLEAR:
                return False
            if any(part_distance(pt, n, objects) < CLEAR for n in EAR):
                return False
        tcps.append(tcp)
    hist = _M["ghist"] if _M["ghist"] is not None else []
    _M["ghist"] = hist
    if not hist or hist[-1][0] != _M["step"]:
        hist.append((_M["step"], [p.copy() for p in tcps]))
        del hist[: -(STILL_STEPS + 1)]
    if len(hist) < STILL_STEPS:
        return False
    dt_step = 1.0 / 20.0
    for i in range(2):
        drift = max(float(np.linalg.norm(h[1][i] - tcps[i])) for h in hist)
        step_spd = float(np.linalg.norm(hist[-1][1][i] - hist[-2][1][i])) / dt_step if len(hist) >= 2 else 99.0
        if drift > STILL_SPEED or step_spd > STILL_SPEED:
            return False
    return True


def score(objects) -> float:
    _track(objects)
    if os.environ.get("CORN_DEBUG") and _M["step"] % 10 == 0:
        print(f"[corn dbg] step {_M['step']} {progress(objects)}", flush=True)
    if _M["nan"] or _M["dropped"] or _M["crushed"]:
        return 0.0
    peeled = int(half_peeled(objects, "f")) + int(half_peeled(objects, "n"))
    if peeled == 0:
        return 0.0
    if peeled == 1:
        return 0.25
    if not (husk_detached(objects) and cob_on_board(objects)):
        return 0.50
    all_waste_in = all(in_bowl(objects, n) for n in WASTE_NAMES)
    if not (all_waste_in and hands_clear(objects)):
        return 0.75
    return 1.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def progress(objects) -> dict:
    return {
        "peel_f": round(_M["peel"]["f"], 1),
        "peel_n": round(_M["peel"]["n"], 1),
        "half_f": int(half_peeled(objects, "f")),
        "half_n": int(half_peeled(objects, "n")),
        "bend": round(_M["bend"], 1),
        "snapped": int(_M["snapped"]),
        "cob_board": int(cob_on_board(objects)),
        "waste_bowl": sum(int(in_bowl(objects, n)) for n in WASTE_NAMES),
        "dropped": int(_M["dropped"]),
        "crushed": int(_M["crushed"]),
        "peak_N": round(_M["peak_cob_force"], 1),
    }


VARIATION = {
    "obj:corn_cob": {"dx": 0.008, "dy": 0.010, "yaw": 0.05},
}
