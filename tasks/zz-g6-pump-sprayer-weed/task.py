"""Ornamental garden bed spot-weeding station on the station table: an unclamped Chapin 20000 1-gallon poly compression
sprayer (19.1 cm OD, 28.5 cm shoulder, 31.3 cm tall to seated T-plunger, 590 g dry + 500 mL tap water + 70 g brass
outlet tee & Winters PFQ800 0-15 PSI gauge = 1.16 kg, 3,285 cm3 air headspace, +y D-handle lug at (21.5, 20.1, 22.5) cm)
stands centred at (21.5, 8.5) cm inside a fixed 22.0 x 22.0 x 1.8 cm rimmed HDPE spill-containment tray, with its
11.5 cm shut-off handle + colliding Y-prismatic spring trigger lever + 30.5 cm angled brass spray wand (180 g total)
seated in the open-top twin U-saddle holster cradle at (20.5, -31.0, 17.5) cm (nozzle over a 7.0 x 4.4 cm front drip cup
at (43.7, -31.0) cm). Across a clear corridor in front sits a fixed Novelty Countryside 18-in Flower Box #16190
(20.3 x 45.7 x 16.5 cm) at (44.5, -4.0) cm with an absorbent catch deck at z = 14.0 cm and +x splash baffle: a commercial
PE artificial hosta shoot anchored at its crown at (43.5, +12.5, 14.8) cm inside the planter (zero rear lever) arches
along -y via its 14 mm petiole stem ((43.5, +4.5, 17.8) cm) so its 7.6 x 10.4 cm hosta blade (sheathed 100% edge-to-edge
on both faces in two adjacent 7.6 x 5.2 cm TeeJet 20301-2N yellow water-sensitive cards) drapes over a 9.0 cm dandelion
weed crown (with a TeeJet 20301-2N card) at (43.5, -5.5, 14.2) cm.

Run it:
  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Pump the 1-gallon garden sprayer into its green 4 to 8 PSI low-drift range, hold the overhanging hosta leaf back off the dandelion weed in the planter box, spot-spray the dandelion crown for at least 2 seconds without wetting the hosta leaf, and return the shut-off wand to its holster."
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import pathlib
import sys
import types

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import grip_limited as gl  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
RUN_LIMIT = 1200

INSTRUCTION = (
    "Pump the 1-gallon garden sprayer into its green 4 to 8 PSI low-drift range, "
    "hold the overhanging hosta leaf back off the dandelion weed in the planter box, "
    "spot-spray the dandelion crown for at least 2 seconds without wetting the hosta leaf, "
    "and return the shut-off wand to its holster."
)

# -- colours ----------------------------------------------------------------------------------------------------------
WHITE_POLY = (0.93, 0.94, 0.92)
BLACK = (0.11, 0.11, 0.12)
DARK_GREY = (0.24, 0.25, 0.27)
TRAY_GREY = (0.52, 0.54, 0.56)
BRASS = (0.78, 0.64, 0.24)
STEEL = (0.75, 0.76, 0.78)
TERRACOTTA = (0.72, 0.36, 0.22)
PEAT_DECK = (0.24, 0.18, 0.14)
HOSTA_GREEN = (0.16, 0.48, 0.18)
WEED_GREEN = (0.26, 0.56, 0.16)
TEEJET_YELLOW = (0.96, 0.86, 0.14)
TEEJET_BLUE = (0.10, 0.36, 0.88)
GAUGE_GREEN = (0.14, 0.68, 0.24)
GAUGE_RED = (0.86, 0.14, 0.12)
MIST_CYAN = (0.82, 0.94, 1.00)
BAFFLE_CLEAR = (0.82, 0.88, 0.92)

BENCH_X = (-0.15, 0.80)
BENCH_Y = (-0.64, 0.64)

# -- 1. Rimmed HDPE spill-containment tray at (0.215, 0.085): 22.0 x 22.0 x 1.8 cm + front drip cup at (0.437, -0.310) -
TRAY_X, TRAY_Y = 0.215, 0.085
TRAY_W, TRAY_H, TRAY_FLR = 0.220, 0.018, 0.004
TRAY_WALL = 0.006

TRAY_BASE = Box("tray_base", pos=(TRAY_X, TRAY_Y, TOP + TRAY_FLR / 2), size=(TRAY_W, TRAY_W, TRAY_FLR), color=TRAY_GREY)
TRAY_N = Box("tray_rim_px", pos=(TRAY_X + TRAY_W / 2 - TRAY_WALL / 2, TRAY_Y, TOP + TRAY_H / 2),
             size=(TRAY_WALL, TRAY_W, TRAY_H), color=TRAY_GREY)
TRAY_S = Box("tray_rim_mx", pos=(TRAY_X - TRAY_W / 2 + TRAY_WALL / 2, TRAY_Y, TOP + TRAY_H / 2),
             size=(TRAY_WALL, TRAY_W, TRAY_H), color=TRAY_GREY)
TRAY_L = Box("tray_rim_py", pos=(TRAY_X, TRAY_Y + TRAY_W / 2 - TRAY_WALL / 2, TOP + TRAY_H / 2),
             size=(TRAY_W - 2 * TRAY_WALL, TRAY_WALL, TRAY_H), color=TRAY_GREY)
TRAY_R = Box("tray_rim_my", pos=(TRAY_X, TRAY_Y - TRAY_W / 2 + TRAY_WALL / 2, TOP + TRAY_H / 2),
             size=(TRAY_W - 2 * TRAY_WALL, TRAY_WALL, TRAY_H), color=TRAY_GREY)

SADDLE_Y = -0.310
DRIP_X, DRIP_Y = 0.437, SADDLE_Y
DRIP_CUP = Box("nozzle_drip_cup", pos=(DRIP_X, DRIP_Y, TOP + 0.009), size=(0.070, 0.044, 0.018), color=TRAY_GREY)

# Open-top twin U-saddle holster cradle at (0.205, -0.310): rear saddle x = 0.156 & front saddle x = 0.254 (8.2 cm open
# bay around x = 0.205), 38 mm clear slot width supporting the wand handle by gravity!
SADDLE_TOP_Z = TOP + 0.163
HOLSTER_REAR = Box("holster_rear_post", pos=(0.156, SADDLE_Y, TOP + 0.0815), size=(0.016, 0.046, 0.163), color=DARK_GREY)
HOLSTER_FRONT = Box("holster_front_post", pos=(0.254, SADDLE_Y, TOP + 0.0815), size=(0.016, 0.046, 0.163), color=DARK_GREY)
HOLSTER_CHEEK_RL = Box("holster_cheek_rl", pos=(0.156, SADDLE_Y + 0.0210, SADDLE_TOP_Z + 0.013),
                       size=(0.016, 0.004, 0.026), color=DARK_GREY)
HOLSTER_CHEEK_RR = Box("holster_cheek_rr", pos=(0.156, SADDLE_Y - 0.0210, SADDLE_TOP_Z + 0.013),
                       size=(0.016, 0.004, 0.026), color=DARK_GREY)
HOLSTER_CHEEK_FL = Box("holster_cheek_fl", pos=(0.254, SADDLE_Y + 0.0210, SADDLE_TOP_Z + 0.013),
                       size=(0.016, 0.004, 0.026), color=DARK_GREY)
HOLSTER_CHEEK_FR = Box("holster_cheek_fr", pos=(0.254, SADDLE_Y - 0.0210, SADDLE_TOP_Z + 0.013),
                       size=(0.016, 0.004, 0.026), color=DARK_GREY)
HOLSTER_BACKSTOP = Box("holster_backstop", pos=(0.143, SADDLE_Y, SADDLE_TOP_Z + 0.012),
                       size=(0.006, 0.046, 0.024), color=DARK_GREY)

# -- 2. Unclamped Chapin 20000 1-Gal Poly Sprayer Tank (1.16 kg: 590 g dry + 500 mL water + 70 g brass tee/gauge) -----
TANK_R = 0.0955         # 19.1 cm OD
TANK_BODY_H = 0.281     # bottom at z = TOP + 0.004 -> shoulder at z = TOP + 0.285
TANK_MASS = 1.160       # 11.38 N gravity weight (< 13.8 N plunger seal upstroke drag!)
TANK_Z0 = TOP + TRAY_FLR + TANK_BODY_H / 2  # TOP + 0.1445

# Local offsets on TANK relative to TANK origin (0.215, 0.085, TOP + 0.1445):
# +y vertical D-handle lug protrudes outside TANK_R (y_loc = +0.116 -> world (0.215, +0.201, TOP + 0.225), 22 mm X-width)
BRACE_LOC = (0.0, +0.116, +0.0805)
BRACE_WIDTH_X = 0.022
GAUGE_LOC = (-0.076, +0.052, +0.122)   # world (0.139, +0.137, TOP + 0.2665): Winters PFQ800 6.35 cm 0-15 PSI dial
TEE_LOC = (-0.042, -0.078, +0.108)     # world (0.173, +0.007, TOP + 0.2525): 1/4-in NPT brass outlet tee barb


class SprayerTank(Cylinder):
    """Unclamped Chapin 20000 1-gal HDPE tank + +y D-handle lug + brass tee + Winters PFQ800 0-15 PSI live gauge."""


TANK = SprayerTank(
    "sprayer_tank",
    pos=(TRAY_X, TRAY_Y, TANK_Z0),
    radius=TANK_R,
    height=TANK_BODY_H,
    mass=TANK_MASS,
    color=WHITE_POLY,
)

# -- 3. Prismatic T-handle pump plunger (0.11 kg, 11.5 cm stroke along TANK local Z; 13.8 N Viton cup-seal drag) ------
PLUNGER_STROKE_MAX = 0.115  # 0.0 m seated -> +0.115 m top stop
PLUNGER_Z0 = TOP + 0.302    # seated T-bar centre (30.2 cm above tabletop; top of 2.2 cm bar = 31.3 cm)
PLUNGER_BAR_Y_OFFSET = -0.020  # T-bar spans y = +0.005..+0.125 so Right arm grips (0.215, +0.033) cleanly!
PLUNGER_BAR_W_X = 0.022
PLUNGER_MASS = 0.110

# Boyle's law parameters: 3,785 mL working capacity - 500 mL tap water = 3,285 cm3 air headspace;
# 3.8 cm ID pump bore (11.34 cm2) x 11.5 cm stroke = 130.4 cm3 -> dP = 14.7 * (130.4 / 3285) = +0.584 PSI per 11.5 cm stroke!
P_ATM_PSI = 14.7
PUMP_AREA_CM2 = math.pi * (1.9 ** 2)  # 11.341 cm2 (3.8 cm bore)
HEADSPACE_CM3 = 3285.0
SEAL_DRAG_N = 13.8       # > 11.38 N tank weight -> unbraced upstroke lifts tank off tray!
STROKE_ARM_M = 0.090     # >= 9.0 cm pull-up arms check valve
STROKE_SEAT_M = 0.020    # <= 2.0 cm push-down completes compression stroke
PSI_BLEED_RATE = 0.18    # PSI/s bleed while spraying

PSI_GREEN_LO = 4.0
PSI_GREEN_HI = 8.0
PSI_OVER_MAX = 10.0
GAUGE_MAX_PSI = 15.0
GAUGE_SPAN_DEG = 270.0


class PumpPlunger(Box):
    """Compound T-handle + stainless pump shaft sliding inside TANK along Z."""


PLUNGER = PumpPlunger(
    "pump_plunger",
    pos=(TRAY_X, TRAY_Y + PLUNGER_BAR_Y_OFFSET, PLUNGER_Z0),
    size=(PLUNGER_BAR_W_X, 0.120, 0.022),
    mass=PLUNGER_MASS,
    color=BLACK,
)

# -- 4. Spray wand (0.160 kg) + colliding Y-prismatic shut-off trigger lever (0.020 kg) = 0.180 kg total ---------------
# Seated passively on HOLSTER_REAR & HOLSTER_FRONT floors (z_floor = TOP + 0.163; 24 mm tall handle -> centre TOP + 0.1752).
# Valve handle grip_body spans y_loc = -0.0135 .. +0.0015 (15 mm wide); colliding red WAND_LEVER sits at y_loc = +0.0100
# (8 mm wide -> outer face at +0.0140, total uncompressed span = 27.5 mm across Y) on a Y-axis PrismaticJoint (-12..0 mm,
# 350 N/m return spring = 4.2 N at 12 mm full squeeze)!
WAND_X0, WAND_Y0, WAND_Z0 = 0.205, SADDLE_Y, SADDLE_TOP_Z + 0.0122
WAND_SPAN_W_Y = 0.0275      # 27.5 mm uncompressed outer span across grip_body (-13.5 mm) + wand_lever (+14.0 mm)
WAND_GRIP_H_Z = 0.024
WAND_BODY_MASS = 0.160
WAND_LEVER_MASS = 0.020
WAND_LEVER_LOC = np.array([+0.004, +0.0100, 0.0], dtype=float)
WAND_LEVER_TRAVEL_M = 0.0120  # 12.0 mm inward (-Y) full trigger travel
WAND_LEVER_SPRING_NM = 350.0  # 350 N/m -> 4.2 N return spring at 12 mm squeeze
WAND_TIP_LOC = np.array([+0.232, 0.0, -0.095], dtype=float)
_noz_ang = math.radians(62.0)
WAND_NOZ_DIR_LOC = np.array([math.cos(_noz_ang), 0.0, -math.sin(_noz_ang)], dtype=float)
WAND_BARB_LOC = np.array([-0.058, 0.0, 0.0], dtype=float)
HOSE_MAX_LEN = 0.860  # 34 in = 86.4 cm Chapin hose


class SprayWand(Box):
    """Compound Chapin 6-4607 shut-off valve body + 30.5 cm angled brass wand & nozzle."""


class WandLever(Box):
    """Colliding spring-loaded shut-off squeeze lever sliding along WAND local Y (-12.0 .. 0.0 mm)."""


WAND = SprayWand(
    "spray_wand",
    pos=(WAND_X0, WAND_Y0, WAND_Z0),
    size=(0.115, 0.026, WAND_GRIP_H_Z),
    mass=WAND_BODY_MASS,
    color=BLACK,
)

WAND_LEVER = WandLever(
    "wand_lever",
    pos=(WAND_X0 + float(WAND_LEVER_LOC[0]), WAND_Y0 + float(WAND_LEVER_LOC[1]), WAND_Z0),
    size=(0.082, 0.008, 0.018),
    mass=WAND_LEVER_MASS,
    color=GAUGE_RED,
)

# -- 5. Novelty Countryside 18-in Flower Box #16190 (20.3 x 45.7 x 16.5 cm) at (0.445, -0.040) ------------------------
BOX_X, BOX_Y = 0.445, -0.040
BOX_LX, BOX_LY, BOX_H = 0.203, 0.457, 0.165
DECK_Z = TOP + 0.140
BOX_WALL = 0.012

PLANTER_DECK = Box("planter_deck", pos=(BOX_X, BOX_Y, TOP + 0.070), size=(BOX_LX - 0.01, BOX_LY - 0.01, 0.140),
                   color=PEAT_DECK)
PLANTER_PX = Box("planter_px", pos=(BOX_X + BOX_LX / 2 - BOX_WALL / 2, BOX_Y, TOP + BOX_H / 2),
                 size=(BOX_WALL, BOX_LY, BOX_H), color=TERRACOTTA)
PLANTER_MX = Box("planter_mx", pos=(BOX_X - BOX_LX / 2 + BOX_WALL / 2, BOX_Y, TOP + BOX_H / 2),
                 size=(BOX_WALL, BOX_LY, BOX_H), color=TERRACOTTA)
PLANTER_PY = Box("planter_py", pos=(BOX_X, BOX_Y + BOX_LY / 2 - BOX_WALL / 2, TOP + BOX_H / 2),
                 size=(BOX_LX - 2 * BOX_WALL, BOX_WALL, BOX_H), color=TERRACOTTA)
PLANTER_MY = Box("planter_my", pos=(BOX_X, BOX_Y - BOX_LY / 2 + BOX_WALL / 2, TOP + BOX_H / 2),
                 size=(BOX_LX - 2 * BOX_WALL, BOX_WALL, BOX_H), color=TERRACOTTA)
SPLASH_BAFFLE = Box("splash_baffle", pos=(BOX_X + BOX_LX / 2 - 0.003, BOX_Y, TOP + BOX_H + 0.050),
                    size=(0.005, BOX_LY, 0.100), color=BAFFLE_CLEAR)

# Dandelion weed rosette + TeeJet 20301-2N (7.6 x 5.2 cm) yellow water-sensitive card at (0.435, -0.055, TOP + 0.142)
WEED_X, WEED_Y, WEED_Z = 0.435, -0.055, DECK_Z + 0.002
WEED_ROSETTE = Cylinder("dandelion_rosette", pos=(WEED_X, WEED_Y, WEED_Z), radius=0.045, height=0.004, color=WEED_GREEN)

# Realistic artificial hosta shoot (Nearly Natural / Vickerman PE frond): crown socket inside planter at (0.435, +0.125, TOP + 0.148),
# ZERO rear extension behind crown! Natural 14 mm petiole stem arches along -Y through (0.435, +0.045, TOP + 0.178) to the
# 7.6 x 10.4 cm TeeJet-clad hosta leaf blade at (0.435, -0.055, TOP + 0.166) directly covering the dandelion!
HINGE_X, HINGE_Y, HINGE_Z = 0.435, +0.125, TOP + 0.148
PETIOLE_GRIP_LOC = np.array([0.0, -0.080, +0.030], dtype=float)   # world rest: (0.435, +0.045, TOP + 0.178) — between crown & blade!
LEAF_BLADE_LOC = np.array([0.0, -0.180, +0.018], dtype=float)     # world rest: (0.435, -0.055, TOP + 0.166) — over dandelion!
LEAF_BLADE_SIZE = np.array([0.076, 0.104, 0.005], dtype=float)    # 7.6 x 10.4 cm = exact 2x TeeJet 20301-2N (7.6 x 5.2 cm)!
PETIOLE_DIAM = 0.014
LEAF_MASS = 0.045
LEAF_SPRING_NM_PER_RAD = 0.45  # 0.45 N*m/rad = 0.007854 N*m/deg in USD RevoluteJoint angular drive
LEAF_MAX_SWING_DEG = 75.0
LEAF_JOINT_STOP_DEG = 82.0


class HostaLeaf(Box):
    """Commercial PE artificial hosta frond (crown -> 14 mm petiole stem -> 7.6x10.4 cm TeeJet-clad blade; zero rear lever)."""


HOSTA_LEAF = HostaLeaf(
    "hosta_leaf",
    pos=(HINGE_X, HINGE_Y, HINGE_Z),
    size=(0.076, 0.104, 0.010),
    mass=LEAF_MASS,
    color=HOSTA_GREEN,
)

OBJECTS = [
    TRAY_BASE, TRAY_N, TRAY_S, TRAY_L, TRAY_R, DRIP_CUP,
    HOLSTER_REAR, HOLSTER_FRONT, HOLSTER_CHEEK_RL, HOLSTER_CHEEK_RR,
    HOLSTER_CHEEK_FL, HOLSTER_CHEEK_FR, HOLSTER_BACKSTOP,
    PLANTER_DECK, PLANTER_PX, PLANTER_MX, PLANTER_PY, PLANTER_MY, SPLASH_BAFFLE, WEED_ROSETTE,
    TANK, PLUNGER, WAND, WAND_LEVER, HOSTA_LEAF,
]

TWIN_EXTRA = [
    {"name": "plunger_slide", "kind": "joint", "real": "Chapin 20000 3.8 cm pump cylinder + Viton cup seal (13.8 N)",
     "note": "Z-prismatic 0..11.5 cm stroke on unclamped 1.16 kg tank"},
    {"name": "wand_lever_slide", "kind": "joint", "real": "Chapin 6-4607 spring-loaded shut-off trigger valve (350 N/m)",
     "note": "Y-prismatic -12..0 mm squeeze travel of colliding wand_lever against wand grip"},
    {"name": "hosta_crown_hinge", "kind": "joint",
     "real": "Nearly Natural PE hosta shoot in 3D-printed PLA soil crown clevis + McMaster-Carr 9271K33 torsion spring",
     "note": "X-axis revolute 0..82 deg at (43.5, +12.5, 14.8) cm lifting petiole & 7.6x10.4 cm TeeJet-clad blade"},
    {"name": "sprayer_hose", "kind": "body", "size_mm": [860, 8, 8], "colour": "black",
     "pose": [0.19, -0.15, TOP + 0.18], "real": "Chapin 20000 34-inch (86.4 cm) reinforced PVC hose"},
]

# -- scoring & physical thresholds ------------------------------------------------------------------------------------
PAD_CONTACT_MIN_N = 0.02     # >= 0.02 N real PhysX normal contact force on BOTH tip_left and tip_right required for grasp
TANK_LIFT_MAX = 0.020        # > 2.0 cm tank lift off tray floor = Score 0
TANK_SHIFT_MAX = 0.030       # > 3.0 cm horizontal tank shift from (21.5, 8.5) = Score 0
TANK_TILT_MAX_DEG = 15.0     # > 15 deg tank tilt = Score 0
WEED_AIM_R_MAX = 0.055       # <= 5.5 cm horizontal spray aim from (43.5, -5.5)
WEED_AIM_Z_LO = 0.025        # nozzle tip >= 2.5 cm above dandelion crown
WEED_AIM_Z_HI = 0.120        # nozzle tip <= 12.0 cm above dandelion crown
LEVER_OPEN_MM = 8.0          # >= 8.0 mm physical lever travel opens spray valve
LEVER_FULL_MM = 10.0         # >= 10.0 mm full physical lever squeeze for 0.85 tier
LEVER_SHUT_MM = 2.0          # <= 2.0 mm closed at end
SPRAY_GOAL_S = 2.0           # >= 2.0 s cumulative spot-spray on weed crown
MIN_STROKES = 7              # >= 7 full strokes -> >= 4.08 PSI inside 4.0-8.0 PSI green band
LEAF_CLEAR_DEG = 38.0        # >= 38 deg swing lifts 7.6x10.4 cm blade completely off (43.5, -5.5)
LEAF_SETTLE_DEG = 12.0       # <= 12 deg at end after release
HOLSTER_POS_TOL = 0.030      # <= 3.0 cm of (20.5, -31.0, 17.5)
HOLSTER_ANG_TOL_DEG = 18.0   # <= 18 deg of holstered orientation
CLEAR_DIST = 0.10            # >= 10.0 cm gripper clearance
STILL_TRAVEL = 0.015         # <= 1.5 cm TCP travel over final 1.0 s

_P: dict = {}
_ENV: dict = {"env": None}


def _gauge_angle_deg(psi_val: float) -> float:
    frac = max(0.0, min(1.0, float(psi_val) / GAUGE_MAX_PSI))
    return 225.0 - frac * GAUGE_SPAN_DEG


def _register_module() -> None:
    if __name__ not in sys.modules:
        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _make_shape(stage, path: str, kind: str, pos, size, vis_path: str, phys_path: str | None = None,
                rot_xyz_deg: tuple[float, float, float] = (0.0, 0.0, 0.0), patch_r: float = 0.0):
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    if kind == "cyl":
        g = UsdGeom.Cylinder.Define(stage, path)
        g.CreateRadiusAttr(0.5)
        g.CreateHeightAttr(1.0)
        g.CreateAxisAttr("Z")
    elif kind == "sph":
        g = UsdGeom.Sphere.Define(stage, path)
        g.CreateRadiusAttr(0.5)
    else:
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(g)
    tp = xf.AddTranslateOp()
    tp.Set(Gf.Vec3d(*(float(v) for v in pos)))
    rx, ry, rz = rot_xyz_deg
    if rx:
        xf.AddRotateXOp().Set(float(rx))
    if ry:
        xf.AddRotateYOp().Set(float(ry))
    if rz:
        xf.AddRotateZOp().Set(float(rz))
    sc = xf.AddScaleOp()
    sc.Set(Gf.Vec3f(*(float(v) for v in size)))
    bind_visual_material(path, vis_path, stage=stage)
    if phys_path is not None:
        gp = g.GetPrim()
        UsdPhysics.CollisionAPI.Apply(gp)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(gp)
        cp.CreateContactOffsetAttr(0.002)
        cp.CreateRestOffsetAttr(0.0)
        if patch_r > 0.0:
            cp.CreateTorsionalPatchRadiusAttr(float(patch_r))
        bind_physics_material(path, phys_path, stage=stage)
    return g, tp, sc


def _spawn_tank(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root_xf = UsdGeom.Xform.Define(stage, prim_path)
    root_xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root_xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, qx, qy, qz))

    prim = root_xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    ma = UsdPhysics.MassAPI.Apply(prim)
    ma.CreateMassAttr(float(TANK_MASS))
    ma.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, -0.060))
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateSolverVelocityIterationCountAttr(12)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)

    looks = f"{prim_path}/Looks"
    UsdGeom.Xform.Define(stage, looks)

    def look(nm: str, col, rough: float = 0.35, metal: float = 0.0, opac: float = 1.0):
        lp = f"{looks}/{nm}"
        c = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(col), roughness=rough, metallic=metal, opacity=opac)
        c.func(lp, c)
        return lp

    poly_lk = look("poly_lk", WHITE_POLY, rough=0.28, opac=0.88)
    water_lk = look("water_lk", (0.55, 0.78, 0.92), rough=0.20, opac=0.75)
    blk_lk = look("blk_lk", BLACK, rough=0.40)
    brass_lk = look("brass_lk", BRASS, rough=0.25, metal=0.70)
    steel_lk = look("steel_lk", STEEL, rough=0.28, metal=0.55)
    w_lk = look("w_lk", (0.97, 0.97, 0.96), rough=0.30)
    g_lk = look("g_lk", GAUGE_GREEN, rough=0.35)
    r_lk = look("r_lk", GAUGE_RED, rough=0.35)

    tank_phys = f"{looks}/tank_phys"
    sim_utils.RigidBodyMaterialCfg(static_friction=0.45, dynamic_friction=0.35).func(
        tank_phys, sim_utils.RigidBodyMaterialCfg(static_friction=0.45, dynamic_friction=0.35)
    )
    grip_phys = f"{looks}/grip_phys"
    sim_utils.RigidBodyMaterialCfg(static_friction=2.2, dynamic_friction=1.8).func(
        grip_phys, sim_utils.RigidBodyMaterialCfg(static_friction=2.2, dynamic_friction=1.8)
    )

    _make_shape(stage, f"{prim_path}/body", "cyl", (0.0, 0.0, 0.0),
                (2 * TANK_R, 2 * TANK_R, TANK_BODY_H), poly_lk, tank_phys, patch_r=0.015)
    _make_shape(stage, f"{prim_path}/water_vis", "cyl", (0.0, 0.0, -TANK_BODY_H / 2 + 0.025),
                (2 * (TANK_R - 0.005), 2 * (TANK_R - 0.005), 0.046), water_lk, None)
    _make_shape(stage, f"{prim_path}/neck", "cyl", (0.0, 0.0, TANK_BODY_H / 2 + 0.008),
                (0.072, 0.072, 0.018), blk_lk, tank_phys)

    _make_shape(stage, f"{prim_path}/brace_bar", "cube", BRACE_LOC,
                (BRACE_WIDTH_X, 0.044, 0.056), blk_lk, grip_phys, patch_r=0.018)

    _make_shape(stage, f"{prim_path}/brass_tee", "cyl", TEE_LOC, (0.018, 0.018, 0.032), brass_lk, None)
    _make_shape(stage, f"{prim_path}/prv_knob", "cyl", (0.052, +0.045, TANK_BODY_H / 2 + 0.010),
                (0.016, 0.016, 0.022), r_lk, None)

    gx, gy, gz = GAUGE_LOC
    g_d = 0.0635
    _make_shape(stage, f"{prim_path}/gauge_case", "cyl", (gx, gy, gz), (g_d, g_d, 0.018), steel_lk, None)
    _make_shape(stage, f"{prim_path}/gauge_face", "cyl", (gx, gy, gz + 0.0095), (g_d - 0.006, g_d - 0.006, 0.001), w_lk, None)
    r_arc = 0.022
    for k, pv in enumerate(np.linspace(PSI_GREEN_LO, PSI_GREEN_HI, 7)):
        ang = _gauge_angle_deg(float(pv))
        rad = math.radians(ang)
        _make_shape(stage, f"{prim_path}/g_ok_{k}", "cube",
                    (gx + r_arc * math.cos(rad), gy + r_arc * math.sin(rad), gz + 0.0104),
                    (0.0065, 0.0032, 0.001), g_lk, None, rot_xyz_deg=(0.0, 0.0, ang))
    for k, pv in enumerate(np.linspace(PSI_OVER_MAX, GAUGE_MAX_PSI, 5)):
        ang = _gauge_angle_deg(float(pv))
        rad = math.radians(ang)
        _make_shape(stage, f"{prim_path}/g_hi_{k}", "cube",
                    (gx + r_arc * math.cos(rad), gy + r_arc * math.sin(rad), gz + 0.0104),
                    (0.0060, 0.0028, 0.001), r_lk, None, rot_xyz_deg=(0.0, 0.0, ang))
    for pv in range(0, 16, 3):
        ang = _gauge_angle_deg(float(pv))
        rad = math.radians(ang)
        _make_shape(stage, f"{prim_path}/g_tk_{pv}", "cube",
                    (gx + (r_arc + 0.002) * math.cos(rad), gy + (r_arc + 0.002) * math.sin(rad), gz + 0.0106),
                    (0.0050, 0.0014, 0.001), blk_lk, None, rot_xyz_deg=(0.0, 0.0, ang))

    needle_xf = UsdGeom.Xform.Define(stage, f"{prim_path}/gauge_needle")
    needle_xf.AddTranslateOp().Set(Gf.Vec3d(float(gx), float(gy), float(gz + 0.0114)))
    n_rz = needle_xf.AddRotateZOp()
    n_rz.Set(float(_gauge_angle_deg(0.0)))
    _P["gauge_needle_rz"] = n_rz
    nb = UsdGeom.Cube.Define(stage, f"{prim_path}/gauge_needle/blade")
    nb.CreateSizeAttr(1.0)
    bxf = UsdGeom.Xformable(nb)
    bxf.AddTranslateOp().Set(Gf.Vec3d(0.010, 0.0, 0.0))
    bxf.AddScaleOp().Set(Gf.Vec3f(0.022, 0.0024, 0.0012))
    bind_visual_material(f"{prim_path}/gauge_needle/blade", r_lk, stage=stage)

    return prim


def _spawn_plunger(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root_xf = UsdGeom.Xform.Define(stage, prim_path)
    root_xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root_xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, qx, qy, qz))

    prim = root_xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(PLUNGER_MASS))
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSolverPositionIterationCountAttr(32)

    looks = f"{prim_path}/Looks"
    UsdGeom.Xform.Define(stage, looks)
    blk_lk = f"{looks}/blk"
    sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BLACK), roughness=0.35).func(
        blk_lk, sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BLACK), roughness=0.35)
    )
    st_lk = f"{looks}/steel"
    sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(STEEL), roughness=0.25, metallic=0.6).func(
        st_lk, sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(STEEL), roughness=0.25, metallic=0.6)
    )
    g_phys = f"{looks}/grip"
    sim_utils.RigidBodyMaterialCfg(static_friction=2.2, dynamic_friction=1.8).func(
        g_phys, sim_utils.RigidBodyMaterialCfg(static_friction=2.2, dynamic_friction=1.8)
    )

    _make_shape(stage, f"{prim_path}/t_bar", "cube", (0.0, 0.0, 0.0),
                (PLUNGER_BAR_W_X, 0.120, 0.022), blk_lk, g_phys, patch_r=0.018)
    _make_shape(stage, f"{prim_path}/shaft", "cyl", (0.0, -PLUNGER_BAR_Y_OFFSET, -0.075),
                (0.014, 0.014, 0.140), st_lk, None)
    return prim


def _spawn_wand(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root_xf = UsdGeom.Xform.Define(stage, prim_path)
    root_xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root_xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, qx, qy, qz))

    prim = root_xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    ma = UsdPhysics.MassAPI.Apply(prim)
    ma.CreateMassAttr(float(WAND_BODY_MASS))
    ma.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateSolverVelocityIterationCountAttr(12)
    rb.CreateMaxDepenetrationVelocityAttr(0.8)
    rb.CreateLinearDampingAttr(0.5)
    rb.CreateAngularDampingAttr(0.8)

    looks = f"{prim_path}/Looks"
    UsdGeom.Xform.Define(stage, looks)
    blk_lk = f"{looks}/blk"
    sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BLACK), roughness=0.38).func(
        blk_lk, sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BLACK), roughness=0.38)
    )
    br_lk = f"{looks}/brass"
    sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BRASS), roughness=0.25, metallic=0.75).func(
        br_lk, sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BRASS), roughness=0.25, metallic=0.75)
    )
    mist_lk = f"{looks}/mist"
    sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(MIST_CYAN), emissive_color=linear_rgb(MIST_CYAN),
                                roughness=0.4, opacity=0.45).func(
        mist_lk, sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(MIST_CYAN), emissive_color=linear_rgb(MIST_CYAN),
                                             roughness=0.4, opacity=0.45)
    )
    w_phys = f"{looks}/w_phys"
    sim_utils.RigidBodyMaterialCfg(static_friction=1.2, dynamic_friction=1.0).func(
        w_phys, sim_utils.RigidBodyMaterialCfg(static_friction=1.2, dynamic_friction=1.0)
    )

    # 11.5 cm black valve grip body (15 mm wide along Y: y_loc = -0.0135 .. +0.0015, leaving +y recess for colliding WAND_LEVER!)
    _make_shape(stage, f"{prim_path}/grip_body", "cube", (0.0, -0.0060, 0.0),
                (0.115, 0.015, WAND_GRIP_H_Z), blk_lk, w_phys, patch_r=0.016)
    # Rear & front saddle collars (26 mm wide outside the middle finger bay so handle sits square in 38 mm U-saddles!)
    _make_shape(stage, f"{prim_path}/collar_rear", "cube", (-0.049, 0.0, 0.0),
                (0.016, 0.026, WAND_GRIP_H_Z), blk_lk, w_phys)
    _make_shape(stage, f"{prim_path}/collar_front", "cube", (+0.049, 0.0, 0.0),
                (0.016, 0.026, WAND_GRIP_H_Z), blk_lk, w_phys)

    p_a = np.array([0.056, 0.0, 0.0])
    p_b = np.array([0.212, 0.0, -0.058])
    v1 = p_b - p_a
    l1 = float(np.linalg.norm(v1))
    pitch1 = math.degrees(math.atan2(-v1[2], v1[0]))
    _make_shape(stage, f"{prim_path}/wand_tube", "cube", 0.5 * (p_a + p_b), (l1, 0.008, 0.008),
                br_lk, w_phys, rot_xyz_deg=(0.0, pitch1, 0.0))

    p_c = WAND_TIP_LOC
    v2 = p_c - p_b
    l2 = float(np.linalg.norm(v2))
    pitch2 = math.degrees(math.atan2(-v2[2], v2[0]))
    _make_shape(stage, f"{prim_path}/nozzle_tip", "cube", 0.5 * (p_b + p_c), (l2, 0.011, 0.011),
                br_lk, w_phys, rot_xyz_deg=(0.0, pitch2, 0.0))

    mist_paths = []
    for k, dist_s in enumerate((0.022, 0.044, 0.068)):
        pt = WAND_TIP_LOC + dist_s * WAND_NOZ_DIR_LOC
        r_m = 0.008 + 0.18 * dist_s
        g, _, _ = _make_shape(stage, f"{prim_path}/spray_cone_{k}", "sph", pt, (2 * r_m, 2 * r_m, 2 * r_m), mist_lk, None)
        UsdGeom.Imageable(g.GetPrim()).MakeInvisible()
        mist_paths.append(f"{prim_path}/spray_cone_{k}")
    _P["spray_mist"] = mist_paths
    return prim


def _spawn_wand_lever(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root_xf = UsdGeom.Xform.Define(stage, prim_path)
    root_xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root_xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, qx, qy, qz))

    prim = root_xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(WAND_LEVER_MASS))
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateSolverVelocityIterationCountAttr(12)
    rb.CreateMaxDepenetrationVelocityAttr(0.8)

    looks = f"{prim_path}/Looks"
    UsdGeom.Xform.Define(stage, looks)
    red_lk = f"{looks}/red"
    sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(GAUGE_RED), roughness=0.35).func(
        red_lk, sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(GAUGE_RED), roughness=0.35)
    )
    lev_phys = f"{looks}/lev_phys"
    sim_utils.RigidBodyMaterialCfg(static_friction=1.4, dynamic_friction=1.1, restitution=0.0).func(
        lev_phys, sim_utils.RigidBodyMaterialCfg(static_friction=1.4, dynamic_friction=1.1, restitution=0.0)
    )

    # Real colliding shut-off trigger bar (7.4 cm along X inside the 8.2 cm grip bay x 8 mm along Y x 18 mm along Z)!
    _make_shape(stage, f"{prim_path}/lever_bar", "cube", (0.0, 0.0, 0.0),
                (0.074, 0.008, 0.018), red_lk, lev_phys, patch_r=0.014)
    return prim


def _spawn_hosta(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root_xf = UsdGeom.Xform.Define(stage, prim_path)
    root_xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root_xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, qx, qy, qz))

    prim = root_xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(LEAF_MASS))
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSolverPositionIterationCountAttr(24)

    looks = f"{prim_path}/Looks"
    UsdGeom.Xform.Define(stage, looks)
    grn_lk = f"{looks}/grn"
    sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(HOSTA_GREEN), roughness=0.42).func(
        grn_lk, sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(HOSTA_GREEN), roughness=0.42)
    )
    yel_lk = f"{looks}/yel"
    sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(TEEJET_YELLOW), roughness=0.50).func(
        yel_lk, sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(TEEJET_YELLOW), roughness=0.50)
    )
    blu_lk = f"{looks}/blu"
    sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(TEEJET_BLUE), roughness=0.45).func(
        blu_lk, sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(TEEJET_BLUE), roughness=0.45)
    )
    l_phys = f"{looks}/l_phys"
    sim_utils.RigidBodyMaterialCfg(static_friction=1.8, dynamic_friction=1.4).func(
        l_phys, sim_utils.RigidBodyMaterialCfg(static_friction=1.8, dynamic_friction=1.4)
    )

    # Natural arching PE hosta petiole stem from crown (0, 0, 0) -> petiole mid (0, -0.080, +0.030) -> leaf blade (0, -0.180, +0.018):
    # ZERO extension in +y behind crown! Colliding 14 mm cylindrical petiole stem runs along Y right between crown & blade!
    _make_shape(stage, f"{prim_path}/petiole_rise", "cyl", (0.0, -0.032, +0.014), (0.012, 0.012, 0.068),
                grn_lk, None, rot_xyz_deg=(-66.0, 0.0, 0.0))
    _make_shape(stage, f"{prim_path}/petiole_grip", "cyl", PETIOLE_GRIP_LOC,
                (PETIOLE_DIAM, PETIOLE_DIAM, 0.056), grn_lk, l_phys, rot_xyz_deg=(90.0, 0.0, 0.0), patch_r=0.016)
    _make_shape(stage, f"{prim_path}/petiole_to_blade", "cyl", (0.0, -0.122, +0.025), (0.011, 0.011, 0.042),
                grn_lk, None, rot_xyz_deg=(+80.0, 0.0, 0.0))

    # 7.6 x 10.4 cm hosta leaf blade + full-face TeeJet 20301-2N yellow water-sensitive cards (two 7.6 x 5.2 cm side-by-side!)
    _make_shape(stage, f"{prim_path}/blade_base", "cube", LEAF_BLADE_LOC, LEAF_BLADE_SIZE, grn_lk, l_phys)
    _make_shape(stage, f"{prim_path}/teejet_dry_top", "cube",
                LEAF_BLADE_LOC + np.array([0.0, 0.0, 0.0030]), (0.076, 0.104, 0.0012), yel_lk, None)
    _make_shape(stage, f"{prim_path}/teejet_dry_bot", "cube",
                LEAF_BLADE_LOC - np.array([0.0, 0.0, 0.0030]), (0.076, 0.104, 0.0012), yel_lk, None)
    g_wet, _, _ = _make_shape(stage, f"{prim_path}/teejet_wet_top", "cube",
                              LEAF_BLADE_LOC + np.array([0.0, 0.0, 0.0038]), (0.076, 0.104, 0.0014), blu_lk, None)
    UsdGeom.Imageable(g_wet.GetPrim()).MakeInvisible()
    _P["hosta_wet_vis"] = f"{prim_path}/teejet_wet_top"
    return prim


def _custom_object(spec, spawn_fn):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    _register_module()

    @configclass
    class CustomCfg(sim_utils.SpawnerCfg):
        func: object = clone(spawn_fn)

    return Object(
        name=spec.name,
        object_type=ObjectType.RIGID,
        spawner_cfg=CustomCfg(),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)),
    )


def _author_scene_joints(root: str) -> None:
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    _P["root"] = root
    P = lambda o: f"{root}/{o.name}"  # noqa: E731
    jr = f"{root}/sprayer_joints"
    UsdGeom.Xform.Define(stage, jr)

    def nocollide(a: str, b: str) -> None:
        pa, pb = stage.GetPrimAtPath(a), stage.GetPrimAtPath(b)
        if pa and pa.IsValid() and pb and pb.IsValid():
            UsdPhysics.FilteredPairsAPI.Apply(pa).GetFilteredPairsRel().AddTarget(Sdf.Path(b))

    cradle_phys = f"{jr}/cradle_phys"
    sim_utils.RigidBodyMaterialCfg(static_friction=0.20, dynamic_friction=0.15, restitution=0.0).func(
        cradle_phys, sim_utils.RigidBodyMaterialCfg(static_friction=0.20, dynamic_friction=0.15, restitution=0.0)
    )
    for ho in (HOLSTER_REAR, HOLSTER_FRONT, HOLSTER_CHEEK_RL, HOLSTER_CHEEK_RR, HOLSTER_CHEEK_FL, HOLSTER_CHEEK_FR, HOLSTER_BACKSTOP):
        hp = stage.GetPrimAtPath(P(ho))
        if hp and hp.IsValid():
            for sub in Usd.PrimRange(hp):
                if sub.HasAPI(UsdPhysics.CollisionAPI):
                    PhysxSchema_col = sub.GetAttribute("physxCollision:contactOffset")
                    if PhysxSchema_col and PhysxSchema_col.IsValid():
                        PhysxSchema_col.Set(0.001)
                    bind_physics_material(str(sub.GetPath()), cradle_phys, stage=stage)
        nocollide(P(ho), P(WAND_LEVER))

    pad_phys = f"{jr}/neoprene_pad_phys"
    sim_utils.RigidBodyMaterialCfg(static_friction=2.2, dynamic_friction=1.8, restitution=0.0).func(
        pad_phys, sim_utils.RigidBodyMaterialCfg(static_friction=2.2, dynamic_friction=1.8, restitution=0.0)
    )
    _P["neoprene_pad_phys"] = pad_phys

    # 1. Z-axis PrismaticJoint: PLUNGER inside TANK (0 .. +0.115 m, 13.8 N Coulomb Viton cup-seal friction)
    pj = UsdPhysics.PrismaticJoint.Define(stage, f"{jr}/plunger_joint")
    pj.CreateBody0Rel().SetTargets([Sdf.Path(P(TANK))])
    pj.CreateBody1Rel().SetTargets([Sdf.Path(P(PLUNGER))])
    rel0 = np.array(PLUNGER.pos, float) - np.array(TANK.pos, float)
    pj.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in rel0)))
    pj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    pj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    pj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    pj.CreateAxisAttr("Z")
    pj.CreateLowerLimitAttr(0.0)
    pj.CreateUpperLimitAttr(float(PLUNGER_STROKE_MAX))
    pj.CreateExcludeFromArticulationAttr(True)
    pd = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear")
    pd.CreateTypeAttr("force")
    pd.CreateStiffnessAttr(0.0)
    pd.CreateDampingAttr(320.0)
    pd.CreateMaxForceAttr(float(SEAL_DRAG_N))
    pd.CreateTargetVelocityAttr(0.0)
    nocollide(P(TANK), P(PLUNGER))
    _P["plunger_drive"] = pd

    # 2. Y-axis PrismaticJoint: colliding WAND_LEVER on WAND (-0.012 .. 0.0 m, 350 N/m return spring = 4.2 N at 12 mm)
    lj = UsdPhysics.PrismaticJoint.Define(stage, f"{jr}/wand_lever_joint")
    lj.CreateBody0Rel().SetTargets([Sdf.Path(P(WAND))])
    lj.CreateBody1Rel().SetTargets([Sdf.Path(P(WAND_LEVER))])
    lj.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in WAND_LEVER_LOC)))
    lj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    lj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    lj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    lj.CreateAxisAttr("Y")
    lj.CreateLowerLimitAttr(-float(WAND_LEVER_TRAVEL_M))
    lj.CreateUpperLimitAttr(0.0)
    lj.CreateExcludeFromArticulationAttr(True)
    ld = UsdPhysics.DriveAPI.Apply(lj.GetPrim(), "linear")
    ld.CreateTypeAttr("force")
    ld.CreateStiffnessAttr(float(WAND_LEVER_SPRING_NM))
    ld.CreateDampingAttr(14.0)
    ld.CreateMaxForceAttr(25.0)
    ld.CreateTargetPositionAttr(0.0)
    nocollide(P(WAND), P(WAND_LEVER))

    # 3. X-axis RevoluteJoint: HOSTA_LEAF crown anchored in PLANTER_DECK at (HINGE_X, HINGE_Y, HINGE_Z)
    hj = UsdPhysics.RevoluteJoint.Define(stage, f"{jr}/hosta_hinge")
    hj.CreateBody0Rel().SetTargets([Sdf.Path(P(PLANTER_DECK))])
    hj.CreateBody1Rel().SetTargets([Sdf.Path(P(HOSTA_LEAF))])
    h_rel = np.array([HINGE_X, HINGE_Y, HINGE_Z], float) - np.array(PLANTER_DECK.pos, float)
    hj.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in h_rel)))
    hj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    hj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    hj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    hj.CreateAxisAttr("X")
    hj.CreateLowerLimitAttr(-LEAF_JOINT_STOP_DEG)
    hj.CreateUpperLimitAttr(0.0)
    hj.CreateExcludeFromArticulationAttr(True)
    hd = UsdPhysics.DriveAPI.Apply(hj.GetPrim(), "angular")
    hd.CreateTypeAttr("force")
    hd.CreateStiffnessAttr(float(math.radians(LEAF_SPRING_NM_PER_RAD)))
    hd.CreateDampingAttr(0.0025)
    hd.CreateMaxForceAttr(1.2)
    hd.CreateTargetPositionAttr(0.0)
    for po in (PLANTER_DECK, PLANTER_PX, PLANTER_MX, PLANTER_PY, PLANTER_MY, SPLASH_BAFFLE, WEED_ROSETTE):
        nocollide(P(po), P(HOSTA_LEAF))

    # 5. Dandelion TeeJet 20301-2N (7.6 x 5.2 cm) dry yellow card + blue sprayed overlay + dynamic 12-segment PVC hose
    looks = f"{jr}/Looks"
    UsdGeom.Xform.Define(stage, looks)
    yel_lk = f"{looks}/yel"
    sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(TEEJET_YELLOW), roughness=0.50).func(
        yel_lk, sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(TEEJET_YELLOW), roughness=0.50)
    )
    blu_lk = f"{looks}/blu"
    sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(TEEJET_BLUE), roughness=0.45).func(
        blu_lk, sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(TEEJET_BLUE), roughness=0.45)
    )
    blk_lk = f"{looks}/hose_blk"
    sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BLACK), roughness=0.45).func(
        blk_lk, sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BLACK), roughness=0.45)
    )

    _make_shape(stage, f"{jr}/weed_card_dry", "cube", (WEED_X, WEED_Y, WEED_Z + 0.0028),
                (0.052, 0.076, 0.0012), yel_lk, None)
    gw, _, gw_sc = _make_shape(stage, f"{jr}/weed_card_wet", "cube", (WEED_X, WEED_Y, WEED_Z + 0.0036),
                               (0.010, 0.010, 0.0014), blu_lk, None)
    UsdGeom.Imageable(gw.GetPrim()).MakeInvisible()
    _P["weed_wet_path"] = f"{jr}/weed_card_wet"
    _P["weed_wet_sc"] = gw_sc

    hose_segs = []
    for i in range(12):
        sp = f"{jr}/hose_seg_{i}"
        g = UsdGeom.Cylinder.Define(stage, sp)
        g.CreateRadiusAttr(0.004)
        g.CreateHeightAttr(1.0)
        g.CreateAxisAttr("Z")
        xf = UsdGeom.Xformable(g)
        tp = xf.AddTranslateOp()
        op = xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
        sc = xf.AddScaleOp()
        bind_visual_material(sp, blk_lk, stage=stage)
        hose_segs.append((tp, op, sc))
    _P["hose_segs"] = hose_segs


def _wrap_last(obj):
    from isaaclab.sim.utils import clone

    cfg = obj.spawner_cfg
    orig = cfg.func
    base = getattr(orig, "__wrapped__", orig)

    def spawn(prim_path, c, *a, **k):
        prim = base(prim_path, c, *a, **k)
        _author_scene_joints(str(prim_path).rsplit("/", 1)[0])
        return prim

    cfg.func = clone(spawn)
    obj.object_cfg = obj._init_object_cfg()
    return obj


CUSTOM_SPAWNERS = {
    SprayerTank: _spawn_tank,
    PumpPlunger: _spawn_plunger,
    SprayWand: _spawn_wand,
    WandLever: _spawn_wand_lever,
    HostaLeaf: _spawn_hosta,
}
CUSTOM_NAMES = (TANK.name, PLUNGER.name, WAND.name, WAND_LEVER.name, HOSTA_LEAF.name)


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_sprayer_patched", False):
        return
    orig = scene.task_object
    orig_states = scene.object_states

    def task_object(spec):
        obj = None
        for cls_t, fn in CUSTOM_SPAWNERS.items():
            if isinstance(spec, cls_t):
                obj = _custom_object(spec, fn)
                break
        if obj is None:
            obj = orig(spec)
        if getattr(spec, "name", None) == OBJECTS[-1].name:
            _wrap_last(obj)
        return obj

    def object_states(env, names):
        _ENV["env"] = env
        out = orig_states(env, names)
        base = env.unwrapped
        origin = base.scene.env_origins[0]
        for nm in CUSTOM_NAMES:
            if nm in out:
                pose = base.arena_world.get_pose_w(nm)[0]
                pos = (pose[:3] - origin).detach().cpu().numpy().astype(np.float64)
                quat = pose[3:].detach().cpu().numpy().astype(np.float64)
                st = out[nm]
                out[nm] = type(st)(
                    pos=pos, quat_xyzw=quat, lin_vel=st.lin_vel, ang_vel=st.ang_vel, low=st.low, high=st.high
                )
        return out

    task_object._sprayer_patched = True
    scene.task_object = task_object
    scene.object_states = object_states


def _patch_embodiment(emb_mod) -> None:
    cls = emb_mod.BimanualYamEmbodiment
    if getattr(cls.__init__, "_sprayer_cs_patched", False):
        return
    orig_init = cls.__init__

    def __init__(self, *a, **kw):
        kw["contact_sensors"] = True
        return orig_init(self, *a, **kw)

    __init__._sprayer_cs_patched = True
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


# -- episode state & kinematics helpers -------------------------------------------------------------------------------
class _Episode:
    def __init__(self):
        self.last = None
        self.last_sim_step = -1
        self.step = 0
        self.time = 0.0
        self.psi = 0.0
        self.strokes = 0
        self.prev_stroke = 0.0
        self.stroke_armed = False
        self.stroke_peak = 0.0
        self.pumped_green = False
        self.leaf_cleared_aimed = False
        self.weed_spray_s = 0.0
        self.lever_mm = 0.0
        self.violated = False
        self.why = ""
        self.hosta_wetted = False
        self.held: dict[str, str | None] = {"left": None, "right": None}
        self.was_open: dict[str, bool] = {"left": True, "right": True}
        self.pads_bound = False
        self.clear_open_s = 0.0
        self.tcp_hist: list = []


_EP = _Episode()


def _dt() -> float:
    env = _ENV["env"]
    try:
        return float(env.unwrapped.step_dt)
    except Exception:  # noqa: BLE001
        return 0.05


def _np_arr(t) -> np.ndarray:
    x = getattr(t, "torch", t)
    return np.asarray(x.detach().cpu().numpy() if hasattr(x, "detach") else x, dtype=np.float64)


def _finite(a) -> bool:
    return a is not None and bool(np.isfinite(np.asarray(a, dtype=float)).all())


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def _tilt_deg(R: np.ndarray) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))


def _mat_to_quat_wxyz(R: np.ndarray) -> tuple[float, float, float, float]:
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s)


def _aperture(side: str, target: bool = False) -> float:
    env = _ENV["env"]
    if env is None:
        return 0.094
    base = env.unwrapped
    rob = base.scene[f"{side}_robot"]
    names = list(rob.joint_names)
    if target:
        try:
            g = float(_np_arr(base.action_manager.get_term(f"{side}_gripper").raw_actions).reshape(-1)[0])
            return float(np.clip(g, 0.0, 1.0)) * 0.0939
        except Exception:  # noqa: BLE001
            qt = _np_arr(rob.data.joint_pos_target[0]).reshape(-1)
            return abs(float(qt[names.index("joint7")])) + abs(float(qt[names.index("joint8")]))
    q = _np_arr(rob.data.joint_pos[0]).reshape(-1)
    return abs(float(q[names.index("joint7")])) + abs(float(q[names.index("joint8")]))


def _pad_forces_by_side() -> dict[str, tuple[float, float]]:
    """Read (tip_left_N, tip_right_N) PhysX normal contact forces from yam_arena's built-in ContactSensor per arm."""
    out = {"left": (0.0, 0.0), "right": (0.0, 0.0)}
    env = _ENV["env"]
    if env is None:
        return out
    try:
        from yam_arena.runtime import contact_forces

        sensors = env.unwrapped.scene.sensors
        for side in ("left", "right"):
            key = f"{side}_contacts"
            if key in sensors:
                cf = contact_forces(sensors[key])
                out[side] = (float(cf.get("tip_left", 0.0)), float(cf.get("tip_right", 0.0)))
    except Exception:  # noqa: BLE001
        pass
    return out


def _bind_neoprene_pads_once() -> None:
    if _EP.pads_bound or "neoprene_pad_phys" not in _P or "root" not in _P:
        return
    try:
        from isaaclab.sim.utils import bind_physics_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Usd, UsdPhysics

        st = get_current_stage()
        pmat = _P["neoprene_pad_phys"]
        if not st.GetPrimAtPath(pmat).IsValid():
            return
        for arm in ("LeftArm", "RightArm"):
            ap = st.GetPrimAtPath(f"{_P['root']}/{arm}")
            if ap and ap.IsValid():
                for p in Usd.PrimRange(ap, Usd.TraverseInstanceProxies()):
                    ps = str(p.GetPath())
                    if ("tip_left" in ps or "tip_right" in ps) and p.HasAPI(UsdPhysics.CollisionAPI):
                        bind_physics_material(ps, pmat, stage=st)
        _EP.pads_bound = True
    except Exception:  # noqa: BLE001
        pass


def tank_metrics(objects) -> tuple[float, float, float]:
    """Return (lift_m, xy_shift_m, tilt_deg) of TANK relative to nominal tray rest pose."""
    t_st = objects[TANK.name]
    R_t = _rot(t_st.quat_xyzw)
    bot_z = float((np.asarray(t_st.pos, float) - R_t[:, 2] * (TANK_BODY_H / 2))[2])
    lift = max(0.0, bot_z - (TOP + TRAY_FLR))
    shift = math.hypot(float(t_st.pos[0]) - TRAY_X, float(t_st.pos[1]) - TRAY_Y)
    return lift, shift, _tilt_deg(R_t)


def plunger_stroke_m(objects) -> float:
    """Extension of PLUNGER along TANK local +Z (0.0 m seated .. 0.115 m top)."""
    t_st, p_st = objects[TANK.name], objects[PLUNGER.name]
    R_t = _rot(t_st.quat_xyzw)
    rel = R_t.T @ (np.asarray(p_st.pos, float) - np.asarray(t_st.pos, float))
    nom_z = PLUNGER_Z0 - TANK_Z0
    return float(max(0.0, rel[2] - nom_z))


def wand_lever_mm(objects) -> float:
    """True PhysX inward compression (mm) of colliding WAND_LEVER along WAND local -Y (0.0 mm shut .. 12.0 mm full)."""
    w_st, l_st = objects[WAND.name], objects[WAND_LEVER.name]
    R_w = _rot(w_st.quat_xyzw)
    rel = R_w.T @ (np.asarray(l_st.pos, float) - np.asarray(w_st.pos, float))
    squeeze_m = float(WAND_LEVER_LOC[1]) - float(rel[1])
    return float(np.clip(squeeze_m * 1000.0, 0.0, 14.0))


def hosta_swing_deg(objects) -> float:
    """Swing angle of HOSTA_LEAF upward/+y away from dandelion (0 deg resting over weed .. +75 deg lifted clear)."""
    h_st = objects[HOSTA_LEAF.name]
    R_h = _rot(h_st.quat_xyzw)
    return float(max(0.0, math.degrees(math.atan2(-R_h[2, 1], R_h[2, 2]))))


def wand_tip_and_dir(objects) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    w_st = objects[WAND.name]
    R_w = _rot(w_st.quat_xyzw)
    w_pos = np.asarray(w_st.pos, float)
    tip = w_pos + R_w @ WAND_TIP_LOC
    u_noz = R_w @ WAND_NOZ_DIR_LOC
    u_noz = u_noz / (np.linalg.norm(u_noz) + 1e-12)
    barb = w_pos + R_w @ WAND_BARB_LOC
    return tip, u_noz, barb


def spray_aim_metrics(objects) -> tuple[float, float, bool]:
    """Return (horiz_err_m, height_above_weed_m, hits_hosta_blade) for live wand nozzle cone."""
    tip, u_noz, _ = wand_tip_and_dir(objects)
    dz_above = float(tip[2] - WEED_Z)
    if u_noz[2] >= -0.15 or dz_above <= 0.0:
        horiz_err = 9.99
    else:
        s_ground = (WEED_Z - float(tip[2])) / float(u_noz[2])
        hit_xy = tip[:2] + s_ground * u_noz[:2]
        horiz_err = math.hypot(float(hit_xy[0]) - WEED_X, float(hit_xy[1]) - WEED_Y)

    # Check full 7.6 x 10.4 cm hosta leaf blade occlusion across 12 deg spray cone rays
    h_st = objects[HOSTA_LEAF.name]
    R_h = _rot(h_st.quat_xyzw)
    h_pos = np.asarray(h_st.pos, float)
    half = 0.5 * LEAF_BLADE_SIZE + np.array([0.006, 0.006, 0.006])

    perp1 = np.cross(u_noz, np.array([0.0, 1.0, 0.0]))
    if np.linalg.norm(perp1) < 1e-6:
        perp1 = np.cross(u_noz, np.array([1.0, 0.0, 0.0]))
    perp1 = perp1 / np.linalg.norm(perp1)
    perp2 = np.cross(u_noz, perp1)
    tan_cone = math.tan(math.radians(12.0))
    rays = [u_noz]
    for a in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi):
        r_v = u_noz + tan_cone * (math.cos(a) * perp1 + math.sin(a) * perp2)
        rays.append(r_v / np.linalg.norm(r_v))

    hits_leaf = False
    max_s = max(0.05, dz_above / max(0.2, -float(u_noz[2])) + 0.04)
    for rv in rays:
        p0 = R_h.T @ (tip - h_pos) - LEAF_BLADE_LOC
        d0 = R_h.T @ rv
        t_min, t_max = -0.05, max_s
        ok = True
        for ax in range(3):
            if abs(float(d0[ax])) < 1e-8:
                if abs(float(p0[ax])) > float(half[ax]):
                    ok = False
                    break
            else:
                t1 = (-float(half[ax]) - float(p0[ax])) / float(d0[ax])
                t2 = (+float(half[ax]) - float(p0[ax])) / float(d0[ax])
                if t1 > t2:
                    t1, t2 = t2, t1
                t_min = max(t_min, t1)
                t_max = min(t_max, t2)
                if t_min > t_max:
                    ok = False
                    break
        if ok:
            hits_leaf = True
            break

    return horiz_err, dz_above, hits_leaf


def wand_holstered_ok(objects) -> bool:
    if _EP.held["left"] == WAND.name or _EP.held["right"] == WAND.name:
        return False
    w_st = objects[WAND.name]
    w_pos = np.asarray(w_st.pos, float)
    pos_err = float(np.linalg.norm(w_pos - np.array([WAND_X0, WAND_Y0, WAND_Z0], float)))
    R_w = _rot(w_st.quat_xyzw)
    cos_a = max(-1.0, min(1.0, 0.5 * (float(np.trace(R_w)) - 1.0)))
    ang_err = math.degrees(math.acos(cos_a))
    return bool(pos_err <= HOLSTER_POS_TOL and ang_err <= HOLSTER_ANG_TOL_DEG)


def _clear_welds() -> None:
    GRIP.reset()
    WGRIP.reset()


def _grasp_candidates() -> list[tuple[str, np.ndarray, np.ndarray, float, int]]:
    """Return list of (target_name, loc_center_in_body, half_extents_in_body, pinch_width_m, along_axis_idx)."""
    return [
        # 1. TANK +y vertical D-handle lug: 22 mm X-width, pinched across local X, sliding along Z (axis 2)
        (TANK.name, np.asarray(BRACE_LOC, float), np.array([0.018, 0.026, 0.032]), BRACE_WIDTH_X, 2),
        # 2. PLUNGER T-bar: 22 mm X-width, 120 mm along local Y (axis 1)
        (PLUNGER.name, np.zeros(3, float), np.array([0.018, 0.062, 0.018]), PLUNGER_BAR_W_X, 1),
        # 3. WAND shut-off grip + colliding WAND_LEVER: 27.5 mm uncompressed Y-span, 82 mm open grip bay along X (axis 0)
        (WAND.name, np.zeros(3, float), np.array([0.044, 0.022, 0.020]), WAND_SPAN_W_Y, 0),
        # 4. HOSTA_LEAF natural 14 mm petiole stem between crown & blade: 56 mm along local Y (axis 1)
        (HOSTA_LEAF.name, np.asarray(PETIOLE_GRIP_LOC, float), np.array([0.018, 0.030, 0.018]), PETIOLE_DIAM, 1),
    ]


def _bpath(o) -> str | None:
    return f"{_P['root']}/{o.name}" if "root" in _P else None


def _grip_bodies() -> list[str]:
    return [p for p in (_bpath(TANK), _bpath(PLUNGER), _bpath(HOSTA_LEAF)) if p]


def _engage_ok(side: str, mid, body: str) -> bool:
    """Pads must close on the named handle region (lug, T-bar, petiole stem, wand grip bay), not anywhere on the body,
    and a hand holds only one thing at a time."""
    if GRIP.holding(side) or WGRIP.holding(side):
        return False
    objects = _EP.last
    if objects is None:
        return False
    nm = body.rsplit("/", 1)[-1]
    for c_nm, loc_c, half, _w, _ax in _grasp_candidates():
        if c_nm != nm:
            continue
        b_st = objects[nm]
        rel = _rot(b_st.quat_xyzw).T @ (np.asarray(mid, float) - np.asarray(b_st.pos, float)) - loc_c
        return bool(np.all(np.abs(rel) <= half))
    return False


class _WandGrip(gl.LimitedGrip):
    """LimitedGrip on the shut-off handle: one pad bears on the handle body, the other on its spring trigger lever, so
    pad contact counts on either part. The lever gives under the pads (350 N/m), so the jaws stall only a fraction of a
    millimetre short of their command; 'squeezing' therefore means commanded to within 0.5 mm of the measured opening
    (closed onto the handle) instead of 1 mm past it. Caps unchanged."""

    SQUEEZE_M = -0.0005

    def pad_forces(self, side: str) -> dict:
        """Per-pad PhysX normal contact force from the arm's built-in finger-pad ContactSensor (tip_left, tip_right).
        Which part each pad presses (handle body or trigger lever) is checked by _engage_ok's grip-bay box."""
        w = _bpath(WAND)
        return {w: _pad_forces_by_side().get(side, (0.0, 0.0))} if w else {}

    def update(self, env) -> None:
        old = gl.SQUEEZE_MARGIN
        gl.SQUEEZE_MARGIN = self.SQUEEZE_M
        try:
            super().update(env)
        finally:
            gl.SQUEEZE_MARGIN = old


WAND_K_ROT = 4.0  # N m/rad on the rigid shut-off handle (stiffer, flatter grip faces than the round bar/lug/stem)
PAD_K_ROT = 1.5  # N m/rad rotational stiffness of a handle rocking in the 2 mm neoprene-sleeved pads (lib default 50 = bare)
_JROOT = lambda: f"{_P['root']}/sprayer_joints" if "root" in _P else None  # noqa: E731
GRIP = gl.LimitedGrip(_grip_bodies, label="tank lug, pump T-handle or hosta petiole", name="pad_grip",
                      joint_root=_JROOT, engage_ok=_engage_ok, k_rot=PAD_K_ROT, log_prefix="[sprayer grip]")
WGRIP = _WandGrip(lambda: _bpath(WAND), label="shut-off handle and trigger", name="wand_grip",
                  joint_root=_JROOT, engage_ok=_engage_ok, k_rot=WAND_K_ROT, lose_steps=0,
                  log_prefix="[sprayer wand grip]")


def _update_grasps(objects) -> None:
    """Force-limited two-pad grips (cablekit grip_limited.LimitedGrip): engage only on PhysX contact on BOTH finger pads
    with the jaws squeezing, compliant capped spring joint, slips above 50 N / 0.50 N m / 1.50 N m, releases on open.
    WAND_LEVER stays free on its Y-prismatic joint so finger-pad motion drives lever travel."""
    env = _ENV["env"]
    if env is None or "root" not in _P:
        return
    _bind_neoprene_pads_once()
    GRIP.update(env)
    WGRIP.update(env)
    for side in ("left", "right"):
        b = GRIP.held_body(side) or WGRIP.held_body(side)
        _EP.held[side] = b.rsplit("/", 1)[-1] if b else None


def _sync_visuals(objects=None, spraying: bool = False) -> None:
    if "gauge_needle_rz" not in _P:
        return
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        st = get_current_stage()
        _P["gauge_needle_rz"].Set(float(_gauge_angle_deg(_EP.psi)))

        for mp in _P.get("spray_mist", ()):
            prim = st.GetPrimAtPath(mp)
            if prim and prim.IsValid():
                im = UsdGeom.Imageable(prim)
                im.MakeVisible() if spraying else im.MakeInvisible()

        hw_p = st.GetPrimAtPath(_P.get("hosta_wet_vis", ""))
        if hw_p and hw_p.IsValid():
            im = UsdGeom.Imageable(hw_p)
            im.MakeVisible() if _EP.hosta_wetted else im.MakeInvisible()

        ww_p = st.GetPrimAtPath(_P.get("weed_wet_path", ""))
        if ww_p and ww_p.IsValid():
            im = UsdGeom.Imageable(ww_p)
            if _EP.weed_spray_s > 0.05:
                im.MakeVisible()
                frac = min(1.0, _EP.weed_spray_s / SPRAY_GOAL_S)
                _P["weed_wet_sc"].Set(Gf.Vec3f(float(0.014 + 0.038 * frac), float(0.018 + 0.058 * frac), 0.0014))
            else:
                im.MakeInvisible()

        if objects is not None and "hose_segs" in _P:
            t_st = objects[TANK.name]
            p0 = np.asarray(t_st.pos, float) + _rot(t_st.quat_xyzw) @ np.asarray(TEE_LOC, float)
            _, _, p2 = wand_tip_and_dir(objects)
            mid = 0.5 * (p0 + p2)
            sag = max(0.03, 0.14 - 0.18 * float(np.linalg.norm(p2 - p0)))
            p1 = np.array([mid[0] - 0.04, mid[1] - 0.02, min(p0[2], p2[2]) - sag], dtype=float)
            p1[2] = max(TOP + 0.020, float(p1[2]))
            segs = _P["hose_segs"]
            n_s = len(segs)
            pts = []
            for k in range(n_s + 1):
                u = k / n_s
                pts.append(((1 - u) ** 2) * p0 + 2 * (1 - u) * u * p1 + (u ** 2) * p2)
            for k, (tp, op, sc) in enumerate(segs):
                a, b = pts[k], pts[k + 1]
                dv = b - a
                ln = max(1e-4, float(np.linalg.norm(dv)))
                z_ax = dv / ln
                ref = np.array([1.0, 0.0, 0.0]) if abs(z_ax[0]) < 0.8 else np.array([0.0, 1.0, 0.0])
                x_ax = np.cross(ref, z_ax)
                x_ax /= np.linalg.norm(x_ax)
                y_ax = np.cross(z_ax, x_ax)
                R_s = np.column_stack([x_ax, y_ax, z_ax])
                qw, qx, qy, qz = _mat_to_quat_wxyz(R_s)
                tp.Set(Gf.Vec3d(*(float(v) for v in (0.5 * (a + b)))))
                op.Set(Gf.Quatd(float(qw), float(qx), float(qy), float(qz)))
                sc.Set(Gf.Vec3f(1.0, 1.0, float(ln * 1.06)))
    except Exception:  # noqa: BLE001
        pass


def reset_episode() -> None:
    global _EP
    _EP = _Episode()
    _clear_welds()
    _sync_visuals(None, False)


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_sprayer_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._sprayer_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_sprayer_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            out = base(self, *a, **k)
            try:
                from yam_arena.scene import object_states

                st = object_states(self._env, self.task.names)
                _update(st)
            except Exception as e:  # noqa: BLE001
                print(f"[sprayer] step hook failed: {e!r}", flush=True)
            return out

        _task_step._sprayer_hooked = True
        cls._task_step = _task_step


_install_hooks()


def _robots():
    env = _ENV["env"]
    if env is None:
        return None, None
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return base, origin


def gripper_points() -> list[np.ndarray]:
    base, origin = _robots()
    if base is None:
        return []
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    pts = []
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        t = np.asarray(tcp_world(rob), dtype=np.float64) - origin
        tip = np.asarray(pad_tips_world(rob), dtype=np.float64) - origin
        ax = (tip - t) / (np.linalg.norm(tip - t) + 1e-12)
        pts += [tip, t, t - 0.02 * ax, t - 0.04 * ax]
        lp = link_poses(rob)
        for n in ("tip_left", "tip_right"):
            if n in lp:
                pts.append(np.asarray(lp[n][:3, 3], dtype=np.float64) - origin)
    return pts


def _tcps():
    base, origin = _robots()
    if base is None:
        return None
    from yam_arena.runtime import tcp_world

    return [np.asarray(tcp_world(base.scene[k]), float) - origin for k in ("left_robot", "right_robot")]


def _cyl_dist(p, c, r: float, h: float) -> float:
    dr = max(0.0, math.hypot(float(p[0]) - float(c[0]), float(p[1]) - float(c[1])) - r)
    dz = max(0.0, abs(float(p[2]) - float(c[2])) - h / 2)
    return math.hypot(dr, dz)


def _box_dist_pose(p, pos, quat_xyzw, size) -> float:
    R = _rot(quat_xyzw)
    loc = R.T @ (np.asarray(p, float) - np.asarray(pos, float))
    return float(np.linalg.norm(np.maximum(np.abs(loc) - np.asarray(size, float) / 2, 0.0)))


def hands_clear(objects) -> bool:
    if _EP.held["left"] is not None or _EP.held["right"] is not None:
        return False
    pts = gripper_points()
    if not pts:
        return False
    for side in ("left", "right"):
        if _aperture(side, target=False) < 0.055:
            return False
    t_st = objects[TANK.name]
    p_st = objects[PLUNGER.name]
    w_st = objects[WAND.name]
    h_st = objects[HOSTA_LEAF.name]
    R_h = _rot(h_st.quat_xyzw)
    h_mid = np.asarray(h_st.pos, float) + R_h @ np.array([0.0, -0.115, +0.018], float)
    for p in pts:
        if _cyl_dist(p, t_st.pos, TANK_R + 0.025, TANK_BODY_H + 0.02) < CLEAR_DIST:
            return False
        if _box_dist_pose(p, p_st.pos, p_st.quat_xyzw, PLUNGER.size) < CLEAR_DIST:
            return False
        if _box_dist_pose(p, w_st.pos, w_st.quat_xyzw, (0.32, 0.04, 0.14)) < CLEAR_DIST:
            return False
        if _box_dist_pose(p, h_mid, h_st.quat_xyzw, (0.09, 0.24, 0.06)) < CLEAR_DIST:
            return False
        if _cyl_dist(p, WEED_ROSETTE.pos, WEED_ROSETTE.radius, WEED_ROSETTE.height) < CLEAR_DIST:
            return False
        for fb in (
            TRAY_BASE, TRAY_N, TRAY_S, TRAY_L, TRAY_R, DRIP_CUP,
            HOLSTER_REAR, HOLSTER_FRONT, HOLSTER_CHEEK_RL, HOLSTER_CHEEK_RR,
            HOLSTER_CHEEK_FL, HOLSTER_CHEEK_FR, HOLSTER_BACKSTOP,
            PLANTER_DECK, PLANTER_PX, PLANTER_MX, PLANTER_PY, PLANTER_MY, SPLASH_BAFFLE,
        ):
            if _box_dist_pose(p, fb.pos, (0.0, 0.0, 0.0, 1.0), fb.size) < CLEAR_DIST:
                return False
    return True


def hands_still() -> bool:
    h = _EP.tcp_hist
    if len(h) < 2 or h[-1][0] - h[0][0] < 1.0 - 1e-6:
        return False
    pts = np.asarray([x[1] for x in h], dtype=float)
    moved = np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=2), axis=0)
    return bool(np.max(moved) <= STILL_TRAVEL)


def _update(objects) -> None:
    env = _ENV["env"]
    if env is not None:
        sim_step = int(env.unwrapped.common_step_counter)
        if _EP.last_sim_step == sim_step:
            return
        _EP.last_sim_step = sim_step
    elif objects is _EP.last:
        return
    _EP.last = objects
    prev_viol = _EP.violated
    dt = _dt()
    _EP.step += 1
    _EP.time += dt
    tcps = _tcps()
    if tcps is not None:
        _EP.tcp_hist.append((_EP.time, tcps))
        while len(_EP.tcp_hist) > 2 and _EP.tcp_hist[1][0] <= _EP.time - 1.0 + 1e-6:
            _EP.tcp_hist.pop(0)

    for nm in CUSTOM_NAMES:
        if not (_finite(objects[nm].pos) and _finite(objects[nm].quat_xyzw)):
            _EP.violated, _EP.why = True, f"non-finite {nm}"
            return

    _update_grasps(objects)
    _EP.lever_mm = wand_lever_mm(objects)

    # 1. Tank stability & Boyle's-law plunger pumping
    lift_m, shift_m, tilt_d = tank_metrics(objects)
    stroke = plunger_stroke_m(objects)

    if stroke >= STROKE_ARM_M:
        _EP.stroke_armed = True
        _EP.stroke_peak = max(_EP.stroke_peak, stroke)
    elif _EP.stroke_armed and stroke <= STROKE_SEAT_M:
        swept_cm3 = PUMP_AREA_CM2 * (_EP.stroke_peak * 100.0)
        _EP.psi += P_ATM_PSI * (swept_cm3 / HEADSPACE_CM3)
        _EP.strokes += 1
        print(f"[sprayer] stroke {_EP.strokes}: peak={_EP.stroke_peak*100:.1f}cm -> P={_EP.psi:.2f} PSI", flush=True)
        _EP.stroke_armed = False
        _EP.stroke_peak = 0.0

    if "plunger_drive" in _P:
        try:
            f_air = float(_EP.psi) * 0.6895 * PUMP_AREA_CM2
            ds = stroke - _EP.prev_stroke
            _P["plunger_drive"].GetMaxForceAttr().Set(float(SEAL_DRAG_N if ds >= -1e-4 else SEAL_DRAG_N + f_air))
        except Exception:  # noqa: BLE001
            pass
    _EP.prev_stroke = stroke

    if _EP.psi > PSI_GREEN_HI:
        _EP.pumped_green = False
    elif _EP.strokes >= MIN_STROKES and PSI_GREEN_LO <= _EP.psi <= PSI_GREEN_HI and stroke <= STROKE_SEAT_M:
        _EP.pumped_green = True

    # 2. Hosta clearance & physical shut-off lever spray physics
    swing_d = hosta_swing_deg(objects)
    horiz_err, dz_above, hits_leaf = spray_aim_metrics(objects)
    aim_ok = (horiz_err <= WEED_AIM_R_MAX) and (WEED_AIM_Z_LO <= dz_above <= WEED_AIM_Z_HI)
    wand_unclipped = not wand_holstered_ok(objects)

    if _EP.pumped_green and swing_d >= LEAF_CLEAR_DEG and wand_unclipped and aim_ok and not hits_leaf:
        _EP.leaf_cleared_aimed = True

    spraying = (_EP.lever_mm >= LEVER_OPEN_MM) and (_EP.psi >= 0.5)
    if _EP.lever_mm >= LEVER_OPEN_MM:
        if _EP.psi < 3.5 or _EP.psi > PSI_GREEN_HI:
            if hits_leaf and _EP.psi >= 0.5:
                _EP.hosta_wetted = True
            if not _EP.violated:
                _EP.violated, _EP.why = True, f"triggered outside 3.5-8.0 PSI ({_EP.psi:.2f} PSI)"
        elif hits_leaf:
            _EP.hosta_wetted = True
            if not _EP.violated:
                _EP.violated, _EP.why = True, "spray wetted hosta leaf (TeeJet card blue)"
        elif not aim_ok:
            if not _EP.violated:
                _EP.violated, _EP.why = True, f"spray off dandelion target (r={horiz_err*100:.1f}cm, dz={dz_above*100:.1f}cm)"
        else:
            if _EP.lever_mm >= LEVER_FULL_MM and swing_d >= LEAF_CLEAR_DEG:
                _EP.weed_spray_s += dt
            _EP.psi = max(0.0, _EP.psi - PSI_BLEED_RATE * dt)

    # 3. Global disqualifiers (latched over entire episode)
    t_st = objects[TANK.name]
    w_st = objects[WAND.name]
    _, _, barb = wand_tip_and_dir(objects)
    tee_w = np.asarray(t_st.pos, float) + _rot(t_st.quat_xyzw) @ np.asarray(TEE_LOC, float)
    hose_dist = float(np.linalg.norm(barb - tee_w))

    if not _EP.violated:
        if lift_m > TANK_LIFT_MAX:
            _EP.violated, _EP.why = True, f"unbraced tank lifted {lift_m*100:.1f} cm > 2.0 cm"
        elif shift_m > TANK_SHIFT_MAX:
            _EP.violated, _EP.why = True, f"tank shifted {shift_m*100:.1f} cm > 3.0 cm"
        elif tilt_d > TANK_TILT_MAX_DEG:
            _EP.violated, _EP.why = True, f"tank tilted {tilt_d:.1f} deg > 15 deg"
        elif _EP.psi > PSI_OVER_MAX:
            _EP.violated, _EP.why = True, f"tank overpressurized ({_EP.psi:.2f} PSI > 10.0 PSI)"
        elif swing_d > LEAF_MAX_SWING_DEG:
            _EP.violated, _EP.why = True, f"hosta leaf bent {swing_d:.1f} deg > 75 deg"
        elif hose_dist > HOSE_MAX_LEN:
            _EP.violated, _EP.why = True, f"hose pulled taut {hose_dist*100:.1f} cm > 86 cm"
        elif float(t_st.pos[2]) < TOP - 0.02 or float(w_st.pos[2]) < TOP - 0.02:
            _EP.violated, _EP.why = True, "tank or wand left table"

    if hands_clear(objects):
        _EP.clear_open_s += dt
    else:
        _EP.clear_open_s = 0.0

    if _EP.violated and not prev_viol:
        print(f"[sprayer] VIOLATED at step {_EP.step} (t={_EP.time:.2f}s): {_EP.why}", flush=True)

    _sync_visuals(objects, spraying)


def score(objects) -> float:
    """Score 0 if ever: spray wets any part of the 7.6x10.4 cm hosta leaf, valve opens (>=8 mm) off-target (>5.5 cm from
    (43.5,-5.5) or outside 2.5-12.0 cm height) or outside 3.5-8.0 PSI, tank pressure >10 PSI, unclamped tank lifts >2.0 cm,
    shifts >3.0 cm, or tilts >15 deg, hosta bends >75 deg, hose stretches >86 cm, or tank/wand leaves table.
    Else: 0.25 if >=7 strokes (>=9 cm) reach 4.0-8.0 PSI with plunger seated (<=2 cm); 0.55 if hosta swung >=38 deg clear
    while wand aimed over weed; 0.85 if physical shut-off lever squeezed >=10 mm for >=2.0 s onto weed; 1.00 if valve shut
    (<=2 mm), wand resting in holster, hosta settled (<=12 deg), and both grippers open (>=5.5 cm), >=10 cm clear, moving
    <=1.5 cm over final 1.0 s."""
    _update(objects)
    if _EP.violated:
        return 0.0
    if not (_EP.pumped_green and plunger_stroke_m(objects) <= STROKE_SEAT_M):
        return 0.0
    if not _EP.leaf_cleared_aimed:
        return 0.25
    if _EP.weed_spray_s < SPRAY_GOAL_S:
        return 0.55
    end_ok = (
        _EP.lever_mm <= LEVER_SHUT_MM
        and wand_holstered_ok(objects)
        and objects[WAND.name].resting(0.02, 1.0)
        and hosta_swing_deg(objects) <= LEAF_SETTLE_DEG
        and hands_clear(objects)
        and _EP.clear_open_s >= 1.0 - 1e-6
        and hands_still()
    )
    return 1.0 if end_ok else 0.85


def success(objects) -> bool:
    _update(objects)
    return bool(score(objects) == 1.0 and objects[WAND.name].resting(0.02, 1.0))


def distance(objects) -> float:
    _update(objects)
    if _EP.violated:
        return 4.0
    if not _EP.pumped_green:
        return 3.0 - min(1.0, _EP.psi / PSI_GREEN_LO)
    if _EP.weed_spray_s < SPRAY_GOAL_S:
        return 1.0 + (SPRAY_GOAL_S - _EP.weed_spray_s) / SPRAY_GOAL_S
    w_pos = np.asarray(objects[WAND.name].pos, float)
    return float(np.linalg.norm(w_pos - np.array([WAND_X0, WAND_Y0, WAND_Z0], float)))


def progress(objects) -> dict:
    _update(objects)
    lift_m, shift_m, _ = tank_metrics(objects)
    horiz_err, dz_above, hits_leaf = spray_aim_metrics(objects)
    return {
        "strokes": int(_EP.strokes),
        "psi": round(_EP.psi, 2),
        "pumped_green": float(_EP.pumped_green),
        "hosta_deg": round(hosta_swing_deg(objects), 1),
        "lever_mm": round(_EP.lever_mm, 1),
        "aim_err_cm": round(horiz_err * 100.0, 2),
        "dz_cm": round(dz_above * 100.0, 2),
        "hits_leaf": float(hits_leaf),
        "weed_spray_s": round(_EP.weed_spray_s, 2),
        "holstered": float(wand_holstered_ok(objects)),
        "tank_lift_mm": round(lift_m * 1000.0, 1),
        "tank_shift_mm": round(shift_m * 1000.0, 1),
        "hands_clear": float(hands_clear(objects)),
        "hands_still": float(hands_still()),
        "safe": float(not _EP.violated),
    }


VARIATION: dict = {
    "obj:sprayer_tank": {"dx": 0.010, "dy": 0.010, "yaw": 0.08},
}
