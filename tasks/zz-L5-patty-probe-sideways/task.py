"""Check a burger patty's doneness: hold it with tongs and probe sideways into its centre.

Scene (metres from the arm-base midpoint on the station table, x forward, y left, z up):
- Griddle: unheated Lodge LDP3 reversible cast-iron griddle (42.5 x 24.1 x 1.6 cm), flat side up, long axis along y,
  centred at x 0.34, y 0.00 on a wipeable 60 x 44 x 0.15 cm food-safe silicone mat, oiled (patty-iron friction 0.20).
- Patty: cold precooked quarter-pound beef patty (10.5 cm diameter, 1.6 cm thick, 100 g, refrigerated at 4 C, 4 mm
  white top-centre dot) centred at x 0.30, y 0.00 on the griddle.
- Thermometer: ThermoWorks Thermapen ONE folding-probe instant-read digital thermometer (yellow ABS body 15.6 x 4.8 x
  1.9 cm, 115 g, backlit 7-segment LCD, motion-sensing auto-wake; 10.9 cm x 3.0 mm stainless probe unfolded +x,
  tapering to a 1.5 mm sensing tip, 3 mm red tape band 5.25 cm from the tip), flat on the table at x 0.22, y -0.28.
- Tongs: unlocked 30.5 cm (12 in) stainless-steel spring tongs (165 g, scalloped jaws sprung 12.5 cm open along +x),
  standing on edge on the table with rear hinge centred at x 0.13, y +0.305.

Run a trial:
  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
      -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 900 \\
      --instruction "Take the burger patty's centre temperature with the thermometer, probe in through its side, then put the thermometer and tongs down."
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
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
CONTROL_HZ = 20.0
DT = 1.0 / CONTROL_HZ

INSTRUCTION = ("Take the burger patty's centre temperature with the thermometer, probe in through its side, then put "
               "the thermometer and tongs down.")

# -- colours (sRGB) ---------------------------------------------------------------------------------------------------
IRON = (0.11, 0.11, 0.12)
MAT_COL = (0.74, 0.80, 0.84)
RING_MARK = (0.34, 0.34, 0.36)
BEEF = (0.33, 0.19, 0.12)
SEAR = (0.16, 0.09, 0.06)
WHITE = (0.96, 0.96, 0.95)
YELLOW = (0.95, 0.76, 0.10)
STEEL = (0.74, 0.75, 0.77)
DARK = (0.14, 0.14, 0.15)
RED = (0.86, 0.09, 0.09)
LCD_BG = (0.68, 0.78, 0.66)
LCD_INK = (0.10, 0.13, 0.10)

# -- Lodge LDP3 reversible cast-iron griddle (42.5 x 24.1 x 1.6 cm, flat side up, fixed) -----------------------------
GR_X, GR_Y = 0.340, 0.000
GR_W, GR_L, GR_T = 0.241, 0.425, 0.016  # along x (9.5 in), along y (16.75 in), thick (0.63 in)
GR_X0, GR_X1 = GR_X - GR_W / 2, GR_X + GR_W / 2  # 0.2195 .. 0.4605
GR_Y0, GR_Y1 = GR_Y - GR_L / 2, GR_Y + GR_L / 2  # -0.2125 .. +0.2125
GRIDDLE = Box("griddle", pos=(GR_X, GR_Y, TOP + GR_T / 2), size=(GR_W, GR_L, GR_T), color=IRON)

# -- cooked chilled quarter-pound beef patty (10.5 cm dia, 1.6 cm thick, 100 g, 4 C) ---------------------------------
P_R, P_T, P_M = 0.0525, 0.016, 0.100
P_X, P_Y = 0.300, 0.000
PATTY = Cylinder("patty", pos=(P_X, P_Y, TOP + GR_T + P_T / 2 + 0.0002), radius=P_R, height=P_T, mass=P_M, color=BEEF)

# -- ThermoWorks Thermapen ONE (15.6 x 4.8 x 1.9 cm, 115 g; 10.9 cm x 3.0 mm probe unfolded +x, 1.5 mm tip) ----------
TB_L, TB_W, TB_T, TB_M = 0.156, 0.048, 0.019, 0.115
PROBE_L, PROBE_R, TIP_R = 0.109, 0.0015, 0.00075
TIP_LOCAL = np.array([TB_L / 2 + PROBE_L, 0.0, 0.0])  # +0.187 m from body centre
SHAFT_BASE_LOCAL = np.array([TB_L / 2, 0.0, 0.0])
MARK_FROM_TIP = 0.0525  # 5.25 cm red depth band (= P_R)
TH_X, TH_Y = 0.220, -0.280
THERMO = Box("thermometer", pos=(TH_X, TH_Y, TOP + TB_T / 2 + 0.0003), size=(TB_L, TB_W, TB_T), mass=TB_M,
             color=YELLOW)

# -- 30.5 cm (12 in) unlocked stainless spring tongs (hinge + two symmetric spring-hinged scalloped arms) -------------
THG_L, THG_W, THG_H, THG_M = 0.028, 0.024, 0.024, 0.035
TA_L, TA_THK, TA_H, TA_M = 0.285, 0.0035, 0.024, 0.065  # total length rear-to-tip = 30.5 cm, total mass = 165 g
PAD_L, PAD_THK, PAD_H = 0.050, 0.0045, 0.028
PIN_DX, PIN_DY = 0.010, 0.006
THETA0_DEG = 13.5
THETA0 = math.radians(THETA0_DEG)
TG_GRIP_D = 0.095  # grip cushion midpoint from pivot pin (10.5 cm from rear of hinge)
TG_HX, TG_HY = 0.130, 0.305
TG_Z0 = TOP + THG_H / 2 + 0.0003

TONGS_HINGE = Box("tongs_hinge", pos=(TG_HX, TG_HY, TG_Z0), size=(THG_L, THG_W, THG_H), mass=THG_M, color=STEEL)


def _tong_arm_spec(name: str, sgn: int) -> Box:
    yaw = sgn * THETA0
    u = np.array([math.cos(yaw), math.sin(yaw)])
    pin = np.array([TG_HX + PIN_DX, TG_HY + sgn * PIN_DY])
    c = pin + (TA_L / 2) * u
    return Box(name, pos=(float(c[0]), float(c[1]), TG_Z0), size=(TA_L, TA_THK, TA_H), mass=TA_M, color=STEEL,
               yaw=float(yaw))


TONG_ARM_A = _tong_arm_spec("tong_arm_a", +1)
TONG_ARM_B = _tong_arm_spec("tong_arm_b", -1)
TONG_PARTS = (TONGS_HINGE.name, TONG_ARM_A.name, TONG_ARM_B.name)

OBJECTS = [GRIDDLE, PATTY, THERMO, TONGS_HINGE, TONG_ARM_A, TONG_ARM_B]

TWIN_EXTRA = [
    {"name": "silicone_mat", "kind": "decal", "size_mm": [440, 600, 1.5], "colour": "light blue-grey",
     "pose": [0.31, 0.0, TOP], "real": "60 x 44 cm food-safe silicone counter mat under griddle"},
    {"name": "tongs_spring_joints", "kind": "joint", "real": "12 in stainless spring tongs rear torsion spring",
     "note": "two symmetric revolute joints (0..13.5 deg half-angle) sprung open to 12.5 cm paddle clearance"},
    {"name": "probe_in_meat", "kind": "joint", "real": "cooked beef gripping and resisting the 3 mm probe shaft",
     "note": "sim: while tip is inside patty, a D6 joint applies 2.5 N-capped axial damping and 1500 N/m lateral springs"},
    {"name": "thermapen_lcd", "kind": "decal", "size_mm": [46, 28, 1], "colour": "green-grey LCD with 7-segment digits",
     "pose": [TH_X, TH_Y, TOP + TB_T], "real": "Thermapen ONE auto-waking digital temperature readout"},
]

# -- physics & scoring parameters (matching README.md and task.json 1:1) ----------------------------------------------
MU_GRIDDLE = 0.20       # oiled cast iron ("min" combine mode -> patty-griddle mu = 0.20)
MU_PATTY = 0.50         # patty base ("average" combine mode)
MU_TONGS = 0.95         # scalloped stainless jaws on crust ("max" combine mode -> tong-patty mu = 0.95)

INSERT_N = 2.5          # N: sliding/piercing resistance along the probe shaft
AX_DAMP = 15000.0       # N s/m along probe, clamped at INSERT_N (saturates at >= 0.17 mm/s; 0 N when held still)
LAT_K, LAT_C, LAT_MAX = 1500.0, 20.0, 25.0
ROT_K, ROT_C = 0.6, 0.02

TONG_REST_K = 2.5 * math.pi / 180.0   # N m/deg (2.5 N m/rad) unheld spring keeping jaws open at THETA0
TONG_HELD_K = 18.0 * math.pi / 180.0  # N m/deg (18.0 N m/rad) squeeze drive stiffness onto the patty
TONG_DAMP = 0.25 * math.pi / 180.0    # N m s/deg (0.25 N m s/rad)
TONG_MAX_TAU = 4.5                    # N m (~17.3 N max clamping force per paddle at d=0.26 m)

THERM_TAU = 0.40        # s: Thermapen ONE tip thermistor time constant (<=1 s to within 0.3 C)
TEMP_AIR_C = 22.0       # C: ambient room temperature
TEMP_CORE_C = 4.0       # C: refrigerated cooked patty centre temperature
TEMP_READ_MAX_C = 8.0   # C: valid stabilised chilled reading threshold (<= 8 C / 46 F)
TEMP_STAB_RATE = 1.5    # C/s: stabilised rate bound at the end of the 2 s hold

HAND_TOUCH = 0.006      # m: bare gripper < 6 mm from patty -> Score 0
TONG_CONTACT = 0.006    # m: tong jaw < 6 mm from patty required whenever probe is inside patty
SLIDE_MAX = 0.020       # m: patty centre horizontal slide > 2.0 cm -> Score 0
TILT_MAX = 30.0         # deg: patty tilt > 30 deg -> Score 0
RIM_ENTRY_MIN = 0.010   # m: >= 1.0 cm sideways rim insertion for 0.2 tier
CENTRE_R = 0.008        # m: tip within 8 mm horizontally of patty centre
CENTRE_H = 0.004        # m: tip within 4 mm vertically of mid-thickness (4..12 mm above griddle)
ANGLE_MAX = 20.0        # deg: probe shaft within 20 deg of horizontal
READ_STEPS = 40         # steps: 2.0 s continuous hold at 20 Hz
OUT_CLEAR = 0.020       # m: probe >= 2.0 cm clear of patty at end
TOOL_REST_Z = 0.030     # m: tool part centres within 3.0 cm of table top
TOOL_REST_V = 0.020     # m/s: tool linear speed < 2.0 cm/s
OPEN_APERTURE = 0.030   # m: both grippers open >= 3.0 cm
PARK_CLEAR = 0.100      # m: grippers >= 10.0 cm clear of griddle, patty, thermometer and tongs
PARK_SPEED = 0.020      # m/s: grippers moving < 2.0 cm/s
PARK_STEPS = 20         # steps: 1.0 s parked hold at 20 Hz

# 7-segment bitmask: (top, top-left, top-right, mid, bot-left, bot-right, bot)
SEG7 = {
    "0": (1, 1, 1, 0, 1, 1, 1),
    "1": (0, 0, 1, 0, 0, 1, 0),
    "2": (1, 0, 1, 1, 1, 0, 1),
    "3": (1, 0, 1, 1, 0, 1, 1),
    "4": (0, 1, 1, 1, 0, 1, 0),
    "5": (1, 1, 0, 1, 0, 1, 1),
    "6": (1, 1, 0, 1, 1, 1, 1),
    "7": (1, 0, 1, 0, 0, 1, 0),
    "8": (1, 1, 1, 1, 1, 1, 1),
    "9": (1, 1, 1, 1, 0, 1, 1),
    "-": (0, 0, 0, 1, 0, 0, 0),
    " ": (0, 0, 0, 0, 0, 0, 0),
}

_P: dict = {}


# -- quaternion & geometry helpers ------------------------------------------------------------------------------------
def _qrot(q_wxyz, v) -> np.ndarray:
    w, x, y, z = q_wxyz
    u = np.array([x, y, z], dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def _qmat(q_wxyz) -> np.ndarray:
    return np.stack([_qrot(q_wxyz, [1, 0, 0]), _qrot(q_wxyz, [0, 1, 0]), _qrot(q_wxyz, [0, 0, 1])], axis=1)


def _quat_from_R(R: np.ndarray) -> tuple[float, float, float, float]:
    from scipy.spatial.transform import Rotation

    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return float(w), float(x), float(y), float(z)


def _seg_dist(p, a, b) -> float:
    p, a, b = np.asarray(p, float), np.asarray(a, float), np.asarray(b, float)
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(float(np.dot(ab, ab)), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _point_cyl_dist(p, c, R_mat, radius: float, height: float) -> float:
    loc = R_mat.T @ (np.asarray(p, float) - np.asarray(c, float))
    dr = max(0.0, math.hypot(float(loc[0]), float(loc[1])) - radius)
    dz = max(0.0, abs(float(loc[2])) - height / 2.0)
    return math.hypot(dr, dz)


def _seg_cyl_dist(a, b, c, R_mat, radius: float, height: float, n: int = 17) -> float:
    a, b = np.asarray(a, float), np.asarray(b, float)
    return min(_point_cyl_dist(a + (b - a) * (i / (n - 1)), c, R_mat, radius, height) for i in range(n))


def _point_obb_dist(p, c, R_mat, size) -> float:
    loc = R_mat.T @ (np.asarray(p, float) - np.asarray(c, float))
    half = np.asarray(size, float) / 2.0
    return float(np.linalg.norm(np.maximum(0.0, np.abs(loc) - half)))


def _point_aabb_dist(p, lo, hi) -> float:
    p = np.asarray(p, float)
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(np.asarray(lo, float) - p, p - np.asarray(hi, float)))))


# -- USD scene authoring ----------------------------------------------------------------------------------------------
def _author(root: str) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    gr = f"{root}/{GRIDDLE.name}"
    pa = f"{root}/{PATTY.name}"
    th = f"{root}/{THERMO.name}"
    tgh = f"{root}/{TONGS_HINGE.name}"
    tga = f"{root}/{TONG_ARM_A.name}"
    tgb = f"{root}/{TONG_ARM_B.name}"
    _P.update(root=root, griddle=gr, patty=pa, thermo=th, tongs_hinge=tgh, tong_a=tga, tong_b=tgb, segs=[])

    def look(name, rgb, rough=0.45, metallic=0.1):
        path = f"{root}/Looks/pp_{name}"
        if not stage.GetPrimAtPath(path).IsValid():
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(rgb), roughness=rough, metallic=metallic)
            cfg.func(path, cfg)
        return path

    def part(parent, name, kind, pos, size=None, radius=None, height=None, axis="Z", yaw=0.0, rgb=STEEL,
             rough=0.45, metallic=0.1, collide=False):
        path = f"{parent}/{name}"
        if kind == "box":
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
            x = UsdGeom.Xformable(g)
            x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
            if yaw:
                x.AddRotateZOp().Set(float(math.degrees(yaw)))
            x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        else:
            g = UsdGeom.Cylinder.Define(stage, path)
            g.CreateRadiusAttr(float(radius))
            g.CreateHeightAttr(float(height))
            g.CreateAxisAttr(axis)
            x = UsdGeom.Xformable(g)
            x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
            if yaw:
                x.AddRotateZOp().Set(float(math.degrees(yaw)))
        bind_visual_material(path, look(name.rstrip("0123456789_"), rgb, rough, metallic), stage=stage)
        if collide:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        return path

    # 1. Wipeable silicone counter mat under griddle (visual) + griddle corner grips & patty start tick marks
    mat_xf = f"{root}/pp_decor"
    UsdGeom.Xform.Define(stage, mat_xf)
    part(mat_xf, "silicone_mat", "box", (0.31, 0.0, TOP + 0.00075), size=(0.44, 0.60, 0.0015), rgb=MAT_COL, rough=0.75)
    for sgn in (-1, 1):
        part(gr, f"ear_{'l' if sgn > 0 else 'r'}", "box", (0.0, sgn * (GR_L / 2 - 0.012), GR_T / 2 - 0.002),
             size=(0.085, 0.022, 0.005), rgb=IRON, rough=0.6, metallic=0.3)
    for k in range(8):
        ang = 2 * math.pi * k / 8
        rx = (P_X - GR_X) + (P_R + 0.004) * math.cos(ang)
        ry = (P_Y - GR_Y) + (P_R + 0.004) * math.sin(ang)
        part(gr, f"start_tick_{k}", "box", (rx, ry, GR_T / 2 + 0.0003), size=(0.005, 0.002, 0.0006), yaw=ang,
             rgb=RING_MARK, rough=0.6)

    # 2. Patty sear marks and 4 mm white food-safe centre dot on top face
    for k, dy in enumerate((-0.026, -0.009, 0.009, 0.026)):
        chord = 2.0 * math.sqrt(max(0.001, P_R * P_R - dy * dy)) * 0.82
        part(pa, f"sear_{k}", "box", (0.0, dy, P_T / 2 + 0.0003), size=(chord, 0.005, 0.0006), yaw=0.22,
             rgb=SEAR, rough=0.8)
    part(pa, "centre_dot", "cyl", (0.0, 0.0, P_T / 2 + 0.0005), radius=0.0020, height=0.0008, axis="Z", rgb=WHITE,
         rough=0.4)

    # 3. Thermapen ONE probe, tapered tip, 52.5 mm red depth band, pivot hub, backlit LCD + 2x 7-segment digits
    half = TB_L / 2
    shaft_main = PROBE_L - 0.015
    part(th, "probe_shaft", "cyl", (half + shaft_main / 2, 0.0, 0.0), radius=PROBE_R, height=shaft_main, axis="X",
         rgb=STEEL, rough=0.25, metallic=0.85, collide=True)
    part(th, "probe_tip", "cyl", (half + PROBE_L - 0.0075, 0.0, 0.0), radius=TIP_R, height=0.015, axis="X",
         rgb=STEEL, rough=0.25, metallic=0.85, collide=True)
    part(th, "depth_band", "cyl", (half + PROBE_L - MARK_FROM_TIP, 0.0, 0.0), radius=PROBE_R + 0.0008, height=0.003,
         axis="X", rgb=RED, rough=0.35)
    part(th, "probe_hub", "cyl", (half - 0.008, 0.0, 0.0), radius=0.011, height=TB_T + 0.001, axis="Z",
         rgb=DARK, rough=0.6)
    lcd_x, lcd_y = +0.022, 0.0
    part(th, "lcd_window", "box", (lcd_x, lcd_y, TB_T / 2 + 0.0004), size=(0.046, 0.028, 0.0008), rgb=LCD_BG, rough=0.3)
    part(th, "lcd_bezel", "box", (lcd_x, lcd_y, TB_T / 2 + 0.0002), size=(0.050, 0.032, 0.0006), rgb=DARK, rough=0.5)
    # 2 digits read along the body (+x = top of digits when viewed from right/top):
    seg_paths = []
    for d_idx, dy_c in enumerate((+0.0065, -0.0065)):  # tens at +y, ones at -y
        sx, sy, sthk, slen = lcd_x, dy_c, 0.0020, 0.0085
        coords = [
            (sx + 0.0085, sy, (sthk, slen, 0.0006)),          # 0: top
            (sx + 0.0043, sy + 0.0042, (slen, sthk, 0.0006)), # 1: top-left
            (sx + 0.0043, sy - 0.0042, (slen, sthk, 0.0006)), # 2: top-right
            (sx, sy, (sthk, slen, 0.0006)),                   # 3: mid
            (sx - 0.0043, sy + 0.0042, (slen, sthk, 0.0006)), # 4: bot-left
            (sx - 0.0043, sy - 0.0042, (slen, sthk, 0.0006)), # 5: bot-right
            (sx - 0.0085, sy, (sthk, slen, 0.0006)),          # 6: bot
        ]
        d_segs = []
        for s_i, (px, py, sz) in enumerate(coords):
            sp = part(th, f"seg_{d_idx}_{s_i}", "box", (px, py, TB_T / 2 + 0.0010), size=sz, rgb=LCD_INK, rough=0.4)
            d_segs.append(sp)
        seg_paths.append(d_segs)
    _P["segs"] = seg_paths

    # 4. Tongs details: rear spring loop, dark non-slip handle cushions, scalloped concave paddles (with collision)
    part(tgh, "spring_loop", "cyl", (-THG_L / 2, 0.0, 0.0), radius=0.012, height=THG_H, axis="Z",
         rgb=STEEL, rough=0.3, metallic=0.8)
    for sgn, arm_p, tag in ((+1, tga, "a"), (-1, tgb, "b")):
        part(arm_p, f"cushion_{tag}", "box", (-TA_L / 2 + TG_GRIP_D, sgn * 0.0028, 0.0),
             size=(0.044, 0.0060, TA_H - 0.002), yaw=-sgn * THETA0, rgb=DARK, rough=0.75, collide=True)
        # Scalloped jaw at tip, angled slightly inward (6 deg) to cradle the round patty rim:
        inward = -sgn * math.radians(6.0)
        part(arm_p, f"paddle_{tag}", "box", (TA_L / 2 - PAD_L / 2, -sgn * 0.002, (PAD_H - TA_H) / 2),
             size=(PAD_L, PAD_THK, PAD_H), yaw=inward, rgb=STEEL, rough=0.35, metallic=0.8, collide=True)
        for lobe_i, lx in enumerate((-0.015, 0.0, +0.015)):
            part(arm_p, f"lobe_{tag}_{lobe_i}", "cyl", (TA_L / 2 - PAD_L / 2 + lx, -sgn * 0.002, PAD_H - TA_H / 2),
                 radius=0.0075, height=PAD_THK, axis="Y", yaw=inward, rgb=STEEL, rough=0.35, metallic=0.8)

    # 5. Symmetric revolute joints for the spring tongs (stabilized with hub inertia & joint armature)
    hmass = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(tgh))
    hmass.CreateDiagonalInertiaAttr(Gf.Vec3f(0.004, 0.004, 0.004))
    jroot = f"{root}/pp_joints"
    UsdGeom.Xform.Define(stage, jroot)
    for sgn, arm_p, jname, lo_d, hi_d in ((+1, tga, "tong_joint_a", -THETA0_DEG, 1.0),
                                          (-1, tgb, "tong_joint_b", -1.0, +THETA0_DEG)):
        rj = UsdPhysics.RevoluteJoint.Define(stage, f"{jroot}/{jname}")
        rj.CreateBody0Rel().SetTargets([Sdf.Path(tgh)])
        rj.CreateBody1Rel().SetTargets([Sdf.Path(arm_p)])
        rj.CreateAxisAttr("Z")
        rj.CreateLocalPos0Attr(Gf.Vec3f(float(PIN_DX), float(sgn * PIN_DY), 0.0))
        half_a = 0.5 * sgn * THETA0
        rj.CreateLocalRot0Attr(Gf.Quatf(float(math.cos(half_a)), 0.0, 0.0, float(math.sin(half_a))))
        rj.CreateLocalPos1Attr(Gf.Vec3f(float(-TA_L / 2), 0.0, 0.0))
        rj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        rj.CreateLowerLimitAttr(float(lo_d))
        rj.CreateUpperLimitAttr(float(hi_d))
        rj.CreateExcludeFromArticulationAttr(True)
        PhysxSchema.PhysxJointAPI.Apply(rj.GetPrim()).CreateArmatureAttr(0.002)
        drv = UsdPhysics.DriveAPI.Apply(rj.GetPrim(), "angular")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(float(TONG_REST_K))
        drv.CreateDampingAttr(float(TONG_DAMP))
        drv.CreateMaxForceAttr(float(TONG_MAX_TAU))
        drv.CreateTargetPositionAttr(0.0)
        _P[jname] = str(rj.GetPath())

    # 6. Pre-authored disabled runtime joints (probe_in_meat + 4 grasp-weld slots) and static collision filters
    # Pre-authoring disabled joints & FilteredPairsAPI before PhysX parses the stage prevents runtime CreateJoint
    # snap warnings and ensures closed finger pads never fight welded tool handles or hinged tong arms.
    pj = UsdPhysics.Joint.Define(stage, f"{jroot}/probe_in_meat")
    pj.CreateBody0Rel().SetTargets([Sdf.Path(pa)])
    pj.CreateBody1Rel().SetTargets([Sdf.Path(th)])
    pj.CreateExcludeFromArticulationAttr(True)
    pj.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    pj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    pj.CreateLocalPos1Attr(Gf.Vec3f(*map(float, TIP_LOCAL)))
    pj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    pj.CreateJointEnabledAttr(False)
    pj_prim = pj.GetPrim()
    for dof, k_d, c_d, fmax in (
        ("transX", 0.0, AX_DAMP, INSERT_N),
        ("transY", LAT_K, LAT_C, LAT_MAX),
        ("transZ", LAT_K, LAT_C, LAT_MAX),
        ("rotY", ROT_K * math.pi / 180.0, ROT_C * math.pi / 180.0, 100.0),
        ("rotZ", ROT_K * math.pi / 180.0, ROT_C * math.pi / 180.0, 100.0),
    ):
        pdr = UsdPhysics.DriveAPI.Apply(pj_prim, dof)
        pdr.CreateTypeAttr("force")
        pdr.CreateStiffnessAttr(float(k_d))
        pdr.CreateDampingAttr(float(c_d))
        pdr.CreateMaxForceAttr(float(fmax))
        pdr.CreateTargetPositionAttr(0.0)
        pdr.CreateTargetVelocityAttr(0.0)
    _P["probe_joint"] = str(pj.GetPath())

    for side_cap in ("Left", "Right"):
        g_prim = f"{root}/{side_cap}Arm/Geometry/base/link1/link2/link3/link4/link5/gripper"
        for bname in (THERMO.name, TONGS_HINGE.name):
            gj = UsdPhysics.FixedJoint.Define(stage, f"{jroot}/grasp_{side_cap.lower()}_{bname}")
            gj.CreateBody0Rel().SetTargets([Sdf.Path(g_prim)])
            gj.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{bname}")])
            gj.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            gj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            gj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            gj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            gj.CreateExcludeFromArticulationAttr(True)
            gj.CreateJointEnabledAttr(False)

    # Only filter patty<->thermometer (probe shaft enters via D6 joint) and internal tong hinge self-pairs;
    # finger pads collide physically with both tools so grasp welds trigger strictly on two-pad contact force > 0.
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(th)).CreateFilteredPairsRel().SetTargets([Sdf.Path(pa)])
    for a_p, others in ((tgh, [tga, tgb]), (tga, [tgh, tgb]), (tgb, [tgh, tga])):
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a_p)).CreateFilteredPairsRel().SetTargets(
            [Sdf.Path(o_p) for o_p in others]
        )
    _P["cviews"] = None
    _P["cstage"] = None

    # 7. Physics materials: griddle ("min", 0.20), patty ("average", 0.50), tongs ("max", 0.95)
    def phys_mat(name, mu, mode):
        mp = f"{root}/pp_{name}"
        mc = sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu, friction_combine_mode=mode,
                                            restitution=0.0)
        mc.func(mp, mc)
        return mp

    m_gr = phys_mat("mat_griddle", MU_GRIDDLE, "min")
    m_pa = phys_mat("mat_patty", MU_PATTY, "average")
    m_tg = phys_mat("mat_tongs", MU_TONGS, "max")
    m_th = phys_mat("mat_thermo", 0.65, "average")
    for body_p, mp in ((gr, m_gr), (pa, m_pa), (tgh, m_tg), (tga, m_tg), (tgb, m_tg), (th, m_th)):
        for p in Usd.PrimRange(stage.GetPrimAtPath(body_p)):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                bind_physics_material(str(p.GetPath()), mp, stage=stage)

    _enable_contact_processing()
    for body_p in (pa, th, tgh, tga, tgb):
        prim_b = stage.GetPrimAtPath(body_p)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim_b)
        rb.CreateSolverPositionIterationCountAttr(16)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateLinearDampingAttr(0.4)
        rb.CreateAngularDampingAttr(0.8)
        cr = PhysxSchema.PhysxContactReportAPI.Apply(prim_b)
        cr.CreateThresholdAttr(0.0)
    for prim in stage.Traverse():
        ps = prim.GetPath().pathString
        if ps.startswith(root + "/") and ("/LeftArm/" in ps or "/RightArm/" in ps) and ps.endswith(("/tip_left", "/tip_right")):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    _set_lcd_text("--")


def _set_lcd_text(txt: str) -> None:
    segs = _P.get("segs")
    if not segs:
        return
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdGeom

        st = get_current_stage()
        chars = (txt + "  ")[:2]
        for d_idx, ch in enumerate(chars):
            bits = SEG7.get(ch, SEG7[" "])
            for s_i, bit in enumerate(bits):
                prim = st.GetPrimAtPath(segs[d_idx][s_i])
                if prim and prim.IsValid():
                    img = UsdGeom.Imageable(prim)
                    if bit:
                        img.MakeVisible()
                    else:
                        img.MakeInvisible()
    except Exception:  # noqa: BLE001
        pass


def _wrap_cfg(spec, obj):
    from isaaclab.sim.utils import clone

    cfg = obj.spawner_cfg
    if spec.name == OBJECTS[-1].name:
        orig = cfg.func
        base = getattr(orig, "__wrapped__", orig)

        def spawn(prim_path, c, *a, **k):
            prim = base(prim_path, c, *a, **k)
            _author(str(prim_path).rsplit("/", 1)[0])
            return prim

        cfg.func = clone(spawn)
    obj.object_cfg = obj._init_object_cfg()
    return obj


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_sideways_patty_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        obj = orig(spec)
        if getattr(spec, "name", None) == OBJECTS[-1].name:
            _wrap_cfg(spec, obj)
        return obj

    task_object._sideways_patty_patched = True
    scene.task_object = task_object


class _SceneFinder(importlib.abc.MetaPathFinder):
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
elif not any(isinstance(f, _SceneFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _SceneFinder())


# -- episode state & runtime physics hooks ----------------------------------------------------------------------------
_FRESH = dict(
    env=None, step=0, tracked_step=-1, nan=False,
    p0=None, th0=None, prev_loc=None,
    max_slide=0.0, max_tilt=0.0, off_griddle=False,
    touched_patty=False, min_hand_patty=1.0,
    no_tongs=False, bad_entry=False, through=False,
    joint_on=False, entry_rim=False, entry_angle_ok=False, entry_loc=None, max_rim_depth=0.0, depth=0.0,
    reached_centre=False, held_steps=0, max_held_steps=0, read_valid=False,
    awake=False, temp_c=TEMP_AIR_C, temp_rate=0.0, lcd_shown="--",
    tip_loc=None, angle_deg=0.0,
    weld_left=None, weld_right=None,
    ghist=None, pad_net_f=None, why="",
)
_M: dict = {}


def _reset_model() -> None:
    if "patty" in _P:
        _joint_off()
        _welds_off()
        _drive_tongs(None)
        _set_lcd_text("--")
    env = _M.get("env")
    _M.clear()
    _M.update(_FRESH)
    _M["ghist"] = []
    _M["pad_net_f"] = np.zeros(4, dtype=np.float64)
    _M["env"] = env


_reset_model()


def _fail(reason: str) -> None:
    if not _M["why"]:
        _M["why"] = reason
        print(f"[patty] FAIL at step {_M['step']}: {reason}", flush=True)


def _np(t) -> np.ndarray:
    t = getattr(t, "torch", t)
    if hasattr(t, "detach"):
        return np.asarray(t.detach().cpu().numpy(), dtype=np.float64)
    if hasattr(t, "numpy"):
        return np.asarray(t.numpy(), dtype=np.float64)
    return np.asarray(t, dtype=np.float64)


def _body(objects, name: str) -> tuple[np.ndarray, np.ndarray]:
    """Body centre (x, y, z) in env frame and orientation (w, x, y, z)."""
    env = _M.get("env")
    if env is not None:
        base = env.unwrapped
        pose = _np(base.arena_world.get_pose_w(name)[0]).reshape(-1)
        origin = _np(base.scene.env_origins[0]).reshape(-1)
        x, y, z, w = pose[3:7]
        return pose[:3] - origin, np.array([w, x, y, z], dtype=np.float64)
    o = objects[name]
    x, y, z, w = np.asarray(o.quat_xyzw, dtype=np.float64)
    return np.asarray(o.pos, dtype=np.float64), np.array([w, x, y, z], dtype=np.float64)


def _aperture(side: str, target: bool = False) -> float:
    """Total jaw opening (m) = |joint7| + |joint8| (0 closed .. ~0.094 fully open; 0.0 fail-closed if no env)."""
    env = _M.get("env")
    if env is None:
        return 0.0
    rob = env.unwrapped.scene[f"{side}_robot"]
    names = list(rob.joint_names)
    q = _np(rob.data.joint_pos_target[0] if target else rob.data.joint_pos[0]).reshape(-1)
    return abs(float(q[names.index("joint7")])) + abs(float(q[names.index("joint8")]))


def _gripper_points() -> dict[str, list[np.ndarray]]:
    """Measured world points (env frame) of left and right grippers: TCP, pad tips, finger pads, housing."""
    env = _M.get("env")
    if env is None:
        return {}
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    base = env.unwrapped
    origin = _np(base.scene.env_origins[0]).reshape(-1)
    out: dict[str, list[np.ndarray]] = {}
    for side in ("left", "right"):
        rob = base.scene[f"{side}_robot"]
        lp = link_poses(rob)
        gl = lp["gripper"]
        gR, gp = np.asarray(gl[:3, :3], float), np.asarray(gl[:3, 3], float) - origin
        half = 0.5 * _aperture(side, target=False)
        pts = [np.asarray(tcp_world(rob), float) - origin,
               np.asarray(pad_tips_world(rob), float) - origin,
               gp]
        for past in (0.0, PAD_TIP_PAST_TCP):
            c = gp + gR @ (np.array(TCP_OFFSET, float) + np.array([0.0, 0.0, past]))
            for s in (-1.0, 1.0):
                pts.append(c + s * half * gR[:, 1])
        for fn in ("tip_left", "tip_right"):
            if fn in lp:
                pts.append(np.asarray(lp[fn][:3, 3], float) - origin)
        out[side] = pts
    return out


def geom(objects=None) -> dict:
    """Current probe tip, shaft base, probe axis, patty frame, and tip coordinates in the patty frame."""
    tp, tq = _body(objects, THERMO.name)
    pp, pq = _body(objects, PATTY.name)
    tR, pR = _qmat(tq), _qmat(pq)
    tip = tp + tR @ TIP_LOCAL
    base_pt = tp + tR @ SHAFT_BASE_LOCAL
    ax = tR[:, 0]
    loc = pR.T @ (tip - pp)
    r = float(math.hypot(loc[0], loc[1]))
    inside = bool(r < P_R and abs(float(loc[2])) < P_T / 2.0)
    angle = math.degrees(math.asin(min(1.0, abs(float(ax[2])))))
    tilt = math.degrees(math.acos(float(np.clip(pR[2, 2], -1.0, 1.0))))
    return dict(tip=tip, base_pt=base_pt, ax=ax, tp=tp, tq=tq, tR=tR, pp=pp, pq=pq, pR=pR,
                loc=loc, r=r, h=float(loc[2]), inside=inside, angle=angle, tilt=tilt)


def tongs_grip_point(objects=None) -> tuple[np.ndarray, np.ndarray]:
    """Midpoint of the tongs' cushioned grip zone and tongs' forward unit vector in env frame."""
    hp, hq = _body(objects, TONGS_HINGE.name)
    hR = _qmat(hq)
    return hp + hR @ np.array([PIN_DX + TG_GRIP_D, 0.0, 0.0]), hR[:, 0]


def tongs_jaw_distances(objects, pp, pR) -> tuple[float, float]:
    """Distance (m) from each tong arm/scalloped paddle (A, B) to the patty's cylindrical side rim."""
    dists = []
    for sgn, arm_n in ((+1, TONG_ARM_A.name), (-1, TONG_ARM_B.name)):
        ap, aq = _body(objects, arm_n)
        aR = _qmat(aq)
        q_lo0 = ap + aR @ np.array([TA_L / 2 - PAD_L, -sgn * 0.002, -TA_H / 2.0])
        q_lo1 = ap + aR @ np.array([TA_L / 2, -sgn * 0.002, -TA_H / 2.0])
        loc_lo0 = pR.T @ (q_lo0 - pp)
        loc_lo1 = pR.T @ (q_lo1 - pp)
        if min(float(loc_lo0[2]), float(loc_lo1[2])) > P_T / 2.0 - 0.002:
            dists.append(1.0)
            continue
        p0 = ap + aR @ np.array([-TA_L / 2, 0.0, 0.0])
        p1 = ap + aR @ np.array([+TA_L / 2, -sgn * 0.002, 0.0])
        d_arm = max(0.0, _seg_cyl_dist(p0, p1, pp, pR, P_R, P_T, n=13) - PAD_THK / 2.0)
        for dz in (-TA_H / 2.0, PAD_H - TA_H / 2.0):
            q0 = ap + aR @ np.array([TA_L / 2 - PAD_L, -sgn * 0.002, dz])
            q1 = ap + aR @ np.array([TA_L / 2, -sgn * 0.002, dz])
            d_arm = min(d_arm, max(0.0, _seg_cyl_dist(q0, q1, pp, pR, P_R, P_T, n=7) - PAD_THK / 2.0))
        dists.append(float(d_arm))
    return dists[0], dists[1]


def tongs_patty_distance(objects, pp, pR) -> float:
    """Worst-case jaw clearance (m) across BOTH tong paddles (both jaws must clamp within 6 mm of the patty)."""
    d_a, d_b = tongs_jaw_distances(objects, pp, pR)
    return float(max(d_a, d_b))


def tongs_held_by_hand(gpts: dict[str, list[np.ndarray]], objects=None) -> bool:
    if _M.get("weld_left") == "tongs" or _M.get("weld_right") == "tongs":
        return True
    pnf = _M.get("pad_net_f")
    if pnf is not None:
        gp, _ = tongs_grip_point(objects)
        for side, (i_l, i_r) in (("left", (0, 1)), ("right", (2, 3))):
            if float(pnf[i_l]) >= 0.02 and float(pnf[i_r]) >= 0.02:
                pts = gpts.get(side, ())
                if any(float(np.linalg.norm(p - gp)) < 0.032 for p in pts):
                    return True
    return False


# -- probe-in-meat joint (sim hack) -----------------------------------------------------------------------------------
def _frame_with_x(d) -> np.ndarray:
    d = np.asarray(d, float) / np.linalg.norm(d)
    up = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    y = np.cross(up, d)
    y /= np.linalg.norm(y)
    return np.stack([d, y, np.cross(d, y)], axis=1)


def _crossing_rim(p_out: np.ndarray | None, p_in: np.ndarray) -> tuple[bool, np.ndarray]:
    """Segment check from outside (p_out) to inside (p_in) in patty frame: (crossed_side_rim, crossing_pt)."""
    p_in = np.asarray(p_in, dtype=np.float64)
    if p_out is None:
        rim = (P_R - math.hypot(float(p_in[0]), float(p_in[1]))) <= (P_T / 2.0 - abs(float(p_in[2])))
        return bool(rim), p_in.copy()
    p_out = np.asarray(p_out, dtype=np.float64)
    for t in np.linspace(0.0, 1.0, 201):
        p = p_out + t * (p_in - p_out)
        r = math.hypot(float(p[0]), float(p[1]))
        z = abs(float(p[2]))
        if r <= P_R and z <= P_T / 2.0:
            return bool((P_R - r) <= (P_T / 2.0 - z)), p.copy()
    return False, p_in.copy()


def _entered_via_rim(prev_loc: np.ndarray | None, cur_loc: np.ndarray) -> bool:
    """Exact segment intersection check in patty frame: True iff entry crossed cylindrical side wall r = P_R."""
    return _crossing_rim(prev_loc, cur_loc)[0]


def _joint_on(g: dict, rim: bool, entry_pt: np.ndarray) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf

    pR, tR = g["pR"], g["tR"]
    J0 = pR.T @ _frame_with_x(g["ax"])
    entry_local = g["loc"].copy()
    J1 = tR.T @ pR @ J0
    st = get_current_stage()
    j = st.GetPrimAtPath(_P["probe_joint"])
    if not (j and j.IsValid()):
        return
    j.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, entry_local)))
    j.GetAttribute("physics:localRot0").Set(Gf.Quatf(*_quat_from_R(J0)))
    j.GetAttribute("physics:localPos1").Set(Gf.Vec3f(*map(float, TIP_LOCAL)))
    j.GetAttribute("physics:localRot1").Set(Gf.Quatf(*_quat_from_R(J1)))
    j.GetAttribute("physics:jointEnabled").Set(True)
    _M["joint_on"] = True
    _M["entry_rim"] = bool(rim)
    _M["entry_angle_ok"] = bool(g["angle"] <= ANGLE_MAX)
    _M["entry_loc"] = np.asarray(entry_pt, dtype=np.float64).copy()
    ent, J0c = entry_local.copy(), J0.copy()
    _M["entry_w"] = lambda gg: gg["pp"] + gg["pR"] @ ent
    _M["dir_w"] = lambda gg: gg["pR"] @ J0c[:, 0]
    print(f"[patty] probe entered at step {_M['step']} via {'rim' if rim else 'TOP/BOTTOM'}: "
          f"loc={np.round(entry_local * 1000, 1)} mm angle={g['angle']:.1f} deg", flush=True)


def _joint_off() -> None:
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        j = get_current_stage().GetPrimAtPath(_P.get("probe_joint", ""))
        if j and j.IsValid():
            j.GetAttribute("physics:jointEnabled").Set(False)
    except Exception:  # noqa: BLE001
        pass
    _M["joint_on"] = False


# -- PhysX two-pad contact sensor & spring-tongs coupling -------------------------------------------------------------
PAD_CONTACT_MIN_N = 0.02  # N: minimum real PhysX contact force required on EACH finger pad to engage grasp weld
_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _enable_contact_processing() -> None:
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # pragma: no cover
        pass
    try:
        from isaaclab.app.settings_manager import get_settings_manager

        get_settings_manager().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # pragma: no cover
        pass


def _make_contact_view() -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab_physx.physics import PhysxManager
    from pxr import UsdPhysics

    if "thermo" not in _P or "root" not in _P:
        return
    sv = PhysxManager.get_physics_sim_view()
    if sv is None:
        return
    stage = get_current_stage()
    root = _P["root"]
    found: dict[tuple[str, str], str] = {}
    for prim in stage.Traverse():
        ps = prim.GetPath().pathString
        if not ps.startswith(root + "/") or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        for arm_tag in ("LeftArm", "RightArm"):
            if f"/{arm_tag}/" in ps:
                for tip_tag in ("tip_left", "tip_right"):
                    if ps.endswith(f"/{tip_tag}"):
                        found[(arm_tag, tip_tag)] = ps
    keys = [("LeftArm", "tip_left"), ("LeftArm", "tip_right"), ("RightArm", "tip_left"), ("RightArm", "tip_right")]
    if not all(k in found for k in keys):
        return
    # One RigidContactView per finger pad (1-to-many filter requirement in PhysX):
    f_globs = [f"{_P['thermo']}*", f"{_P['tongs_hinge']}*", f"{_P['tong_a']}*", f"{_P['tong_b']}*"]
    views = [sv.create_rigid_contact_view([found[k]], filter_patterns=[f_globs], max_contact_data_count=256)
             for k in keys]
    _P["cviews"] = views
    _P["cstage"] = stage
    print(f"[patty] initialized 4 finger-pad PhysX contact views (filters={len(f_globs)})", flush=True)


def _poll_pad_contacts(dt: float = 1.0 / 240.0) -> None:
    """Read PhysX contact forces on all 4 finger pads after each physics substep."""
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        st = get_current_stage()
        if _P.get("cviews") is None or _P.get("cstage") is not st:
            _make_contact_view()
        views = _P.get("cviews")
        if not views:
            return
        vals = np.zeros(4, dtype=np.float64)
        for idx, vw in enumerate(views):
            nf = _np(vw.get_net_contact_forces(dt=float(dt))).reshape(-1, 3)
            vals[idx] = float(np.linalg.norm(nf[0]))
        if _M.get("pad_net_f") is not None and np.all(np.isfinite(vals)):
            _M["pad_net_f"] = np.maximum(_M["pad_net_f"], vals)
    except Exception as e:  # pragma: no cover
        if not _M.get("cview_err"):
            _M["cview_err"] = True
            print(f"[patty] contact view error: {e!r}", flush=True)


def _patch_physx_mgr(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr, "_sideways_patty_contacts", False):
        return
    base_cfg = mgr._configure_physics.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        _enable_contact_processing()

    mgr._configure_physics = classmethod(_configure_physics)
    mgr.register_callback(_poll_pad_contacts, mod.IsaacEvents.POST_PHYSICS_STEP, wrap_weak_ref=False)
    mgr._sideways_patty_contacts = True


class _PhysxMgrFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != _PHYSX_MGR:
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
            _patch_physx_mgr(module)

        spec.loader.exec_module = exec_module
        return spec


if _PHYSX_MGR in sys.modules:
    _patch_physx_mgr(sys.modules[_PHYSX_MGR])
elif not any(isinstance(f, _PhysxMgrFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _PhysxMgrFinder())


def _drive_tongs(ap_cmd: float | None) -> None:
    """Update symmetric spring-tong revolute drives: sprung open at rest, or squeezed by commanded jaw closure."""
    if "tong_joint_a" not in _P:
        return
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        st = get_current_stage()
        if ap_cmd is None:
            delta_deg = 0.0
            k_val = TONG_REST_K
        else:
            # Map commanded jaw squeeze 66.5 mm (light carry, paddles 12.2 cm open) .. 59.5 mm (firm side-wall clamp)
            frac = float(np.clip((0.0665 - ap_cmd) / (0.0665 - 0.0595), 0.0, 1.0))
            delta_deg = frac * 8.5
            k_val = TONG_HELD_K if frac > 0.05 else TONG_REST_K
        ja = st.GetPrimAtPath(_P["tong_joint_a"])
        jb = st.GetPrimAtPath(_P["tong_joint_b"])
        if ja and ja.IsValid() and jb and jb.IsValid():
            ja.GetAttribute("drive:angular:physics:targetPosition").Set(float(-delta_deg))
            ja.GetAttribute("drive:angular:physics:stiffness").Set(float(k_val))
            jb.GetAttribute("drive:angular:physics:targetPosition").Set(float(+delta_deg))
            jb.GetAttribute("drive:angular:physics:stiffness").Set(float(k_val))
    except Exception:  # noqa: BLE001
        pass


def _weld_step(side: str) -> None:
    """Grasp-assist fixed constraint (stand-in for nitrile pad friction): engages ONLY when BOTH finger pads have
    positive PhysX contact force against the tool AND jaws are commanded closed; releases as soon as jaws open."""
    env = _M.get("env")
    if env is None or "thermo" not in _P:
        return
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf
    from yam_arena.runtime import link_poses

    st = get_current_stage()
    base = env.unwrapped
    rob = base.scene[f"{side}_robot"]
    ap_tgt = _aperture(side, target=True)
    ap_meas = _aperture(side, target=False)
    held = _M.get(f"weld_{side}")

    # Release weld when jaws open past tool handle width (thermometer 48 mm -> >=54 mm; tongs 67.8 mm -> >=74 mm)
    if held:
        rel_thresh = 0.074 if held == "tongs" else 0.054
        if ap_tgt >= rel_thresh or ap_tgt > ap_meas + 0.008:
            bname = TONGS_HINGE.name if held == "tongs" else THERMO.name
            jp = st.GetPrimAtPath(f"{_P['root']}/pp_joints/grasp_{side}_{bname}")
            if jp and jp.IsValid():
                jp.GetAttribute("physics:jointEnabled").Set(False)
            _M[f"weld_{side}"] = None
            if held == "tongs":
                _drive_tongs(None)
            print(f"[patty] {side} released {held} at step {_M['step']} (ap_tgt={ap_tgt * 1000:.1f} mm)", flush=True)
        elif held == "tongs":
            _drive_tongs(ap_tgt)
        return

    # Require positive PhysX net contact force (> 0.02 N) on BOTH finger pads simultaneously while jaws command squeeze
    _poll_pad_contacts(DT / 12.0)
    pnf = _M.get("pad_net_f")
    if pnf is None:
        return
    i_l, i_r = (0, 1) if side == "left" else (2, 3)
    fl, fr = float(pnf[i_l]), float(pnf[i_r])
    if fl < PAD_CONTACT_MIN_N or fr < PAD_CONTACT_MIN_N:
        return

    from yam_arena.runtime import pad_tips_world, tcp_world

    origin = _np(base.scene.env_origins[0]).reshape(-1)
    mid_pt = 0.5 * ((np.asarray(tcp_world(rob), float) - origin) + (np.asarray(pad_tips_world(rob), float) - origin))

    # Identify which pinched tool lies between the two contacting pads + verify jaws are commanded closed onto it
    tp, tq = _body(None, THERMO.name)
    th_loc = _qmat(tq).T @ (mid_pt - tp)
    on_thermo = (ap_tgt <= 0.046 and abs(float(th_loc[0])) <= TB_L / 2.0 and abs(float(th_loc[1])) <= 0.016
                 and abs(float(th_loc[2])) <= TB_T / 2.0 + 0.015
                 and (TB_W - 0.006) <= ap_meas <= (TB_W + 0.010))

    gp_tg, _ = tongs_grip_point(None)
    on_tongs = (ap_tgt <= 0.068 and float(np.linalg.norm(mid_pt - gp_tg)) <= 0.028 and 0.054 <= ap_meas <= 0.073)

    if on_thermo:
        label, bname = "thermometer", THERMO.name
    elif on_tongs:
        label, bname = "tongs", TONGS_HINGE.name
    else:
        return

    other = "right" if side == "left" else "left"
    if _M.get(f"weld_{other}") == label:
        return

    bp, bq = _body(None, bname)
    gl = link_poses(rob)["gripper"]
    g_pos, g_R = np.asarray(gl[:3, 3], float) - origin, np.asarray(gl[:3, :3], float)
    rel_p = g_R.T @ (bp - g_pos)
    rel_R = g_R.T @ _qmat(bq)
    j = st.GetPrimAtPath(f"{_P['root']}/pp_joints/grasp_{side}_{bname}")
    if not (j and j.IsValid()):
        return
    j.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, rel_p)))
    j.GetAttribute("physics:localRot0").Set(Gf.Quatf(*_quat_from_R(rel_R)))
    j.GetAttribute("physics:jointEnabled").Set(True)
    _M[f"weld_{side}"] = label
    if label == "tongs":
        _drive_tongs(ap_tgt)
    print(f"[patty] {side} grasped {label} on two-pad contact at step {_M['step']} "
          f"(F_L={fl:.2f} N, F_R={fr:.2f} N, ap={ap_meas * 1000:.1f} mm)", flush=True)


def _welds_off() -> None:
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        st = get_current_stage()
        for side in ("left", "right"):
            for bname in (THERMO.name, TONGS_HINGE.name):
                j = st.GetPrimAtPath(f"{_P.get('root')}/pp_joints/grasp_{side}_{bname}")
                if j and j.IsValid():
                    j.GetAttribute("physics:jointEnabled").Set(False)
    except Exception:  # noqa: BLE001
        pass


# -- per-step tracking (called once per control step) -----------------------------------------------------------------
def _step_track(objects=None) -> None:
    if _M.get("env") is None or _M["tracked_step"] == _M["step"]:
        return
    _M["tracked_step"] = _M["step"]
    try:
        g = geom(objects)
    except Exception:  # noqa: BLE001
        return

    if not (np.all(np.isfinite(g["tip"])) and np.all(np.isfinite(g["pp"]))):
        _M["nan"] = True
        _fail("non-finite physics state")
        return

    if _M["p0"] is None:
        _M["p0"] = g["pp"].copy()
        _M["th0"] = g["tp"].copy()

    slide = float(np.linalg.norm((g["pp"] - _M["p0"])[:2]))
    _M["max_slide"] = max(_M["max_slide"], slide)
    _M["max_tilt"] = max(_M["max_tilt"], g["tilt"])
    if slide > SLIDE_MAX:
        _fail(f"patty slid {slide * 1000:.1f} mm (> {SLIDE_MAX * 1000:.0f} mm)")
    if g["tilt"] > TILT_MAX:
        _fail(f"patty tilted {g['tilt']:.1f} deg (> {TILT_MAX:.0f} deg)")

    px, py, pz = float(g["pp"][0]), float(g["pp"][1]), float(g["pp"][2])
    if not (GR_X0 + P_R <= px <= GR_X1 - P_R and GR_Y0 + P_R <= py <= GR_Y1 - P_R) \
            or pz < TOP + GR_T - 0.004:
        _M["off_griddle"] = True
        _fail("patty left the griddle")

    # Bare-hand patty check + gripper history for parked check
    gpts = _gripper_points()
    all_gpts = [p for pts in gpts.values() for p in pts]
    if all_gpts:
        d_hand = min(_point_cyl_dist(p, g["pp"], g["pR"], P_R, P_T) for p in all_gpts)
        _M["min_hand_patty"] = min(_M["min_hand_patty"], d_hand)
        if d_hand < HAND_TOUCH and not _M["touched_patty"]:
            _M["touched_patty"] = True
            _fail(f"bare gripper touched patty ({d_hand * 1000:.1f} mm)")
        _M["ghist"].append((
            np.array(all_gpts, dtype=np.float64),
            _aperture("left", target=False),
            _aperture("right", target=False),
            bool(_M.get("weld_left") or _M.get("weld_right")),
        ))
        if len(_M["ghist"]) > PARK_STEPS + 1:
            _M["ghist"].pop(0)

    # Tong clamping contact with patty (both paddles within 6 mm while tongs are held)
    d_tongs = tongs_patty_distance(objects, g["pp"], g["pR"])
    tongs_on_patty = bool(d_tongs <= TONG_CONTACT and tongs_held_by_hand(gpts, objects))

    # Motion-wake & tip thermistor model on the Thermapen ONE
    if not _M["awake"] and float(np.linalg.norm(g["tp"] - _M["th0"])) > 0.005:
        _M["awake"] = True
        _M["temp_c"] = TEMP_AIR_C
    if _M["awake"]:
        if g["inside"]:
            r_frac = min(1.0, g["r"] / P_R)
            t_env = TEMP_CORE_C + 3.0 * (r_frac * r_frac)
        else:
            t_env = TEMP_AIR_C
        prev_t = _M["temp_c"]
        _M["temp_c"] = prev_t + (t_env - prev_t) * (1.0 - math.exp(-DT / THERM_TAU))
        _M["temp_rate"] = abs(_M["temp_c"] - prev_t) / DT
        disp = f"{int(round(_M['temp_c'])):2d}"
        if disp != _M["lcd_shown"]:
            _M["lcd_shown"] = disp
            _set_lcd_text(disp)

    # Probe insertion, shaft face check, withdrawal, and exit tracking
    if _M["joint_on"]:
        depth = float(np.dot(g["tip"] - _M["entry_w"](g), _M["dir_w"](g)))
        _M["depth"] = depth
        if g["inside"]:
            ax_loc = g["pR"].T @ g["ax"]
            for s_back in (0.015, 0.030, 0.045):
                q_sh = g["loc"] - s_back * ax_loc
                if math.hypot(float(q_sh[0]), float(q_sh[1])) < P_R - 0.0015 and abs(float(q_sh[2])) > P_T / 2.0 + 0.001:
                    if not _M["through"]:
                        _M["through"] = True
                        _fail("probe shaft cut through top/bottom face of patty")
                    break
        else:
            rim_exit, p_exit = _crossing_rim(g["loc"], _M["prev_loc"])
            ent_loc = _M["entry_loc"] if _M["entry_loc"] is not None else p_exit
            clean_backout = bool(rim_exit and depth <= 0.004
                                 and float(np.linalg.norm((p_exit - ent_loc)[:2])) <= 0.014)
            _joint_off()
            if clean_backout:
                print(f"[patty] probe withdrawn at step {_M['step']}", flush=True)
            elif not _M["through"]:
                _M["through"] = True
                _fail(f"probe exited patty outside entry hole at loc={np.round(g['loc'] * 1000, 1)} mm")
    elif g["inside"]:
        rim, entry_pt = _crossing_rim(_M["prev_loc"], g["loc"])
        _joint_on(g, rim, entry_pt)
        if not rim:
            _M["bad_entry"] = True
            _fail("probe entered through top/bottom face instead of side rim")

    # Enforce tong stabilization whenever probe tip is inside patty
    if (_M["joint_on"] or g["inside"]) and not tongs_on_patty:
        _M["no_tongs"] = True
        _fail(f"probe inside patty without held tongs clamping patty (max_jaw_d={d_tongs * 1000:.1f} mm)")

    _M["prev_loc"] = g["loc"].copy()
    _M["tip_loc"] = g["loc"].copy()
    _M["angle_deg"] = g["angle"]

    bad = bool(_M["nan"] or _M["touched_patty"] or _M["no_tongs"] or _M["bad_entry"] or _M["through"]
               or _M["off_griddle"] or _M["max_slide"] > SLIDE_MAX or _M["max_tilt"] > TILT_MAX)

    if (not bad and _M["joint_on"] and _M["entry_rim"] and _M["entry_angle_ok"]
            and g["inside"] and g["angle"] <= ANGLE_MAX):
        pen = P_R - g["r"]
        _M["max_rim_depth"] = max(_M["max_rim_depth"], pen)
        in_centre = bool(g["r"] <= CENTRE_R and abs(g["h"]) <= CENTRE_H)
        if in_centre:
            _M["reached_centre"] = True
            _M["held_steps"] += 1
            _M["max_held_steps"] = max(_M["max_held_steps"], _M["held_steps"])
            if (_M["held_steps"] >= READ_STEPS and _M["awake"] and _M["temp_c"] <= TEMP_READ_MAX_C
                    and _M["temp_rate"] <= TEMP_STAB_RATE and not _M["read_valid"]):
                _M["read_valid"] = True
                print(f"[patty] valid centre reading {_M['temp_c']:.1f} C held 2.0 s at step {_M['step']} "
                      f"(tip={np.round(g['loc'] * 1000, 1)} mm)", flush=True)
        else:
            _M["held_steps"] = 0
    else:
        _M["held_steps"] = 0


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_sideways_patty_reset", False):
        orig_reset = cls.reset

        def reset(self, *a, **k):
            _reset_model()
            obs = orig_reset(self, *a, **k)
            if getattr(getattr(self, "task", None), "module", None) is not None and \
                    getattr(self.task.module, "INSTRUCTION", None) == INSTRUCTION:
                _M["env"] = self._env
                _set_lcd_text("--")
                for _ in range(4):
                    self._base.sim.render()
                return self._observation()
            return obs

        reset._sideways_patty_reset = True
        cls.reset = reset

    if not getattr(cls._task_step, "_sideways_patty_step", False):
        orig_step = cls._task_step

        def _task_step(self, *a, **k):
            is_ours = (getattr(getattr(self, "task", None), "module", None) is not None and
                       getattr(self.task.module, "INSTRUCTION", None) == INSTRUCTION)
            if is_ours:
                _M["env"] = self._env
                _M["step"] += 1
                _M["pad_net_f"] = np.zeros(4, dtype=np.float64)
            out = orig_step(self, *a, **k)
            if is_ours:
                for side in ("left", "right"):
                    try:
                        _weld_step(side)
                    except Exception as e:  # noqa: BLE001
                        if not _M.get("weld_err"):
                            _M["weld_err"] = True
                            print(f"[patty] weld error: {e!r}", flush=True)
                _step_track(None)
            return out

        _task_step._sideways_patty_step = True
        cls._task_step = _task_step


_install_hooks()


# -- end-state checks & pure scoring logic ----------------------------------------------------------------------------
def _overlaps_griddle_xy(lo: np.ndarray, hi: np.ndarray) -> bool:
    return not (hi[0] < GR_X0 or lo[0] > GR_X1 or hi[1] < GR_Y0 or lo[1] > GR_Y1)


def _thermo_aabb(tp: np.ndarray, tR: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    corners = []
    for x in (-TB_L / 2.0, TB_L / 2.0 + PROBE_L):
        for y in (-TB_W / 2.0, TB_W / 2.0):
            for z in (-TB_T / 2.0, TB_T / 2.0):
                corners.append(tp + tR @ np.array([x, y, z]))
    arr = np.array(corners)
    return arr.min(axis=0), arr.max(axis=0)


def probe_withdrawn(objects) -> bool:
    if _M["joint_on"]:
        return False
    g = geom(objects)
    d = _seg_cyl_dist(g["base_pt"], g["tip"], g["pp"], g["pR"], P_R, P_T, n=17)
    return bool(d >= OUT_CLEAR)


def tools_down_off_griddle(objects) -> bool:
    if _M.get("weld_left") or _M.get("weld_right"):
        return False
    # Thermometer flat on table off griddle (LCD facing up, not dropped below table) and resting
    tp, tq = _body(objects, THERMO.name)
    tR = _qmat(tq)
    th_lo, th_hi = _thermo_aabb(tp, tR)
    th_o = objects[THERMO.name]
    if not (TOP <= tp[2] <= TOP + TOOL_REST_Z) or th_lo[2] < TOP - 0.005:
        return False
    if float(tR[2, 2]) < math.cos(math.radians(25.0)):
        return False
    if _overlaps_griddle_xy(th_lo, th_hi) or not th_o.resting(TOOL_REST_V, 0.5):
        return False
    # Tongs on table off griddle (not dropped below table) and resting
    for nm in TONG_PARTS:
        o = objects[nm]
        bp, _ = _body(objects, nm)
        lo, hi = np.asarray(o.low, float), np.asarray(o.high, float)
        if not (TOP <= bp[2] <= TOP + TOOL_REST_Z) or lo[2] < TOP - 0.005:
            return False
        if _overlaps_griddle_xy(lo, hi) or not o.resting(TOOL_REST_V, 0.5):
            return False
    return True


def arms_parked(objects) -> bool:
    if _M.get("env") is None or _M.get("weld_left") or _M.get("weld_right"):
        return False
    hist = _M["ghist"]
    if not hist or len(hist) < PARK_STEPS + 1:
        return False
    g = geom(objects)
    th_lo, th_hi = _thermo_aabb(g["tp"], g["tR"])
    gr_lo = np.array([GR_X0, GR_Y0, TOP])
    gr_hi = np.array([GR_X1, GR_Y1, TOP + GR_T])
    tg_boxes = [(np.asarray(objects[nm].low, float), np.asarray(objects[nm].high, float)) for nm in TONG_PARTS]

    prev_pts = None
    for k, (frame, ap_l, ap_r, welded) in enumerate(hist):
        if welded or ap_l < OPEN_APERTURE or ap_r < OPEN_APERTURE or len(frame) < 14:
            return False
        for p in frame:
            if _point_aabb_dist(p, gr_lo, gr_hi) < PARK_CLEAR:
                return False
            if _point_cyl_dist(p, g["pp"], g["pR"], P_R, P_T) < PARK_CLEAR:
                return False
            if _point_aabb_dist(p, th_lo, th_hi) < PARK_CLEAR:
                return False
            if any(_point_aabb_dist(p, lo, hi) < PARK_CLEAR for lo, hi in tg_boxes):
                return False
        if k > 0 and prev_pts is not None:
            vmax = float(np.linalg.norm(frame - prev_pts, axis=1).max()) * CONTROL_HZ
            if vmax >= PARK_SPEED:
                return False
        prev_pts = frame
    return True


def evaluate_flags(flags: dict, out_ok: bool, down_ok: bool, park_ok: bool) -> float:
    """Pure deterministic tier function matching README.md and task.json."""
    if (flags.get("nan") or flags.get("touched_patty") or flags.get("no_tongs") or flags.get("bad_entry")
            or flags.get("through") or flags.get("off_griddle")
            or flags.get("max_slide", 0.0) > SLIDE_MAX or flags.get("max_tilt", 0.0) > TILT_MAX):
        return 0.0
    if flags.get("read_valid"):
        if out_ok and down_ok and park_ok:
            return 1.0
        return 0.7
    if flags.get("reached_centre"):
        return 0.5
    if flags.get("max_rim_depth", 0.0) >= RIM_ENTRY_MIN:
        return 0.2
    return 0.0


def score(objects) -> float:
    if _M.get("env") is None:
        return 0.0
    _step_track(objects)
    out_ok = probe_withdrawn(objects) if _M["read_valid"] else False
    down_ok = tools_down_off_griddle(objects) if out_ok else False
    park_ok = arms_parked(objects) if down_ok else False
    return float(evaluate_flags(_M, out_ok, down_ok, park_ok))


def success(objects) -> bool:
    return score(objects) >= 1.0


def progress(objects) -> dict:
    _step_track(objects)
    loc = _M["tip_loc"] if _M["tip_loc"] is not None else np.zeros(3)
    out_ok = probe_withdrawn(objects) if _M.get("env") is not None else False
    down_ok = tools_down_off_griddle(objects) if _M.get("env") is not None else False
    park_ok = arms_parked(objects) if _M.get("env") is not None else False
    return {
        "rim_depth_mm": round(_M["max_rim_depth"] * 1000.0, 1),
        "reached_centre": float(_M["reached_centre"]),
        "held_s": round(_M["max_held_steps"] / CONTROL_HZ, 2),
        "temp_c": round(_M["temp_c"], 1),
        "read_valid": float(_M["read_valid"]),
        "probe_out": float(out_ok),
        "tools_down": float(down_ok),
        "parked": float(park_ok),
        "slide_mm": round(_M["max_slide"] * 1000.0, 1),
        "tip_mm": [round(float(v) * 1000.0, 1) for v in loc],
        "why": _M["why"],
    }


VARIATION = {
    "obj:patty": {"dx": 0.008, "dy": 0.008, "yaw": 0.30},
    "obj:thermometer": {"dx": 0.010, "dy": 0.010, "yaw": 0.08},
    "obj:tongs_hinge": {"dx": 0.010, "dy": 0.010, "yaw": 0.08},
}
