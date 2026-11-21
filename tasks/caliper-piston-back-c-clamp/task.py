"""On-vehicle brake pad service: with the front wheel removed, push a single-piston floating caliper's piston flush
into its bore using a 4 in C-clamp over the worn inner pad without cocking the piston or kinking or stretching the
attached flexible brake hose, then back off and remove the clamp.

Scene (metres from the arm-base midpoint, x forward, y left; heights above the table). Inside a rigid wheel-well
fender arch / inner liner stands a compact-car front corner (Honda Civic class): steering knuckle, MacPherson strut,
4x100 wheel hub with lug studs, and 262x21 mm vented brake rotor carrying the caliper bracket and outer pad. The
unbolted single-piston cast-iron caliper (54 mm piston, 1.80 kg; Cardone 19-B1833 ref) rests slot-up on a hardwood
support block on the lower control arm in front of the rotor, tethered from its inboard banjo fitting to the chassis
hardline bracket on the wheel-well liner by a 6-segment articulated EPDM rubber brake hose (12 mm OD, 18.5 cm). The
worn inner pad sits clipped on the piston, which stands 8.0 mm out of its bore. Outside the +y wheel arch, an IRWIN
225104 4 in C-clamp (0.51 kg, 73 mm jaw opening, 3 in throat, 9/16-12 screw, sliding T-pin) hangs on a tool rack.

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
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = (
    "Push the brake caliper's piston flush into its bore with the C-clamp over the worn pad without cocking the "
    "piston or kinking or stretching the brake hose, then remove the clamp."
)

CAL_COL = (0.30, 0.31, 0.33)  # cast-iron caliper housing
PISTON_COL = (0.88, 0.90, 0.93)  # machined chrome-steel piston
PAD_COL = (0.82, 0.22, 0.16)  # red anti-squeal shimmed steel backing plate
LINING_COL = (0.38, 0.33, 0.28)  # worn friction material
CLAMP_COL = (0.10, 0.28, 0.62)  # ductile-iron blue C-clamp frame
STEEL = (0.72, 0.73, 0.75)
ROTOR_COL = (0.58, 0.60, 0.63)
HUB_COL = (0.44, 0.45, 0.48)
BRASS = (0.78, 0.64, 0.26)
WOOD = (0.80, 0.66, 0.45)
ARM_COL = (0.19, 0.19, 0.20)
LINER_COL = (0.14, 0.14, 0.15)  # molded polypropylene wheel-well liner
FENDER_COL = (0.75, 0.77, 0.80)  # painted fender arch lip
HOSE_COL = (0.10, 0.10, 0.11)  # SAE J1401 EPDM rubber brake hose
RACK_COL = (0.35, 0.36, 0.38)

# -- caliper & piston (world frame; rests free on support block on lower control arm inside wheel well) ----------------
YC = -0.015  # bore axis y (-1.5 cm)
ZA = 0.148  # bore axis height above table (14.8 cm)
X_BACK = 0.365  # caliper housing back face (where C-clamp swivel pad bears)
X_FACE = 0.415  # caliper bore face (piston protrudes toward +x)
HOUSE_Y, HOUSE_Z = 0.116, 0.068  # inboard housing y and z extents (7 mm wall around 54 mm bore)
OUT0 = 0.008  # piston protrusion at start (8.0 mm)
PIST_R, PIST_L = 0.027, 0.030  # 54 mm diameter x 30 mm steel piston
PLATE_T, LINING_T = 0.006, 0.003  # 6 mm steel backing plate + 3 mm worn lining
PAD_Y, PAD_Z = 0.110, 0.054  # backing plate 11.0 x 5.4 cm (top 3 mm below housing top, visible from top_cam)
X_FING0, X_FING1 = 0.466, 0.482  # outboard caliper fingers
BRIDGE_Y, BRIDGE_TOP = 0.060, ZA - 0.029  # lower bridge under disc slot (open top inspection window)
CAL_BOTTOM = ZA - HOUSE_Z / 2  # 0.114 m above table
CAL_M, PIST_M = 1.80, 0.25
X_LINING = X_FACE + OUT0 + PLATE_T + LINING_T  # 0.432 m at start (span X_LINING - X_BACK = 67 mm)

CALIPER = Box("caliper", pos=((X_BACK + X_FACE) / 2, YC, TOP + ZA), size=(X_FACE - X_BACK, HOUSE_Y, HOUSE_Z),
              mass=CAL_M, color=CAL_COL)
_PX0 = X_FACE + OUT0 - PIST_L / 2
PISTON = Cylinder("piston", pos=(_PX0, YC, TOP + ZA), radius=PIST_R, height=PIST_L, mass=PIST_M, color=PISTON_COL)

# -- on-vehicle corner: lower control arm, support block, hub + 262 mm rotor, knuckle/strut, wheel-well arch ----------
BLOCK_H = 0.040
ARM_TOP = CAL_BOTTOM - BLOCK_H  # 0.074 m
CTRL_ARM = Box("control_arm", pos=(0.465, -0.045, TOP + ARM_TOP / 2), size=(0.080, 0.240, ARM_TOP), color=ARM_COL)
BLOCK = Box("wood_block", pos=(0.425, YC, TOP + ARM_TOP + BLOCK_H / 2), size=(0.090, 0.070, BLOCK_H), color=WOOD)

ROTOR_POS = (0.548, -0.075, 0.168)  # 262 mm diameter x 21 mm vented front brake disc, axis along x
ROTOR_R, ROTOR_T = 0.131, 0.021
ROTOR = Cylinder("brake_rotor", pos=(ROTOR_POS[0], ROTOR_POS[1], TOP + ROTOR_POS[2]), radius=ROTOR_R, height=ROTOR_T,
                 color=ROTOR_COL)
KNUCKLE = Box("steering_knuckle", pos=(0.585, -0.075, TOP + 0.140), size=(0.052, 0.105, 0.200), color=ARM_COL)

# rigid static wheel-well inner fender liner & arch framing the brake corner
LINER_BACK = Box("liner_back", pos=(0.625, -0.065, TOP + 0.165), size=(0.020, 0.360, 0.330), color=LINER_COL)
LINER_RIGHT_F = Box("liner_right_front", pos=(0.405, -0.235, TOP + 0.090), size=(0.110, 0.020, 0.180), color=LINER_COL)
LINER_RIGHT_R = Box("liner_right_rear", pos=(0.540, -0.235, TOP + 0.155), size=(0.160, 0.020, 0.310), color=LINER_COL)
LINER_LEFT_F = Box("liner_left_front", pos=(0.425, 0.105, TOP + 0.080), size=(0.100, 0.018, 0.160), color=LINER_COL)
LINER_LEFT_R = Box("liner_left_rear", pos=(0.550, 0.105, TOP + 0.155), size=(0.150, 0.018, 0.310), color=LINER_COL)
LINER_HOOD = Box("liner_hood", pos=(0.580, -0.065, TOP + 0.320), size=(0.090, 0.358, 0.020), color=FENDER_COL)

# -- flexible hydraulic brake hose (6 articulated EPDM rubber capsules from caliper banjo port to chassis bracket) ----
HOSE_N = 6
HOSE_R = 0.006  # 12 mm OD SAE J1401 flexible rubber brake hose
HOSE_MASS = 0.072  # kg total across 6 segments
BANJO_WORLD0 = np.array([0.390, YC - HOUSE_Y / 2 - 0.008, TOP + ZA + 0.008])  # (0.390, -0.081, TOP + 0.156)
BRACKET_WORLD = np.array([0.490, -0.212, TOP + 0.205])  # fixed steel bracket on right wheel-well liner


def _hose_nodes() -> np.ndarray:
    """Smooth 3D cubic Bezier curve (7 nodes for 6 capsule segments) with natural downward/outward service loop."""
    p0 = BANJO_WORLD0
    p1 = np.array([0.402, -0.132, TOP + 0.132])
    p2 = np.array([0.452, -0.188, TOP + 0.158])
    p3 = BRACKET_WORLD
    ts = np.linspace(0.0, 1.0, HOSE_N + 1)[:, None]
    return (1 - ts) ** 3 * p0 + 3 * (1 - ts) ** 2 * ts * p1 + 3 * (1 - ts) * ts ** 2 * p2 + ts ** 3 * p3


_HOSE_PTS = _hose_nodes()
_HOSE_SEGS = [(_HOSE_PTS[i], _HOSE_PTS[i + 1]) for i in range(HOSE_N)]
HOSE_ARC = float(sum(np.linalg.norm(b - a) for a, b in _HOSE_SEGS))  # ~0.185 m hose length
HOSE_REST_SPAN = float(np.linalg.norm(BRACKET_WORLD - BANJO_WORLD0))  # ~0.172 m chord at start
HOSE_MAX_SPAN = 0.180  # m: score 0 if banjo-to-bracket chord pulled taut past 18.0 cm (17.2 cm rest, 18.5 cm hose)
HOSE_MAX_BEND_DEG = 55.0  # deg: score 0 if any inter-segment bend exceeds 55 deg (kinked < ~3.8 cm radius)
HOSE_STIFF = 0.20  # N m/rad per joint bend axis
HOSE_DAMP = 0.03  # N m s/rad

HOSE_OBJS = [
    Cylinder(
        f"hose_{i}",
        pos=tuple(((a + b) / 2).tolist()),
        radius=HOSE_R,
        height=float(np.linalg.norm(b - a)),
        mass=HOSE_MASS / HOSE_N,
        color=HOSE_COL,
    )
    for i, (a, b) in enumerate(_HOSE_SEGS)
]
HOSE_NAMES = tuple(o.name for o in HOSE_OBJS)

# -- C-clamp (IRWIN Quick-Grip 225104 4 in ductile-iron C-clamp; clamp-local x along screw, z up spine) ---------------
BOSS = (0.020, 0.022, 0.024)
SPINE_W = 0.018  # 18 mm wide vertical I-beam grip web (parallel jaws close firmly along y with 45-50 N normal force)
FLANGE_W, FLANGE_T = 0.026, 0.004  # 26 mm cast lower I-beam flange under the grip web
THROAT = 0.076  # 3 in throat depth (screw axis to top bar inner face)
TOPBAR_T = 0.022  # 22 mm tall top bar web
ANVIL_X0, ANVIL_X1 = 0.118, 0.130  # fixed jaw face at 11.8 cm -> initial opening ANVIL_X0 - E0 = 73 mm (6 mm margin)
ANVIL_W, ANVIL_LOW = 0.020, -0.029
TIP_R, TIP_T = 0.011, 0.006  # 22 mm swivel pad
SCREW_R = 0.00714  # 9/16 in
HEAD_R, HEAD_X0, HEAD_X1 = 0.009, -0.031, -0.010
PIN_X, PIN_R, PIN_L = -0.022, 0.0045, 0.102  # 10.2 cm x 9 mm sliding steel T-pin
PIN_GRIP = 0.036  # radial grip distance on T-pin from screw axis
LEAD = 0.0254 / 12  # 9/16-12 UNC: 2.117 mm per full turn
CLAMP_M, HANDLE_M, TIP_M = 0.40, 0.06, 0.05
E0 = 0.045  # swivel pad face ahead of boss centre at start (jaw opening 73 mm)
E_MAX = 0.102  # fully closed
TOPBAR_X0, TOPBAR_X1 = -BOSS[0] / 2, ANVIL_X1
GRIP_X = 0.072  # ribbed grip flat centre on top bar (near clamp CoM)

# tool rack outside the +y wheel-well arch
ROD_R, ROD_L = 0.005, 0.095
CL0 = (0.335, 0.170, 0.148)  # clamp boss centre at start (x, y, height above table)


def _clamp_com_x() -> float:
    parts = [
        ((TOPBAR_X0 + TOPBAR_X1) / 2, (TOPBAR_X1 - TOPBAR_X0) * SPINE_W * TOPBAR_T),
        (0.0, BOSS[0] * BOSS[1] * BOSS[2]),
        (-BOSS[0] / 2 + SPINE_W / 2, SPINE_W * SPINE_W * (THROAT - BOSS[2] / 2)),
        ((ANVIL_X0 + ANVIL_X1) / 2, (ANVIL_X1 - ANVIL_X0) * ANVIL_W * (THROAT - ANVIL_LOW)),
    ]
    xf = sum(x * v for x, v in parts) / sum(v for _, v in parts)
    return (xf * CLAMP_M + (HEAD_X0 + HEAD_X1) / 2 * HANDLE_M + E0 / 2 * TIP_M) / (CLAMP_M + HANDLE_M + TIP_M)


ROD_X = 0.102  # rack rod x in clamp frame (inside throat ahead of CoM)
assert 0.0 < _clamp_com_x() < ROD_X
assert HOSE_MAX_SPAN < HOSE_ARC
ROD = (CL0[0] + ROD_X, CL0[1], CL0[2] + THROAT - ROD_R)
REST_TOP = CL0[2] - BOSS[2] / 2

CLAMP = Box("c_clamp", pos=(CL0[0] + (TOPBAR_X0 + TOPBAR_X1) / 2, CL0[1], TOP + CL0[2] + THROAT + TOPBAR_T / 2),
            size=(TOPBAR_X1 - TOPBAR_X0, SPINE_W, TOPBAR_T), mass=CLAMP_M, color=CLAMP_COL)
HANDLE = Cylinder("t_handle", pos=(CL0[0] + (HEAD_X0 + HEAD_X1) / 2, CL0[1], TOP + CL0[2]), radius=HEAD_R,
                  height=HEAD_X1 - HEAD_X0, mass=HANDLE_M, color=STEEL)
_SHANK_L = 0.090
SCREW = Cylinder("clamp_screw", pos=(CL0[0] + E0 - _SHANK_L / 2, CL0[1], TOP + CL0[2]), radius=SCREW_R,
                 height=_SHANK_L, mass=TIP_M, color=STEEL)
ROD_OBJ = Cylinder("rack_rod", pos=(ROD[0], ROD[1], TOP + ROD[2]), radius=ROD_R, height=ROD_L, color=RACK_COL)
RACK_POST = Box("rack_post", pos=(ROD[0], ROD[1] + ROD_L / 2 + 0.010, TOP + (ROD[2] + 0.020) / 2),
                size=(0.016, 0.020, ROD[2] + 0.020), color=RACK_COL)
REST = Box("clamp_rest", pos=(CL0[0] + 0.001, CL0[1], TOP + REST_TOP / 2), size=(0.018, 0.040, REST_TOP), color=WOOD)

OBJECTS = [
    CTRL_ARM, BLOCK, ROTOR, KNUCKLE, LINER_BACK, LINER_RIGHT_F, LINER_RIGHT_R, LINER_LEFT_F, LINER_LEFT_R, LINER_HOOD,
    ROD_OBJ, RACK_POST, REST, *HOSE_OBJS, CALIPER, PISTON, CLAMP, HANDLE, SCREW,
]
_AXIS_X = {PISTON.name, HANDLE.name, SCREW.name, ROTOR.name}
_AXIS_Y = {ROD_OBJ.name}

TWIN_EXTRA = [
    {"name": "piston_in_bore", "kind": "joint",
     "real": "Cardone 19-B1833 54 mm piston + square-cut EPDM seal + cracked bleeder tube into catch bottle",
     "note": "sim: prismatic along bore with 150 N elasto-plastic seal slip + 6e4 N s/m viscous drag"},
    {"name": "clamp_screw", "kind": "joint", "real": "9/16-12 UNC screw in IRWIN 225104 C-clamp boss",
     "note": "sim: handle revolute + screw prismatic tracking handle turn at 2.117 mm/turn, self-locking"},
    {"name": "brake_hose_joints", "kind": "joint",
     "real": "SAE J1401 12 mm OD EPDM flexible brake hose from caliper banjo bolt to liner bracket",
     "note": "sim: 6 capsule segments joined by 2-axis bend springs (0.20 N m/rad)"},
]

PISTON_K = 1.0e6  # N/m elastic shear stiffness of square-cut seal (slips at PISTON_SEAL = 150 N -> 0.15 mm)
PISTON_DAMP = 6.0e4  # N s/m viscous fluid backflow drag through cracked bleeder tube
PISTON_SEAL = 150.0  # N static/sliding friction of square-cut EPDM bore seal
HANDLE_DAMP = 0.03  # N m s/rad unloaded screw/swivel drag
THREAD_T0 = 0.03  # N m unloaded thread torque
THREAD_EFF = 0.30  # lubricated steel 9/16-12 thread efficiency
SCREW_K, SCREW_C = 4.0e5, 2.0e3  # screw thread drive (N/m, N s/m)
GRIP_MU = (2.8, 2.4)  # high-tack elastomer pad / ribbed grip friction (pure contact grasp, no weld)

_P: dict = {}


# -- authoring: detailed caliper/hub/rotor/strut/clamp geometry, joints, hose chain, and contact materials ------------
def _author(root: str) -> None:
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    cal, pis, clp, han, scr = (f"{root}/{o.name}" for o in (CALIPER, PISTON, CLAMP, HANDLE, SCREW))
    rot_p, knu_p, lrf_p = (f"{root}/{o.name}" for o in (ROTOR, KNUCKLE, LINER_RIGHT_R))
    hose_ps = [f"{root}/{n}" for n in HOSE_NAMES]
    _P.update(caliper=cal, piston=pis, clamp=clp, handle=han, screw=scr, root=root, hose=hose_ps)

    def look(name, rgb, rough=0.45, metal=0.2):
        path = f"{root}/Looks/{name}"
        if not stage.GetPrimAtPath(path).IsValid():
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(rgb), roughness=rough, metallic=metal)
            cfg.func(path, cfg)
        return path

    def part(parent, name, kind, pos, size=None, radius=None, height=None, axis="Z", rgb=STEEL, collide=False,
             rot_wxyz=None):
        path = f"{parent}/{name}"
        if kind == "box":
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(g)
            xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
            if rot_wxyz is not None:
                xf.AddOrientOp().Set(Gf.Quatf(*rot_wxyz))
            xf.AddScaleOp().Set(Gf.Vec3f(*size))
        else:
            g = UsdGeom.Cylinder.Define(stage, path)
            g.CreateRadiusAttr(radius)
            g.CreateHeightAttr(height)
            g.CreateAxisAttr(axis)
            xf = UsdGeom.Xformable(g)
            xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
            if rot_wxyz is not None:
                xf.AddOrientOp().Set(Gf.Quatf(*rot_wxyz))
        bind_visual_material(path, look(f"{name}_{ int(rgb[0]*100)}_{int(rgb[1]*100)}", rgb), stage=stage)
        if collide:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        return path

    def box_lo_hi(parent, name, c0, lo, hi, rgb, collide=True):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        ctr = (lo + hi) / 2 - np.asarray(c0, float)
        return part(parent, name, "box", tuple(ctr), size=tuple(hi - lo), rgb=rgb, collide=collide)

    z = lambda h: TOP + h  # noqa: E731

    # 1. Caliper details: lower bridge, side frame ribs (leaving wide top inspection window), outboard fingers, ears,
    #    bore rim, bleeder screw + cap, and brass brake-hose banjo block on -y side
    c0 = np.array(CALIPER.pos)
    box_lo_hi(cal, "bridge_low", c0, (X_FACE - 0.002, YC - BRIDGE_Y / 2, z(CAL_BOTTOM)),
              (X_FING1, YC + BRIDGE_Y / 2, z(BRIDGE_TOP)), CAL_COL)
    for s, tag in ((1, "l"), (-1, "r")):
        yc_rib = YC + s * 0.046
        box_lo_hi(cal, f"side_rib_{tag}", c0, (X_FACE - 0.002, yc_rib - 0.008, z(ZA - 0.012)),
                  (X_FING1, yc_rib + 0.008, z(ZA + 0.012)), CAL_COL)
        ye = YC + s * HOUSE_Y / 2
        box_lo_hi(cal, f"ear_{tag}", c0, (X_BACK + 0.008, min(ye, ye + s * 0.016), z(ZA - 0.011)),
                  (X_BACK + 0.032, max(ye, ye + s * 0.016), z(ZA + 0.011)), CAL_COL)
    box_lo_hi(cal, "fingers", c0, (X_FING0, YC - 0.045, z(CAL_BOTTOM)), (X_FING1, YC + 0.045, z(ZA + 0.026)), CAL_COL)
    part(cal, "bore_rim", "cyl", (X_FACE - c0[0] + 0.0008, 0.0, 0.0), radius=PIST_R + 0.004, height=0.0016, axis="X",
         rgb=(0.18, 0.18, 0.19))
    part(cal, "bleeder", "cyl", (0.380 - c0[0], 0.040, HOUSE_Z / 2 + 0.007), radius=0.0045, height=0.014, axis="Z",
         rgb=STEEL)
    box_lo_hi(cal, "banjo_block", c0, (0.382, YC - HOUSE_Y / 2 - 0.014, z(ZA + 0.001)),
              (0.398, YC - HOUSE_Y / 2, z(ZA + 0.016)), BRASS, collide=True)
    part(cal, "banjo_bolt", "cyl", (0.390 - c0[0], -HOUSE_Y / 2 - 0.016, 0.008), radius=0.006, height=0.006, axis="Y",
         rgb=STEEL)

    # 2. Piston + clipped worn inner pad (red shimmed backing plate + ears + dark friction lining)
    p0 = np.array(PISTON.pos)
    xf = X_FACE + OUT0
    box_lo_hi(pis, "pad_plate", p0, (xf, YC - PAD_Y / 2, z(ZA - 0.024)), (xf + PLATE_T, YC + PAD_Y / 2, z(ZA + 0.030)),
              PAD_COL)
    box_lo_hi(pis, "pad_lining", p0, (xf + PLATE_T, YC - PAD_Y / 2 + 0.004, z(ZA - 0.021)),
              (xf + PLATE_T + LINING_T, YC + PAD_Y / 2 - 0.004, z(ZA + 0.027)), LINING_COL)

    # 3. Hub flange + 4x100 wheel lug studs + mounted caliper bracket + outer pad on the 262 mm rotor & strut on knuckle
    r0 = np.array(ROTOR.pos)
    part(rot_p, "hub_hat", "cyl", (-ROTOR_T / 2 - 0.013, 0.0, 0.0), radius=0.068, height=0.026, axis="X", rgb=HUB_COL,
         collide=True)
    part(rot_p, "spindle_nut", "cyl", (-ROTOR_T / 2 - 0.031, 0.0, 0.0), radius=0.019, height=0.010, axis="X", rgb=STEEL)
    for k in range(4):
        ang = math.pi / 4 + k * math.pi / 2
        part(rot_p, f"stud_{k}", "cyl", (-ROTOR_T / 2 - 0.036, 0.050 * math.cos(ang), 0.050 * math.sin(ang)),
             radius=0.006, height=0.020, axis="X", rgb=STEEL, collide=True)
    # caliper bracket + outer brake pad still mounted on upper-inboard rim of the rotor
    box_lo_hi(rot_p, "cal_bracket", r0, (ROTOR_POS[0] - 0.022, -0.065, z(0.245)),
              (ROTOR_POS[0] + 0.022, 0.025, z(0.292)), ARM_COL, collide=True)
    box_lo_hi(rot_p, "outer_pad", r0, (ROTOR_POS[0] - ROTOR_T / 2 - 0.010, -0.055, z(0.240)),
              (ROTOR_POS[0] - ROTOR_T / 2 - 0.001, 0.020, z(0.284)), PAD_COL, collide=False)

    kn0 = np.array(KNUCKLE.pos)
    part(knu_p, "strut_tube", "cyl", (0.005, -0.015, 0.115), radius=0.027, height=0.160, axis="Z", rgb=ARM_COL,
         collide=True)
    part(knu_p, "strut_spring", "cyl", (0.005, -0.015, 0.155), radius=0.052, height=0.085, axis="Z",
         rgb=(0.22, 0.23, 0.25), collide=False)

    # chassis hardline bracket + steel brake tube on right wheel-well liner
    lr0 = np.array(LINER_RIGHT_R.pos)
    box_lo_hi(lrf_p, "hose_bracket", lr0, (0.478, -0.225, z(0.195)), (0.504, -0.202, z(0.220)), STEEL, collide=False)
    part(lrf_p, "hard_line", "cyl", (0.490 - lr0[0], 0.018, 0.095), radius=0.003, height=0.095, axis="Z", rgb=STEEL)
    # fender arch painted outer trim strips
    box_lo_hi(f"{root}/{LINER_LEFT_F.name}", "trim_lf", np.array(LINER_LEFT_F.pos),
              (0.375, 0.096, z(0.154)), (0.475, 0.116, z(0.164)), FENDER_COL, collide=False)
    box_lo_hi(f"{root}/{LINER_RIGHT_F.name}", "trim_rf", np.array(LINER_RIGHT_F.pos),
              (0.350, -0.246, z(0.174)), (0.460, -0.224, z(0.184)), FENDER_COL, collide=False)

    # 4. C-clamp frame: lower I-beam flange under top bar (supports parallel-jaw contact grip), boss, spine, anvil,
    #    sliding T-pin on handle, and swivel pad on screw
    k0 = np.array(CLAMP.pos)
    bx, by, bz = CL0[0], CL0[1], TOP + CL0[2]
    # lower I-beam flange along the grip saddle (x = 0.032..0.106 in clamp frame) + shoulder keeper ribs
    box_lo_hi(clp, "grip_flange", k0, (bx + 0.032, by - FLANGE_W / 2, bz + THROAT),
              (bx + 0.106, by + FLANGE_W / 2, bz + THROAT + FLANGE_T), CLAMP_COL, collide=True)
    for sx, nm in ((0.036, "rib_back"), (0.102, "rib_front")):
        box_lo_hi(clp, nm, k0, (bx + sx - 0.004, by - FLANGE_W / 2 + 0.001, bz + THROAT),
                  (bx + sx + 0.004, by + FLANGE_W / 2 - 0.001, bz + THROAT + TOPBAR_T), CLAMP_COL, collide=True)
    box_lo_hi(clp, "boss", k0, (bx - BOSS[0] / 2, by - BOSS[1] / 2, bz - BOSS[2] / 2),
              (bx + BOSS[0] / 2, by + BOSS[1] / 2, bz + BOSS[2] / 2), CLAMP_COL)
    box_lo_hi(clp, "spine", k0, (bx - BOSS[0] / 2, by - SPINE_W / 2, bz + BOSS[2] / 2),
              (bx - BOSS[0] / 2 + SPINE_W, by + SPINE_W / 2, bz + THROAT), CLAMP_COL)
    box_lo_hi(clp, "anvil", k0, (bx + ANVIL_X0, by - ANVIL_W / 2, bz + ANVIL_LOW),
              (bx + ANVIL_X1, by + ANVIL_W / 2, bz + THROAT), CLAMP_COL)

    h0 = np.array(HANDLE.pos)
    part(han, "pin", "cyl", (bx + PIN_X - h0[0], 0.0, 0.0), radius=PIN_R, height=PIN_L, axis="Y", rgb=STEEL,
         collide=True)
    s0 = np.array(SCREW.pos)
    part(scr, "swivel", "cyl", (bx + E0 - TIP_T / 2 - s0[0], 0.0, 0.0), radius=TIP_R, height=TIP_T, axis="X",
         rgb=STEEL, collide=True)

    # solver accuracy on loaded contact & joint bodies
    for pth in (cal, pis, clp, han, scr, *hose_ps):
        pr = stage.GetPrimAtPath(pth)
        if pr.IsValid():
            rb = PhysxSchema.PhysxRigidBodyAPI.Apply(pr)
            rb.CreateSolverPositionIterationCountAttr(32)
            rb.CreateSolverVelocityIterationCountAttr(8)
            rb.CreateMaxDepenetrationVelocityAttr(1.0)

    def wpose(path):
        m = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)
        return np.array(m.ExtractTranslation(), float), m.ExtractRotationQuat()

    jroot = f"{root}/clamp_joints"
    UsdGeom.Xform.Define(stage, jroot)

    def joint(kind, name, b0, b1, anchor, axis="X"):
        J = {"rev": UsdPhysics.RevoluteJoint, "pri": UsdPhysics.PrismaticJoint}[kind].Define(stage, f"{jroot}/{name}")
        t0, _ = wpose(b0)
        t1, _ = wpose(b1)
        J.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
        J.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        J.CreateAxisAttr(axis)
        a = np.asarray(anchor, float)
        J.CreateLocalPos0Attr(Gf.Vec3f(*(a - t0)))
        J.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        J.CreateLocalPos1Attr(Gf.Vec3f(*(a - t1)))
        J.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        J.CreateExcludeFromArticulationAttr(True)
        return J

    # piston in bore: elasto-plastic square-cut seal + viscous fluid drag
    pj = joint("pri", "piston_bore", cal, pis, (X_FACE, YC, TOP + ZA))
    pj.CreateLowerLimitAttr(-OUT0 - 0.0003)
    pj.CreateUpperLimitAttr(0.0)
    d = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(PISTON_K)
    d.CreateDampingAttr(PISTON_DAMP)
    d.CreateTargetPositionAttr(0.0)
    _P["piston_bore"] = str(pj.GetPath())

    # T-handle revolute + screw prismatic in clamp boss
    hj = joint("rev", "handle_turn", clp, han, (bx, by, bz))
    dh = UsdPhysics.DriveAPI.Apply(hj.GetPrim(), "angular")
    dh.CreateTypeAttr("force")
    dh.CreateStiffnessAttr(0.0)
    dh.CreateDampingAttr(HANDLE_DAMP * math.pi / 180)
    _P["handle_turn"] = str(hj.GetPath())

    sj = joint("pri", "screw_slide", clp, scr, (bx, by, bz))
    sj.CreateLowerLimitAttr(-0.002)
    sj.CreateUpperLimitAttr(E_MAX - E0)
    ds = UsdPhysics.DriveAPI.Apply(sj.GetPrim(), "linear")
    ds.CreateTypeAttr("force")
    ds.CreateStiffnessAttr(SCREW_K)
    ds.CreateDampingAttr(SCREW_C)
    ds.CreateTargetPositionAttr(0.0)
    _P["screw_slide"] = str(sj.GetPath())

    # 5. Articulated flexible brake hose: 6 capsules joined by 2-axis bend springs (stress-free in spawn curve)
    def gf_to_wxyz(q):
        im = q.GetImaginary()
        return np.array([float(q.GetReal()), float(im[0]), float(im[1]), float(im[2])])

    def wxyz_to_gf(q):
        return Gf.Quatf(float(q[0]), float(q[1]), float(q[2]), float(q[3]))

    def qinv(q):
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def hose_bend_joint(idx: int, parent_path: str | None, child_path: str | None, world_pt: np.ndarray):
        jp = f"{jroot}/hose_bend_{idx}"
        j = UsdPhysics.Joint.Define(stage, jp)
        if parent_path is not None:
            j.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
            tp, qp_gf = wpose(parent_path)
            qp = gf_to_wxyz(qp_gf)
            lp0 = _qrot(qinv(qp), world_pt - tp)
        else:
            qp = np.array([1.0, 0.0, 0.0, 0.0])
            lp0 = world_pt
        if child_path is not None:
            j.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
            tc, qc_gf = wpose(child_path)
            qc = gf_to_wxyz(qc_gf)
            lp1 = _qrot(qinv(qc), world_pt - tc)
        else:
            qc = qp.copy()
            lp1 = np.zeros(3)
        rel_q = _qmul(qinv(qp), qc)
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, lp0)))
        j.CreateLocalRot0Attr(wxyz_to_gf(rel_q))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, lp1)))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        prim = j.GetPrim()
        for dof in ("transX", "transY", "transZ", "rotZ"):
            lim = UsdPhysics.LimitAPI.Apply(prim, dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        for dof in ("rotX", "rotY"):
            lim = UsdPhysics.LimitAPI.Apply(prim, dof)
            lim.CreateLowAttr(-65.0)
            lim.CreateHighAttr(65.0)
            drv = UsdPhysics.DriveAPI.Apply(prim, dof)
            drv.CreateTypeAttr("force")
            drv.CreateStiffnessAttr(HOSE_STIFF * math.pi / 180)
            drv.CreateDampingAttr(HOSE_DAMP * math.pi / 180)
            drv.CreateTargetPositionAttr(0.0)

    hose_bend_joint(0, cal, hose_ps[0], _HOSE_PTS[0])
    for i in range(1, HOSE_N):
        hose_bend_joint(i, hose_ps[i - 1], hose_ps[i], _HOSE_PTS[i])
    hose_bend_joint(HOSE_N, None, hose_ps[-1], _HOSE_PTS[-1])  # anchored at fixed chassis bracket on wheel-well liner

    # filter internal contacts between coupled neighbours
    pairs = [(scr, han), (clp, scr), (clp, han), (cal, pis), (cal, hose_ps[0]), (hose_ps[-1], lrf_p)]
    pairs += [(hose_ps[i - 1], hose_ps[i]) for i in range(1, HOSE_N)]
    for a_p, b_p in pairs:
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a_p)).CreateFilteredPairsRel().AddTarget(Sdf.Path(b_p))

    # physics materials: ordinary dry cast iron / steel on seating surfaces; high-tack elastomer/ribbed grip on the
    # C-clamp top-bar grip saddle and T-pin so parallel-jaw grasps hold purely by physical contact (no weld)
    iron_mat = f"{root}/iron_material"
    mc = sim_utils.RigidBodyMaterialCfg(static_friction=0.45, dynamic_friction=0.38)
    mc.func(iron_mat, mc)
    grip_mat = f"{root}/grip_material"
    gc = sim_utils.RigidBodyMaterialCfg(
        static_friction=GRIP_MU[0], dynamic_friction=GRIP_MU[1], friction_combine_mode="max", restitution=0.0,
        restitution_combine_mode="min",
    )
    gc.func(grip_mat, gc)

    for path in (cal, pis, scr):
        for p in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                bind_physics_material(str(p.GetPath()), iron_mat, stage=stage)
    for p in Usd.PrimRange(stage.GetPrimAtPath(clp)):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            nm = p.GetName()
            use_grip = nm in (CLAMP.name, "collision", "grip_flange", "rib_back", "rib_front") or p.GetPath() == Sdf.Path(clp)
            bind_physics_material(str(p.GetPath()), grip_mat if use_grip else iron_mat, stage=stage)
    for p in Usd.PrimRange(stage.GetPrimAtPath(han)):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            bind_physics_material(str(p.GetPath()), grip_mat, stage=stage)
    print(f"[clamp] authored wheel-well corner, hose and C-clamp under {root}", flush=True)


def _wrap_cfg(spec, obj):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import clone
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg
    from yam_arena.station import aim, linear_rgb

    if spec.name in HOSE_NAMES:
        idx = HOSE_NAMES.index(spec.name)
        a, b = _HOSE_SEGS[idx]
        seg_l = float(np.linalg.norm(b - a))
        obj.spawner_cfg = sim_utils.CapsuleCfg(
            radius=HOSE_R,
            height=max(seg_l - 0.5 * HOSE_R, 0.004),
            rigid_props=PhysxRigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
            collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.003, rest_offset=0.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.7, dynamic_friction=0.6),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(HOSE_COL), roughness=0.65),
        )
        obj.initial_pose = Pose(position_xyz=tuple(((a + b) / 2).tolist()), rotation_xyzw=aim(b - a))
        obj.object_cfg = obj._init_object_cfg()
        return obj

    cfg = obj.spawner_cfg
    if spec.name in _AXIS_X:
        cfg.axis = "X"
    if spec.name in _AXIS_Y:
        cfg.axis = "Y"
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
    if getattr(scene.task_object, "_clamp_patched", False):
        return
    orig = scene.task_object
    special = _AXIS_X | _AXIS_Y | set(HOSE_NAMES) | {OBJECTS[-1].name}

    def task_object(spec):
        obj = orig(spec)
        if getattr(spec, "name", None) in special:
            _wrap_cfg(spec, obj)
        return obj

    task_object._clamp_patched = True
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


# -- state tracking, every control step ----------------------------------------------------------------------------------
COCK_MAX = math.radians(3.0)  # screw axis > 3 deg off bore axis while piston advances
COCK_PUSH = 0.001  # piston pushed >= 1.0 mm inward while cocked
UNSEATED_PUSH = 0.001  # piston pushed >= 1.0 mm inward without clamp seated

_FRESH = dict(
    env=None, step=0, cal0=None, out=OUT0, out_min=OUT0, cocked=False, cock_deg=0.0, max_cock_deg=0.0,
    tilt_deg=0.0, max_tilt_deg=0.0, slide=0.0, max_slide=0.0, seated=False, ever_seated=False, nan=False,
    dropped=False, unseated_push=False, unseated_ref=OUT0, e=E0, turn=0.0, cock_ref=OUT0,
    max_engaged_cock_deg=0.0, g=None, theta=0.0, th_prev=None, tgt_set=0.0, pending_tgt=None,
    seal_tgt=0.0, pending_seal=None, hdamp_set=HANDLE_DAMP, pending_hdamp=None, load=0.0,
    hose_span=HOSE_REST_SPAN, max_hose_span=HOSE_REST_SPAN, hose_bend_deg=0.0, max_hose_bend_deg=0.0,
    hose_strained=False, hose_kinked=False,
)
_M: dict = {}


def _reset_model() -> None:
    env = _M.get("env")
    _M.clear()
    _M.update(_FRESH)
    _M["env"] = env


_reset_model()


def _qrot(q, v):
    """Rotate v by quaternion q given as (w, x, y, z)."""
    w, x, y, z = q
    u = np.array([x, y, z])
    v = np.asarray(v, float)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def _qmul(a, b):
    """Quaternion product, (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _body(objects, name):
    """Rigid body origin (world, env 0) and orientation (w, x, y, z)."""
    env = _M.get("env")
    if env is not None:
        base = env.unwrapped
        pose = base.arena_world.get_pose_w(name)[0].detach().cpu().numpy().astype(np.float64)
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
        x, y, z, w = pose[3:7]
        return pose[:3] - origin, np.array([w, x, y, z])
    o = objects[name]
    x, y, z, w = np.asarray(o.quat_xyzw, float)
    return np.asarray(o.pos, float), np.array([w, x, y, z])


def axes(objects) -> dict:
    """Live geometry: caliper bore axis, clamp screw axis, piston protrusion, contact gaps, and hose state."""
    cp, cq = _body(objects, CALIPER.name)
    pp, _ = _body(objects, PISTON.name)
    kp, kq = _body(objects, CLAMP.name)
    sp, _ = _body(objects, SCREW.name)
    _, hq = _body(objects, HANDLE.name)
    cx = _qrot(cq, [1, 0, 0])
    kx, kz = _qrot(kq, [1, 0, 0]), _qrot(kq, [0, 0, 1])
    out = float(np.dot(pp - cp, cx)) - (_PX0 - CALIPER.pos[0] - OUT0)
    boss = kp + _qrot(kq, [CL0[0] - CLAMP.pos[0], 0.0, TOP + CL0[2] - CLAMP.pos[2]])
    e = float(np.dot(sp - boss, kx)) + _SHANK_L / 2
    back = cp - cx * (X_FACE - X_BACK) / 2
    lining = cp + cx * ((X_FACE - X_BACK) / 2 + out + PLATE_T + LINING_T)
    anvil = boss + kx * ANVIL_X0
    tip = boss + kx * e
    w0, v0 = kq[0], -kq[1:]
    w1, v1 = hq[0], hq[1:]
    rw = w0 * w1 - np.dot(v0, v1)
    rv = w0 * v1 + w1 * v0 + np.cross(v0, v1)
    turn = 2.0 * math.atan2(float(rv[0]), float(rw))
    banjo_off = BANJO_WORLD0 - np.asarray(CALIPER.pos, float)
    banjo = cp + _qrot(cq, banjo_off)
    hose_span = float(np.linalg.norm(BRACKET_WORLD - banjo))
    dirs = []
    for hn in HOSE_NAMES:
        if hn in objects or _M.get("env") is not None:
            _, hqi = _body(objects, hn)
            dirs.append(_qrot(hqi, [0, 0, 1]))
    max_bend = 0.0
    for i in range(1, len(dirs)):
        max_bend = max(max_bend, math.degrees(math.acos(float(np.clip(np.dot(dirs[i - 1], dirs[i]), -1.0, 1.0)))))
    return dict(
        cx=cx, kx=kx, kz=kz, out=out, turn=turn, e=e, boss=boss, tip=tip, anvil=anvil, back=back, lining=lining,
        cock=float(math.acos(np.clip(abs(np.dot(cx, kx)), -1.0, 1.0))),
        tip_gap=float(np.dot(back - tip, cx)),
        anvil_gap=float(np.dot(anvil - lining, cx)),
        lateral=float(np.linalg.norm(np.cross(cx, tip - back))),
        anvil_lateral=float(np.linalg.norm(np.cross(cx, anvil - lining))),
        hose_span=hose_span,
        hose_bend_deg=max_bend,
    )


SEAT_GAP = 0.002  # swivel pad and anvil each within 2.0 mm axially of caliper back and worn pad face
SEAT_LAT = 0.020  # swivel pad and anvil each within 2.0 cm radially of the bore axis


def seated(g) -> bool:
    return bool(
        float(np.dot(g["cx"], g["kx"])) > 0.0
        and abs(g["tip_gap"]) <= SEAT_GAP
        and abs(g["anvil_gap"]) <= SEAT_GAP
        and g["lateral"] <= SEAT_LAT
        and g["anvil_lateral"] <= SEAT_LAT
    )


def _track(objects) -> None:
    try:
        g = axes(objects)
    except Exception:
        return
    if not np.isfinite(g["out"]):
        _M["nan"] = True
        return
    if _M.get("env") is None and not _M.get("unit_test"):
        return
    for nm in (CALIPER.name, PISTON.name, CLAMP.name, HANDLE.name, SCREW.name):
        if nm in objects and float(np.asarray(objects[nm].low, float)[2]) < TOP - 0.02:
            _M["dropped"] = True
    cp, cq = _body(objects, CALIPER.name)
    if _M["cal0"] is None:
        _M["cal0"] = (cp.copy(), _qrot(cq, [0, 0, 1]))
    p0, z0 = _M["cal0"]
    _M["tilt_deg"] = math.degrees(math.acos(float(np.clip(np.dot(_qrot(cq, [0, 0, 1]), z0), -1, 1))))
    _M["slide"] = float(np.linalg.norm((cp - p0)[:2]))
    _M["max_tilt_deg"] = max(_M["max_tilt_deg"], _M["tilt_deg"])
    _M["max_slide"] = max(_M["max_slide"], _M["slide"])
    _M["out"] = g["out"]
    _M["out_min"] = min(_M["out_min"], g["out"])
    _M["e"] = g["e"]
    _M["seated"] = seated(g)
    _M["ever_seated"] |= _M["seated"]
    _M["cock_deg"] = math.degrees(g["cock"])
    _M["turn"] = g["turn"]
    _M["hose_span"] = g["hose_span"]
    _M["max_hose_span"] = max(_M["max_hose_span"], g["hose_span"])
    _M["hose_bend_deg"] = g["hose_bend_deg"]
    _M["max_hose_bend_deg"] = max(_M["max_hose_bend_deg"], g["hose_bend_deg"])
    if g["hose_span"] > HOSE_MAX_SPAN and not _M["hose_strained"]:
        _M["hose_strained"] = True
        print(f"[clamp] brake hose stretched taut: span {g['hose_span'] * 100:.1f} cm > {HOSE_MAX_SPAN * 100:.1f} cm",
              flush=True)
    if g["hose_bend_deg"] > HOSE_MAX_BEND_DEG and not _M["hose_kinked"]:
        _M["hose_kinked"] = True
        print(f"[clamp] brake hose kinked: bend {g['hose_bend_deg']:.1f} deg > {HOSE_MAX_BEND_DEG:.1f} deg",
              flush=True)
    _M["g"] = g
    # unseated push: piston forced >= 1.0 mm inward without the C-clamp seated across caliper back and worn pad
    if _M["seated"]:
        _M["unseated_ref"] = g["out"]
    else:
        _M["unseated_ref"] = max(_M["unseated_ref"], g["out"])
        if _M["unseated_ref"] - g["out"] >= UNSEATED_PUSH and not _M["unseated_push"]:
            _M["unseated_push"] = True
            print(f"[clamp] piston pushed {(_M['unseated_ref'] - g['out']) * 1000:.2f} mm without clamp seated",
                  flush=True)
    # cocked push: while seated, the clamp pushes the piston >= 1.0 mm inward with screw > 3.0 deg off bore axis
    if not _M["seated"] or g["cock"] <= COCK_MAX:
        _M["cock_ref"] = g["out"]
    else:
        _M["cock_ref"] = max(_M["cock_ref"], g["out"])
        _M["max_cock_deg"] = max(_M["max_cock_deg"], math.degrees(g["cock"]))
        if _M["cock_ref"] - g["out"] >= COCK_PUSH and not _M["cocked"]:
            _M["cocked"] = True
            print(f"[clamp] piston pushed crooked: screw {math.degrees(g['cock']):.1f} deg off bore", flush=True)
    if _M["seated"]:
        _M["max_engaged_cock_deg"] = max(_M["max_engaged_cock_deg"], math.degrees(g["cock"]))


# -- physics substep: self-locking screw thread + square-cut piston seal -------------------------------------------------
def _views(mgr):
    if _M.get("views") is None:
        sv = mgr.get_physics_sim_view()
        if sv is None or "clamp" not in _P:
            return None
        _M["views"] = {k: sv.create_rigid_body_view(_P[k]) for k in ("caliper", "piston", "clamp", "handle", "screw")}
    return _M["views"]


HANDLE_REF_W = 2.0  # rad/s nominal turning rate converting thread friction torque to implicit joint damping


def _drive(mgr) -> None:
    try:
        v = _views(mgr)
        if v is None:
            return
        _M["subs"] = _M.get("subs", 0) + 1
        T = {k: np.asarray(v[k].get_transforms().numpy(), dtype=np.float64).reshape(-1) for k in v}
        if not all(np.isfinite(t).all() for t in T.values()):
            _M["nan"] = True
            return
        wxyz = lambda t: np.array([t[6], t[3], t[4], t[5]])  # noqa: E731
        qk, qh = wxyz(T["clamp"]), wxyz(T["handle"])
        kx = _qrot(qk, [1.0, 0.0, 0.0])
        rel = _qmul(np.array([qk[0], -qk[1], -qk[2], -qk[3]]), qh)
        th = 2.0 * math.atan2(rel[1], rel[0])
        if _M["th_prev"] is None:
            _M["th_prev"] = th
        _M["theta"] += (th - _M["th_prev"] + math.pi) % (2 * math.pi) - math.pi
        _M["th_prev"] = th
        boss = T["clamp"][:3] + _qrot(qk, [CL0[0] - CLAMP.pos[0], 0.0, TOP + CL0[2] - CLAMP.pos[2]])
        x = float(np.dot(T["screw"][:3] - boss, kx)) - (E0 - _SHANK_L / 2)
        tgt = float(np.clip(_M["theta"] * LEAD / (2 * math.pi), -0.002, E_MAX - E0))
        if abs(tgt - _M["tgt_set"]) > 2e-6:
            _M["tgt_set"] = tgt
            _M["pending_tgt"] = tgt
        load = max(0.0, SCREW_K * (_M["tgt_set"] - x))
        _M["load"] = load
        tq_fric = THREAD_T0 + load * LEAD / (2 * math.pi * THREAD_EFF)
        hdamp = tq_fric / HANDLE_REF_W
        if abs(hdamp - _M["hdamp_set"]) > 0.005:
            _M["hdamp_set"] = hdamp
            _M["pending_hdamp"] = hdamp
        qc = wxyz(T["caliper"])
        cx = _qrot(qc, [1.0, 0.0, 0.0])
        xp = float(np.dot(T["piston"][:3] - T["caliper"][:3], cx)) - (_PX0 - CALIPER.pos[0])
        slip = PISTON_SEAL / PISTON_K
        if abs(_M["seal_tgt"] - xp) > slip:
            stgt = float(np.clip(xp + math.copysign(slip, _M["seal_tgt"] - xp), -OUT0 - slip, 0.0))
            if abs(stgt - _M["seal_tgt"]) > 1e-6:
                _M["seal_tgt"] = stgt
                _M["pending_seal"] = stgt
    except Exception as e:  # pragma: no cover
        if _M.get("err") is None:
            _M["err"] = repr(e)
            print(f"[clamp] model error: {e!r}", flush=True)


def _set_target() -> None:
    t = _M.get("pending_tgt")
    s = _M.get("pending_seal")
    hd = _M.get("pending_hdamp")
    if t is None and s is None and hd is None:
        return
    from isaaclab.sim.utils.stage import get_current_stage

    st = get_current_stage()
    if t is not None and "screw_slide" in _P:
        _M["pending_tgt"] = None
        st.GetPrimAtPath(_P["screw_slide"]).GetAttribute("drive:linear:physics:targetPosition").Set(t)
    if s is not None and "piston_bore" in _P:
        _M["pending_seal"] = None
        st.GetPrimAtPath(_P["piston_bore"]).GetAttribute("drive:linear:physics:targetPosition").Set(s)
    if hd is not None and "handle_turn" in _P:
        _M["pending_hdamp"] = None
        st.GetPrimAtPath(_P["handle_turn"]).GetAttribute("drive:angular:physics:damping").Set(hd * math.pi / 180)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_clamp_hooked", False):
        return
    base_step = mgr.step.__func__

    def step(cls) -> None:
        _set_target()
        base_step(cls)
        _drive(cls)

    step._clamp_hooked = True
    mgr.step = classmethod(step)


class _MgrFinder(importlib.abc.MetaPathFinder):
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
            _patch_manager(module)

        spec.loader.exec_module = exec_module
        return spec


def _bind_finger_friction() -> None:
    """Bind elastomer pad friction to the robot finger pads once after env build (pure contact grasp, no weld)."""
    if _M.get("fingers_bound") or "root" not in _P:
        return
    from isaaclab.sim.utils import bind_physics_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Usd, UsdPhysics

    stage = get_current_stage()
    gmat = f"{_P['root']}/grip_material"
    if not stage.GetPrimAtPath(gmat).IsValid():
        return
    n_bound = 0
    for arm in ("LeftArm", "RightArm"):
        ap = stage.GetPrimAtPath(f"/World/envs/env_0/{arm}")
        if not ap.IsValid():
            continue
        for p in Usd.PrimRange(ap, Usd.TraverseInstanceProxies()):
            ps = str(p.GetPath())
            if ("tip_left" in ps or "tip_right" in ps) and p.HasAPI(UsdPhysics.CollisionAPI):
                bind_physics_material(ps, gmat, stage=stage)
                n_bound += 1
    _M["fingers_bound"] = True
    if n_bound:
        print(f"[clamp] bound elastomer pad friction to {n_bound} finger colliders", flush=True)


def _install_hooks() -> None:
    if _PHYSX_MGR in sys.modules:
        _patch_manager(sys.modules[_PHYSX_MGR])
    elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _MgrFinder())
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
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
    if not getattr(cls._task_step, "_clamp_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _M["env"] = self._env
            _M["step"] += 1
            _bind_finger_friction()
            return base(self, *a, **k)

        _task_step._clamp_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- scoring -------------------------------------------------------------------------------------------------------------
TILT_MAX = 15.0  # deg: caliper tipped on support block
SLIDE_MAX = 0.030  # m: caliper slid > 3.0 cm horizontally on support block
OUT_HALF = 0.004  # m: partial tier <= 4.0 mm out (at least half of 8.0 mm retracted)
OUT_DONE = 0.001  # m: full retraction <= 1.0 mm out (flush)
CLEAR = 0.10  # m: gripper TCPs >= 10.0 cm from caliper, piston/pad, brake hose, and C-clamp
OFF = 0.020  # m: C-clamp >= 2.0 cm clear of caliper, piston/pad, and brake hose
STILL_STEPS = 20  # 1.0 s at 20 Hz
STILL_MOVE = 0.010  # m: each gripper TCP moves <= 1.0 cm over the last 1.0 s


def grippers() -> list[np.ndarray]:
    if _M.get("unit_grippers") is not None:
        return _M["unit_grippers"]
    env = _M.get("env")
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


def jaws_released() -> bool:
    """Both grippers open beyond closed pinch (>= 20 mm jaw opening)."""
    if _M.get("unit_jaws_open") is not None:
        return bool(_M["unit_jaws_open"])
    env = _M.get("env")
    if env is None:
        return False
    base = env.unwrapped
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        idx = list(rob.joint_names).index("joint7")
        q7 = float(rob.data.joint_pos.torch[0, idx].detach().cpu().item())
        if -2.0 * q7 < 0.020:
            return False
    return True


def _box_dist(p, o) -> float:
    lo, hi = np.asarray(o.low, float), np.asarray(o.high, float)
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(lo - p, p - hi))))


def clamp_off(objects) -> bool:
    """Clamp (frame, handle, screw) on the table or rack and >= 2.0 cm clear of caliper, piston/pad and brake hose."""
    k = objects[CLAMP.name]
    if float(np.asarray(k.low, float)[2]) < TOP - 0.01:
        return False
    targets = (CALIPER.name, PISTON.name, *(n for n in HOSE_NAMES if n in objects))
    for cn in targets:
        lo1, hi1 = np.asarray(objects[cn].low, float), np.asarray(objects[cn].high, float)
        for kn in (CLAMP.name, HANDLE.name, SCREW.name):
            lo2, hi2 = np.asarray(objects[kn].low, float), np.asarray(objects[kn].high, float)
            gap = np.maximum(0.0, np.maximum(lo1 - hi2, lo2 - hi1))
            if float(np.linalg.norm(gap)) < OFF:
                return False
    return True


def clamp_resting(objects) -> bool:
    return bool(objects[CLAMP.name].resting(0.02, 0.5))


def hands_clear(objects) -> bool:
    """Both grippers open (>= 2.0 cm), TCPs >= 10.0 cm from caliper, piston/pad, hose and clamp, still for 1.0 s."""
    if not jaws_released():
        return False
    g = grippers()
    if not g:
        return False
    hist = _M.setdefault("ghist", [])
    if not hist or hist[-1][0] != _M["step"]:
        hist.append((_M["step"], [p.copy() for p in g]))
        del hist[: -(STILL_STEPS + 1)]
    still = len(hist) > STILL_STEPS and all(
        max(float(np.linalg.norm(h[1][i] - g[i])) for h in hist) <= STILL_MOVE for i in range(len(g))
    )
    names = (CALIPER.name, CLAMP.name, PISTON.name, HANDLE.name, SCREW.name, *(n for n in HOSE_NAMES if n in objects))
    return still and all(_box_dist(t, objects[n]) >= CLEAR for t in g for n in names)


def score(objects) -> float:
    _track(objects)
    if os.environ.get("CAL_DEBUG") and _M["step"] % 10 == 0:
        try:
            g = axes(objects)
            print(
                f"[clamp dbg] step {_M['step']} out {g['out'] * 1000:.2f} e {g['e'] * 1000:.1f} "
                f"tip_gap {g['tip_gap'] * 1000:.1f} anvil_gap {g['anvil_gap'] * 1000:.1f} "
                f"cock {math.degrees(g['cock']):.1f} hose_span {g['hose_span'] * 100:.1f}cm "
                f"hose_bend {g['hose_bend_deg']:.1f}deg M {progress(objects)}",
                flush=True,
            )
        except Exception as e:  # noqa: BLE001
            print("[clamp dbg] err", repr(e), flush=True)
    if (
        _M["nan"]
        or _M["dropped"]
        or _M["cocked"]
        or _M["unseated_push"]
        or _M["hose_strained"]
        or _M["hose_kinked"]
        or _M["max_tilt_deg"] > TILT_MAX
        or _M["max_slide"] > SLIDE_MAX
    ):
        return 0.0
    out = _M["out"]
    if out > OUT_HALF:
        return 0.25 if _M["seated"] else 0.0
    if not _M["ever_seated"]:
        return 0.0
    if out > OUT_DONE:
        return 0.5
    if not (clamp_off(objects) and clamp_resting(objects) and hands_clear(objects)):
        return 0.8
    return 1.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def progress(objects) -> dict:
    return {
        "piston_out_mm": round(_M["out"] * 1000, 2),
        "seated": float(_M["seated"]),
        "ever_seated": float(_M["ever_seated"]),
        "cocked": float(_M["cocked"]),
        "unseated_push": float(_M["unseated_push"]),
        "hose_strained": float(_M["hose_strained"]),
        "hose_kinked": float(_M["hose_kinked"]),
        "dropped": float(_M["dropped"]),
        "max_cock_deg": round(_M["max_cock_deg"], 2),
        "max_engaged_cock_deg": round(_M["max_engaged_cock_deg"], 2),
        "max_tilt_deg": round(_M["max_tilt_deg"], 2),
        "max_slide_mm": round(_M["max_slide"] * 1000, 1),
        "max_hose_span_cm": round(_M["max_hose_span"] * 100, 2),
        "max_hose_bend_deg": round(_M["max_hose_bend_deg"], 1),
    }


VARIATION = {
    "obj:caliper": {"dx": 0.008, "dy": 0.008, "yaw": 0.04},
    "obj:c_clamp": {"dx": 0.005, "dy": 0.005, "yaw": 0.03},
}
