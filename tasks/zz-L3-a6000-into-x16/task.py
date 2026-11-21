"""Install an NVIDIA RTX A6000 in an unplugged workstation's top PCIe x16 slot.

Scene (metres from the arm-base midpoint, x forward, y left; heights above the table). An unplugged Fractal Design
Define 7 Compact mid-tower (427 mm depth along y x 210 mm width along z x 474 mm height along x; 454 mm steel body
from x 0.165 to 0.619 plus 20 mm feet to 0.639, rear wall at y -0.195, front bezel at y +0.232) lies horizontally on
its right side panel with its left side panel and rear expansion covers 1-2 removed, open side facing +z. Inside sits
a horizontal ATX motherboard (305 x 244 x 1.6 mm, top face z 0.026) with a CPU cooler block, four DDR4 DIMMs, and two
PCIe x16 slots running along y: top Slot 1 at x 0.340 (mouth z 0.03725, floor z 0.02925, white winged retention latch
initially open 38 deg at its +y end) and lower Slot 3 two ATX pitches away at x 0.38064. An NVIDIA RTX A6000
(266.7 x 111.15 x 38.0 mm dual-slot, 1.18 kg) stands upright connector-down in a slotted black ESD polyethylene foam
cradle centred at (0.120, -0.015), bracket toward -y, top edge at z 0.1307.

Because the card weighs 1.18 kg (above one YAM arm's ~0.9 kg payload at reach) and spans 266.7 mm, both arms carry it
together by opposite ends of its top edge, lift it over the 21.0 cm case rim, register the bracket bottom tab in the
rear-wall tab pocket while aligning the keyed 89 mm gold-finger edge in Slot 1, and press both ends down evenly against
36 N total connector contact resistance (18 N per end) until seated within 1.5 mm of the slot floor and level within
1.5 deg, snapping Slot 1's retention latch closed into the card's hook.

Run it:
  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 600 \\
    --instruction "Install the graphics card in the top x16 slot of the workstation, then let go."
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "Install the graphics card in the top x16 slot of the workstation, then let go."
RUN_LIMIT_STEPS = 600  # 30 s at 20 Hz (demo completes in 349 steps; ceil(1.5 * 349 / 300) * 300 = 600)

# -- visual palette -----------------------------------------------------------------------------------------------------
CASE_COL = (0.10, 0.10, 0.11)       # matte charcoal steel chassis
SHROUD_COL = (0.07, 0.07, 0.08)     # PSU shroud & interior steel
BOARD_COL = (0.05, 0.16, 0.09)      # dark green FR-4 ATX PCB
VRM_COL = (0.62, 0.64, 0.67)        # brushed aluminium heatsinks / rear I/O shield
SLOT_COL = (0.03, 0.03, 0.035)      # black LCP PCIe connector housing
LATCH_COL = (0.94, 0.94, 0.91)      # high-contrast white PCIe retention wing
CARD_COL = (0.11, 0.11, 0.12)       # dark anodised RTX A6000 shroud
BACK_COL = (0.16, 0.16, 0.18)       # dark gunmetal backplate
TRIM_COL = (0.82, 0.66, 0.28)       # RTX A6000 champagne-gold bezel trim
FAN_COL = (0.04, 0.04, 0.045)       # radial blower intake
GOLD = (0.95, 0.76, 0.28)           # gold-plated PCIe edge fingers
PCB_EDGE_COL = (0.08, 0.28, 0.14)   # green PCB substrate strip
STEEL = (0.74, 0.75, 0.78)          # stainless PCIe dual-slot I/O bracket
FOAM_COL = (0.13, 0.14, 0.16)       # closed-cell black/charcoal ESD polyethylene foam
CPU_COL = (0.18, 0.19, 0.22)        # CPU pump/heatsink block
RAM_COL = (0.14, 0.28, 0.52)        # blue anodised DDR4 heatspreaders

# -- workstation chassis (fixed): Fractal Design Define 7 Compact lying on right side panel -----------------------------
X_CASE0 = 0.165                     # tower top panel outer face
CASE_BODY_X = 0.454                 # steel chassis height (474 mm overall incl. 20 mm feet at +x)
FOOT_X = 0.020
X_CASE1 = X_CASE0 + CASE_BODY_X     # 0.619
Y_REAR = -0.195                     # rear wall outer face
CASE_D = 0.427                      # rear-to-front depth along y
Y_FRONT = Y_REAR + CASE_D           # +0.232
CASE_Z = 0.210                      # tower width (vertical rim height when lying on right side)
WALL = 0.004                        # panel collision thickness

# -- ATX motherboard (305 x 244 x 1.6 mm, fixed, horizontal at z = 0.026 m) --------------------------------------------
BOARD_T = 0.0016
Z_TRAY = 0.0179
Z_BOARD = 0.0260                    # motherboard top surface above table (tray + 6.5 mm standoffs + 1.6 mm PCB)
BOARD_X0, BOARD_DX = 0.179, 0.305   # x: 0.179 .. 0.484
BOARD_Y0, BOARD_DY = -0.177, 0.244  # y: -0.177 .. +0.067

# Surrounding prohibited motherboard components (contacting any scores 0)
CPU_POS = (0.245, -0.105, Z_BOARD + 0.024)
CPU_SIZE = (0.075, 0.075, 0.048)    # top at z = 0.074
DIMM_X0, DIMM_LEN, DIMM_W, DIMM_H = 0.184, 0.1334, 0.0045, 0.038  # x: 0.184..0.3174, top at z = 0.064
DIMM_YS = (-0.024, -0.014, -0.004, 0.006)

# -- PCIe x16 slots (Slot 1 target, Slot 3 lower x16 two ATX pitches toward +x) -----------------------------------------
PITCH = 0.02032                     # ATX expansion slot pitch (20.32 mm)
X_SLOT1 = 0.3400                    # Slot 1 centre plane
X_SLOT3 = X_SLOT1 + 2 * PITCH       # 0.38064 (Slot 2 at 0.36032 is covered by the dual-slot cooler)
CONN_H = 0.01125                    # connector height above board (11.25 mm)
Z_MOUTH = Z_BOARD + CONN_H          # 0.03725 (3.725 cm)
SEAT_D = 0.0080                     # full seating depth of gold fingers inside slot (8.0 mm)
Z_FLOOR = Z_MOUTH - SEAT_D          # 0.02925 (2.925 cm)
CAV_W = 0.0034                      # slot cavity width along x (for 1.57 mm PCB, incl. mouth chamfer clearance)
CONN_W = 0.0076                     # connector housing outer width along x

BR_Y = -0.1775                      # seated bracket plate outer (-y) face
FING0 = 0.0420                      # CEM datum: bracket outer face to first gold finger (42.0 mm)
FING_SHORT, KEY_W, FING_LONG = 0.01120, 0.00280, 0.07500  # 89.00 mm total edge connector (bevelled key notch)
FING_L = FING_SHORT + KEY_W + FING_LONG
SLOT_Y0 = BR_Y + FING0 - 0.0020     # -0.1375
SLOT_Y1 = BR_Y + FING0 + FING_L + 0.0020  # -0.0445
KEY_Y = BR_Y + FING0 + FING_SHORT + KEY_W / 2  # -0.12290 (polarisation key centre)
KEY_RW = 0.0009                     # bevelled key ridge width along y (0.95 mm clearance per side in notch)

LATCH_PIVOT_Y = SLOT_Y1 + 0.0045    # -0.0412 (hinge of winged retention latch at +y end of slot)
LATCH_OPEN_DEG = 38.0               # cocked open angle (+38 deg toward +y) at start; 0 deg when snapped closed

# -- graphics card (free): NVIDIA RTX A6000 (266.7 x 111.15 x 38.0 mm, 1.18 kg) ----------------------------------------
CARD_L = 0.2667                     # length along y
CARD_H = 0.11115                    # total height from gold-finger bottom to card top edge along z
FING_D = 0.0085                     # gold-finger protrusion below shroud bottom along z
BODY_H = CARD_H - FING_D            # 0.10265 shroud/PCB body height
BODY_T = 0.0380                     # dual-slot thickness along x
PCB_T = 0.00157                     # PCB thickness
PCB_DX = -0.0140                    # PCB plane offset from body centre (5 mm inside -x backplate face)
CARD_M = 1.18                       # kg

BRK_T = 0.0009                      # 0.9 mm stainless bracket steel
BRK_X_W = 2 * PITCH                 # 40.64 mm dual-slot bracket width
FLANGE_L = 0.0120                   # 12.0 mm top mounting flange bent toward -y
TAB_DROP = 0.0130                   # bottom tab extends 13.0 mm below shroud bottom (4.5 mm below gold fingers)
TAB_W = 0.0090                      # 9.0 mm tab width along x
HOOK_L = 0.0080                     # 8.0 mm retention hook tail past FING_LONG

# Seated card pose (world table-relative)
X_BODY_SEAT = X_SLOT1 - PCB_DX      # 0.3540
Y_CARD_SEAT = BR_Y + BRK_T + CARD_L / 2  # -0.04325
Z_TOP_SEAT = Z_FLOOR + CARD_H       # 0.14040
Z_LEDGE = 0.1400                    # rear chassis bracket mounting ledge top face (0.4 mm below seated flange)
Z_SLIT_TOP = 0.0320                 # rear-wall bracket tab pocket mouth

# -- ESD polyethylene foam cradle (fixed) & starting card pose ----------------------------------------------------------
CRADLE_X = 0.1200                   # 12.0 cm forward
CRADLE_Y = -0.0150                  # -1.5 cm (offset +2.83 cm in y from seated pose so 3D x-y registration is needed)
CRADLE_BASE_Z = 0.0100
Z_SHROUD_REST = 0.0280              # foam support shoulder height (gold fingers & tab hang freely in open cutouts)
CRADLE_WALL_TOP = 0.0580            # 30 mm side walls above shoulder (overall cradle 76 x 252 x 58 mm)

CARD_Z0 = Z_SHROUD_REST + BODY_H / 2 + 0.0002  # 0.079525 (top edge at 0.13085 m)
CARD_P0 = np.array([CRADLE_X, CRADLE_Y, CARD_Z0])
CARD = Box("gpu_card", pos=(float(CARD_P0[0]), float(CARD_P0[1]), TOP + float(CARD_P0[2])),
           size=(BODY_T, CARD_L, BODY_H), mass=CARD_M, color=CARD_COL)


def _cradle_parts() -> list[Box]:
    cx, cy = CRADLE_X, CRADLE_Y
    p = [
        # Base pad: 76 x 252 x 10 mm
        Box("esd_cradle_base", pos=(cx, cy, TOP + CRADLE_BASE_Z / 2), size=(0.076, 0.252, CRADLE_BASE_Z),
            color=FOAM_COL),
        # Two support shoulders under flat shroud spans (avoiding bracket tab at y_loc=-0.134 and fingers/hook [-0.093,+0.008])
        Box("esd_shoulder_r", pos=(cx, cy - 0.113, TOP + (CRADLE_BASE_Z + Z_SHROUD_REST) / 2),
            size=(0.040, 0.028, Z_SHROUD_REST - CRADLE_BASE_Z), color=FOAM_COL),
        Box("esd_shoulder_l", pos=(cx, cy + 0.070, TOP + (CRADLE_BASE_Z + Z_SHROUD_REST) / 2),
            size=(0.040, 0.106, Z_SHROUD_REST - CRADLE_BASE_Z), color=FOAM_COL),
    ]
    # Front (-x) and back (+x) side walls forming a 39.4 mm channel (0.7 mm clearance per side on 38.0 mm card)
    wt = 0.0183
    gap = 0.0394
    wh = CRADLE_WALL_TOP - CRADLE_BASE_Z
    for nm, sx in (("near", -1), ("far", 1)):
        p.append(Box(f"esd_wall_{nm}", pos=(cx + sx * (gap / 2 + wt / 2), cy, TOP + CRADLE_BASE_Z + wh / 2),
                     size=(wt, 0.252, wh), color=FOAM_COL))
    return p


def _case_parts() -> list[Box]:
    p = []
    xc = (X_CASE0 + X_CASE1) / 2
    yc = (Y_REAR + Y_FRONT) / 2
    # Right side panel lying flat on the table, motherboard tray above it
    p.append(Box("case_right_bottom", pos=(xc, yc, TOP + WALL / 2), size=(CASE_BODY_X, CASE_D, WALL), color=CASE_COL))
    p.append(Box("mb_tray", pos=(xc - 0.04, yc - 0.02, TOP + Z_TRAY - 0.001),
                 size=(0.350, 0.340, 0.002), color=SHROUD_COL))
    # Tower top frame rail at X_CASE0 (snap-off top panel & top fan bracket removed per Define 7 Compact Builder's Guide,
    # leaving the 40 mm structural rail), plus full 210 mm tower bottom wall (at X_CASE1) and front bezel (at Y_FRONT)
    TOP_RAIL_Z = 0.040
    p.append(Box("case_top_rail", pos=(X_CASE0 + WALL / 2, yc, TOP + TOP_RAIL_Z / 2),
                 size=(WALL, CASE_D, TOP_RAIL_Z), color=CASE_COL))
    p.append(Box("case_bot_wall", pos=(X_CASE1 - WALL / 2, yc, TOP + CASE_Z / 2),
                 size=(WALL, CASE_D, CASE_Z), color=CASE_COL))
    p.append(Box("case_front_bezel", pos=(xc, Y_FRONT - WALL / 2, TOP + CASE_Z / 2),
                 size=(CASE_BODY_X, WALL, CASE_Z), color=CASE_COL))
    # Four 20 mm feet protruding at +x (tower bottom)
    for i, (fy, fz) in enumerate(((Y_REAR + 0.045, 0.030), (Y_REAR + 0.045, CASE_Z - 0.030),
                                  (Y_FRONT - 0.045, 0.030), (Y_FRONT - 0.045, CASE_Z - 0.030))):
        p.append(Box(f"case_foot_{i}", pos=(X_CASE1 + FOOT_X / 2, fy, TOP + fz),
                     size=(FOOT_X, 0.042, 0.026), color=SHROUD_COL))
    # PSU shroud inside +x end of chassis
    p.append(Box("psu_shroud", pos=(0.565, yc, TOP + 0.084), size=(0.096, CASE_D - 2 * WALL, 0.160), color=SHROUD_COL))
    # Rear wall at Y_REAR with dual-slot cutout across Slots 1 & 2 (x = 0.328 .. 0.372, z = 0.026 .. Z_LEDGE)
    yw = Y_REAR + WALL / 2
    x_open0, x_open1 = X_SLOT1 - 0.012, X_SLOT1 + 2 * PITCH - 0.008  # 0.328 .. 0.3726
    p.append(Box("case_rear_near", pos=((X_CASE0 + x_open0) / 2, yw, TOP + CASE_Z / 2),
                 size=(x_open0 - X_CASE0, WALL, CASE_Z), color=CASE_COL))
    p.append(Box("case_rear_far", pos=((x_open1 + X_CASE1) / 2, yw, TOP + CASE_Z / 2),
                 size=(X_CASE1 - x_open1, WALL, CASE_Z), color=CASE_COL))
    p.append(Box("case_rear_upper", pos=((x_open0 + x_open1) / 2, yw, TOP + (Z_LEDGE + CASE_Z) / 2),
                 size=(x_open1 - x_open0, WALL, CASE_Z - Z_LEDGE), color=CASE_COL))
    # Rear I/O shield plate beside CPU/VRM
    p.append(Box("rear_io_shield", pos=(0.252, Y_REAR + WALL + 0.001, TOP + 0.050),
                 size=(0.130, 0.002, 0.044), color=VRM_COL))
    # Bracket mounting ledge (top at Z_LEDGE = 0.1400 m; bracket top flange rests flush on it when seated)
    y_ledge0, y_ledge1 = Y_REAR + WALL, BR_Y - 0.0007  # -0.1910 .. -0.1782
    p.append(Box("slot_ledge", pos=((x_open0 + x_open1) / 2, (y_ledge0 + y_ledge1) / 2, TOP + Z_LEDGE - 0.003),
                 size=(x_open1 - x_open0, y_ledge1 - y_ledge0, 0.006), color=STEEL))
    # Bracket bottom tab pocket (4.4 mm y-slit x 15 mm x-window centred on (X_SLOT1, BR_Y + BRK_T/2))
    ymid = BR_Y + BRK_T / 2  # -0.17705
    slit_h = Z_SLIT_TOP - 0.014
    zc = TOP + 0.014 + slit_h / 2
    p.append(Box("slit_y_out", pos=(X_SLOT1, ymid - 0.0022 - 0.002, zc), size=(0.022, 0.004, slit_h), color=SHROUD_COL))
    p.append(Box("slit_y_in", pos=(X_SLOT1, ymid + 0.0022 + 0.002, zc), size=(0.022, 0.004, slit_h), color=SHROUD_COL))
    p.append(Box("slit_x_lo", pos=(X_SLOT1 - 0.0075 - 0.003, ymid, zc), size=(0.006, 0.0044, slit_h), color=SHROUD_COL))
    p.append(Box("slit_x_hi", pos=(X_SLOT1 + 0.0075 + 0.003, ymid, zc), size=(0.006, 0.0044, slit_h), color=SHROUD_COL))
    return p


def _board_parts() -> list[Box]:
    p = [
        Box("motherboard", pos=(BOARD_X0 + BOARD_DX / 2, BOARD_Y0 + BOARD_DY / 2, TOP + Z_BOARD - BOARD_T / 2),
            size=(BOARD_DX, BOARD_DY, BOARD_T), color=BOARD_COL),
        Box("vrm_heatsink_a", pos=(0.235, -0.158, TOP + Z_BOARD + 0.013), size=(0.082, 0.022, 0.026), color=VRM_COL),
        Box("vrm_heatsink_b", pos=(0.192, -0.105, TOP + Z_BOARD + 0.013), size=(0.020, 0.068, 0.026), color=VRM_COL),
        Box("pch_heatsink", pos=(0.415, 0.015, TOP + Z_BOARD + 0.007), size=(0.052, 0.058, 0.014), color=VRM_COL),
        Box("cpu_cooler", pos=(CPU_POS[0], CPU_POS[1], TOP + CPU_POS[2]), size=CPU_SIZE, color=CPU_COL),
    ]
    for i, dy in enumerate(DIMM_YS):
        p.append(Box(f"dimm_{i}", pos=(DIMM_X0 + DIMM_LEN / 2, dy, TOP + Z_BOARD + DIMM_H / 2),
                     size=(DIMM_LEN, DIMM_W, DIMM_H), color=RAM_COL))
    # Build PCIe x16 Slot 1 and Slot 3 housings (walls + seating floor + end caps + polarisation key ridge)
    yl = SLOT_Y1 - SLOT_Y0
    ym = (SLOT_Y0 + SLOT_Y1) / 2
    zm = TOP + (Z_BOARD + Z_MOUTH) / 2
    wall_t = (CONN_W - CAV_W) / 2  # 0.0023
    for tag, xs in (("s1", X_SLOT1), ("s3", X_SLOT3)):
        for nm, sx in (("near", -1), ("far", 1)):
            p.append(Box(f"{tag}_wall_{nm}", pos=(xs + sx * (CAV_W / 2 + wall_t / 2), ym, zm),
                         size=(wall_t, yl + 0.008, CONN_H), color=SLOT_COL))
        p.append(Box(f"{tag}_floor", pos=(xs, ym, TOP + (Z_BOARD + Z_FLOOR) / 2),
                     size=(CAV_W, yl, Z_FLOOR - Z_BOARD), color=SLOT_COL))
        for nm, yy in (("e0", SLOT_Y0 - 0.002), ("e1", SLOT_Y1 + 0.002)):
            p.append(Box(f"{tag}_{nm}", pos=(xs, yy, zm), size=(CAV_W, 0.004, CONN_H), color=SLOT_COL))
        key_h = SEAT_D - 0.0012
        p.append(Box(f"{tag}_key", pos=(xs, KEY_Y, TOP + Z_FLOOR + key_h / 2),
                     size=(CAV_W, KEY_RW, key_h), color=SLOT_COL))
    return p


OBJECTS = [*_case_parts(), *_board_parts(), *_cradle_parts(), CARD]

TWIN_EXTRA = [
    {"name": "x16_contacts", "kind": "force",
     "real": "PCIe x16 connector's 82 contact pairs wiping the 89 mm gold-finger strip (~30-45 N typical, 94.3 N spec max)",
     "note": "sim: dry vertical sliding resistance up to 18 N at each end of the gold-finger strip inside Slot 1 (36 N total)"},
    {"name": "retention_latch", "kind": "joint",
     "real": "Slot 1's hinged PCIe x16 wing latch cammed upright into the card's rear hook at full seating",
     "note": "sim: latch lever rotates from 38 deg open to 0 deg closed and engages a fixed joint once seated & level"},
    {"name": "grasp_weld", "kind": "joint",
     "real": "parallel finger pads pinching opposite ends of the 1.18 kg RTX A6000's top edge",
     "note": "sim: fixed joint per hand when jaws close on the card's top-edge grip band; releases immediately on open"},
]

_P: dict = {}


# -- USD authoring: RTX A6000 visual/collision details & hinged PCIe latch levers ---------------------------------------
def _author(root: str) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    card = f"{root}/{CARD.name}"
    _P.update(card=card, root=root)

    def look(name, rgb, rough=0.38, metal=0.25):
        path = f"{root}/Looks/{name}"
        if not stage.GetPrimAtPath(path).IsValid():
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(rgb), roughness=rough, metallic=metal)
            cfg.func(path, cfg)
        return path

    def box(parent, name, ctr, size, rgb, collide=True, rough=0.38, metal=0.25):
        path = f"{parent}/{name}"
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
        x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, ctr)))
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        bind_visual_material(path, look(name, rgb, rough, metal), stage=stage)
        if collide:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        return path

    def cyl(parent, name, ctr, radius, length, rgb, rough=0.35, metal=0.2):
        path = f"{parent}/{name}"
        g = UsdGeom.Cylinder.Define(stage, path)
        g.CreateRadiusAttr(float(radius))
        g.CreateHeightAttr(float(length))
        g.CreateAxisAttr("X")
        x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, ctr)))
        bind_visual_material(path, look(name, rgb, rough, metal), stage=stage)
        return path

    # Card frame: origin at body centre; x across thickness (-x backplate, +x blower), y along length (-y bracket), z up
    zb = -BODY_H / 2
    zt = BODY_H / 2
    yb = -CARD_L / 2
    fy0 = yb - BRK_T + FING0

    # Gold PCIe x16 edge segments + retention hook tail (colliding so slot walls & key ridge enforce true registration)
    box(card, "fingers_short", (PCB_DX, fy0 + FING_SHORT / 2, zb - FING_D / 2),
        (PCB_T, FING_SHORT, FING_D), GOLD, rough=0.22, metal=0.85)
    box(card, "fingers_long", (PCB_DX, fy0 + FING_SHORT + KEY_W + FING_LONG / 2, zb - FING_D / 2),
        (PCB_T, FING_LONG, FING_D), GOLD, rough=0.22, metal=0.85)
    hook_y0 = fy0 + FING_L + 0.0020
    box(card, "retention_hook_stem", (PCB_DX, hook_y0 + 0.0020, zb - 0.0025),
        (PCB_T, 0.0040, 0.0050), PCB_EDGE_COL, collide=False)
    box(card, "retention_hook_foot", (PCB_DX, hook_y0 + HOOK_L / 2, zb - FING_D + 0.0018),
        (PCB_T, HOOK_L, 0.0036), PCB_EDGE_COL, collide=False)

    # Recognizable RTX A6000 visual features: champagne-gold frame & top bar, dark backplate, radial blower fan
    box(card, "backplate_vis", (-BODY_T / 2 - 0.0003, 0.0, 0.0), (0.0006, CARD_L - 0.002, BODY_H - 0.002),
        BACK_COL, collide=False, rough=0.35, metal=0.6)
    box(card, "gold_top_trim", (0.003, 0.010, zt + 0.0004), (BODY_T - 0.008, CARD_L - 0.024, 0.0008),
        TRIM_COL, collide=False, rough=0.25, metal=0.8)
    box(card, "gold_front_bezel", (BODY_T / 2 + 0.0004, -0.025, 0.0), (0.0008, 0.150, BODY_H - 0.006),
        TRIM_COL, collide=False, rough=0.25, metal=0.8)
    box(card, "shroud_dark_inset", (BODY_T / 2 + 0.0007, -0.032, 0.0), (0.0006, 0.125, BODY_H - 0.022),
        CARD_COL, collide=False, rough=0.45, metal=0.3)
    cyl(card, "blower_ring", (BODY_T / 2 + 0.0008, 0.082, 0.0), 0.038, 0.0016, TRIM_COL, rough=0.25, metal=0.8)
    cyl(card, "blower_intake", (BODY_T / 2 + 0.0011, 0.082, 0.0), 0.034, 0.0016, FAN_COL, rough=0.6, metal=0.1)
    cyl(card, "blower_hub", (BODY_T / 2 + 0.0015, 0.082, 0.0), 0.012, 0.0018, TRIM_COL, rough=0.25, metal=0.8)

    # Full-height dual-slot stainless-steel I/O bracket (-y end): vertical plate, bent top flange, bottom alignment tab
    bx = PCB_DX - 0.0045 + BRK_X_W / 2
    by = yb - BRK_T / 2
    box(card, "bracket_plate", (bx, by, 0.0), (BRK_X_W, BRK_T, BODY_H), STEEL, collide=True, rough=0.28, metal=0.8)
    box(card, "bracket_flange", (bx, yb - BRK_T - FLANGE_L / 2, zt + BRK_T / 2),
        (BRK_X_W, FLANGE_L, BRK_T), STEEL, collide=True, rough=0.28, metal=0.8)
    box(card, "bracket_tab", (PCB_DX, by, zb - TAB_DROP / 2),
        (TAB_W, BRK_T, TAB_DROP), STEEL, collide=True, rough=0.28, metal=0.8)

    # Hinged white retention latch levers on Slot 1 (starts OPEN 38 deg) and Slot 3 (closed 0 deg)
    for tag, xs, ang0 in (("s1", X_SLOT1, LATCH_OPEN_DEG), ("s3", X_SLOT3, 0.0)):
        lp = f"{root}/{tag}_latch_visual"
        xf = UsdGeom.Xform.Define(stage, lp)
        xo = UsdGeom.Xformable(xf)
        xo.AddTranslateOp().Set(Gf.Vec3d(float(xs), float(LATCH_PIVOT_Y), float(TOP + Z_BOARD + 0.004)))
        xo.AddRotateXOp().Set(float(-ang0))  # negative rot about +x swings +z tip toward +y (away from card)
        box(lp, f"{tag}_lever_arm", (0.0, 0.0, 0.011), (0.007, 0.005, 0.022), LATCH_COL, collide=False, rough=0.3, metal=0.1)
        box(lp, f"{tag}_lever_wing", (0.0, 0.003, 0.021), (0.013, 0.008, 0.004), LATCH_COL, collide=False, rough=0.3, metal=0.1)
        _P[f"{tag}_latch"] = lp

    # Low-friction connector & pocket physics material so accurately aligned parts slide without rigid-facet chatter
    mat = f"{root}/card_material"
    mc = sim_utils.RigidBodyMaterialCfg(static_friction=0.30, dynamic_friction=0.22, restitution=0.0)
    mc.func(mat, mc)
    slick = f"{root}/slot_material"
    ms = sim_utils.RigidBodyMaterialCfg(static_friction=0.08, dynamic_friction=0.06, restitution=0.0)
    ms.func(slick, ms)
    for p in Usd.PrimRange(stage.GetPrimAtPath(card)):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            bind_physics_material(str(p.GetPath()), mat, stage=stage)
    for o in OBJECTS:
        if o.name.startswith(("s1_", "s3_", "slit_", "slot_ledge", "esd_")):
            q = stage.GetPrimAtPath(f"{root}/{o.name}")
            if q and q.IsValid():
                for c in Usd.PrimRange(q):
                    if c.HasAPI(UsdPhysics.CollisionAPI):
                        bind_physics_material(str(c.GetPath()), slick, stage=stage)


def _set_latch_angle(deg_open: float) -> None:
    if _M.get("latch_drawn") == deg_open or "s1_latch" not in _P:
        return
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdGeom

        prim = get_current_stage().GetPrimAtPath(_P["s1_latch"])
        if prim and prim.IsValid():
            x = UsdGeom.Xformable(prim)
            for op in x.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeRotateX:
                    op.Set(float(-deg_open))
            _M["latch_drawn"] = deg_open
    except Exception:  # noqa: BLE001
        pass


def _wrap_cfg(spec, obj):
    from isaaclab.sim.utils import clone

    cfg = obj.spawner_cfg
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
    if getattr(scene.task_object, "_gpu_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        obj = orig(spec)
        if getattr(spec, "name", None) == OBJECTS[-1].name:
            _wrap_cfg(spec, obj)
        return obj

    task_object._gpu_patched = True
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


# -- runtime state ------------------------------------------------------------------------------------------------------
_FRESH = dict(
    env=None, step=0, nan=False, dropped=False, board_hit=False, one_hand=False, one_hand_n=0,
    latched=False, latch_step=None, latch_deg=LATCH_OPEN_DEG, latch_drawn=None,
    welded_left=False, welded_right=False, views=None, subs=0, why="", ghist=None, fric=(0.0, 0.0), g=None,
)
_M: dict = {}


def _reset_model() -> None:
    if "root" in _P:
        _weld_off()
    env = _M.get("env")
    _M.clear()
    _M.update({k: (list(v) if isinstance(v, list) else v) for k, v in _FRESH.items()})
    _M["env"] = env
    _set_latch_angle(LATCH_OPEN_DEG)


_reset_model()


def _qrot(q, v):
    """Rotate v by quaternion q given as (w, x, y, z)."""
    w, x, y, z = q
    u = np.array([x, y, z], dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def _body(objects, name):
    """Rigid body centre (env-local world frame, z includes TOP) and orientation (w, x, y, z)."""
    env = _M.get("env")
    if env is not None and objects is None:
        base = env.unwrapped
        pose = base.arena_world.get_pose_w(name)[0].detach().cpu().numpy().astype(np.float64)
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
        x, y, z, w = pose[3:7]
        return pose[:3] - origin, np.array([w, x, y, z], dtype=np.float64)
    if objects is not None and name in objects:
        o = objects[name]
        x, y, z, w = np.asarray(o.quat_xyzw, dtype=np.float64)
        return np.asarray(o.pos, dtype=np.float64), np.array([w, x, y, z], dtype=np.float64)
    return np.r_[CARD_P0[:2], TOP + CARD_P0[2]], np.array([1.0, 0.0, 0.0, 0.0])


def card_frame(objects=None):
    """Card body centre (table-relative: z above table) and its world axes (cx, cy, cz)."""
    p, q = _body(objects, CARD.name)
    p = p - np.array([0.0, 0.0, TOP])
    return p, q, _qrot(q, [1, 0, 0]), _qrot(q, [0, 1, 0]), _qrot(q, [0, 0, 1])


# Key points in card local frame
FING_A = np.array([PCB_DX, -CARD_L / 2 - BRK_T + FING0, -BODY_H / 2 - FING_D])          # finger bottom, bracket end
FING_B = FING_A + np.array([0.0, FING_L, 0.0])                                           # finger bottom, latch end
TAB_TIP = np.array([PCB_DX, -CARD_L / 2 - BRK_T / 2, -BODY_H / 2 - TAB_DROP])            # bracket tab bottom tip
FLANGE_PT = np.array([PCB_DX, -CARD_L / 2 - BRK_T - FLANGE_L / 2, BODY_H / 2])           # bracket flange underside


def geometry(objects=None) -> dict:
    """Card alignment & seating geometry relative to top PCIe x16 Slot 1."""
    p, q, cx, cy, cz = card_frame(objects)
    a = p + _qrot(q, FING_A)
    b = p + _qrot(q, FING_B)
    tab = p + _qrot(q, TAB_TIP)
    flange = p + _qrot(q, FLANGE_PT)
    ya = BR_Y + FING0
    yb = ya + FING_L
    # Sample card lower points (shroud corners + fingers + tab) for drop & prohibited-component contact checks
    sample_pts = [a, b, tab] + [
        p + _qrot(q, [sx * BODY_T / 2, sy * CARD_L / 2, sz * BODY_H / 2])
        for sx in (-1, 1) for sy in (-1, 0, 1) for sz in (-1, 1)
    ]
    # Pitch (along-slot tilt in y-z plane) and roll (side tilt in x-z plane), plus yaw misalignment from +y
    pitch = math.degrees(math.asin(float(np.clip(cy[2], -1.0, 1.0))))
    roll = math.degrees(math.asin(float(np.clip(cx[2], -1.0, 1.0))))
    yaw = math.degrees(math.atan2(float(cy[0]), float(cy[1])))
    tilt = math.degrees(math.acos(float(np.clip(cz[2], -1.0, 1.0))))
    return dict(
        p=p, q=q, cx=cx, cy=cy, cz=cz, a=a, b=b, tab=tab, flange=flange, pts=sample_pts,
        depth_a=float(Z_MOUTH - a[2]), depth_b=float(Z_MOUTH - b[2]),
        dx_a=float(a[0] - X_SLOT1), dx_b=float(b[0] - X_SLOT1),
        dy_a=float(a[1] - ya), dy_b=float(b[1] - yb),
        dx3_a=float(a[0] - X_SLOT3), dx3_b=float(b[0] - X_SLOT3),
        flange_gap=float(flange[2] - Z_LEDGE),
        pitch=pitch, roll=roll, yaw=yaw, tilt=tilt,
        low=float(min(pt[2] for pt in sample_pts)),
    )


# -- geometric thresholds (identical in task.json and README.md) --------------------------------------------------------
SLOT_TOL_X = 0.0017    # finger within +/-1.7 mm of Slot 1 centre plane (cavity half-width 1.7 mm)
SLOT_IN_DEPTH = 0.0010 # >= 1.0 mm below Z_MOUTH (finger bottom <= 3.625 cm)
SEAT_MIN_DEPTH = 0.0070  # depth >= 7.0 mm (within 1.0 mm of 8.0 mm floor -> <= 1.5 mm gold finger visible above lip!)
SLOT_TOL_Y = 0.0015    # +/-1.5 mm along slot when seated
LEVEL_DEG = 1.5        # card level within 1.5 deg of motherboard normal (tilt <= 1.5 deg and |yaw| <= 1.5 deg)


def in_slot(g: dict, which: str = "a") -> bool:
    return (g[f"depth_{which}"] >= SLOT_IN_DEPTH
            and abs(g[f"dx_{which}"]) <= SLOT_TOL_X
            and abs(g[f"dy_{which}"]) <= 0.0040)


def tab_in(g: dict) -> bool:
    t = g["tab"]
    ymid = BR_Y + BRK_T / 2
    return bool(t[2] <= Z_SLIT_TOP - 0.0030
                and abs(t[0] - X_SLOT1) <= 0.0075
                and abs(t[1] - ymid) <= 0.0022)


def registered(g: dict) -> bool:
    """0.3 stage: bracket tab in rear-wall pocket and both ends of the 89 mm finger strip inside Slot 1."""
    return tab_in(g) and in_slot(g, "a") and in_slot(g, "b") and abs(g["yaw"]) <= 5.0


def seated(g: dict) -> bool:
    """0.6 stage: cumulative with 0.3, seated >=7.0 mm (within 1.0 mm of floor; <=1.5 mm gold showing), level <=1.5 deg."""
    return (registered(g)
            and g["depth_a"] >= SEAT_MIN_DEPTH and g["depth_b"] >= SEAT_MIN_DEPTH
            and abs(g["dy_a"]) <= SLOT_TOL_Y and abs(g["dy_b"]) <= SLOT_TOL_Y
            and g["tilt"] <= LEVEL_DEG and abs(g["yaw"]) <= LEVEL_DEG)


def hits_prohibited_board(g: dict) -> bool:
    """True if card contacts CPU cooler, RAM DIMMs, lower PCIe Slot 3, or motherboard outside Slot 1."""
    # Lower PCIe Slot 3 entry/contact
    if (g["depth_a"] > 0.0002 and abs(g["dx3_a"]) <= 0.0038) or (g["depth_b"] > 0.0002 and abs(g["dx3_b"]) <= 0.0038):
        return True
    # Bounding boxes of CPU cooler and 4 RAM DIMMs (grown by 2 mm contact margin)
    boxes = [
        (CPU_POS[0] - CPU_SIZE[0] / 2 - 0.002, CPU_POS[0] + CPU_SIZE[0] / 2 + 0.002,
         CPU_POS[1] - CPU_SIZE[1] / 2 - 0.002, CPU_POS[1] + CPU_SIZE[1] / 2 + 0.002,
         Z_BOARD + CPU_SIZE[2] + 0.002),
        (DIMM_X0 - 0.002, DIMM_X0 + DIMM_LEN + 0.002,
         DIMM_YS[0] - DIMM_W / 2 - 0.002, DIMM_YS[-1] + DIMM_W / 2 + 0.002,
         Z_BOARD + DIMM_H + 0.002),
    ]
    for pt in g["pts"]:
        x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
        for x0, x1, y0, y1, z1 in boxes:
            if x0 <= x <= x1 and y0 <= y <= y1 and z <= z1:
                return True
        # Touching bare motherboard surface outside Slot 1 / rear tab pocket
        if BOARD_X0 <= x <= BOARD_X0 + BOARD_DX and BOARD_Y0 <= y <= BOARD_Y0 + BOARD_DY:
            if z <= Z_BOARD + 0.002 and not (abs(x - X_SLOT1) <= 0.022 and y <= SLOT_Y1 + 0.012):
                return True
    return False


# -- PhysX substep force model: 36 N total x16 contact insertion resistance (18 N per end) ------------------------------
# Force-limited pad grip (replaces a rigid weld). YAM gripper actuator effort = 50 N squeeze per finger (i2rt
# linear_4310, yam_arena GRIPPER effort). Friction-limited hold = 2 pads x mu 0.5 x 50 N = 50 N. Torque from the
# ~20 x 30 mm pad contact patches: twist about the pad normal = 2 pads x mu 0.5 x 50 N x 10 mm mean radius = 0.5 N m;
# tipping in the pad plane = 2 pads x 50 N x 15 mm half pad length = 1.5 N m.
GRIP_SQUEEZE_N = 50.0
GRIP_MU = 0.5
GRIP_HOLD_N = 2 * GRIP_MU * GRIP_SQUEEZE_N          # 50 N
GRIP_TWIST_NM = 2 * GRIP_MU * GRIP_SQUEEZE_N * 0.010  # 0.5 N m about the pad normal (card x)
GRIP_TIP_NM = 2 * GRIP_SQUEEZE_N * 0.015              # 1.5 N m about the in-plane axes (card y, z)
GRIP_K = 20000.0      # N/m pad shear stiffness (compliant grip, ~1 mm per 20 N)
GRIP_C = 200.0        # N s/m
GRIP_KR = 50.0        # N m/rad
GRIP_CR = 0.5         # N m s/rad
FRIC_END_N = 18.0  # N at each end of the 89 mm gold-finger edge inside Slot 1 (36 N combined)


def _views(mgr):
    import warp as wp

    if _M.get("views") is None:
        sv = mgr.get_physics_sim_view()
        if sv is None or "card" not in _P:
            return None
        v = sv.create_rigid_body_view(_P["card"])
        raw = v.get_transforms()
        _M["dev"] = str(getattr(raw, "device", "cuda:0"))
        _M["idx"] = wp.array(np.array([0], dtype=np.int32), dtype=wp.int32, device=_M["dev"])
        _M["views"] = v
    return _M["views"]


def _drive(mgr) -> None:
    try:
        import warp as wp

        v = _views(mgr)
        if v is None or _M["latched"]:
            return
        _M["subs"] += 1
        T = np.asarray(v.get_transforms().numpy(), dtype=np.float64).reshape(-1)
        V = np.asarray(v.get_velocities().numpy(), dtype=np.float64).reshape(-1)
        if not (np.isfinite(T).all() and np.isfinite(V).all()):
            _M["nan"] = True
            return
        q = np.array([T[6], T[3], T[4], T[5]], dtype=np.float64)
        c = T[:3]
        ox = oy = oz = 0.0
        if _M.get("env") is not None:
            org = _M["env"].unwrapped.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
            ox, oy, oz = float(org[0]), float(org[1]), float(org[2])
        f_tot, t_tot = np.zeros(3), np.zeros(3)
        fr = []
        m_eff = 0.5 * CARD_M
        for loc in (FING_A, FING_B):
            r = _qrot(q, loc)
            pt = c + r
            px, py, pz = pt[0] - ox, pt[1] - oy, pt[2] - oz - TOP
            depth = Z_MOUTH - pz
            if depth <= 0.0003 or abs(px - X_SLOT1) > 0.003 or not (SLOT_Y0 - 0.004 <= py <= SLOT_Y1 + 0.004):
                fr.append(0.0)
                continue
            vel = V[:3] + np.cross(V[3:6], r)
            vz = float(vel[2])
            f = -math.copysign(min(FRIC_END_N, 0.5 * m_eff * abs(vz) * 240.0), vz)
            fv = np.array([0.0, 0.0, f])
            f_tot += fv
            t_tot += np.cross(r, fv)
            fr.append(f)
        _M["fric"] = tuple(fr)
        if np.any(f_tot):
            v.apply_forces_and_torques_at_position(
                wp.array(np.asarray([f_tot], dtype=np.float32), dtype=wp.float32, device=_M["dev"]),
                wp.array(np.asarray([t_tot], dtype=np.float32), dtype=wp.float32, device=_M["dev"]),
                None, _M["idx"], True,
            )
    except Exception as e:  # pragma: no cover
        if _M.get("err") is None:
            _M["err"] = repr(e)
            print(f"[gpu] drive error: {e!r}", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_gpu_hooked", False):
        return
    base_step = mgr.step.__func__

    def step(cls) -> None:
        base_step(cls)
        _drive(cls)

    step._gpu_hooked = True
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


# -- grasp-weld & retention-latch weld (disclosed in README & task.json fidelity) ---------------------------------------
GRIP_TOP_BAND = 0.036  # pad midpoint within top 36 mm of the card's upper edge


def _np(t):
    t = getattr(t, "torch", t)
    return np.asarray(t.detach().cpu().numpy() if hasattr(t, "detach") else t, dtype=np.float64)


def _gripper_path(side: str):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    key = f"gpath_{side}"
    if _P.get(key):
        return _P[key]
    for prim in get_current_stage().Traverse():
        ps = prim.GetPath().pathString
        if "/env_0/" in ps and f"/{side.capitalize()}Arm" in ps and ps.endswith("/gripper") and \
                prim.HasAPI(UsdPhysics.RigidBodyAPI):
            _P[key] = ps
            return ps
    return None


def _quatf(R):
    from pxr import Gf
    from scipy.spatial.transform import Rotation

    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return Gf.Quatf(float(w), float(x), float(y), float(z))


def _closing(rob) -> bool:
    names = list(rob.joint_names)
    q_t = _np(rob.data.joint_pos_target[0]).reshape(-1)
    return float(q_t[names.index("joint7")]) > -0.005


def on_card_top(tip) -> bool:
    """True when pad midpoint ``tip`` (table frame) lies inside the card's upper-edge grip band."""
    p, q, *_ = card_frame()
    R = np.stack([_qrot(q, [1, 0, 0]), _qrot(q, [0, 1, 0]), _qrot(q, [0, 0, 1])], 1)
    loc = R.T @ (np.asarray(tip, dtype=np.float64) - p)
    return bool(
        abs(loc[0]) <= BODY_T / 2 + 0.008
        and abs(loc[1]) <= CARD_L / 2 - 0.008
        and BODY_H / 2 - GRIP_TOP_BAND <= loc[2] <= BODY_H / 2 + 0.008
    )


def _weld_side(side: str) -> None:
    env = _M.get("env")
    if env is None or "card" not in _P or "root" not in _P:
        return
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, Sdf, UsdPhysics
    from yam_arena.constants import FINGER_JOINTS
    from yam_arena.runtime import link_poses, pad_tips_world

    base = env.unwrapped
    rob = base.scene[f"{side}_robot"]
    closing = _closing(rob)
    st = get_current_stage()
    jp = f"{_P['root']}/grasp_weld_{side}"
    j = st.GetPrimAtPath(jp)
    on = bool(_M.get(f"welded_{side}"))
    if on and not closing:
        if j and j.IsValid():
            j.GetAttribute("physics:jointEnabled").Set(False)
        _M[f"welded_{side}"] = False
        print(f"[gpu] {side} weld off at step {_M['step']} (peak grip load {_M.get(f'peak_{side}', 0.0):.1f} N, "
              f"slips {_M.get('slips', 0)})", flush=True)
        return
    if on or not closing:
        return
    o = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    tip = np.asarray(pad_tips_world(rob), dtype=np.float64) - o - np.array([0.0, 0.0, TOP])
    ids = rob.find_joints(FINGER_JOINTS, preserve_order=True)[0]
    gap = float(np.abs(_np(rob.data.joint_pos[0])[ids]).sum())
    if not on_card_top(tip):
        if not _M.get(f"miss_log_{side}"):
            _M[f"miss_log_{side}"] = True
            p_c, q_c, *_ = card_frame()
            R_c = np.stack([_qrot(q_c, [1, 0, 0]), _qrot(q_c, [0, 1, 0]), _qrot(q_c, [0, 0, 1])], 1)
            print(f"[gpu] {side} closing off card band at step {_M['step']}: loc={np.round((R_c.T @ (tip - p_c))*1000,1)}mm gap={gap*1000:.1f}mm", flush=True)
        return
    # Finger-joint gap reads ~5.7 mm above the pad-to-pad distance (pad thickness). Jaws closing on the 38 mm card stall
    # at 43.7-44.8 mm, so a gap inside [BODY_T+3 mm, BODY_T+7.5 mm] while closing means both pads are pressed on the
    # card faces (two-pad contact). A wider gap means the pads have not reached the card; a narrower one means they missed it.
    if not (BODY_T + 0.003 <= gap <= BODY_T + 0.0075):
        return
    kp, kq = _body(None, CARD.name)
    kR = np.stack([_qrot(kq, [1, 0, 0]), _qrot(kq, [0, 1, 0]), _qrot(kq, [0, 0, 1])], 1)
    g = link_poses(rob)["gripper"]
    gp, gR = g[:3, 3] - o, g[:3, :3]
    gpath = _gripper_path(side)
    if gpath is None:
        return
    # Joint frame sits at the pad midpoint on the card (c1, card frame) so forces and torques act at the pads.
    tip_e = tip + np.array([0.0, 0.0, TOP])
    c1 = kR.T @ (tip_e - kp)
    rel_p = gR.T @ (tip_e - gp)
    rel_R = gR.T @ kR
    if not (j and j.IsValid()):
        # Generic 6-DOF joint with spring-damper drives on every axis, each capped at the pad friction/torque limit:
        # under a larger load the drive saturates and the card slips in the jaws (see _grip_slip).
        jj = UsdPhysics.Joint.Define(st, jp)
        jj.CreateBody0Rel().SetTargets([Sdf.Path(gpath)])
        jj.CreateBody1Rel().SetTargets([Sdf.Path(_P["card"])])
        jj.CreateExcludeFromArticulationAttr(True)
        jj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        jj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        prim = jj.GetPrim()
        d2r = np.pi / 180.0
        for ax in ("transX", "transY", "transZ"):
            d = UsdPhysics.DriveAPI.Apply(prim, ax)
            d.CreateTypeAttr("force"); d.CreateStiffnessAttr(GRIP_K); d.CreateDampingAttr(GRIP_C)
            d.CreateMaxForceAttr(GRIP_HOLD_N); d.CreateTargetPositionAttr(0.0)
        for ax, cap in (("rotX", GRIP_TWIST_NM), ("rotY", GRIP_TIP_NM), ("rotZ", GRIP_TIP_NM)):
            d = UsdPhysics.DriveAPI.Apply(prim, ax)   # USD angular gains are per degree
            d.CreateTypeAttr("force"); d.CreateStiffnessAttr(GRIP_KR * d2r); d.CreateDampingAttr(GRIP_CR * d2r)
            d.CreateMaxForceAttr(cap); d.CreateTargetPositionAttr(0.0)
        j = prim
    # Anchor at the current relative pose: no snap at engage.
    j.GetAttribute("physics:localPos1").Set(Gf.Vec3f(*map(float, c1)))
    j.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, rel_p)))
    j.GetAttribute("physics:localRot0").Set(_quatf(rel_R))
    j.GetAttribute("physics:jointEnabled").Set(True)
    _M[f"anchor_{side}"] = (rel_p.copy(), rel_R.copy(), c1.copy())
    _M[f"welded_{side}"] = True
    print(f"[gpu] {side} weld ON at step {_M['step']} (gap={gap*1000:.1f}mm)", flush=True)


def _grip_slip(side: str) -> None:
    """Coulomb-style slip: when the spring load needed to hold the card exceeds the pad limit, the card slides to its
    new place in the jaws (the anchor moves) instead of being held rigidly."""
    if not _M.get(f"welded_{side}") or _M.get(f"anchor_{side}") is None:
        return
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf
    from scipy.spatial.transform import Rotation
    from yam_arena.runtime import link_poses

    base = _M["env"].unwrapped
    rob = base.scene[f"{side}_robot"]
    o = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    kp, kq = _body(None, CARD.name)
    kR = np.stack([_qrot(kq, [1, 0, 0]), _qrot(kq, [0, 1, 0]), _qrot(kq, [0, 0, 1])], 1)
    g = link_poses(rob)["gripper"]
    gp, gR = g[:3, 3] - o, g[:3, :3]
    a_p, a_R, c1 = _M[f"anchor_{side}"]
    rel_p, rel_R = gR.T @ (kp + kR @ c1 - gp), gR.T @ kR
    f_lin = GRIP_K * float(np.linalg.norm(rel_p - a_p))
    rv = Rotation.from_matrix(a_R.T @ rel_R).as_rotvec()   # card-frame rotation error
    t_tw = GRIP_KR * abs(float(rv[0]))
    t_tip = GRIP_KR * float(np.hypot(rv[1], rv[2]))
    _M[f"peak_{side}"] = max(_M.get(f"peak_{side}", 0.0), f_lin)
    if f_lin > GRIP_HOLD_N or t_tw > GRIP_TWIST_NM or t_tip > GRIP_TIP_NM:
        j = get_current_stage().GetPrimAtPath(f"{_P['root']}/grasp_weld_{side}")
        if j and j.IsValid():
            j.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, rel_p)))
            j.GetAttribute("physics:localRot0").Set(_quatf(rel_R))
        _M[f"anchor_{side}"] = (rel_p.copy(), rel_R.copy(), c1)
        _M["slips"] = _M.get("slips", 0) + 1
        if _M["slips"] <= 20 or _M["slips"] % 50 == 0:
            print(f"[gpu] {side} grip slip #{_M['slips']} at step {_M['step']}: load {f_lin:.1f} N, twist {t_tw:.2f} N m, "
                  f"tip {t_tip:.2f} N m", flush=True)


def _latch_weld() -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, Sdf, UsdPhysics

    st = get_current_stage()
    jp = f"{_P['root']}/latch_weld"
    j = st.GetPrimAtPath(jp)
    env = _M["env"].unwrapped
    o = env.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    kp, kq = _body(None, CARD.name)
    if not (j and j.IsValid()):
        jj = UsdPhysics.FixedJoint.Define(st, jp)
        jj.CreateBody1Rel().SetTargets([Sdf.Path(_P["card"])])
        jj.CreateExcludeFromArticulationAttr(True)
        jj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        jj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j = jj.GetPrim()
    j.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, kp + o)))
    j.GetAttribute("physics:localRot0").Set(Gf.Quatf(float(kq[0]), float(kq[1]), float(kq[2]), float(kq[3])))
    j.GetAttribute("physics:jointEnabled").Set(True)


def _weld_off() -> None:
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        for nm in ("grasp_weld_left", "grasp_weld_right", "latch_weld"):
            j = get_current_stage().GetPrimAtPath(f"{_P.get('root')}/{nm}")
            if j and j.IsValid():
                j.GetAttribute("physics:jointEnabled").Set(False)
    except Exception:  # noqa: BLE001
        pass


# -- per-step tracking (latches whole-run violations identically to task.json / README.md) ------------------------------
CRADLE_LOW0 = CARD_Z0 - BODY_H / 2 - TAB_DROP  # ~0.0152 m (initial lowest point = bracket tab tip in cradle)
LIFT_CLEAR_Z = 0.0220    # lowest point > 2.2 cm above table (= >6.8 mm above initial rest) while outside Slot 1
ONE_HAND_STEPS = 10      # 0.5 s at 20 Hz
DROP_FLOOR_Z = 0.0100    # lowest point < 1.0 cm above table (touches table or case floor)


def holders() -> int:
    return int(bool(_M.get("welded_left"))) + int(bool(_M.get("welded_right")))


def _track(objects=None) -> None:
    if _M.get("env") is None and objects is None:
        return
    try:
        g = geometry(objects)
    except Exception:  # noqa: BLE001
        return
    if not np.isfinite(g["p"]).all():
        _M["nan"] = True
        return
    _M["g"] = g
    supported_by_slot = in_slot(g, "a") or in_slot(g, "b") or _M["latched"]
    lifted = g["low"] > LIFT_CLEAR_Z and not supported_by_slot
    if lifted and holders() == 1:
        _M["one_hand_n"] += 1
        if _M["one_hand_n"] > ONE_HAND_STEPS and not _M["one_hand"]:
            _M["one_hand"] = True
            _M["why"] = "card lifted with only one gripper for >0.5 s"
            print(f"[gpu] {_M['why']} at step {_M['step']}", flush=True)
    else:
        _M["one_hand_n"] = 0

    if g["low"] < DROP_FLOOR_Z and not _M["dropped"]:
        _M["dropped"] = True
        _M["why"] = "card touched table or case floor"
        print(f"[gpu] {_M['why']} at step {_M['step']} (low={g['low']:.4f})", flush=True)

    if hits_prohibited_board(g) and not _M["board_hit"]:
        _M["board_hit"] = True
        _M["why"] = "card contacted prohibited motherboard component or lower slot"
        print(f"[gpu] {_M['why']} at step {_M['step']}", flush=True)

    if seated(g) and not _M["latched"] and not (_M["dropped"] or _M["board_hit"] or _M["one_hand"]):
        _M["latched"] = True
        _M["latch_step"] = _M["step"]
        _M["latch_deg"] = 0.0
        if _M.get("env") is not None and "root" in _P:
            _latch_weld()
        print(f"[gpu] seated & latched at step {_M['step']}: depths=({g['depth_a']*1000:.2f}, {g['depth_b']*1000:.2f}) mm "
              f"tilt={g['tilt']:.2f} deg yaw={g['yaw']:.2f} deg", flush=True)

    if _M["latched"]:
        _M["latch_deg"] = 0.0
    elif in_slot(g, "b"):
        # Progressive camming of the latch lever as the card's rear hook approaches seating depth
        frac = float(np.clip((g["depth_b"] - 0.003) / (SEAT_MIN_DEPTH - 0.003), 0.0, 0.75))
        _M["latch_deg"] = LATCH_OPEN_DEG * (1.0 - 0.5 * frac)
    else:
        _M["latch_deg"] = LATCH_OPEN_DEG
    _set_latch_angle(_M["latch_deg"])


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
    if not getattr(cls._task_step, "_gpu_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _M["env"] = self._env
            _M["step"] += 1
            out = base(self, *a, **k)
            try:
                for side in ("left", "right"):
                    _weld_side(side)
                    _grip_slip(side)
                _track()
            except Exception as e:  # noqa: BLE001
                if not _M.get("hook_err"):
                    _M["hook_err"] = True
                    print(f"[gpu] step hook failed: {e!r}", flush=True)
            return out

        _task_step._gpu_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- scoring ------------------------------------------------------------------------------------------------------------
CLEAR = 0.10        # >= 10 cm from card bounding box
STILL_STEPS = 40    # 2.0 s at 20 Hz (the episode ends on success, so the video ends on >= 2 s of the parked end state)
STILL_MOVE = 0.01   # <= 1.0 cm TCP excursion over 2.0 s


def grippers() -> list[np.ndarray]:
    env = _M.get("env")
    if env is None:
        return _M.get("mock_grippers") or []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [
        np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin - np.array([0.0, 0.0, TOP])
        for k in ("left_robot", "right_robot")
    ]


def jaws_open() -> bool:
    env = _M.get("env")
    if env is None:
        return bool(_M.get("mock_jaws_open", False))
    from yam_arena.constants import FINGER_JOINTS

    base = env.unwrapped
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        ids = rob.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        gap = float(np.abs(_np(rob.data.joint_pos[0])[ids]).sum())
        if gap < 0.020:  # each open jaw >= 20 mm aperture
            return False
    return True


def card_dist(pt) -> float:
    """Euclidean distance from point ``pt`` (table frame) to the card's full bounding box (incl. bracket & fingers)."""
    p, q, *_ = card_frame()
    R = np.stack([_qrot(q, [1, 0, 0]), _qrot(q, [0, 1, 0]), _qrot(q, [0, 0, 1])], 1)
    loc = R.T @ (np.asarray(pt, dtype=np.float64) - p)
    lo = np.array([-BODY_T / 2 - 0.005, -CARD_L / 2 - BRK_T - FLANGE_L, -BODY_H / 2 - TAB_DROP])
    hi = np.array([BODY_T / 2 + 0.003, CARD_L / 2, BODY_H / 2 + BRK_T])
    d = np.maximum(np.maximum(lo - loc, loc - hi), 0.0)
    return float(np.linalg.norm(d))


def hands_clear() -> bool:
    g = grippers()
    if len(g) < 2 or holders() > 0 or not jaws_open():
        return False
    hist = _M["ghist"] if _M["ghist"] is not None else []
    _M["ghist"] = hist
    if not hist or hist[-1][0] != _M["step"]:
        hist.append((_M["step"], [p.copy() for p in g]))
        del hist[:-(STILL_STEPS + 1)]
    still = len(hist) > STILL_STEPS and all(
        max(float(np.linalg.norm(h[1][i] - g[i])) for h in hist) <= STILL_MOVE for i in range(len(g))
    )
    return still and all(card_dist(t) >= CLEAR for t in g)


def score(objects) -> float:
    """Cumulative task score in {0.0, 0.3, 0.6, 0.85, 1.0}."""
    if _M.get("env") is None and objects is not None:
        _track(objects)
    if _M["nan"] or _M["dropped"] or _M["board_hit"] or _M["one_hand"]:
        return 0.0
    g = _M.get("g")
    if g is None:
        return 0.0
    if _M["latched"] and seated(g) and _M["latch_deg"] <= 5.0:
        return 1.0 if hands_clear() else 0.85
    if seated(g):
        return 0.6
    if registered(g):
        return 0.3
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def progress(objects) -> dict:
    g = _M.get("g") or {}
    return {
        "registered": float(bool(g) and registered(g)),
        "seated": float(bool(g) and seated(g)),
        "latched": float(_M["latched"]),
        "depth_a_mm": round(g.get("depth_a", 0.0) * 1000, 2),
        "depth_b_mm": round(g.get("depth_b", 0.0) * 1000, 2),
        "tilt_deg": round(g.get("tilt", 0.0), 2),
        "one_hand": float(_M["one_hand"]),
        "dropped": float(_M["dropped"]),
        "board_hit": float(_M["board_hit"]),
    }


VARIATION = {"obj:gpu_card": {"dx": 0.008, "dy": 0.012, "yaw": 0.04}}
