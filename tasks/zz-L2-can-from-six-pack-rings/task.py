"""Take one can of soda out of a six-pack's plastic ring carrier and stand it on a coaster; the other five stay in
their rings.

Scene (metres from the arm-base midpoint on the station laminate counter, 1.20 x 0.80 m at z = 0.7475 m; x forward,
y left; heights above the counter). Six unopened 12 fl oz (355 mL) aluminium soda cans (CMI 211 x 413 body with 202
end, e.g. Coca-Cola Classic 12 fl oz: 66.0 mm body diameter, 122.2 mm total height, 52.4 mm neck groove at z =
113.0-119.0 mm, 54.8 mm top double-seam rim at z = 119.0-122.2 mm, 383 g each, pull-tab pointing +x) stand upright in
a 2 x 3 grid (67.0 mm centre-to-centre pitch, 133.0 x 200.0 mm pack footprint inside a 137.0 x 204.0 mm countertop
outline mark) centred at (0.340, +0.040): near row x = 0.3065, far row x = 0.3735; columns y = -0.0270, +0.0400,
+0.1070. Any one of the six cans may be removed.

The six cans are joined at their necks (z = 116.0 mm) by a commercial Hi-Cone 202-end 2 x 3 LDPE six-pack ring carrier
(0.38 mm / 15 mil low-density polyethylene sheet: six circular holes of 44.0 mm unstretched inner diameter and 8.0 mm
band width, 60.0 mm unstretched outer ring diameter, joined by seven 10.0 mm wide inter-ring webs). Installed on the
cans, each 44.0 mm aperture is pre-stretched 19% to 52.4 mm in the neck groove beneath the 54.8 mm top seam rim. A
round natural cork coaster (IKEA 365+, 100.0 mm diameter, 5.0 mm thick, fixed) sits centred at (0.205, -0.105).

Carrier mechanics: at every 240 Hz physics substep, the massless (4.5 g real) LDPE six-ring sheet solves its
stiffness-weighted horizontal force-and-yaw equilibrium across the engaged cans' necks (z = 116.0 mm), applying
calibrated 3D ring-and-web restoring forces (950 N/m taut web + seated upright ring stiffness, blending to 440 N/m local
ring-band stretch as a can tips onto its 54.8 mm chime, 550 N/m axial chime retention, and 14 N s/m internal relative
carrier damping) to every ringed can and driving a live deformable 3D USD mesh whose rings stretch with the cans and
snap back to their 44.0 mm relaxed inner diameter once peeled off. Pulling straight up catches all round under the
54.8 mm seam rim and lifts neighbours or tears the carrier at >= 28.0 N; pulling with one arm without holding the pack
drags the five cans across the counter (sliding friction mu = 0.35/0.28, slip ~5.3-6.6 N << >=14.1 N peel load). Holding
the pack down with one hand while tipping a can >= 18.0 deg relative to the carrier and stretching its ring >= 32.0 mm
peels the ring cleanly over the rim. Both YAM parallel grippers grasp cans directly via 240 Hz PhysX rigid-body contact
and rubber finger-pad friction (50 N jaw clamp limit).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 600 \\
           --instruction "Take one can of soda out of the six-pack and stand it on the coaster."
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
from yam_arena.task_file import TABLE_TOP, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Take one can of soda out of the six-pack and stand it on the coaster."

# -- 12 fl oz aluminium beverage cans (CMI 211 x 413 body, 202 end) --------------------------------------------------
CAN_R = 0.0330                    # 66.0 mm outer body diameter
CAN_H = 0.1222                    # 122.2 mm total height
BODY_H = 0.1060                   # 0..106.0 mm straight cylindrical wall
SHOULDER_R, SHOULDER_H = 0.0298, 0.0070   # 106.0..113.0 mm tapered shoulder
NECK_R, NECK_H = 0.0262, 0.0060           # 113.0..119.0 mm neck groove (52.4 mm dia)
SEAM_R, SEAM_H = 0.0274, 0.0032           # 119.0..122.2 mm 202 double-seam chime (54.8 mm dia)
CAN_M = 0.383                     # 383 g per full unopened 355 mL can (6-pack = 2.30 kg)
SEAT_Z = 0.1160                   # ring seat centre height above can base (middle of neck groove)

PITCH = 0.0670                    # 67.0 mm centre-to-centre spacing in the 2x3 pack (133 x 200 mm footprint)
PACK_X, PACK_Y = 0.3400, 0.0400   # pack centre (34.0, +4.0) cm
ROWS = (round(PACK_X - PITCH / 2, 6), round(PACK_X + PITCH / 2, 6))       # (0.3065, 0.3735)
COLS = (round(PACK_Y - PITCH, 6), round(PACK_Y, 6), round(PACK_Y + PITCH, 6))  # (-0.0270, 0.0400, 0.1070)

CANS: dict[str, tuple[float, float]] = {}
for _ri, _rx in enumerate(ROWS):
    for _ci, _cy in enumerate(COLS):
        CANS[f"can_{'nf'[_ri]}{'rcl'[_ci]}"] = (_rx, _cy)
CAN_NAMES = tuple(CANS)

# -- Hi-Cone 202-end 2x3 LDPE ring carrier (0.38 mm sheet) -----------------------------------------------------------
RING_IN_FREE = 0.0220             # 44.0 mm unstretched inner aperture diameter (radius 22.0 mm)
RING_BAND_W = 0.0080              # 8.0 mm ring band width -> 60.0 mm unstretched outer ring diameter
RING_OUT_FREE = RING_IN_FREE + RING_BAND_W  # 30.0 mm unstretched outer radius
RING_IN_SEAT = NECK_R + 0.0002    # 26.4 mm inner radius when pre-stretched onto 52.4 mm neck
RING_OUT_SEAT = RING_IN_SEAT + RING_BAND_W  # 34.4 mm outer radius on neck/shoulder (visible around 33.0 mm body)
WEB_W = 0.0100                    # 10.0 mm wide inter-ring webs (7 webs joining adjacent rings)
FILM_VIS_H = 0.0018               # 1.8 mm visual mesh thickness for camera legibility (real sheet 0.38 mm)

# Calibrated 240 Hz spring-peel-tear mechanics
STRETCH_SEAT_K = 950.0            # N/m taut web + seated upright ring stiffness (tilt <= 4 deg)
STRETCH_PEEL_K = 440.0            # N/m local ring-band stretch stiffness when tipped >= 14 deg onto 54.8 mm chime
AXIAL_K = 550.0                   # N/m axial retention under the 54.8 mm seam chime
REL_DAMP_C = 14.0                 # N s/m internal relative carrier damping (zero drag on rigid pack translation)
CAN_LIN_DAMP = 1.0                # 1/s light PhysX rigid-body linear damping on cans
CAN_ANG_DAMP = 4.0                # 1/s PhysX rigid-body angular damping on cans

PEEL_TILT_DEG = 18.0              # deg minimum can-to-carrier tilt to lift outer arc onto top seam
PEEL_STRETCH = 0.0320             # m minimum radial ring stretch (32.0 mm -> >=14.1 N >> ~5.3-6.6 N 5-can slip)
TEAR_FORCE = 28.0                 # N ring/web tension that visibly tears the 0.38 mm LDPE carrier

# -- IKEA 365+ cork coaster (10.0 cm dia, 0.5 cm thick) --------------------------------------------------------------
COASTER_R, COASTER_T = 0.0500, 0.0050
COASTER_XY = (0.2050, -0.1050)    # (20.5, -10.5) cm

# -- colours ----------------------------------------------------------------------------------------------------------
RED = (0.76, 0.05, 0.07)          # crimson soda can body
STRIPE_W = (0.95, 0.95, 0.96)     # white can label band
ALU = (0.78, 0.80, 0.83)          # bright aluminium lid/seam/shoulder
NECK_ALU = (0.42, 0.44, 0.48)     # shadowed neck groove for crisp ring contrast
DARK_SLOT = (0.18, 0.19, 0.21)    # lid score / pull-tab recess
FILM_RGB = (0.95, 0.97, 1.00)     # frosty white LDPE ring carrier
FILM_TORN_RGB = (0.95, 0.35, 0.25)
CORK_DARK = (0.42, 0.26, 0.14)    # toasted cork rim
CORK_TOP = (0.56, 0.38, 0.22)     # natural cork face
MARK_RGB = (0.30, 0.32, 0.35)     # countertop pack outline mark

CAN_OBJS = [
    Cylinder(n, pos=(x, y, TOP + BODY_H / 2), radius=CAN_R, height=BODY_H, mass=CAN_M, color=RED)
    for n, (x, y) in CANS.items()
]
COASTER = Cylinder("coaster", pos=(COASTER_XY[0], COASTER_XY[1], TOP + COASTER_T / 2), radius=COASTER_R,
                   height=COASTER_T, color=CORK_DARK)
OBJECTS = [COASTER, *CAN_OBJS]
_OFFS3 = np.array([[CANS[n][0] - PACK_X, CANS[n][1] - PACK_Y, 0.0] for n in CAN_NAMES], dtype=np.float64)

OUTLINE_W, OUTLINE_L = 2 * CAN_R + PITCH + 0.004, 2 * CAN_R + 2 * PITCH + 0.004  # 137 x 204 mm

TWIN_EXTRA = [
    {"name": "ring_carrier_sheet", "kind": "body",
     "real": "Hi-Cone 202-end 2x3 LDPE six-pack ring carrier (0.38 mm sheet, six 44 mm holes, 8 mm bands, 10 mm webs)",
     "size_mm": [126, 194, 1], "colour": "frosty white", "pose": [PACK_X, PACK_Y, TOP + SEAT_Z]},
    {"name": "ring_on_neck", "kind": "joint",
     "real": "44 mm LDPE ring pre-stretched onto 52.4 mm can neck under 54.8 mm 202 top seam",
     "note": "sim: 240 Hz elastic carrier equilibrium + per-can neck forces (950 N/m seated web, 440 N/m tilted peel, "
             "550 N/m axial, 14 N s/m internal damping); peels at >=18 deg tilt and >=32 mm stretch; tears at >=28 N"},
    {"name": "pack_outline", "kind": "decal", "size_mm": [137, 204], "colour": "dark grey",
     "pose": [PACK_X, PACK_Y, TOP + 0.0004], "note": "2 mm wide countertop reset outline around the 2x3 six-pack"},
]

# -- scoring constants (matched exactly to task.json and README.md) ---------------------------------------------------
LIFT_MAX = 0.0150                 # 1.5 cm max base lift of any remaining ringed can
SLIDE_MAX = 0.0300                # 3.0 cm max horizontal centre slide of any remaining ringed can
TILT_RINGED_MAX = 20.0            # 20.0 deg max tilt of any remaining ringed can at any time
TILT_DROP_MAX = 45.0              # 45.0 deg max tilt of a freed can while unheld (dropped / fallen)
TILT_UPRIGHT = 10.0               # 10.0 deg upright tolerance at the end
BASE_ON_SURF_TOL = 0.0050         # 0.5 cm vertical base-contact tolerance (counter or coaster top)
CLEAR_FROM_PACK = 0.0200          # 2.0 cm horizontal wall-to-wall / wall-to-carrier clearance for 0.6
COASTER_TOL = 0.0170              # 1.7 cm horizontal base-centre tolerance on 10.0 cm coaster (wholly inside: 3.3+1.7=5.0)
CAN_REST_LIN = 0.020              # 2.0 cm/s linear rest threshold for freed can on coaster
CAN_REST_ANG = 0.20               # 0.20 rad/s angular rest threshold for freed can on coaster
GRIPPER_CLEAR = 0.100             # 10.0 cm minimum gripper clearance from cans, carrier and coaster
GRIPPER_SPEED_MAX = 0.020         # 2.0 cm/s maximum gripper speed during 1.0 s end hold
HOLD_STEPS_REQ = 20               # 1.0 s continuous hold at 20 Hz

# -- physical grasp-state detection thresholds (pure PhysX rigid-body pad contact) ------------------------------------
HELD_APERTURE_MAX = 0.072         # m: jaws closed tighter than 72 mm around a 66 mm can body / 54.8 mm seam
HELD_AXIS_TOL = 0.024             # m: pad-midpoint horizontal offset from can axis when straddling the upper wall

# -- deformable carrier mesh topology (6 rings x 20 segments + 7 webs) ------------------------------------------------
N_RING_SEG = 20
_THETA = np.linspace(0.0, 2.0 * math.pi, N_RING_SEG, endpoint=False)
_COS_T, _SIN_T = np.cos(_THETA), np.sin(_THETA)


def _web_pairs() -> list[tuple[int, int]]:
    pairs = []
    for i, a in enumerate(CAN_NAMES):
        for j in range(i + 1, len(CAN_NAMES)):
            b = CAN_NAMES[j]
            if abs(math.hypot(CANS[a][0] - CANS[b][0], CANS[a][1] - CANS[b][1]) - PITCH) < 1e-5:
                pairs.append((i, j))
    return pairs


_WEB_PAIRS = _web_pairs()         # 7 inter-ring webs (4 along y, 3 along x)


def _build_mesh_faces() -> np.ndarray:
    faces: list[tuple[int, int, int]] = []
    # Each ring has N_RING_SEG * 4 vertices: [in_top, out_top, out_bot, in_bot] per angular station
    for ri in range(len(CAN_NAMES)):
        base = ri * N_RING_SEG * 4
        for k in range(N_RING_SEG):
            kn = (k + 1) % N_RING_SEG
            a0, a1, a2, a3 = base + 4 * k, base + 4 * k + 1, base + 4 * k + 2, base + 4 * k + 3
            b0, b1, b2, b3 = base + 4 * kn, base + 4 * kn + 1, base + 4 * kn + 2, base + 4 * kn + 3
            faces += [
                (a0, a1, b1), (a0, b1, b0),  # top
                (a3, b2, a2), (a3, b3, b2),  # bottom
                (a0, b0, b3), (a0, b3, a3),  # inner wall
                (a1, a2, b2), (a1, b2, b1),  # outer wall
            ]
    # Each web has 8 box vertices
    wbase = len(CAN_NAMES) * N_RING_SEG * 4
    for wi in range(len(_WEB_PAIRS)):
        b = wbase + 8 * wi
        # 0..3 at end A (left-top, right-top, right-bot, left-bot), 4..7 at end B
        faces += [
            (b + 0, b + 1, b + 5), (b + 0, b + 5, b + 4),  # top
            (b + 3, b + 6, b + 2), (b + 3, b + 7, b + 6),  # bottom
            (b + 0, b + 4, b + 7), (b + 0, b + 7, b + 3),  # side 1
            (b + 1, b + 2, b + 6), (b + 1, b + 6, b + 5),  # side 2
            (b + 0, b + 3, b + 2), (b + 0, b + 2, b + 1),  # cap A
            (b + 4, b + 5, b + 6), (b + 4, b + 6, b + 7),  # cap B
        ]
    return np.asarray(faces, dtype=np.int32)


_MESH_FACES = _build_mesh_faces()
_P: dict = {}


# -- quaternion & geometry helpers ------------------------------------------------------------------------------------
def _qrot(q, v) -> np.ndarray:
    """Rotate vector v by quaternion q given as (w, x, y, z)."""
    w, x, y, z = q
    u = np.array([x, y, z], dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def _qmat(q) -> np.ndarray:
    """3x3 rotation matrix from quaternion (w, x, y, z)."""
    w, x, y, z = np.asarray(q, dtype=np.float64)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n > 0:
        w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _carrier_mesh_points(cp: np.ndarray, cR: np.ndarray, can_poses: dict[str, tuple[np.ndarray, np.ndarray]],
                         off_map: dict[str, bool], torn_ring: str | None = None) -> np.ndarray:
    """Compute live 3D vertex positions (in env_0 local frame) of the deformable LDPE carrier sheet."""
    nv = len(CAN_NAMES) * N_RING_SEG * 4 + len(_WEB_PAIRS) * 8
    pts = np.zeros((nv, 3), dtype=np.float32)
    hz = FILM_VIS_H / 2.0

    ring_centres = []
    ring_R = []
    ring_rout = []

    for i, n in enumerate(CAN_NAMES):
        ox, oy = CANS[n][0] - PACK_X, CANS[n][1] - PACK_Y
        anchor = cp + cR @ np.array([ox, oy, 0.0])
        is_off = bool(off_map.get(n, False))
        is_torn = (torn_ring == n)
        if not is_off and n in can_poses:
            kp, kq = can_poses[n]
            kR = _qmat(kq)
            neck = kp + kR @ np.array([0.0, 0.0, SEAT_Z - BODY_H / 2])
            # Blend ring plane half-way toward the tilted can neck so outer edge visibly rides up toward the seam
            z_blend = cR[:, 2] + 0.55 * kR[:, 2]
            z_blend /= max(np.linalg.norm(z_blend), 1e-9)
            x_blend = cR[:, 0] - np.dot(cR[:, 0], z_blend) * z_blend
            x_blend /= max(np.linalg.norm(x_blend), 1e-9)
            y_blend = np.cross(z_blend, x_blend)
            Rb = np.column_stack([x_blend, y_blend, z_blend])
            rin, rout = RING_IN_SEAT, RING_OUT_SEAT
            ctr_in = neck
            # Outer rim stretches asymmetrically between neck and carrier anchor so web pull is visually taut
            ctr_out = 0.72 * neck + 0.28 * anchor
        else:
            Rb = cR
            rin, rout = RING_IN_FREE, RING_OUT_FREE
            ctr_in = anchor - np.array([0.0, 0.0, 0.002 if is_torn else 0.0])
            ctr_out = ctr_in

        ring_centres.append(ctr_out)
        ring_R.append(Rb)
        ring_rout.append(rout)

        base = i * N_RING_SEG * 4
        for k in range(N_RING_SEG):
            d_local = np.array([_COS_T[k], _SIN_T[k], 0.0])
            # If torn on this ring, open a visible split on the outer corner
            if is_torn and k in (9, 10):
                d_local = np.array([_COS_T[8], _SIN_T[8], -0.25])
            p_in = ctr_in + Rb @ (rin * d_local)
            p_out = ctr_out + Rb @ (rout * d_local)
            nrm = Rb[:, 2] * hz
            pts[base + 4 * k + 0] = p_in + nrm
            pts[base + 4 * k + 1] = p_out + nrm
            pts[base + 4 * k + 2] = p_out - nrm
            pts[base + 4 * k + 3] = p_in - nrm

    wbase = len(CAN_NAMES) * N_RING_SEG * 4
    hw = WEB_W / 2.0
    for wi, (ia, ib) in enumerate(_WEB_PAIRS):
        na, nb = CAN_NAMES[ia], CAN_NAMES[ib]
        ca, cb = ring_centres[ia], ring_centres[ib]
        d = cb - ca
        dist = float(np.linalg.norm(d))
        u = d / max(dist, 1e-9)
        nrm = 0.5 * (ring_R[ia][:, 2] + ring_R[ib][:, 2])
        nrm /= max(np.linalg.norm(nrm), 1e-9)
        side = np.cross(nrm, u)
        side /= max(np.linalg.norm(side), 1e-9)
        pa = ca + u * (ring_rout[ia] - 0.0018)
        pb = cb - u * (ring_rout[ib] - 0.0018)
        if torn_ring in (na, nb):
            # Visibly sever the web connected to a torn ring
            mid = 0.45 * pa + 0.55 * pb - np.array([0.0, 0.0, 0.008])
            pb = mid
        b = wbase + 8 * wi
        pts[b + 0] = pa + side * hw + nrm * hz
        pts[b + 1] = pa - side * hw + nrm * hz
        pts[b + 2] = pa - side * hw - nrm * hz
        pts[b + 3] = pa + side * hw - nrm * hz
        pts[b + 4] = pb + side * hw + nrm * hz
        pts[b + 5] = pb - side * hw + nrm * hz
        pts[b + 6] = pb - side * hw - nrm * hz
        pts[b + 7] = pb + side * hw - nrm * hz

    return pts


def _initial_mesh_points() -> np.ndarray:
    cp0 = np.array([PACK_X, PACK_Y, TOP + SEAT_Z], dtype=np.float64)
    cR0 = np.eye(3, dtype=np.float64)
    poses0 = {
        n: (np.array([x, y, TOP + BODY_H / 2], dtype=np.float64), np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64))
        for n, (x, y) in CANS.items()
    }
    return _carrier_mesh_points(cp0, cR0, poses0, {})


# -- USD authoring ----------------------------------------------------------------------------------------------------
def _author(root: str) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, Vt
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    _P["root"] = root
    for n in CAN_NAMES:
        _P[n] = f"{root}/{n}"

    def look(name, rgb, rough=0.38, metal=0.0, opacity=None):
        path = f"{root}/Looks/sp_{name}"
        if not stage.GetPrimAtPath(path).IsValid():
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(rgb), roughness=rough, metallic=metal)
            if opacity is not None:
                cfg.opacity = opacity
            cfg.func(path, cfg)
        return path

    def cyl(parent, name, pos, radius, height, lk, collide):
        g = UsdGeom.Cylinder.Define(stage, f"{parent}/{name}")
        g.CreateRadiusAttr(float(radius))
        g.CreateHeightAttr(float(height))
        g.CreateAxisAttr("Z")
        UsdGeom.Xformable(g).AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        bind_visual_material(str(g.GetPath()), lk, stage=stage)
        if collide:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        return str(g.GetPath())

    def box(parent, name, pos, size, yaw, lk, collide=False):
        g = UsdGeom.Cube.Define(stage, f"{parent}/{name}")
        g.CreateSizeAttr(1.0)
        x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        if yaw:
            x.AddRotateZOp().Set(float(math.degrees(yaw)))
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        bind_visual_material(str(g.GetPath()), lk, stage=stage)
        if collide:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        return str(g.GetPath())

    alu_lk = look("can_alu", ALU, rough=0.26, metal=0.82)
    red_lk = look("can_red_grip", RED, rough=0.38, metal=0.15)
    neck_lk = look("can_neck", NECK_ALU, rough=0.35, metal=0.70)
    slot_lk = look("can_slot", DARK_SLOT, rough=0.50, metal=0.30)
    stripe_lk = look("can_stripe", STRIPE_W, rough=0.32, metal=0.0)
    film_lk = look("carrier_ldpe", FILM_RGB, rough=0.28, metal=0.0)
    cork_lk = look("coaster_face", CORK_TOP, rough=0.82, metal=0.0)
    mark_lk = look("pack_outline", MARK_RGB, rough=0.70, metal=0.0)

    # Can top geometry (upper grip sleeve, shoulder, 52.4 mm neck groove, 54.8 mm 202 double-seam rim, +x pull-tab)
    zb = -BODY_H / 2.0
    grip_h = 0.0360
    for n in CAN_NAMES:
        p = _P[n]
        cyl(p, "grip_wall", (0.0, 0.0, zb + BODY_H - grip_h / 2), CAN_R + 0.00015, grip_h, red_lk, True)
        cyl(p, "shoulder", (0.0, 0.0, zb + BODY_H + SHOULDER_H / 2), SHOULDER_R, SHOULDER_H, alu_lk, True)
        cyl(p, "neck", (0.0, 0.0, zb + BODY_H + SHOULDER_H + NECK_H / 2), NECK_R, NECK_H, neck_lk, True)
        cyl(p, "seam", (0.0, 0.0, zb + CAN_H - SEAM_H / 2), SEAM_R, SEAM_H, alu_lk, True)
        cyl(p, "lid_panel", (0.0, 0.0, zb + CAN_H - 0.0014), SEAM_R - 0.0022, 0.0012, alu_lk, False)
        # Stay-on pull-tab & drink opening oriented along +x so can yaw is visually unambiguous
        box(p, "drink_score", (0.010, 0.0, zb + CAN_H - 0.0005), (0.011, 0.009, 0.0006), 0.0, slot_lk, False)
        box(p, "pull_tab", (-0.003, 0.0, zb + CAN_H - 0.0002), (0.019, 0.010, 0.0009), 0.0, alu_lk, False)
        # Crisp white label band round mid-body so cans read immediately as branded soda cans
        cyl(p, "label_band", (0.0, 0.0, 0.008), CAN_R + 0.0003, 0.016, stripe_lk, False)
        cyl(p, "base_chime", (0.0, 0.0, zb + 0.0025), CAN_R - 0.0035, 0.005, alu_lk, False)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(p))
        rb.CreateSolverPositionIterationCountAttr().Set(16)
        rb.CreateSolverVelocityIterationCountAttr().Set(4)
        rb.CreateLinearDampingAttr().Set(float(CAN_LIN_DAMP))
        rb.CreateAngularDampingAttr().Set(float(CAN_ANG_DAMP))

    # Cork coaster inner face visual + high-friction cork physics material
    cst_path = f"{root}/{COASTER.name}"
    cyl(cst_path, "cork_face", (0.0, 0.0, 0.0004), COASTER_R - 0.0035, COASTER_T, cork_lk, False)

    # Countertop reset outline (137 x 204 mm rectangular frame around the 2x3 six-pack, visual only)
    mk_root = f"{root}/pack_outline"
    UsdGeom.Xform.Define(stage, mk_root)
    bw = 0.0022
    box(mk_root, "near", (PACK_X - OUTLINE_W / 2, PACK_Y, TOP + 0.0004), (bw, OUTLINE_L, 0.0006), 0.0, mark_lk, False)
    box(mk_root, "far", (PACK_X + OUTLINE_W / 2, PACK_Y, TOP + 0.0004), (bw, OUTLINE_L, 0.0006), 0.0, mark_lk, False)
    box(mk_root, "right", (PACK_X, PACK_Y - OUTLINE_L / 2, TOP + 0.0004), (OUTLINE_W, bw, 0.0006), 0.0, mark_lk, False)
    box(mk_root, "left", (PACK_X, PACK_Y + OUTLINE_L / 2, TOP + 0.0004), (OUTLINE_W, bw, 0.0006), 0.0, mark_lk, False)

    mesh_path = f"{root}/ring_carrier_mesh"
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateFaceVertexCountsAttr([3] * len(_MESH_FACES))
    mesh.CreateFaceVertexIndicesAttr(_MESH_FACES.ravel().tolist())
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(_initial_mesh_points()))
    bind_visual_material(mesh_path, film_lk, stage=stage)
    _P["carrier_mesh"] = mesh_path
    _P["film_lk"] = film_lk
    _P["film_torn_lk"] = look("carrier_torn", FILM_TORN_RGB, rough=0.35, metal=0.0)

    # Physics materials: lower can on laminate counter (mu 0.35 / 0.28), upper grasp zone vs rubber gripper pads
    # (mu 1.15 / 1.00, matching nitrile-on-aluminium), and high-grip cork coaster (mu 0.75 / 0.65)
    can_mat = f"{root}/can_material"
    mc = sim_utils.RigidBodyMaterialCfg(
        static_friction=0.35, dynamic_friction=0.28, restitution=0.0, friction_combine_mode="min"
    )
    mc.func(can_mat, mc)
    can_grip_mat = f"{root}/can_grip_material"
    mg = sim_utils.RigidBodyMaterialCfg(
        static_friction=1.15, dynamic_friction=1.00, restitution=0.0, friction_combine_mode="max"
    )
    mg.func(can_grip_mat, mg)
    cork_mat = f"{root}/cork_material"
    cc = sim_utils.RigidBodyMaterialCfg(
        static_friction=0.75, dynamic_friction=0.65, restitution=0.0, friction_combine_mode="max"
    )
    cc.func(cork_mat, cc)
    grip_parts = {"grip_wall", "shoulder", "neck", "seam"}
    for n in CAN_NAMES:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(_P[n])):
            pname = prim.GetName()
            is_grip = pname in grip_parts
            if prim.HasAPI(UsdPhysics.MaterialAPI):
                m_api = UsdPhysics.MaterialAPI(prim)
                m_api.CreateStaticFrictionAttr().Set(1.15 if is_grip else 0.35)
                m_api.CreateDynamicFrictionAttr().Set(1.00 if is_grip else 0.28)
                m_api.CreateRestitutionAttr().Set(0.0)
                PhysxSchema.PhysxMaterialAPI.Apply(prim).CreateFrictionCombineModeAttr().Set("max" if is_grip else "min")
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                pc = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                pc.CreateContactOffsetAttr().Set(0.005)
                pc.CreateRestOffsetAttr().Set(0.0)
                bind_physics_material(str(prim.GetPath()), can_grip_mat if is_grip else can_mat, stage=stage)
    for prim in Usd.PrimRange(stage.GetPrimAtPath(cst_path)):
        if prim.HasAPI(UsdPhysics.MaterialAPI):
            m_api = UsdPhysics.MaterialAPI(prim)
            m_api.CreateStaticFrictionAttr().Set(0.75)
            m_api.CreateDynamicFrictionAttr().Set(0.65)
            m_api.CreateRestitutionAttr().Set(0.0)
            PhysxSchema.PhysxMaterialAPI.Apply(prim).CreateFrictionCombineModeAttr().Set("max")
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            bind_physics_material(str(prim.GetPath()), cork_mat, stage=stage)
    tbl_prim = stage.GetPrimAtPath(f"{root}/table")
    if tbl_prim and tbl_prim.IsValid():
        bind_physics_material(f"{root}/table", can_mat, stage=stage, stronger_than_descendants=True)


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
    if getattr(scene.task_object, "_sixpack_v2_patched", False):
        return
    import isaaclab.sim as sim_utils

    orig = scene.task_object

    def task_object(spec):
        obj = orig(spec)
        sname = getattr(spec, "name", None)
        scfg = getattr(obj, "spawner_cfg", None)
        rp = getattr(scfg, "rigid_props", None)
        if scfg is not None and sname in CAN_NAMES:
            scfg.physics_material = sim_utils.RigidBodyMaterialCfg(
                static_friction=0.35, dynamic_friction=0.28, restitution=0.0, friction_combine_mode="min"
            )
            if rp is not None:
                rp.linear_damping = CAN_LIN_DAMP
                rp.angular_damping = CAN_ANG_DAMP
            obj.object_cfg = obj._init_object_cfg()
        elif scfg is not None and sname == COASTER.name:
            scfg.physics_material = sim_utils.RigidBodyMaterialCfg(
                static_friction=0.75, dynamic_friction=0.65, restitution=0.0, friction_combine_mode="max"
            )
            obj.object_cfg = obj._init_object_cfg()
        if sname == OBJECTS[-1].name:
            _wrap_cfg(spec, obj)
        return obj

    task_object._sixpack_v2_patched = True
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


# -- episode state & physics hooks ------------------------------------------------------------------------------------
_FRESH = dict(
    env=None,
    active_substep=False,
    substeps_this_step=0,
    step=0,
    nan=False,
    off={},
    ever_off=[],
    stretch={},
    axial={},
    tilt={},
    force={},
    peak_force=0.0,
    torn=False,
    torn_ring=None,
    torn_visual_pending=False,
    start=None,
    max_lift_by={},
    max_slide_by={},
    max_tilt_by={},
    max_lift=0.0,
    max_slide=0.0,
    max_tilt5=0.0,
    dropped=False,
    held_left=None,
    held_right=None,
    carrier_p=None,
    carrier_R=None,
    prev_grip=None,
    grip_speed=99.0,
    full_hold_steps=0,
)
_M: dict = {}


def _np(t) -> np.ndarray:
    x = getattr(t, "torch", t)
    if hasattr(x, "detach"):
        return np.asarray(x.detach().cpu().numpy(), dtype=np.float64)
    return np.asarray(x, dtype=np.float64)


def _body(objects, name: str) -> tuple[np.ndarray, np.ndarray]:
    """Body position (env_0 m) and orientation quaternion (w, x, y, z)."""
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


def _body_vel_vec(objects, name: str) -> tuple[np.ndarray, np.ndarray]:
    """(lin_vel_w, ang_vel_w) 3-vectors of rigid object ``name``."""
    env = _M.get("env")
    if env is not None:
        base = env.unwrapped
        obj = base.scene[name]
        lv = _np(obj.data.root_lin_vel_w[0]).reshape(3)
        av = _np(obj.data.root_ang_vel_w[0]).reshape(3)
        return lv, av
    o = objects[name]
    lv = np.asarray(getattr(o, "lin_vel", np.zeros(3)), dtype=np.float64)
    av = np.asarray(getattr(o, "ang_vel", np.zeros(3)), dtype=np.float64)
    return lv, av


def _body_vel(objects, name: str) -> tuple[float, float]:
    """(linear_speed_m_s, angular_speed_rad_s) of rigid object ``name``."""
    lv, av = _body_vel_vec(objects, name)
    return float(np.linalg.norm(lv)), float(np.linalg.norm(av))


def _clear_wrenches() -> None:
    env = _M.get("env")
    if env is None:
        return
    try:
        import torch

        base = env.unwrapped
        z = torch.zeros((1, 1, 3), dtype=torch.float32, device=base.device)
        for n in CAN_NAMES:
            base.scene.rigid_objects[n].permanent_wrench_composer.set_forces_and_torques(z, z, is_global=True)
    except Exception:  # noqa: BLE001
        pass


def _reset_model() -> None:
    _clear_wrenches()
    if "root" in _P:
        try:
            if _P.get("carrier_mesh") and _P.get("film_lk"):
                from isaaclab.sim.utils import bind_visual_material
                from isaaclab.sim.utils.stage import get_current_stage

                bind_visual_material(_P["carrier_mesh"], _P["film_lk"], stage=get_current_stage())
        except Exception:  # noqa: BLE001
            pass
    env = _M.get("env")
    _M.clear()
    for k, v in _FRESH.items():
        _M[k] = v.copy() if isinstance(v, (dict, list)) else v
    _M["env"] = env
    _M["carrier_p"] = np.array([PACK_X, PACK_Y, TOP + SEAT_Z], dtype=np.float64)
    _M["carrier_R"] = np.eye(3, dtype=np.float64)


def _ring_k(tilt_deg: float) -> float:
    """Smooth radial stiffness blend from taut seated ring/web (<=4 deg) to local chime-peel stretch (>=14 deg)."""
    alpha = min(1.0, max(0.0, (tilt_deg - 4.0) / 10.0))
    return STRETCH_SEAT_K + alpha * (STRETCH_PEEL_K - STRETCH_SEAT_K)


def _quat_tilt_deg(kq: np.ndarray) -> float:
    return math.degrees(math.acos(float(np.clip(_qrot(kq, [0.0, 0.0, 1.0])[2], -1.0, 1.0))))


def _carrier_equilibrium(poses: dict[str, tuple[np.ndarray, np.ndarray]],
                         off_map: dict[str, bool]) -> tuple[np.ndarray, np.ndarray]:
    """Stiffness-weighted 2D Procrustes yaw + mean-height equilibrium (cp, cR) of the 4.5 g LDPE sheet."""
    idx = [i for i, n in enumerate(CAN_NAMES) if not off_map.get(n)]
    if not idx:
        cp = _M.get("carrier_p")
        cR = _M.get("carrier_R")
        return (
            cp if cp is not None else np.array([PACK_X, PACK_Y, TOP + SEAT_Z], dtype=np.float64),
            cR if cR is not None else np.eye(3, dtype=np.float64),
        )
    necks = np.array(
        [poses[CAN_NAMES[i]][0] + _qrot(poses[CAN_NAMES[i]][1], [0.0, 0.0, SEAT_Z - BODY_H / 2]) for i in idx],
        dtype=np.float64,
    )
    if len(idx) == 1:
        cR = np.eye(3, dtype=np.float64)
        cp = necks[0] - _OFFS3[idx[0]]
        return cp, cR
    w = np.array([_ring_k(_quat_tilt_deg(poses[CAN_NAMES[i]][1])) for i in idx], dtype=np.float64)
    W = w / max(float(w.sum()), 1e-9)
    P_bar = np.sum(W[:, None] * _OFFS3[idx, :2], axis=0)
    Q_bar = np.sum(W[:, None] * necks[:, :2], axis=0)
    P = _OFFS3[idx, :2] - P_bar
    Q = necks[:, :2] - Q_bar
    c_val = float(np.sum(w * (P[:, 0] * Q[:, 0] + P[:, 1] * Q[:, 1])))
    s_val = float(np.sum(w * (P[:, 0] * Q[:, 1] - P[:, 1] * Q[:, 0])))
    yaw = math.atan2(s_val, max(c_val, 1e-9))
    cy, sy = math.cos(yaw), math.sin(yaw)
    cR = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    cp = np.array([0.0, 0.0, float(necks[:, 2].mean())], dtype=np.float64)
    cp[:2] = Q_bar - (cR[:2, :2] @ P_bar)
    return cp, cR


def _carrier_state(objects) -> tuple[np.ndarray, np.ndarray]:
    poses = {n: _body(objects, n) for n in CAN_NAMES}
    return _carrier_equilibrium(poses, _M.get("off", {}))


def ring_kinematics(cp: np.ndarray, cR: np.ndarray, kp: np.ndarray, kq: np.ndarray,
                    can_name: str) -> tuple[float, float, float, float, np.ndarray, np.ndarray]:
    """Return (stretch_m, axial_m, tilt_deg, force_N, vec_rad, nz_car) of ``can_name``'s ring relative to carrier."""
    ox, oy = CANS[can_name][0] - PACK_X, CANS[can_name][1] - PACK_Y
    seat_car = cp + cR @ np.array([ox, oy, 0.0])
    nz_car = cR[:, 2]
    seat_can = kp + _qrot(kq, [0.0, 0.0, SEAT_Z - BODY_H / 2])
    ax_can = _qrot(kq, [0.0, 0.0, 1.0])
    delta = seat_can - seat_car
    d_ax = float(np.dot(delta, nz_car))
    vec_rad = delta - d_ax * nz_car
    d_rad = float(np.linalg.norm(vec_rad))
    tilt_deg = math.degrees(math.acos(float(np.clip(np.dot(ax_can, nz_car), -1.0, 1.0))))
    k_rad = _ring_k(tilt_deg)
    force = math.hypot(k_rad * d_rad, AXIAL_K * abs(d_ax))
    return d_rad, d_ax, tilt_deg, force, vec_rad, nz_car


def _update_carrier_mesh(cp: np.ndarray, cR: np.ndarray, poses: dict[str, tuple[np.ndarray, np.ndarray]]) -> None:
    if not _P.get("carrier_mesh"):
        return
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom, Vt

    stage = get_current_stage()
    prim = stage.GetPrimAtPath(_P["carrier_mesh"])
    if not (prim and prim.IsValid()):
        return
    pts = _carrier_mesh_points(cp, cR, poses, _M["off"], _M.get("torn_ring"))
    UsdGeom.Mesh(prim).GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pts))


def _update_held_state(poses: dict[str, tuple[np.ndarray, np.ndarray]]) -> None:
    """Detect physical parallel-jaw grasps from measured jaw aperture and finger-pad positions."""
    env = _M.get("env")
    if env is None:
        return
    from yam_arena.runtime import pad_tips_world, tcp_world

    base = env.unwrapped
    origin = _np(base.scene.env_origins[0]).reshape(-1)
    for side in ("left", "right"):
        rob = base.scene[f"{side}_robot"]
        names = list(rob.joint_names)
        q = _np(rob.data.joint_pos[0]).reshape(-1)
        aperture = abs(float(q[names.index("joint7")])) + abs(float(q[names.index("joint8")]))
        if aperture > HELD_APERTURE_MAX:
            _M[f"held_{side}"] = None
            continue
        mid = 0.5 * ((np.asarray(tcp_world(rob), dtype=np.float64) - origin)
                     + (np.asarray(pad_tips_world(rob), dtype=np.float64) - origin))
        best_n = None
        best_d = HELD_AXIS_TOL
        top_z = CAN_H - BODY_H / 2 + 0.012
        bot_z = -BODY_H / 2
        for n, (kp, kq) in poses.items():
            loc = _qmat(kq).T @ (mid - kp)
            if bot_z <= float(loc[2]) <= top_z:
                d_xy = math.hypot(float(loc[0]), float(loc[1]))
                if d_xy <= best_d:
                    best_d = d_xy
                    best_n = n
        _M[f"held_{side}"] = best_n


def held_by(name: str) -> bool:
    return name in (_M.get("held_left"), _M.get("held_right"))


def gripper_points() -> np.ndarray | None:
    """Return (6, 3) array of [tcp, pad_left, pad_right] for both arms in env_0 frame, or None if unavailable."""
    env = _M.get("env")
    if env is None:
        return _M.get("mock_grip_pts")
    try:
        from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

        base = env.unwrapped
        origin = _np(base.scene.env_origins[0]).reshape(-1)
        pts = []
        for side in ("left_robot", "right_robot"):
            rob = base.scene[side]
            tcp = np.asarray(tcp_world(rob), dtype=np.float64) - origin
            tip_mid = np.asarray(pad_tips_world(rob), dtype=np.float64) - origin
            lp = link_poses(rob)
            q = _np(rob.data.joint_pos[0]).reshape(-1)
            names = list(rob.joint_names)
            ap = abs(float(q[names.index("joint7")])) + abs(float(q[names.index("joint8")]))
            half = 0.5 * ap * lp["gripper"][:3, 0]
            pts.extend([tcp, tip_mid + half, tip_mid - half])
        arr = np.asarray(pts, dtype=np.float64)
        return arr if arr.shape == (6, 3) and np.isfinite(arr).all() else None
    except Exception:  # noqa: BLE001
        return None


def grippers_tcp() -> list[np.ndarray]:
    pts = gripper_points()
    if pts is None:
        return []
    return [pts[0], pts[3]]


def _tilt_deg(q) -> float:
    return math.degrees(math.acos(float(np.clip(_qrot(q, [0.0, 0.0, 1.0])[2], -1.0, 1.0))))


def _seg_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(float(np.dot(ab, ab)), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _cyl_surface_dist(p: np.ndarray, pos: np.ndarray, q: np.ndarray, z_lo: float, z_hi: float, radius: float) -> float:
    ax = _qrot(q, [0.0, 0.0, 1.0])
    c_mid = pos + ax * (0.5 * (z_lo + z_hi))
    rel = p - c_mid
    dz_signed = float(np.dot(rel, ax))
    d_rad = float(np.linalg.norm(rel - dz_signed * ax))
    half_h = 0.5 * (z_hi - z_lo)
    over_r = max(0.0, d_rad - radius)
    over_z = max(0.0, abs(dz_signed) - half_h)
    return math.hypot(over_r, over_z)


def min_gripper_clearance(objects) -> float:
    """Minimum distance (m) from any gripper TCP/pad point to any can, the carrier, or the coaster."""
    pts = gripper_points()
    if pts is None:
        return -1.0
    best = float("inf")
    for n in CAN_NAMES:
        kp, kq = _body(objects, n)
        for p in pts:
            best = min(best, _cyl_surface_dist(p, kp, kq, -BODY_H / 2, CAN_H - BODY_H / 2, CAN_R))
    cp, cR = _carrier_state(objects)
    # Bounding box of the 2x3 carrier sheet in carrier frame
    half_car = np.array([PITCH / 2 + RING_OUT_SEAT, PITCH + RING_OUT_SEAT, 0.004])
    for p in pts:
        loc = np.abs(cR.T @ (p - cp)) - half_car
        best = min(best, float(np.linalg.norm(np.maximum(loc, 0.0))))
        # Coaster cylinder
        dxy_c = max(0.0, math.hypot(p[0] - COASTER_XY[0], p[1] - COASTER_XY[1]) - COASTER_R)
        dz_c = max(0.0, abs(p[2] - (TOP + COASTER_T / 2)) - COASTER_T / 2)
        best = min(best, math.hypot(dxy_c, dz_c))
    return best


def freed_can() -> str | None:
    off = [n for n in CAN_NAMES if _M["off"].get(n)]
    return off[0] if len(off) == 1 else None


def cleared_pack(objects, n: str) -> bool:
    kp, _ = _body(objects, n)
    cp, cR = _carrier_state(objects)
    for m in CAN_NAMES:
        ox, oy = CANS[m][0] - PACK_X, CANS[m][1] - PACK_Y
        seat_car = cp + cR @ np.array([ox, oy, 0.0])
        r_ring = RING_OUT_FREE if m == n else RING_OUT_SEAT
        if math.hypot(kp[0] - seat_car[0], kp[1] - seat_car[1]) < CAN_R + r_ring + CLEAR_FROM_PACK:
            return False
        if m == n:
            continue
        mp, _ = _body(objects, m)
        if math.hypot(kp[0] - mp[0], kp[1] - mp[1]) < 2.0 * CAN_R + CLEAR_FROM_PACK:
            return False
    return True


def pack_five_upright(objects) -> bool:
    for n in CAN_NAMES:
        if _M["off"].get(n):
            continue
        kp, kq = _body(objects, n)
        base_z = kp[2] - (BODY_H / 2) * _qrot(kq, [0.0, 0.0, 1.0])[2]
        if _tilt_deg(kq) > TILT_UPRIGHT or abs(base_z - TOP) > BASE_ON_SURF_TOL:
            return False
    return True


def freed_on_coaster(objects, n: str) -> bool:
    kp, kq = _body(objects, n)
    base_pos = kp - _qrot(kq, [0.0, 0.0, BODY_H / 2])
    base_z = float(base_pos[2])
    dxy = math.hypot(float(base_pos[0]) - COASTER_XY[0], float(base_pos[1]) - COASTER_XY[1])
    lv, av = _body_vel(objects, n)
    return bool(
        dxy <= COASTER_TOL
        and _tilt_deg(kq) <= TILT_UPRIGHT
        and abs(base_z - (TOP + COASTER_T)) <= BASE_ON_SURF_TOL
        and lv <= CAN_REST_LIN
        and av <= CAN_REST_ANG
        and not held_by(n)
    )


def _substep_carrier(base) -> None:
    """Run 240 Hz elastic carrier equilibrium, peel/tear transitions, and GPU ring force updates before sim.step()."""
    if not _M.get("active_substep"):
        return
    import torch

    origin = _np(base.scene.env_origins[0]).reshape(3)
    poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    vels: dict[str, np.ndarray] = {}
    for n in CAN_NAMES:
        pose = _np(base.arena_world.get_pose_w(n)[0]).reshape(7)
        if not np.isfinite(pose).all():
            _M["nan"] = True
            return
        x, y, z_q, w = pose[3:7]
        poses[n] = (pose[:3] - origin, np.array([w, x, y, z_q], dtype=np.float64))
        vels[n] = _np(base.scene.rigid_objects[n].data.root_lin_vel_w.torch[0]).reshape(3)

    _M["substeps_this_step"] = _M.get("substeps_this_step", 0) + 1
    cp, cR = _carrier_equilibrium(poses, _M["off"])
    _M["carrier_p"], _M["carrier_R"] = cp, cR

    changed = False
    for n in CAN_NAMES:
        if _M["off"].get(n):
            continue
        kp, kq = poses[n]
        d_rad, d_ax, t_deg, force, _, _ = ring_kinematics(cp, cR, kp, kq, n)
        _M["stretch"][n] = d_rad
        _M["axial"][n] = d_ax
        _M["tilt"][n] = t_deg
        _M["force"][n] = force
        _M["peak_force"] = max(_M["peak_force"], force)

        if t_deg >= PEEL_TILT_DEG and d_rad >= PEEL_STRETCH and force < TEAR_FORCE:
            _M["off"][n] = True
            _M["ever_off"].append(n)
            changed = True
            print(f"[sixpack] peeled ring off {n} at step {_M['step']} "
                  f"(tilt {t_deg:.1f} deg, stretch {d_rad * 1000:.1f} mm, load {force:.1f} N)", flush=True)
        elif force >= TEAR_FORCE:
            _M["torn"] = True
            _M["torn_ring"] = n
            _M["torn_visual_pending"] = True
            _M["off"][n] = True
            _M["ever_off"].append(n)
            changed = True
            print(f"[sixpack] TORN carrier at {n} step {_M['step']} (load {force:.1f} N >= {TEAR_FORCE:.1f} N)",
                  flush=True)

    if changed:
        cp, cR = _carrier_equilibrium(poses, _M["off"])
        _M["carrier_p"], _M["carrier_R"] = cp, cR

    engaged = [n for n in CAN_NAMES if not _M["off"].get(n)]
    if engaged:
        w_arr = np.array([_ring_k(_quat_tilt_deg(poses[n][1])) for n in engaged], dtype=np.float64)
        w_arr /= max(float(w_arr.sum()), 1e-9)
        v_mean = sum(w_arr[i] * vels[engaged[i]] for i in range(len(engaged)))
    else:
        v_mean = np.zeros(3, dtype=np.float64)

    dev = base.device
    nz_car = cR[:, 2]
    z = torch.zeros((1, 1, 3), dtype=torch.float32, device=dev)
    for n in CAN_NAMES:
        obj = base.scene.rigid_objects[n]
        if _M["off"].get(n):
            obj.permanent_wrench_composer.set_forces_and_torques(z, z, is_global=True)
            continue
        kp, kq = poses[n]
        _, d_ax, t_deg, _, vec_rad, _ = ring_kinematics(cp, cR, kp, kq, n)
        k_rad = _ring_k(t_deg)
        f_vec = -(k_rad * vec_rad + AXIAL_K * d_ax * nz_car) - REL_DAMP_C * (vels[n] - v_mean)
        f_mag = float(np.linalg.norm(f_vec))
        if f_mag > 32.0:
            f_vec = f_vec * (32.0 / f_mag)
        ft = torch.tensor(f_vec, dtype=torch.float32, device=dev).view(1, 1, 3) if f_mag > 1e-5 else z
        obj.permanent_wrench_composer.set_forces_and_torques(ft, z, is_global=True)


def _step_sim(env) -> None:
    _M["env"] = env
    _M["active_substep"] = True
    _M["step"] += 1
    if _M.get("substeps_this_step", 0) == 0:
        _substep_carrier(env.unwrapped)
    _M["substeps_this_step"] = 0

    poses = {n: _body(None, n) for n in CAN_NAMES}
    if any(not np.isfinite(p).all() for p, _ in poses.values()):
        _M["nan"] = True
        return

    if _M["start"] is None:
        _M["start"] = {n: p.copy() for n, (p, _) in poses.items()}

    cp, cR = _carrier_equilibrium(poses, _M["off"])
    _M["carrier_p"], _M["carrier_R"] = cp, cR

    if _M.get("torn_visual_pending"):
        _M["torn_visual_pending"] = False
        if _P.get("carrier_mesh") and _P.get("film_torn_lk"):
            from isaaclab.sim.utils import bind_visual_material
            from isaaclab.sim.utils.stage import get_current_stage

            bind_visual_material(_P["carrier_mesh"], _P["film_torn_lk"], stage=get_current_stage())

    # 2. Physical grasp-state update (from measured jaw aperture & pad poses)
    _update_held_state(poses)

    # 3. Continuous disqualifier tracking over the whole trial (per can while ringed, evaluated over remaining cans)
    for n, (kp, kq) in poses.items():
        t_world = _tilt_deg(kq)
        if _M["off"].get(n):
            if not held_by(n) and t_world > TILT_DROP_MAX:
                _M["dropped"] = True
            continue
        p0 = _M["start"][n]
        base_z = float(kp[2] - (BODY_H / 2) * _qrot(kq, [0.0, 0.0, 1.0])[2])
        _M["max_lift_by"][n] = max(_M["max_lift_by"].get(n, 0.0), base_z - TOP)
        _M["max_slide_by"][n] = max(_M["max_slide_by"].get(n, 0.0), math.hypot(kp[0] - p0[0], kp[1] - p0[1]))
        _M["max_tilt_by"][n] = max(_M["max_tilt_by"].get(n, 0.0), t_world)

    ringed = [m for m in CAN_NAMES if not _M["off"].get(m)]
    _M["max_lift"] = max((_M["max_lift_by"].get(m, 0.0) for m in ringed), default=0.0)
    _M["max_slide"] = max((_M["max_slide_by"].get(m, 0.0) for m in ringed), default=0.0)
    _M["max_tilt5"] = max((_M["max_tilt_by"].get(m, 0.0) for m in ringed), default=0.0)

    # 4. Gripper speed and continuous 1.0 s end-state hold counter
    gpts = gripper_points()
    if gpts is not None:
        if _M["prev_grip"] is not None:
            _M["grip_speed"] = float(np.max(np.linalg.norm(gpts - _M["prev_grip"], axis=1))) * 20.0
        else:
            _M["grip_speed"] = 0.0
        _M["prev_grip"] = gpts.copy()
    else:
        _M["grip_speed"] = 99.0

    n_free = freed_can()
    ready_1 = bool(
        not gate()
        and n_free is not None
        and cleared_pack(None, n_free)
        and pack_five_upright(None)
        and freed_on_coaster(None, n_free)
        and _M.get("held_left") is None
        and _M.get("held_right") is None
        and min_gripper_clearance(None) >= GRIPPER_CLEAR
        and _M["grip_speed"] <= GRIPPER_SPEED_MAX
    )
    _M["full_hold_steps"] = (_M["full_hold_steps"] + 1) if ready_1 else 0

    # 5. Deformable carrier mesh update
    _update_carrier_mesh(cp, cR, poses)


def gate() -> str:
    """Empty string unless a latched Score-0 condition occurred at any time in the trial."""
    if _M["nan"]:
        return "nan"
    if _M["torn"]:
        return "carrier torn"
    if len(_M["ever_off"]) > 1:
        return "more than one can unringed"
    if _M["max_lift"] > LIFT_MAX:
        return "ringed can lifted > 1.5 cm"
    if _M["max_slide"] > SLIDE_MAX:
        return "ringed can slid > 3.0 cm"
    if _M["max_tilt5"] > TILT_RINGED_MAX:
        return "ringed can tipped > 20 deg"
    if _M["dropped"]:
        return "freed can dropped or tipped > 45 deg"
    return ""


def score(objects) -> float:
    if os.environ.get("SIX_DEBUG") and _M["step"] % 15 == 0:
        print(f"[sixpack dbg] step {_M['step']} {progress(objects)}", flush=True)
    if gate():
        return 0.0
    n = freed_can()
    if n is None:
        return 0.0
    if not (cleared_pack(objects, n) and pack_five_upright(objects)):
        return 0.3
    if not (
        freed_on_coaster(objects, n)
        and _M.get("held_left") is None
        and _M.get("held_right") is None
        and min_gripper_clearance(objects) >= GRIPPER_CLEAR
        and _M["grip_speed"] <= GRIPPER_SPEED_MAX
        and _M["full_hold_steps"] >= HOLD_STEPS_REQ
    ):
        return 0.6
    return 1.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    return 1.0 - score(objects)


def progress(objects) -> dict:
    n = freed_can()
    return {
        "gate": gate() or "ok",
        "off": list(_M["off"].keys()),
        "cleared": int(bool(n and cleared_pack(objects, n))),
        "five_upright": int(bool(n and pack_five_upright(objects))),
        "on_coaster": int(bool(n and freed_on_coaster(objects, n))),
        "clear_mm": round(min_gripper_clearance(objects) * 1000.0, 1),
        "hold_steps": _M["full_hold_steps"],
        "max_lift_mm": round(_M["max_lift"] * 1000.0, 1),
        "max_slide_mm": round(_M["max_slide"] * 1000.0, 1),
        "max_tilt5_deg": round(_M["max_tilt5"], 1),
        "peak_force_N": round(_M["peak_force"], 1),
        "held": (_M.get("held_left"), _M.get("held_right")),
    }


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_sixpack_v2_hooked", False):
        orig_reset = cls.reset

        def reset(self, *a, **k):
            _reset_model()
            obs = orig_reset(self, *a, **k)
            if getattr(getattr(self, "task", None), "module", None) is None or \
                    getattr(self.task.module, "INSTRUCTION", None) != INSTRUCTION:
                return obs
            _M["env"] = self._env
            sc = self._base.scene
            if not getattr(sc.write_data_to_sim, "_sixpack_substep", False):
                orig_write = sc.write_data_to_sim

                def write_data_to_sim(_env_ref=self._env, _base_ref=self._base, _orig=orig_write):
                    if _M.get("env") is _env_ref and _M.get("active_substep"):
                        try:
                            _substep_carrier(_base_ref)
                        except Exception as exc:  # noqa: BLE001
                            if not _M.get("substep_err"):
                                _M["substep_err"] = True
                                print(f"[sixpack] substep error: {exc!r}", flush=True)
                    return _orig()

                write_data_to_sim._sixpack_substep = True
                sc.write_data_to_sim = write_data_to_sim

            poses = {n: _body(None, n) for n in CAN_NAMES}
            _M["start"] = {n: p.copy() for n, (p, _) in poses.items()}
            cp, cR = _carrier_equilibrium(poses, _M["off"])
            _M["carrier_p"], _M["carrier_R"] = cp, cR
            _update_carrier_mesh(cp, cR, poses)
            for _ in range(5):
                self._base.sim.render()
                self._env.step(self._ready)
            _M["active_substep"] = True
            return self._observation()

        reset._sixpack_v2_hooked = True
        cls.reset = reset

    if not getattr(cls._task_step, "_sixpack_v2_hooked", False):
        orig_step = cls._task_step

        def _task_step(self, *a, **k):
            if getattr(getattr(self, "task", None), "module", None) is not None and \
                    getattr(self.task.module, "INSTRUCTION", None) == INSTRUCTION:
                try:
                    _step_sim(self._env)
                except Exception as exc:  # noqa: BLE001
                    if not _M.get("hook_err"):
                        _M["hook_err"] = True
                        print(f"[sixpack] step hook error: {exc!r}", flush=True)
            return orig_step(self, *a, **k)

        _task_step._sixpack_v2_hooked = True
        cls._task_step = _task_step


_reset_model()
_install_hooks()

VARIATION = {
    "pack_xy": {"dx": 0.015, "dy": 0.015, "yaw": 0.08},
    "obj:coaster": {"dx": 0.015, "dy": 0.015},
}
