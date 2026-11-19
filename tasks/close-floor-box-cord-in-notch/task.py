"""Office floor box set flush in a 4.5 cm raised floor on the station table: a white well (12 x 14 cm inside, 4.5 cm
deep, 1 cm walls), its rim level with the floor, with a NEMA 5-15R duplex receptacle (107 mm with its strap) fixed face up
on its bottom and a grey lid hinged along the box's left edge. The lid's far edge has a cord opening (20 mm wide,
25 mm deep) 5.5 cm right of the box's centre line, over a 1 cm deep notch in the far wall's top. Each outlet has two flat-blade slots 12.7 mm apart and a ground hole.
A black cord leaves a fixed service box on the floor beyond the box on the right and runs across the floor to a
one-outlet power strip on the robot's right, where its right-angle NEMA 5-15P plug (two flat blades 6.3 x 1.5 x 15.9
mm, a 4.8 mm round ground pin 3.2 mm longer) starts fully plugged in. The robot must open the lid, plug into the floor
box so the plug body sits on the receptacle face with its prongs in their openings, lay the cord across the far wall
where the lid's opening comes down, close the lid flat and let go. A lid closed over the cord's natural path
(straight over the far wall behind the receptacle, beside the opening) rests on the cord: a pinched cord.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (FLOORBOX_DEMO=...): correct (default, 1.0) | tier04 (0.4) | tier06 (0.6) | nothing | hold |
lazy | pinch | perch | underpush (each 0).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "Plug the black cord into the floor box outlet, close the lid without pinching the cord, then let go."

WHITE = (0.93, 0.93, 0.92)
IVORY = (0.90, 0.88, 0.80)
LID_GREY = (0.62, 0.64, 0.66)
BLACK = (0.05, 0.05, 0.05)
BRASS = (0.78, 0.66, 0.35)

# -- floor-box well (fixed) ----------------------------------------------------------------------------------------
X_IN0, X_IN1 = 0.145, 0.265  # inside, along x (120 mm: a duplex receptacle with its 107 mm strap fits)
Y_IN0, Y_IN1 = -0.07, 0.07  # inside, along y
WC = np.array([(X_IN0 + X_IN1) / 2, (Y_IN0 + Y_IN1) / 2])
WT, WH = 0.010, 0.045  # wall thickness, wall height (well depth)
RIM = TOP + WH
NOTCH_Y, NOTCH_W, NOTCH_D = -0.055, 0.020, 0.025  # cord opening in the lid's far edge (centre y, width, depth along x)
NOTCH_H = 0.010  # under the opening, a cord notch 10 mm deep in the far wall's top (same width)
FLOOR = RIM  # the floor around the box: the box is set flush, its rim level with the floor
X_OUT0, X_OUT1 = X_IN0 - WT, X_IN1 + WT
FLOOR_C = (0.78, 0.76, 0.72)


def _well() -> list[Box]:
    zc = TOP + WH / 2
    oy = (Y_IN1 - Y_IN0) + 2 * WT
    ix = X_IN1 - X_IN0
    out = [
        Box("well_wall_near", pos=(X_IN0 - WT / 2, WC[1], zc), size=(WT, oy, WH), color=WHITE),
        Box("well_wall_left", pos=(WC[0], Y_IN1 + WT / 2, zc), size=(ix, WT, WH), color=WHITE),
        Box("well_wall_right", pos=(WC[0], Y_IN0 - WT / 2, zc), size=(ix, WT, WH), color=WHITE),
    ]
    y_lo, y_hi = WC[1] - oy / 2, WC[1] + oy / 2
    n0, n1 = NOTCH_Y - NOTCH_W / 2, NOTCH_Y + NOTCH_W / 2
    xf = X_IN1 + WT / 2
    out += [
        Box("well_wall_far_r", pos=(xf, (y_lo + n0) / 2, zc), size=(WT, n0 - y_lo, WH), color=WHITE),
        Box("well_wall_far_l", pos=(xf, (n1 + y_hi) / 2, zc), size=(WT, y_hi - n1, WH), color=WHITE),
        Box("well_notch_sill", pos=(xf, NOTCH_Y, TOP + (WH - NOTCH_H) / 2), size=(WT, NOTCH_W, WH - NOTCH_H), color=WHITE),
    ]
    # the floor: four slabs at rim height round the box (x 0.07-0.85, y -0.55..0.55), so the box is set flush in it
    fx0, fx1, fy = 0.07, 0.85, 0.55
    yo0, yo1 = WC[1] - oy / 2, WC[1] + oy / 2
    for name, x0, x1, y0, y1 in (("near", fx0, X_OUT0, -fy, fy), ("far", X_OUT1, fx1, -fy, fy),
                                 ("left", X_OUT0, X_OUT1, yo1, fy), ("right", X_OUT0, X_OUT1, -fy, yo0)):
        out.append(Box(f"floor_{name}", pos=((x0 + x1) / 2, (y0 + y1) / 2, zc), size=(x1 - x0, y1 - y0, WH), color=FLOOR_C))
    return out


# -- NEMA 5-15 geometry (plug and receptacle) ------------------------------------------------------------------------
BLADE_W, BLADE_T, BLADE_L = 0.0063, 0.0015, 0.0159  # flat blade: width, thickness, length
BLADE_GAP = 0.0127  # blade spacing, centre to centre
PIN_D = 0.0048  # round ground pin diameter
PIN_L = BLADE_L + 0.0032  # 3.2 mm longer than the blades
PIN_OFF = 0.0119  # ground pin centre to the middle of the line joining the blades
SLOT_L, SLOT_W = 0.0070, 0.0023  # receptacle blade slot: 7.0 x 2.3 mm for a 6.3 x 1.5 mm blade (0.35-0.4 mm a side)
HOLE = 0.0054  # receptacle ground hole, 5.4 mm for the 4.8 mm pin (square opening: the kit has no round holes)

# Plug body: a right-angle moulded plug, cord leaving one end at mid-height. Plug frame = cable frame: +x points away
# from the cord, so the ground pin sits on the cord side of the blades.
PL, PW, PH = 0.036, 0.030, 0.020  # body length (along the cord), width, height
BLADE_LX = 0.006  # blade line, plug-local x
PIN_LX = BLADE_LX - PIN_OFF  # ground pin, plug-local x (-0.0059)

# Duplex receptacle, face up, long axis along x, two outlets. The plug goes in with the cord leaving towards +x
# (towards the far wall and the lid's cord opening), so in world terms: blades at x = outlet - 0.006, pin at x = outlet + 0.0059.
XR, YR = 0.205, 0.0  # receptacle face centre
FACE_X, FACE_Y = 0.070, 0.034  # moulded face (34 mm wide, as a standard duplex body)
FACE_T = 0.003
Z_FACE = TOP + 0.022  # receptacle face height (blades and pin need room below it)
OUTLETS = (XR - 0.019, XR + 0.019)  # x of the plug body centre when seated in each outlet (near, far)


def _slots(ox: float) -> list[tuple[float, float, float, float]]:
    """Holes in the receptacle face for the outlet whose seated plug centre is at x = ox: (x0, x1, y0, y1)."""
    bx = ox - BLADE_LX
    px = ox - PIN_LX
    out = [(bx - SLOT_L / 2, bx + SLOT_L / 2, YR + s * BLADE_GAP / 2 - SLOT_W / 2, YR + s * BLADE_GAP / 2 + SLOT_W / 2)
           for s in (-1, 1)]
    out.append((px - HOLE / 2, px + HOLE / 2, YR - HOLE / 2, YR + HOLE / 2))
    return out


HOLES = [h for ox in OUTLETS for h in _slots(ox)]


def _socket_boxes(prefix: str, centre, yaw: float, outlets_local, face_x: float, face_y: float, color,
                  slot=(SLOT_L, SLOT_W), hole=HOLE, base=TOP) -> list[Box]:
    """A face-up receptacle: a face plate with the slot and ground-hole openings cut out (as strips) on a hollow
    body. Built in the frame of a seated plug (x along the plug, away from its cord), turned by ``yaw`` about
    ``centre``; ``outlets_local`` are the plug-centre positions along that x."""
    holes = []
    for u in outlets_local:
        bx, px = u + BLADE_LX, u + PIN_LX
        holes += [(bx - slot[0] / 2, bx + slot[0] / 2, s * BLADE_GAP / 2 - slot[1] / 2, s * BLADE_GAP / 2 + slot[1] / 2)
                  for s in (-1, 1)]
        holes.append((px - hole / 2, px + hole / 2, -hole / 2, hole / 2))
    c, s_ = math.cos(yaw), math.sin(yaw)

    def box(name, lx, ly, z, size):
        return Box(name, pos=(float(centre[0] + c * lx - s_ * ly), float(centre[1] + s_ * lx + c * ly), float(z)),
                   size=size, color=color, yaw=yaw)

    x0, x1, y0, y1 = -face_x / 2, face_x / 2, -face_y / 2, face_y / 2
    xs = sorted({x0, x1, *[v for h in holes for v in h[:2]]})
    z_face = base + (Z_FACE - TOP)
    zc = z_face - FACE_T / 2
    out, k = [], 0
    for a, b in zip(xs[:-1], xs[1:]):
        if b - a < 1e-6:
            continue
        cuts = sorted((h[2], h[3]) for h in holes if h[0] < (a + b) / 2 < h[1])
        ys, y = [], y0
        for c0, c1 in cuts:
            ys.append((y, c0))
            y = c1
        ys.append((y, y1))
        for ya, yb in ys:
            if yb - ya > 1e-6:
                out.append(box(f"{prefix}_face_{k}", (a + b) / 2, (ya + yb) / 2, zc, (b - a, yb - ya, FACE_T)))
                k += 1
    bh = z_face - FACE_T - base
    bz = base + bh / 2
    t = 0.003
    out += [
        box(f"{prefix}_body_a", x0 + t / 2, 0.0, bz, (t, face_y, bh)),
        box(f"{prefix}_body_b", x1 - t / 2, 0.0, bz, (t, face_y, bh)),
        box(f"{prefix}_body_c", 0.0, y1 - t / 2, bz, (face_x - 2 * t, t, bh)),
        box(f"{prefix}_body_d", 0.0, y0 + t / 2, bz, (face_x - 2 * t, t, bh)),
    ]
    return out


STRAP_L, STRAP_W, STRAP_T = 0.107, 0.022, 0.0012  # the receptacle's metal mounting strap, ears at both ends


def _receptacle() -> list[Box]:
    """The floor box's duplex receptacle (plug frame yaw pi: a seated plug's cord leaves towards the far wall), with
    the two ears of its 107 mm mounting strap sticking out beyond the moulded face."""
    out = _socket_boxes("recept", (XR, YR), math.pi, [XR - ox for ox in OUTLETS], FACE_X, FACE_Y, IVORY)
    ear = (STRAP_L - FACE_X) / 2
    for i, sx in enumerate((-1, 1)):
        out.append(Box(f"recept_strap_ear_{i}", pos=(XR + sx * (FACE_X + ear) / 2, YR, Z_FACE - FACE_T + STRAP_T / 2),
                       size=(ear, STRAP_W, STRAP_T), color=(0.62, 0.62, 0.60)))
    return out


# -- lid hinged along the left edge, with a cord opening and a lift handle ------------------------------------------
LID_X0, LID_X1 = X_OUT0, X_OUT1 - 0.004  # lid spans x 0.135 .. 0.271: it closes inside the box frame's outer edge
LID_X = LID_X1 - LID_X0
LID_Y0, LID_Y1 = Y_IN0 - WT, Y_IN1 + WT + 0.005  # y -0.08 .. 0.085, its hinge edge resting 5 mm onto the floor
LID_Y = LID_Y1 - LID_Y0
LID_T = 0.006
LID_Z0 = RIM + LID_T / 2
# Hinge along the lid's left edge (axis along x), so the opened lid stands beside the box and leaves the far wall clear.
HINGE_Y = LID_Y1
HINGE_MAX = 100.0  # deg: the hinge stop, just past upright, so an opened lid stays open
OPEN_TO = 106.0  # deg: the demo swings the handle a little past the stop
# The lid is a plate with a cord opening cut into its far edge (NOTCH_W wide, NOTCH_D deep, centred y = NOTCH_Y): the
# main plate (hinged) plus two edge strips welded to it on either side of the opening.
LID_XM = LID_X1 - NOTCH_D  # where the opening starts
LID = Box("floor_box_lid", pos=((LID_X0 + LID_XM) / 2, (LID_Y0 + LID_Y1) / 2, LID_Z0), size=(LID_XM - LID_X0, LID_Y, LID_T),
          mass=0.042, color=LID_GREY)
LID_EDGES = [Box(f"floor_box_lid_edge_{i}", pos=((LID_XM + LID_X1) / 2, (y0 + y1) / 2, LID_Z0),
                 size=(NOTCH_D, y1 - y0, LID_T), mass=0.004, color=LID_GREY)
             for i, (y0, y1) in enumerate(((LID_Y0, NOTCH_Y - NOTCH_W / 2), (NOTCH_Y + NOTCH_W / 2, LID_Y1)))]
# Lift handle: a 46 mm bail bar on its own pin (parallel to the hinge) between two low posts, 7 cm from the hinge, its
# top 18 mm above the lid. The fingers pinch the bar across y; the bar turns on its pin, so the pinch doesn't lock the lid.
BAR_S = (0.046, 0.010, 0.010)
POST_H = 0.008
KNOB_P0 = np.array([0.185, HINGE_Y - 0.070, RIM + LID_T + POST_H + BAR_S[2] / 2])
KNOB = Box("lid_handle_bar", pos=tuple(KNOB_P0.tolist()), size=BAR_S, mass=0.004, color=(0.40, 0.42, 0.45))
POSTS = [Box(f"lid_handle_post_{i}", pos=(float(KNOB_P0[0] + s * 0.0265), float(KNOB_P0[1]), RIM + LID_T + POST_H / 2 + 0.0025),
             size=(0.005, 0.006, POST_H + 0.005), mass=0.002, color=(0.40, 0.42, 0.45)) for i, s in enumerate((-1, 1))]

# -- service box, power strip and power cord -----------------------------------------------------------------------
# At the start the plug sits in a one-outlet power strip on the table to the robot's right (the cord's temporary
# supply), level and fully seated, so it is picked up the way it would be unplugged.
R = 0.003  # 6 mm cord
ANCHOR = (0.70, -0.10)  # where the cord leaves the service box
STRIP_C = (0.125, -0.255)  # power-strip outlet: the seated plug's centre
CORD_YAW = math.atan2(STRIP_C[1] - ANCHOR[1], STRIP_C[0] - ANCHOR[0])
_U = np.array([math.cos(CORD_YAW), math.sin(CORD_YAW), 0.0])  # along the cord, from the service box to the plug
# The kit's cord settles its plug 6.5 mm back towards the anchor from its nominal spawn point (measured), so the plug
# (with its blades and pin) spawns 6.5 mm further out and 1.5 mm above the strip; it settles and drops into its outlet.
SPAWN_SHIFT = 0.0065
CORD_LEN = float(np.hypot(STRIP_C[0] - ANCHOR[0], STRIP_C[1] - ANCHOR[1]) - PL / 2 + SPAWN_SHIFT)
Z_SFACE = FLOOR + (Z_FACE - TOP)  # power-strip face height (the strip stands on the floor)
PLUG_Z = Z_SFACE + PH / 2 + (PIN_L - BLADE_L) + 0.0015  # plug centre at spawn: pin tip 1.5 mm above the strip face
SERVICE = Box("service_box", pos=(float(ANCHOR[0] - _U[0] * 0.036), float(ANCHOR[1] - _U[1] * 0.036), FLOOR + 0.025),
              size=(0.06, 0.07, 0.05), color=(0.45, 0.46, 0.48), yaw=CORD_YAW)
CORD = cable("cord", start=(ANCHOR[0], ANCHOR[1], PLUG_Z), yaw=CORD_YAW, length=CORD_LEN, radius=R,
             anchor_start=True, end_plug=Plug(size=(PL, PW, PH), mass=0.04, color=(0.08, 0.08, 0.09)), color=BLACK,
             bend_stiffness=0.01, bend_damping=0.001)
P0 = np.array([STRIP_C[0] + _U[0] * SPAWN_SHIFT, STRIP_C[1] + _U[1] * SPAWN_SHIFT, PLUG_Z])  # plug centre at spawn
PLUG_REST = np.array([STRIP_C[0], STRIP_C[1], Z_SFACE + PH / 2])  # seated in the strip
STRIP = _socket_boxes("strip", STRIP_C, CORD_YAW, [0.0], 0.044, 0.040, (0.22, 0.22, 0.24), slot=(0.010, 0.004), hole=0.008,
                      base=FLOOR)


def _plug_part(local) -> tuple[float, float, float]:
    """World position at spawn of a point given in the plug frame (the cable frame: yaw CORD_YAW)."""
    c, s_ = math.cos(CORD_YAW), math.sin(CORD_YAW)
    return (float(P0[0] + c * local[0] - s_ * local[1]), float(P0[1] + s_ * local[0] + c * local[1]),
            float(P0[2] + local[2]))


BLADE_LOCAL = [np.array([BLADE_LX, s * BLADE_GAP / 2, -PH / 2 - BLADE_L / 2]) for s in (1, -1)]
PIN_LOCAL = np.array([PIN_LX, 0.0, -PH / 2 - PIN_L / 2])
BLADES = [Box(f"plug_blade_{i}", pos=_plug_part(b), size=(BLADE_W, BLADE_T, BLADE_L), mass=0.005, color=BRASS,
              yaw=CORD_YAW) for i, b in enumerate(BLADE_LOCAL)]
PIN = Cylinder("plug_ground_pin", pos=_plug_part(PIN_LOCAL), radius=PIN_D / 2, height=PIN_L, mass=0.005, color=BRASS,
               yaw=CORD_YAW)

# -- blade contacts: what grips the blades in a real receptacle -----------------------------------------------------
# Each blade slot of the floor box receptacle and of the power strip has a spring contact on each side of the blade,
# 7 mm below the face: a rounded brass contact on a short slide, preloaded against a stop so it stands 0.15 mm into
# the blade's path. A blade going in pushes both contacts back and slides between them; their friction holds the plug
# in and resists pushing it home. Four contacts per outlet: total sliding resistance 4 * MU_C * CONTACT_N = 13.3 N,
# the 3 lbf (13 N) a UL 498 receptacle must at least hold a two-blade gauge against (it must also release a gauge
# at 15 lbf, 67 N, at most). The rounded contact face is the lead-in, so going in the blade tips first meet a short
# peak of a few times that.
CONTACT_R = 0.0025  # contact face radius (its curve is the blade's lead-in)
CONTACT_P = 0.00015  # how far a contact at rest stands into the blade's path
CONTACT_DZ = 0.007  # contact centre below the face
CONTACT_TRAVEL = 0.0015  # how far a contact can be pushed back
MU_C = 0.5  # contact-blade friction
CONTACT_N = 6.67  # N: spring force on the blade per contact (near constant: long preload)
CONTACT_K = 2000.0  # N/m
CONTACT_PRE = CONTACT_N / CONTACT_K  # 3.3 mm of preload past the rest stop
RETAIN_N = 4 * MU_C * CONTACT_N  # 13.3 N (3 lbf) per outlet


def _contacts(prefix: str, centre, yaw: float, outlets_local, base: float):
    """(Sphere, slide-axis sign, yaw) for each blade contact of a socket built by _socket_boxes with the same frame."""
    c, s_ = math.cos(yaw), math.sin(yaw)
    z = base + (Z_FACE - TOP) - CONTACT_DZ
    out = []
    for i, u in enumerate(outlets_local):
        lx = u + BLADE_LX
        for j, sb in enumerate((-1, 1)):
            for sg in (-1, 1):  # the contact's side of the blade (plug-frame y)
                ly = sb * BLADE_GAP / 2 + sg * (BLADE_T / 2 + CONTACT_R - CONTACT_P)
                pos = (float(centre[0] + c * lx - s_ * ly), float(centre[1] + s_ * lx + c * ly), float(z))
                name = f"{prefix}_contact_{i}{j}{'ab'[(sg + 1) // 2]}"
                out.append((Sphere(name, pos=pos, radius=CONTACT_R, mass=0.005, color=BRASS), sg, yaw))
    return out


RECEPT_CONTACTS = _contacts("recept", (XR, YR), math.pi, [XR - ox for ox in OUTLETS], TOP)
STRIP_CONTACTS = _contacts("strip", STRIP_C, CORD_YAW, [0.0], FLOOR)
CONTACTS = RECEPT_CONTACTS + STRIP_CONTACTS
STRIP_GRIP_STEP = 20  # the strip's contacts close on the blades once the plug has settled into it (1 s)

OBJECTS = [*_well(), *_receptacle(), *STRIP, LID, *LID_EDGES, *POSTS, KNOB, SERVICE, *BLADES, PIN,
           *(c[0] for c in CONTACTS), CORD]


# -- joints authored right after the cable (the kit's only spawn hook) ---------------------------------------------
# Lid: revolute hinge to the world along the far edge (0 = shut, stop at 100 deg). Knob: its own pin parallel to the
# hinge, so a firm pinch does not lock the lid. Blades and ground pin: fixed joints to the plug body.


_CONTACT_DRIVES: list = []  # (joint path, side sign, held open at spawn)
_STEP = {"n": 0}


def _install_spawn_hook() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_floorbox_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORD.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        h = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/lid_hinge")
        h.CreateBody1Rel().SetTargets([Sdf.Path(f"{parent}/{LID.name}")])
        h.CreateAxisAttr("X")  # opening = negative rotation about +x (the free edge at -y rises)
        h.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + LID.pos[0]), float(w[1] + HINGE_Y), float(w[2] + LID_Z0)))
        h.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        h.CreateLocalPos1Attr(Gf.Vec3f(0.0, float(LID_Y / 2), 0.0))
        h.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        h.CreateLowerLimitAttr(-HINGE_MAX)
        h.CreateUpperLimitAttr(2.0)
        h.CreateExcludeFromArticulationAttr(True)

        def joint(name, b0, b1, p0, revolute=False):
            j = (UsdPhysics.RevoluteJoint if revolute else UsdPhysics.FixedJoint).Define(stage, f"{prim_path}/{name}")
            if revolute:
                j.CreateAxisAttr("X")
            j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
            j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
            j.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in p0)))
            j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            j.CreateExcludeFromArticulationAttr(True)

        for p in (*POSTS, *LID_EDGES):
            joint(f"{p.name}_weld", f"{parent}/{LID.name}", f"{parent}/{p.name}", np.array(p.pos) - np.array(LID.pos))
        joint("handle_pin", f"{parent}/{LID.name}", f"{parent}/{KNOB.name}", KNOB_P0 - np.array(LID.pos), revolute=True)
        plug = f"{prim_path}/end_plug"
        for b, loc in zip(BLADES, BLADE_LOCAL):
            joint(f"{b.name}_weld", plug, f"{parent}/{b.name}", loc)
        joint("pin_weld", plug, f"{parent}/{PIN.name}", PIN_LOCAL)

        # blade contacts: a slide (world -> contact) along the plug-frame y, a preloaded spring drive pushing the
        # contact towards the blade against its rest stop
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material

        mat = f"{prim_path}/Looks/contact_brass"
        mc = sim_utils.RigidBodyMaterialCfg(static_friction=MU_C, dynamic_friction=MU_C, friction_combine_mode="min")
        mc.func(mat, mc)
        for sph, sg, yaw in CONTACTS:
            cp = f"{parent}/{sph.name}"
            q = Gf.Quatf(float(math.cos(yaw / 2)), 0.0, 0.0, float(math.sin(yaw / 2)))
            j = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/{sph.name}_slide")
            j.CreateBody1Rel().SetTargets([Sdf.Path(cp)])
            j.CreateAxisAttr("Y")
            j.CreateLocalPos0Attr(Gf.Vec3f(*(float(w[k] + sph.pos[k]) for k in range(3))))
            j.CreateLocalRot0Attr(q)
            j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            j.CreateLocalRot1Attr(q)
            j.CreateLowerLimitAttr(0.0 if sg > 0 else -CONTACT_TRAVEL)
            j.CreateUpperLimitAttr(CONTACT_TRAVEL if sg > 0 else 0.0)
            j.CreateExcludeFromArticulationAttr(True)
            d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
            d.CreateTypeAttr("force")
            d.CreateStiffnessAttr(CONTACT_K)
            d.CreateDampingAttr(5.0)
            open_ = sph.name.startswith("strip")  # held back until the plug has settled into the strip
            d.CreateTargetPositionAttr(sg * (CONTACT_TRAVEL + 0.002) if open_ else -sg * CONTACT_PRE)
            _CONTACT_DRIVES.append((str(j.GetPath()), sg, open_))
            for p in stage.Traverse():
                ps = p.GetPath().pathString
                if (ps == cp or ps.startswith(cp + "/")) and p.HasAPI(UsdPhysics.CollisionAPI):
                    bind_physics_material(ps, mat, stage=stage)
        return prim

    spawn._floorbox_hook = True
    cablekit._spawn_cable = spawn


_install_spawn_hook()

# -- scoring -------------------------------------------------------------------------------------------------------
SEAT_GAP = 0.0015  # fully seated: plug body's face at most 1.5 mm above the receptacle face (>= 14.4 mm of blade in)
TILT_MAX = 8.0  # deg: blades within 8 deg of vertical
CLOSED_DEG = 1.3  # lid shut: within 1.3 deg of flat (its free edge within 3.7 mm of the rim)
OPEN_DEG = 60.0  # lid left open: at least 60 deg up
RELEASE_DIST = 0.035  # both TCPs at least 3.5 cm from the cord, plug, blades, pin, lid and knob

_ENV: dict = {"env": None}


def _install_hooks() -> None:
    """Keep a handle on the env so score() can see the grippers."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (no embodiment: offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_floorbox_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            _STEP["n"] += 1
            if _STEP["n"] == STRIP_GRIP_STEP:
                _close_strip_contacts()
            return base(self, *a, **k)

        _task_step._floorbox_hooked = True
        cls._task_step = _task_step


_install_hooks()


def _close_strip_contacts() -> None:
    """Let the strip's contacts spring onto the blades of the plug that has settled into the strip."""
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        stage = get_current_stage()
        for path, sg, open_ in _CONTACT_DRIVES:
            if open_:
                stage.GetPrimAtPath(path).GetAttribute("drive:linear:physics:targetPosition").Set(-sg * CONTACT_PRE)
    except Exception as e:  # noqa: BLE001
        print(f"[floorbox] closing the strip contacts failed: {e!r}", flush=True)


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def lid_deg(objects) -> float:
    """Lid opening angle about its hinge (deg, 0 = shut); 90 if unreadable."""
    st = objects[LID.name]
    if not np.isfinite(st.quat_xyzw).all():
        return 90.0
    free = _rot(st.quat_xyzw) @ np.array([0.0, -1.0, 0.0])  # hinge -> free edge
    return float(math.degrees(math.atan2(free[2], -free[1])))


def seated_outlet(objects) -> int | None:
    """Index of the outlet the plug is fully seated in, else None. Seated: each blade's centre inside its own slot
    and the pin's centre inside the ground hole (horizontally), blades within 8 deg of vertical, and the plug body's
    face (the blades' root) at most 1.5 mm above the receptacle face."""
    parts = [objects[b.name] for b in BLADES] + [objects[PIN.name]]
    if not all(np.isfinite(p.pos).all() and np.isfinite(p.quat_xyzw).all() for p in parts):
        return None
    for p in parts:
        ax = _rot(p.quat_xyzw)[:, 2]
        if math.degrees(math.acos(min(1.0, abs(float(ax[2]))))) > TILT_MAX:
            return None
    roots = [float(p.pos[2] + _rot(p.quat_xyzw)[2, 2] * L / 2) for p, L in zip(parts, (BLADE_L, BLADE_L, PIN_L))]
    face = min(roots[:2])
    if not (Z_FACE - 0.002 <= face <= Z_FACE + SEAT_GAP):
        return None
    pts = [p.pos[:2] for p in parts]
    for i, ox in enumerate(OUTLETS):
        holes = _slots(ox)
        ok = True
        # blades may sit in either slot of the pair (the plug's two blades are the same size)
        bl = sorted(pts[:2], key=lambda v: v[1])
        for xy, h, (hx, hy) in ((bl[0], holes[0], (BLADE_W, BLADE_T)), (bl[1], holes[1], (BLADE_W, BLADE_T)),
                                (pts[2], holes[2], (PIN_D, PIN_D))):
            cx, cy = (h[0] + h[1]) / 2, (h[2] + h[3]) / 2
            if abs(xy[0] - cx) > (h[1] - h[0] - hx) / 2 + 0.0005 or abs(xy[1] - cy) > (h[3] - h[2] - hy) / 2 + 0.0005:
                ok = False
        if ok:
            return i
    return None


def face_gap(objects) -> float:
    """Height of the plug body's face above the receptacle face (m), for logs."""
    b = objects[BLADES[0].name]
    return float(b.pos[2] + _rot(b.quat_xyzw)[2, 2] * BLADE_L / 2 - Z_FACE)


def cord_in_notch(objects) -> bool:
    """The cord crosses the far wall inside the lid's cord opening (and the rim notch under it): some point of its
    centre line lies over the far wall (3 mm margin each side), between the opening's sides (1 mm margin), and no
    higher than the shut lid's top."""
    pts = _dense(objects[CORD.name].polyline())
    x0, x1 = X_IN1 - 0.003, X_OUT1 + 0.003
    ok = ((pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (np.abs(pts[:, 1] - NOTCH_Y) <= NOTCH_W / 2 + 0.001)
          & (pts[:, 2] <= RIM + LID_T))
    return bool(ok.any())


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    if not np.isfinite(poly).all():
        return np.asarray(poly)
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * (k / n) for k in range(1, n + 1)]
    return np.asarray(out)


def grippers() -> list[np.ndarray] | None:
    """World positions of both grippers' TCP (13 mm above the finger pad tips); None if unknown."""
    env = _ENV["env"]
    if env is None:
        return None
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


def _box_dist(p, st) -> float:
    d = np.maximum(np.maximum(st.low - p, p - st.high), 0.0)
    return float(np.linalg.norm(d))


def released(objects) -> bool:
    """Both grippers at least 3.5 cm from every part of the cord, plug, blades, pin, lid and knob. An unreadable
    gripper state counts as not released."""
    gs = grippers()
    if not gs or not all(np.isfinite(g).all() for g in gs):
        return False
    cord = objects[CORD.name]
    pts = _dense(cord.polyline())
    plug = cord.end_plug
    for g in gs:
        if float(np.min(np.linalg.norm(pts - g, axis=1))) < RELEASE_DIST:
            return False
        if plug is not None and float(np.linalg.norm(np.asarray(plug) - g)) < RELEASE_DIST + PL / 2:
            return False
        for o in (LID, *LID_EDGES, *POSTS, KNOB, *BLADES, PIN):
            if _box_dist(g, objects[o.name]) < RELEASE_DIST:
                return False
    return True


def score(objects) -> float:
    """Gates (score 0): the sim state is unreadable; a gripper within 3.5 cm of the cord, plug, blades, pin, lid or
    knob (or its state unreadable); the plug not fully seated in an outlet (blades and pin in their own slot and
    hole, within 8 deg of vertical, plug face at most 1.5 mm above the receptacle face); the lid neither shut flat
    (within 1.3 deg) with the cord in its opening nor left open (60 deg or more), i.e. resting on the cord.
    Ladder: 0.4 plug seated, lid left open, cord not in the opening; 0.6 plug seated, lid left open, cord in the
    opening; 1.0 plug seated, cord in the opening, lid shut flat."""
    cord = objects[CORD.name]
    if not (np.isfinite(cord.polyline()).all() and np.isfinite(objects[LID.name].pos).all()):
        return 0.0
    if not released(objects) or seated_outlet(objects) is None:
        return 0.0
    notch = cord_in_notch(objects)
    deg = lid_deg(objects)
    if deg >= OPEN_DEG:
        return 0.6 if notch else 0.4  # box left open: unfinished, but nothing pinched
    if deg <= CLOSED_DEG and notch:
        return 1.0
    return 0.0  # lid resting on the cord (pinched) or ajar


def success(objects) -> bool:
    return bool(score(objects) == 1.0 and objects[LID.name].resting() and
                np.linalg.norm(objects[CORD.name].lin_vel) < 0.02)


def distance(objects) -> float:
    """How far (m) the plug still is from seated in the near outlet."""
    p = objects[CORD.name].end_plug
    if p is None or not np.isfinite(p).all():
        return 1.0
    return float(np.linalg.norm(np.asarray(p) - np.array([OUTLETS[0], YR, Z_FACE + PH / 2])))


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    return {
        "lid_opened": float(lid_deg(objects) >= OPEN_DEG),
        "plug_seated": float(seated_outlet(objects) is not None),
        "cord_in_notch": float(cord_in_notch(objects)),
        "lid_shut": float(lid_deg(objects) <= CLOSED_DEG),
        "released": float(released(objects)),
    }


VARIATION = {}  # fixed layout: nothing is jittered (the plug parts are welded to the cord)


# -- scripted demo (cabledemo.py) ----------------------------------------------------------------------------------
# Motions use only the fixed layout (box, receptacle, hinge and power strip never move), the arms' measured joint
# positions and one fixed grip offset measured offline (GRIP_DX/GRIP_DY). ctx.states() appears only in logging and the
# offline calibration print, never in a motion. See the README's "Scripted demo" section for the steps.
#
# Left arm: pinch the lid's handle bar, swing the lid open onto its stop, let go. Right arm: lift the plug out of the
# power strip, hold it over the near outlet, lower until the ground pin presses on the face, search over the face until
# the measured height drops (pin in its hole), push home, press its top. Then it pushes the cord sideways along the far
# wall to where the lid's cord opening comes down. The left arm swings the lid back down and lets it fall shut.

FINGER_PINCH = -0.020  # pads ~44 mm apart
HI = FLOOR + 0.12
PARK_L = np.array([0.14, 0.26, TOP + 0.14])
PARK_R = np.array([0.14, -0.26, TOP + 0.14])


def _log(ctx, tag=""):
    """Debug print of the scored state (logging only: nothing here steers the arms)."""
    if not os.environ.get("FLOORBOX_LOG", "1") == "1":
        return
    st = ctx.states()
    print(f"[t={ctx.t:5.2f}] {tag} seated={seated_outlet(st)} gap={face_gap(st) * 1000:.1f}mm "
          f"plug={np.round(st[CORD.name].end_plug, 4)} notch={cord_in_notch(st)} lid={lid_deg(st):.1f}deg "
          f"rel={released(st)} score={score(st)}", flush=True)
    try:
        g = np.asarray(ctx.obs.state[__import__("inspect_robots_yam.packing", fromlist=["STATE_KEY"]).STATE_KEY])
        print(f"    knob={np.round(st[KNOB.name].pos, 4)} tcpL={np.round(_meas_tcp(ctx, 'left'), 4)} "
              f"tcpR={np.round(_meas_tcp(ctx, 'right'), 4)} grip L/R={g[6]:.2f}/{g[13]:.2f} "
              f"pin={np.round(st[PIN.name].pos, 4)}", flush=True)
        pts = _dense(st[CORD.name].polyline())
        w = pts[(pts[:, 0] >= X_IN1 - 0.005) & (pts[:, 0] <= X_IN1 + WT + 0.005)]
        if len(w):
            print(f"    cord at wall: y={w[:, 1].min():.4f}..{w[:, 1].max():.4f} z-rim={(w[:, 2].min() - RIM) * 1000:.1f}mm",
                  flush=True)
        if os.environ.get("FLOORBOX_CALIB") == "1":
            dsp = {sph.name: float(np.hypot(*(np.asarray(st[sph.name].pos[:2]) - np.asarray(sph.pos[:2])))) * 1000
                   for sph, _, _ in CONTACTS}
            print("    contacts pushed back (mm): " + " ".join(f"{k.split('_', 1)[0][0]}{k[-3:]}={v:.2f}"
                                                         for k, v in dsp.items()), flush=True)
            near = pts[(pts[:, 0] >= X_IN1 - 0.02) & (pts[:, 0] <= X_OUT1 + 0.03)][::3]
            print("    cord near wall (x-X_OUT1 mm, y mm, z-rim mm): " + " ".join(
                f"({(q[0] - X_OUT1) * 1000:.0f},{q[1] * 1000:.0f},{(q[2] - RIM) * 1000:.1f})" for q in near), flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"    (debug failed: {e})", flush=True)


def _meas_tcp(ctx, side: str) -> np.ndarray:
    """TCP from the measured joint positions in the observation (robot state, not object state)."""
    from inspect_robots_yam.packing import STATE_KEY
    from yam_arena import ik
    from yam_arena.station import BASES

    q = np.asarray(ctx.obs.state[STATE_KEY], dtype=float)
    q = q[0:6] if side == "left" else q[7:13]
    return np.asarray(ik.tcp_pose(q, np.asarray(BASES[side], dtype=float))[0], dtype=float)


# lid geometry known from the layout: hinge line and the knob's radius about it
_KNOB_V = KNOB_P0 - np.array([0.0, HINGE_Y, LID_Z0])  # handle bar centre relative to the hinge line when shut


def _knob_at(deg: float) -> np.ndarray:
    t = math.radians(deg)
    y, z = _KNOB_V[1] * math.cos(t) + _KNOB_V[2] * math.sin(t), -_KNOB_V[1] * math.sin(t) + _KNOB_V[2] * math.cos(t)
    return np.array([KNOB_P0[0], HINGE_Y + y, LID_Z0 + z])


KNOB_GRIP_DZ = 0.008  # TCP above the bar's centre: fingertips 5 mm below it (the bar's underside)
LYAW = 0.0  # left grip on the handle: fingers close along y (across the bar), wrist camera along the hinge axis


_DBG: dict = {}


def _swing(arm, a0: float, a1: float, step=6.0, dur=0.5):
    n = max(1, int(abs(a1 - a0) / step))
    for k in range(1, n + 1):
        yield from arm.move(_knob_at(a0 + (a1 - a0) * k / n) + np.array([0, 0, KNOB_GRIP_DZ]), yaw=LYAW, duration=dur)
        if _DBG.get("ctx") is not None and os.environ.get("FLOORBOX_SWINGDBG"):
            c = _DBG["ctx"]
            st = c.states()
            g = np.asarray(c.obs.state["joint_pos"])
            print(f"[swing] cmd={a0 + (a1 - a0) * k / n:5.1f} lid={lid_deg(st):5.1f} gripL={g[6]:.2f} "
                  f"tcpL={np.round(_meas_tcp(c, 'left'), 4)} knob={np.round(st[KNOB.name].pos, 4)}", flush=True)


def _open_lid(ctx, flags):
    from yam_arena.constants import FINGER_CLOSED

    L = ctx.left
    k = _knob_at(0.0)
    yield from L.move(k + np.array([0, 0, 0.05]), yaw=LYAW, finger=FINGER_PINCH, duration=1.5)
    yield from L.move(k + np.array([0, 0, KNOB_GRIP_DZ]), yaw=LYAW, duration=1.0)
    yield from L.grip(FINGER_CLOSED, 0.6)
    _log(ctx, "knob pinched")
    yield from _swing(L, 0.0, 45.0)
    _log(ctx, "swing 45")
    yield from _swing(L, 45.0, OPEN_TO)
    _log(ctx, "swung")
    yield from L.wait(0.3)
    yield from L.grip(FINGER_PINCH, 0.5)
    p = L.tcp()
    # the fingers are on the outer (+y) side of the upright lid, the same side as the park pose
    yield from L.move((p[0], p[1] + 0.022, p[2] + 0.01), yaw=LYAW, duration=0.6)
    yield from L.move((p[0], p[1] + 0.06, TOP + 0.17), yaw=LYAW, duration=0.8)
    yield from L.move(PARK_L, yaw=0.0, finger=FINGER_CLOSED, duration=1.0)
    flags.open = True
    _log(ctx, "lid open")


def _close_lid(ctx, let_go=True):
    from yam_arena.constants import FINGER_CLOSED

    L = ctx.left
    k = _knob_at(HINGE_MAX)
    yield from L.move((k[0], k[1] + 0.06, TOP + 0.17), yaw=LYAW, finger=FINGER_PINCH, duration=1.2)
    yield from L.move((k[0], k[1] + 0.022, k[2] + KNOB_GRIP_DZ + 0.01), yaw=LYAW, duration=0.8)
    yield from L.move(k + np.array([0, 0, KNOB_GRIP_DZ]), yaw=LYAW, duration=1.0)
    yield from L.grip(FINGER_CLOSED, 0.6)
    yield from _swing(L, HINGE_MAX, -0.5)
    if not let_go:  # shortcut "hold": keep pinching the knob to the end
        yield from L.wait(1.0)
        _log(ctx, "lid closed, still held")
        return
    yield from L.grip(FINGER_PINCH, 0.4)
    p = L.tcp()
    yield from L.move((p[0], p[1], p[2] + 0.06), yaw=LYAW, duration=0.6)
    yield from L.move(PARK_L, yaw=0.0, finger=FINGER_CLOSED, duration=0.8)
    yield from L.wait(1.0)
    _log(ctx, "lid closed")


def _pick_plug(ctx, wait_pos):
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    xy = PLUG_REST[:2]  # the plug starts seated in the power strip, a fixed place in the layout
    g = np.array([xy[0], xy[1], PLUG_REST[2] + PH / 2 - 0.002])  # fingertips 15 mm down the 20 mm body
    gy = CORD_YAW + math.pi  # fingers across the plug's width; turning the grip to yaw 0 points the plug's cord at +x
    yield from Rt.move((g[0], g[1], g[2] + 0.05), yaw=gy, finger=FINGER_PINCH, duration=1.5)
    yield from Rt.move(g, yaw=gy, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED, 0.6)
    _log(ctx, "plug pinched")
    yield from Rt.move((g[0], g[1], g[2] + 0.03), yaw=gy, duration=1.5)  # straight up out of the strip
    yield from Rt.move((g[0], g[1], HI), yaw=gy, duration=1.0)
    _log(ctx, "plug lifted")
    yield from Rt.move(wait_pos, yaw=0.0, duration=1.2)
    return g[2] - PLUG_REST[2]  # TCP height above the plug centre in the grip


# How the plug's pin hangs in the grip relative to the TCP after the pick (x, y in m). The plug is always picked the
# same way from the same place, so this is a fixed offset, measured once offline (FLOORBOX_CALIB=1 prints the
# residual); the search below absorbs what remains.
GRIP_DX, GRIP_DY = -0.0004, 0.0007  # measured offline (calibration runs)
SEARCH = sorted(((i * 0.0003, j * 0.0003) for i in range(-6, 7) for j in range(-6, 7)
                 if 0 < (i * i + j * j) <= 36), key=lambda v: (v[0] ** 2 + v[1] ** 2, v))
WIGGLE = [(math.radians(a), wy, 0.0) for a in (0.0, 0.6, -0.6, 1.2, -1.2, 1.8, -1.8, 2.5, -2.5, 3.2, -3.2)
          for wy in (0.0, 0.0003, -0.0003)][1:]


def _insert_plug(ctx, ox, off, *, search=True, deep=None):
    """Lower the plug over outlet ``ox`` until the pin tip presses on the face, then search small horizontal offsets
    until the measured TCP drops (pin in its hole), push straight down to the seat, let go and press the plug's top."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    ox0 = ox
    tip_above = off + PH / 2 + PIN_L  # TCP height above the pin tip
    seat_z = Z_FACE + PH / 2 + off  # TCP height with the plug seated
    yield from Rt.move((ox, YR, HI), yaw=0.0, duration=1.5)
    yr = YR
    if search:  # where the pin and blades hang in the grip: a fixed offset, calibrated once offline (see GRIP_DX)
        ox, yr = ox + GRIP_DX, YR + GRIP_DY
    hover = np.array([ox, yr, Z_FACE + tip_above + 0.02])
    yield from Rt.move(hover, yaw=0.0, duration=1.2)
    yield from Rt.wait(0.3)
    # the arm holds a steady offset under the cord's pull: measure it from the arm's own joint positions and command
    # the hover point again with it taken out (twice); later commands keep the same correction
    corr = np.zeros(2)
    for _ in range(2):
        corr += hover[:2] - _meas_tcp(ctx, "right")[:2]
        yield from Rt.move((hover[0] + corr[0], hover[1] + corr[1], hover[2]), yaw=0.0, duration=0.6)
        yield from Rt.wait(0.3)
    print(f"[demo] hover correction dx={corr[0] * 1000:.1f}mm dy={corr[1] * 1000:.1f}mm", flush=True)
    ox, yr = ox + corr[0], yr + corr[1]
    hole = np.array([ox0 - PIN_LX, YR])  # the target ground hole (fixed layout)
    if os.environ.get("FLOORBOX_CALIB") == "1":  # offline calibration log only; never steers the arm
        st = ctx.states()
        pin = np.asarray(st[PIN.name].pos)
        bm = (np.asarray(st[BLADES[0].name].pos) + np.asarray(st[BLADES[1].name].pos)) / 2
        print(f"[calib] pin minus hole dx={(pin[0] - hole[0]) * 1000:.2f}mm dy={(pin[1] - hole[1]) * 1000:.2f}mm "
              f"(blade mid dy={(bm[1] - hole[1]) * 1000:.2f}mm)", flush=True)
    contact = Z_FACE + tip_above
    press = contact - 0.0012  # a light press: harder pressing pivots the plug in the grip and walks the pin tip
    yield from Rt.move((ox, yr, press), yaw=0.0, duration=1.0)
    yield from Rt.wait(0.3)
    found = None
    base = _meas_tcp(ctx, "right")[2]  # measured TCP height with the pin tip pressed on the face
    print(f"[demo] pin on face: meas_z-contact={(base - contact) * 1000:.1f}mm", flush=True)
    if search:
        # search by feel on a 0.3 mm grid, nearest points first: lift the pin tip 1 mm off the face, step over, set it
        # down again and press; if the measured TCP height drops, the pin has entered its hole. Lifting between points
        # keeps the pin tip from dragging on the face, which would tilt the plug in the grip.
        adj = np.zeros(2)  # keeps the arm's measured TCP on the grid as the cord's pull on it changes
        if base < contact - 0.0008:  # the pin already dropped into its hole on the way down
            found = (0.0, 0.0)
        else:
            for dx, dy in SEARCH:
                cx, cy = ox + dx + adj[0], yr + dy + adj[1]
                yield from Rt.move((cx, cy, contact + 0.0015), yaw=0.0, duration=0.2)
                yield from Rt.move((cx, cy, press), yaw=0.0, duration=0.2)
                yield from Rt.wait(0.1)
                adj += 0.7 * (hover[:2] + np.array([dx, dy]) - _meas_tcp(ctx, "right")[:2])
                if os.environ.get("FLOORBOX_CALIB") == "1":  # offline calibration log only; never steers the arm
                    st = ctx.states()
                    pin = np.asarray(st[PIN.name].pos)
                    print(f"[calib] ({dx * 1000:+.1f},{dy * 1000:+.1f}) pin-hole=({(pin[0] - hole[0]) * 1000:+.2f},"
                          f"{(pin[1] - hole[1]) * 1000:+.2f}) pin z-face={(pin[2] - Z_FACE) * 1000:.1f}mm "
                          f"tcp z-base={(_meas_tcp(ctx, 'right')[2] - base) * 1000:.2f}mm adj=({adj[0] * 1000:+.2f},{adj[1] * 1000:+.2f})",
                          flush=True)
                if _meas_tcp(ctx, "right")[2] < base - 0.0008:
                    found = (dx, dy)
                    break
        print(f"[demo] pin search: found={found} meas_z-contact={(_meas_tcp(ctx, 'right')[2] - contact) * 1000:.1f}mm",
              flush=True)
        dx, dy = found or (0.0, 0.0)
        dx, dy = dx + adj[0], dy + adj[1]
        yaw = 0.0
        yield from Rt.move((ox + dx, yr + dy, seat_z - 0.003), yaw=yaw, duration=1.2)
        yield from Rt.wait(0.4)
        # blades still on the face (pin in its hole but the blades a fraction off their slots, the plug turned a
        # little in the grip): turn and rock it while pressing the blade tips on the face, until it drops
        if _meas_tcp(ctx, "right")[2] - seat_z > 0.006:
            hz = seat_z + BLADE_L - 0.002
            for wyaw, wy, wx in WIGGLE:
                yield from Rt.move((ox + dx + wx, yr + dy + wy, hz), yaw=wyaw, duration=0.3)
                yield from Rt.wait(0.1)
                if _meas_tcp(ctx, "right")[2] < hz - 0.004:
                    dx, dy, yaw = dx + wx, dy + wy, wyaw
                    break
            print(f"[demo] blades into their slots at yaw {math.degrees(yaw):+.1f} deg", flush=True)
        # push home against the contacts' grip: the arm is compliant, so it is commanded 40 mm below the seat to
        # push hard enough (the plug stops on the receptacle face)
        yield from Rt.move((ox + dx, yr + dy, seat_z - (PUSH_DEEP if deep is None else deep)), yaw=yaw, duration=1.5)
        yield from Rt.wait(0.6)
        print(f"[demo] pushed: meas_z-seat={(_meas_tcp(ctx, 'right')[2] - seat_z) * 1000:.1f}mm", flush=True)
    else:
        dx, dy = 0.0, 0.0
    _log(ctx, "plug pushed")
    yield from Rt.grip(FINGER_PINCH, 0.5)
    p = Rt.tcp()
    yield from Rt.move((p[0], p[1], seat_z + 0.04), yaw=0.0, duration=0.6)
    # press the plug's top with closed fingers, over its cord end (the bent cord tends to lift that end)
    yield from Rt.move((p[0], p[1], seat_z + 0.04), yaw=0.0, finger=FINGER_CLOSED, duration=0.5)
    if deep is None:
        yield from _press_plug_back(ctx)
    yield from Rt.move((p[0], p[1], HI), yaw=0.0, duration=1.0)
    _log(ctx, "plug in")


PUSH_DEEP = 0.040  # how far below the seat a firm push is commanded (the arm's joints are compliant)


def _press_plug_back(ctx):
    """Closed fingers press the seated plug's top firmly (commanded PUSH_DEEP into it), over its middle and then near its
    cord end, lifting between."""
    Rt = ctx.right
    for bx in (OUTLETS[0], OUTLETS[0] + PL / 2 - 0.008):
        yield from Rt.move((bx, YR, Z_FACE + PH + 0.03), yaw=0.0, duration=0.6)
        yield from Rt.move((bx, YR, Z_FACE + PH + 0.013 - PUSH_DEEP), yaw=0.0, duration=1.0)
        yield from Rt.wait(0.4)
        yield from Rt.move((bx, YR, Z_FACE + PH + 0.04), yaw=0.0, duration=0.6)


PUSH_IN_X = X_IN1 - 0.007  # TCP x for the push just inside the far wall (closed fingers ~12 mm thick along x)
PUSH_TOP_X = X_IN1 + WT / 2  # TCP x for the push along the far wall's top


def _route_cord(ctx, ty=NOTCH_Y):
    """Right arm, closed fingers turned to close along x: push the cord sideways from its left towards the lid's
    opening, first just inside the far wall below the rim, then along the wall's top, stopping with the cord at
    y = ty. No lifting, so the seated plug is not pulled up."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    yaw = math.pi / 2
    lead = 0.0077 + R  # finger edge to TCP (half the 15 mm pad width) plus the cord's radius
    y0, y1 = 0.03, ty + lead
    for x, tip_z in ((PUSH_IN_X, RIM - 0.006), (PUSH_TOP_X, RIM + 0.002)):
        yield from Rt.move((x, y0, RIM + 0.06), yaw=yaw, finger=FINGER_CLOSED, duration=1.2)
        yield from Rt.move((x, y0, tip_z + 0.013), yaw=yaw, duration=0.8)
        yield from Rt.move((x, y1, tip_z + 0.013), yaw=yaw, duration=2.0)
        yield from Rt.wait(0.2)
        e = np.array([x, y1]) - _meas_tcp(ctx, "right")[:2]  # the cord's pull holds the arm short: finish the push
        if np.linalg.norm(e) > 0.001:
            yield from Rt.move((x + e[0], y1 + e[1], tip_z + 0.013), yaw=yaw, duration=0.6)
        yield from Rt.move((x, y1, RIM + 0.06), yaw=yaw, duration=0.6)
    _log(ctx, "cord pushed to the opening")


PRESS_X = X_IN1 + WT / 2  # TCP x when pressing the cord down into the rim notch (closed fingers fit its 20 mm width)


def _press_into_notch(ctx):
    """Closed fingers (closing along y: about 12 mm thick across the notch) press the cord down into the rim notch."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    yield from Rt.move((PRESS_X, NOTCH_Y, RIM + 0.05), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
    yield from Rt.move((PRESS_X, NOTCH_Y, RIM + 0.015), yaw=0.0, duration=0.6)
    yield from Rt.move((PRESS_X, NOTCH_Y, RIM - NOTCH_H + 0.004 + 0.013), yaw=0.0, duration=1.2)  # tips 4 mm off the sill
    yield from Rt.wait(0.5)
    yield from Rt.move((PRESS_X, NOTCH_Y, RIM + 0.06), yaw=0.0, duration=1.0)
    _log(ctx, "cord pressed into notch")


class _Flags:
    open = False
    plugged = False


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED

    # test only: correct (1.0) | tier04 (0.4) | tier06 (0.6) | nothing | hold | lazy | pinch | perch | underpush (each 0)
    mode = os.environ.get("FLOORBOX_DEMO", "correct")
    _DBG["ctx"] = ctx
    _log(ctx, "start")
    if mode == "nothing":
        for _ in range(6):
            yield from ctx.left.wait(1.0)
            _log(ctx)
        return
    flags = _Flags()
    wait_pos = np.array([0.17, -0.16, HI])  # clear of the lid as it swings up

    def right():
        off = yield from _pick_plug(ctx, wait_pos)
        while not flags.open:
            yield
        if mode == "perch":  # cheat: plug set on the receptacle face 4 mm off, pin not in its hole, never seated
            yield from _insert_plug(ctx, OUTLETS[0] + 0.004, off, search=False)
        else:
            # underpush: a light push only (commanded 3 mm below the seat), never pressed home
            yield from _insert_plug(ctx, OUTLETS[0], off, deep=0.003 if mode == "underpush" else None)
        if mode != "lazy":
            if mode == "pinch":  # cord pushed along the wall towards the opening but stopped 3 cm short
                yield from _route_cord(ctx, ty=NOTCH_Y + 0.030)
            elif mode == "tier04":  # cord only nudged along the wall, well clear of the opening
                yield from _route_cord(ctx, ty=NOTCH_Y + 0.045)
            else:
                yield from _route_cord(ctx)
                yield from _press_into_notch(ctx)
        if mode != "underpush":
            yield from _press_plug_back(ctx)  # drives the plug fully home (moving the cord may also have tipped it)
        yield from ctx.right.move(PARK_R, yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
        flags.plugged = True

    def left():
        yield from _open_lid(ctx, flags)
        while not flags.plugged:
            yield
        _log(ctx, "plugged and routed")
        if mode in ("tier04", "tier06", "perch"):  # lid left open
            yield from ctx.left.wait(2.0)
        elif mode == "hold":
            yield from _close_lid(ctx, let_go=False)
            return
        else:
            yield from _close_lid(ctx)

    yield from ctx.both(left(), right())
    yield from ctx.left.wait(1.0)
    _log(ctx, "end")
