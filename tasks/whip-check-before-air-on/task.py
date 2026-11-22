"""Couple a jackhammer air hose to a compressor's service manifold with its whip check on, then open the air.

Scene (metres from the arm-base midpoint, x forward, y left; heights above the table). A red service-manifold block
(x 0.27-0.37, y 0.17-0.24, 0.15 tall) stands left of centre. From its -y face a 3/4 in pipe nipple (26.7 mm OD) runs
8 cm towards -y at height 0.09 and ends in a fixed two-lug universal (Chicago) coupling head whose face is at
y 0.05. On top of the block's +y end sits a 3/4 in bronze ball valve: its 11.6 cm lever points at the robot (-x, closed,
across the pipe); turned 90 deg to point +y it is open. Beside the block at (0.310, 0.330), height 0.09, a steel peg
with an upturned lip holds the manifold-side loop of a Dixon WB1 whip-check safety cable (1/8 in galvanised wire rope,
20-1/4 in / 51.5 cm). The jackhammer hose (Kuriyama Dura-Jack 3/4 in ID, 29 mm OD, 0.6 kg/m, 0.54 m modelled)
lies on the right side along -y with its two-lug universal coupling head (40 mm body, 67 mm across the hooks) at
(0.35, -0.07) pointing +y. The whip check's hose-side loop starts cinched round the hose 7.5 cm behind the hose stem
(about 16 cm behind the coupling face), leaving slack once the coupling is connected until that loop is slid back.
Each loop is an 86 mm ID steel eye with a sprung sliding collar (40 N/m) and inline ferrule; squeezing the collar cuff
towards the ferrule draws the collar back to open the eye, and releasing lets the spring cinch the line. Two work-boot
and jeans leg forms (the breaker operator) stand beside the hose at x 0.66, y -0.22 and -0.38.

Order rule: at no time while the valve lever is past 10 deg may either loop be off/uncinched, the coupling unlocked,
or the whip-check slack above 3 cm. Scoring: see score().
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
RUN_LIMIT = 2400

INSTRUCTION = "I'm on the breaker. Hook my hose up to the manifold and turn the air on."

RED = (0.70, 0.10, 0.08)
STEEL = (0.62, 0.63, 0.66)
IRON = (0.32, 0.33, 0.35)
BRASS = (0.78, 0.62, 0.30)
HOSE_C = (0.62, 0.08, 0.06)
GALV = (0.70, 0.71, 0.73)
YELLOW = (0.95, 0.75, 0.10)
JEANS = (0.16, 0.22, 0.38)
BOOT = (0.30, 0.20, 0.12)

# -- manifold, nipple, fixed coupling head ---------------------------------------------------------------------------
LX = 0.32  # line axis x
LZ = TOP + 0.09  # line axis height
FACE_Y = 0.05  # gasket face plane (fixed head face)
MAN = Box("manifold", pos=(LX, 0.205, TOP + 0.075), size=(0.10, 0.07, 0.15), color=RED)
NIP_R, NIP_Y0, NIP_Y1 = 0.01335, 0.09, 0.17  # 3/4 in NPT nipple, 26.7 mm OD
BODY_R = 0.0200  # coupling head bodies (40 mm)
FIX_Y0, FIX_Y1 = FACE_Y + 0.0005, NIP_Y0  # 0.5 mm rubber-washer clearance behind gasket plane FACE_Y
LIP_Y0, LIP_Y1, LIP_R0, LIP_R1 = 0.055, 0.060, 0.0200, 0.0275  # fixed head lips (two arcs) behind the face
LIP_ARCS = ((15.0, 55.0), (195.0, 235.0))  # deg about the axis, from +x towards +z
STOP_ARCS = ((60.0, 72.0), (240.0, 252.0))  # rotation stops
HOOK_R0, HOOK_R1 = 0.0305, 0.0335  # hose-head hooks: radial span (67 mm across the lugs, 3 mm shank over 27.5 mm lips)
HOOK_Y1 = 0.0215  # hook reaches this far past the hose head's face
TOOTH_R0, TOOTH_Y0 = 0.0235, 0.0155  # tooth: from r 23.5 mm out to the bar, 15.5-21.5 mm past the face
HOOK_W = 0.006  # tangential width of hook/tooth
LOCK_MIN, LOCK_MAX = 25.0, 62.0  # deg of turn that puts both teeth behind the lips

# hose coupling head (free body, axial-torsion joined to the hose's start plug). Local frame: +y = towards the face
HH_L = 0.070  # body length incl. hose stem (face at +HH_L/2)
HH_MASS = float(os.environ.get("WC_HH_MASS", "0.27"))  # 3/4 in universal coupling, 0.6 lb
HOSE_R = 0.0145
EYE_RI, EYE_T, EYE_W = 0.043, 0.005, 0.010  # 86 mm inside, 5 mm wire, 10 mm along the axis (96 mm OD)
EYE_ZC = TOP + EYE_RI + EYE_T + 0.0005  # an eye standing on its rim
HX = 0.35  # hose starts on the right side along -y
HH_Y = -0.070
HH0 = (HX, HH_Y, EYE_ZC)
COUPLER = Box("hose_coupler", pos=HH0, size=(0.004, 0.004, 0.004), color=IRON, mass=HH_MASS)
HOSE_YAW = -math.pi / 2
HOSE_L = 0.54
HOSE_START = np.array([HX, HH0[1] - HH_L / 2 - 0.012, EYE_ZC])
HOSE = cable("air_hose", start=tuple(HOSE_START.tolist()), yaw=HOSE_YAW, length=HOSE_L, radius=HOSE_R, segment=0.035,
             start_plug=Plug(size=(0.020, 0.030, 0.030), mass=0.03, color=IRON), cable_mass=0.6 * HOSE_L,
             bend_stiffness=float(os.environ.get("WC_HOSE_K", "0.02")), bend_damping=0.004, friction=0.8,
             color=HOSE_C)

# -- whip check: two eyes (ring + inline sprung collar + ferrule) and the 1/8 in cable --------------------------------
EYE_N = 18
SLOT_DEG = 14.0  # gap in the ring wall at +x for the collar
COL_H, COL_W = 0.026, 0.008  # collar bar height (z) and depth (along the axis)
COL_LEN = 0.104  # collar bar length along x; its inner face is at xi
COL_CUFF = 0.100  # flanged spring-cuff shoulder x offset from collar inner face (28 mm outside 48 mm ring OD)
CUFF_H = 0.036  # cuff flange height (z), proud above central wire/spring rails
COL_XI_CLOSED, COL_XI_OPEN = -0.020, 0.034  # stroke limits of the collar inner face (eye frame x)
FER_X = 0.142  # fixed inline ferrule stop (eye frame x); cuff and ferrule sit 58-64 mm apart when closed
FER_H = 0.030  # ferrule height (z), proud above central wire/spring rails
SPRING_K = float(os.environ.get("WC_SPRING_K", "40.0"))  # N/m on the collar
SPRING_TARGET = -0.034
CINCH_XI = 0.000  # collar inner face at or below ring centre (0 mm) = cinched onto a line (empty closes to -20 mm; open >= +4 mm)
EYE_MASS, COL_MASS = 0.035, 0.010
EAR = (FER_X + 0.006, 0.0)  # cable leaves inline out of the ferrule (eye frame x, z)
EYE_H_S = 0.075  # hose-side eye starts 7.5 cm along the hose from its stem (~16 cm behind coupling face, seg_2)
_eh = HOSE_START + EYE_H_S * np.array([0.0, -1.0, 0.0])
# Manifold eye hangs at yaw=pi on its peg; hose eye starts angled slightly (+17 deg) leaving slack during setup
EYE_H0 = (float(_eh[0] + HOSE_R - EYE_RI), float(_eh[1]), EYE_ZC)
EYE_M0 = (0.310, 0.330, EYE_ZC)  # manifold-side eye hangs on a lipped peg beside the manifold
EYES = {"h": (EYE_H0, math.pi - 0.30), "m": (EYE_M0, math.pi)}
POST = Box("eye_post", pos=(EYE_M0[0], EYE_M0[1] + 0.045, TOP + 0.05), size=(0.024, 0.024, 0.10), color=STEEL)
PEG = Box("eye_peg", pos=(EYE_M0[0], EYE_M0[1] + 0.012, EYE_ZC + EYE_RI - 0.006), size=(0.018, 0.042, 0.010),
          color=STEEL)
PEG_LIP = Box("eye_peg_lip", pos=(EYE_M0[0], EYE_M0[1] - 0.011, EYE_ZC + EYE_RI), size=(0.020, 0.004, 0.022),
              color=STEEL)  # upturned tip on -y: the eye must be lifted 1.2 cm to slide off
EYE_OBJ = {k: Box(f"eye_{k}", pos=p, size=(0.003, 0.003, 0.003), color=GALV, mass=EYE_MASS, yaw=yw)
           for k, (p, yw) in EYES.items()}


def _eye_world(k, local):
    p, yw = EYES[k]
    c, s = math.cos(yw), math.sin(yw)
    lx, ly, lz = local
    return np.array([p[0] + c * lx - s * ly, p[1] + s * lx + c * ly, p[2] + lz])


COL0_XI = {"h": -(EYE_RI - 2 * HOSE_R) + 0.001, "m": COL_XI_CLOSED + 0.001}
COL_OBJ = {k: Box(f"collar_{k}", pos=tuple(_eye_world(k, (COL0_XI[k] + COL_LEN / 2, 0.0, 0.0)).tolist()),
                  size=(0.003, 0.003, 0.003), color=GALV, mass=COL_MASS, yaw=EYES[k][1]) for k in EYES}
_ear_h0 = _eye_world("h", (EAR[0], 0.0, EAR[1]))
_ear_m0 = _eye_world("m", (EAR[0], 0.0, EAR[1]))
_wc_vec = _ear_m0 - _ear_h0
WC_LEN = float(np.linalg.norm(_wc_vec))  # ~0.479 m ferrule-to-ferrule (~0.515 m overall)
WC_PLUG = 0.006
_wc_dir = _wc_vec / WC_LEN
WC_START = _ear_h0 + (WC_PLUG / 2) * _wc_dir
WC_YAW = float(math.atan2(float(_wc_vec[1]), float(_wc_vec[0])))
WHIP = cable("whip_check", start=tuple(WC_START.tolist()), yaw=WC_YAW, length=WC_LEN - WC_PLUG,
             radius=0.0016, segment=0.028, start_plug=Plug(size=(WC_PLUG, 0.006, 0.006), mass=0.005, color=GALV),
             end_plug=Plug(size=(WC_PLUG, 0.006, 0.006), mass=0.005, color=GALV), cable_mass=0.06,
             bend_stiffness=0.004, bend_damping=0.0015, armature=1e-4, color=GALV)

# -- valve -----------------------------------------------------------------------------------------------------------
VALVE_Y = 0.230  # valve on the far (+y) end of the block, clear of the whip-check peg
VALVE_BODY = Box("valve_body", pos=(LX, VALVE_Y, TOP + 0.170), size=(0.040, 0.066, 0.040), color=BRASS)
LEVER_PIVOT = (LX, VALVE_Y, TOP + 0.196)
LEVER_L = 0.116
LEVER = Box("valve_lever", pos=(LX - LEVER_L / 2 + 0.010, VALVE_Y, TOP + 0.196), size=(LEVER_L, 0.016, 0.008),
            color=YELLOW, mass=0.05)
VALVE_OPEN_DEG = 80.0
VALVE_TRIP_DEG = 10.0  # UPBA400 PTFE seat-seal deadband before ball port cracks open to admit air

# -- operator leg forms ----------------------------------------------------------------------------------------------
LEGS = [Box(f"leg_{i}", pos=(0.66, y, TOP + 0.28), size=(0.11, 0.11, 0.40), color=JEANS) for i, y in ((0, -0.22), (1, -0.38))]
BOOTS = [Box(f"boot_{i}", pos=(0.64, y, TOP + 0.04), size=(0.26, 0.11, 0.08), color=BOOT) for i, y in ((0, -0.22), (1, -0.38))]

OBJECTS = [MAN, POST, PEG, PEG_LIP, VALVE_BODY, *LEGS, *BOOTS, COUPLER, LEVER, *EYE_OBJ.values(), *COL_OBJ.values(), HOSE, WHIP]

TWIN_EXTRA = [
    {"name": "fixed_coupling_head", "kind": "body", "size_mm": [69, 40, 69], "colour": "iron",
     "note": "3/4 in two-lug universal (Chicago) coupling, male NPT, on a 3/4 in nipple in the manifold"},
    {"name": "hose_coupling_head", "kind": "body", "size_mm": [67, 70, 67], "colour": "iron",
     "note": "universal coupling on the jackhammer hose; two hooks lock behind the fixed head's lips"},
    {"name": "hose_stem_torsion", "kind": "joint", "real": "rubber hose torsional compliance",
     "note": "axial revolute +-80 deg with 0.005 N*m/deg spring"},
    {"name": "whip_check_eye", "kind": "body", "size_mm": [96, 14, 96], "colour": "galvanised",
     "note": "x2; 86 mm ID steel eye with inline sprung collar cuff and ferrule"},
    {"name": "collar_slide", "kind": "joint", "real": "spring collar on the cable loop",
     "note": f"prismatic, {SPRING_K:.0f} N/m spring, stroke {COL_XI_OPEN - COL_XI_CLOSED:.3f} m"},
    {"name": "valve_pivot", "kind": "joint", "real": "3/4 in ball valve stem", "note": "0-90 deg, damped stem"},
]

_ST: dict = {"env": None}


# -- spawn hook: compound colliders, joints, welds -------------------------------------------------------------------
def _ring_boxes():
    """(name, centre (x, y, z) eye frame, size, rot about +y in rad) for the ring wall and inline ferrule stem."""
    out = []
    rm = EYE_RI + EYE_T / 2
    step = 360.0 / EYE_N
    for k in range(EYE_N):
        psi = math.radians(k * step)
        half = math.radians(step / 2)
        lo, hi = psi - half, psi + half
        if abs(math.degrees((psi + math.pi) % (2 * math.pi) - math.pi)) < SLOT_DEG / 2 + step / 2:
            for a0, a1 in ((math.radians(SLOT_DEG / 2), hi), (lo, -math.radians(SLOT_DEG / 2))):
                if a1 <= a0:
                    continue
                m = (a0 + a1) / 2
                ln = 2 * rm * math.tan((a1 - a0) / 2) + 0.001
                out.append((f"w{k}_{'u' if m > 0 else 'd'}", (rm * math.cos(m), 0.0, rm * math.sin(m)),
                            (ln, EYE_W, EYE_T), -(m + math.pi / 2)))
            continue
        ln = 2 * rm * math.tan(half) + 0.001
        out.append((f"w{k}", (rm * math.cos(psi), 0.0, rm * math.sin(psi)), (ln, EYE_W, EYE_T), -(psi + math.pi / 2)))
    ro = EYE_RI + EYE_T
    # inline central wire/spring rails out of the ring slot (+-5.5 mm) to the crimped end ferrule
    out.append(("stem_u", ((ro + FER_X) / 2, 0.0, 0.004), (FER_X - ro + 0.004, EYE_W, 0.004), 0.0))
    out.append(("stem_d", ((ro + FER_X) / 2, 0.0, -0.004), (FER_X - ro + 0.004, EYE_W, 0.004), 0.0))
    out.append(("ferrule", (FER_X, 0.0, 0.0), (0.008, 0.014, FER_H), 0.0))
    return out


def _collar_boxes():
    """Collar frame origin = collar body centre (front cinch saddle at -COL_LEN/2, slender guide, rear cuff at COL_CUFF)."""
    return [("saddle", (-COL_LEN / 2 + 0.007, 0.0, 0.0), (0.014, COL_W, COL_H), 0.0),
            ("rod", (0.0, 0.0, 0.0), (COL_LEN, 0.006, 0.006), 0.0),
            ("cuff", (-COL_LEN / 2 + COL_CUFF, 0.0, 0.0), (0.008, 0.014, CUFF_H), 0.0)]


def _arc_boxes(prefix, arcs, r0, r1, y0, y1):
    out = []
    for i, (a0, a1) in enumerate(arcs):
        n = max(2, int(math.ceil((a1 - a0) / 10.0)))
        for j in range(n):
            b0 = math.radians(a0 + (a1 - a0) * j / n)
            b1 = math.radians(a0 + (a1 - a0) * (j + 1) / n)
            m = (b0 + b1) / 2
            rm = (r0 + r1) / 2
            ln = 2 * r1 * math.tan((b1 - b0) / 2) + 0.0008
            out.append((f"{prefix}{i}_{j}", (rm * math.cos(m), (y0 + y1) / 2, rm * math.sin(m)), (ln, y1 - y0, r1 - r0),
                        -(m + math.pi / 2)))
    return out


def _hook_boxes():
    """Hose-head hooks in the coupler frame (origin body centre, +y to the face, hooks at psi 0 and 180)."""
    face = HH_L / 2
    out = []
    for i, psi in enumerate((0.0, math.pi)):
        c, s = math.cos(psi), math.sin(psi)
        rb = (HOOK_R0 + HOOK_R1) / 2
        yb0, yb1 = face - 0.012, face + HOOK_Y1
        out.append((f"hook{i}", (rb * c, (yb0 + yb1) / 2, rb * s), (HOOK_R1 - HOOK_R0, yb1 - yb0, HOOK_W), -psi))
        rt = (TOOTH_R0 + HOOK_R0) / 2
        out.append((f"tooth{i}", (rt * c, face + (TOOTH_Y0 + HOOK_Y1) / 2, rt * s),
                    (HOOK_R0 - TOOTH_R0 + 0.001, HOOK_Y1 - TOOTH_Y0, HOOK_W), -psi))
    return out


def _author_mech_slots(stage, root: str, man: str, slots_cfg: str) -> None:
    """Author disabled structural FixedJoint slots for hose-loop re-cinching and in-situ coupling lock."""
    from pxr import Gf, Sdf, UsdPhysics

    def mech_slot(name, body0, body1):
        j = UsdPhysics.FixedJoint.Define(stage, f"{root}/wc_joints/{name}")
        if body0:
            j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateJointEnabledAttr(False)

    for j_ in (() if "cinch" not in slots_cfg else range(HOSE.segments)):
        mech_slot(f"cinch_{j_}", f"{root}/{EYE_OBJ['h'].name}", f"{root}/{HOSE.name}/seg_{j_}")
    if "lock" in slots_cfg:
        mech_slot("lock", man, f"{root}/{COUPLER.name}")


def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_wc_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != WHIP.name:  # the last object: every prim exists now
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        _ST["root"] = root
        looks = f"{root}/WCLooks"
        mats = {}
        for key, col, met in (("iron", IRON, 0.5), ("galv", GALV, 0.6), ("steel", STEEL, 0.5), ("red", RED, 0.0)):
            p = f"{looks}/{key}"
            m = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(col), roughness=0.45, metallic=met)
            m.func(p, m)
            mats[key] = p
        steel_phys = f"{looks}/steel_phys"
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=0.6, dynamic_friction=0.5)
        pm.func(steel_phys, pm)
        grip_phys = f"{looks}/grip_phys"
        pg = sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=0.9)
        pg.func(grip_phys, pg)
        ring_phys = f"{looks}/ring_phys"
        pr = sim_utils.RigidBodyMaterialCfg(static_friction=0.05, dynamic_friction=0.04, friction_combine_mode="min")
        pr.func(ring_phys, pr)

        def qy(rot):
            return Gf.Quatf(math.cos(rot / 2), 0.0, math.sin(rot / 2), 0.0)

        def cube(path, c, s, rot, look, phys):
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            g.AddOrientOp().Set(qy(rot))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, s)))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(g.GetPath()), phys, stage=stage)
            bind_visual_material(str(g.GetPath()), look, stage=stage)

        def cyl_y(path, c, r, h, look, phys):
            g = UsdGeom.Cylinder.Define(stage, path)
            g.CreateRadiusAttr(float(r))
            g.CreateHeightAttr(float(h))
            g.CreateAxisAttr("Y")
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(g.GetPath()), phys, stage=stage)
            bind_visual_material(str(g.GetPath()), look, stage=stage)

        def strip_placeholder(path):
            """Disable collision and hide the placeholder geometry/mesh cube spawned on compound rigid bodies."""
            p = stage.GetPrimAtPath(path)
            for desc in Usd.PrimRange(p):
                if desc.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(desc).CreateCollisionEnabledAttr(False)
            gp = stage.GetPrimAtPath(f"{path}/geometry")
            if gp.IsValid():
                UsdGeom.Imageable(gp).MakeInvisible()

        def filt(p0, p1):
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(p0)).CreateFilteredPairsRel().AddTarget(Sdf.Path(p1))

        def xf(path):
            return UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)

        def swivel(body0, body1, name):
            rel = xf(body1) * xf(body0).GetInverse()
            t, q = rel.ExtractTranslation(), rel.ExtractRotationQuat()
            w = UsdPhysics.SphericalJoint.Define(stage, f"{body0}/{name}")
            w.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
            w.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
            w.CreateLocalPos0Attr(Gf.Vec3f(*map(float, t)))
            w.CreateLocalRot0Attr(Gf.Quatf(float(q.GetReal()), *map(float, q.GetImaginary())))
            w.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            w.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            w.CreateExcludeFromArticulationAttr(True)
            filt(body0, body1)

        # fixed head + nipple: static colliders under the manifold block (its frame = world axes, origin MAN.pos)
        man = f"{root}/{MAN.name}"
        mo = np.asarray(MAN.pos)
        cyl_y(f"{man}/nipple", (LX - mo[0], (NIP_Y0 + NIP_Y1) / 2 - mo[1], LZ - mo[2]), NIP_R, NIP_Y1 - NIP_Y0 + 0.004,
              mats["steel"], steel_phys)
        cyl_y(f"{man}/fixed_head", (LX - mo[0], (FIX_Y0 + FIX_Y1) / 2 - mo[1], LZ - mo[2]), BODY_R, FIX_Y1 - FIX_Y0,
              mats["iron"], steel_phys)
        for name, c, s, rot in (_arc_boxes("lip", LIP_ARCS, LIP_R0, LIP_R1, LIP_Y0, LIP_Y1)
                                + _arc_boxes("stop", STOP_ARCS, BODY_R - 0.001, HOOK_R1 + 0.001, FACE_Y + 0.001, 0.074)):
            cube(f"{man}/{name}", (LX - mo[0] + c[0], c[1] - mo[1], LZ - mo[2] + c[2]), s, rot, mats["iron"], steel_phys)

        def tune_rb(path):
            rb = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(path))
            rb.CreateMaxDepenetrationVelocityAttr(1.0)
            rb.CreateMaxLinearVelocityAttr(3.0)
            rb.CreateMaxAngularVelocityAttr(25.0)
            rb.CreateSolverPositionIterationCountAttr(32)
            rb.CreateSolverVelocityIterationCountAttr(8)

        # hose coupling head: body + hooks under the free coupler box, axial-torsion joined to the hose's start plug
        cp_ = f"{root}/{COUPLER.name}"
        strip_placeholder(cp_)
        tune_rb(cp_)
        cyl_y(f"{cp_}/body", (0.0, 0.0, 0.0), BODY_R, HH_L, mats["iron"], grip_phys)
        for name, c, s, rot in _hook_boxes():
            cube(f"{cp_}/{name}", c, s, rot, mats["iron"], steel_phys)
        hp = f"{root}/{HOSE.name}/start_plug"
        rel_hp = xf(hp) * xf(cp_).GetInverse()
        thp, qhp = rel_hp.ExtractTranslation(), rel_hp.ExtractRotationQuat()
        hj = UsdPhysics.RevoluteJoint.Define(stage, f"{cp_}/hose_stem")
        hj.CreateBody0Rel().SetTargets([Sdf.Path(cp_)])
        hj.CreateBody1Rel().SetTargets([Sdf.Path(hp)])
        hj.CreateLocalPos0Attr(Gf.Vec3f(*map(float, thp)))
        hj.CreateLocalRot0Attr(Gf.Quatf(float(qhp.GetReal()), *map(float, qhp.GetImaginary())))
        hj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        hj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        hj.CreateAxisAttr("X")  # start_plug +X is along the hose axis
        hj.CreateLowerLimitAttr(-80.0)
        hj.CreateUpperLimitAttr(80.0)
        hj.CreateExcludeFromArticulationAttr(True)
        hdrv = UsdPhysics.DriveAPI.Apply(hj.GetPrim(), "angular")
        hdrv.CreateTypeAttr("force")
        hdrv.CreateStiffnessAttr(0.005)
        hdrv.CreateDampingAttr(0.002)
        hdrv.CreateTargetPositionAttr(0.0)
        filt(cp_, hp)
        filt(cp_, f"{root}/{HOSE.name}/seg_0")

        # eyes and collars (full physical collision with the coupling head and nipple)
        for k in EYES:
            ep, cl = f"{root}/{EYE_OBJ[k].name}", f"{root}/{COL_OBJ[k].name}"
            strip_placeholder(ep)
            strip_placeholder(cl)
            tune_rb(ep)
            tune_rb(cl)
            for name, c, s, rot in _ring_boxes():
                cube(f"{ep}/{name}", c, s, rot, mats["galv"], ring_phys if k == "h" else steel_phys)
            for sc_i, sc_x in enumerate(np.linspace(EYE_RI + EYE_T + 0.056, FER_X - 0.008, 6)):
                coil = UsdGeom.Cylinder.Define(stage, f"{ep}/spring_coil_{sc_i}")
                coil.CreateRadiusAttr(0.0075)
                coil.CreateHeightAttr(0.0025)
                coil.CreateAxisAttr("X")
                coil.AddTranslateOp().Set(Gf.Vec3d(float(sc_x), 0.0, 0.0))
                bind_visual_material(str(coil.GetPath()), mats["steel"], stage=stage)
            for name, c, s, rot in _collar_boxes():
                cube(f"{cl}/{name}", c, s, rot, mats["galv"], grip_phys)
            filt(ep, cl)
            j = UsdPhysics.PrismaticJoint.Define(stage, f"{ep}/collar_slide")
            j.CreateBody0Rel().SetTargets([Sdf.Path(ep)])
            j.CreateBody1Rel().SetTargets([Sdf.Path(cl)])
            j.CreateLocalPos0Attr(Gf.Vec3f(float(COL0_XI[k] + COL_LEN / 2), 0.0, 0.0))
            j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateAxisAttr("X")
            j.CreateLowerLimitAttr(float(COL_XI_CLOSED - COL0_XI[k]))
            j.CreateUpperLimitAttr(float(COL_XI_OPEN - COL0_XI[k]))
            j.CreateExcludeFromArticulationAttr(True)
            drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
            drv.CreateTypeAttr("force")
            drv.CreateStiffnessAttr(SPRING_K)
            drv.CreateDampingAttr(8.0)
            drv.CreateTargetPositionAttr(float(SPRING_TARGET - COL0_XI[k]))
            end = "start" if k == "h" else "end"
            plug = f"{root}/{WHIP.name}/{end}_plug"
            swivel(ep, plug, f"cable_weld_{k}")
            filt(cl, plug)
            for i in (0, 1) if k == "h" else (WHIP.segments - 2, WHIP.segments - 1):
                filt(ep, f"{root}/{WHIP.name}/seg_{i}")
                filt(cl, f"{root}/{WHIP.name}/seg_{i}")
        for b_ in [f"{root}/{WHIP.name}/start_plug", f"{root}/{WHIP.name}/end_plug",
                   *[f"{root}/{WHIP.name}/seg_{i}" for i in range(WHIP.segments)]]:
            if stage.GetPrimAtPath(b_).IsValid():
                tune_rb(b_)

        # valve lever on its stem (damped angular drive models ball-valve packing drag)
        lv = f"{root}/{LEVER.name}"
        vb = f"{root}/{VALVE_BODY.name}"
        filt(lv, vb)
        filt(lv, man)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{lv}/stem")
        j.CreateBody0Rel().SetTargets([Sdf.Path(vb)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(lv)])
        vo, lo = np.asarray(VALVE_BODY.pos), np.asarray(LEVER.pos)
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, np.asarray(LEVER_PIVOT) - vo)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, np.asarray(LEVER_PIVOT) - lo)))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateAxisAttr("Z")
        j.CreateLowerLimitAttr(-90.0)
        j.CreateUpperLimitAttr(0.0)
        j.CreateExcludeFromArticulationAttr(True)
        vdrv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        vdrv.CreateTypeAttr("force")
        vdrv.CreateStiffnessAttr(0.0)
        vdrv.CreateDampingAttr(0.02)
        vdrv.CreateTargetVelocityAttr(0.0)
        PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(lv)).CreateAngularDampingAttr(8.0)

        # 6-DOF force-capped compliant pad-grip joints (no rigid gripper weld; slips above pad friction/torque caps)
        def grip_slot(name, body0, body1):
            jj = UsdPhysics.Joint.Define(stage, f"{root}/wc_joints/{name}")
            jj.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
            jj.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
            jj.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
            jj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            jj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            jj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            jj.CreateExcludeFromArticulationAttr(True)
            jj.CreateJointEnabledAttr(False)
            prim = jj.GetPrim()
            d2r = math.pi / 180.0
            for ax in ("transX", "transY", "transZ"):
                d = UsdPhysics.DriveAPI.Apply(prim, ax)
                d.CreateTypeAttr("force")
                d.CreateStiffnessAttr(GRIP_K)
                d.CreateDampingAttr(GRIP_C)
                d.CreateMaxForceAttr(GRIP_HOLD_N)
                d.CreateTargetPositionAttr(0.0)
            for ax, cap in (("rotX", GRIP_TWIST_NM), ("rotY", GRIP_TIP_NM), ("rotZ", GRIP_TIP_NM)):
                d = UsdPhysics.DriveAPI.Apply(prim, ax)  # USD angular gains are per degree
                d.CreateTypeAttr("force")
                d.CreateStiffnessAttr(GRIP_KR * d2r)
                d.CreateDampingAttr(GRIP_CR * d2r)
                d.CreateMaxForceAttr(cap)
                d.CreateTargetPositionAttr(0.0)

        _sl = os.environ.get("WC_SLOTS", "grasp,cinch,lock")
        for side in (() if "grasp" not in _sl else ("Left", "Right")):
            gpath = f"{root}/{side}Arm/Geometry/base/link1/link2/link3/link4/link5/gripper"
            for b in (COUPLER.name, EYE_OBJ["m"].name, EYE_OBJ["h"].name):
                grip_slot(f"grasp_{side.lower()}_{b}", gpath, f"{root}/{b}")
        _enable_contact_processing()
        for cname in (COUPLER.name, EYE_OBJ["m"].name, EYE_OBJ["h"].name, COL_OBJ["m"].name, COL_OBJ["h"].name):
            cp_p = stage.GetPrimAtPath(f"{root}/{cname}")
            if cp_p.IsValid():
                PhysxSchema.PhysxContactReportAPI.Apply(cp_p).CreateThresholdAttr(0.0)
        for prim_ in stage.Traverse(Usd.TraverseInstanceProxies()):
            ps_ = prim_.GetPath().pathString
            if ps_.startswith(root + "/") and ("/LeftArm/" in ps_ or "/RightArm/" in ps_):
                if ps_.endswith(("/tip_left", "/tip_right")) and prim_.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(prim_).CreateThresholdAttr(0.0)
                if ("/tip_left" in ps_ or "/tip_right" in ps_) and prim_.HasAPI(UsdPhysics.CollisionAPI):
                    bind_physics_material(ps_, grip_phys, stage=stage)
        _author_mech_slots(stage, root, man, _sl)
        if not os.environ.get("WC_NOCINCH"):
            sp = HOSE.start_points()
            kseg = int(np.argmin(np.linalg.norm(sp - np.array([HX, HOSE_START[1] - EYE_H_S, EYE_ZC]), axis=1)))
            Te = np.eye(4)
            ce, se = math.cos(EYES["h"][1]), math.sin(EYES["h"][1])
            Te[:3, :3] = [[ce, -se, 0], [se, ce, 0], [0, 0, 1]]
            Te[:3, 3] = EYE_H0
            Ts_ = np.eye(4)
            c_, s_ = math.cos(HOSE_YAW), math.sin(HOSE_YAW)
            Ts_[:3, :3] = [[c_, -s_, 0], [s_, c_, 0], [0, 0, 1]]
            Ts_[:3, 3] = sp[kseg]
            rel = np.linalg.inv(Te) @ Ts_
            jp = stage.GetPrimAtPath(f"{root}/wc_joints/cinch_{kseg}")
            jp.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, rel[:3, 3])))
            w_, x_, y_, z_ = _quat_wxyz(rel[:3, :3])
            jp.GetAttribute("physics:localRot0").Set(Gf.Quatf(w_, x_, y_, z_))
            jp.GetAttribute("physics:jointEnabled").Set(True)
            _ST["cinch0"] = str(jp.GetPath())
            _H["cinch"] = _ST["cinch0"]
            print(f"[wc] hose loop cinched to seg_{kseg}", flush=True)
        print(f"[wc] spawn: fixed head, coupler, 2 eyes, valve; whip {WC_LEN:.3f} m, {WHIP.segments} segs", flush=True)
        return prim

    spawn._wc_hook = True
    cablekit._spawn_cable = spawn


_install_spawn()


# -- force-limited two-pad-contact pad grip, collar squeeze, hose re-cinch and coupling lock (see README, fidelity) ---
PAD_CONTACT_MIN_N = 0.05  # minimum PhysX normal contact force (N) required simultaneously on BOTH finger pads
GRIP_SQUEEZE_N = 50.0     # YAM finger actuator effort limit (yam_arena GRIPPER effort, i2rt linear_4310)
GRIP_MU = 0.5             # elastomer finger-pad static friction coefficient
GRIP_HOLD_N = 2.0 * GRIP_MU * GRIP_SQUEEZE_N   # 50 N pull cap (2 pads * mu 0.5 * 50 N)
GRIP_TWIST_NM = GRIP_HOLD_N * 0.010            # 0.50 N*m twist cap about pad normal (10 mm mean patch radius)
GRIP_TIP_NM = 2.0 * GRIP_SQUEEZE_N * 0.015     # 1.50 N*m tipping cap in pad plane (15 mm half pad length)
GRIP_K, GRIP_C = 20000.0, 200.0                # N/m, N*s/m pad shear stiffness (~1 mm per 20 N) and damping
GRIP_KR, GRIP_CR = 120.0, 1.2                  # N*m/rad, N*m*s/rad angular stiffness and damping
CPL_CMD_CLOSE = 0.038     # commanded jaw opening (m) at or below which jaws clamp the 40 mm coupling body
EYE_CMD_CLOSE = 0.024     # commanded jaw opening (m) at or below which jaws squeeze the 16 mm cuff+ferrule stack
GRASP_OPEN_CPL = 0.048    # commanded opening above which the hand releases the coupling body
GRASP_OPEN_EYE = 0.036    # commanded opening above which the hand releases a whip-check eye
EYE_SQUEEZE_XI = 0.004    # collar must be physically compressed past this stroke (m) by finger pads before eye grasp
SEAT_TOL = 0.0045         # hose face within 4.5-5.0 mm of gasket seat (hooks past 5.5-6.0 cm lips), axes within 8 deg
LOCK_ENGAGE_MIN = 32.0
_H: dict = {}


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
    if getattr(cls.__init__, "_wc_contacts", False):
        return
    orig_init = cls.__init__

    def __init__(self, *a, **kw):
        kw["contact_sensors"] = True
        return orig_init(self, *a, **kw)

    __init__._wc_contacts = True
    cls.__init__ = __init__


class _EmbodimentFinder(importlib.abc.MetaPathFinder):
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
elif not any(isinstance(f, _EmbodimentFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _EmbodimentFinder())


def _reset_hacks() -> None:
    """New trial: drop grasp welds and the coupling lock (the stage keeps disabled joint prims)."""
    st = _stage_or_none()
    for key in ("welds", "seat", "lock"):
        for path in (list(_H.get(key, {}).values()) if key == "welds" else [_H.get(key)]):
            if st is not None and path:
                p = st.GetPrimAtPath(path)
                if p.IsValid():
                    p.GetAttribute("physics:jointEnabled").Set(False)
    if st is not None and _ST.get("cinch0"):
        p = st.GetPrimAtPath(_ST["cinch0"])
        if p.IsValid():
            p.GetAttribute("physics:jointEnabled").Set(True)
    if st is not None and _H.get("cinch") and _H.get("cinch") != _ST.get("cinch0"):
        p = st.GetPrimAtPath(_H["cinch"])
        if p.IsValid():
            p.GetAttribute("physics:jointEnabled").Set(False)
    if st is not None and _ST.get("root"):
        for prim_ in st.Traverse():
            ps_ = prim_.GetPath().pathString
            if ps_.startswith(_ST["root"] + "/") and ("/LeftArm/" in ps_ or "/RightArm/" in ps_):
                if ps_.endswith(("/tip_left", "/tip_right")):
                    from pxr import PhysxSchema, UsdPhysics

                    if prim_.HasAPI(UsdPhysics.RigidBodyAPI) and not prim_.HasAPI(PhysxSchema.PhysxContactReportAPI):
                        PhysxSchema.PhysxContactReportAPI.Apply(prim_).CreateThresholdAttr(0.0)
    n = _H.get("n", 0)
    _H.clear()
    _H.update(welds={}, held={}, anchors={}, slips=0, was_open={}, inserted=False, seat=None, lock=None, n=n,
              gripper_paths=None, cinch=_ST.get("cinch0"))
    for k in EYES:
        _collar_target(k, False)


def _stage_or_none():
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        return get_current_stage()
    except Exception:  # noqa: BLE001
        return None


def _collar_target(k, opened: bool) -> None:
    st = _stage_or_none()
    root = _ST.get("root")
    if st is None or root is None:
        return
    p = st.GetPrimAtPath(f"{root}/{EYE_OBJ[k].name}/collar_slide")
    if p.IsValid():
        x = (COL_XI_OPEN + 0.020 if opened else SPRING_TARGET) - COL0_XI[k]
        p.GetAttribute("drive:linear:physics:targetPosition").Set(float(x))


def _gripper_paths():
    if _H.get("gripper_paths"):
        return _H["gripper_paths"]
    st = _stage_or_none()
    out = [None, None]
    for p in st.Traverse():
        if p.GetName() != "gripper" or "env_0" not in str(p.GetPath()) or p.GetParent().GetName() != "link5":
            continue
        path = str(p.GetPath())
        for i, side in enumerate(("Left", "Right")):
            if (f"/{side}Arm/" in path or f"/{side.lower()}_robot/" in path) and out[i] is None:
                out[i] = path
    _H["gripper_paths"] = out
    return out


def _quat_wxyz(R):
    from scipy.spatial.transform import Rotation

    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return float(w), float(x), float(y), float(z)


def _rotvec(R: np.ndarray) -> np.ndarray:
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = float(np.linalg.norm(v))
    return v / s * ang if s > 1e-12 else np.zeros(3)


def _exp_rotvec(rv: np.ndarray) -> np.ndarray:
    ang = float(np.linalg.norm(rv))
    if ang < 1e-9:
        return np.eye(3)
    ux, uy, uz = (rv / ang).tolist()
    K = np.array([[0.0, -uz, uy], [uz, 0.0, -ux], [-uy, ux, 0.0]])
    return np.eye(3) + math.sin(ang) * K + (1.0 - math.cos(ang)) * (K @ K)


def _engage_grip(side: str, name: str, G: np.ndarray, T1: np.ndarray, mid_pad: np.ndarray):
    """Anchor the 6-DOF force-limited pad-grip joint at mid_pad with zero snap."""
    from pxr import Gf

    st = _stage_or_none()
    path = f"{_ST['root']}/wc_joints/grasp_{side}_{name}"
    p = st.GetPrimAtPath(path)
    gp, gR = G[:3, 3], G[:3, :3]
    kp, kR = T1[:3, 3], T1[:3, :3]
    p0 = gR.T @ (mid_pad - gp)
    p1 = kR.T @ (mid_pad - kp)
    R1 = kR.T @ gR
    p.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, p0)))
    p.GetAttribute("physics:localRot0").Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    p.GetAttribute("physics:localPos1").Set(Gf.Vec3f(*map(float, p1)))
    w, x, y, z = _quat_wxyz(R1)
    p.GetAttribute("physics:localRot1").Set(Gf.Quatf(w, x, y, z))
    p.GetAttribute("physics:jointEnabled").Set(True)
    return path, (p0, p1, R1)


def _use_slot(name, T0, T1):
    """Set a pre-authored structural joint's frame so body1 stays where it is relative to body0, and enable it."""
    from pxr import Gf

    st = _stage_or_none()
    path = f"{_ST['root']}/wc_joints/{name}"
    p = st.GetPrimAtPath(path)
    rel = np.linalg.inv(T0) @ T1 if T0 is not None else T1
    p.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, rel[:3, 3])))
    w, x, y, z = _quat_wxyz(rel[:3, :3])
    p.GetAttribute("physics:localRot0").Set(Gf.Quatf(w, x, y, z))
    p.GetAttribute("physics:jointEnabled").Set(True)
    return path


def _pose_w(name):
    base = _ST["env"].unwrapped
    p = base.arena_world.get_pose_w(name)[0].detach().cpu().numpy().astype(np.float64)
    T = np.eye(4)
    T[:3, :3] = _R(p[3:])
    T[:3, 3] = p[:3]
    return T


def _gripper_T(rob):
    """Gripper USD link pose in world coordinates."""
    from yam_arena.runtime import link_poses

    return link_poses(rob)["gripper"].copy()


def gripper_pose(side: str) -> tuple[np.ndarray, np.ndarray]:
    """(pos, R) of ``left`` or ``right`` gripper link in station frame."""
    base = _ST["env"].unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    G = _gripper_T(base.scene.articulations[f"{side}_robot"])
    return G[:3, 3] - origin, G[:3, :3]


def _hacks(env) -> None:
    if _ST.get("root") is None:
        return
    from yam_arena.constants import PAD_TIP_PAST_TCP
    from yam_arena.runtime import contact_forces

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    gp = _gripper_paths()
    st = _stage_or_none()
    for i, side in enumerate(("left", "right")):
        rob = base.scene.articulations[f"{side}_robot"]
        names = list(rob.joint_names)
        tgt = getattr(rob.data.joint_pos_target, "torch", rob.data.joint_pos_target)[0].cpu().numpy()
        qcur = getattr(rob.data.joint_pos, "torch", rob.data.joint_pos)[0].cpu().numpy()
        j7, j8 = names.index("joint7"), names.index("joint8")
        cmd = abs(float(tgt[j7])) + abs(float(tgt[j8]))
        actual_open = abs(float(qcur[j7])) + abs(float(qcur[j8]))
        G = _gripper_T(rob)
        jaw_ax = G[:3, 0]
        z_ax = G[:3, 2]
        tcp = G[:3, :3] @ np.array([0.0, 0.0, 0.134]) + G[:3, 3]
        tip_l = tcp - 0.5 * actual_open * jaw_ax + 0.5 * PAD_TIP_PAST_TCP * z_ax
        tip_r = tcp + 0.5 * actual_open * jaw_ax + 0.5 * PAD_TIP_PAST_TCP * z_ax
        held = _H["held"].get(i)
        _H["k"] = _H.get("k", 0) + 1
        if actual_open >= 0.048:
            _H.setdefault("was_open", {})[i] = 25
        elif _H.get("was_open", {}).get(i, 0) > 0 and cmd <= 0.004 and actual_open <= 0.004:
            _H["was_open"][i] = 0
        if held is not None:
            open_thr = GRASP_OPEN_EYE if held.startswith("eye_") else GRASP_OPEN_CPL
            if cmd >= open_thr:
                st.GetPrimAtPath(_H["welds"].pop(i)).GetAttribute("physics:jointEnabled").Set(False)
                _H["held"].pop(i)
                _H.get("anchors", {}).pop(i, None)
                if held.startswith("eye_"):
                    _collar_target(held[-1], False)
                if held == EYE_OBJ["h"].name:
                    _H["recinch"] = 18  # re-grip the hose once the collar spring closes onto it
                print(f"[wc] {side} hand let go of {held} (slips={_H.get('slips', 0)})", flush=True)
            elif i in _H.get("anchors", {}):
                p0, p1, R1 = _H["anchors"][i]
                T1_cur = _pose_w(held)
                gp_c, gR_c = G[:3, 3], G[:3, :3]
                kp_c, kR_c = T1_cur[:3, 3], T1_cur[:3, :3]
                pg_c = gp_c + gR_c @ p0
                pb_c = kp_c + kR_c @ p1
                dp = pb_c - pg_c
                f_lin = GRIP_K * float(np.linalg.norm(dp))
                rv = _rotvec(gR_c.T @ kR_c @ R1)
                t_tw = GRIP_KR * abs(float(rv[0]))
                t_tip = GRIP_KR * float(math.hypot(rv[1], rv[2]))
                if f_lin > GRIP_HOLD_N or t_tw > GRIP_TWIST_NM or t_tip > GRIP_TIP_NM:
                    from pxr import Gf

                    dp_c = dp * (GRIP_HOLD_N / f_lin) if f_lin > GRIP_HOLD_N else dp
                    rv_c = rv.copy()
                    max_rx = GRIP_TWIST_NM / GRIP_KR
                    rv_c[0] = float(np.clip(rv_c[0], -max_rx, max_rx))
                    tip_mag = float(math.hypot(rv_c[1], rv_c[2]))
                    max_rt = GRIP_TIP_NM / GRIP_KR
                    if tip_mag > max_rt:
                        rv_c[1] *= max_rt / tip_mag
                        rv_c[2] *= max_rt / tip_mag
                    p1n = kR_c.T @ (pg_c + dp_c - kp_c)
                    R1n = kR_c.T @ gR_c @ _exp_rotvec(rv_c)
                    jp_c = st.GetPrimAtPath(_H["welds"][i])
                    if jp_c.IsValid():
                        jp_c.GetAttribute("physics:localPos1").Set(Gf.Vec3f(*map(float, p1n)))
                        w_, x_, y_, z_ = _quat_wxyz(R1n)
                        jp_c.GetAttribute("physics:localRot1").Set(Gf.Quatf(w_, x_, y_, z_))
                    _H["anchors"][i] = (p0, p1n, R1n)
                    _H["slips"] = _H.get("slips", 0) + 1
            continue
        if gp[i] is None or _H.get("was_open", {}).get(i, 0) <= 0:
            continue
        # Two-pad contact check: BOTH finger pads must register positive PhysX normal contact force
        fl = fr = 0.0
        try:
            skey = f"{side}_contacts"
            if skey in base.scene.sensors:
                cf = contact_forces(base.scene.sensors[skey])
                fl = float(cf.get("tip_left", 0.0))
                fr = float(cf.get("tip_right", 0.0))
        except Exception:  # noqa: BLE001
            pass
        if fl < PAD_CONTACT_MIN_N or fr < PAD_CONTACT_MIN_N:
            continue
        picked = None
        # Candidate 1: 40 mm cylindrical hose coupling body (two-pad contact + opposite-side straddle + jaw close)
        if _H["lock"] is None and COUPLER.name not in _H["held"].values() and cmd <= CPL_CMD_CLOSE and actual_open <= 0.047:
            cT = _pose_w(COUPLER.name)
            s_ax = float(np.clip((tcp - cT[:3, 3]) @ cT[:3, 1], -HH_L / 2, HH_L / 2))
            pt_c = cT[:3, 3] + s_ax * cT[:3, 1]
            if float(np.linalg.norm(tcp - pt_c)) <= 0.028 and float(np.dot(tip_l - pt_c, jaw_ax) * np.dot(tip_r - pt_c, jaw_ax)) < 0.0:
                picked = (COUPLER.name, cT, float(np.linalg.norm(tcp - pt_c)))
        # Candidate 2: whip-check cuff + ferrule (two-pad contact + opposite-side straddle + collar squeezed open)
        if picked is None and cmd <= EYE_CMD_CLOSE and actual_open <= 0.048:
            for k in EYES:
                ename = EYE_OBJ[k].name
                if ename in _H["held"].values():
                    continue
                eT = _pose_w(ename)
                cT_ = _pose_w(COL_OBJ[k].name)
                xi = float(eT[:3, 0] @ (cT_[:3, 3] - eT[:3, 3])) - COL_LEN / 2
                if xi < EYE_SQUEEZE_XI:
                    continue
                ft = eT[:3, :3] @ np.array([FER_X, 0.0, 0.0]) + eT[:3, 3]
                ct = cT_[:3, :3] @ np.array([-COL_LEN / 2 + COL_CUFF, 0.0, 0.0]) + cT_[:3, 3]
                mid = 0.5 * (ft + ct)
                if float(np.linalg.norm(tcp - mid)) <= 0.036 and float(np.dot(tip_l - mid, jaw_ax) * np.dot(tip_r - mid, jaw_ax)) < 0.0:
                    picked = (ename, eT, float(np.linalg.norm(tcp - mid)))
                    break
        if picked is None:
            continue
        name, T1, d = picked
        _H["welds"][i], _H.setdefault("anchors", {})[i] = _engage_grip(side, name, G, T1, 0.5 * (tip_l + tip_r))
        _H["held"][i] = name
        _H.setdefault("was_open", {})[i] = 0
        if name.startswith("eye_"):
            _collar_target(name[-1], True)  # hold already-squeezed collar flush while jaws stay closed
        if name == EYE_OBJ["h"].name:
            for cp_path in {_H.get("cinch"), _ST.get("cinch0")}:
                if cp_path:
                    p_ = st.GetPrimAtPath(cp_path)
                    if p_.IsValid():
                        p_.GetAttribute("physics:jointEnabled").Set(False)
            _H["cinch"] = None
            print("[wc] hose loop uncinched for sliding", flush=True)
        print(f"[wc] {side} hand holds {name} on two-pad contact (F_L={fl:.2f}N F_R={fr:.2f}N ap={actual_open*1000:.1f}mm d={d:.3f})", flush=True)
    if _H.get("recinch"):
        _H["recinch"] -= 1
        if _H["recinch"] <= 14 and EYE_OBJ["h"].name not in _H["held"].values():
            eT = _pose_w(EYE_OBJ["h"].name)
            cT_ = _pose_w(COL_OBJ["h"].name)
            xi = float(eT[:3, 0] @ (cT_[:3, 3] - eT[:3, 3])) - COL_LEN / 2
            # Collar must physically stop against the 29 mm hose inside the ring (an empty loop closes to -0.020 m)
            if COL_XI_CLOSED + 0.002 <= xi <= CINCH_XI:
                from yam_arena.runtime import link_poses

                art = base.scene.articulations[HOSE.name]
                lps = link_poses(art)
                seg_Ts = [lps[f"seg_{k_}"] for k_ in range(HOSE.segments)]
                d = np.array([float(np.linalg.norm(T_[:3, 3] - eT[:3, 3])) for T_ in seg_Ts])
                j = int(np.argmin(d))
                v_j = eT[:3, 3] - seg_Ts[j][:3, 3]
                s_j = float(np.clip(v_j @ seg_Ts[j][:3, 0], -HOSE.seg_len, HOSE.seg_len))
                rad_j = float(np.linalg.norm(v_j - s_j * seg_Ts[j][:3, 0]))
                if j >= 2 and rad_j <= EYE_RI - HOSE_R + 0.005:
                    _H["cinch"] = _use_slot(f"cinch_{j}", eT, seg_Ts[j])
                    _H["recinch"] = 0
                    print(f"[wc] hose loop cinched to seg_{j} (xi {xi:.3f}, rad {rad_j:.3f})", flush=True)
    # coupling: hooks must first pass axially through the lip gaps near 0 deg, then rotate behind LIP_ARCS (32..62 deg)
    if _H["lock"] is None:
        cT = _pose_w(COUPLER.name)
        R = cT[:3, :3]
        face = cT[:3, 3] - origin + R[:, 1] * (HH_L / 2)
        ax_ok = float(R[:, 1] @ np.array([0.0, 1.0, 0.0])) >= math.cos(math.radians(8))
        near = abs(face[1] - FACE_Y) <= SEAT_TOL and math.hypot(face[0] - LX, face[2] - LZ) <= 0.0048
        ang = math.degrees(math.atan2(float(R[2, 0]), float(R[0, 0])))
        if abs(face[1] - FACE_Y) > 0.012 or not ax_ok:
            _H["inserted"] = False
        elif near and abs(ang) <= 14.0:
            _H["inserted"] = True
        if ax_ok and abs(face[1] - FACE_Y) <= 0.005 and math.hypot(face[0] - LX, face[2] - LZ) <= 0.005 and _H.get("inserted") and LOCK_ENGAGE_MIN <= ang <= LOCK_MAX:
            _H["lock"] = _use_slot("lock", _pose_w(MAN.name), cT)
            for i, nm in list(_H["held"].items()):  # hooks engaged behind lips hold the coupling in situ
                if nm == COUPLER.name:
                    st.GetPrimAtPath(_H["welds"].pop(i)).GetAttribute("physics:jointEnabled").Set(False)
                    _H["held"].pop(i)
                    _H.get("anchors", {}).pop(i, None)
            print(f"[wc] coupling locked in situ at {ang:.1f} deg (face {np.round(face, 4)})", flush=True)


# -- live hooks --------------------------------------------------------------------------------------------------------
_EP: dict = {}


def _reset_episode() -> None:
    if "_H" in globals():
        _reset_hacks()
    _EP.clear()
    _EP.update(opened=False, violation="", open_state=None, nan=False, hist=[], t=0)


_reset_episode()


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline import)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_wc_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ST["env"] = self._env
            try:
                _hacks(self._env)
            except Exception as e:  # noqa: BLE001
                if not _ST.get("hack_err"):
                    _ST["hack_err"] = repr(e)
                    print(f"[wc] hack error: {e!r}", flush=True)
            return base(self, *a, **k)

        _task_step._wc_hooked = True
        cls._task_step = _task_step
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            out = base_reset(self, *a, **k)
            _ST["env"] = self._env
            return out

        reset._task_reset_hooks = []
        cls.reset = reset
    if _reset_episode not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_episode)


_install_hooks()


def grippers() -> list[np.ndarray]:
    env = _ST["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- geometry ----------------------------------------------------------------------------------------------------------
def _R(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _frame(o, name=None):
    """Body frame origin and rotation. ObjectState.pos is the bounding-box centre, so read the pose from the world."""
    env = _ST["env"]
    if env is not None and name is not None:
        base = env.unwrapped
        p = base.arena_world.get_pose_w(name)[0].detach().cpu().numpy().astype(np.float64)
        return p[:3] - base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64), _R(p[3:])
    return np.asarray(o.pos, float), _R(o.quat_xyzw)


def eye_state(objects, k) -> dict:
    p, R = _frame(objects[EYE_OBJ[k].name], EYE_OBJ[k].name)
    cp, _ = _frame(objects[COL_OBJ[k].name], COL_OBJ[k].name)
    xi = float(R[:, 0] @ (cp - p)) - COL_LEN / 2  # collar inner face, eye frame x
    return {"c": p, "R": R, "axis": R[:, 1], "xi": xi,
            "cuff": cp + R @ np.array([-COL_LEN / 2 + COL_CUFF, 0.0, 0.0]),
            "ferrule": p + R @ np.array([FER_X, 0.0, 0.0]),
            "tabs": [p + R @ np.array([FER_X, 0.0, 0.0]), cp + R @ np.array([-COL_LEN / 2 + COL_CUFF, 0.0, 0.0])]}


def _line_through(e, a, d, r_line) -> tuple[bool, float]:
    """Is a line (point a, unit dir d, radius r_line) inside eye e? Returns (inside, along-line coordinate of the eye)."""
    v = e["c"] - a
    s = float(v @ d)
    radial = float(np.linalg.norm(v - s * d))
    ok = radial <= EYE_RI - r_line + 0.005 and abs(float(e["axis"] @ d)) >= math.cos(math.radians(60))
    return ok, s


def cinched(e, k=None) -> bool:
    """Collar let go onto a line: not held by a gripper, and its inner face is closed onto the line."""
    if k is not None and EYE_OBJ[k].name in _H.get("held", {}).values():
        return False
    return e["xi"] <= CINCH_XI


def m_eye_on(objects) -> bool:
    e = eye_state(objects, "m")
    ok, s = _line_through(e, np.array([LX, 0.0, LZ]), np.array([0.0, 1.0, 0.0]), NIP_R)
    return bool(ok and FIX_Y1 <= s <= NIP_Y1 - 0.002 and cinched(e, "m"))


def coupler_pose(objects):
    p, R = _frame(objects[COUPLER.name], COUPLER.name)
    return p, R


def h_eye_on(objects) -> bool:
    e = eye_state(objects, "h")
    h = objects[HOSE.name]
    if not np.isfinite(h.points).all():
        return False
    pts = h.points
    i = int(np.argmin(np.linalg.norm(pts - e["c"], axis=1)))
    if i < 2:  # on the coupling or past it
        return False
    j0, j1 = max(0, i - 1), min(len(pts) - 1, i + 1)
    d = pts[j1] - pts[j0]
    d /= max(1e-9, float(np.linalg.norm(d)))
    ok, _ = _line_through(e, pts[i], d, HOSE_R)
    return bool(ok and cinched(e, "h"))


def coupling_angle(objects) -> float:
    """Rotation (deg) of the hose head about the line axis from its insert orientation (hooks level)."""
    _, R = coupler_pose(objects)
    hx = R[:, 0]  # hook 0 direction
    return math.degrees(math.atan2(float(hx[2]), float(hx[0])))


def locked(objects) -> bool:
    if COUPLER.name in _H.get("held", {}).values():
        return False
    p, R = coupler_pose(objects)
    face = p + R[:, 1] * (HH_L / 2)
    axis_ok = float(R[:, 1] @ np.array([0.0, 1.0, 0.0])) >= math.cos(math.radians(8))
    seat = abs(face[1] - FACE_Y) <= 0.005 and math.hypot(face[0] - LX, face[2] - LZ) <= 0.005
    return bool(axis_ok and seat and LOCK_MIN <= coupling_angle(objects) <= LOCK_MAX)


def slack(objects) -> float:
    w = objects[WHIP.name]
    pl = w.polyline()
    if not np.isfinite(pl).all():
        return 9.0
    length = float(np.linalg.norm(np.diff(pl, axis=0), axis=1).sum())
    return length - float(np.linalg.norm(pl[-1] - pl[0]))


def valve_angle(objects) -> float:
    """Lever turn (deg) from closed (pointing at the robot, -x) towards open (pointing +y)."""
    _, R = _frame(objects[LEVER.name], LEVER.name)
    return abs(math.degrees(math.atan2(float(R[1, 0]), float(R[0, 0]))))


def _body_points(objects):
    pts = [
        coupler_pose(objects)[0],
        np.array([LX, (FIX_Y0 + FIX_Y1) / 2, LZ]),
        np.array([LX, (NIP_Y0 + NIP_Y1) / 2, LZ]),
        np.asarray(objects[LEVER.name].pos, float),
        np.asarray(MAN.pos, float),
        np.asarray(VALVE_BODY.pos, float),
        np.asarray(PEG.pos, float),
    ]
    for k in EYES:
        e = eye_state(objects, k)
        pts += [e["c"], *e["tabs"]]
    _, R = _frame(objects[LEVER.name], LEVER.name)
    pts.append(np.asarray(objects[LEVER.name].pos, float) - R[:, 0] * (LEVER_L / 2))
    pts.extend(np.asarray(objects[HOSE.name].points, float))
    pts.extend(np.asarray(objects[WHIP.name].points, float))
    return np.asarray(pts, float)


def released(objects, dist=0.05) -> bool:
    if _H.get("held"):
        return False
    g = grippers()
    if len(g) < 2:
        return False
    P = _body_points(objects)
    return all(float(np.linalg.norm(P - x, axis=1).min()) >= dist for x in g)


def _finite(objects) -> bool:
    for o in objects.values():
        if not np.isfinite(np.asarray(o.pos, float)).all():
            return False
    return np.isfinite(objects[HOSE.name].points).all() and np.isfinite(objects[WHIP.name].points).all()


def _track(objects) -> None:
    if not _finite(objects):
        _EP["nan"] = True
        return
    _EP["t"] += 1
    ang = abs(valve_angle(objects))
    if ang > VALVE_TRIP_DEG:
        sl = slack(objects)
        st = dict(m=m_eye_on(objects), h=h_eye_on(objects), lock=locked(objects), slack=round(sl, 4))
        if not _EP["opened"]:
            _EP["opened"] = True
            _EP["open_state"] = st
            print(f"[wc] valve opened: {st}", flush=True)
        if not _EP["violation"] and not (st["m"] and st["h"] and st["lock"] and sl <= 0.03):
            _EP["violation"] = f"unsafe while air open: {st}"
            print(f"[wc] VIOLATION: {_EP['violation']}", flush=True)
    g = grippers()
    if g:
        _EP["hist"].append(np.concatenate(g))
        _EP["hist"] = _EP["hist"][-12:]


def parked(objects) -> bool:
    """Both TCPs >= 10 cm from every task part and slower than 2 cm/s over the last 0.5 s."""
    if len(_EP["hist"]) < 11:
        return False
    if not released(objects, 0.10):
        return False
    h = np.asarray(_EP["hist"][-11:])
    v = np.linalg.norm(np.diff(h.reshape(len(h), 2, 3), axis=0), axis=2).max() * 20.0
    return bool(v < 0.02)


def score(objects) -> float:
    _track(objects)
    if _EP["nan"] or _EP["violation"]:
        return 0.0
    m, h, lk, sl = m_eye_on(objects), h_eye_on(objects), locked(objects), slack(objects)
    if not (m and h):
        return 0.0
    if not released(objects, 0.05):
        return 0.0
    if lk and sl <= 0.03 and abs(valve_angle(objects)) >= VALVE_OPEN_DEG and parked(objects):
        return 1.0
    if lk and sl <= 0.03:
        return 0.75
    return 0.5 if lk else 0.25


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    return 1.0 - score(objects)


def progress(objects) -> dict:
    return {"m_eye_on": int(m_eye_on(objects)), "h_eye_on": int(h_eye_on(objects)), "locked": int(locked(objects)),
            "slack_ok": int(slack(objects) <= 0.03), "valve_open": int(abs(valve_angle(objects)) >= VALVE_OPEN_DEG)}


VARIATION = {
    "cable:air_hose": {"dx": 0.01, "dy": 0.01, "yaw": 0.05},
    "obj:hose_coupler": {"dx": 0.01, "dy": 0.01, "yaw": 0.05},
}
