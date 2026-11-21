"""Hang up a stick welder's electrode holder: welder off, rod stub into the stub can, holder in its insulated hanger.

Scene (metres from the arm-base midpoint, x forward, y left; heights above the steel welding table). The electrode
holder (Hobart 770027, 300 A spring-jaw stinger, 37 cm, 0.68 kg) lies on its side, jaws towards +y: jaw pivot pin at
(0.42, 0.12), insulated handle running back to y -0.19, spring lever on the robot side (-x). A 20 cm 1/8 in E7018 rod
stub (3.2 mm dia, 20 g) stands upright in the lower jaw's 90-degree groove at (0.42, 0.165), its lower end 8 mm above
the steel table (flush with the jaw underside). A 40 cm #6 rubber welding lead (11 mm OD) runs straight back along -y
from the handle end toward the mock welder (25 x 20 x 16 cm box centred at (0.58, -0.52), x 0.455-0.705, y -0.62 to
-0.42). On the welder's front face at (0.448, -0.55, 0.11 up) is the red I/O power rocker switch in its ON (0 deg)
position; beside it at y -0.59 is a battery-powered amber power indicator lamp (lit while ON, dark when OFF). A round
steel stub can (12 cm dia, 12 cm tall) stands at (0.22, 0.28). The insulated hanger — two white nylon U-saddles (6 cm
inside width) on 2.5 cm pine posts, saddle floors 0.15 up, at x 0.30, y -0.10 and -0.32 — stands to the right.

Holder mechanics: handle+lower jaw (0.55 kg, with 90-degree groove cheeks at x_h 0.045) and lever+upper jaw (0.13 kg)
turn about the pivot pin; a 2.4 N m/rad torsion spring clamps the rod with ~19 N. Squeezing the lever ~11 deg onto the
handle opens the jaws ~7 mm so the stub can be drawn straight up out of the groove. Rocker: bistable hinge, 0 deg = ON,
pushed in past 12 deg = OFF.

Scoring: see score().
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "I'm going on break. Hang up the stinger."

BRASS = (0.78, 0.62, 0.30)
HANDLE_C = (0.10, 0.10, 0.11)
LEVER_C = (0.12, 0.12, 0.13)
ROD_C = (0.45, 0.40, 0.33)
WELDER_C = (0.12, 0.30, 0.62)
WOOD = (0.55, 0.40, 0.24)
NYLON = (0.92, 0.92, 0.88)
CAN_C = (0.55, 0.56, 0.58)
BLACK = (0.04, 0.04, 0.04)
SWITCH_C = (0.80, 0.08, 0.06)

# -- electrode holder (holder frame: origin on the pivot pin, +x towards the jaw tips, +y = lever side, z up) --------
YAW_H = math.pi / 2  # holder x along world +y; holder +y (lever side) is world -x
PIV = np.array([0.42, 0.12, TOP + 0.016])  # pivot pin, lying on its side (handle 32 mm thick)
HZ = 0.016  # half thickness of the handle (vertical when lying on its side)
JAW_X0, JAW_X1, JAW_T, JAW_H = 0.000, 0.060, 0.012, 0.020  # -0.310..+0.060 = 37.0 cm overall length
HANDLE_X0, HANDLE_X1, HANDLE_Y0, HANDLE_Y1 = -0.310, -0.020, -0.034, -0.002
LEVER_A, LEVER_B, LEVER_T = (-0.010, 0.005), (-0.200, 0.037), 0.010  # lever centreline ends, thickness
ROD_X = 0.045  # rod in the 90-degree groove, this far in front of the pin
ROD_R, ROD_L = 0.0016, 0.20
ROD_Z0 = 0.008  # lower rod end height above the steel table at start (flush with jaw underside)
SQUEEZE_X = -0.140  # gripper squeezes lever onto handle here
OPEN_MAX_DEG = 14.0
SPRING_K = 2.4  # N m/rad
SPRING_TARGET_DEG = -16.0  # spring target beyond shut: 0.85 N m at 4.3 deg (rod in) -> ~19 N pinch at the groove
A_MASS, B_MASS = 0.55, 0.13
ROD_MASS = 0.02


def hw(p) -> np.ndarray:
    """Holder-frame point -> world, at the starting pose."""
    c, s = math.cos(YAW_H), math.sin(YAW_H)
    return PIV + np.array([c * p[0] - s * p[1], s * p[0] + c * p[1], p[2] if len(p) > 2 else 0.0])


HOLDER_A = Box("holder_handle", pos=tuple(PIV.tolist()), size=(0.006, 0.006, 0.006), color=HANDLE_C, mass=A_MASS,
               yaw=YAW_H)
JAW0_DEG = math.degrees(math.atan2(2 * ROD_R + 0.0002, ROD_X))  # upper jaw resting on the rod
HOLDER_B = Box("holder_lever", pos=tuple(PIV.tolist()), size=(0.006, 0.006, 0.006), color=LEVER_C, mass=B_MASS,
               yaw=YAW_H + math.radians(JAW0_DEG))
_rod0 = hw((ROD_X, ROD_R + 0.0002))
ROD = Cylinder("rod_stub", pos=(float(_rod0[0]), float(_rod0[1]), TOP + ROD_Z0 + ROD_L / 2), radius=ROD_R, height=ROD_L,
               color=ROD_C, mass=ROD_MASS)

# -- welder, lead, rocker ------------------------------------------------------------------------------------------
WELDER = Box("welder", pos=(0.58, -0.52, TOP + 0.08), size=(0.25, 0.20, 0.16), color=WELDER_C)
ROCKER_P = (0.455 - 0.007, -0.55, TOP + 0.11)
ROCKER = Box("power_switch", pos=ROCKER_P, size=(0.012, 0.030, 0.044), color=SWITCH_C, mass=0.01)
ROCKER_OFF_DEG = 12.0
_rear = hw((HANDLE_X0, (HANDLE_Y0 + HANDLE_Y1) / 2))
LEAD_L = 0.40
LEAD = cable("welding_lead", start=(float(_rear[0]), float(_rear[1]) - LEAD_L - 0.022, TOP + 0.0055), yaw=math.pi / 2,
             length=LEAD_L, radius=0.0055, segment=0.03, anchor_start=True,
             end_plug=Plug(size=(0.016, 0.016, 0.016), mass=0.01, color=BLACK), cable_mass=0.12 * LEAD_L,
             bend_stiffness=float(os.environ.get("ST_LEAD_K", "0.004")), bend_damping=0.002, color=BLACK)

# -- stub can (static cylindrical base + 16 circular staves added in the spawn hook) --------------------------------
CAN_XY, CAN_R, CAN_H = (0.22, 0.28), 0.06, 0.12
CAN = Cylinder("stub_can", pos=(CAN_XY[0], CAN_XY[1], TOP + 0.002), radius=CAN_R, height=0.004, color=CAN_C)

# -- insulated hanger: two nylon U-saddles on wooden posts ---------------------------------------------------------
RACK_X = 0.30
SADDLE_Y = (-0.10, -0.32)
SADDLE_Z = TOP + 0.15  # saddle floor (top face)
SADDLE_W = 0.060  # inside width
POSTS = [Box(f"hanger_post_{i}", pos=(RACK_X, y, TOP + (SADDLE_Z - TOP - 0.006) / 2), size=(0.025, 0.025, SADDLE_Z - TOP - 0.006),
             color=WOOD) for i, y in enumerate(SADDLE_Y)]
SADDLES = []
for i, y in enumerate(SADDLE_Y):
    SADDLES.append(Box(f"saddle_{i}_floor", pos=(RACK_X, y, SADDLE_Z - 0.003), size=(SADDLE_W + 0.016, 0.030, 0.006), color=NYLON))
    for s in (-1, 1):
        SADDLES.append(Box(f"saddle_{i}_wall{'ab'[(s + 1) // 2]}", pos=(RACK_X + s * (SADDLE_W / 2 + 0.004), y, SADDLE_Z + 0.015),
                           size=(0.008, 0.030, 0.030), color=NYLON))

OBJECTS = [WELDER, CAN, *POSTS, *SADDLES, ROCKER, HOLDER_A, HOLDER_B, ROD, LEAD]

TWIN_EXTRA = [
    {"name": "holder_pivot", "kind": "joint", "real": "electrode holder jaw pin with torsion spring",
     "note": f"revolute 0-{OPEN_MAX_DEG:.0f} deg, {SPRING_K} N m/rad toward shut"},
    {"name": "holder_parts", "kind": "body", "size_mm": [370, 70, 32], "colour": "black/brass",
     "note": "handle+lower jaw 0.55 kg (with 90 deg groove cheeks), lever+upper jaw 0.13 kg (Hobart 770027)"},
    {"name": "power_switch_pivot", "kind": "joint", "real": "20 A I/O panel rocker switch", "note": "0 deg ON, >=12 deg OFF"},
    {"name": "power_lamp", "kind": "decal", "size_mm": [16, 18, 22], "colour": "amber", "pose": [0.455, -0.59, 0.15],
     "note": "12 V amber indicator lamp wired through the rocker switch to a 9 V battery inside the mock welder"},
    {"name": "stub_can_wall", "kind": "body", "size_mm": [120, 120, 120], "colour": "steel", "note": "open 12 cm steel can"},
]

_ST: dict = {"env": None}


def holder_parts():
    """(name, body 'a'|'b', centre (holder frame), size, yaw in holder frame, colour key)."""
    lc = ((LEVER_A[0] + LEVER_B[0]) / 2, (LEVER_A[1] + LEVER_B[1]) / 2)
    ll = math.hypot(LEVER_B[0] - LEVER_A[0], LEVER_B[1] - LEVER_A[1])
    ly = math.atan2(LEVER_B[1] - LEVER_A[1], LEVER_B[0] - LEVER_A[0])
    return [
        ("handle", "a", ((HANDLE_X0 + HANDLE_X1) / 2, (HANDLE_Y0 + HANDLE_Y1) / 2, 0.0),
         (HANDLE_X1 - HANDLE_X0, HANDLE_Y1 - HANDLE_Y0, 2 * HZ), 0.0, "black"),
        ("neck", "a", (-0.012, -0.010, 0.0), (0.024, 0.020, JAW_H), 0.0, "brass"),
        ("jaw", "a", ((JAW_X0 + JAW_X1) / 2, -JAW_T / 2, 0.0), (JAW_X1 - JAW_X0, JAW_T, JAW_H), 0.0, "brass"),
        # 90-degree electrode groove cheeks flanking ROD_X on the lower jaw (4.8 mm slot, 2.0 mm deep)
        ("groove_f", "a", (ROD_X + 0.0034, 0.0010, 0.0), (0.0020, 0.0020, JAW_H), 0.0, "brass"),
        ("groove_r", "a", (ROD_X - 0.0034, 0.0010, 0.0), (0.0020, 0.0020, JAW_H), 0.0, "brass"),
        ("jaw", "b", ((JAW_X0 + JAW_X1) / 2, JAW_T / 2, 0.0), (JAW_X1 - JAW_X0, JAW_T, JAW_H), 0.0, "brass"),
        ("lever", "b", (lc[0], lc[1], 0.0), (ll, LEVER_T, 0.024), ly, "black"),
    ]


# -- spawn hook --------------------------------------------------------------------------------------------------------
def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_st_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != LEAD.name:
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        looks = f"{root}/STLooks"
        mats = {}
        for key, col, met in (("black", HANDLE_C, 0.0), ("brass", BRASS, 0.6), ("can", CAN_C, 0.5)):
            p = f"{looks}/{key}"
            m = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(col), roughness=0.45, metallic=met)
            m.func(p, m)
            mats[key] = p
        grip_phys = f"{looks}/grip_phys"
        pg = sim_utils.RigidBodyMaterialCfg(static_friction=1.4, dynamic_friction=1.2)
        pg.func(grip_phys, pg)
        jaw_phys = f"{looks}/jaw_phys"
        pj = sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=0.9)
        pj.func(jaw_phys, pj)
        can_phys = f"{looks}/can_phys"
        pc = sim_utils.RigidBodyMaterialCfg(static_friction=0.6, dynamic_friction=0.5)
        pc.func(can_phys, pc)

        def cube(path, c, s, yaw, look, phys, patch=0.0):
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            g.AddOrientOp().Set(Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, s)))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            if patch:
                cp.CreateTorsionalPatchRadiusAttr(patch)
                cp.CreateMinTorsionalPatchRadiusAttr(patch / 2)
            bind_physics_material(str(g.GetPath()), phys, stage=stage)
            bind_visual_material(str(g.GetPath()), look, stage=stage)

        def no_collide(path):
            p = stage.GetPrimAtPath(path)
            for child in [p, *p.GetChildren()]:
                if child.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)

        def filt(p0, p1):
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(p0)).CreateFilteredPairsRel().AddTarget(Sdf.Path(p1))

        def xf(path):
            return UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)

        pa, pb = f"{root}/{HOLDER_A.name}", f"{root}/{HOLDER_B.name}"
        no_collide(pa)
        no_collide(pb)
        for name, body, c, s, yw, look in holder_parts():
            cube(f"{(pa if body == 'a' else pb)}/{name}", c, s, yw, mats[look],
                 jaw_phys if look == "brass" else grip_phys, patch=0.0 if look == "brass" else 0.030)
        filt(pa, pb)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{pa}/pivot_pin")
        j.CreateBody0Rel().SetTargets([Sdf.Path(pa)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(pb)])
        for at in (j.CreateLocalPos0Attr, j.CreateLocalPos1Attr):
            at(Gf.Vec3f(0, 0, 0))
        for at in (j.CreateLocalRot0Attr, j.CreateLocalRot1Attr):
            at(Gf.Quatf(1, 0, 0, 0))
        j.CreateAxisAttr("Z")
        j.CreateLowerLimitAttr(0.0)
        j.CreateUpperLimitAttr(OPEN_MAX_DEG)
        j.CreateExcludeFromArticulationAttr(True)
        drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(SPRING_K * math.pi / 180)  # USD angular gains are per degree
        drv.CreateDampingAttr(0.01 * math.pi / 180)
        drv.CreateTargetPositionAttr(SPRING_TARGET_DEG)
        # lead welded to the handle end at its authored pose
        plug = f"{root}/{LEAD.name}/end_plug"
        rel = xf(plug) * xf(pa).GetInverse()
        t, q = rel.ExtractTranslation(), rel.ExtractRotationQuat()
        w = UsdPhysics.FixedJoint.Define(stage, f"{pa}/lead_weld")
        w.CreateBody0Rel().SetTargets([Sdf.Path(pa)])
        w.CreateBody1Rel().SetTargets([Sdf.Path(plug)])
        w.CreateLocalPos0Attr(Gf.Vec3f(*map(float, t)))
        w.CreateLocalRot0Attr(Gf.Quatf(float(q.GetReal()), *map(float, q.GetImaginary())))
        w.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        w.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        w.CreateExcludeFromArticulationAttr(True)
        filt(pa, plug)
        filt(pb, plug)
        for i in (LEAD.segments - 1, LEAD.segments - 2):
            filt(pa, f"{root}/{LEAD.name}/seg_{i}")
            filt(pb, f"{root}/{LEAD.name}/seg_{i}")
        # rocker on its pin (axis y through its centre), bistable detent (0 deg ON, 18 deg OFF)
        rk, wd = f"{root}/{ROCKER.name}", f"{root}/{WELDER.name}"
        filt(rk, wd)
        rj = UsdPhysics.RevoluteJoint.Define(stage, f"{rk}/rocker_pin")
        rj.CreateBody0Rel().SetTargets([Sdf.Path(wd)])
        rj.CreateBody1Rel().SetTargets([Sdf.Path(rk)])
        rj.CreateLocalPos0Attr(Gf.Vec3f(*map(float, np.asarray(ROCKER_P) - np.asarray(WELDER.pos))))
        rj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        rj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        rj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        rj.CreateAxisAttr("Y")
        rj.CreateLowerLimitAttr(0.0)
        rj.CreateUpperLimitAttr(20.0)
        rj.CreateExcludeFromArticulationAttr(True)
        rdrv = UsdPhysics.DriveAPI.Apply(rj.GetPrim(), "angular")
        rdrv.CreateTypeAttr("force")
        rdrv.CreateStiffnessAttr(0.06 * math.pi / 180)
        rdrv.CreateDampingAttr(0.02 * math.pi / 180)
        rdrv.CreateTargetPositionAttr(0.0)
        _ST["rocker_drv"] = str(rj.GetPath())
        PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(rk)).CreateAngularDampingAttr(20.0)

        # amber power indicator lamp (visible from both top_cam and right_cam) + clear white I / O rocker markings
        def vis_cube(path, c, sz, col):
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
            g.CreateDisplayColorAttr([Gf.Vec3f(*linear_rgb(col))])
            x = UsdGeom.Xformable(g)
            x.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            x.AddScaleOp().Set(Gf.Vec3f(*map(float, sz)))
            return g

        wo = np.asarray(WELDER.pos)
        # Bezel + unlit dark-amber lens + bright-amber lit lens at front-top corner (y = -0.59)
        lx, ly, lz = 0.455 - wo[0], -0.59 - wo[1], TOP + 0.148 - wo[2]
        vis_cube(f"{wd}/lamp_base", (lx + 0.002, ly, lz), (0.016, 0.024, 0.028), (0.14, 0.14, 0.15))
        vis_cube(f"{wd}/power_lamp_off", (lx - 0.002, ly, lz + 0.002), (0.014, 0.016, 0.022), (0.20, 0.14, 0.04))
        lamp = vis_cube(f"{wd}/power_lamp", (lx - 0.003, ly, lz + 0.003), (0.016, 0.018, 0.024), (1.0, 0.78, 0.08))
        _ST["lamp"] = str(lamp.GetPath())
        # White 'I' mark on top half (ON) and hollow 'O' mark on bottom half (OFF) of the red rocker switch
        wht = (0.96, 0.96, 0.96)
        vis_cube(f"{rk}/mark_I", (-0.0065, 0.0, 0.011), (0.002, 0.0035, 0.011), wht)
        vis_cube(f"{rk}/mark_O_t", (-0.0065, 0.0, -0.0065), (0.002, 0.010, 0.0025), wht)
        vis_cube(f"{rk}/mark_O_b", (-0.0065, 0.0, -0.0165), (0.002, 0.010, 0.0025), wht)
        vis_cube(f"{rk}/mark_O_l", (-0.0065, 0.004, -0.0115), (0.002, 0.0025, 0.010), wht)
        vis_cube(f"{rk}/mark_O_r", (-0.0065, -0.004, -0.0115), (0.002, 0.0025, 0.010), wht)
        # stub can wall: 16 staves round the cylindrical base
        cn = f"{root}/{CAN.name}"
        n = 16
        for i in range(n):
            a = 2 * math.pi * i / n
            r = CAN_R - 0.0008
            cube(f"{cn}/stave{i}", (r * math.cos(a), r * math.sin(a), CAN_H / 2),
                 (0.0016, 2 * CAN_R * math.tan(math.pi / n) + 0.001, CAN_H), a, mats["can"], can_phys)
        print(f"[st] spawn: holder (pivot, spring, 90d groove), lead weld, I/O rocker+lamp, can; lead {LEAD.segments} segs", flush=True)
        return prim

    spawn._st_hook = True
    cablekit._spawn_cable = spawn


_install_spawn()


# -- live hooks --------------------------------------------------------------------------------------------------------
_EP: dict = {}


def _set_rocker_target(deg: float) -> None:
    path = _ST.get("rocker_drv")
    if not path or _ST.get("rocker_tgt") == deg:
        return
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        prim = get_current_stage().GetPrimAtPath(path)
        if prim and prim.IsValid():
            UsdPhysics.DriveAPI.Get(prim, "angular").GetTargetPositionAttr().Set(float(deg))
            _ST["rocker_tgt"] = deg
    except Exception:  # noqa: BLE001
        pass


def _reset_episode() -> None:
    _EP.clear()
    _EP.update(violation="", nan=False, hist=[], start={})
    _ST["lamp_on"] = None
    _ST["rocker_tgt"] = None
    _set_rocker_target(0.0)


_reset_episode()


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_st_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ST["env"] = self._env
            if self.task is not None and getattr(self.task.module, "__file__", None) == __file__:
                from yam_arena.scene import object_states

                _tracked(object_states(self._env, self.task.names))
            return base(self, *a, **k)

        _task_step._st_hooked = True
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


# -- dense gripper + finger collision surfaces (gripper, tip_left, tip_right from yam_arm.urdf) ------------------------
_GRIPPER_PTS: dict = {}


def _rpy_mat(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
            @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
            @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]))


def _densify_tris(V: np.ndarray, step: float = 0.0025, grid: float = 0.002) -> np.ndarray:
    """Sample all triangle faces and edges so no surface point is more than ~step/2 from a sample."""
    out = [V.reshape(-1, 3)]
    a, b, c = V[:, 0], V[:, 1], V[:, 2]
    max_e = np.maximum(np.linalg.norm(b - a, axis=1),
                       np.maximum(np.linalg.norm(c - a, axis=1), np.linalg.norm(c - b, axis=1)))
    ks = np.ceil(max_e / step).astype(int)
    for k in range(2, int(ks.max()) + 1):
        sub = V[ks == k]
        if not len(sub):
            continue
        u, w = np.meshgrid(np.linspace(0.0, 1.0, k + 1), np.linspace(0.0, 1.0, k + 1))
        m = (u + w) <= 1.0 + 1e-9
        u, w = u[m], w[m]
        pts = ((1.0 - u - w)[None, :, None] * sub[:, 0:1, :]
               + u[None, :, None] * sub[:, 1:2, :]
               + w[None, :, None] * sub[:, 2:3, :])
        out.append(pts.reshape(-1, 3))
    P = np.vstack(out)
    return np.unique(np.round(P / grid).astype(np.int32), axis=0).astype(np.float64) * grid


def _load_gripper_pts() -> dict:
    """Dense surface points of gripper, tip_left and tip_right collision meshes in each link's frame, cached."""
    if _GRIPPER_PTS:
        return _GRIPPER_PTS
    import re
    import struct

    import yam_arena

    root = pathlib.Path(yam_arena.__file__).resolve().parent / "assets" / "yam"
    urdf = (root / "yam_arm.urdf").read_text()
    pat = re.compile(r'<collision>\s*<origin xyz="([^"]+)"(?:\s+rpy="([^"]+)")?[^>]*/>\s*<geometry>\s*<mesh filename="([^"]+)"')
    for link in ("gripper", "tip_left", "tip_right"):
        block = urdf[urdf.index(f'<link name="{link}"'):]
        block = block[:block.index("</link>")]
        tris = []
        for m in pat.finditer(block):
            off = np.array([float(v) for v in m.group(1).split()])
            R = _rpy_mat(*[float(v) for v in m.group(2).split()]) if m.group(2) else np.eye(3)
            data = (root / m.group(3)).read_bytes()
            n = struct.unpack("<I", data[80:84])[0]
            tri = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
            tris.append(tri["v"].astype(np.float64) @ R.T + off)
        _GRIPPER_PTS[link] = _densify_tris(np.vstack(tris))
    return _GRIPPER_PTS


def gripper_points() -> list[np.ndarray]:
    """World surface points on both grippers (housing + both fingers of each arm)."""
    env = _ST["env"]
    if env is None:
        return []
    from yam_arena.runtime import link_poses

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    gp = _load_gripper_pts()
    out = []
    for k in ("left_robot", "right_robot"):
        lp = link_poses(base.scene[k])
        for link in ("gripper", "tip_left", "tip_right"):
            T = lp[link]
            out.append(gp[link] @ T[:3, :3].T + T[:3, 3] - origin)
    return out


finger_points = gripper_points


# -- geometry ----------------------------------------------------------------------------------------------------------
def _R(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose(objects, name):
    """Body frame origin and rotation (ObjectState.pos is the bounding-box centre, so read the pose itself)."""
    env = _ST["env"]
    o = objects[name]
    if env is not None:
        base = env.unwrapped
        p = base.arena_world.get_pose_w(name)[0].detach().cpu().numpy().astype(np.float64)
        return p[:3] - base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64), _R(p[3:])
    return np.asarray(o.pos, float), _R(o.quat_xyzw)


def jaw_angle(objects) -> float:
    _, Ra = pose(objects, HOLDER_A.name)
    _, Rb = pose(objects, HOLDER_B.name)
    rel = Ra.T @ Rb
    return math.degrees(math.atan2(rel[1, 0], rel[0, 0]))


def rocker_angle(objects) -> float:
    _, R = pose(objects, ROCKER.name)
    return math.degrees(math.atan2(R[0, 2], R[2, 2]))


def welder_off(objects) -> bool:
    return rocker_angle(objects) >= ROCKER_OFF_DEG


def rod_ends(objects):
    p, R = pose(objects, ROD.name)
    return p - R[:, 2] * ROD_L / 2, p + R[:, 2] * ROD_L / 2


def holder_local(objects, pt) -> np.ndarray:
    p, R = pose(objects, HOLDER_A.name)
    return R.T @ (np.asarray(pt) - p)


def rod_in_jaws(objects) -> bool:
    """Any part of the rod inside the jaws' clamping region (in front of the pin, between the jaw faces)."""
    a, b = rod_ends(objects)
    for t in np.linspace(0, 1, 21):
        q = holder_local(objects, a + t * (b - a))
        if JAW_X0 + 0.004 <= q[0] <= JAW_X1 + 0.002 and -0.004 <= q[1] <= 0.010 and abs(q[2]) <= JAW_H / 2 + 0.002:
            return True
    return False


def rod_in_can(objects) -> bool:
    """Rod resting inside the stub can (lower end near the can floor, midpoint within the rim)."""
    if float(np.linalg.norm(objects[ROD.name].lin_vel)) >= 0.05:
        return False
    a, b = rod_ends(objects)
    lo = a if a[2] < b[2] else b
    hi = b if a[2] < b[2] else a
    inside = lambda q: math.hypot(q[0] - CAN_XY[0], q[1] - CAN_XY[1]) <= CAN_R - 0.002  # noqa: E731
    return bool(inside(lo) and lo[2] <= TOP + 0.030 and inside((lo + hi) / 2)
                and (lo + hi)[2] / 2 <= TOP + CAN_H + ROD_L / 2)


SADDLE_PTS = (-0.055, -0.275)  # nominal holder-frame x sections aligned with the two saddles


def holder_in_hanger(objects) -> bool:
    """Any part of the holder inside either nylon U-saddle's channel."""
    p, R = pose(objects, HOLDER_A.name)
    xs = np.linspace(HANDLE_X0, JAW_X1, 36)
    line = p[None, :] + (np.column_stack([xs, np.full_like(xs, -0.010), np.zeros_like(xs)]) @ R.T)
    for y_s in SADDLE_Y:
        hit = ((np.abs(line[:, 0] - RACK_X) <= SADDLE_W / 2 + 0.008)
               & (np.abs(line[:, 1] - y_s) <= 0.025)
               & (line[:, 2] >= SADDLE_Z - 0.004)
               & (line[:, 2] <= SADDLE_Z + 0.036))
        if np.any(hit):
            return True
    return False


def holder_hung(objects) -> bool:
    """Insulated handle resting in both nylon saddles, with the holder clear of the steel table."""
    if objects[HOLDER_A.name].low[2] <= TOP + 0.05 or objects[HOLDER_B.name].low[2] <= TOP + 0.05:
        return False
    if float(np.linalg.norm(objects[HOLDER_A.name].lin_vel)) >= 0.05:
        return False
    p, R = pose(objects, HOLDER_A.name)
    hy = (HANDLE_Y0 + HANDLE_Y1) / 2
    xs = np.linspace(HANDLE_X0, HANDLE_X1, 30)
    line = p[None, :] + (np.column_stack([xs, np.full_like(xs, hy), np.zeros_like(xs)]) @ R.T)
    for y_s in SADDLE_Y:
        in_saddle = ((np.abs(line[:, 0] - RACK_X) <= SADDLE_W / 2)
                     & (np.abs(line[:, 1] - y_s) <= 0.022)
                     & (line[:, 2] >= SADDLE_Z + 0.010)
                     & (line[:, 2] <= SADDLE_Z + 0.028))
        if not np.any(in_saddle):
            return False
    return True


def _holder_pts(objects):
    p, R = pose(objects, HOLDER_A.name)
    pts = [p + R @ np.array([x, -0.018, 0.0]) for x in (0.05, 0.0, -0.08, -0.14, -0.2, -0.28)]
    pb, Rb = pose(objects, HOLDER_B.name)
    pts += [pb + Rb @ np.array([x, 0.02, 0.0]) for x in (-0.06, -0.14, -0.19)]
    return np.asarray(pts)


def _rod_dist_pts(P, objects) -> float:
    a, b = rod_ends(objects)
    d = b - a
    t = np.clip((P - a) @ d / (d @ d), 0, 1)
    return float(np.linalg.norm(P - (a + t[:, None] * d), axis=1).min())


def _lamp(on: bool) -> None:
    """The welder's amber power lamp glows bright amber while ON and dims to dark amber when OFF."""
    path = _ST.get("lamp")
    if not path or _ST.get("lamp_on") is on:
        return
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdGeom

        prim = get_current_stage().GetPrimAtPath(path)
        if prim and prim.IsValid():
            im = UsdGeom.Imageable(prim)
            im.MakeVisible() if on else im.MakeInvisible()
            _ST["lamp_on"] = on
    except Exception as e:  # noqa: BLE001
        print(f"[st] lamp update failed: {e!r}", flush=True)


def _rod_dist(g, objects) -> float:
    a, b = rod_ends(objects)
    d = b - a
    t = float(np.clip((g - a) @ d / (d @ d), 0, 1))
    return float(np.linalg.norm(g - (a + t * d)))


TOUCH_ROD = ROD_R + 0.0015  # any gripper/finger surface point within 1.5 mm of the rod surface counts as contact
LIVE_TABLE_Z = TOP + 0.002  # energized rod end touching the steel welding table
MOVED = 0.02  # holder or rod displaced > 2 cm from start while the welder is ON
CLEAR = 0.15  # parked: TCPs at least 15 cm clear of holder, rod and switch


def _tracked(objects) -> None:
    for o in objects.values():
        if not np.isfinite(np.asarray(o.pos, float)).all():
            _EP["nan"] = True
            return
    g = grippers()
    ang = rocker_angle(objects)
    _set_rocker_target(18.0 if ang >= 10.0 else 0.0)
    off = ang >= ROCKER_OFF_DEG
    _lamp(not off)
    if not off and not _EP["violation"]:
        gps = gripper_points()
        if not gps:
            _EP["violation"] = "missing gripper state"
        for pts in gps:
            if _rod_dist_pts(pts, objects) < TOUCH_ROD:
                _EP["violation"] = _EP["violation"] or "gripper touched the rod while the welder is ON"
                break
        ra, rb = rod_ends(objects)
        if min(float(ra[2]), float(rb[2])) <= LIVE_TABLE_Z:
            _EP["violation"] = _EP["violation"] or "energized rod touched the steel table"
        p0 = _EP.setdefault("start", {})
        pa, Ra = pose(objects, HOLDER_A.name)
        pr, _ = pose(objects, ROD.name)
        for key, pt in ((HOLDER_A.name, pa), (f"{HOLDER_A.name}_rear", pa + Ra @ np.array([-0.20, -0.018, 0.0])),
                        (ROD.name, pr)):
            p0.setdefault(key, pt)
            if np.linalg.norm(pt - p0[key]) > MOVED:
                _EP["violation"] = _EP["violation"] or f"{key.removesuffix('_rear')} moved while the welder is ON"
    if rod_in_jaws(objects) and holder_in_hanger(objects):
        _EP["violation"] = _EP["violation"] or "holder hung with the rod in"
    for name in (HOLDER_A.name, ROD.name):
        if objects[name].low[2] < TOP - 0.05:
            _EP["violation"] = _EP["violation"] or f"{name} fell off the table"
    if g:
        _EP["hist"].append((np.concatenate(g), _min_clear(objects)))
        _EP["hist"] = _EP["hist"][-12:]


def _min_clear(objects) -> float:
    g = grippers()
    if not g:
        return 0.0
    pts = list(_holder_pts(objects)) + [np.asarray(ROCKER_P)]
    best = 1.0
    for x in g:
        best = min(best, float(np.linalg.norm(np.asarray(pts) - x, axis=1).min()), _rod_dist(x, objects))
    return best


def parked(objects) -> bool:
    if len(_EP["hist"]) < 11 or _min_clear(objects) < CLEAR:
        return False
    recent = _EP["hist"][-11:]
    if any(c < CLEAR for _, c in recent):
        return False
    h = np.asarray([p for p, _ in recent])
    v = np.linalg.norm(np.diff(h.reshape(len(h), 2, 3), axis=0), axis=2).max() * 20.0
    return bool(v < 0.02)


def score(objects) -> float:
    _tracked(objects)
    if _EP["nan"] or _EP["violation"]:
        return 0.0
    if not welder_off(objects):
        return 0.0
    if _min_clear(objects) < 0.05:
        return 0.0
    if rod_in_jaws(objects):
        return 0.2
    if not rod_in_can(objects):
        return 0.4
    if holder_hung(objects) and parked(objects):
        return 1.0
    return 0.6


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    return 1.0 - score(objects)


def progress(objects) -> dict:
    return {"welder_off": int(welder_off(objects)), "rod_out": int(not rod_in_jaws(objects)),
            "rod_in_can": int(rod_in_can(objects)), "holder_hung": int(holder_hung(objects))}


VARIATION = {
    "obj:holder_handle": {"dx": 0.01, "dy": 0.01, "yaw": 0.05},
    "obj:rod_stub": {"dx": 0.01, "dy": 0.01, "yaw": 0.0},
}
