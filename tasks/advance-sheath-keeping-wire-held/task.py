"""A training phantom for the Seldinger technique lies on the station table. A skin-coloured block (20 x 6 x 3 cm) has
a tissue tract (19 x 13.5 mm, 10 cm deep) behind its entry face, then a narrow bore. A sprung lip in the tract's mouth
presses down on whatever passes under it. A 0.9 mm black guidewire runs 12 cm into the phantom and 38 cm out of it along
one straight line (heading 70 deg); a white band on the wire sits just outside the entry. A free blue sheath (a closed
tube: 9 cm shaft, 5 cm hub) lies around the wire with its nose 26 cm from the entry and the wire's tail 2 cm inside its
hub. A sprung valve pad in the hub presses on the wire, so the sheath drags the wire with it unless the wire is held.

The goal: push the sheath along the wire and into the tract until its hub meets the skin, then draw the wire out
through the sheath, without the wire's tip moving in the phantom before the sheath is seated. One arm must hold the
wire while the other pushes the sheath; the holds swap when the tail comes out of the sheath (the left arm cannot reach
the tail, the right arm cannot reach the entry).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("On this training phantom, push the blue sheath along the wire and into the phantom until its hub meets "
               "the skin, then draw the wire out through the sheath. Let go of everything at the end.")

# -- layout: everything lies on one line through the entry at heading 70 deg -----------------------------------------
# s = distance (m) from the phantom's entry face along the line towards the robot's right front; s < 0 is inside it.
HEAD = math.radians(70.0)
D = np.array([math.cos(HEAD), math.sin(HEAD)])  # unit vector pointing INTO the phantom
NRM = np.array([-D[1], D[0]])
PORT = np.array([0.33, 0.14])  # the phantom's entry face, table coordinates


def at(s: float, v: float = 0.0) -> np.ndarray:
    """Table xy of the point s along the line (s > 0 outside the phantom) and v to its side."""
    return PORT - s * D + v * NRM


SKIN = (0.93, 0.74, 0.62)
LIP_C = (0.30, 0.31, 0.33)
BLUE = (0.10, 0.35, 0.85)
BAND_COLOR = (0.97, 0.97, 0.97)

# phantom: fixed block from s 0 to s -0.20. Behind the face a tract fits the sheath's shaft (floor = table), then a
# narrow bore carries the wire on. The roof over the tract's first 4 cm is open: the sprung lip sits in that slot.
PH_L, PH_W, PH_H = 0.20, 0.06, 0.03
TRACT_L, TR_W, TR_TOP = 0.10, 0.019, 0.0135
BORE_W, BORE_TOP = 0.009, 0.0105
FACE_T = 0.004  # the face above the tract (s 0 to -4 mm) is solid: the sheath's taller hub stops against it
SLOT0, SLOT1 = -FACE_T, -0.044  # open slot in the tract roof that holds the lip
WIN1 = -0.140  # the bore's roof is open from the tract's end to here: a window over the wire's tip (s -0.12), so a
#                camera above sees the tip and two marks 1.5 cm either side of where it starts


def _pbox(name, s0, s1, v0, v1, z0, z1, color=SKIN):
    c = at((s0 + s1) / 2, (v0 + v1) / 2)
    return Box(name, pos=(float(c[0]), float(c[1]), TOP + (z0 + z1) / 2), size=(abs(s1 - s0), v1 - v0, z1 - z0),
               yaw=HEAD, color=color)


PHANTOM = [
    _pbox("tract_side_a", 0.0, -TRACT_L, TR_W / 2, PH_W / 2, 0.0, PH_H),
    _pbox("tract_side_b", 0.0, -TRACT_L, -PH_W / 2, -TR_W / 2, 0.0, PH_H),
    _pbox("face_top", 0.0, SLOT0, -TR_W / 2, TR_W / 2, TR_TOP, PH_H),
    _pbox("tract_roof", SLOT1, -TRACT_L, -TR_W / 2, TR_W / 2, TR_TOP, PH_H),
    _pbox("tract_end_a", -TRACT_L, -TRACT_L - 0.004, BORE_W / 2, TR_W / 2, 0.0, PH_H),
    _pbox("tract_end_b", -TRACT_L, -TRACT_L - 0.004, -TR_W / 2, -BORE_W / 2, 0.0, PH_H),
    _pbox("tract_end_top", -TRACT_L, -TRACT_L - 0.004, -BORE_W / 2, BORE_W / 2, BORE_TOP, PH_H),
    _pbox("bore_side_a", -TRACT_L - 0.004, -PH_L, BORE_W / 2, PH_W / 2, 0.0, PH_H),
    _pbox("bore_side_b", -TRACT_L - 0.004, -PH_L, -PH_W / 2, -BORE_W / 2, 0.0, PH_H),
    _pbox("bore_roof", WIN1, -PH_L, -BORE_W / 2, BORE_W / 2, BORE_TOP, PH_H),
    _pbox("bore_end", -PH_L + 0.01, -PH_L, -BORE_W / 2, BORE_W / 2, 0.0, BORE_TOP),
]

# guidewire: thin black wire, tip 12 cm inside the phantom (the tract and bore have no floor), tail 38 cm out; inside
# the sheath it runs in the lumen, on the lumen floor
WIRE_D = 0.0009  # 0.9 mm: a standard 0.035-inch guidewire
R = WIRE_D / 2
HUB_FLOOR = 0.0015  # the sheath's lumen floor (the sheath is a closed tube)
WIRE_Z = TOP + HUB_FLOOR + R + 0.0005  # spawn height; outside the hub the wire settles onto the table
TIP_S, TAIL_S = -0.12, 0.38
WIRE_LEN = TAIL_S - TIP_S
WIRE_MU = 0.5  # slippery coated wire; "min" friction combine (spawn hook)
_tip = at(TIP_S)
WIRE = cable("wire", start=(float(_tip[0]), float(_tip[1]), WIRE_Z), yaw=HEAD + math.pi, length=WIRE_LEN, radius=R,
             cable_mass=0.015, color=(0.08, 0.08, 0.09), bend_stiffness=0.03, bend_damping=0.003, friction=WIRE_MU)
BAND_SEG = int(round((0.0 - TIP_S) / WIRE.seg_len))  # the segment from s 0 to s +2 cm (just outside the entry)
BAND_S0 = TIP_S + (BAND_SEG + 0.5) * WIRE.seg_len  # its centre at the start

# sheath: one free rigid body, a closed tube. A 9 cm shaft (14 x 12 mm outside, 9 x 7 mm lumen) that fits the tract,
# then a 5 cm hub 27.5 mm tall that does not fit. The lumen has a 1.5 mm floor along the whole sheath; at the nose the
# floor tapers to a thin lead-in edge (1 cm wedge) that slides under the wire. The hub's valve pad pinches the wire
# against that floor.
SH_L, SH_W, SH_H = 0.14, 0.014, 0.012
RAMP_L = 0.010  # the lumen floor's lead-in wedge at the nose
HUB_L, HUB_TOP = 0.05, 0.0275
SHAFT_L = SH_L - HUB_L
NOSE0 = 0.26
SH_C = NOSE0 + SH_L / 2  # s of the sheath centre
ROOF_B = 0.0085  # shaft bore roof underside
VALVE_L, VALVE_W, VALVE_H, VALVE_M = 0.020, 0.008, 0.010, 0.003
BAR_T = 0.004
SH_SPAWN = 0.0005


def _sbox(name, s0, s1, v0, v1, z0, z1, mass, color=BLUE):
    c = at((s0 + s1) / 2, (v0 + v1) / 2)
    return Box(name, pos=(float(c[0]), float(c[1]), TOP + SH_SPAWN + (z0 + z1) / 2),
               size=(abs(s1 - s0), v1 - v0, z1 - z0), yaw=HEAD, mass=mass, color=color)


REAR0 = NOSE0 + SH_L
HUB0 = REAR0 - HUB_L  # hub front (s)
# the body itself is the sheath's outer envelope (mass and inertia; its own collider is switched off in the hook)
SHEATH = _sbox("sheath", NOSE0, REAR0, -SH_W / 2, SH_W / 2, 0.0, SH_H, 0.030)
SH_PARTS = [
    _sbox("sheath_wall_a", NOSE0, REAR0, 0.0045, SH_W / 2, 0.0, SH_H, 0.003),
    _sbox("sheath_wall_b", NOSE0, REAR0, -SH_W / 2, -0.0045, 0.0, SH_H, 0.003),
    _sbox("sheath_roof", NOSE0, HUB0, -0.0045, 0.0045, ROOF_B, SH_H, 0.003),
    _sbox("hub_wall_a", HUB0, REAR0, 0.0045, SH_W / 2, SH_H, HUB_TOP - 0.003, 0.001),
    _sbox("hub_wall_b", HUB0, REAR0, -SH_W / 2, -0.0045, SH_H, HUB_TOP - 0.003, 0.001),
    _sbox("hub_cap", HUB0, REAR0, -SH_W / 2, SH_W / 2, HUB_TOP - 0.003, HUB_TOP, 0.002),
    _sbox("hub_bar_f", HUB0, HUB0 + BAR_T, -0.0045, 0.0045, ROOF_B, HUB_TOP - 0.003, 0.001),
    _sbox("hub_bar_r", HUB0 + BAR_T + VALVE_L + 0.004, HUB0 + 2 * BAR_T + VALVE_L + 0.004, -0.0045, 0.0045,
          ROOF_B, HUB_TOP - 0.003, 0.001),
    _sbox("sheath_floor", NOSE0 + RAMP_L, REAR0, -0.0045, 0.0045, 0.0, HUB_FLOOR, 0.002),
]
_v0 = HUB0 + BAR_T + 0.002
VALVE_Z0 = HUB_FLOOR + 2 * R + 0.0005  # pad bottom just above the wire at spawn
VALVE = Box("valve_pad", pos=(*map(float, at(_v0 + VALVE_L / 2)), TOP + SH_SPAWN + VALVE_Z0 + VALVE_H / 2),
            size=(VALVE_L, VALVE_W, VALVE_H), yaw=HEAD, mass=VALVE_M, color=(0.05, 0.20, 0.55))
VALVE_K, VALVE_PRE = 20.0, 0.020  # spring (N/m) and preload travel (m): about 0.4 N on the wire
_SC = np.array([*at(SH_C), TOP + SH_SPAWN + SH_H / 2])
_RI = np.array([[math.cos(HEAD), math.sin(HEAD), 0.0], [-math.sin(HEAD), math.cos(HEAD), 0.0], [0.0, 0.0, 1.0]])
SH_REL = {p.name: _RI @ (np.asarray(p.pos) - _SC) for p in SH_PARTS}  # offsets in the sheath's own frame
VALVE_REL = _RI @ (np.asarray(VALVE.pos) - _SC)

# sprung lip in the tract's mouth: a block on a vertical slide, spring-loaded down onto the shaft's roof. At rest it
# hangs 1 mm below the roof's height, so the shaft lifts it on entry; a chamfer on its front guides the nose under it.
LIP_L, LIP_W, LIP_H = 0.024, 0.012, 0.014
LIP_S0, LIP_S1 = -0.016, -0.040
LIP_Z0 = SH_H - 0.001
LIP_K = 80.0  # N/m
LIP_PRE = 0.010  # preload travel (m): 0.8 N at rest, about 0.9 N with the shaft under it
LIP_TRAVEL = 0.008
LIP_DAMP = 1.2  # N s/m: about critical damping for the 4 g lip on its spring
_lc = at((LIP_S0 + LIP_S1) / 2)
LIP = Box("port_lip", pos=(float(_lc[0]), float(_lc[1]), TOP + LIP_Z0 + LIP_H / 2), size=(LIP_L, LIP_W, LIP_H),
          yaw=HEAD, mass=0.004, color=LIP_C)

# two red tape marks across the phantom's top, 1.5 cm either side of the tip's start, framing the window
MARKS = [_pbox(f"tip_mark_{k}", m + 0.00075, m - 0.00075, -PH_W / 2, PH_W / 2, PH_H, PH_H + 0.0005, (0.85, 0.1, 0.1))
         for k, m in (("in", TIP_S - 0.015), ("out", TIP_S + 0.015))]

OBJECTS = [*PHANTOM, *MARKS, LIP, SHEATH, VALVE, WIRE]

TWIN_EXTRA = [
    *({"name": p.name, "kind": "body", "size_mm": [round(v * 1000, 1) for v in p.size], "colour": "blue",
       "pose": [round(float(v), 4) for v in (p.pos[0], p.pos[1], p.pos[2] - TOP)], "note": "part of the one-piece sheath"}
      for p in SH_PARTS),
    {"name": "wire_band", "kind": "decal", "size_mm": [20, round(WIRE_D * 1000, 1)], "colour": "white",
     "pose": [float(at(0.01)[0]), float(at(0.01)[1]), round(R, 4)]},
]

VARIATION = {"cable:wire": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}  # the wire runs through the fixed bore: no variation


# -- spawn hook: build the sheath, the sprung lip and valve, paint the band --------------------------------------------
def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_sheath_task", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != WIRE.name:
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        mat = stage.GetPrimAtPath(f"{prim_path}/Looks/cable_physics")
        PhysxSchema.PhysxMaterialAPI.Apply(mat).CreateFrictionCombineModeAttr().Set("min")
        root = str(prim_path).rsplit("/", 1)[0]
        w = UsdGeom.Xformable(stage.GetPrimAtPath(root)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        W = np.array([w[0], w[1], w[2]], dtype=float)
        body = f"{root}/{SHEATH.name}"

        # the envelope box only carries mass: no collider, not drawn; inertia of a 14 x 12 mm x 14 cm block
        for p in Usd.PrimRange(stage.GetPrimAtPath(body)):
            if p.GetPath().pathString != body and p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
            if p.IsA(UsdGeom.Gprim):
                UsdGeom.Imageable(p).MakeInvisible()
        m = SHEATH.mass
        ma = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(body))
        ma.CreateDiagonalInertiaAttr(Gf.Vec3f(m * (SH_W ** 2 + SH_H ** 2) / 12, m * (SH_L ** 2 + SH_H ** 2) / 12,
                                              m * (SH_L ** 2 + SH_W ** 2) / 12))
        ma.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))

        look = f"{prim_path}/Looks/sheath_blue"
        vm = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BLUE), roughness=0.5)
        vm.func(look, vm)
        pmat = f"{prim_path}/Looks/sheath_physics"
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=1.2, dynamic_friction=1.0)
        pm.func(pmat, pm)

        def cube(path, rel, size, lk, orient=None, phys=None):
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, rel)))
            if orient is not None:
                g.AddOrientOp().Set(orient)
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(g.GetPath()), phys or pmat, stage=stage)
            bind_visual_material(str(g.GetPath()), lk, stage=stage)

        size = {p.name: p.size for p in SH_PARTS}
        for name, rel in SH_REL.items():
            cube(f"{body}/{name}", rel, size[name], look)
        # lead-in wedge at the nose: from a thin edge at the nose up to the lumen floor (sheath frame: +x to the nose)
        xa, xb, w = SH_C - NOSE0 - RAMP_L, SH_C - NOSE0, 0.0045
        z0 = -SH_H / 2
        z1 = z0 + HUB_FLOOR
        pts = [(xa, -w, z0), (xa, w, z0), (xa, -w, z1), (xa, w, z1), (xb, -w, z0), (xb, w, z0),
               (xb, -w, z0 + 0.0001), (xb, w, z0 + 0.0001)]
        ramp = UsdGeom.Mesh.Define(stage, f"{body}/nose_ramp")
        ramp.CreatePointsAttr([Gf.Vec3f(*map(float, q)) for q in pts])
        ramp.CreateFaceVertexCountsAttr([4] * 6)
        ramp.CreateFaceVertexIndicesAttr([0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1, 2, 3, 7, 6, 0, 2, 6, 4, 1, 5, 7, 3])
        UsdPhysics.CollisionAPI.Apply(ramp.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(ramp.GetPrim()).CreateApproximationAttr("convexHull")
        rcp = PhysxSchema.PhysxCollisionAPI.Apply(ramp.GetPrim())
        rcp.CreateContactOffsetAttr(0.002)
        rcp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(ramp.GetPath()), pmat, stage=stage)
        bind_visual_material(str(ramp.GetPath()), look, stage=stage)

        # sprung lip: chamfer on its front (pad frame: +x points into the phantom), then a vertical slide to the world
        lip = f"{root}/{LIP.name}"
        llook = f"{prim_path}/Looks/lip"
        lm = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(LIP_C), roughness=0.6)
        lm.func(llook, lm)
        slick = f"{prim_path}/Looks/lip_lead_in"
        sm = sim_utils.RigidBodyMaterialCfg(static_friction=0.1, dynamic_friction=0.1)
        sm.func(slick, sm)
        PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(slick)).CreateFrictionCombineModeAttr().Set("min")
        a = np.array([-LIP_L / 2, 0.0, -LIP_H / 2 - 0.0003])  # covers the block's front-bottom edge
        b = np.array([-LIP_L / 2 - 0.012, 0.0, -LIP_H / 2 + 0.0035])  # 17 deg lead-in, smooth (slick)
        beta = math.atan2(b[2] - a[2], a[0] - b[0])  # x -> (a - b) direction
        n = np.array([math.sin(beta), 0.0, math.cos(beta)])
        t = 0.003
        cube(f"{lip}/chamfer", (a + b) / 2 + n * t / 2, (float(np.linalg.norm(b - a)), LIP_W, t), llook,
             Gf.Quatf(math.cos(beta / 2), 0.0, math.sin(beta / 2), 0.0), phys=slick)
        j = UsdPhysics.PrismaticJoint.Define(stage, f"{root}/port_lip_slide")
        j.CreateBody1Rel().SetTargets([Sdf.Path(lip)])
        j.CreateAxisAttr("Z")
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, W + np.asarray(LIP.pos))))
        q = Gf.Quatf(math.cos(HEAD / 2), 0.0, 0.0, math.sin(HEAD / 2))
        j.CreateLocalRot0Attr(q)
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLowerLimitAttr(0.0)
        j.CreateUpperLimitAttr(LIP_TRAVEL)
        j.CreateExcludeFromArticulationAttr(True)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(LIP_K)
        d.CreateDampingAttr(LIP_DAMP)
        d.CreateTargetPositionAttr(-LIP_PRE)
        fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(lip))  # it rides in its slot
        for p in PHANTOM:
            fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(f"{root}/{p.name}"))

        # hub valve: the pad slides vertically in the hub, spring-loaded down onto the wire
        valve = f"{root}/{VALVE.name}"
        jv = UsdPhysics.PrismaticJoint.Define(stage, f"{root}/valve_slide")
        jv.CreateBody0Rel().SetTargets([Sdf.Path(body)])
        jv.CreateBody1Rel().SetTargets([Sdf.Path(valve)])
        jv.CreateAxisAttr("Z")
        jv.CreateLocalPos0Attr(Gf.Vec3f(*map(float, VALVE_REL)))
        jv.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        jv.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        jv.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        jv.CreateLowerLimitAttr(-2 * R)  # with no wire under it the pad stops 0.5 mm above the floor
        jv.CreateUpperLimitAttr(0.006)
        jv.CreateExcludeFromArticulationAttr(True)
        dv = UsdPhysics.DriveAPI.Apply(jv.GetPrim(), "linear")
        dv.CreateTypeAttr("force")
        dv.CreateStiffnessAttr(VALVE_K)
        dv.CreateDampingAttr(0.05)
        dv.CreateTargetPositionAttr(-VALVE_PRE)
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(valve)).CreateFilteredPairsRel().AddTarget(Sdf.Path(body))
        try:
            blook = f"{prim_path}/Looks/band"
            bm = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BAND_COLOR), roughness=0.5)
            bm.func(blook, bm)
            bind_visual_material(f"{prim_path}/seg_{BAND_SEG}/geom", blook, stage=stage)
        except Exception as e:  # pragma: no cover - never break the scene for a colour
            print(f"[sheath task] band colour not applied: {e}", flush=True)
        return prim

    spawn._sheath_task = True
    cablekit._spawn_cable = spawn


_install_spawn()

# -- episode memory (latched until the next reset) and env handle -----------------------------------------------------
_STATE = {"env": None, "slipped": False, "seated_once": False, "max_slip": 0.0, "broken": False}


def _reset_episode() -> None:
    _STATE.update(slipped=False, seated_once=False, max_slip=0.0, broken=False)


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_sheath_env", False):
        base_step = cls._task_step

        def _task_step(self, *a, **k):
            _STATE["env"] = self._env
            return base_step(self, *a, **k)

        _task_step._sheath_env = True
        cls._task_step = _task_step
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._task_reset_hooks = []
        cls.reset = reset
    if _reset_episode not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_episode)


_install_hooks()


def grippers() -> list[np.ndarray]:
    """World TCP positions of both grippers (env 0), [] offline."""
    env = _STATE["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- geometry ---------------------------------------------------------------------------------------------------------
SLIP_TOL = 0.015  # the wire's tip may not move more than 1.5 cm along the bore before the sheath is seated
ADVANCE_MIN = 0.12  # sheath nose advanced at least 12 cm (from s 0.26 to s 0.14 or closer)
SEAT_TOL = 0.005  # hub front within 5 mm of the skin face
SEAT_NOSE = -SHAFT_L  # nose s when the hub touches the face
AXIS_TOL = 0.010  # sheath centre line within 1 cm of the line
TIP_OUT = 0.03  # wire tip at least 3 cm behind the sheath's rear
RELEASE = 0.035  # both grippers at least 3.5 cm from the wire, the sheath and its valve


def _rot(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = min(200, max(1, int(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def s_of(p) -> float:
    return float((PORT - np.asarray(p, dtype=float)[:2]) @ D)


def v_of(p) -> float:
    return float((np.asarray(p, dtype=float)[:2] - PORT) @ NRM)


def _finite(objects) -> bool:
    w, sh = objects["wire"], objects["sheath"]
    return bool(np.isfinite(w.points).all() and np.isfinite(sh.pos).all() and np.isfinite(sh.quat_xyzw).all())


def sheath_ends(objects) -> tuple[np.ndarray, np.ndarray]:
    """(nose, rear) centre points of the sheath, world."""
    sh = objects["sheath"]
    ax = _rot(sh.quat_xyzw) @ np.array([1.0, 0.0, 0.0])  # local +x points into the phantom
    c = np.asarray(sh.pos, dtype=float)
    return c + ax * SH_L / 2, c - ax * SH_L / 2


def nose_s(objects) -> float:
    return s_of(sheath_ends(objects)[0])


def on_axis(objects) -> bool:
    n, r = sheath_ends(objects)
    return bool(max(abs(v_of(n)), abs(v_of(r))) <= AXIS_TOL and max(n[2], r[2]) <= TOP + 0.02)


def seated(objects) -> bool:
    """The shaft is in the tract and the hub's front is within 5 mm of the skin face."""
    return bool(on_axis(objects) and nose_s(objects) <= SEAT_NOSE + SEAT_TOL)


def band_shift(objects) -> float:
    """How far (m) the band centre is from where it started."""
    b = np.asarray(objects["wire"].points[BAND_SEG], dtype=float)
    b0 = np.array([*at(BAND_S0), TOP + R])
    return float(np.linalg.norm(b - b0))


def tip_s(objects) -> float:
    return s_of(objects["wire"].start_tip)


def tip_shift(objects) -> float:
    """How far (m) the wire's tip has moved along the bore from where it started (in or out)."""
    return abs(tip_s(objects) - TIP_S)


def wire_out(objects) -> bool:
    """The wire's tip is out past the sheath's rear by at least 3 cm."""
    return bool(tip_s(objects) >= s_of(sheath_ends(objects)[1]) + TIP_OUT)


def _sheath_pts(objects) -> np.ndarray:
    n, r = sheath_ends(objects)
    pts = [n + (r - n) * t for t in np.linspace(0.0, 1.0, 17)]
    pts.append(np.asarray(objects["valve_pad"].pos, dtype=float))
    return np.array(pts)


def released(objects) -> bool:
    w = _dense(objects["wire"].polyline())
    sp = _sheath_pts(objects)
    return all(min(float(np.linalg.norm(w - g, axis=1).min()), float(np.linalg.norm(sp - g, axis=1).min())) >= RELEASE
               for g in grippers())


def _track(objects) -> None:
    """Latch, every control step: the wire's tip moving more than 1.5 cm along the bore (in or out) at any time before
    the sheath has first been seated. Never cleared until the next trial. A non-finite simulator state is latched too (the trial scores 0)."""
    if not _finite(objects):
        _STATE["broken"] = True
        return
    if not _STATE["seated_once"] and seated(objects):
        _STATE["seated_once"] = True
    if not _STATE["seated_once"]:
        sh = tip_shift(objects)
        _STATE["max_slip"] = max(_STATE["max_slip"], sh)
        if sh > SLIP_TOL:
            _STATE["slipped"] = True


def score(objects) -> float:
    """0 if: before the sheath was first seated, the wire's tip moved more than 1.5 cm along the bore at any time; a
    gripper is within 3.5 cm of the wire or the sheath at the end; the sheath is off the line or its nose advanced
    less than 12 cm. Otherwise 0.3: nose advanced 12 cm or more, not seated; 0.6: seated (hub within 5 mm of the
    skin), wire not yet out; 1.0: seated and the wire tip at least 3 cm out past the sheath's rear."""
    _track(objects)
    if _STATE["broken"] or _STATE["slipped"] or not released(objects) or not on_axis(objects):
        return 0.0
    if NOSE0 - nose_s(objects) < ADVANCE_MIN:
        return 0.0
    if not seated(objects):
        return 0.3
    return 1.0 if wire_out(objects) else 0.6


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and objects["sheath"].resting(0.01, 0.5) and objects["wire"].resting(0.02))


def distance(objects) -> float:
    _track(objects)
    if _STATE["broken"] or _STATE["slipped"]:
        return 1.0
    return float(max(0.0, nose_s(objects) - SEAT_NOSE) + max(0.0, s_of(sheath_ends(objects)[1]) + TIP_OUT
                                                            - tip_s(objects)))


def progress(objects) -> dict:
    _track(objects)
    if _STATE["broken"]:
        return {"wire_kept": 0, "advanced": 0, "seated": 0, "wire_out": 0, "released": 0}
    return {"wire_kept": int(not _STATE["slipped"]),
            "advanced": float(min(1.0, max(0.0, (NOSE0 - nose_s(objects)) / (NOSE0 - SEAT_NOSE)))),
            "seated": int(seated(objects)), "wire_out": int(seated(objects) and wire_out(objects)),
            "released": int(released(objects))}


# -- closed-loop scripted demo (cabledemo.py; it reads the simulator state) --------------------------------------------
YAW = HEAD  # both wrist cameras look along the line towards the entry (where the sheath goes)
GRASP_OPEN = -0.014  # pads ~28 mm apart
WIRE_GRIP_Z = TOP + 0.010  # TCP aimed 3 mm below pad-tips-on-table: the pads press on the table either side of the wire
SH_GRIP_Z = TOP + SH_H / 2 + 0.010
HI = TOP + 0.07
FACE_CLEAR = 0.022  # a pinching TCP stays at least this far outside the face
FIRST_NOSE = 0.115  # right arm's push ends with the nose here
L_REACH = 0.13  # farthest s at which the left arm pinches the sheath


def _log(ctx, tag=""):
    st = ctx.states()
    if not _finite(st):
        bad = [k for k, o in st.items() if hasattr(o, "pos") and not np.isfinite(np.asarray(o.pos, dtype=float)).all()]
        print(f"[t={ctx.t:5.2f}] {tag}: NON-FINITE STATE {bad}", flush=True)
        return
    _track(st)
    w = st["wire"]
    print(f"[t={ctx.t:5.2f}] {tag}: nose_s={nose_s(st):.3f} slip={tip_shift(st) * 1000:.1f}mm "
          f"(max {_STATE['max_slip'] * 1000:.1f}) band={band_shift(st) * 1000:.1f}mm tip_s={tip_s(st):.3f} rear_s={s_of(sheath_ends(st)[1]):.3f} "
          f"axis={on_axis(st)} seated={seated(st)} valve_z={st['valve_pad'].pos[2] - TOP:.4f} "
          f"lip_z={st['port_lip'].pos[2] - TOP - LIP_H / 2:.4f} wire_zmin={w.points[:, 2].min() - TOP:.4f} "
          f"wire_v={np.abs(w.lin_vel).max():.3f} buckle={buckle(st) * 1000:.1f}mm rel={released(st)} "
          f"score={score(st):.2f}", flush=True)
    if os.environ.get("SHEATH_DEBUG") == "1":
        n, r = sheath_ends(st)
        g = grippers()
        f = (lambda q: [round(float(x), 4) for x in (s_of(q), v_of(q), q[2] - TOP)])
        print(f"    dbg nose={f(n)} rear={f(r)} tcpL={f(g[0]) if g else None} tcpR={f(g[1]) if g else None}",
              flush=True)
        print("    wire " + " ".join(f"{s_of(q) * 100:.1f}/{v_of(q) * 1000:.0f}/{(q[2] - TOP) * 1000:.1f}"
                                     for q in st["wire"].points), flush=True)


def buckle(st) -> float:
    """Largest sideways offset (m) of the wire from the line between the entry and the sheath's nose."""
    ns = nose_s(st)
    v = [abs(v_of(q)) for q in st["wire"].points if 0.0 < s_of(q) < ns]
    return max(v) if v else 0.0


def _relax(arm, arm_i, ctx, max_back=0.03):
    """Back the pinched sheath off until the wire in front of it lies straight again."""
    for _ in range(6):
        st = ctx.states()
        if buckle(st) < 0.004:
            break
        t = _tcp(arm_i)
        g = at(s_of(t) + 0.005, 0.0)
        max_back -= 0.005
        if max_back < 0:
            break
        yield from arm.move((g[0], g[1], SH_GRIP_Z), yaw=YAW, duration=0.5)
    _log(ctx, "relaxed")


def _wire_at(st, s):
    """Live wire point nearest to the line coordinate s (by projection)."""
    p = st["wire"].points
    i = int(np.argmin([abs(s_of(q) - s) for q in p]))
    return np.asarray(p[i], dtype=float)


def _pinch_wire(arm, ctx, s):
    from yam_arena.constants import FINGER_CLOSED

    p = _wire_at(ctx.states(), s)
    yield from arm.move((p[0], p[1], HI), yaw=YAW, finger=GRASP_OPEN, duration=1.5)
    p = _wire_at(ctx.states(), s)
    yield from arm.move((p[0], p[1], WIRE_GRIP_Z + (p[2] - TOP - R)), yaw=YAW, duration=1.0)
    yield from arm.grip(FINGER_CLOSED, 0.6)


def _pinch_sheath(arm, ctx, from_nose):
    """Pinch the sheath across its walls `from_nose` metres behind its nose."""
    from yam_arena.constants import FINGER_CLOSED

    st = ctx.states()
    n, r = sheath_ends(st)
    p = n + (r - n) * (from_nose / SH_L)
    yield from arm.move((p[0], p[1], HI), yaw=YAW, finger=GRASP_OPEN, duration=1.5)
    yield from arm.move((p[0], p[1], SH_GRIP_Z), yaw=YAW, duration=1.0)
    yield from arm.grip(FINGER_CLOSED, 0.6)


def _tcp(arm_i):
    g = grippers()
    return g[arm_i] if g else None


def _push(arm, arm_i, ctx, goal_nose_s, step=0.03, dur=1.2, tries=30, z=SH_GRIP_Z):
    """Slide the pinched sheath along the line until its nose reaches goal_nose_s, closed loop on the live nose and
    the live TCP. Stops early if the sheath stalls (hub against the skin)."""
    last, stalls = None, 0
    for k in range(tries):
        st = ctx.states()
        ns = nose_s(st)
        if ns <= goal_nose_s + 0.002 or (last is not None and abs(last - ns) < 0.001 and ns <= goal_nose_s + 0.008):
            break
        stalls = stalls + 1 if last is not None and abs(last - ns) < 0.001 else 0
        if stalls >= 3:
            _log(ctx, "push stalled")
            break
        last = ns
        ds = min(step, ns - goal_nose_s + 0.001)
        t = _tcp(arm_i)
        g = at(s_of(t) - ds, 0.0)
        yield from arm.move((g[0], g[1], z), yaw=YAW, duration=dur * ds / step + 0.3)
        if k % 3 == 2 or os.environ.get("SHEATH_DEBUG") == "1":
            _log(ctx, f"push {k}")


def _pull(arm, arm_i, ctx, until, goal_s, z=None, step=0.04, dur=1.2):
    """Draw the pinched wire away from the entry along the line, step by step, until `until(states)` holds or the
    TCP reaches goal_s."""
    z = WIRE_GRIP_Z if z is None else z  # pads stay down at the table, below the thin wire's centre
    for k in range(20):
        st = ctx.states()
        if until(st):
            break
        t = _tcp(arm_i)
        s1 = min(goal_s, s_of(t) + step)
        if s1 <= s_of(t) + 0.002:
            break
        g = at(s1, 0.0)
        yield from arm.move((g[0], g[1], z), yaw=YAW, duration=dur)
        if k % 2 == 1:
            _log(ctx, f"pull {k}")
        if s_of(_tcp(arm_i)) < s_of(t) + 0.003:  # the hand has stopped (end of its low reach)
            break


def _let_go(arm, arm_i, ctx):
    yield from arm.grip(GRASP_OPEN, 0.5)
    t = _tcp(arm_i)
    yield from arm.move((t[0], t[1], HI), yaw=YAW, duration=0.8)


def _clear(ctx):
    yield from ctx.both(ctx.left.move((0.20, 0.25, TOP + 0.18), yaw=0.0, duration=1.2),
                        ctx.right.move((0.20, -0.25, TOP + 0.18), yaw=0.0, duration=1.2))
    for _ in range(6):
        yield from ctx.left.wait(0.5)
    _log(ctx, "end")


def _insert(L, ctx, goal_nose):
    """Left arm: pinch the sheath as far back as it reaches, push until the hand nears the face, let go (the lip
    holds the sheath once its nose is in), re-pinch further back (on the hub at the end) and push again."""
    for k in range(5):
        st = ctx.states()
        ns = nose_s(st)
        if ns <= goal_nose + 0.004 or (k and seated(st)):
            break
        grip_s = min(L_REACH, s_of(sheath_ends(st)[1]) - 0.02)  # fingers clear of the wire leaving the rear
        if k:
            yield from _let_go(L, 0, ctx)
            _log(ctx, f"left let go of the sheath ({k})")
        yield from _pinch_sheath(L, ctx, grip_s - ns)
        _log(ctx, f"left pinched sheath {grip_s - ns:.3f} behind nose ({k})")
        g = max(goal_nose, nose_s(ctx.states()) - (s_of(_tcp(0)) - FACE_CLEAR))
        yield from _push(L, 0, ctx, g, step=0.025)
        _log(ctx, f"left push done ({k})")


def _withdraw(R, ctx, near_s=0.20, far_s=0.37):
    """Right arm draws the held wire out through the sheath hand over hand, its pads down at the table so the thin
    wire stays between them: it pulls along the line to the end of its low reach (far_s), lets go, pinches the wire
    again at near_s and pulls again, until the tip is out past the sheath's rear."""
    out = lambda st: tip_s(st) >= s_of(sheath_ends(st)[1]) + TIP_OUT + 0.02  # noqa: E731
    for k in range(8):
        if out(ctx.states()):
            break
        t0, h0 = tip_s(ctx.states()), s_of(_tcp(1))
        yield from _pull(R, 1, ctx, out, far_s, step=0.04)
        st = ctx.states()
        _log(ctx, f"stroke {k}: hand {s_of(_tcp(1)) - h0:.3f} wire {tip_s(st) - t0:.3f}")
        if out(st):
            break
        yield from _let_go(R, 1, ctx)
        yield from _pinch_wire(R, ctx, max(near_s, s_of(sheath_ends(st)[1]) + 0.04))


def demo_script(ctx):
    """SHEATH_DEMO=correct (default). Variants: nothing | probe (wait 10 s, wire at rest) | hold (correct, but the left
    arm keeps the hub pinched) | pushfree (right arm pushes the sheath as far as it reaches, wire never held) |
    pullfirst (left arm draws the wire 6 cm out first, then the correct sequence) | onearm (right arm only: push 4 cm,
    let go, pinch the wire behind the sheath, pull it back, repeat) | shallow (correct, but the shaft is pushed only
    3 cm into the tract) | tier03 (stop after the first push, let go) | tier06 (stop once seated, let go) | lipcheck
    (correct, but the left arm lets go of the hub before the wire is drawn out)."""
    mode = os.environ.get("SHEATH_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    yield from L.wait(1.0)
    _log(ctx, "settled")
    if mode in ("nothing", "probe"):
        for k in range(10 if mode == "probe" else 5):
            yield from L.wait(1.0)
            if mode == "probe":
                _log(ctx, f"probe {k}")
        _log(ctx, "end")
        return
    if mode == "pushfree":
        yield from _pinch_sheath(Rt, ctx, 0.07)
        yield from _push(Rt, 1, ctx, 0.03)
        _log(ctx, "pushed")
        yield from _let_go(Rt, 1, ctx)
        yield from _clear(ctx)
        return
    if mode == "onearm":
        for rep in range(3):
            yield from _pinch_sheath(Rt, ctx, 0.07)
            yield from _push(Rt, 1, ctx, nose_s(ctx.states()) - 0.04)
            yield from _let_go(Rt, 1, ctx)
            rear = s_of(sheath_ends(ctx.states())[1])
            yield from _pinch_wire(Rt, ctx, min(rear + 0.02, s_of(ctx.states()["wire"].end_tip) - 0.01))
            _log(ctx, f"onearm wire pinched {rep}")
            yield from _pull(Rt, 1, ctx, lambda st: band_shift(st) < 0.004, s_of(_tcp(1)) + 0.06, step=0.01)
            yield from _let_go(Rt, 1, ctx)
            _log(ctx, f"onearm rep {rep}")
        yield from _clear(ctx)
        return
    if mode == "pullfirst":
        yield from _pinch_wire(L, ctx, 0.05)
        yield from _pull(L, 0, ctx, lambda st: False, 0.11, step=0.03)
        _log(ctx, "wire drawn first")
        yield from _let_go(L, 0, ctx)
    # 1) left arm holds the wire just outside the entry; right arm pinches the sheath 7 cm behind its nose
    yield from ctx.both(_pinch_wire(L, ctx, 0.05), _pinch_sheath(Rt, ctx, 0.07))
    _log(ctx, "gripped")
    # 2) right arm slides the sheath 14.5 cm along the held wire
    yield from _push(Rt, 1, ctx, FIRST_NOSE)
    _log(ctx, "first push done")
    yield from _relax(Rt, 1, ctx)
    # 3) right arm lets go of the sheath and takes the wire's tail, now out of the sheath's rear
    yield from _let_go(Rt, 1, ctx)
    _log(ctx, "right let go of sheath")
    tail = s_of(ctx.states()["wire"].end_tip)
    yield from _pinch_wire(Rt, ctx, tail - 0.06)
    _log(ctx, "tail held")
    if mode == "tier03":
        yield from _let_go(L, 0, ctx)
        yield from _let_go(Rt, 1, ctx)
        yield from _clear(ctx)
        return
    # 4) left arm lets go of the wire and pushes the sheath into the tract against the lip, re-pinching further back
    yield from _let_go(L, 0, ctx)
    yield from _insert(L, ctx, -0.03 if mode == "shallow" else SEAT_NOSE - 0.002)
    _log(ctx, "inserted")
    if mode == "tier06":
        yield from _let_go(Rt, 1, ctx)
        yield from _let_go(L, 0, ctx)
        yield from _clear(ctx)
        return
    # 5) left arm keeps the hub; right arm draws the wire out through the sheath, hand over hand
    if mode == "lipcheck":  # nobody holds the sheath: only the lip keeps it in while the valve drags on the wire
        yield from _let_go(L, 0, ctx)
        _log(ctx, "left let go of the hub")
    yield from _withdraw(Rt, ctx)
    _log(ctx, "wire out")
    if mode == "hold":
        yield from _let_go(Rt, 1, ctx)
        yield from Rt.move((0.20, -0.25, TOP + 0.18), yaw=0.0, duration=1.2)
        yield from L.wait(3.0)
        _log(ctx, "end")
        return
    yield from _let_go(Rt, 1, ctx)
    yield from _let_go(L, 0, ctx)
    yield from _clear(ctx)
