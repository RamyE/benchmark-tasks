"""Put a raincoat on a small-dog form and fasten its neck and belly straps under the body."""
from __future__ import annotations
import importlib.abc, importlib.util, math, os, sys  # noqa: E401
import numpy as np
from yam_arena.task_file import TABLE_TOP, Box
SCENE, TOP, SCORE_OVER, MAX_STEPS = "station", TABLE_TOP["station"], "end", 1500
INSTRUCTION = "Put the raincoat on the dog and fasten both straps, then let go."

SHELL_C, LINING_C, BIND_C = (0.95, 0.78, 0.10), (0.55, 0.57, 0.60), (0.12, 0.22, 0.45)
FORM_C, HOOK_C, LOOP_C, EYE_C = (0.93, 0.93, 0.91), (0.08, 0.08, 0.09), (0.42, 0.44, 0.47), (0.05, 0.05, 0.05)


def _quat_xyzw(R):
    from scipy.spatial.transform import Rotation
    return tuple(float(v) for v in Rotation.from_matrix(np.asarray(R, float)).as_quat())
def _rotmat_xyzw(q):
    from scipy.spatial.transform import Rotation
    return Rotation.from_quat(np.asarray(q, float)).as_matrix()
def _frame_x(xdir):
    x = np.asarray(xdir, float) / np.linalg.norm(xdir)
    up = np.array([0, 0, 1.0]) if abs(x[2]) < 0.9 else np.array([0, 1.0, 0])
    y = np.cross(up, x); y /= np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])
def P(x, y, z): return np.array([x, y, TOP + z], float)


XD, BODY_Z, BODY_R, LEG_R = 0.38, 0.23, 0.06, 0.015
BELLY_Z, BACK_Z = BODY_Z - BODY_R, BODY_Z + BODY_R
FRONT_LEG_Y, REAR_LEG_Y, NECK_BASE_Y = -0.01, -0.16, 0.08
FORM_CAPS = [
    ("body", P(XD, -0.10, BODY_Z), P(XD, 0.06, BODY_Z), BODY_R), ("neck", P(XD, 0.08, 0.25), P(XD, 0.14, 0.31), 0.035),
    ("skull", P(XD, 0.155, 0.34), P(XD, 0.175, 0.342), 0.04), ("muzzle", P(XD, 0.19, 0.33), P(XD, 0.235, 0.32), 0.02),
    ("ear_l", P(XD - 0.028, 0.15, 0.37), P(XD - 0.035, 0.155, 0.395), 0.01), ("ear_r", P(XD + 0.028, 0.15, 0.37), P(XD + 0.035, 0.155, 0.395), 0.01),
    ("tail", P(XD, -0.15, 0.245), P(XD, -0.20, 0.27), 0.01),
    ("leg_fn", P(XD - 0.03, FRONT_LEG_Y, LEG_R), P(XD - 0.03, FRONT_LEG_Y, BODY_Z), LEG_R), ("leg_ff", P(XD + 0.03, FRONT_LEG_Y, LEG_R), P(XD + 0.03, FRONT_LEG_Y, BODY_Z), LEG_R),
    ("leg_rn", P(XD - 0.03, REAR_LEG_Y, LEG_R), P(XD - 0.03, REAR_LEG_Y, BODY_Z), LEG_R), ("leg_rf", P(XD + 0.03, REAR_LEG_Y, LEG_R), P(XD + 0.03, REAR_LEG_Y, BODY_Z), LEG_R),
]
EYES = (P(XD - 0.025, 0.188, 0.35), P(XD + 0.025, 0.188, 0.35))

T_P = 0.003
STRIP_W = [0.055, 0.053, 0.052, 0.052, 0.053, 0.055]
U_EDGE = np.concatenate([[0.0], np.cumsum(STRIP_W)])
U_MID = float(U_EDGE[3])
N_SEG, SEG_L = 3, 0.10
COAT_L = N_SEG * SEG_L
V0 = NECK_BASE_Y - COAT_L
PANEL_MASS, TAB_MASS, LINK_MASS = 0.006, 0.002, 0.003
HINGE_DAMP, K_ACROSS, K_ALONG = 3e-4, 0.05, 0.03
RC, XA, Y_OFF = 0.08, XD, -0.55
ZC = T_P / 2 + 0.001 - RC * math.cos(U_MID / RC)
TAB_SIZE = (0.024, 0.012, 0.022)
TAB_NAMES = [f"tab_{j}" for j in range(N_SEG)]

def arc_pt(u, r=RC):
    a = (u - U_MID) / RC
    return np.array([XA + r * math.sin(a), 0.0, TOP + ZC + r * math.cos(a)])

def _chord_R(pa, pb):
    u = (pb - pa) / np.linalg.norm(pb - pa); v = np.array([0.0, 1.0, 0.0])
    return np.column_stack([u, v, np.cross(u, v)])

def panel_index(i, j): return j * 6 + i
PANEL_NAMES = [f"coat_{j}_{i}" for j in range(N_SEG) for i in range(6)]

def panel_spawn(i, j):
    pa, pb = arc_pt(U_EDGE[i]), arc_pt(U_EDGE[i + 1])
    return (pa + pb) / 2 + np.array([0.0, Y_OFF + (j + 0.5) * SEG_L, 0.0]), _chord_R(pa, pb)

STRAPS = [("neck", COAT_L - 0.02, 0.025, 0.16, 4), ("belly", 0.135, 0.038, 0.16, 4)]
HOOK_L, HOOK_PILE, LOOP_PILE = 0.055, 0.001, 0.001
LOOP_U, LOOP_V = (0.002, 0.052), 0.040
FASTEN_OVERLAP, FASTEN_ALIGN_DEG = 0.03, 30.0
PILE_CONTACT_TOL, PRESS_MIN_DEFLECT, FASTEN_S = 0.001, 0.002, 0.25
STRAP_R = RC + T_P + 0.001

def strap_seg(s): return int(STRAPS[s][1] // SEG_L)
def link_names(s): return [f"strap_{STRAPS[s][0]}_{k}" for k in range(STRAPS[s][4])]
def link_len(s): return STRAPS[s][3] / STRAPS[s][4]
def _strap_u(s, t): return U_EDGE[-1] - t

def link_spawn(s, k):
    ln = link_len(s)
    pa, pb = arc_pt(_strap_u(s, k * ln), STRAP_R), arc_pt(_strap_u(s, (k + 1) * ln), STRAP_R)
    return (pa + pb) / 2 + np.array([0.0, Y_OFF + STRAPS[s][1], 0.0]), _chord_R(pa, pb)

class CoatPart(Box): quat = (0.0, 0.0, 0.0, 1.0)
class CoatLast(CoatPart): pass


OBJECTS, SPAWN = [], {}
for _j in range(N_SEG):
    for _i in range(6):
        _c, _R = panel_spawn(_i, _j)
        _m = PANEL_MASS - TAB_MASS if _i == 2 else PANEL_MASS
        _o = CoatPart(f"coat_{_j}_{_i}", pos=tuple(map(float, _c)), size=(STRIP_W[_i], SEG_L, T_P), mass=_m, color=SHELL_C)
        _o.quat = _quat_xyzw(_R); SPAWN[_o.name] = (_c, _R); OBJECTS.append(_o)
for _j in range(N_SEG):
    _cp, _Rp = SPAWN[f"coat_{_j}_2"]
    _ct = _cp + _Rp @ np.array([0.012, (-0.02, 0.0, 0.02)[_j], T_P / 2 + TAB_SIZE[2] / 2])
    _o = CoatPart(f"tab_{_j}", pos=tuple(map(float, _ct)), size=TAB_SIZE, mass=TAB_MASS, color=BIND_C)
    _o.quat = _quat_xyzw(_Rp); SPAWN[_o.name] = (_ct, _Rp); OBJECTS.append(_o)
for _s in range(len(STRAPS)):
    for _k in range(STRAPS[_s][4]):
        _c, _R = link_spawn(_s, _k)
        _cls = CoatLast if (_s == len(STRAPS) - 1 and _k == STRAPS[_s][4] - 1) else CoatPart
        _o = _cls(link_names(_s)[_k], pos=tuple(map(float, _c)), size=(link_len(_s), STRAPS[_s][2], T_P), mass=LINK_MASS, color=HOOK_C)
        _o.quat = _quat_xyzw(_R); SPAWN[_o.name] = (_c, _R); OBJECTS.append(_o)
LINK_NAMES = [n for s in range(len(STRAPS)) for n in link_names(s)]
ALL_NAMES = PANEL_NAMES + TAB_NAMES + LINK_NAMES
FREE = [o.name for o in OBJECTS if o.free]
_P: dict = {}


def _author(stage, parent: str) -> None:
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb
    w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    W, looks, mats = np.array([w[0], w[1], w[2]], float), f"{parent}/CoatLooks", {}
    for key, col, rough in (("form", FORM_C, 0.5), ("hook", HOOK_C, 0.95), ("loop", LOOP_C, 0.95),
                            ("eye", EYE_C, 0.3), ("lining", LINING_C, 0.9), ("bind", BIND_C, 0.7)):
        p = f"{looks}/{key}"; m = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(col), roughness=rough); m.func(p, m); mats[key] = p
    fp = f"{looks}/form_phys"; pm = sim_utils.RigidBodyMaterialCfg(static_friction=0.6, dynamic_friction=0.5); pm.func(fp, pm)
    def q4(R): q = _quat_xyzw(R); return Gf.Quatf(q[3], q[0], q[1], q[2])
    for name, a, b, r in FORM_CAPS:
        g = UsdGeom.Capsule.Define(stage, f"{parent}/dog_form/{name}")
        g.CreateRadiusAttr(float(r)); g.CreateHeightAttr(max(float(np.linalg.norm(b - a)), 1e-4)); g.CreateAxisAttr("X")
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, (a + b) / 2 + W))); g.AddOrientOp().Set(q4(_frame_x(b - a)))
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim()); cp.CreateContactOffsetAttr(0.002); cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(g.GetPath()), fp, stage=stage); bind_visual_material(str(g.GetPath()), mats["form"], stage=stage)
    for k, e in enumerate(EYES):
        s = UsdGeom.Sphere.Define(stage, f"{parent}/dog_form/eye_{k}")
        s.CreateRadiusAttr(0.007); s.AddTranslateOp().Set(Gf.Vec3d(*map(float, e + W))); bind_visual_material(str(s.GetPath()), mats["eye"], stage=stage)
    def patch(body, key, center, size, tag):
        c = UsdGeom.Cube.Define(stage, f"{parent}/{body}/{tag}")
        c.CreateSizeAttr(1.0); c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center))); c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        bind_visual_material(str(c.GetPath()), mats[key], stage=stage)
    for nm in PANEL_NAMES:
        i = int(nm.rsplit("_", 1)[1])
        patch(nm, "lining", (0, 0, -(T_P / 2 + 0.0004)), (STRIP_W[i] * 0.999, SEG_L * 0.999, 0.0008), "lining")
        if nm.startswith(f"coat_{N_SEG - 1}_"): patch(nm, "bind", (0, SEG_L / 2 - 0.008, T_P / 2 + 0.0004), (STRIP_W[i], 0.016, 0.0009), "bind")
    for s in range(len(STRAPS)):
        ln, wd, t_h0 = link_len(s), STRAPS[s][2], STRAPS[s][3] - HOOK_L
        for k, nm in enumerate(link_names(s)):
            lo, hi = max(k * ln, t_h0), (k + 1) * ln
            if hi > lo: patch(nm, "hook", ((lo + hi) / 2 - (k + 0.5) * ln, 0, -(T_P / 2 + HOOK_PILE / 2)), (hi - lo, wd * 0.95, HOOK_PILE), "hook")
            if k * ln <= t_h0 < (k + 1) * ln: patch(nm, "form", (t_h0 - (k + 0.5) * ln, 0, T_P / 2 + 0.0005), (0.002, wd * 0.96, 0.0009), "stitch")
        j = strap_seg(s)
        patch(PANEL_NAMES[panel_index(0, j)], "loop", ((LOOP_U[0] + LOOP_U[1] - STRIP_W[0]) / 2, STRAPS[s][1] - (j + 0.5) * SEG_L, T_P / 2 + LOOP_PILE / 2),
              (LOOP_U[1] - LOOP_U[0], LOOP_V, LOOP_PILE), f"loop_{s}")
    def mk_joint(cls, path, b0, b1, p0=(0, 0, 0), R0=None, p1=(0, 0, 0), R1=None, en=True):
        j = cls.Define(stage, path); j.CreateExcludeFromArticulationAttr(True); j.CreateJointEnabledAttr(en)
        j.CreateBody0Rel().SetTargets([Sdf.Path(b0)]); j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, p0))); j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0) if R0 is None else q4(R0))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, p1))); j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0) if R1 is None else q4(R1))
        return j
    def hinge(path, b0, b1, pw, axis, lim=None, k_rad=0.0):
        Rj = _frame_x(axis); c0, R0 = SPAWN[b0]; c1, R1 = SPAWN[b1]
        j = mk_joint(UsdPhysics.RevoluteJoint, path, f"{parent}/{b0}", f"{parent}/{b1}", R0.T @ (pw - c0), R0.T @ Rj, R1.T @ (pw - c1), R1.T @ Rj)
        j.CreateAxisAttr("X")
        if lim is not None: j.CreateLowerLimitAttr(-lim); j.CreateUpperLimitAttr(lim)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force"); d.CreateStiffnessAttr(k_rad * math.pi / 180); d.CreateDampingAttr(HINGE_DAMP * math.pi / 180)
        d.CreateTargetPositionAttr(0.0); d.CreateTargetVelocityAttr(0.0)
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"{parent}/{b0}")).CreateFilteredPairsRel().AddTarget(Sdf.Path(f"{parent}/{b1}"))
    hp = f"{parent}/{LINK_NAMES[-1]}"
    for j in range(N_SEG):
        cp, Rp = SPAWN[f"coat_{j}_2"]; ct, _ = SPAWN[f"tab_{j}"]
        mk_joint(UsdPhysics.FixedJoint, f"{hp}/t_{j}", f"{parent}/coat_{j}_2", f"{parent}/tab_{j}", Rp.T @ (ct - cp))
        fp_rel = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"{parent}/tab_{j}")).CreateFilteredPairsRel()
        for nb in (f"coat_{j}_2", f"coat_{j}_3"): fp_rel.AddTarget(Sdf.Path(f"{parent}/{nb}"))
    for j in range(N_SEG):
        yv = Y_OFF + (j + 0.5) * SEG_L
        for i in range(5):
            hinge(f"{hp}/h_{j}_{i}", PANEL_NAMES[panel_index(i, j)], PANEL_NAMES[panel_index(i + 1, j)], arc_pt(U_EDGE[i + 1]) + [0.0, yv, 0.0], (0, 1.0, 0), 150.0, K_ACROSS)
    for j in range(N_SEG - 1):
        yb = Y_OFF + (j + 1) * SEG_L
        for i in range(6):
            c, _ = SPAWN[PANEL_NAMES[panel_index(i, j)]]
            hinge(f"{hp}/a_{j}_{i}", PANEL_NAMES[panel_index(i, j)], PANEL_NAMES[panel_index(i, j + 1)], np.array([c[0], yb, c[2]]), (1.0, 0, 0), 120.0, K_ALONG)
    for s in range(len(STRAPS)):
        yv, prev, loop_nm = Y_OFF + STRAPS[s][1], PANEL_NAMES[panel_index(5, strap_seg(s))], PANEL_NAMES[panel_index(0, strap_seg(s))]
        for k, nm in enumerate(link_names(s)):
            pw = (arc_pt(U_EDGE[-1]) + arc_pt(U_EDGE[-1], STRAP_R)) / 2 if k == 0 else arc_pt(_strap_u(s, k * link_len(s)), STRAP_R)
            hinge(f"{hp}/s_{s}_{k}", prev, nm, pw + [0.0, yv, 0.0], (0, 1.0, 0))
            mk_joint(UsdPhysics.FixedJoint, f"{hp}/rcw_{s}_{nm}", f"{parent}/{loop_nm}", f"{parent}/{nm}", en=False)
            prev = nm
    for side in ("left", "right"):
        gpth = f"{parent}/{side.capitalize()}Arm/Geometry/base/link1/link2/link3/link4/link5/gripper"
        for nm in ALL_NAMES:
            mk_joint(UsdPhysics.SphericalJoint if nm.startswith("strap_") else UsdPhysics.FixedJoint, f"{hp}/grasp_{side}_{nm}", gpth, f"{parent}/{nm}", en=False)
    _P["parent"], _P["hp"] = parent, hp


def _spawn_last(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils.stage import get_current_stage
    prim = spawn_cuboid(prim_path, cfg, translation, orientation, **kwargs)
    _author(get_current_stage(), str(prim_path).rsplit("/", 1)[0])
    return prim

def _part_object(spec: CoatPart):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import CollisionPropertiesCfg, MassPropertiesCfg, RigidBodyPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.station import linear_rgb
    if __name__ not in sys.modules:
        import types
        mod = types.ModuleType(__name__); mod.__dict__.update(globals()); sys.modules[__name__] = mod
    kw = dict(size=tuple(spec.size), rigid_props=RigidBodyPropertiesCfg(), mass_props=MassPropertiesCfg(mass=spec.mass),
              collision_props=CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0.0),
              physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.8, dynamic_friction=0.7),
              visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.8))
    if isinstance(spec, CoatLast):
        @configclass
        class LastCfg(sim_utils.CuboidCfg): func: object = clone(_spawn_last)
        cfg = LastCfg(**kw)
    else:
        cfg = sim_utils.CuboidCfg(**kw)
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=tuple(spec.quat)))

def _patch_scene(scene) -> None:
    if not getattr(scene, "_coat_patched", False):
        orig = scene.task_object
        scene.task_object = lambda spec: _part_object(spec) if isinstance(spec, CoatPart) else orig(spec)
        scene._coat_patched = True

def _patch_emb(emb) -> None:
    cls = emb.BimanualYamEmbodiment
    if not getattr(cls.__init__, "_coat_cs", False):
        orig = cls.__init__
        def __init__(self, *a, **kw): kw["contact_sensors"] = True; orig(self, *a, **kw)
        __init__._coat_cs = True; cls.__init__ = __init__

class _Finder(importlib.abc.MetaPathFinder):
    _coat = True
    def find_spec(self, fullname, path, target=None):
        if fullname not in ("yam_arena.scene", "yam_arena.embodiment"): return None
        sys.meta_path.remove(self)
        try: spec = importlib.util.find_spec(fullname)
        finally: sys.meta_path.insert(0, self)
        if spec and spec.loader:
            orig_exec = spec.loader.exec_module
            spec.loader.exec_module = lambda m: (orig_exec(m), (_patch_scene if fullname.endswith("scene") else _patch_emb)(m))
        return spec

if "yam_arena.scene" in sys.modules: _patch_scene(sys.modules["yam_arena.scene"])
if "yam_arena.embodiment" in sys.modules: _patch_emb(sys.modules["yam_arena.embodiment"])
if not any(getattr(f, "_coat", False) for f in sys.meta_path): sys.meta_path.insert(0, _Finder())

_ENV: dict = {"env": None}
_DEBUG = bool(os.environ.get("RC_DEBUG"))

class _Episode:
    def __init__(self):
        self.prev_pts, self.still_s = None, 0.0
        self.press_s, self.fastened = [0.0] * len(STRAPS), [False] * len(STRAPS)
        self.held, self.grasp_jp = {"left": None, "right": None}, {"left": None, "right": None}


_EP = _Episode()
PAD_FORCE_MIN, PAD_HULL_TOL, PAD_CLAMP_EACH, PAD_CLAMP_SUM, OPEN_JAW_MIN = 0.05, 0.0015, 0.0004, 0.0016, 0.020


def reset_episode() -> None:
    global _EP; _EP = _Episode()
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        hp = _P.get("hp", "")
        if hp:
            for prim in get_current_stage().GetPrimAtPath(hp).GetChildren():
                nm = prim.GetName()
                if nm.startswith("rcw_") or nm.startswith("grasp_"):
                    a = prim.GetAttribute("physics:jointEnabled")
                    if a: a.Set(False)
    except Exception: pass

def _finite(a) -> bool: return a is not None and bool(np.isfinite(np.asarray(a, dtype=float)).all())

def _robots():
    env = _ENV["env"]
    if env is None: return None, None
    base = env.unwrapped
    return base, base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)

def poses():
    base, origin = _robots()
    if base is None: return None
    out = {}
    for n in ALL_NAMES:
        p = base.arena_world.get_pose_w(n)[0].detach().cpu().numpy().astype(np.float64)
        if not _finite(p): return None
        out[n] = (p[:3] - origin, _rotmat_xyzw(p[3:]))
    return out

def half_size(name):
    if name.startswith("tab_"): return np.asarray(TAB_SIZE, float) / 2
    if name.startswith("strap_"):
        s = 0 if "_neck_" in name else 1
        return np.array([link_len(s) / 2, STRAPS[s][2] / 2, T_P / 2])
    return np.array([STRIP_W[int(name.rsplit("_", 1)[1])] / 2, SEG_L / 2, T_P / 2])
def _pad_forces(base, side) -> tuple[float, float]:
    try:
        from yam_arena.runtime import contact_forces
        cf = contact_forces(base.scene.sensors[f"{side}_contacts"])
        return float(cf.get("tip_left", 0.0)), float(cf.get("tip_right", 0.0))
    except Exception: return 0.0, 0.0
def _q(R):
    from pxr import Gf
    x, y, z, w = _quat_xyzw(R)
    return Gf.Quatf(float(w), float(x), float(y), float(z))
def _set_joint(st, path, p0, R0, p1, R1):
    from pxr import Gf
    j = st.GetPrimAtPath(path)
    if not (j and j.IsValid()): return
    j.GetAttribute("physics:jointEnabled").Set(False)
    j.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, p0))); j.GetAttribute("physics:localRot0").Set(_q(R0))
    j.GetAttribute("physics:localPos1").Set(Gf.Vec3f(*map(float, p1))); j.GetAttribute("physics:localRot1").Set(_q(R1))
    j.GetAttribute("physics:jointEnabled").Set(True)
def _joint_vec(rob, target=False):
    src = rob.data.joint_pos_target if target else rob.data.joint_pos
    return (src.torch if hasattr(src, "torch") else src)[0].detach().cpu().numpy().reshape(-1)
_BOX_GRID = np.array([[x, y, z] for x in (-0.5, 0.0, 0.5) for y in (-0.5, 0.0, 0.5) for z in (-0.5, 0.0, 0.5)])
def _hull_box_dist(hulls, cb_w, Rb, sz) -> float:
    from yam_arena.analysis import hulls_to_box
    d, _ = hulls_to_box(hulls, cb_w, Rb, sz)
    pts = cb_w + (_BOX_GRID * np.asarray(sz, float)) @ Rb.T
    for _, h in hulls:
        d = min(d, float((pts @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1).min()))
    return d
def _pinch_boxes(ps): return [(nm, c, R, half_size(nm) * 2.0) for nm, (c, R) in ps.items()]

def _grasp_update(ps) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from yam_arena.analysis import world_hulls
    from yam_arena.runtime import link_poses, pad_tips_world
    base, o = _robots()
    st, hp, boxes = get_current_stage(), _P["hp"], None
    for side in ("left", "right"):
        rob = base.scene[f"{side}_robot"]
        names = list(rob.joint_names)
        j7, j8 = names.index("joint7"), names.index("joint8")
        qt, qa = _joint_vec(rob, True), _joint_vec(rob, False)
        d7, d8 = float(qt[j7] - qa[j7]), float(qt[j8] - qa[j8])
        clamping = d7 >= PAD_CLAMP_EACH and d8 >= PAD_CLAMP_EACH and (d7 + d8 >= PAD_CLAMP_SUM)
        releasing = d7 < -0.002 or d8 < -0.002 or qt[j7] <= -OPEN_JAW_MIN or qt[j8] <= -OPEN_JAW_MIN
        cur_jp = _EP.grasp_jp[side]
        j = st.GetPrimAtPath(cur_jp) if cur_jp else None
        on = bool(j and j.IsValid() and j.GetAttribute("physics:jointEnabled").Get())
        if on and releasing:
            j.GetAttribute("physics:jointEnabled").Set(False); _EP.held[side] = _EP.grasp_jp[side] = None; continue
        if on or not clamping: continue
        fl, fr = _pad_forces(base, side)
        if fl < PAD_FORCE_MIN or fr < PAD_FORCE_MIN:
            if _DEBUG: print(f"[coat] {side} clamp no 2-pad force (fL={fl:.2f}N fR={fr:.2f}N)", flush=True)
            continue
        lp = link_poses(rob); g = lp["gripper"]; gp, gR = g[:3, 3] - o, g[:3, :3]
        hL, hR = world_hulls(lp, ["tip_left"]), world_hulls(lp, ["tip_right"])
        tip_mid = np.asarray(pad_tips_world(rob), float) - o
        pad_L, pad_R = tip_mid + gR[:, 0] * float(qa[j7]), tip_mid - gR[:, 0] * float(qa[j8])
        boxes = _pinch_boxes(ps) if boxes is None else boxes
        best = None
        for nm, cb, Rb, sz in boxes:
            if float(np.linalg.norm(cb - tip_mid)) > 0.06: continue
            dL = _hull_box_dist(hL, cb + o, Rb, sz)
            if dL > PAD_HULL_TOL: continue
            dR = _hull_box_dist(hR, cb + o, Rb, sz)
            if dR > PAD_HULL_TOL or float((pad_L - cb) @ gR[:, 0]) * float((pad_R - cb) @ gR[:, 0]) >= 0.0: continue
            rank = max(dL, dR) - (0.002 if nm.startswith("strap_") else 0.0)
            if best is None or rank < best[0]:
                hs = sz / 2.0
                best = (rank, nm, dL, dR, cb + Rb @ np.clip(Rb.T @ (((pad_L + pad_R) / 2.0) - cb), -hs, hs))
        if best is None:
            if _DEBUG: print(f"[coat] {side} clamp (fL={fl:.2f}N,fR={fr:.2f}N) no 2-pad hull pinch at {np.round(tip_mid,3)}", flush=True)
            continue
        _, n, dL, dR, pin = best
        if _DEBUG: print(f"[coat] {side} TWO-PAD PINCH {n}: fL={fl:.2f}N fR={fr:.2f}N dL={dL*1000:.2f}mm dR={dR*1000:.2f}mm", flush=True)
        c, Rc = ps[n]
        jp = f"{hp}/grasp_{side}_{n}"
        if n.startswith("strap_"):
            _set_joint(st, jp, gR.T @ (pin - gp), np.eye(3), Rc.T @ (pin - c), np.eye(3))
        else:
            _set_joint(st, jp, gR.T @ (c - gp), gR.T @ Rc, np.zeros(3), np.eye(3))
        _EP.held[side], _EP.grasp_jp[side] = n, jp

HOOK_SAMPLES = 0.005

def hook_on_loop(ps, s):
    loop = PANEL_NAMES[panel_index(0, strap_seg(s))]
    cL, RL = ps[loop]
    vloc = STRAPS[s][1] - (strap_seg(s) + 0.5) * SEG_L
    u_lo, u_hi = LOOP_U[0] - STRIP_W[0] / 2, LOOP_U[1] - STRIP_W[0] / 2
    ln, total, cos_al, loop_top = link_len(s), STRAPS[s][3], math.cos(math.radians(FASTEN_ALIGN_DEG)), T_P / 2 + LOOP_PILE
    hits, links = 0, set()
    for k, nm in enumerate(link_names(s)):
        c, R = ps[nm]
        if float(R[:, 2] @ RL[:, 2]) < cos_al: continue
        for t in np.arange(k * ln + HOOK_SAMPLES / 2, (k + 1) * ln, HOOK_SAMPLES):
            if t < total - HOOK_L: continue
            loc = RL.T @ (c + R[:, 0] * (t - (k + 0.5) * ln) - R[:, 2] * (T_P / 2 + HOOK_PILE) - cL)
            if u_lo <= loc[0] <= u_hi and abs(loc[1] - vloc) <= LOOP_V / 2 and -0.004 <= loc[2] - loop_top <= PILE_CONTACT_TOL:
                hits += 1; links.add(nm)
    return hits * HOOK_SAMPLES, sorted(links)

def _pressing_loop(ps, s) -> bool:
    from yam_arena import ik
    from yam_arena.analysis import world_hulls
    from yam_arena.constants import ARM_JOINTS
    from yam_arena.runtime import link_poses
    from yam_arena.station import BASES
    base, o = _robots()
    if base is None: return False
    loop = PANEL_NAMES[panel_index(0, strap_seg(s))]
    cL, RL = ps[loop]
    vloc = STRAPS[s][1] - (strap_seg(s) + 0.5) * SEG_L
    cp = cL + RL @ np.array([(LOOP_U[0] + LOOP_U[1] - STRIP_W[0]) / 2, vloc, T_P + LOOP_PILE + HOOK_PILE / 2])
    sz = np.array([LOOP_U[1] - LOOP_U[0], LOOP_V, T_P + LOOP_PILE + HOOK_PILE])
    for side in ("left", "right"):
        rob = base.scene[f"{side}_robot"]
        if _hull_box_dist(world_hulls(link_poses(rob), ["tip_left", "tip_right"]), cp + o, RL, sz) > 0.003: continue
        idx = [list(rob.joint_names).index(j) for j in ARM_JOINTS]
        pa = ik.tcp_pose(_joint_vec(rob, False)[idx], BASES[side])[0]
        pt = ik.tcp_pose(_joint_vec(rob, True)[idx], BASES[side])[0]
        if float((pt - pa) @ (-RL[:, 2])) >= PRESS_MIN_DEFLECT: return True
    return False

def _fasten_update(ps, dt) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    for s in range(len(STRAPS)):
        if _EP.fastened[s]: continue
        ov, links = hook_on_loop(ps, s)
        _EP.press_s[s] = _EP.press_s[s] + dt if (ov >= FASTEN_OVERLAP and _pressing_loop(ps, s)) else 0.0
        if _EP.press_s[s] < FASTEN_S: continue
        st, hp, loop = get_current_stage(), _P["hp"], PANEL_NAMES[panel_index(0, strap_seg(s))]
        cL, RL = ps[loop]
        for nm in links:
            c, Rc = ps[nm]
            _set_joint(st, f"{hp}/rcw_{s}_{nm}", RL.T @ (c - cL), RL.T @ Rc, np.zeros(3), np.eye(3))
        _EP.fastened[s] = True


def _install_hooks() -> None:
    try: from yam_arena import inspect_embodiment as ie
    except Exception: return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_coat_hooked", False):
        orig = cls.reset
        def reset(self, *a, **k): reset_episode(); return orig(self, *a, **k)
        reset._coat_hooked = True; cls.reset = reset
    if not getattr(cls._task_step, "_coat_hooked", False):
        base = cls._task_step
        def _task_step(self, *a, **k):
            _ENV["env"] = self._env; out = base(self, *a, **k)
            try:
                dt, ps = 1.0 / float(self.info.control_hz), poses()
                if ps is not None and "parent" in _P: _grasp_update(ps); _fasten_update(ps, dt); _watch_still(ps, dt)
            except Exception as e: print(f"[coat] step hook failed: {e!r}", flush=True)
            return out
        _task_step._coat_hooked = True; cls._task_step = _task_step

_install_hooks()
CLEAR, PARK_SPEED, PARK_S = 0.10, 0.02, 0.5
SPINE_TOL, CENTRE_TOL = 0.05, 0.03


def gripper_points() -> list[np.ndarray]:
    base, origin = _robots()
    if base is None: return []
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world
    pts = []
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        t, tip = np.asarray(tcp_world(rob), float) - origin, np.asarray(pad_tips_world(rob), float) - origin
        ax = (tip - t) / (np.linalg.norm(tip - t) + 1e-12)
        lp = link_poses(rob); g = lp["gripper"]
        pts += [tip, t, t - 0.02 * ax, t - 0.04 * ax, t - 0.08 * ax, t - 0.134 * ax, np.asarray(g[:3, 3] + g[:3, :3] @ np.array([0.0, -0.08, 0.0]), float) - origin]
        pts += [np.asarray(lp[fk][:3, 3], float) - origin for fk in ("tip_left", "tip_right") if fk in lp]
    return pts

def _dist_form(p) -> float:
    return min(float(np.linalg.norm(p - (a + np.clip((p - a) @ (b - a) / max((b - a) @ (b - a), 1e-12), 0, 1) * (b - a)))) - r for _, a, b, r in FORM_CAPS)

def _dist_box(p, n, c, R) -> float:
    return float(np.linalg.norm(np.maximum(np.abs(R.T @ (p - c)) - half_size(n), 0.0)))

def released(ps, pts=None) -> bool:
    pts = gripper_points() if pts is None else pts
    return bool(pts and ps is not None and not any(v is not None for v in _EP.held.values())
                and all(_finite(p) and _dist_form(p) >= CLEAR and all(_dist_box(p, n, c, R) >= CLEAR for n, (c, R) in ps.items()) for p in pts))

def _watch_still(ps, dt: float) -> None:
    pts = gripper_points(); prev, _EP.prev_pts = _EP.prev_pts, pts
    if not pts or prev is None or len(prev) != len(pts): _EP.still_s = 0.0; return
    speed = max(float(np.linalg.norm(np.asarray(p) - np.asarray(q))) for p, q in zip(pts, prev)) / dt
    _EP.still_s = _EP.still_s + dt if (speed < PARK_SPEED and released(ps, pts)) else 0.0

def grippers_open() -> bool:
    base, _ = _robots()
    if base is None: return False
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]; qv, jn = _joint_vec(rob, False), list(rob.joint_names)
        if float(qv[jn.index("joint7")]) > -OPEN_JAW_MIN or float(qv[jn.index("joint8")]) > -OPEN_JAW_MIN: return False
    return True


def parked(ps) -> bool: return released(ps) and _EP.still_s >= PARK_S and grippers_open()
def spine_points(ps):
    return [ps[PANEL_NAMES[panel_index(2, j)]][0] + ps[PANEL_NAMES[panel_index(2, j)]][1][:, 0] * STRIP_W[2] / 2 for j in range(N_SEG)]
def orientation_ok(ps) -> bool:
    return all(float(ps[PANEL_NAMES[panel_index(i, j)]][1][1, 1]) >= 0.5 and float(ps[PANEL_NAMES[panel_index(i, j)]][1][2, 2]) >= 0.5
               for j in range(N_SEG) for i in (2, 3))
def on_back(ps) -> bool:
    return all(abs(p[0] - XD) <= SPINE_TOL and BACK_Z - 0.02 <= p[2] - TOP <= BACK_Z + 0.03 and -0.19 <= p[1] <= 0.15 for p in spine_points(ps))
def neck_end_y(ps) -> float:
    return float(np.mean([float((ps[PANEL_NAMES[panel_index(i, N_SEG - 1)]][0] + ps[PANEL_NAMES[panel_index(i, N_SEG - 1)]][1][:, 1] * SEG_L / 2)[1]) for i in (2, 3)]))
def centred(ps) -> bool:
    return all(abs(p[0] - XD) <= CENTRE_TOL for p in spine_points(ps)) and abs(neck_end_y(ps) - NECK_BASE_Y) <= CENTRE_TOL
def under_body(ps, s) -> bool:
    y_lo, y_hi = (FRONT_LEG_Y + LEG_R, NECK_BASE_Y + 0.06) if s == 0 else (REAR_LEG_Y + LEG_R, FRONT_LEG_Y - LEG_R)
    return any(ps[nm][0][2] - TOP < BELLY_Z + 0.005 and abs(ps[nm][0][0] - XD) <= 0.05 and y_lo <= ps[nm][0][1] <= y_hi for nm in link_names(s))
def strap_done(ps, s) -> bool:
    return _EP.fastened[s] and under_body(ps, s) and hook_on_loop(ps, s)[0] >= FASTEN_OVERLAP
def score(objects=None) -> float:
    ps = poses()
    if ps is None or any(v is not None for v in _EP.held.values()) or not on_back(ps) or not orientation_ok(ps): return 0.0
    if not centred(ps): return 0.25
    done = [strap_done(ps, k) for k in range(len(STRAPS))]
    if not any(done): return 0.5
    return 1.0 if (all(done) and parked(ps)) else 0.75

def success(objects) -> bool: return score(objects) >= 1.0
def distance(objects) -> float:
    ps = poses()
    return 1.0 if ps is None else float(np.mean([np.linalg.norm(p - P(XD, p[1], BACK_Z)) for p in spine_points(ps)]))
def progress(objects) -> dict:
    ps = poses()
    return {} if ps is None else {"orient": float(orientation_ok(ps)), "on_back": float(on_back(ps)), "centred": float(centred(ps)),
        "neck_ov": hook_on_loop(ps, 0)[0], "belly_ov": hook_on_loop(ps, 1)[0], "neck_fast": float(_EP.fastened[0]), "belly_fast": float(_EP.fastened[1]),
        "neck_under": float(under_body(ps, 0)), "belly_under": float(under_body(ps, 1)), "parked": float(parked(ps))}


VARIATION = {"coat": {"dx": 0.01, "dy": 0.015, "yaw": 0.05}}
TWIN_EXTRA = [
    {"name": "dog_form", "kind": "body", "real": "PLA small-dog form bolted to table", "note": "fixed; 11 capsules"},
    {"name": "coat_hinges", "kind": "joint", "real": "ripstop + 2 mm EVA + 3 spine tabs", "note": "18 sprung panels + 3 tabs"},
    {"name": "strap_hinges", "kind": "joint", "real": "25/38 mm black webbing", "note": "four 4 cm links/strap"},
    {"name": "hook_loop", "kind": "joint", "real": "hook-and-loop tape", "note": "welds on <=1 mm contact + >=2 mm press"},
]
