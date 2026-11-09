"""Floss between two training teeth. A dental training model stands on the table in front of the robot: two white
tooth blocks side by side (front and back) on a pink gum base. The front tooth is fixed; the back tooth sits on a
sprung slide that presses its crown ridge against the front tooth's, so the two crowns meet in a tight contact 1 cm
below their tops. Below the contact the teeth stand 1 cm apart, and a dark-pink papilla (the gum between the teeth)
fills the bottom of that gap, 3.7 cm below the contact. A white floss cord lies on the table in front of the model,
each end joined to a rigid pull tab through a short elastic link.

The robot holds a tab in each hand, lays the floss in the notch above the contact, tensions it and eases it past with
a back-and-forth along the floss (hands kept well above the gum, so it does not snap down into the papilla), curves
it against the side of each tooth in the gap and moves it up and down, then brings it back up past the contact, puts
it down and lets go. Scoring: see score() and the README (snap, touch and pull-apart latches; 0.3 / 0.6 / 1.0).
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
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"


def E(k, d):
    return type(d)(os.environ.get(k, d))


INSTRUCTION = ("A white plastic two-tooth model stands on a red block on the table. Pass the floss down between the two "
               "plastic teeth, wipe the side of each tooth in the gap, then take the floss out and let go.")

WHITE = (0.93, 0.92, 0.88)
GUM = (0.85, 0.45, 0.50)
PAPILLA_C = (0.80, 0.30, 0.38)
FLOSS_C = (0.97, 0.97, 0.97)
TAB_C = (0.15, 0.45, 0.80)

# -- the model (x forward, y left; the gap plane is x = XC) --------------------------------------------------------------
XC = 0.36  # centre plane of the gap between the teeth
GH = 0.005  # half-gap below the contact: gap faces at XC -+ 5 mm
TW, TD, TH = 0.05, 0.04, 0.055  # tooth thickness (x), depth (y), height (z)
BASE_H = 0.04  # gum base height
BASE_X0, BASE_X1 = XC - GH - TW - 0.005, XC + GH + TW + 0.015
BASE = Box("gum_base", pos=((BASE_X0 + BASE_X1) / 2, 0.0, TOP + BASE_H / 2), size=(BASE_X1 - BASE_X0, TD, BASE_H),
           color=GUM)
TOOTH_Z0 = TOP + BASE_H
FRONT = Box("tooth_front", pos=(XC - GH - TW / 2, 0.0, TOOTH_Z0 + TH / 2), size=(TW, TD, TH), color=WHITE)
BACK_X0 = XC + GH + TW / 2
TOOTH_M = 0.05
BACK = Box("tooth_back", pos=(BACK_X0, 0.0, TOOTH_Z0 + 0.001 + TH / 2), size=(TW, TD, TH), mass=TOOTH_M, color=WHITE)
RB = 0.006  # crown bulge radius (a round ridge along y on each gap face)
REST_GAP = 0.0004  # bulges 0.4 mm apart at rest: closed to a 3 mm floss
Z_C = TOOTH_Z0 + TH - 0.010  # contact height (1 cm below the tooth tops)
TOOTH_TOP = TOOTH_Z0 + TH
PAP_H = E("FL_PAPH", 0.008)
PAP_W = 2 * GH - 0.0005  # fills the gap: against the front tooth, 0.5 mm clear of the sliding back tooth
PAPILLA = Box("papilla", pos=(XC - GH + PAP_W / 2, 0.0, TOOTH_Z0 + PAP_H / 2), size=(PAP_W, TD, PAP_H), color=PAPILLA_C)
PAP_TOP = TOOTH_Z0 + PAP_H
K_T = E("FL_KT", 120.0)  # N/m slide spring
PRE_T = E("FL_PRE", 0.7)  # N preload at rest
TRAVEL = 0.008
TOOTH_MU = E("FL_TMU", 0.15)

# -- floss and tabs -----------------------------------------------------------------------------------------------------
R = 0.0015
SEG = 0.01
L_F = E("FL_LEN", 0.20)
FX0 = 0.24  # floss lies on the table along y at x 0.24
FLOSS = cable("floss", start=(FX0, -L_F / 2, TOP + R + 0.0005), yaw=math.pi / 2, length=L_F, radius=R, segment=SEG,
              cable_mass=E("FL_MASS", 0.006), bend_stiffness=0.0003, bend_damping=E("FL_BD", 0.002),
              friction=E("FL_MU", 0.3), collision_sides=E("FL_SIDES", 8), color=FLOSS_C)
_TIP0 = np.asarray(FLOSS.start, float)
_TIP1 = _TIP0 + FLOSS.length * FLOSS.direction
TAB_L, TAB_W, TAB_H = 0.025, 0.012, 0.025  # 25 mm along the floss, 12 mm thick, 25 mm tall (upright fin)
TAB_M = E("FL_TABM", 0.003)
GAP = 0.003
K_LINK = E("FL_KL", 100.0)  # N/m elastic link
S_MAX = E("FL_SMAX", 0.12)  # link travel stop (m), far beyond the 4.5 cm tear mark
F_BREAK = E("FL_BREAK", 0.0)  # N; 0 = unbreakable (the pull-apart is latched by the score at TEAR)
TAB_Z = TOP + TAB_H / 2 + 0.0005
TAB_DZ = (TOP + R + 0.0005) - TAB_Z  # the floss joins the tab's face this far below the tab centre
TAB_R = Box("tab_right", pos=(FX0, float(_TIP0[1] - GAP - TAB_L / 2), TAB_Z), size=(TAB_L, TAB_W, TAB_H),
            yaw=math.pi / 2, mass=TAB_M, color=TAB_C)
TAB_LF = Box("tab_left", pos=(FX0, float(_TIP1[1] + GAP + TAB_L / 2), TAB_Z), size=(TAB_L, TAB_W, TAB_H),
             yaw=math.pi / 2, mass=TAB_M, color=TAB_C)

MODEL = (BASE.name, FRONT.name, BACK.name, PAPILLA.name)


def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_floss_spawn", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != FLOSS.name:
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        w = UsdGeom.Xformable(stage.GetPrimAtPath(root)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        mat = f"{root}/floss_looks/enamel_physics"
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=TOOTH_MU, dynamic_friction=TOOTH_MU)
        pm.func(mat, pm)
        PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(mat)).CreateFrictionCombineModeAttr().Set("min")
        look = f"{root}/floss_looks/enamel"
        lk = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(WHITE), roughness=0.3)
        lk.func(look, lk)
        for n in (FRONT.name, BACK.name):
            for p in Usd.PrimRange(stage.GetPrimAtPath(f"{root}/{n}")):
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    bind_physics_material(str(p.GetPath()), mat, stage=stage)
        # crown bulges: a round ridge (capsule along y) on each gap face, touching at the contact
        for n, cx in ((FRONT.name, XC - RB - REST_GAP / 2), (BACK.name, XC + RB + REST_GAP / 2)):
            body = next(o for o in (FRONT, BACK) if o.name == n)
            cap = UsdGeom.Capsule.Define(stage, f"{root}/{n}/crown")
            cap.CreateAxisAttr("Y")
            cap.CreateRadiusAttr(RB)
            cap.CreateHeightAttr(TD - 2 * RB)
            cap.AddTranslateOp().Set(Gf.Vec3d(float(cx - body.pos[0]), 0.0, float(Z_C - body.pos[2])))
            UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(cap.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(cap.GetPath()), mat, stage=stage)
            bind_visual_material(str(cap.GetPath()), look, stage=stage)
            ops = [(op.GetOpName(), op.Get()) for op in UsdGeom.Xformable(stage.GetPrimAtPath(f"{root}/{n}")).GetOrderedXformOps()]
            print(f"[floss] crown on {n}: parent ops {ops}", flush=True)
        # the back tooth slides along x on a preloaded spring (lower limit = crowns 0.4 mm apart)
        back = f"{root}/{BACK.name}"
        pj = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/tooth_slide")
        pj.CreateBody1Rel().SetTargets([Sdf.Path(back)])
        pj.CreateAxisAttr("X")
        pj.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + BACK.pos[0]), float(w[1] + BACK.pos[1]), float(w[2] + BACK.pos[2])))
        pj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        pj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        pj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        pj.CreateLowerLimitAttr(0.0)
        pj.CreateUpperLimitAttr(TRAVEL)
        d = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(K_T)
        d.CreateDampingAttr(E("FL_DT", 8.0))
        d.CreateTargetPositionAttr(-PRE_T / K_T)
        pj.CreateExcludeFromArticulationAttr(True)
        # elastic links tab -> floss end, breaking above F_BREAK
        n = FLOSS.segments
        for tab, seg, sgn in ((TAB_R, 0, -1.0), (TAB_LF, n - 1, 1.0)):
            j = UsdPhysics.Joint.Define(stage, f"{prim_path}/{tab.name}_link")
            j.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{tab.name}")])
            j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_{seg}")])
            j.CreateLocalPos0Attr(Gf.Vec3f(float(-sgn * (TAB_L / 2 + GAP)), 0.0, float(TAB_DZ)))
            j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            j.CreateLocalPos1Attr(Gf.Vec3f(float(sgn * SEG / 2), 0.0, 0.0))
            j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            for dof in ("transY", "transZ", "rotX"):
                lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
                lim.CreateLowAttr(1.0)
                lim.CreateHighAttr(-1.0)
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), "transX")
            lim.CreateLowAttr(0.0 if sgn < 0 else -S_MAX)
            lim.CreateHighAttr(S_MAX if sgn < 0 else 0.0)
            dr = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "transX")
            dr.CreateTypeAttr("force")
            dr.CreateTargetPositionAttr(0.0)
            dr.CreateStiffnessAttr(K_LINK)
            dr.CreateDampingAttr(E("FL_DL", 0.5))
            if F_BREAK > 0:
                j.CreateBreakForceAttr(F_BREAK)
            j.CreateExcludeFromArticulationAttr(True)
            tp = stage.GetPrimAtPath(f"{root}/{tab.name}")
            fp = UsdPhysics.FilteredPairsAPI.Apply(tp)
            for i in range(max(0, seg - 2), min(n, seg + 3)):
                fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(f"{prim_path}/seg_{i}"))
            PhysxSchema.PhysxRigidBodyAPI.Apply(tp).CreateAngularDampingAttr(2.0)
            PhysxSchema.PhysxContactReportAPI.Apply(tp)
        print(f"[floss] spawn: K_T {K_T} pre {PRE_T} mu_tooth {TOOTH_MU} link K {K_LINK} S_MAX {S_MAX} "
              f"break {F_BREAK}", flush=True)
        return prim

    spawn._floss_spawn = True
    cablekit._spawn_cable = spawn


_install_spawn()

OBJECTS = [BASE, FRONT, BACK, PAPILLA, TAB_R, TAB_LF, FLOSS]

TWIN_EXTRA = [
    {"name": "crown_ridges", "kind": "body", "size_mm": [12, 40, 12], "colour": "white",
     "note": "round ridge (12 mm rod, along y) on each tooth's gap face, 1 cm below the top; the two ridges touch"},
    {"name": "tooth_slide", "kind": "joint", "real": "linear rail (MGN7) + compression spring, 0.7 N preload, 120 N/m",
     "note": "back tooth slides away from the front tooth by up to 8 mm"},
    {"name": "tab_links", "kind": "joint", "real": "elastic loop (rubber band) with a 4.5 N magnetic breakaway at the tab",
     "note": "each floss end joins its tab through a 100 N/m elastic link; stretched 4.5 cm counts as pulled apart"},
]


# -- env handle ---------------------------------------------------------------------------------------------------------
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_floss_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._floss_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- contact monitor: robot links and tabs x the model (per physics substep) --------------------------------------------
TOUCH_SEP = 0.001
_ROBOT_PRIMS = ("/LeftArm/", "/RightArm/")
_MON: dict = {"touch": None, "views": None, "polls": 0, "err": None}


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def _make_views(mgr) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    sv = mgr.get_physics_sim_view()
    if sv is None:
        return
    stage = get_current_stage()
    paths = [p.GetPath().pathString for p in stage.Traverse() if "/env_0/" in p.GetPath().pathString]
    robot = [p for p in paths if any(r in p for r in _ROBOT_PRIMS)
             and "PhysxContactReportAPI" in stage.GetPrimAtPath(p).GetAppliedSchemas()]
    tabs = [p for p in paths if p.endswith(f"/{TAB_R.name}") or p.endswith(f"/{TAB_LF.name}")]
    targets = [p for p in paths if stage.GetPrimAtPath(p).HasAPI(UsdPhysics.CollisionAPI)
               and any(f"/{m}/" in p + "/" for m in MODEL)]
    sensors = robot + tabs
    if not robot or len(tabs) != 2 or not targets:
        raise RuntimeError(f"floss contact monitor: robot {len(robot)}, tabs {tabs}, model {targets}")
    v = sv.create_rigid_contact_view(sensors, filter_patterns=[targets] * len(sensors),
                                     max_contact_data_count=64 * len(sensors) * len(targets))
    _MON["views"] = (v, sensors, len(targets))
    print(f"[floss] contact monitor: {len(robot)} robot links + {len(tabs)} tabs x {len(targets)} model parts",
          flush=True)


def _poll_contacts(mgr) -> None:
    try:
        if _MON["views"] is None:
            _make_views(mgr)
        if _MON["views"] is None:
            return
        v, sensors, nf = _MON["views"]
        ns = len(sensors)
        _, _, _, sep, counts, starts = v.get_contact_data(dt=mgr.get_physics_dt())
        counts = _np(counts).reshape(ns, nf)
        hit = None
        if counts.any():
            starts = _np(starts).reshape(ns, nf)
            sep = _np(sep).reshape(-1)
            for i, j in zip(*np.nonzero(counts)):
                s = int(starts[i, j])
                if float(sep[s:s + int(counts[i, j])].min()) <= TOUCH_SEP:
                    hit = sensors[i].rsplit("/", 1)[-1]
                    break
    except Exception as e:  # pragma: no cover
        if _MON["err"] is None:
            _MON["err"] = repr(e)
            print(f"[floss] contact monitor error: {e!r}", flush=True)
        return
    _MON["polls"] += 1
    if hit and _MON["touch"] is None:
        _MON["touch"] = hit
        print(f"[floss] {hit} touched the model (poll {_MON['polls']})", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_floss_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__
    base_step = mgr.step.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        try:
            import carb

            carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        except Exception:  # pragma: no cover
            pass

    def step(cls) -> None:
        base_step(cls)
        _poll_contacts(cls)

    step._floss_hooked = True
    mgr._configure_physics = classmethod(_configure_physics)
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


def _patch_physics() -> None:
    if _PHYSX_MGR in sys.modules:
        _patch_manager(sys.modules[_PHYSX_MGR])
    elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _MgrFinder())


def _install_reset() -> None:
    _patch_physics()
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_floss_reset", False):
        return
    base_reset = cls.reset

    def reset(self, *a, **k):
        _MON["views"] = None
        out = base_reset(self, *a, **k)
        _reset_episode()
        return out

    reset._floss_reset = True
    cls.reset = reset


# -- episode state (every violation latches until the next reset) --------------------------------------------------------
_ST: dict = {}


def _face_state() -> dict:
    return {"dir": 0, "start": None, "ext": None, "half": 0}


def _reset_episode() -> None:
    _MON["touch"] = None
    _ST.clear()
    _ST.update(nan=False, torn=False, snap=False, opened=False, passed=False, prev=None, open_max=0.0,
               faces={"front": _face_state(), "back": _face_state()}, zmin_gap=9.0,
               wiped={"front": False, "back": False})


_reset_episode()
_install_reset()


# -- geometry -----------------------------------------------------------------------------------------------------------
SNAP_V = 0.12  # m/s
SNAP_T = 0.01  # a snap needs the floss under tension: a link stretched 1 cm (about 1 N); slack floss falling is harmless
DT = 0.05  # control step
STROKE = 0.008
HALF_STROKES = 4  # 2 up-and-down strokes
WIPE_D = 0.0005  # floss surface within 0.5 mm of the face (touching it)
INNER = 0.004  # stay this far inside the tooth's depth for "in the gap"


def _xaxis(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)])


def _zaxis(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def _tab_c(objects, tab) -> np.ndarray:
    return np.asarray(objects[tab.name].pos, dtype=float)


_D0 = math.hypot(TAB_L / 2 + GAP, TAB_DZ)  # tab centre to floss tip at rest


def link_stretch(objects) -> tuple[float, float]:
    """Elastic-link stretch (m) at the right (start) and left (end) tab: tab centre to floss tip, minus its rest
    length (independent of how the tab is turned in the hand)."""
    c = objects[FLOSS.name]
    return (float(np.linalg.norm(np.asarray(c.start_tip, float) - _tab_c(objects, TAB_R))) - _D0,
            float(np.linalg.norm(np.asarray(c.end_tip, float) - _tab_c(objects, TAB_LF))) - _D0)


TEAR = 0.045  # link stretched to 4.5 cm (4.5 N): pulled apart


BACK_C0 = (XC + REST_GAP / 2 + BACK_X0 + TW / 2) / 2  # reported centre of the back tooth (box + crown) at rest


def opening(objects) -> float:
    """How far the back tooth has been pushed away from the front tooth (m)."""
    return float(objects[BACK.name].pos[0] - BACK_C0)


def back_face(objects) -> float:
    return BACK_X0 - TW / 2 + opening(objects)


def _dense(poly, step=0.002) -> np.ndarray:
    poly = np.asarray(poly, float)
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
    return np.asarray(out)


def _between(objects, pts) -> np.ndarray:
    """Points inside the space between the two teeth (top view, inside the tooth depth)."""
    return ((np.abs(pts[:, 1]) <= TD / 2 - INNER) & (pts[:, 0] >= XC - GH - 0.001)
            & (pts[:, 0] <= back_face(objects) + 0.001))


def below_contact(objects, pts) -> np.ndarray:
    return _between(objects, pts) & (pts[:, 2] <= Z_C - 0.004)


def in_model_gap(objects) -> bool:
    """Any floss point between the teeth below their tops (the floss is not out)."""
    pts = _dense(objects[FLOSS.name].polyline())
    return bool((_between(objects, pts) & (pts[:, 2] <= TOOTH_TOP + R)).any())


def _finite(objects) -> bool:
    c = objects[FLOSS.name]
    return bool(np.isfinite(c.points).all() and np.abs(c.points).max() < 5.0 and np.isfinite(_tab_c(objects, TAB_R)).all()
                and np.isfinite(_tab_c(objects, TAB_LF)).all() and np.isfinite(objects[BACK.name].pos).all())


def _stroke(fs: dict, h: float | None) -> None:
    """Half-strokes of the floss against one face: monotone runs >= STROKE between reversals (2 mm hysteresis)."""
    if h is None:  # floss came off the face: a run already long enough still counts
        if fs["dir"] != 0 and fs["start"] is not None and abs(fs["ext"] - fs["start"]) >= STROKE:
            fs["half"] += 1
        fs["dir"], fs["start"], fs["ext"] = 0, None, None
        return
    if fs["start"] is None:
        fs["start"] = fs["ext"] = h
        return
    if fs["dir"] == 0:
        if abs(h - fs["start"]) > 0.002:
            fs["dir"] = 1 if h > fs["start"] else -1
            fs["ext"] = h
    elif (h - fs["ext"]) * fs["dir"] > 0:
        fs["ext"] = h
    elif abs(h - fs["ext"]) > 0.002:
        if abs(fs["ext"] - fs["start"]) >= STROKE:
            fs["half"] += 1
        fs["start"] = fs["ext"]
        fs["dir"] = -fs["dir"]
        fs["ext"] = h


def half_strokes(name: str) -> int:
    fs = _ST["faces"][name]
    cur = 1 if fs["dir"] != 0 and fs["start"] is not None and abs(fs["ext"] - fs["start"]) >= STROKE else 0
    return fs["half"] + cur


def wiped(name: str) -> bool:
    return bool(_ST["wiped"][name])


def _face_height(objects, pts, name) -> float | None:
    """Mean height of the floss lying against one gap face (surface within 2 mm), below the contact; None if not."""
    m = below_contact(objects, pts) & (pts[:, 2] >= PAP_TOP)
    if name == "front":
        d = pts[:, 0] - (XC - GH)
    else:
        d = back_face(objects) - pts[:, 0]
    m &= d <= R + WIPE_D
    if m.sum() < 5:  # at least 1 cm of floss against the face
        return None
    return float(pts[m, 2].mean())


def _track(objects) -> None:
    st = _ST
    if not _finite(objects):
        if not st["nan"]:
            print("[floss] non-finite state", flush=True)
        st["nan"] = True
        return
    c = objects[FLOSS.name]
    pts_seg = np.asarray(c.points, float)
    prev = st["prev"]
    if prev is not None and np.array_equal(prev, pts_seg):
        return  # scored twice in one control step
    st["prev"] = pts_seg.copy()
    if max(link_stretch(objects)) >= TEAR:
        if not st["torn"]:
            print(f"[floss] tabs pulled apart past the link (stretch {link_stretch(objects)})", flush=True)
        st["torn"] = True
    op = opening(objects)
    st["open_max"] = max(st["open_max"], op)
    if op >= 0.0015:
        st["opened"] = True
    pts = _dense(c.polyline())
    bc = below_contact(objects, pts)
    if bc.any():
        st["zmin_gap"] = min(st["zmin_gap"], float(pts[bc, 2].min()))
        if st["opened"] and not st["passed"]:
            st["passed"] = True
            print(f"[floss] floss passed below the contact (opening max {st['open_max'] * 1000:.1f} mm)", flush=True)
    # snap: a floss segment reaches the gum zone moving down faster than SNAP_V while the floss is under tension
    if prev is not None:
        foot = (np.abs(pts_seg[:, 1]) <= TD / 2) & (np.abs(pts_seg[:, 0] - XC) <= GH + max(op, 0.0) + 0.002)
        drop = prev[:, 2] - pts_seg[:, 2]
        taut = max(link_stretch(objects)) >= SNAP_T
        hit = foot & (pts_seg[:, 2] <= PAP_TOP + R + 0.003) & (drop > SNAP_V * DT) & taut
        if hit.any() and not st["snap"]:
            st["snap"] = True
            print(f"[floss] floss snapped into the gum (drop {drop[hit].max() * 1000:.1f} mm in one step)", flush=True)
    for name in ("front", "back"):
        _stroke(st["faces"][name], _face_height(objects, pts, name))
        if half_strokes(name) >= HALF_STROKES and not st["wiped"][name]:
            st["wiped"][name] = True
            print(f"[floss] {name} tooth's gap face wiped", flush=True)


RELEASE_DIST = 0.03


def released(objects) -> bool:
    c = objects[FLOSS.name]
    pts = np.vstack([_dense(c.polyline()), _tab_c(objects, TAB_R)[None], _tab_c(objects, TAB_LF)[None]])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def violated() -> str | None:
    st = _ST
    if st["nan"]:
        return "non-finite state"
    if st["torn"]:
        return "tabs pulled apart past the link"
    if st["snap"]:
        return "snapped into the gum"
    if _MON["touch"]:
        return f"{_MON['touch']} touched the model"
    return None


def score(objects) -> float:
    _track(objects)
    if violated() or not _ST["passed"] or not released(objects):
        return 0.0
    if wiped("front") and wiped("back"):
        return 1.0 if not in_model_gap(objects) else 0.6
    return 0.3


def success(objects) -> bool:
    if score(objects) < 1.0:
        return False
    return bool(objects[FLOSS.name].resting(max_speed=0.02))


def distance(objects) -> float:
    if violated():
        return 1.0
    return 1.0 - (0.3 * _ST["passed"] + 0.15 * min(half_strokes("front"), 4) / 4
                  + 0.15 * min(half_strokes("back"), 4) / 4 + 0.4 * (not in_model_gap(objects)))


def progress(objects) -> dict:
    _track(objects)
    return {"contact_opened": int(_ST["opened"]), "passed_contact": int(_ST["passed"]),
            "front_face_wiped": 1.0 if wiped("front") else min(half_strokes("front"), HALF_STROKES) / HALF_STROKES,
            "back_face_wiped": 1.0 if wiped("back") else min(half_strokes("back"), HALF_STROKES) / HALF_STROKES,
            "floss_out": int(not in_model_gap(objects)), "released": int(released(objects)),
            "no_violation": int(violated() is None)}


VARIATION = {"cable:floss": {"dx": 0.01, "dy": 0.01, "yaw": 0.05}, "obj:tab_right": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},
             "obj:tab_left": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}


# -- demo ---------------------------------------------------------------------------------------------------------------
GRIP_OPEN = -0.016
TAB_DX = 0.003
GDZ = 0.008  # TCP above the tab centre when pinched
ATT = GDZ - TAB_DZ  # TCP height above the floss attachment point


def demo_script(ctx):
    """Both arms take a tab, carry the floss over the model and lower it into the notch above the contact with the
    hands just below the contact on either side. A short back-and-forth along the floss eases it past (closed loop on
    the floss height in the gap). The hands then curve the floss against the front tooth's gap face and move it up and
    down twice, then the back tooth's, bring it back up out of the gap with the same back-and-forth, carry it to the
    front, put it down and let go. FLOSS_DEMO selects a test variant (see README)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    mode = os.environ.get("FLOSS_DEMO", "")
    YL, YR = -math.pi / 2, math.pi / 2  # wrist cams look in towards the teeth

    def log(tag):
        st = ctx.states()
        if not _finite(st):
            print(f"[demo t={ctx.t:5.1f}] {tag}: non-finite state viol={violated()}", flush=True)
            return
        sa, sb = link_stretch(st)
        pts = _dense(st[FLOSS.name].polyline())
        bc = below_contact(st, pts)
        btw = _between(st, pts)
        zg = float(pts[btw, 2].min() - TOP) if btw.any() else -1
        print(f"[demo t={ctx.t:5.1f}] {tag}: stretch=({sa:.3f},{sb:.3f}) open={opening(st) * 1000:.1f}mm "
              f"zgap={zg:.4f} below={int(bc.sum())} half=({half_strokes('front')},{half_strokes('back')}) "
              f"passed={int(_ST['passed'])} out={int(not in_model_gap(st))} viol={violated()} score={score(st):.2f}",
              flush=True)
        if os.environ.get("FLOSS_DEBUG"):
            print(f"   tabR {np.round(_tab_c(st, TAB_R) - [0, 0, TOP], 3)} tabL {np.round(_tab_c(st, TAB_LF) - [0, 0, TOP], 3)}"
                  f" tcpL {np.round(L.tcp() - [0, 0, TOP], 3)} tcpR {np.round(Rt.tcp() - [0, 0, TOP], 3)}", flush=True)
            print(f"   floss z {np.round(st[FLOSS.name].points[:, 2] - TOP, 3).tolist()}", flush=True)
            print(f"   back x {st[BACK.name].pos[0]:.5f} open_max {_ST['open_max']*1000:.2f}mm", flush=True)
            print(f"   floss y {np.round(st[FLOSS.name].points[:, 1], 3).tolist()}", flush=True)

    def take(arm, tab, yaw):
        c = _tab_c(ctx.states(), tab)
        yield from arm.move((c[0], c[1], TOP + 0.08), yaw=yaw, finger=GRIP_OPEN, duration=1.5)
        yield from arm.move((c[0], c[1], c[2] + GDZ), yaw=yaw, finger=GRIP_OPEN, duration=0.8)
        yield from arm.grip(FINGER_CLOSED)
        yield from arm.wait(0.3)

    def hands(x, y, za, dur, yl=0.0):
        """Both hands: floss attachment points at (x, +-y, za) (za above the table), shifted along y by yl. The pinched
        tabs ride about 3 mm beyond the TCP in x, so the TCPs aim 3 mm short."""
        x = x - TAB_DX
        yield from ctx.both(L.move((x, y + yl, TOP + za + ATT), yaw=YL, duration=dur),
                            Rt.move((x, -y + yl, TOP + za + ATT), yaw=YR, duration=dur))

    def release_and_retreat():
        """Open, lift 4 cm straight up, then sweep back towards each arm's own base and aside (clear of the top view)."""
        yield from ctx.both(L.grip(GRIP_OPEN, 0.4), Rt.grip(GRIP_OPEN, 0.4))
        pl, pr = L.tcp(), Rt.tcp()
        yield from ctx.both(L.move((pl[0], pl[1], pl[2] + 0.04), yaw=YL, duration=0.4),
                            Rt.move((pr[0], pr[1], pr[2] + 0.04), yaw=YR, duration=0.4))
        yield from ctx.both(L.move((0.12, 0.27, TOP + 0.16), yaw=YL, finger=FINGER_OPEN, duration=1.0),
                            Rt.move((0.12, -0.27, TOP + 0.16), yaw=YR, finger=FINGER_OPEN, duration=1.0))
        for _ in range(4):
            yield from L.wait(0.5)
        log("end")

    def slacken_in_gap(ys, z0):
        """Slacken the floss slowly (hands in) where it is, so it settles slack between the teeth, before letting go."""
        ysl = y0 - 0.015
        for i in range(1, 9):
            yield from hands(XC, ys + (ysl - ys) * i / 8, z0, 0.4)
        yield from hands(XC, ysl, z0, 1.0)

    log("start")
    if os.environ.get("FLOSS_DEBUG"):
        st0 = ctx.states()
        print(f"   back tooth pos {np.round(np.asarray(st0[BACK.name].pos) - [0, 0, TOP], 4)} front {np.round(np.asarray(st0[FRONT.name].pos) - [0, 0, TOP], 4)}", flush=True)
    if mode == "none":
        yield from L.wait(4.0)
        log("end")
        return
    if mode == "onearm":  # right arm alone: drapes the floss across the notch by one tab and works it down
        yield from take(Rt, TAB_R, YR)
        yield from Rt.move((FX0, -0.05, TOP + 0.16), yaw=YR, duration=1.5)
        yield from Rt.move((XC, 0.06, TOP + 0.16), yaw=YR, duration=1.5)
        yield from Rt.move((XC, -0.10, TOP + 0.12), yaw=YR, duration=2.0)
        log("one-arm draped")
        yield from Rt.move((XC, -0.13, TOP + 0.065 + ATT), yaw=YR, duration=1.5)
        for k in range(16):
            yield from Rt.move((XC, -0.13 - 0.004 * k + (0.012 if k % 2 else -0.012), TOP + 0.065 + ATT), yaw=YR,
                               duration=0.4)
        log("one-arm worked")
        yield from Rt.move((XC, -0.13, TOP + 0.12), yaw=YR, duration=1.0)
        yield from Rt.move((FX0, -0.12, TOP + 0.05), yaw=YR, duration=1.5)
        yield from Rt.grip(GRIP_OPEN, 0.4)
        yield from Rt.move((0.12, -0.27, TOP + 0.16), yaw=YR, finger=FINGER_OPEN, duration=1.0)
        for _ in range(4):
            yield from L.wait(0.5)
        log("end")
        return
    yield from ctx.both(take(L, TAB_LF, YL), take(Rt, TAB_R, YR))
    log("grasped")
    y0 = L_F / 2 + GAP + TAB_L / 2
    yield from hands(FX0, y0 - 0.005, 0.13 - ATT + 0.0185, 1.5)
    yield from hands(XC, y0 - 0.005, 0.13 - ATT + 0.0185, 2.0)
    log("over the model")
    za = E("FL_ZA", 0.065)
    ys = E("FL_YS", 0.105)
    yield from hands(XC, ys, (Z_C - TOP) + 0.012, 1.5)
    log("on the notch")
    if mode == "yank":  # pulls the hands apart with the floss over the contact
        yield from hands(XC, 0.20, za, 1.0)
        yield from hands(XC, 0.12, za + 0.04, 1.0)
        log("yanked")
    if mode == "fingers":  # right hand lets go and presses the floss into the contact with its fingertips
        yield from hands(XC, 0.125, (Z_C - TOP) + 0.004, 1.0)
        yield from Rt.grip(GRIP_OPEN, 0.4)
        yield from Rt.move((XC, -0.11, TOP + 0.13), yaw=0.0, finger=FINGER_CLOSED, duration=1.0)
        yield from Rt.move((XC, -0.005, TOP + 0.13), yaw=0.0, duration=1.0)
        yield from Rt.move((XC, -0.005, Z_C + 0.004 + 0.013), yaw=0.0, duration=1.5)
        yield from Rt.move((XC, -0.005, Z_C - 0.004 + 0.013), yaw=0.0, duration=1.0)
        log("pressed with fingertips")
        yield from Rt.move((XC, -0.11, TOP + 0.13), yaw=0.0, duration=1.0)
        yield from L.move((FX0, 0.12, TOP + 0.05), yaw=YL, duration=1.5)
        yield from release_and_retreat()
        return
    yield from hands(XC, ys, za, 1.5)
    log("hands below the contact")
    s_t = E("FL_ST", 0.03)
    for _ in range(80):  # take up tension closed loop on the link stretch; ease off the moment the floss is past
        st = ctx.states()
        sa, sb = link_stretch(st)
        if _ST["passed"]:
            break
        if max(sa, sb) >= s_t or violated():
            break
        ys += 0.0007
        yield from hands(XC, ys, za, 0.05)
        if os.environ.get("FLOSS_DEBUG"):
            print(f"   take-up ys {ys:.3f} stretch ({sa:.3f},{sb:.3f}) open {opening(st)*1000:.1f}", flush=True)
    log("taut")
    if mode == "snap":  # taut floss forced through by shoving both hands 3 cm down: it pops past and hits the gum
        yield from hands(XC, ys, za - 0.03, 0.15)
        log("forced through")
        yield from L.wait(1.0)
        yield from hands(XC, ys, 0.07, 1.0)
    amp = E("FL_SAW", 0.012)
    k = 0
    while not _ST["passed"] and k < E("FL_NSAW", 16):
        yl = amp if k % 2 == 0 else -amp
        yield from hands(XC, ys, za, 0.4, yl=yl)
        k += 1
        log(f"saw {k}")
    zw0, zw1 = E("FL_ZW0", 0.054), E("FL_ZW1", 0.076)
    yield from hands(XC, ys, zw1, 0.4)
    log("past the contact" if _ST["passed"] else "NOT past")
    if mode == "pass":  # partial: past the contact, then lays the floss on the gum and lets go (no wiping)
        yield from slacken_in_gap(ys, zw1)
        yield from release_and_retreat()
        return
    yield from L.wait(1.0)
    log("settled")
    dxw = E("FL_DXW", 0.02)
    for face, sx in (("front", -1), ("back", 1)) if mode != "front" else (("front", -1),):
        x = XC + sx * dxw
        yield from hands(x, ys, zw1, 1.0)
        log(f"against the {face} face")
        for _ in range(int(E("FL_NSTROKE", 2))):
            yield from hands(x, ys, zw0, 1.0)
            log(f"{face} down")
            yield from hands(x, ys, zw1, 1.0)
            log(f"{face} up")
        yield from hands(XC, ys, zw1, 0.8)
    log("wiped")
    if mode == "leave":  # partial: wiped both faces, lays the floss on the gum and lets go with it between the teeth
        yield from slacken_in_gap(ys, zw1)
        yield from release_and_retreat()
        return
    # lift out: raise the hands past the contact with the same back-and-forth, closed loop on the floss in the gap
    zo = zw1
    k = 0
    while k < 40:
        st = ctx.states()
        if not below_contact(st, _dense(st[FLOSS.name].polyline())).any():
            break
        zo = min(zo + 0.003, (Z_C - TOP) + 0.03)
        yl = amp if k % 2 == 0 else -amp
        yield from hands(XC, ys, zo, 0.4, yl=yl)
        k += 1
    log("lifted past the contact")
    yield from hands(XC, ys, (TOOTH_TOP - TOP) + 0.06, 0.8)  # up clear of the tooth tops
    yield from hands(XC, y0 - 0.015, (TOOTH_TOP - TOP) + 0.06, 0.6)  # slacken: never carry the floss straight and taut
    log("out of the gap")
    yield from hands(FX0, y0 - 0.015, (TOOTH_TOP - TOP) + 0.06, 1.5)
    yield from hands(FX0, y0 - 0.005, TAB_H / 2 + 0.002 + GDZ - ATT, 1.2)  # tabs just above the table
    log("set down")
    if mode == "hold":
        yield from L.wait(3.0)
        log("end")
        return
    yield from release_and_retreat()
