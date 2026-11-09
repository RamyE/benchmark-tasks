"""A model recurve bow on the station table: string it with its bow stringer, then let go.

A wooden riser (3 x 9.2 x 2.5 cm) is fixed on the table at x 34 cm, along y. Two black limbs (0.6 x 15 x 2.5 cm) are
pinned at the riser ends about vertical axes and sprung towards their unstrung rest (0.15 N m/rad, 3 deg preload); each
limb tip has a nock lip on its back (far) side. The white bowstring (4 mm, 35.4 cm) has its bottom eye (red) welded in the
right-hand limb's nock; its top eye (red, rigid) rides along the left-hand limb's top edge, 1.8 cm short of the nock,
where the unbent bow leaves it (the string is straight between the eyes). The bow stringer is two blue 6 mm cords from
cups on the limb tips' belly side to a yellow handle block (5 x 1.6 x 3.5 cm) lying on the table at x 26.5, y 0.

Stringing: draw the handle towards the robot so the stringer bends both limbs, slide the top eye up into the nock
(over its lip) and let go of it, then ease the handle back so the string takes the limbs, and let go of the handle.
The limbs may only be bent with the stringer: touching a limb, or pulling the eye up the limb while the stringer is
not drawn, scores 0.

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

import cablekit  # noqa: E402,F401
import numpy as np  # noqa: E402
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "String the bow with its bow stringer, then let go."

WOOD = (0.55, 0.36, 0.20)
LIMB_C = (0.12, 0.12, 0.14)
STR_C = (0.92, 0.92, 0.88)
EYE_C = (0.85, 0.15, 0.12)
STRINGER_C = (0.15, 0.35, 0.80)
STIRRUP_C = (0.95, 0.75, 0.10)
GREY = (0.6, 0.6, 0.6)

# -- bow geometry (world, m) ---------------------------------------------------------------------------------------
RX = 0.34                      # riser centre x
RISER_GAP = 0.004              # riser ends 4 mm short of the limb pivots, so the limbs swing clear of it
RISER = Box("riser", pos=(RX, 0.0, TOP + 0.0125), size=(0.03, 0.10 - 2 * RISER_GAP, 0.025), color=WOOD)
Y0 = 0.05                      # limb pivots at the riser ends
LT, LL, LH = 0.006, 0.15, 0.025  # limb thickness (x), length (along the limb), height (z)
PX = RX - 0.015 + LT / 2       # limb centre x: belly face flush with the riser's belly face
LZ = TOP + 0.005 + LH / 2      # limb centre z (5 mm above the table)
LIMB_TOP_Z = LZ + LH / 2
LIMB_MASS = 0.02
LIMB_L = Box("limb_top", pos=(PX, Y0 + LL / 2, LZ), size=(LT, LL, LH), mass=LIMB_MASS, color=LIMB_C)
LIMB_R = Box("limb_bottom", pos=(PX, -(Y0 + LL / 2), LZ), size=(LT, LL, LH), mass=LIMB_MASS, color=LIMB_C)
K_LIMB = 0.15                  # N m / rad
C_LIMB = 0.006                 # N m s / rad
PRELOAD_DEG = 3.0
FLEX_MAX_DEG = 50.0

# eyes: rigid printed loops riding on the limbs' top edge, on the back (+x) side
EX, EL, EH = 0.010, 0.006, 0.024           # eye width (x), length (along limb), height
EDX = LT / 2 + 0.001 + EX / 2              # eye centre offset from the limb centre (+x, back side)
EZ = TOP + 0.022 + EH / 2                  # eye centre z: bottom 22 mm above the table (overlaps the lip)
S_SEAT = LL - 0.012                        # eye centre along the limb when seated in the nock
LIP_LEN, LIP_X = 0.004, 0.006
LIP_S = S_SEAT - EL / 2 - LIP_LEN / 2      # lip centre along the limb (eye's lower face rests on it)
LIP_Z0, LIP_Z1 = TOP + 0.012, TOP + 0.027  # lip spans these heights (eye bottom TOP+0.022: 5 mm overlap)
S_HI = S_SEAT + 0.005                      # slider end stop (tip)
S_LO = 0.02
LIFT_MAX = 0.012
THETA_B = math.radians(24.0)               # design brace angle


def _limb_pt(sign, local, th=0.0):
    """World (x, y) of a point given in a limb's local frame (x across, y = s along the limb from the pivot)."""
    a = sign * th
    lx, s = local
    ly = sign * s
    c, sn = math.cos(a), math.sin(a)
    return np.array([PX + c * lx - sn * ly, sign * Y0 + sn * lx + c * ly])


# string: from the bottom eye's +y face to the top eye's -y face; length set so the brace angle is THETA_B
SR = 0.002
_face_b = _limb_pt(1, (EDX, S_SEAT - EL / 2), THETA_B)
STRING_LEN = float(2 * _face_b[1])
S0 = STRING_LEN - (Y0 + S_SEAT - EL / 2) - Y0 + EL / 2   # top eye centre along its limb at rest (string straight)
EYE_TOP = Box("top_eye", pos=(PX + EDX, Y0 + S0, EZ), size=(EX, EL, EH), mass=0.004, color=EYE_C)
EYE_BOT = Box("bottom_eye", pos=(PX + EDX, -(Y0 + S_SEAT), EZ), size=(EX, EL, EH), mass=0.004, color=EYE_C)
LIP_T = Box("nock_lip_top", pos=(PX + LT / 2 + LIP_X / 2, Y0 + LIP_S, (LIP_Z0 + LIP_Z1) / 2),
            size=(LIP_X, LIP_LEN, LIP_Z1 - LIP_Z0), mass=0.002, color=LIMB_C)
LIP_B = Box("nock_lip_bottom", pos=(PX + LT / 2 + LIP_X / 2, -(Y0 + LIP_S), (LIP_Z0 + LIP_Z1) / 2),
            size=(LIP_X, LIP_LEN, LIP_Z1 - LIP_Z0), mass=0.002, color=LIMB_C)
STRING = cable("bowstring", start=(PX + EDX, -(Y0 + S_SEAT) + EL / 2, EZ), yaw=math.pi / 2, length=STRING_LEN,
               radius=SR, color=STR_C, bend_stiffness=0.003, bend_damping=0.0005, cable_mass=0.006)

# stringer: two cord halves from cups at the limb tips to the stirrup's front face
CR = 0.003
CUP_LOCAL = (-(LT / 2 + 0.004), LL - 0.003)
ST_SIZE = (0.05, 0.016, 0.035)
ST_X0 = 0.265
STIRRUP = Box("stirrup", pos=(ST_X0, 0.0, TOP + ST_SIZE[2] / 2), size=ST_SIZE, mass=0.02, color=STIRRUP_C)
CZ = LZ
_st_att = {1: np.array([ST_X0 + ST_SIZE[0] / 2 + 0.0005, 0.005]), -1: np.array([ST_X0 + ST_SIZE[0] / 2 + 0.0005, -0.005])}


def _stringer_half(sign):
    a = _limb_pt(sign, CUP_LOCAL)
    b = _st_att[sign]
    d = b - a
    return cable("stringer_top" if sign > 0 else "stringer_bottom", start=(float(a[0]), float(a[1]), CZ),
                 yaw=math.atan2(d[1], d[0]), length=float(np.linalg.norm(d)), radius=CR, color=STRINGER_C,
                 bend_stiffness=0.004, bend_damping=0.0005, cable_mass=0.008)


STR_L = _stringer_half(1)
STR_R = _stringer_half(-1)

OBJECTS = [RISER, LIMB_L, LIMB_R, EYE_TOP, EYE_BOT, LIP_T, LIP_B, STIRRUP, STRING, STR_L, STR_R]
LAST_CABLE = STR_R.name
SOFT_K = 300.0                 # N/m per tie axis (stringer cords)
STRING_K = 1200.0              # N/m per tie axis (bowstring: stiffer, so the brace holds near its design angle)
SOFT_C = 2.0
SOFT_MAX = 0.03


def _install_spawn_hook() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_bow_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != LAST_CABLE:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        P = {o.name: f"{parent}/{o.name}" for o in OBJECTS}
        w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        q0 = Gf.Quatf(1.0, 0.0, 0.0, 0.0)

        def v(a_):
            return Gf.Vec3f(*(float(t) for t in a_))

        def qz(yaw):
            return Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))

        def lock(j, dofs):
            for dof in dofs:
                lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
                lim.CreateLowAttr(1.0)
                lim.CreateHighAttr(-1.0)

        # limb pivots: revolute about z with a spring (angular drive) preloaded against the rest stop
        for limb, sign in ((LIMB_L, 1), (LIMB_R, -1)):
            h = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/pivot_{limb.name}")
            h.CreateBody1Rel().SetTargets([Sdf.Path(P[limb.name])])
            h.CreateAxisAttr("Z")
            h.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + PX), float(w[1] + sign * Y0), float(w[2] + LZ)))
            h.CreateLocalRot0Attr(q0)
            h.CreateLocalPos1Attr(Gf.Vec3f(0.0, float(-sign * LL / 2), 0.0))
            h.CreateLocalRot1Attr(q0)
            if sign > 0:
                h.CreateLowerLimitAttr(0.0)
                h.CreateUpperLimitAttr(FLEX_MAX_DEG)
            else:
                h.CreateLowerLimitAttr(-FLEX_MAX_DEG)
                h.CreateUpperLimitAttr(0.0)
            drv = UsdPhysics.DriveAPI.Apply(h.GetPrim(), "angular")
            drv.CreateTypeAttr("force")
            drv.CreateStiffnessAttr(K_LIMB * math.pi / 180)
            drv.CreateDampingAttr(C_LIMB * math.pi / 180)
            drv.CreateTargetPositionAttr(-sign * PRELOAD_DEG)
            h.CreateExcludeFromArticulationAttr(True)
        # nock lips welded to the limbs, bottom eye welded in its nock
        for body, limb in ((LIP_T, LIMB_L), (LIP_B, LIMB_R), (EYE_BOT, LIMB_R)):
            j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/weld_{body.name}")
            j.CreateBody0Rel().SetTargets([Sdf.Path(P[limb.name])])
            j.CreateBody1Rel().SetTargets([Sdf.Path(P[body.name])])
            j.CreateLocalPos0Attr(v(np.array(body.pos) - np.array(limb.pos)))
            j.CreateLocalRot0Attr(q0)
            j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            j.CreateLocalRot1Attr(q0)
            j.CreateExcludeFromArticulationAttr(True)
        # top eye: slides along the limb's top edge (joint x = limb axis), may lift up to LIFT_MAX, no rotation
        j = UsdPhysics.Joint.Define(stage, f"{prim_path}/eye_slider")
        j.CreateBody0Rel().SetTargets([Sdf.Path(P[LIMB_L.name])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(P[EYE_TOP.name])])
        j.CreateLocalPos0Attr(v(np.array(EYE_TOP.pos) - np.array(LIMB_L.pos)))
        j.CreateLocalRot0Attr(qz(math.pi / 2))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(qz(math.pi / 2))
        lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), "transX")
        lim.CreateLowAttr(float(S_LO - S0))
        lim.CreateHighAttr(float(S_HI - S0))
        lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), "transZ")
        lim.CreateLowAttr(0.0)
        lim.CreateHighAttr(LIFT_MAX)
        lock(j, ("transY", "rotX", "rotY", "rotZ"))
        j.CreateExcludeFromArticulationAttr(True)

        # cord ends: ball joints (translations locked)
        def ball(name, body, local0, cab, end, k=SOFT_K):
            j = UsdPhysics.Joint.Define(stage, f"{prim_path}/{name}")
            j.CreateBody0Rel().SetTargets([Sdf.Path(P[body.name])])
            seg = f"{parent}/{cab.name}/seg_{0 if end == 'start' else cab.segments - 1}"
            j.CreateBody1Rel().SetTargets([Sdf.Path(seg)])
            j.CreateLocalPos0Attr(v(local0))
            j.CreateLocalRot0Attr(qz(cab.yaw))
            j.CreateLocalPos1Attr(Gf.Vec3f(float((-1 if end == "start" else 1) * cab.seg_len / 2), 0.0, 0.0))
            j.CreateLocalRot1Attr(q0)
            if k > 0:
                # compliant tie: each translation is a stiff spring with a hard stop at SOFT_MAX, so a taut cord
                # between two held bodies stretches its ties a few mm instead of locking the solver
                for dof in ("transX", "transY", "transZ"):
                    lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
                    lim.CreateLowAttr(-SOFT_MAX)
                    lim.CreateHighAttr(SOFT_MAX)
                    drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), dof)
                    drv.CreateTypeAttr("force")
                    drv.CreateStiffnessAttr(k)
                    drv.CreateDampingAttr(SOFT_C)
                    drv.CreateTargetPositionAttr(0.0)
            else:
                lock(j, ("transX", "transY", "transZ"))
            j.CreateExcludeFromArticulationAttr(True)

        sa = np.array(STRING.start)
        ea = sa + STRING.length * STRING.direction
        ball("string_bottom_tie", EYE_BOT, sa - np.array(EYE_BOT.pos), STRING, "start", STRING_K)
        ball("string_top_tie", EYE_TOP, ea - np.array(EYE_TOP.pos), STRING, "end", STRING_K)
        for cab, limb in ((STR_L, LIMB_L), (STR_R, LIMB_R)):
            s0 = np.array(cab.start)
            e0 = s0 + cab.length * cab.direction
            ball(f"cup_{cab.name}", limb, s0 - np.array(limb.pos), cab, "start")
            ball(f"stirrup_{cab.name}", STIRRUP, e0 - np.array(STIRRUP.pos), cab, "end")
        print(f"[bow] joints authored; string {STRING_LEN:.4f} m, top eye s0 {S0:.4f}", flush=True)
        return prim

    spawn._bow_hook = True
    cablekit._spawn_cable = spawn


_install_spawn_hook()

# -- contact monitor: robot links x limbs and lips ---------------------------------------------------------------------
TOUCH_SEP = 0.001
_ROBOT_PRIMS = ("/LeftArm/", "/RightArm/")
LIMB_PARTS = [LIMB_L.name, LIMB_R.name, LIP_T.name, LIP_B.name, EYE_BOT.name]
_MON: dict = {"robot_limb": False, "views": None, "polls": 0, "err": None}


def _reset_monitor() -> None:
    _MON.update(robot_limb=False, polls=0)


def _enable_contact_processing() -> None:
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # pragma: no cover
        pass


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def _body_path(stage, name: str) -> str | None:
    from pxr import UsdPhysics

    for prim in stage.Traverse():
        p = prim.GetPath().pathString
        if "/env_0/" in p and p.rsplit("/", 1)[-1] == name and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return p
    return None


def _make_views(mgr) -> None:
    from isaaclab.sim.utils.stage import get_current_stage

    sv = mgr.get_physics_sim_view()
    if sv is None:
        return
    stage = get_current_stage()
    robot = [p.GetPath().pathString for p in stage.Traverse()
             if "/env_0/" in p.GetPath().pathString and any(r in p.GetPath().pathString for r in _ROBOT_PRIMS)
             and "PhysxContactReportAPI" in p.GetAppliedSchemas()]
    targets = [_body_path(stage, n) for n in LIMB_PARTS]
    if not robot or None in targets:
        raise RuntimeError(f"bow contact monitor: robot {len(robot)}, targets {targets}")
    v1 = sv.create_rigid_contact_view(robot, filter_patterns=[targets] * len(robot),
                                      max_contact_data_count=64 * len(robot) * len(targets))
    _MON["views"] = (v1, len(robot), len(targets))
    _MON["names"] = ([r.rsplit("/", 2)[-2] + "/" + r.rsplit("/", 1)[-1] for r in robot],
                     [t.rsplit("/", 1)[-1] for t in targets])
    print(f"[bow] contact monitor: {len(robot)} robot links x {len(targets)} limb parts", flush=True)


def _hits(view, dt, n_sensors, n_filters) -> np.ndarray:
    _, _, _, sep, counts, starts = view.get_contact_data(dt=dt)
    counts = _np(counts).reshape(n_sensors, n_filters)
    out = np.zeros_like(counts, dtype=bool)
    if not counts.any():
        return out
    starts = _np(starts).reshape(n_sensors, n_filters)
    sep = _np(sep).reshape(-1)
    for i, j in zip(*np.nonzero(counts)):
        s = int(starts[i, j])
        out[i, j] = float(sep[s:s + int(counts[i, j])].min()) <= TOUCH_SEP
    return out


def _poll_contacts(mgr) -> None:
    try:
        if _MON["views"] is None:
            _make_views(mgr)
        if _MON["views"] is None:
            return
        v1, ns, nf = _MON["views"]
        h1 = _hits(v1, mgr.get_physics_dt(), ns, nf)
    except Exception as e:  # pragma: no cover
        if _MON["err"] is None:
            _MON["err"] = repr(e)
            print(f"[bow] contact monitor error: {e!r}", flush=True)
        return
    _MON["polls"] += 1
    if h1.any() and not _MON["robot_limb"]:
        _MON["robot_limb"] = True
        pairs = [f"{_MON['names'][0][i]}-{_MON['names'][1][j]}" for i, j in zip(*np.nonzero(h1))]
        print(f"[bow] contact: robot touched a limb (poll {_MON['polls']}) {pairs}", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_bow_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__
    base_step = mgr.step.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        _enable_contact_processing()

    def step(cls) -> None:
        base_step(cls)
        _poll_contacts(cls)

    step._bow_hooked = True
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


def _install_hooks() -> None:
    _patch_physics()
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_bow_hooked", False):
        base_reset = cls.reset

        def reset(self, *a, **k):
            _MON["views"] = None
            _ST["env"] = self._env
            _reset_episode()
            out = base_reset(self, *a, **k)
            _reset_monitor()
            _reset_episode()
            _ST["env"] = self._env
            return out

        reset._bow_hooked = True
        cls.reset = reset
    if not getattr(cls.step, "_bow_hooked", False):
        base_step = cls.step

        def step(self, *a, **k):
            _ST["env"] = self._env
            out = base_step(self, *a, **k)
            try:
                _track()
            except Exception as e:  # pragma: no cover
                if _ST["err"] is None:
                    _ST["err"] = repr(e)
                    print(f"[bow] tracker error: {e!r}", flush=True)
            return out

        step._bow_hooked = True
        cls.step = step


# -- latched episode state ------------------------------------------------------------------------------------------------
_ST: dict = {"env": None, "err": None}


def _reset_episode() -> None:
    _ST.update(forced=False, nonfinite=False, steps=0, up=False)


_reset_episode()


def grippers() -> list:
    """World TCPs of the two grippers (left, right); [] offline."""
    env = _ST["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


def _live_states():
    env = _ST["env"]
    if env is None:
        return None
    from yam_arena.scene import object_states

    return object_states(env, [o.name for o in OBJECTS])


def _track() -> None:
    """Per control step: latch a non-finite state and an eye pulled up the limb while the stringer is not drawn."""
    st = _live_states()
    if st is None:
        return
    _ST["steps"] += 1
    if not finite(st):
        _ST["nonfinite"] = True
        return
    s, _ = eye_s(st)
    up = s > S0 + FORCE_DS
    crossed, _ST["up"] = up and not _ST["up"], up
    if crossed and not stringer_drawn(st) and not _ST["forced"]:   # each time the eye goes up past the mark
        _ST["forced"] = True
        print(f"[bow] eye pulled {1000 * (s - S0):.1f} mm up the limb with the stringer not drawn "
              f"(handle {1000 * (ST_X0 - float(st[STIRRUP.name].pos[0])):.0f} mm back)", flush=True)


_install_hooks()

TWIN_EXTRA = [
    {"name": "limb_pivots", "kind": "joint", "real": "two M4 shoulder bolts through the riser ends, each limb with a "
     "torsion spring (about 0.15 N m/rad) and a stop at the unbent rest", "note": "limbs turn about vertical axes, 0-50 deg"},
    {"name": "eye_slider", "kind": "joint", "real": "printed loop sliding on the limb's top edge (PTFE tape)",
     "note": "top eye slides along the left-hand limb and may lift 12 mm (to pass over the nock lip)"},
    {"name": "string_ties", "kind": "joint", "real": "string ends tied to the eyes", "note": "ball joints"},
    {"name": "stringer_ties", "kind": "joint", "real": "stringer cords tied to the tip cups and the handle",
     "note": "ball joints"},
]

VARIATION = {"obj:stirrup": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}  # bow and stringer are one jointed assembly: fixed


# -- measures -----------------------------------------------------------------------------------------------------------
FORCE_DS = 0.003         # eye moved more than 3 mm up the limb from where the straight string holds it ...
DRAWN_BACK = 0.080       # ... while the handle is less than 8 cm behind its rest position = forced
BRACED_DEG = 15.0        # both limbs held at least this far bent at the end
CLEAR_EYE = 0.04         # no gripper TCP within 4 cm of the top eye at the end
CLEAR_HANDLE = 0.05      # no gripper TCP within 5 cm of the stringer handle (it is let go)


def _yaw(q) -> float:
    x, y, z, w = (float(t) for t in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def flex_deg(objects, sign) -> float:
    """How far a limb is bent towards the belly (deg; 0 = unbent rest)."""
    limb = objects[(LIMB_L if sign > 0 else LIMB_R).name]
    return sign * math.degrees(_yaw(limb.quat_xyzw))


def eye_s(objects) -> tuple:
    """Top eye centre along the left-hand limb from its pivot (m) and its lift above the slider bed (m)."""
    limb = objects[LIMB_L.name]
    a = _yaw(limb.quat_xyzw)
    axis = np.array([-math.sin(a), math.cos(a)])
    e = np.asarray(objects[EYE_TOP.name].pos, dtype=float)
    return float(np.dot(e[:2] - np.array([PX, Y0]), axis)), float(e[2] - EZ)


def seated(objects) -> bool:
    """Top eye in the nock: within 1.5 mm of the nock position and down behind the lip (lifted < 3 mm)."""
    s, dz = eye_s(objects)
    return s >= S_SEAT - 0.0015 and dz < 0.003


def stringer_drawn(objects) -> bool:
    return float(objects[STIRRUP.name].pos[0]) <= ST_X0 - DRAWN_BACK


def braced(objects) -> bool:
    return flex_deg(objects, 1) >= BRACED_DEG and flex_deg(objects, -1) >= BRACED_DEG


def finite(objects) -> bool:
    return all(np.isfinite(np.asarray(objects[o.name].pos)).all() for o in OBJECTS)


def _near(objects, name, r) -> bool:
    g = grippers()
    p = np.asarray(objects[name].pos, dtype=float)
    return any(float(np.linalg.norm(t - p)) < r for t in g)


def eye_released(objects) -> bool:
    return bool(grippers()) and not _near(objects, EYE_TOP.name, CLEAR_EYE)


def handle_released(objects) -> bool:
    return bool(grippers()) and not _near(objects, STIRRUP.name, CLEAR_HANDLE)


def violated(objects) -> bool:
    return bool(_MON["robot_limb"] or _ST["forced"] or _ST["nonfinite"] or not finite(objects))


def score(objects) -> float:
    if violated(objects):
        return 0.0
    if not (seated(objects) and braced(objects) and eye_released(objects)):
        return 0.0
    return 1.0 if handle_released(objects) else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    s, _ = eye_s(objects)
    return float(abs(S_SEAT - s))


def progress(objects) -> dict:
    if not finite(objects):
        return {"limbs_flexed": 0, "eye_seated": 0, "eye_released": 0, "string_carries": 0, "stringer_released": 0,
                "no_violation": 0}
    return {"limbs_flexed": int(min(flex_deg(objects, 1), flex_deg(objects, -1)) >= 25.0),
            "eye_seated": int(seated(objects)), "eye_released": int(eye_released(objects)),
            "string_carries": int(seated(objects) and braced(objects)),
            "stringer_released": int(handle_released(objects)), "no_violation": int(not violated(objects))}


# -- closed-loop scripted demo (cabledemo.py) -----------------------------------------------------------------------------
GX = ST_SIZE[0] / 2 - 0.012       # grip the handle's rear end, clear of the cords on its front face
GRIP_DZ = 0.015                   # TCP above the eye centre when pinching it
FLEX_TARGET = 29.0
HOME_Z = TOP + 0.10
HOME = {"left": (0.08, 0.52, HOME_Z), "right": (0.08, -0.52, HOME_Z)}


def _log(ctx, tag=""):
    st = ctx.states()
    s, dz = eye_s(st)
    print(f"[t={ctx.t:5.2f}] {tag}: flexL={flex_deg(st, 1):5.1f} flexR={flex_deg(st, -1):5.1f} eye_s={s:.4f} "
          f"dz={dz * 1000:4.1f}mm seated={seated(st)} handle={np.round(st[STIRRUP.name].pos, 3)} "
          f"touch={_MON['robot_limb']} forced={_ST['forced']} score={score(st)}", flush=True)


def _eye_target(ctx, ds, dz):
    o = ctx.states()
    a = _yaw(o[LIMB_L.name].quat_xyzw)
    axis = np.array([-math.sin(a), math.cos(a), 0.0])
    e = np.asarray(o[EYE_TOP.name].pos, dtype=float)
    return e + ds * axis + np.array([0.0, 0.0, dz]), a


def _retreat(arm, side, yaw):
    """Open, lift 4 cm straight up, then sweep back towards the arm's own base and out to the side."""
    from yam_arena.constants import FINGER_OPEN

    yield from arm.grip(FINGER_OPEN, 0.4)
    t = arm.tcp()
    yield from arm.move((t[0], t[1], t[2] + 0.04), yaw=yaw, duration=0.4)
    yield from arm.move(HOME[side], yaw=0.0, duration=1.0)


def _draw(ctx, arm):
    """Grip the stringer handle and draw it towards the robot until both limbs are bent FLEX_TARGET."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    st = ctx.states()[STIRRUP.name].pos
    yield from arm.move((st[0] - GX, 0.0, TOP + 0.09), yaw=0.0, finger=FINGER_OPEN, duration=1.5)
    st = ctx.states()[STIRRUP.name].pos
    yield from arm.move((st[0] - GX, 0.0, TOP + 0.028), yaw=0.0, duration=1.0)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    _log(ctx, "handle gripped")
    p = arm.tcp().copy()
    for _ in range(70):
        o = ctx.states()
        if min(flex_deg(o, 1), flex_deg(o, -1)) >= FLEX_TARGET or not finite(o):
            break
        p[0] -= 0.004
        yield from arm.move(tuple(p.tolist()), yaw=0.0, duration=0.15)
    _log(ctx, "drawn")


def _ease(ctx, arm):
    """Ease the handle back until the string carries the limbs (handle stops following)."""
    p = arm.tcp().copy()
    for _ in range(60):
        if p[0] >= ST_X0 - 0.03:
            break
        p[0] += 0.004
        yield from arm.move(tuple(p.tolist()), yaw=0.0, duration=0.12)
    _log(ctx, "eased")


def _seat(ctx, arm, release=True):
    """Pinch the top eye, lift it over the nock lip, slide it up the limb into the nock, lower it behind the lip."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    e, a = _eye_target(ctx, 0.0, 0.0)
    yield from arm.move((e[0], e[1], e[2] + 0.06), yaw=a, finger=FINGER_OPEN, duration=1.5)
    e, a = _eye_target(ctx, 0.0, 0.0)
    yield from arm.move((e[0], e[1], e[2] + GRIP_DZ), yaw=a, duration=1.0)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    yield from arm.move((e[0], e[1], e[2] + GRIP_DZ + 0.009), yaw=a, duration=0.5)
    p = np.array(arm.tcp(), dtype=float)
    z_cap = p[2] + 0.015
    stuck = 0
    for k in range(60):
        s, dz = eye_s(ctx.states())
        if s >= S_SEAT + 0.002:
            break
        e, a = _eye_target(ctx, 0.0, 0.0)
        ax = np.array([-math.sin(a), math.cos(a)])
        lead = float(np.dot(p[:2] - e[:2], ax))
        if k % 4 == 0:
            print(f"[t={ctx.t:5.2f}] slide: eye_s={s:.4f} dz={dz * 1000:4.1f}mm lead={lead * 1000:4.1f}mm", flush=True)
        stuck = stuck + 1 if (lead >= 0.015 and dz >= 0.006) else 0
        if stuck >= 3:
            break                                              # lifted but not following for 0.45 s: stop pushing
        if dz < 0.006 and p[2] < z_cap:
            p[2] += 0.0015                                     # keep the eye up while it slides over the lip
        if lead < 0.015:                                       # never run the command far ahead of a stuck eye
            p[:2] += 0.003 * ax                                # step the command, not the measured eye
        yield from arm.move(tuple(p.tolist()), yaw=a, duration=0.15)
    e, a = _eye_target(ctx, 0.0, 0.0)
    yield from arm.move((e[0], e[1], EZ + GRIP_DZ - 0.001), yaw=a, duration=0.6)   # down onto the bed, behind the lip
    _log(ctx, "eye in nock")
    if release:
        yield from _retreat(arm, "left" if arm is ctx.left else "right", a)
        _log(ctx, "eye let go")
    return a


def demo_script(ctx):
    """Modes (BOW_DEMO): correct | none | draw_only | push_eye | onearm | hold | tier05 | limb."""
    mode = os.environ.get("BOW_DEMO", "correct")
    L, R = ctx.left, ctx.right
    yield from L.wait(0.5)
    _log(ctx, f"start ({mode})")
    if mode == "none":
        yield from L.wait(4.0)
        _log(ctx, "end")
        return
    if mode == "push_eye":       # slide the eye up the unbent limb by force, no stringer
        yield from _seat(ctx, L)
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    if mode == "limb":           # bend the left-hand limb by hand (push its back face), let it go, then seat the eye
        from yam_arena.constants import FINGER_CLOSED

        side = (LT / 2 + 0.020, 0.09)                  # behind the back face, 9 cm out from the pivot
        a0 = _limb_pt(1, side)
        a1 = _limb_pt(1, side, math.radians(30.0))
        yield from L.move((a0[0], a0[1], TOP + 0.06), yaw=0.0, finger=FINGER_CLOSED, duration=1.5)
        yield from L.move((a0[0], a0[1], TOP + 0.025), yaw=0.0, duration=0.8)
        yield from L.move((a1[0], a1[1], TOP + 0.025), yaw=0.0, duration=1.5)
        _log(ctx, "limb pushed")
        yield from L.move((a1[0], a1[1], TOP + 0.08), yaw=0.0, duration=0.5)
        yield from _seat(ctx, L)
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    if mode == "onearm":         # the left arm alone: draw with the stringer, let go of it, then seat the eye
        yield from _draw(ctx, L)
        yield from _retreat(L, "left", 0.0)
        _log(ctx, "handle let go")
        yield from _seat(ctx, L)
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    yield from _draw(ctx, R)
    yield from R.wait(0.5)
    if mode == "draw_only":      # bend the limbs with the stringer, never seat the eye, ease back and let go
        yield from R.wait(2.0)
        yield from _ease(ctx, R)
        yield from _retreat(R, "right", 0.0)
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    a = yield from _seat(ctx, L, release=(mode != "hold"))
    if mode == "tier05":         # string seated and eye let go, but the stringer handle is still held
        yield from L.wait(3.0)
        _log(ctx, "end")
        return
    yield from _ease(ctx, R)
    yield from _retreat(R, "right", 0.0)
    _log(ctx, "handle let go")
    for _ in range(8):
        yield from L.wait(0.5)
        _log(ctx, "end")
