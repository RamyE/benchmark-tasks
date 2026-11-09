"""A model marina berth on the station table: spring a boat's stern off a tight berth on its bow spring, then bring
all four lines aboard.

A grey dock (8 x 106 x 4 cm) runs left-right in front of the robot with four black posts on it. An orange boat
(10 x 46 x 2.5 cm, bow to the robot's left) floats alongside on two black fenders, berthed between two fixed grey boats
(12 cm ahead, 8 cm astern). Four dark-blue 6 mm lines run slack from cleats on the boat to the posts, each ending in a
red square eye with a grip bail dropped over its post: bow line forward to the far-left post, bow spring aft to the
post left of the middle, stern spring forward to the post right of the middle, stern line aft to the far-right post.
A lamp box on the dock shows amber 5 s before the engine starts and green while it runs.

At t = 30 s the engine runs ahead for 10 s (a 0.3 N push at the stern, turned 20 deg outboard as with the helm put
towards the dock), then goes to neutral. The bow spring makes the stern swing out without the boat driving into the
boat ahead; keeping the bow line as well also swings it, but the score accepts only the bow spring kept. The robot
must lift the other three eyes off their posts and lay them on the deck before 30 s,
keep its hands clear while the engine runs, then lift the bow spring's eye off its post and lay it aboard too.
The bow line and stern line posts are each reachable by one arm only.

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

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("This boat is berthed tight between two others; its engine runs ahead for 10 s, starting in 30 s. "
               "Before then, lay three dock lines on the deck and keep one on its post, so the engine swings the stern "
               "out clear of the other boats. Then bring the last line aboard and let go.")

# -- colours ------------------------------------------------------------------------------------------------------------
DOCK_C = (0.55, 0.55, 0.55)
BLACK = (0.05, 0.05, 0.05)
ORANGE = (0.95, 0.45, 0.10)
GREY_HULL = (0.78, 0.78, 0.80)
WATER_C = (0.25, 0.48, 0.70)
ROPE_C = (0.08, 0.16, 0.50)
EYE_C = (0.85, 0.10, 0.08)

# -- dock, boat, neighbours (m, world frame: x forward, y left, z up) ----------------------------------------------------
DOCK_X0, DOCK_X1, DOCK_H = 0.18, 0.26, 0.040
DOCK_HY = 0.53
DOCK = Box("dock", pos=((DOCK_X0 + DOCK_X1) / 2, 0.0, TOP + DOCK_H / 2), size=(DOCK_X1 - DOCK_X0, 2 * DOCK_HY, DOCK_H),
           color=DOCK_C)
BOAT_L, BOAT_W, BOAT_H = 0.10, 0.46, 0.025  # x (beam), y (length), z
RAIL = (0.004, BOAT_W - 0.02, 0.006)  # toe rail along the outboard deck edge
FENDER_T = 0.010
BOAT_X0 = DOCK_X1 + FENDER_T  # the boat's dock-side edge on its fenders
BOAT_C0 = (BOAT_X0 + BOAT_L / 2, 0.0, TOP + DOCK_H - BOAT_H / 2)  # deck flush with the dock top
DECK_Z = TOP + DOCK_H
BOAT = Box("boat", pos=BOAT_C0, size=(BOAT_L, BOAT_W, BOAT_H), mass=0.20, color=ORANGE)
FENDER_Y = 0.17
FENDERS = [Box(f"fender_{n}", pos=(DOCK_X1 + FENDER_T / 2, s * FENDER_Y, TOP + DOCK_H - 0.0125),
               size=(FENDER_T, 0.04, 0.025), color=BLACK) for n, s in (("fwd", 1), ("aft", -1))]
GAP = {"ahead": 0.12, "astern": 0.08}  # bow / stern to the next boat
NB_L = 0.18
NB_X0 = DOCK_X1 + 0.020  # the neighbours lie on their own (unmodelled) fenders, 2 cm off the dock
NEIGHBOURS = {n: Box(f"boat_{n}", pos=(NB_X0 + BOAT_L / 2, s * (BOAT_W / 2 + GAP[n] + NB_L / 2), BOAT_C0[2]),
                     size=(BOAT_L, NB_L, BOAT_H), color=GREY_HULL) for n, s in (("ahead", 1), ("astern", -1))}
WATER = Box("water", pos=(0.48, 0.0, TOP + 0.004), size=(0.42, 2 * DOCK_HY, 0.008), color=WATER_C)

# the boat floats: free in x, y and yaw, with water drag; height, roll and pitch held
DRAG_LIN = float(os.environ.get("SPRING_DRAG", "3.0"))  # N s/m
DRAG_ROT = float(os.environ.get("SPRING_DRAG_ROT", "0.4"))  # N m s/rad
THRUST = float(os.environ.get("SPRING_THRUST", "0.30"))  # N along the hull's fore-aft axis, at the stern centreline
HELM = math.radians(float(os.environ.get("SPRING_HELM", "20")))  # thrust turned outboard at the stern (helm to the dock)
T_ENG0 = float(os.environ.get("SPRING_T0", "30.0"))  # engine ahead from t 30 s
T_RUN = 10.0  # for 10 s, then neutral
T_ENG1 = T_ENG0 + T_RUN
T_AMBER = 5.0  # amber lamp 5 s before the engine starts

# engine lamps: a black lamp box on the dock; a lit lamp is a coloured block raised 2 cm out of it, an unlit one is
# sunk inside it (out of sight)
LAMP_X, LAMP_Y, LAMP_S = 0.195, 0.0, (0.026, 0.024, 0.020)
LAMP_RISE = 0.020
LAMP_BLOCK = (0.010, 0.020, 0.012)
LAMPS = {"amber": (-0.006, (1.0, 0.62, 0.05)), "green": (0.006, (0.10, 0.85, 0.20))}  # x offset, colour
LAMP_BOX = Box("lamp_box", pos=(LAMP_X, LAMP_Y, TOP + DOCK_H + LAMP_S[2] / 2), size=LAMP_S, color=BLACK)

# -- dock posts (bollards) and the four lines ----------------------------------------------------------------------------
PIN_R, PIN_H = 0.010, 0.035
PIN_X = 0.22
P_OUT = float(os.environ.get("SPRING_P_OUT", "0.48"))
P_IN = 0.05
PIN_Y = {"bow_line": P_OUT, "bow_spring": P_IN, "stern_spring": -P_IN, "stern_line": -P_OUT}
PINS = [Cylinder(f"post_{k}", pos=(PIN_X, y, TOP + DOCK_H + PIN_H / 2), radius=PIN_R, height=PIN_H, color=BLACK)
        for k, y in PIN_Y.items()]
CLEAT = {"bow_line": (0.285, 0.215), "bow_spring": (0.29, 0.19), "stern_spring": (0.29, -0.19),
         "stern_line": (0.285, -0.215)}
KEEP = "bow_spring"

R = 0.003
BAR = 0.006
RING_T = 0.005
RING_W = 0.044
HOLE = RING_W - 2 * BAR  # 32 mm
HOLE_C = BAR / 2 + HOLE / 2  # hole centre from the inner bar (plug) centre, along the line
BAIL_H = 0.045
BAIL_LEG = (0.006, 0.004, BAIL_H)
BAIL_BAR = (0.008, RING_W, 0.006)
EYE_PLUG = Plug(size=(BAR, RING_W, RING_T), mass=0.008, color=EYE_C)
LINE_Z = DECK_Z + R + 0.001

_ONLY = [s for s in os.environ.get("SPRING_ONLY", "").split(",") if s]  # probe: spawn only these lines


SEG = 0.014  # short segments, so a line laid on deck can bunch up instead of jamming
LINE_MU = 0.5
SLACK = {"bow_line": 0.03, "bow_spring": 0.01, "stern_spring": 0.01, "stern_line": 0.03}  # beyond the straight run


def _lay_out(name: str):
    """Start layout of a line: (start, chord unit, end tip, length, sideways unit, bow height). The line lies flat from
    its cleat to its post, bowed sideways towards the dock (sin^2 bump, so it leaves the cleat and reaches the eye
    along the straight run), with SLACK[name] more length than the straight run."""
    S = np.array([*CLEAT[name], LINE_Z])
    post = np.array([PIN_X, PIN_Y[name], LINE_Z])
    u = (post - S) / np.linalg.norm(post - S)
    T = post - (BAR / 2 + HOLE_C) * u
    L = float(np.linalg.norm(T - S)) + SLACK[name]
    n = np.array([-u[1], u[0], 0.0])
    if n[0] > 0:
        n = -n
    s = np.linspace(0.0, 1.0, 400)
    lo, hi = 0.0, 0.2
    for _ in range(50):
        h = (lo + hi) / 2
        P = S + np.outer(s, T - S) + h * np.outer(np.sin(np.pi * s) ** 2, n)
        if np.linalg.norm(np.diff(P, axis=0), axis=1).sum() < L:
            lo = h
        else:
            hi = h
    return S, u, T, L, n, lo


def start_path(name: str) -> np.ndarray:
    S, u, T, L, n, h = _lay_out(name)
    s = np.linspace(0.0, 1.0, 120)
    return S + np.outer(s, T - S) + h * np.outer(np.sin(np.pi * s) ** 2, n)


def _line(name: str):
    """Spawned straight from the cleat, turned towards the bow side just enough that its eye lands clear of the post;
    the reset then lays it along start_path()."""
    S, u, T, L, n, _ = _lay_out(name)
    psi = math.atan2(u[1], u[0])
    sgn = math.copysign(1.0, u[0] * n[1] - u[1] * n[0])
    post = np.array([PIN_X, PIN_Y[name]])
    yaw = psi
    for dd in range(0, 40):
        yaw = psi + sgn * math.radians(dd)
        if np.linalg.norm(S[:2] + (L + BAR / 2 + HOLE_C) * np.array([math.cos(yaw), math.sin(yaw)]) - post) >= 0.045:
            break
    return cable(name, start=tuple(S), yaw=yaw, length=L, radius=R, segment=L / max(6, round(L / SEG)),
                 cable_mass=0.05 * L, bend_stiffness=0.002, bend_damping=0.0005, friction=LINE_MU, end_plug=EYE_PLUG,
                 color=ROPE_C)


LINES = {k: _line(k) for k in CLEAT if not _ONLY or k in _ONLY}

OBJECTS = [DOCK, WATER, BOAT, *FENDERS, *NEIGHBOURS.values(), *PINS, LAMP_BOX, *LINES.values()]

TWIN_EXTRA = [
    {"name": "float", "kind": "joint", "real": "hull floating in a shallow water tray",
     "note": "boat free in x, y and yaw; water drag 3 N s/m and 0.4 N m s/rad"},
    {"name": "engine", "kind": "joint", "real": "small waterproof motor and propeller at the stern, timed by a "
     "microcontroller (about 0.3 N thrust, rudder turned 20 deg so the stern is pushed off the dock)",
     "note": "ahead from 30 s to 40 s after the start, then neutral"},
    {"name": "engine_lamps", "kind": "body", "size_mm": [26, 24, 20], "colour": "black",
     "pose": [LAMP_X, LAMP_Y, DOCK_H], "note": "amber LED lit 25-30 s, green LED lit 30-40 s"},
    {"name": "toe_rail", "kind": "body", "size_mm": [4, 440, 6], "colour": "white",
     "note": "glued along the boat's outboard deck edge"},
    *({"name": f"cleat_{k}", "kind": "joint", "real": "line tied to a small deck cleat", "note": f"at {CLEAT[k]}"}
      for k in CLEAT),
    *({"name": f"eye_{k}", "kind": "body", "size_mm": [44, 44, 50], "colour": "red",
       "note": "square ring of 6 mm bars (32 mm hole) with a 45 mm grip bail over the hole, glued to the line end"}
      for k in CLEAT),
]

VARIATION = {**{f"cable:{k}": {"dx": 0.0, "dy": 0.0, "yaw": 0.0} for k in CLEAT},
             "obj:boat": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}  # lines tied to the boat and laid to their posts: fixed


# -- spawn hook: eye rings with bails, cleat ties, the boat's float joint ------------------------------------------------
_PATHS: dict = {}


def _ring_parts() -> list:
    return [("side_a", (HOLE_C, (RING_W - BAR) / 2, 0.0), (HOLE, BAR, RING_T)),
            ("side_b", (HOLE_C, -(RING_W - BAR) / 2, 0.0), (HOLE, BAR, RING_T)),
            ("outer", (BAR + HOLE, 0.0, 0.0), (BAR, RING_W, RING_T)),
            ("bail_a", (HOLE_C, (RING_W - BAR) / 2, BAIL_H / 2), BAIL_LEG),
            ("bail_b", (HOLE_C, -(RING_W - BAR) / 2, BAIL_H / 2), BAIL_LEG),
            ("bail_bar", (HOLE_C, 0.0, BAIL_H + BAIL_BAR[2] / 2 - 0.002), BAIL_BAR)]


def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_spring_task", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        name = getattr(cfg, "cable_name", "")
        if name not in LINES:
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        boat = f"{root}/{BOAT.name}"
        spec = LINES[name]
        q0 = Gf.Quatf(1, 0, 0, 0)

        def look(path, rgb):
            m = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(rgb), roughness=0.6)
            m.func(path, m)
            return path

        def cube(path, pos, size, lk, collide=True):
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
            if collide:
                UsdPhysics.CollisionAPI.Apply(g.GetPrim())
                cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
                cp.CreateContactOffsetAttr(0.002)
                cp.CreateRestOffsetAttr(0.0)
            bind_visual_material(str(g.GetPath()), lk, stage=stage)
            return g

        # cleat tie: the line's start tip on the deck (ball joint, translations locked)
        cx, cy = CLEAT[name]
        j = UsdPhysics.Joint.Define(stage, f"{prim_path}/cleat_tie")
        j.CreateBody0Rel().SetTargets([Sdf.Path(boat)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        j.CreateLocalPos0Attr(Gf.Vec3f(float(cx - BOAT_C0[0]), float(cy - BOAT_C0[1]), float(LINE_Z - BOAT_C0[2])))
        j.CreateLocalRot0Attr(q0)
        j.CreateLocalPos1Attr(Gf.Vec3f(-spec.seg_len / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(q0)
        for dof in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        j.CreateExcludeFromArticulationAttr(True)
        # (no collision filter: a filtered pair with one link made the hull ignore the whole line and its eye)
        cube(f"{boat}/cleat_{name}", (cx - BOAT_C0[0], cy - BOAT_C0[1], BOAT_H / 2 + 0.003), (0.010, 0.026, 0.006),
             look(f"{prim_path}/Looks/cleat", BLACK), collide=False)
        # the eye: a square ring (the end plug is its inner bar) with a grip bail over the hole
        lk = f"{prim_path}/Looks/end_plug"
        phys = f"{prim_path}/Looks/cable_physics"
        for nm, pos, size in _ring_parts():
            g = cube(f"{prim_path}/end_plug/{nm}", pos, size, lk)
            bind_physics_material(str(g.GetPath()), phys, stage=stage)
        if name != list(LINES)[-1]:
            return prim
        # ---- once (last line): the boat's float joint ----
        bp = stage.GetPrimAtPath(boat)
        w = UsdGeom.Xformable(stage.GetPrimAtPath(root)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        fj = UsdPhysics.Joint.Define(stage, f"{root}/boat_float")
        fj.CreateBody1Rel().SetTargets([Sdf.Path(boat)])  # body0 empty = world
        fj.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + BOAT_C0[0]), float(w[1] + BOAT_C0[1]), float(w[2] + BOAT_C0[2])))
        fj.CreateLocalRot0Attr(q0)
        fj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        fj.CreateLocalRot1Attr(q0)
        jp = fj.GetPrim()
        for dof in ("transZ", "rotX", "rotY"):
            lim = UsdPhysics.LimitAPI.Apply(jp, dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        for dof, damp in (("transX", DRAG_LIN), ("transY", DRAG_LIN), ("rotZ", DRAG_ROT * math.pi / 180)):
            d = UsdPhysics.DriveAPI.Apply(jp, dof)
            d.CreateTypeAttr("force")
            d.CreateStiffnessAttr(0.0)
            d.CreateDampingAttr(damp)
            d.CreateTargetPositionAttr(0.0)
        fj.CreateExcludeFromArticulationAttr(True)
        cube(f"{boat}/toe_rail", (BOAT_L / 2 - RAIL[0] / 2, 0.0, BOAT_H / 2 + RAIL[2] / 2), RAIL,
             look(f"{root}/Looks/toe_rail", (0.96, 0.96, 0.94)))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(bp)
        rb.CreateSolverPositionIterationCountAttr(32)
        _PATHS.update(root=root, boat=boat)
        # engine lamps: gravity-free blocks without colliders on vertical slides, moved every substep
        for nm, (dy, col) in LAMPS.items():
            lp = f"{root}/lamp_{nm}"
            x = UsdGeom.Xform.Define(stage, lp)
            z0 = TOP + DOCK_H + LAMP_S[2] / 2 - 0.004
            x.AddTranslateOp().Set(Gf.Vec3d(LAMP_X + dy, LAMP_Y, z0))
            UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
            UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(0.01)
            PhysxSchema.PhysxRigidBodyAPI.Apply(x.GetPrim()).CreateDisableGravityAttr(True)
            cube(f"{lp}/geom", (0.0, 0.0, 0.0), LAMP_BLOCK, look(f"{root}/lamp_looks_{nm}", col), collide=False)
            sl = UsdPhysics.PrismaticJoint.Define(stage, f"{lp}/slide")
            sl.CreateBody1Rel().SetTargets([Sdf.Path(lp)])
            sl.CreateAxisAttr("Z")
            sl.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + LAMP_X + dy), float(w[1] + LAMP_Y), float(w[2] + z0)))
            sl.CreateLocalRot0Attr(q0)
            sl.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            sl.CreateLocalRot1Attr(q0)
            sl.CreateLowerLimitAttr(0.0)
            sl.CreateUpperLimitAttr(LAMP_RISE + 0.002)
            sl.CreateExcludeFromArticulationAttr(True)
            _PATHS[f"lamp_{nm}"] = lp
        return prim

    spawn._spring_task = True
    cablekit._spawn_cable = spawn


_install_spawn()


def _shape(emb):
    _ST["settling"] = True
    for k, spec in LINES.items():
        cableshape.preshape(emb, spec, start_path(k), hold="ramp", settle_s=0, lift=0.0)
    cableshape.settle(emb, 1.5)
    _ST["settling"] = False


cableshape.on_reset(_shape)


# -- episode state, engine thrust (every physics substep), hooks ---------------------------------------------------------
_ST: dict = {"env": None, "tick": 0, "t": 0.0, "t0": 0.0, "n": 0, "active": False, "bview": None, "err": None, "nan": False,
             "settling": False, "contact": None, "at_start": None, "at_neutral": None, "hand_t": 0.0, "hand": False,
             "hands_in": None, "min_gap": 1.0, "max_yaw": 0.0, "moved": None, "lviews": None, "was_on": {}, "dragged": None}


def _reset_episode() -> None:
    _ST.update(t=0.0, t0=0.0, n=0, active=False, nan=False, contact=None, at_start=None, at_neutral=None, hand_t=0.0,
               hand=False, hands_in=None, min_gap=1.0, max_yaw=0.0, moved=None, was_on={}, dragged=None)


def thrust_at(t: float) -> float:
    if t < T_ENG0 or t > T_ENG1 + 0.5:
        return 0.0
    return THRUST * min(1.0, (t - T_ENG0) / 0.5, max(0.0, (T_ENG1 + 0.5 - t) / 0.5))


def _quat_rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def lamp_on(nm: str, t: float) -> bool:
    if nm == "amber":
        return T_ENG0 - T_AMBER <= t < T_ENG0
    return T_ENG0 <= t < T_ENG1


def _drive_lamps(mgr, t: float, dt: float) -> None:
    """Raise a lit lamp 2 cm out of the lamp box, sink an unlit one (velocity writes with position correction)."""
    import warp as wp

    if _ST.get("lviews") is None:
        sv = mgr.get_physics_sim_view()
        if sv is None or "lamp_green" not in _PATHS:
            return
        _ST["lviews"] = {}
        for nm in LAMPS:
            v = sv.create_rigid_body_view(_PATHS[f"lamp_{nm}"])
            z0 = float(np.asarray(v.get_transforms().numpy()).reshape(-1)[2])
            _ST["lviews"][nm] = (v, z0)
    for nm, (v, z0) in _ST["lviews"].items():
        z = float(np.asarray(v.get_transforms().numpy()).reshape(-1)[2]) - z0
        tgt = LAMP_RISE if lamp_on(nm, t) else 0.0
        vz = float(np.clip((tgt - z) / max(dt, 1e-4) * 0.3, -0.2, 0.2))
        v.set_velocities(wp.array(np.array([[0, 0, vz, 0, 0, 0]], np.float32), dtype=wp.float32, device=_ST["dev"]),
                         _ST["idx"])


def _engine(mgr) -> None:
    """Every physics substep while the engine runs ahead: a force THRUST along the hull's fore-aft axis (towards the
    bow) applied at the stern on the centreline (as a force at the centre plus the matching yaw torque)."""
    try:
        import warp as wp

        if _ST["bview"] is None:
            boat = _PATHS.get("boat")
            sv = mgr.get_physics_sim_view()
            if boat is None or sv is None:
                return
            bview = sv.create_rigid_body_view(boat)
            raw = bview.get_transforms()
            _ST["dev"] = str(getattr(raw, "device", "cuda:0"))
            _ST["idx"] = wp.array(np.array([0], dtype=np.int32), dtype=wp.int32, device=_ST["dev"])
            _ST["bview"] = bview
            print(f"[spring] boat view on {boat}", flush=True)
        dt = mgr.get_physics_dt()
        t = _ST["t0"] + _ST["n"] * dt if _ST["active"] else 0.0
        _ST["n"] += 1
        _drive_lamps(mgr, t if _ST["active"] else 0.0, dt)
        f = thrust_at(t) if (_ST["active"] and not _ST["settling"]) else 0.0
        if f <= 0.0:
            return
        tf = np.asarray(_ST["bview"].get_transforms().numpy()).reshape(-1).astype(np.float64)
        Rm = _quat_rot(tf[3:7])
        fwd = Rm @ np.array([math.sin(HELM), math.cos(HELM), 0.0])
        F = f * fwd
        r = Rm @ np.array([0.0, -BOAT_W / 2, 0.0])
        tq = np.cross(r, F)
        if not (np.isfinite(F).all() and np.isfinite(tq).all()):
            return
        _ST["bview"].apply_forces_and_torques_at_position(
            wp.array(F.reshape(1, 3).astype(np.float32), dtype=wp.float32, device=_ST["dev"]),
            wp.array(tq.reshape(1, 3).astype(np.float32), dtype=wp.float32, device=_ST["dev"]), None, _ST["idx"], True)
    except Exception as e:  # pragma: no cover
        if _ST["err"] is None:
            _ST["err"] = repr(e)
            print(f"[spring] engine error: {e!r}", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_spring_hooked", False):
        return
    base_step = mgr.step.__func__

    def step(cls) -> None:
        base_step(cls)
        _engine(cls)

    step._spring_hooked = True
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
    if not getattr(cls.step, "_spring_hooked", False):
        base_step = cls.step

        def step(self, *a, **k):
            _ST["env"] = self._env
            _ST["t0"], _ST["n"], _ST["active"] = float(self._t), 0, True
            out = base_step(self, *a, **k)
            _ST["t"] = float(self._t)
            _ST["tick"] += 1
            try:
                _monitor()
            except Exception as e:  # pragma: no cover
                if _ST["err"] is None:
                    _ST["err"] = repr(e)
                    print(f"[spring] monitor error: {e!r}", flush=True)
            return out

        step._spring_hooked = True
        cls.step = step
    if not getattr(cls.reset, "_spring_hooked", False):
        base_reset = cls.reset

        def reset(self, *a, **k):
            _ST["env"] = self._env
            _ST["tick"] += 1
            _reset_episode()
            out = base_reset(self, *a, **k)
            _ST["tick"] += 1
            _ST["env"] = self._env
            _reset_episode()
            return out

        reset._spring_hooked = True
        cls.reset = reset


_install_hooks()


def grippers() -> list[np.ndarray]:
    env = _ST["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


def _t(a):
    return getattr(a, "torch", a)


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


# -- geometry -----------------------------------------------------------------------------------------------------------

def boat_pose(objects):
    """(centre (3,), rotation (3,3), yaw deg from the berth) of the hull."""
    b = objects[BOAT.name]
    Rm = _quat_rot(b.quat_xyzw)
    return np.asarray(b.pos, dtype=float), Rm, math.degrees(math.atan2(-Rm[0, 1], Rm[1, 1]))


def hull_corners(objects) -> np.ndarray:
    c, Rm, _ = boat_pose(objects)
    return np.array([c + Rm @ np.array([sx * BOAT_L / 2, sy * BOAT_W / 2, 0.0]) for sx in (-1, 1) for sy in (-1, 1)])


def stern_off(objects) -> float:
    """How far (m) the stern's dock-side corner is from the dock face."""
    c, Rm, _ = boat_pose(objects)
    return float((c + Rm @ np.array([-BOAT_L / 2, -BOAT_W / 2, 0.0]))[0] - DOCK_X1)


def _seg_rect_gap(a, b, lo, hi) -> float:
    """Smallest top-view distance from the segment a-b to the axis-aligned rectangle [lo, hi] (0 if they meet)."""
    best = 1.0
    for s in np.linspace(0.0, 1.0, 25):
        p = a + (b - a) * s
        d = np.maximum(np.maximum(lo - p, p - hi), 0.0)
        best = min(best, float(np.hypot(d[0], d[1])))
    return best


def neighbour_gap(objects) -> float:
    """Smallest top-view distance (m) from the hull outline to either neighbouring boat."""
    cs = hull_corners(objects)[:, :2]
    ring = [cs[0], cs[1], cs[3], cs[2]]
    best = 1.0
    for nb in NEIGHBOURS.values():
        lo = np.array(nb.pos[:2]) - np.array(nb.size[:2]) / 2
        hi = np.array(nb.pos[:2]) + np.array(nb.size[:2]) / 2
        for i in range(4):
            best = min(best, _seg_rect_gap(ring[i], ring[(i + 1) % 4], lo, hi))
    return best


def eye_frame(objects, k):
    """(hole centre (3,), unit along the line towards the eye's far bar) of line k's eye (flat-ring estimate)."""
    c = objects[k]
    plug, seg = np.asarray(c.end_plug, float), np.asarray(c.points[-1], float)
    d = plug - seg
    d = d / max(np.linalg.norm(d), 1e-9)
    return plug + d * HOLE_C, d


def eye_pose(k):
    """(plug centre, rotation) of line k's eye from the live body pose, or None offline."""
    env = _ST["env"]
    if env is None:
        return None
    key = (k, _ST["tick"])
    if key in _EYES:
        return _EYES[key]
    base = env.unwrapped
    art = base.scene[LINES[k].name]
    i = list(art.body_names).index("end_plug")
    origin = _np(_t(base.scene.env_origins)[0]).astype(np.float64)
    p = _np(_t(art.data.body_pos_w)[0, i]).astype(np.float64) - origin
    out = (p, _quat_rot(_np(_t(art.data.body_quat_w)[0, i]).astype(np.float64)))
    if len(_EYES) > 64:
        _EYES.clear()
    _EYES[key] = out
    return out


_EYES: dict = {}  # eye poses cached per control step (the state does not change between steps)


def eye_points(k) -> np.ndarray:
    """Hole centre, bail-bar centre and the ring's four bar centres of line k's eye (env frame)."""
    ep = eye_pose(k)
    if ep is None:
        return np.zeros((0, 3))
    p, Rm = ep
    loc = [(HOLE_C, 0, 0), (HOLE_C, 0, BAIL_H + 0.001), (0, 0, 0), (BAR + HOLE, 0, 0),
           (HOLE_C, (RING_W - BAR) / 2, 0), (HOLE_C, -(RING_W - BAR) / 2, 0)]
    return np.array([p + Rm @ np.array(v, dtype=float) for v in loc])


def on_post(k) -> bool:
    """Line k's eye is on its dock post: the post's axis meets the ring's plane inside the hole (2 mm in from the
    bars), between the dock top and the post top."""
    ep = eye_pose(k)
    if ep is None:
        return False
    p, Rm = ep
    nrm = Rm[:, 2]
    if abs(nrm[2]) < 0.3:  # ring standing on edge: no post through it
        return False
    z = p[2] - (nrm[0] * (PIN_X - p[0]) + nrm[1] * (PIN_Y[k] - p[1])) / nrm[2]
    loc = Rm.T @ (np.array([PIN_X, PIN_Y[k], z]) - p)
    inside = (BAR / 2 + 0.002 <= loc[0] <= BAR / 2 + HOLE - 0.002) and abs(loc[1]) <= HOLE / 2 - 0.002
    return bool(inside and TOP + DOCK_H - 0.005 <= z <= TOP + DOCK_H + PIN_H)


def on_deck(objects, k) -> bool:
    """Line k's eye lies on the boat: its hole centre is over the deck (top view, hull frame) and within 2.5 cm
    above the deck."""
    hp = eye_points(k)
    if len(hp) == 0:
        return False
    h = hp[0]
    c, Rm, _ = boat_pose(objects)
    loc = Rm.T @ (h - c)
    return bool(abs(loc[0]) <= BOAT_L / 2 and abs(loc[1]) <= BOAT_W / 2 and -0.002 <= h[2] - DECK_Z <= 0.025)


def _finite(objects) -> bool:
    b = objects[BOAT.name]
    return bool(np.isfinite(b.pos).all() and np.isfinite(b.quat_xyzw).all()
                and all(np.isfinite(objects[k].points).all() for k in LINES))


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = min(200, max(1, int(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def hull_dist(objects, p) -> float:
    """Distance (m) from point p to the hull box."""
    c, Rm, _ = boat_pose(objects)
    loc = Rm.T @ (np.asarray(p, dtype=float) - c)
    return float(np.linalg.norm(np.maximum(np.abs(loc) - np.array([BOAT_L, BOAT_W, BOAT_H]) / 2, 0.0)))


def gear_dist(objects, p) -> float:
    """Distance (m) from point p to the nearest part of the boat: hull, any line, any eye."""
    pts = np.vstack([_dense(objects[k].polyline()) for k in LINES] + [eye_points(k) for k in LINES])
    return min(hull_dist(objects, p), float(np.linalg.norm(pts - p, axis=1).min()))


HULL_DIST = 0.020  # a TCP this close to the hull = hand on the boat
HULL_T = 0.3  # for more than 0.3 s (in total) = the boat was pushed or held
CLEAR_DIST = 0.040  # stand clear: TCP at least 4 cm from the hull, every line and every eye
HELD_DIST = 0.025  # an eye leaving its post must have a TCP within 2.5 cm of its bail bar (lifted by hand)
CONTACT_GAP = 0.003  # hull within 3 mm of a neighbouring boat = contact
STILL_YAW = 8.0  # before the engine runs the boat must not be swung (yaw) by the robot
SWING_OFF, SWING_YAW = 0.08, 15.0  # at neutral: stern corner >= 8 cm off the dock face and yaw >= 15 deg


def released(objects) -> bool:
    g = grippers()
    return bool(g) and all(gear_dist(objects, p) >= CLEAR_DIST for p in g)


def _monitor() -> None:
    """Every control step: latch what the score needs. Latched until the next trial: a non-finite state; the hull
    within 3 mm of a neighbouring boat; a TCP within 2 cm of the hull for more than 0.3 s in total; the boat moved
    (yawed more than 8 deg) before the engine ran; a TCP within 4 cm of the hull, a line or an eye
    while the engine runs ahead (and 0.5 s after); the eyes' places at engine start; the swing at neutral."""
    env = _ST["env"]
    if env is None or _ST["settling"]:
        return
    from yam_arena import scene as ysc

    objects = ysc.object_states(env, [o.name for o in OBJECTS])
    if not _finite(objects):
        _ST["nan"] = True
        return
    t = _ST["t"]
    gap = neighbour_gap(objects)
    _ST["min_gap"] = min(_ST["min_gap"], gap)
    if gap < CONTACT_GAP and _ST["contact"] is None:
        _ST["contact"] = t
        print(f"[spring] boat touched a neighbour at t={t:.2f}", flush=True)
    c, _, yaw = boat_pose(objects)
    _ST["max_yaw"] = max(_ST["max_yaw"], yaw)
    g = grippers()
    if any(hull_dist(objects, p) < HULL_DIST for p in g):
        _ST["hand_t"] += 0.05
        if _ST["hand_t"] > HULL_T and not _ST["hand"]:
            _ST["hand"] = True
            print(f"[spring] a gripper held the hull at t={t:.2f}", flush=True)
    if t < T_ENG0 and _ST["moved"] is None and abs(yaw) > STILL_YAW:
        _ST["moved"] = t
        print(f"[spring] boat moved before the engine ran at t={t:.2f} (yaw {yaw:.1f})", flush=True)
    if T_ENG0 <= t <= T_ENG1 + 0.5 and _ST["hands_in"] is None and any(gear_dist(objects, p) < CLEAR_DIST for p in g):
        _ST["hands_in"] = t
        print(f"[spring] a gripper was at the boat or its lines while the engine ran (t={t:.2f})", flush=True)
    for k in LINES:  # an eye must leave its post in hand (pinched by its bail), not be pulled off by its line
        on = on_post(k)
        if _ST["was_on"].get(k, True) and not on and _ST["dragged"] is None:
            bail = eye_points(k)[1]
            if not any(float(np.linalg.norm(p - bail)) < HELD_DIST for p in g):
                _ST["dragged"] = (k, t)
                print(f"[spring] {k} eye left its post without a gripper on its bail at t={t:.2f}", flush=True)
        _ST["was_on"][k] = on
    if os.environ.get("SPRING_TRACE") and _ST["tick"] % 4 == 0 and (t < T_ENG0 or t > T_ENG1 + 3.0):
        print(f"[trace t={t:5.2f}] boat=({c[0]:.4f},{c[1]:.4f}) yaw={yaw:+.2f} tcp="
              f"{[np.round(p_, 3).tolist() for p_ in g]} hull_d={[round(hull_dist(objects, p_), 3) for p_ in g]}",
              flush=True)
    if _ST["at_start"] is None and t >= T_ENG0:
        _ST["at_start"] = {k: ("post" if on_post(k) else "deck" if on_deck(objects, k) else "other") for k in LINES}
        print(f"[spring] engine ahead at t={t:.2f}: eyes {_ST['at_start']}", flush=True)
    if _ST["at_neutral"] is None and t >= T_ENG1:
        _ST["at_neutral"] = {"stern_off": stern_off(objects), "yaw": yaw}
        print(f"[spring] neutral at t={t:.2f}: stern {stern_off(objects) * 100:.1f} cm off, yaw {yaw:.1f} deg", flush=True)


def set_up_right() -> bool:
    """At engine start only the bow spring's eye was on its post; the other three eyes lay on the deck."""
    a = _ST["at_start"]
    return bool(a) and all(a.get(k) == ("post" if k == KEEP else "deck") for k in CLEAT)


def swung_clear() -> bool:
    n = _ST["at_neutral"]
    return bool(n) and n["stern_off"] >= SWING_OFF and n["yaw"] >= SWING_YAW


def score(objects) -> float:
    """0 if: the state went non-finite; the boat ever came within 3 mm of a neighbouring boat; an eye left its post with
    no gripper TCP within 2.5 cm of its bail bar (pulled off by its line); a gripper TCP was
    within 2 cm of the hull for more than 0.3 s in total; the boat yawed more than 8 deg before the
    engine ran; a TCP came within 4 cm of the hull, a line or an eye while the engine ran ahead; the engine run is not
    over; at engine start any eye other than the bow spring's was not on the deck, or the bow spring's was not on its
    post; at neutral the stern's dock-side corner was under 8 cm off the dock face or the yaw under 15 deg; at the end
    a TCP is within 4 cm of the hull, a line or an eye. Otherwise 1.0 if all four eyes lie on the deck, else 0.5."""
    if not _finite(objects):
        _ST["nan"] = True
        return 0.0
    if _ST["nan"] or _ST["contact"] is not None or _ST["hand"] or _ST["moved"] is not None or _ST["dragged"]:
        return 0.0
    if _ST["hands_in"] is not None or _ST["at_neutral"] is None:
        return 0.0
    if not set_up_right() or not swung_clear() or not released(objects):
        return 0.0
    return 1.0 if all(on_deck(objects, k) for k in LINES) else 0.5


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and objects[BOAT.name].resting(0.01, 0.5))


def distance(objects) -> float:
    if not _finite(objects):
        return 1.0
    return float(sum(0.0 if on_deck(objects, k) else 0.1 for k in LINES)) + (0.0 if _ST["at_neutral"] else 0.2)


def progress(objects) -> dict:
    if not _finite(objects):
        return {"set_up": 0, "swung_clear": 0, "no_contact": 0, "spring_aboard": 0, "released": 0}
    return {"set_up": int(set_up_right()), "swung_clear": int(swung_clear()),
            "no_contact": int(_ST["contact"] is None), "spring_aboard": int(on_deck(objects, KEEP)),
            "released": int(released(objects))}


# -- closed-loop scripted demo (cabledemo.py) -----------------------------------------------------------------------------
GRIP_DZ = BAIL_H + 0.011  # TCP above the ring's mid-plane when pinching the bail bar
RING_Z = DECK_Z + RING_T / 2  # ring lying on the dock or the deck
LIFT_Z = RING_Z + GRIP_DZ + 0.050  # eye lifted clear of the post top (ring 1.5 cm over it)
FINGER_PART = -0.022
FINGER_LET_GO = -0.025
ARMI = {"left": 0, "right": 1}
# where each eye is laid: hull frame (x across, y along, + = bow)
DROP = {"bow_line": (-0.005, 0.02), "bow_spring": (-0.03, 0.08), "stern_spring": (-0.005, -0.12),
        "stern_line": (-0.005, -0.03)}  # each eye well away from its own cleat, so its line lies out on the deck
# (an eye laid close to its cleat leaves the line jammed between them, shoving the hull along)


def _log(ctx, tag=""):
    st = ctx.states()
    if not _finite(st):
        bad = [k for k in LINES if not np.isfinite(st[k].points).all()]
        print(f"[t={_ST['t']:5.2f}] {tag}: NON-FINITE boat={np.round(st[BOAT.name].pos, 3).tolist()} lines={bad}",
              flush=True)
        return
    c, Rm, yaw = boat_pose(st)
    eyes = {k: ("P" if on_post(k) else "D" if on_deck(st, k) else "-") for k in LINES}
    print(f"[t={_ST['t']:5.2f}] {tag}: boat=({c[0]:.3f},{c[1]:.3f}) yaw={yaw:+.1f} stern_off={stern_off(st) * 100:.1f}cm "
          f"gap={neighbour_gap(st) * 100:.1f}cm min_gap={_ST['min_gap'] * 100:.1f} contact={_ST['contact']} "
          f"eyes={eyes} hand={_ST['hand_t']:.2f} moved={_ST['moved']} in={_ST['hands_in']} drag={_ST['dragged']} "
          f"rel={released(st)} score={score(st):.2f}", flush=True)
    if os.environ.get("SPRING_DEBUG"):
        for k in LINES:
            h = eye_points(k)[0]
            loc = Rm.T @ (h - c)
            pl, Rp = eye_pose(k)
            print(f"    {k}: hole=({h[0]:.3f},{h[1]:.3f},{(h[2] - DECK_Z) * 1000:.1f}mm) hull-local=({loc[0]:.3f},{loc[1]:.3f}) "
                  f"plug_z={(pl[2] - DECK_Z) * 1000:.1f}mm ring_x={np.round(Rp[:, 0], 2).tolist()} "
                  f"ring_up={np.round(Rp[:, 2], 2).tolist()}", flush=True)


def _norm_yaw(y: float) -> float:
    y = (y + math.pi) % (2 * math.pi) - math.pi
    if y > math.pi / 2:
        y -= math.pi
    elif y < -math.pi / 2:
        y += math.pi
    return y


def _drop_xy(st, k) -> np.ndarray:
    c, Rm, _ = boat_pose(st)
    return (c + Rm @ np.array([DROP[k][0], DROP[k][1], 0.0]))[:2]


def _take(arm, ctx, side, k):
    """Pinch line k's bail bar (fingers closing along the ring) and lift the eye straight up off its post."""
    from yam_arena.constants import FINGER_CLOSED

    p, Rm = eye_pose(k)
    yg = _norm_yaw(math.atan2(Rm[1, 0], Rm[0, 0]) - math.pi / 2)
    b = eye_points(k)[1]
    yield from arm.move((b[0], b[1], LIFT_Z), yaw=yg, finger=FINGER_PART, duration=1.4)
    b = eye_points(k)[1]
    cmd = np.array([b[0], b[1], RING_Z + GRIP_DZ])
    yield from arm.move(tuple(cmd), yaw=yg, duration=0.8)
    for _ in range(2):  # closed loop: cancel the TCP's tracking error over the bail
        yield from arm.wait(0.2)
        g = grippers()[ARMI[side]]
        b = eye_points(k)[1]
        cmd[:2] += (b[:2] - g[:2])
        yield from arm.move(tuple(cmd), yaw=yg, duration=0.4)
    yield from arm.grip(FINGER_CLOSED, 0.5)
    yield from arm.move((cmd[0], cmd[1], LIFT_Z), yaw=yg, duration=0.8)
    return yg


def _lay(arm, ctx, side, k, yg, let_go=True, dest=None):
    """Carry the lifted eye over its place on the deck (closed loop on the live hole centre), lower it, let go."""
    for i in range(3):
        st = ctx.states()
        tgt = _drop_xy(st, k) if dest is None else np.asarray(dest, dtype=float)
        g = grippers()[ARMI[side]]
        h = eye_points(k)[0]
        cmd = g[:2] + (tgt - h[:2])
        yield from arm.move((cmd[0], cmd[1], LIFT_Z), yaw=yg, duration=1.4 if i == 0 else 0.5)
    z_drop = (DECK_Z if dest is None else TOP + DOCK_H) + RING_T / 2 + GRIP_DZ + 0.004
    yield from arm.move((cmd[0], cmd[1], z_drop), yaw=yg, duration=0.8)
    if not let_go:
        return
    yield from arm.grip(FINGER_LET_GO, 0.4)
    yield from arm.move((cmd[0], cmd[1], z_drop + 0.04), yaw=yg, duration=0.3)  # open, straight up off the bail


def _cast_off(arm, ctx, side, k, let_go=True, dest=None):
    yg = yield from _take(arm, ctx, side, k)
    _log(ctx, f"{k} lifted")
    yield from _lay(arm, ctx, side, k, yg, let_go, dest)
    _log(ctx, f"{k} laid")


def _park(arm, side):
    """Sweep back towards the arm's own base and out to the side, clear of the top camera's view of the boat (called
    once the gripper is open and lifted clear of the eye)."""
    s = 1.0 if side == "left" else -1.0
    yield from arm.move((0.08, s * 0.52, TOP + 0.10), yaw=0.0, finger=FINGER_LET_GO, duration=0.7)


def _wait_until(ctx, t_end, tag="wait", every=2.0):
    k = 0
    while _ST["t"] < t_end:
        yield from ctx.left.wait(0.5)
        k += 1
        if k % int(every / 0.5) == 0:
            _log(ctx, tag)


def demo_script(ctx):
    """SPRING_DEMO=correct (default): the left arm lifts the bow line's eye off its post and lays it on the deck while
    the right arm does the same with the stern line, then the stern spring; both arms stand clear; the engine runs
    ahead against the bow spring alone and swings the stern out; after neutral the left arm lifts the bow spring's
    eye off its post and lays it on the deck; both arms stand clear. Variants (all must score 0 unless named a tier):
    none | keep_all_park (no line touched) | keep_stern_spring | keep_stern_line | keep_bow_line | cast_all (all four
    aboard before the engine) | early (bow spring lifted while the engine is still ahead) | hold (correct, but the
    last eye is never let go) | push (correct set-up, then a gripper presses the stern out before the engine) |
    onearm (left arm only: everything it can reach) | tier05 (correct, but the bow spring is left on its post)."""
    mode = os.environ.get("SPRING_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, f"start mode={mode} lines={list(LINES)}")
    if mode == "probe":
        yield from _wait_until(ctx, T_ENG1 + 3.0, "probe", 1.0)
        return
    if mode == "none":
        yield from _wait_until(ctx, T_ENG1 + 6.0)
        _log(ctx, "end")
        return
    if mode == "ropelift":  # right arm alone: its own two lines, then the bow line's rope as far left as it reaches,
        # lifted and pulled to flick the bow line's eye off its post
        yield from _cast_off(Rt, ctx, "right", "stern_line")
        yield from _cast_off(Rt, ctx, "right", "stern_spring")
        st = ctx.states()
        pts = np.asarray(st["bow_line"].points, float)
        i = int(np.argmin(np.abs(pts[:, 1] - 0.40)))
        p = pts[i]
        d = pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]
        yw = _norm_yaw(math.atan2(d[1], d[0]) + math.pi / 2)
        print(f"[ropelift] grab the bow line at {np.round(p, 3).tolist()}", flush=True)
        yield from Rt.move((p[0], p[1], TOP + 0.10), yaw=yw, finger=FINGER_PART, duration=1.5)
        yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=yw, duration=1.0)
        print(f"[ropelift] tcp at grab {np.round(grippers()[1], 3).tolist()}", flush=True)
        from yam_arena.constants import FINGER_CLOSED
        yield from Rt.grip(FINGER_CLOSED, 0.5)
        yield from Rt.move((p[0], p[1], TOP + 0.16), yaw=yw, duration=1.2)
        yield from Rt.move((0.30, 0.15, TOP + 0.12), yaw=yw, duration=1.5)
        _log(ctx, "rope pulled")
        yield from Rt.grip(FINGER_LET_GO, 0.5)
        yield from _park(Rt, "right")
        yield from _wait_until(ctx, T_ENG1 + 6.0)
        _log(ctx, "end")
        return
    if mode == "onearm":
        yield from _cast_off(L, ctx, "left", "bow_line")
        yield from _cast_off(L, ctx, "left", "stern_spring")
        yield from _park(L, "left")
        yield from _wait_until(ctx, T_ENG1 + 6.0)
        _log(ctx, "end")
        return
    left_first = {"keep_bow_line": "bow_spring"}.get(mode, "bow_line")
    right = ["stern_spring", "stern_line"]  # the far eye last, once the left arm has parked
    if mode == "keep_stern_spring":
        right = ["stern_line"]
    elif mode == "keep_stern_line":
        right = ["stern_spring"]

    def right_job():
        for k in right:
            yield from _cast_off(Rt, ctx, "right", k)
        yield from _park(Rt, "right")

    def left_job():
        if mode == "keep_stern_spring":
            yield from _cast_off(L, ctx, "left", "bow_line")
            yield from _cast_off(L, ctx, "left", "bow_spring")
        elif mode in ("cast_all",):
            yield from _cast_off(L, ctx, "left", "bow_line")
            yield from _cast_off(L, ctx, "left", "bow_spring")
        else:
            yield from _cast_off(L, ctx, "left", left_first)
        yield from _park(L, "left")

    yield from ctx.both(left_job(), right_job())
    _log(ctx, "set up")
    if mode == "push":  # press the stern outwards by hand before the engine runs
        st = ctx.states()
        c, Rm, _ = boat_pose(st)
        p = c + Rm @ np.array([-BOAT_L / 2 - 0.01, -0.20, 0.0])
        yield from Rt.move((p[0], p[1], DECK_Z + 0.05), yaw=0.0, duration=1.5)
        yield from Rt.move((p[0], p[1], DECK_Z - 0.005), yaw=0.0, duration=0.8)
        yield from Rt.move((p[0] + 0.05, p[1], DECK_Z - 0.005), yaw=0.0, duration=1.5)
        _log(ctx, "pushed")
        yield from _park(Rt, "right")
    yield from _wait_until(ctx, T_ENG0 + (4.0 if mode == "early" else T_RUN + 1.0), "engine")
    if mode == "tier05":
        yield from _wait_until(ctx, T_ENG1 + 4.0)
        _log(ctx, "end")
        return
    if mode != "cast_all":
        yield from _cast_off(L, ctx, "left", "bow_spring", let_go=(mode != "hold"))
    if mode != "hold":
        yield from _park(L, "left")
    yield from _wait_until(ctx, max(_ST["t"] + 2.0, T_ENG1 + 3.0), "after")
    _log(ctx, "end")
