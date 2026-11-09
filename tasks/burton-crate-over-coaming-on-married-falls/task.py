"""Ship's-hatch mock-up: land a slung crate on the pier pad by hauling one cargo fall while easing the other (a
burton or "married falls" rig), lifting it over the hatch coaming on the way.

Scene (metres from the arm-base midpoint, x forward, y left; heights above the table). A dark hatch square (mat
x 0.27-0.49, |y| <= 0.17) has a grey coaming wall across its middle (x 0.30-0.46, 3 cm thick, 7 cm tall), a grey hold
pad at (0.38, +0.10) and a green pier pad at (0.38, -0.10), each 8 x 8 cm. An orange 4 cm, 80 g crate sits on the hold
pad. Two head sheaves (5 cm groove, axle along x) stand 30 cm up at (0.38, +-0.20) on posts behind them. A blue fall is
shackled to the crate's lifting eye, runs up over the hold-side sheave, straight down to a heel sheave 8 cm up at
(0.349, +0.231), under it and along the deck towards the robot to a red toggle; a yellow fall does the same on the pier
side. The hook end of each fall is fixed to the crate; the hand side is free.

The crate hangs from both falls at once, so its position is set by the two hook-side lengths: hauling the yellow while
easing the blue moves it from the hold side to the pier side; both hauled lifts it, both eased lowers it. With one fall
let go the crate swings or drops under the other sheave. Hauling one fall alone drags the crate across the deck into
and over the coaming (the hold pad is 10 cm from it and the pull is mostly sideways). So the transfer needs both hands
at once: one hauls, the other eases in step, and the crate must clear the coaming top (crate bottom > 7.2 cm).

Hands must stay out of the hatch square and off the crate (that is how slung cargo is worked: by the falls).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants: BURTON_DEMO=correct (default) | none | hold | onearm_yellow | onearm_blue | drop | low |
twoblock | replace | carry | gather | tier03 | tier06.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the falls below are simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("This crate has to go from the hold side to the pier: use the two cargo falls to lift it over the "
               "coaming and land it on the green pad, keeping your hands off the load and out of the dark hatch square. "
               "Then let go of both falls.")

GREY = (0.55, 0.56, 0.58)
DARK = (0.22, 0.23, 0.25)
MAT_COL = (0.32, 0.33, 0.36)
PAD_A_COL = (0.62, 0.63, 0.66)
PAD_B_COL = (0.15, 0.62, 0.25)
CRATE_COL = (0.95, 0.48, 0.08)
BLUE = (0.10, 0.30, 0.85)
YELLOW = (0.95, 0.82, 0.10)
RED = (0.85, 0.08, 0.06)

# -- hatch: square mat, coaming wall, two pads ----------------------------------------------------------------------
PX = 0.38  # plane of the rig (x): pads, crate, sheaves
MAT_X = (0.27, 0.49)
MAT_Y = 0.17  # hatch square |y| <= 0.17 (hands may not enter it)
MAT_H = 0.002
MAT = Box("hatch_square", pos=((MAT_X[0] + MAT_X[1]) / 2, 0.0, TOP + MAT_H / 2),
          size=(MAT_X[1] - MAT_X[0], 2 * MAT_Y, MAT_H), color=MAT_COL)
COAM_X = (PX - 0.08, PX + 0.08)
COAM_T, COAM_H = 0.03, 0.07
COAMING = Box("coaming", pos=(PX, 0.0, TOP + MAT_H + COAM_H / 2), size=(COAM_X[1] - COAM_X[0], COAM_T, COAM_H),
              color=GREY)
PAD_Y = 0.10
PAD_S, PAD_H = 0.08, 0.003
PAD_TOP = TOP + MAT_H + PAD_H
PAD_A = Box("pad_hold", pos=(PX, PAD_Y, TOP + MAT_H + PAD_H / 2), size=(PAD_S, PAD_S, PAD_H), color=PAD_A_COL)
PAD_B = Box("pad_pier", pos=(PX, -PAD_Y, TOP + MAT_H + PAD_H / 2), size=(PAD_S, PAD_S, PAD_H), color=PAD_B_COL)

# -- crate ------------------------------------------------------------------------------------------------------------
CR = 0.04
CRATE_M = 0.08
CRATE_Z = PAD_TOP + CR / 2 + 0.0005
CRATE = Box("crate", pos=(PX, PAD_Y, CRATE_Z), size=(CR, CR, CR), mass=CRATE_M, color=CRATE_COL)
CRATE_SPIN_DAMPING = 3.0  # 1/s: swivel and air friction on the crate's spin

# -- sheaves (built in the spawn hook as horizontal cylinders along x) on posts behind them -------------------------
SH_Y = 0.20
SH_Z = 0.30  # sheave axle height above the table
HUB_R, HUB_W = 0.025, 0.012
RIM_R, RIM_W = 0.042, 0.004
AXLE_R = 0.005
POST_DX = 0.06
POST_R = 0.010
POSTS = [Cylinder(f"boom_post_{s}", pos=(PX + POST_DX, sg * SH_Y, TOP + SH_Z / 2), radius=POST_R, height=SH_Z,
                  color=DARK) for s, sg in (("hold", 1), ("pier", -1))]
HEEL_AXLE = 0.035  # heel axle from the sheave out to its block
SHEAVE_C = {+1: np.array([PX, SH_Y, TOP + SH_Z]), -1: np.array([PX, -SH_Y, TOP + SH_Z])}
SHEAVE_MU = 0.1  # smooth sheaves, posts, heel blocks and coaming

# -- falls --------------------------------------------------------------------------------------------------------------
R = 0.003
ATT_DY = 0.005  # the two falls' shackles on the lifting eye, 1 cm apart
ATT_DZ = CR / 2 + R + 0.0005
TOGGLE = Plug(size=(0.024, 0.020, 0.012), mass=0.006, color=RED)
LINE_KW = dict(radius=R, segment=0.02, bend_stiffness=0.004, bend_damping=0.0005, friction=0.6)
RC = HUB_R + R / math.cos(math.pi / 8) + 0.0015 + 0.02 ** 2 / (8 * (HUB_R + 0.005))  # rope centreline radius on a sheave
FALL_Y = SH_Y + RC  # the falls drop from the head sheaves' outboard side at |y| = FALL_Y
HEEL_X = PX - RC  # heel (lead) sheaves: axle along y, at x HEEL_X, |y| FALL_Y, height HEEL_Z
HEEL_Z = 0.08
RUN_Z = HEEL_Z - RC  # the hauling part leaves the heel sheave towards the robot at this height
HEEL_C = {+1: np.array([HEEL_X, FALL_Y, TOP + HEEL_Z]), -1: np.array([HEEL_X, -FALL_Y, TOP + HEEL_Z])}
TOG_X = 0.10  # toggle (tail end) on the deck


def _fall_path(sign: int, crate_y: float = PAD_Y) -> np.ndarray:
    """Centreline of a fall at the start: up from its shackle to the head sheave, over it, straight down to the heel
    sheave, round under it towards the robot, then along the deck to the toggle. sign +1: blue (hold-side sheave);
    -1: yellow (pier-side sheave)."""
    att = np.array([PX, crate_y + sign * ATT_DY, CRATE_Z + ATT_DZ])
    c = SHEAVE_C[sign]
    u_h, v_h = sign * (att[1] - c[1]), att[2] - c[2]  # head-sheave plane: u outboard, v up
    d = math.hypot(u_h, v_h)
    th, a = math.atan2(v_h, u_h), math.acos(RC / d)
    p1 = th - a
    if math.cos(p1) > 0:
        p1 = th + a
    p1 = p1 % (2 * math.pi)  # inboard tangent point, between 90 and 270 deg
    head = [np.array([PX, c[1] + sign * RC * math.cos(t), c[2] + RC * math.sin(t)])
            for t in np.linspace(p1, 0.0, max(4, int(p1 * RC / 0.008)))]
    hc = HEEL_C[sign]
    heel = [np.array([hc[0] + RC * math.cos(t), hc[1], hc[2] + RC * math.sin(t)])
            for t in np.linspace(0.0, -math.pi / 2, 6)]
    run = [heel[-1], np.array([HEEL_X - 0.10, hc[1], TOP + RUN_Z]), np.array([HEEL_X - 0.18, hc[1], TOP + R + 0.001]),
           np.array([TOG_X, hc[1], TOP + R + 0.001])]
    for _ in range(3):  # round the bends (Chaikin), ends kept
        run = [run[0]] + [p for q0, q1 in zip(run[:-1], run[1:]) for p in (0.75 * q0 + 0.25 * q1, 0.25 * q0 + 0.75 * q1)][1:-1] + [run[-1]]
    return np.vstack([att, *head, *heel[1:-1], *run])


def _path_len(p: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


BLUE_LEN = round(_path_len(_fall_path(+1)) - TOGGLE.size[0], 2)
YELLOW_LEN = round(_path_len(_fall_path(-1)) - TOGGLE.size[0], 2)
_SPAWN_Z = CRATE_Z + ATT_DZ
BLUE_FALL = cable("blue_fall", start=(PX, PAD_Y + ATT_DY, _SPAWN_Z), yaw=0.0, length=BLUE_LEN, end_plug=TOGGLE,
                  color=BLUE, cable_mass=0.025 * BLUE_LEN, **LINE_KW)
YELLOW_FALL = cable("yellow_fall", start=(PX, PAD_Y - ATT_DY, _SPAWN_Z), yaw=0.0, length=YELLOW_LEN, end_plug=TOGGLE,
                    color=YELLOW, cable_mass=0.025 * YELLOW_LEN, **LINE_KW)
FALLS = {+1: BLUE_FALL, -1: YELLOW_FALL}

HEEL_BLOCK_Y = FALL_Y + HUB_W / 2 + HEEL_AXLE + 0.006
HEEL_BLOCKS = [Box(f"heel_block_{s}", pos=(HEEL_X, sg * HEEL_BLOCK_Y, TOP + (HEEL_Z + 0.012) / 2),
                   size=(0.030, 0.012, HEEL_Z + 0.012), color=DARK) for s, sg in (("hold", 1), ("pier", -1))]
FIXED = [MAT, COAMING, PAD_A, PAD_B, *POSTS, *HEEL_BLOCKS]
OBJECTS = [*FIXED, CRATE, BLUE_FALL, YELLOW_FALL]

TWIN_EXTRA = [
    {"name": "head_sheave_hold", "kind": "body", "size_mm": [20, 84, 84], "colour": "grey", "pose": SHEAVE_C[1].tolist(),
     "real": "50 mm nylon sheave (12 mm groove, 84 mm cheeks) on an M5 axle cantilevered 6 cm from the post",
     "note": "sim: fixed smooth wheel (friction 0.1) standing in for a free-turning sheave"},
    {"name": "head_sheave_pier", "kind": "body", "size_mm": [20, 84, 84], "colour": "grey",
     "pose": SHEAVE_C[-1].tolist(), "real": "same as head_sheave_hold"},
    {"name": "heel_sheave_hold", "kind": "body", "size_mm": [84, 20, 84], "colour": "grey", "pose": HEEL_C[1].tolist(),
     "real": "same sheave, axle along y, on a 3.5 cm axle from its block", "note": "sim: fixed smooth wheel"},
    {"name": "heel_sheave_pier", "kind": "body", "size_mm": [84, 20, 84], "colour": "grey",
     "pose": HEEL_C[-1].tolist(), "real": "same as heel_sheave_hold"},
    {"name": "fall_shackles", "kind": "joint", "real": "two small swivel shackles on the crate's lifting eye, 1 cm apart",
     "note": "sim: ball joints; crate spin damped 3/s"},
]


def _install_spawn_hook() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_burton_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        name = getattr(cfg, "cable_name", "")
        if name not in (BLUE_FALL.name, YELLOW_FALL.name):
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.schemas import CollisionPropertiesCfg
        from isaaclab.sim.utils import bind_physics_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, Usd, UsdPhysics

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        crate_path = f"{parent}/{CRATE.name}"
        spec = BLUE_FALL if name == BLUE_FALL.name else YELLOW_FALL
        dy = ATT_DY if spec is BLUE_FALL else -ATT_DY
        j = UsdPhysics.Joint.Define(stage, f"{prim_path}/shackle")
        j.CreateBody0Rel().SetTargets([Sdf.Path(crate_path)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, float(dy), float(ATT_DZ)))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(-spec.seg_len / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        for dof in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        j.CreateExcludeFromArticulationAttr(True)
        if spec is not YELLOW_FALL:
            return prim
        crate_prim = stage.GetPrimAtPath(crate_path)
        if crate_prim.IsValid():
            PhysxSchema.PhysxRigidBodyAPI.Apply(crate_prim).CreateAngularDampingAttr(CRATE_SPIN_DAMPING)
        look = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.60, 0.61, 0.63), roughness=0.4)
        rim_look = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.20, 0.21, 0.23), roughness=0.4)
        mat = f"{parent}/sheave_smooth_material"
        mc = sim_utils.RigidBodyMaterialCfg(static_friction=SHEAVE_MU, dynamic_friction=SHEAVE_MU,
                                            friction_combine_mode="min")
        mc.func(mat, mc)
        for side, c in (("hold", SHEAVE_C[1]), ("pier", SHEAVE_C[-1])):
            parts = [(f"sheave_{side}_hub", HUB_R, HUB_W, 0.0, look),
                     (f"sheave_{side}_rim_a", RIM_R, RIM_W, HUB_W / 2 + RIM_W / 2, rim_look),
                     (f"sheave_{side}_rim_b", RIM_R, RIM_W, -(HUB_W / 2 + RIM_W / 2), rim_look),
                     (f"sheave_{side}_axle", AXLE_R, POST_DX, POST_DX / 2, look)]
            sg = 1 if side == "hold" else -1
            hc = HEEL_C[sg]
            heel = [(f"heel_{side}_hub", HUB_R, HUB_W, 0.0, look), (f"heel_{side}_rim_a", RIM_R, RIM_W, HUB_W / 2 + RIM_W / 2, rim_look),
                    (f"heel_{side}_rim_b", RIM_R, RIM_W, -(HUB_W / 2 + RIM_W / 2), rim_look),
                    (f"heel_{side}_axle", AXLE_R, HEEL_AXLE, HUB_W / 2 + HEEL_AXLE / 2, look)]
            for (nm, rad, wid, dx, lk), ax, cen in [(p_, "X", c) for p_ in parts] + [(p_, "Y", None) for p_ in heel]:
                cc = sim_utils.CylinderCfg(radius=rad, height=wid, axis=ax, collision_props=CollisionPropertiesCfg(),
                                           visual_material=lk)
                if ax == "X":
                    tr = (float(c[0] + dx), float(c[1]), float(c[2]))
                else:
                    tr = (float(hc[0]), float(hc[1] + sg * dx), float(hc[2]))
                cc.func(f"{parent}/{nm}", cc, translation=tr)
                for p in Usd.PrimRange(stage.GetPrimAtPath(f"{parent}/{nm}")):
                    if p.HasAPI(UsdPhysics.CollisionAPI):
                        bind_physics_material(str(p.GetPath()), mat, stage=stage)
        for nm in [o.name for o in (*POSTS, *HEEL_BLOCKS, COAMING)]:  # smooth painted rig parts: no belaying a fall on them
            for p in Usd.PrimRange(stage.GetPrimAtPath(f"{parent}/{nm}")):
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    bind_physics_material(str(p.GetPath()), mat, stage=stage)
        return prim

    spawn._burton_hook = True
    cablekit._spawn_cable = spawn


_install_spawn_hook()


# -- start layout: crate put back on pad A, both falls laid over their sheaves -----------------------------------------
def _shape(emb) -> None:
    import torch

    env = getattr(emb, "_env", emb)
    base = env.unwrapped
    dev = base.device
    origin = getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0]
    o = base.scene[CRATE.name]
    ids = torch.tensor([0], device=dev)
    pose = torch.tensor([[*CRATE.pos, 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32, device=dev)
    pose[0, :3] += origin
    try:
        o.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=dev), env_ids=ids)
    except Exception as e:  # pragma: no cover
        print(f"[burton] crate pose write failed: {e!r}", flush=True)
    for sign, spec in FALLS.items():
        cableshape.preshape(emb, spec, _fall_path(sign), settle_s=0, hold="none", lift=0.0)
    cableshape.settle(emb, 2.0)


cableshape.on_reset(_shape)

# -- monitors: crate pose/velocity and robot-crate contact polled every physics substep (240 Hz) ------------------------
TOUCH_SEP = 0.001
COAM_GAP = 0.003
DROP_VZ = 0.40
JERK_V = 0.60
TWO_BLOCK = 0.08  # crate top centre within 8 cm of a sheave axle (lid within ~4 cm of the cheeks)
_ROBOT_PRIMS = ("/LeftArm/", "/RightArm/")
_MON: dict = {"view": None, "body": None, "polls": 0, "err": None}
_LATCH: dict = {}


def _reset_latches() -> None:
    _LATCH.clear()
    _LATCH.update(nan=False, robot_touch=False, coaming=False, drop=False, jerk=False, two_block=False,
                  hand_in_hatch=False, hand_near_crate=False, one_hand_two_falls=False, parted=False, vmax=0.0,
                  vzmax=0.0)


_reset_latches()


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def _quat_mat(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _sep_obb_aabb(c, rot, bc, bh) -> float:
    ha = np.full(3, CR / 2)
    A = [rot[:, i] for i in range(3)]
    B = [np.eye(3)[i] for i in range(3)]
    axes = A + B + [np.cross(a, b) for a in A for b in B]
    d = bc - c
    best = -np.inf
    for ax in axes:
        n = np.linalg.norm(ax)
        if n < 1e-9:
            continue
        ax = ax / n
        ra = sum(ha[i] * abs(np.dot(A[i], ax)) for i in range(3))
        rb = sum(bh[i] * abs(np.dot(B[i], ax)) for i in range(3))
        best = max(best, abs(np.dot(d, ax)) - ra - rb)
    return float(best)


_COAM_BOX = (np.array(COAMING.pos), np.array(COAMING.size) / 2)


def coaming_gap(c, rot) -> float:
    return _sep_obb_aabb(c, rot, *_COAM_BOX)


def _check_pose(c, rot, v) -> None:
    if coaming_gap(c, rot) <= COAM_GAP and not _LATCH["coaming"]:
        _LATCH["coaming"] = True
        print(f"[burton] crate touched the coaming at {np.round(c - [0, 0, TOP], 3)}", flush=True)
    top = c + rot[:, 2] * (CR / 2)
    if min(float(np.linalg.norm(top - s)) for s in SHEAVE_C.values()) < TWO_BLOCK and not _LATCH["two_block"]:
        _LATCH["two_block"] = True
        print(f"[burton] two-blocked at {np.round(c - [0, 0, TOP], 3)}", flush=True)
    sp, vz = float(np.linalg.norm(v)), abs(float(v[2]))
    _LATCH["vmax"] = max(_LATCH["vmax"], sp)
    _LATCH["vzmax"] = max(_LATCH["vzmax"], vz)
    if vz > DROP_VZ and not _LATCH["drop"]:
        _LATCH["drop"] = True
        print(f"[burton] crate dropped/jerked: vz {vz:.2f} m/s at {np.round(c - [0, 0, TOP], 3)}", flush=True)
    if sp > JERK_V and not _LATCH["jerk"]:
        _LATCH["jerk"] = True
        print(f"[burton] crate jerked: {sp:.2f} m/s at {np.round(c - [0, 0, TOP], 3)}", flush=True)


def _enable_contact_processing() -> None:
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # pragma: no cover
        pass


def _make_views(mgr) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    sv = mgr.get_physics_sim_view()
    if sv is None:
        return
    stage = get_current_stage()
    sensors, crate = [], None
    for prim in stage.Traverse():
        p = prim.GetPath().pathString
        if "/env_0/" not in p:
            continue
        if any(r in p for r in _ROBOT_PRIMS) and "PhysxContactReportAPI" in prim.GetAppliedSchemas():
            sensors.append(p)
        elif p.endswith(f"/{CRATE.name}") and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            crate = p
    if not sensors or crate is None:
        raise RuntimeError(f"crate monitor: {len(sensors)} robot links, crate={crate}")
    _MON["view"] = sv.create_rigid_contact_view(sensors, filter_patterns=[[crate]] * len(sensors),
                                                max_contact_data_count=64 * len(sensors))
    _MON["body"] = sv.create_rigid_body_view(crate)
    _MON["stage"] = stage
    print(f"[burton] monitors: {len(sensors)} robot links x {crate}", flush=True)


def _poll(mgr) -> None:
    if not _LATCH.get("armed", False):
        return
    try:
        if _MON["view"] is None or _MON.get("stage") is None:
            _make_views(mgr)
        if _MON["view"] is None:
            return
        _, _, _, sep, counts, starts = _MON["view"].get_contact_data(dt=mgr.get_physics_dt())
        tf = _np(_MON["body"].get_transforms()).reshape(-1)[:7].astype(np.float64)
        vel = _np(_MON["body"].get_velocities()).reshape(-1)[:6].astype(np.float64)
    except Exception as e:  # pragma: no cover
        if _MON["err"] is None:
            _MON["err"] = repr(e)
            print(f"[burton] monitor error: {e!r}", flush=True)
        return
    _MON["polls"] += 1
    counts = _np(counts).reshape(-1)
    if counts.any() and not _LATCH["robot_touch"]:
        starts, sep = _np(starts).reshape(-1), _np(sep).reshape(-1)
        idx = np.concatenate([np.arange(s, s + c) for s, c in zip(starts[counts > 0], counts[counts > 0])])
        if float(sep[idx].min()) <= TOUCH_SEP:
            _LATCH["robot_touch"] = True
            print("[burton] a gripper/arm touched the crate", flush=True)
    if not (np.all(np.isfinite(tf)) and np.all(np.isfinite(vel))):
        _LATCH["nan"] = True
        return
    origin = _MON.get("origin", np.zeros(3))
    _check_pose(tf[:3] - origin, _quat_mat(tf[3:7]), vel[:3])


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_burton_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__
    base_step = mgr.step.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        _enable_contact_processing()

    def step(cls) -> None:
        base_step(cls)
        _poll(cls)

    step._burton_hooked = True
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


_ENV: dict = {"env": None}


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
    if not getattr(cls.reset, "_burton_hooked", False):
        base_reset = cls.reset  # cableshape's reset (shaping + settle) runs inside this call

        def reset(self, *a, **k):
            _MON["view"] = None
            _LATCH["armed"] = False
            out = base_reset(self, *a, **k)
            try:
                _MON["origin"] = _np(self._env.unwrapped.scene.env_origins[0]).astype(np.float64)
            except Exception:  # pragma: no cover
                pass
            _reset_latches()
            _LATCH["armed"] = True
            return out

        reset._burton_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_burton_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._burton_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[tuple[np.ndarray, np.ndarray, float]]:
    """Per arm (left, right): (TCP, pad tips (2,3), |finger joint| m); [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import pad_tips_world, tcp_world

    base = env.unwrapped
    origin = _np(base.scene.env_origins[0]).astype(np.float64)
    out = []
    for k in ("left_robot", "right_robot"):
        art = base.scene[k]
        jp = _np(getattr(art.data.joint_pos, "torch", art.data.joint_pos))[0]
        f = float(abs(jp[list(art.joint_names).index("joint7")]))
        out.append((np.asarray(tcp_world(art), dtype=np.float64) - origin,
                    np.asarray(pad_tips_world(art), dtype=np.float64).reshape(-1, 3) - origin, f))
    return out


# -- geometry on states --------------------------------------------------------------------------------------------------
def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * (k / n) for k in range(1, n + 1)]
    return np.asarray(out)


def crate_pose(objects):
    s = objects[CRATE.name]
    return np.asarray(s.pos, dtype=float), _quat_mat(np.asarray(s.quat_xyzw, dtype=float))


def tilt_deg(rot) -> float:
    return math.degrees(math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))))


def dist_to_crate(p, c, rot) -> float:
    q = rot.T @ (np.asarray(p) - c)
    return float(np.linalg.norm(np.maximum(np.abs(q) - CR / 2, 0.0)))


def _fall_pts(objects, spec) -> np.ndarray:
    cs = objects[spec.name]
    pts = [_dense(cs.polyline())]
    if cs.end_plug is not None:
        pts.append(np.asarray(cs.end_plug)[None])
    return np.vstack(pts)


def _finite(objects) -> bool:
    return bool(all(np.isfinite(objects[s.name].points).all() for s in FALLS.values())
                and np.isfinite(objects[CRATE.name].pos).all())


HATCH_MARGIN = 0.0
NEAR_CRATE = 0.03
ONE_HAND = 0.025
RELEASE = 0.04
PARTED = 0.01


def _track(objects) -> None:
    if not _finite(objects):
        _LATCH["nan"] = True
        return
    g = grippers()
    if not g:
        return
    c, rot = crate_pose(objects)
    bp, yp = _fall_pts(objects, BLUE_FALL), _fall_pts(objects, YELLOW_FALL)
    for spec, gap in zip(FALLS.values(), shackle_gaps(objects)):
        if gap > PARTED and not _LATCH["parted"]:
            _LATCH["parted"] = True
            print(f"[burton] the {spec.name} parted at its shackle (gap {gap:.3f} m)", flush=True)
    for tcp, tips, _ in g:
        pts = np.vstack([tcp[None], tips])
        for p in pts:
            if MAT_X[0] <= p[0] <= MAT_X[1] and abs(p[1]) <= MAT_Y and not _LATCH["hand_in_hatch"]:
                _LATCH["hand_in_hatch"] = True
                print(f"[burton] a hand entered the hatch square at {np.round(p - [0, 0, TOP], 3)}", flush=True)
            if dist_to_crate(p, c, rot) < NEAR_CRATE and not _LATCH["hand_near_crate"]:
                _LATCH["hand_near_crate"] = True
                print("[burton] a hand came within 3 cm of the crate", flush=True)
        if (float(np.linalg.norm(bp - tcp, axis=1).min()) < ONE_HAND
                and float(np.linalg.norm(yp - tcp, axis=1).min()) < ONE_HAND and not _LATCH["one_hand_two_falls"]):
            _LATCH["one_hand_two_falls"] = True
            print("[burton] one hand on both falls", flush=True)


def shackle_gaps(objects) -> list[float]:
    c, rot = crate_pose(objects)
    return [float(np.linalg.norm(np.asarray(objects[spec.name].start_tip) - (c + rot @ np.array([0.0, sign * ATT_DY, ATT_DZ]))))
            for sign, spec in FALLS.items()]


def failure(objects) -> str | None:
    _track(objects)
    if _LATCH["nan"] or not _finite(objects):
        return "nan"
    for k in ("coaming", "drop", "jerk", "two_block", "robot_touch", "hand_in_hatch", "hand_near_crate",
              "one_hand_two_falls", "parted"):
        if _LATCH[k]:
            return k
    return None


def released(objects) -> bool:
    c, rot = crate_pose(objects)
    pts = np.vstack([_fall_pts(objects, s) for s in FALLS.values()])
    for tcp, tips, _ in grippers():
        for p in np.vstack([tcp[None], tips]):
            if float(np.linalg.norm(pts - p, axis=1).min()) < RELEASE or dist_to_crate(p, c, rot) < RELEASE:
                return False
    return True


def landed(objects) -> bool:
    c, rot = crate_pose(objects)
    low = float((c[2] - np.abs(rot[2, :]).sum() * CR / 2))
    return low <= PAD_TOP + 0.006 and tilt_deg(rot) < 15.0


PIER_Y = -0.04
PAD_B_XY = np.array([PX, -PAD_Y])


def score(objects) -> float:
    """0 if at any time: the crate came within 3 mm of the coaming; it moved faster than 0.6 m/s or 0.4 m/s
    vertically (dropped, jerked or swung loose); its lid came within 8 cm of a head sheave's axle (two-blocked); a
    robot part touched it, or a gripper (TCP or pad tip) came within 3 cm of it or entered the hatch square; one gripper
    was within 2.5 cm of both falls at once; or a state went non-finite. Also 0 unless at the end both grippers are 4 cm
    or more from the crate and both falls (let go), and the crate rests on the deck (lowest corner <= 6 mm above the pad
    top) upright (tilt < 15 deg) with its centre at y <= -4 cm (pier side). Then: 1.0 if its centre is within 1.5 cm of
    the green pad's centre and tilt < 10 deg; 0.6 if within 3 cm; else 0.3."""
    if failure(objects) or not released(objects) or not landed(objects):
        return 0.0
    c, rot = crate_pose(objects)
    if c[1] > PIER_Y:
        return 0.0
    off = float(np.linalg.norm(c[:2] - PAD_B_XY))
    if off <= 0.015 and tilt_deg(rot) < 10.0:
        return 1.0
    if off <= 0.03:
        return 0.6
    return 0.3


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and float(np.linalg.norm(objects[CRATE.name].lin_vel)) < 0.01)


def distance(objects) -> float:
    if failure(objects):
        return 1.0
    c, _ = crate_pose(objects)
    return float(np.linalg.norm(c[:2] - PAD_B_XY))


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    if not _finite(objects):
        return {}
    c, rot = crate_pose(objects)
    low = float(c[2] - np.abs(rot[2, :]).sum() * CR / 2)
    return {"lifted": float(low > PAD_TOP + 0.01 or c[1] < PIER_Y), "over_coaming": float(c[1] < 0.0),
            "landed_pier_side": float(landed(objects) and c[1] <= PIER_Y),
            "on_pad": float(landed(objects) and float(np.linalg.norm(c[:2] - PAD_B_XY)) <= 0.03),
            "released": float(released(objects)), "clean": float(failure(objects) is None)}


VARIATION = {"obj:crate": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}  # shackled to both falls laid over the sheaves: fixed


# -- demo ----------------------------------------------------------------------------------------------------------------
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402

GRASP_OPEN = -0.016


def _hook(st) -> np.ndarray:
    c, rot = crate_pose(st)
    return c + rot[:, 2] * ATT_DZ


def _log(ctx, tag=""):
    st = ctx.states()
    if not _finite(st):
        print(f"[t={ctx.t:5.2f}] {tag}: NON-FINITE", flush=True)
        return
    c, rot = crate_pose(st)
    h = _hook(st)
    d = {s: float(np.linalg.norm(h - SHEAVE_C[s])) for s in (1, -1)}
    print(f"[t={ctx.t:5.2f}] {tag}: crate={np.round(c - [0, 0, TOP], 3)} tilt={tilt_deg(rot):.1f} "
          f"v={np.linalg.norm(st[CRATE.name].lin_vel):.3f} dB={d[1]:.3f} dY={d[-1]:.3f} "
          f"L={np.round(ctx.left.tcp() - [0, 0, TOP], 3)} R={np.round(ctx.right.tcp() - [0, 0, TOP], 3)} "
          f"fail={failure(st)} rel={released(st)} score={score(st):.2f} vmax={_LATCH['vmax']:.2f} "
          f"vz={_LATCH['vzmax']:.2f} polls={_MON['polls']} gaps={np.round(shackle_gaps(st), 4)}", flush=True)


# hauling: each hand pinches its fall on the run between the heel sheave and the robot; a = run length from the
# heel sheave's bottom to the hand (bigger a = hauled in)
A_MIN, A_MAX = 0.05, 0.29
GRIP_DZ = 0.011


def _hand_pos(sign, a):
    a = float(np.clip(a, A_MIN, A_MAX))
    return (HEEL_X - a, sign * FALL_Y, TOP + RUN_Z + GRIP_DZ)


def _grasp(arm, ctx, sign, x):
    """Pinch the fall's run at x (closed loop on the live rope), fingers across it, then bring it to run height."""
    spec = FALLS[sign]
    pts = ctx.states()[spec.name].points
    i = int(np.argmin(np.abs(pts[:, 0] - x) + 10 * np.abs(pts[:, 1] - sign * FALL_Y)))
    p = pts[i]
    yield from arm.move((p[0], p[1], TOP + 0.12), yaw=0.0, finger=GRASP_OPEN, duration=1.5)
    p = ctx.states()[spec.name].points[i]
    yield from arm.move((p[0], p[1], p[2] + 0.04), yaw=0.0, duration=0.6)
    p = ctx.states()[spec.name].points[i]
    yield from arm.move((p[0], p[1], p[2] + 0.010), yaw=0.0, duration=0.8)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    a = HEEL_X - float(arm.tcp()[0])
    yield from arm.move(_hand_pos(sign, a), yaw=0.0, duration=0.6)
    print(f"[burton] {spec.name} pinched at seg {i} {np.round(p - [0, 0, TOP], 3)} a={a:.3f}", flush=True)


def _held(ctx, sign, arm) -> float:
    pts = ctx.states()[FALLS[sign].name].points
    return float(np.linalg.norm(pts - arm.tcp(), axis=1).min())


STOW = {1: (0.10, 0.40, TOP + 0.12), -1: (0.10, -0.40, TOP + 0.12)}
DT = 0.1


def _path(h0, wps):
    tgt = [np.array([PX, y, TOP + z]) for y, z in wps]
    out = []
    for a_, b_ in zip(tgt[:-1], tgt[1:]):
        n = max(2, int(np.linalg.norm(b_ - a_) / 0.003))
        out += [a_ + (b_ - a_) * t for t in np.linspace(0, 1, n)[1:]]
    return out


def _work(ctx, hand, dense, tend=(1, -1), extra=30, drop_at=None):
    """Closed loop on the live crate: each step, each tended fall is hauled or eased so the hook's distance from
    that fall's head sheave matches the target hook point (a fall that is too long is hauled in, too short eased)."""
    arms = {1: ctx.left, -1: ctx.right}
    tend = list(tend)
    for k in range(len(dense) + extra):
        hs = dense[min(k, len(dense) - 1)]
        st = ctx.states()
        if not _finite(st):
            _log(ctx, "non-finite")
            return
        if drop_at is not None and k == int(drop_at * len(dense)):
            yield from arms[1].grip(FINGER_OPEN, 0.3)
            tend.remove(1)
            _log(ctx, "blue let go while loaded")
            st = ctx.states()
        h = _hook(st)
        for s in tend:
            d = float(np.linalg.norm(h - SHEAVE_C[s]))
            ds = float(np.linalg.norm(hs - SHEAVE_C[s]))
            hand[s] = float(np.clip(hand[s] + float(np.clip(0.6 * (d - ds), -0.006, 0.006)), A_MIN, A_MAX))
        gens = [arms[s].move(_hand_pos(s, hand[s]), yaw=0.0, duration=DT) for s in tend]
        if len(gens) == 2:
            yield from ctx.both(*gens)
        elif gens:
            yield from gens[0]
        else:
            yield from ctx.left.wait(DT)
        if k % 10 == 0:
            _log(ctx, f"k={k} hs=({hs[1]:.3f},{hs[2] - TOP:.3f}) aB={hand[1]:.3f} aY={hand[-1]:.3f}")


def _let_go(arm, sign):
    """Open, lift 4 cm straight up, then sweep back beside the arm's own base, out of the top camera's view."""
    yield from arm.grip(GRASP_OPEN, 0.4)
    p = arm.tcp()
    yield from arm.move((p[0], p[1], p[2] + 0.04), yaw=0.0, duration=0.4)
    yield from arm.move(STOW[sign], yaw=0.0, finger=FINGER_CLOSED, duration=0.8)


def _settle_crate(ctx, tmax=4.0):
    t = 0.0
    while t < tmax and float(np.linalg.norm(ctx.states()[CRATE.name].lin_vel)) > 0.005:
        yield from ctx.left.wait(0.25)
        t += 0.25


def demo_script(ctx):
    """BURTON_DEMO=correct (default) | none | hold | onearm_yellow | onearm_blue | drop | low | twoblock | replace |
    carry | gather | tier03 | tier06 (test-only variants; see README)."""
    mode = os.environ.get("BURTON_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "none":
        for _ in range(10):
            yield from L.wait(0.5)
        _log(ctx, "end")
        return
    if mode == "carry":  # the left hand takes the crate itself and carries it over (hands off the load: 0)
        c = ctx.states()[CRATE.name].pos
        yield from L.move((c[0], c[1], TOP + 0.15), yaw=0.0, finger=FINGER_OPEN, duration=1.5)
        yield from L.move((c[0], c[1], c[2] + 0.004), yaw=0.0, duration=1.2)
        yield from L.grip(FINGER_CLOSED, 0.6)
        yield from L.move((c[0], c[1], TOP + 0.13), yaw=0.0, duration=1.0)
        yield from L.move((PX, -0.02, TOP + 0.13), yaw=0.0, duration=1.5)
        yield from L.move((PX, -0.02, TOP + 0.05), yaw=0.0, duration=1.0)
        yield from L.grip(FINGER_OPEN, 0.4)
        yield from L.move(STOW[1], yaw=0.0, duration=1.0)
        yield from _settle_crate(ctx)
        _log(ctx, "end")
        return
    if mode == "gather":  # one hand takes both falls together (yellow carried over to blue) and hauls them
        yield from _grasp(Rt, ctx, -1, 0.12)
        yield from Rt.move((0.12, -0.10, TOP + 0.06), yaw=0.0, duration=2.0)
        yield from Rt.move((0.12, 0.10, TOP + 0.06), yaw=0.0, duration=2.0)
        pts = ctx.states()[BLUE_FALL.name].points
        i = int(np.argmin(np.abs(pts[:, 0] - 0.14) + 10 * np.abs(pts[:, 1] - FALL_Y)))
        p = pts[i]
        yield from Rt.grip(GRASP_OPEN, 0.4)
        yield from Rt.move((p[0], p[1] - 0.01, TOP + 0.06), yaw=0.0, duration=1.5)
        yield from Rt.move((p[0], p[1] - 0.005, p[2] + 0.010), yaw=0.0, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED, 0.6)
        _log(ctx, "gathered")
        yield from Rt.move((0.06, 0.20, TOP + 0.06), yaw=0.0, duration=3.0)
        yield from Rt.move((0.06, 0.05, TOP + 0.10), yaw=0.0, duration=3.0)
        _log(ctx, "hauled both")
        yield from Rt.grip(GRASP_OPEN, 0.4)
        yield from Rt.move(STOW[-1], yaw=0.0, duration=1.0)
        yield from _settle_crate(ctx)
        _log(ctx, "end")
        return
    one = mode in ("onearm_yellow", "onearm_blue")
    tend = (-1,) if mode == "onearm_yellow" else (1,) if mode == "onearm_blue" else (1, -1)
    gens = []
    if 1 in tend:
        gens.append(_grasp(L, ctx, +1, 0.16))
    if -1 in tend:
        gens.append(_grasp(Rt, ctx, -1, 0.29))
    if len(gens) == 2:
        yield from ctx.both(*gens)
    else:
        yield from gens[0]
    _log(ctx, "grasped")
    hand = {1: HEEL_X - float(L.tcp()[0]), -1: HEEL_X - float(Rt.tcp()[0])}
    h0 = _hook(ctx.states())
    zl = h0[2] - TOP
    land_y = {"tier03": -0.056, "tier06": -0.123, "replace": h0[1]}.get(mode, -PAD_Y)
    if mode == "low":  # drag it across low: crate bottom below the coaming top
        wps = [(h0[1], zl), (h0[1], zl + 0.02), (0.0, 0.10), (land_y, zl + 0.02), (land_y, zl - 0.01)]
    elif mode == "twoblock":  # over-haul both falls: lid up into the sheaves
        wps = [(h0[1], zl), (h0[1], zl + 0.04), (0.06, 0.20), (0.0, 0.26)]
    elif mode == "replace":
        wps = [(h0[1], zl), (h0[1], zl + 0.05), (0.06, 0.12), (h0[1], zl + 0.04), (h0[1], zl - 0.01)]
    elif mode == "onearm_yellow":  # haul yellow until its hand is at the end of its reach
        wps = [(h0[1], zl), (-PAD_Y, zl + 0.30)]
    elif mode == "onearm_blue":
        wps = [(h0[1], zl), (h0[1] + 0.05, zl + 0.30)]
    else:
        wps = [(h0[1], zl), (h0[1], zl + 0.04), (0.06, 0.145), (0.03, 0.155), (-0.03, 0.155), (-0.06, 0.145),
               (land_y, zl + 0.04), (land_y, zl - 0.01)]
    yield from _work(ctx, hand, _path(h0, wps), tend=tend, extra=30 if not one else 10,
                     drop_at=0.45 if mode == "drop" else None)
    _log(ctx, "path done")
    yield from _settle_crate(ctx)
    _log(ctx, "landed")
    if mode == "hold":  # never lets go
        for _ in range(8):
            yield from ctx.both(L.wait(0.5), Rt.wait(0.5))
        _log(ctx, "end")
        return
    gens = []
    if 1 in tend and mode != "drop":
        gens.append(_let_go(L, 1))
    if -1 in tend:
        gens.append(_let_go(Rt, -1))
    if len(gens) == 2:
        yield from ctx.both(*gens)
    elif gens:
        yield from gens[0]
    if mode == "drop":
        yield from L.move(STOW[1], yaw=0.0, duration=0.8)
    yield from _settle_crate(ctx)
    for _ in range(3):
        yield from L.wait(0.5)
        _log(ctx, "end")
