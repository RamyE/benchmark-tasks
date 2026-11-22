"""Twist-fold a round pop-up car window sunshade into its three-loop coil and slip it into its pouch.

Scene (metres from the arm-base midpoint, x forward, y left; heights above the table). A round pop-up side-window
sunshade, 44 cm across (a 1.15 mm ASTM A228 spring-steel wire hoop sewn into the 1 cm hem of a black mesh panel, 90 g),
lies open and flat on the table, centred at (0.38, 0.0). Its flat nylon storage pouch (22 x 20 cm, 4 cm inside height,
open along its near edge, a 7 x 7 cm thumb notch cut into the top layer at the mouth) lies empty at (0.45, 0.38), mouth
facing the robot (-x).

Sim model (README, Fidelity): the hoop is a closed ring of 48 rigid wire segments (2.88 cm chord) joined by 3-DOF
spring-damper D6 joints calibrated to 1.15 mm spring-steel music wire (EI/L = 0.60 N m/rad bend, GJ/L = 0.47 N m/rad
twist). A force-limited compliant pad grip (50 N pull, 0.50 N m twist, 1.50 N m tip; no rigid weld) engages only on
two-pad contact (both opposing finger pads registering >= 0.02 N PhysX normal contact force while closing) and releases
on open. Once both hands hold opposite arcs (>= 120 deg apart), compress the hoop to <= 16 cm and twist in opposite
senses (>= 20 deg each, >= 90 deg relative) against the calibrated wire springs—performing the pre-buckling work up to
the elastic snap-through peak—joint spring rest-targets ramp over 0.85 s to the uncut Lk=0 three-loop equilibrium
(1.80 N m/rad = 3x bundled wire stiffness) while both hands keep their original grips (zero pose or state writes).
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
import hoopkit  # noqa: E402
import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
RUN_LIMIT_STEPS = 1200

INSTRUCTION = "Fold the sunshade and put it in its pouch."

# -- the shade's hoop (1.15 mm / 19 AWG ASTM A228 music wire: EI = 0.0172 N m^2, GJ = 0.0136 N m^2, chord 2.88 cm) ----
HOOP_R = 0.22  # m, wire centreline radius (44 cm across)
N_SEG = 48
WIRE_R = 0.005  # collider: the 1.15 mm wire in its 1 cm stitched fabric hem
HOOP_C = np.array([0.38, 0.0, TOP + WIRE_R + 0.0005])
HOOP_K = 0.60  # N m/rad per joint, bending (EI / SEG_L = 0.0172 / 0.0288)
HOOP_KT = 0.47  # N m/rad per joint, twist (GJ / SEG_L = 0.0136 / 0.0288)
HOOP = hoopkit.hoop("sunshade", tuple(HOOP_C), hoop_radius=HOOP_R, segments=N_SEG, radius=WIRE_R, yaw=0.0,
                    bend_stiffness=HOOP_K, twist_stiffness=HOOP_KT, bend_damping=0.004, armature=5e-5,
                    cable_mass=0.09, max_bend=1.2, max_twist=2.5, self_collision=False, friction=0.9,
                    color=(0.08, 0.08, 0.09))
DTH = 2 * math.pi / N_SEG
SEG_L = 2 * HOOP_R * math.sin(math.pi / N_SEG)
LOOPS = 3
SEG_PER_LOOP = N_SEG // LOOPS
MIN_LOOP_SEP = 10  # segments on distinct loops of the 3-loop coil are >= 10 segments apart round the 48-seg ring
COIL_R = HOOP_R * math.sin(math.pi / N_SEG) / math.sin(math.pi / SEG_PER_LOOP)  # 0.0738
COIL_A = COIL_R * math.cos(math.pi / SEG_PER_LOOP)  # 0.0724 (14.5 cm across)

# -- the pouch: a rigid sleeve of welded panels (base is the body the others are welded to) --------------------------
P_C = np.array([0.45, 0.38])
P_LX, P_LY = 0.22, 0.20
P_T = 0.003
P_H = 0.040
P_WALL = 0.006
NOTCH_X, NOTCH_Y = 0.07, 0.07
MOUTH_X = float(P_C[0] - P_LX / 2)
NYLON = (0.10, 0.16, 0.38)
POUCH = Box("pouch", pos=(float(P_C[0]), float(P_C[1]), TOP + P_T / 2), size=(P_LX, P_LY, P_T), mass=0.014,
            color=NYLON)
_zt = TOP + P_T + P_H + P_T / 2
_zw = TOP + P_T + P_H / 2
POUCH_PARTS = [
    Box("pouch_back", pos=(float(P_C[0] + P_LX / 2 - P_WALL / 2), float(P_C[1]), _zw), size=(P_WALL, P_LY, P_H),
        mass=0.004, color=NYLON),
    Box("pouch_side_l", pos=(float(P_C[0]), float(P_C[1] + P_LY / 2 - P_WALL / 2), _zw), size=(P_LX - P_WALL, P_WALL, P_H),
        mass=0.004, color=NYLON),
    Box("pouch_side_r", pos=(float(P_C[0]), float(P_C[1] - P_LY / 2 + P_WALL / 2), _zw), size=(P_LX - P_WALL, P_WALL, P_H),
        mass=0.004, color=NYLON),
    Box("pouch_top", pos=(float(MOUTH_X + NOTCH_X + (P_LX - NOTCH_X) / 2), float(P_C[1]), _zt),
        size=(P_LX - NOTCH_X, P_LY, P_T), mass=0.010, color=NYLON),
    Box("pouch_lip_l", pos=(float(MOUTH_X + NOTCH_X / 2), float(P_C[1] + NOTCH_Y / 2 + (P_LY - NOTCH_Y) / 4), _zt),
        size=(NOTCH_X, (P_LY - NOTCH_Y) / 2, P_T), mass=0.002, color=NYLON),
    Box("pouch_lip_r", pos=(float(MOUTH_X + NOTCH_X / 2), float(P_C[1] - NOTCH_Y / 2 - (P_LY - NOTCH_Y) / 4), _zt),
        size=(NOTCH_X, (P_LY - NOTCH_Y) / 2, P_T), mass=0.002, color=NYLON),
]
IN_X0, IN_X1 = -P_LX / 2, P_LX / 2 - P_WALL
IN_Y = P_LY / 2 - P_WALL
IN_Z0, IN_Z1 = P_T / 2, P_T / 2 + P_H

OBJECTS = [POUCH, *POUCH_PARTS, HOOP]

# -- two-pad contact pinch & buckling constants -----------------------------------------------------------------------
PAD_CONTACT_MIN_N = 0.02  # N: minimum PhysX normal contact force required simultaneously on BOTH opposing finger pads
ARM_OPEN_AP = 0.026  # jaw must open >= 2.6 cm before acquiring a pinch (already-closed jaws cannot attach)
EMPTY_CLOSE_AP = 0.005  # closing < 5 mm with no wire between pads disarms the jaw until re-opened
PINCH_CMD_MAX = 0.014  # commanded jaw opening (m) for clamping the 1 cm hem
PINCH_AP_MAX = 0.036  # measured jaw opening (m) when pads squeeze 1 to 3 stacked 1 cm hems
RELEASE_OPEN = 0.022  # commanding jaws past 2.2 cm releases held segments immediately
PAD_HALF_W = 0.015  # half-width of finger pad footprint along wire tangent (m)
PAD_Z_LO, PAD_Z_HI = -0.028, 0.016  # along approach relative to TCP (pad tips at +13 mm)
HOLD_N = 50.0  # N: 2 pads x mu 0.5 x 50 N YAM linear_4310 gripper squeeze effort
TWIST_NM = 0.50  # N m: 50 N x 10 mm mean pad-patch radius
TIP_NM = 1.50  # N m: 2 x 50 N x 15 mm half pad length
PINCH_K_LIN, PINCH_C_LIN = 20000.0, 200.0  # N/m, N s/m compliant pad shear stiffness & damping
PINCH_K_ROT, PINCH_C_ROT = 50.0, 0.5  # N m/rad, N m s/rad compliant pad rotational stiffness & damping

SNAP_ARC = N_SEG // 3  # held segments >= 16 segments (120 deg) apart round the ring
SNAP_EACH = math.radians(20.0)  # each hand turns >= 20 deg in opposite senses about the hand line
SNAP_TWIST = math.radians(90.0)  # relative twist >= 90 deg about the hand line
SNAP_GAP = 0.16  # hands <= 16 cm apart (compressing the 44 cm hoop down to its ~14.5 cm 3-loop width)
SNAP_RAMP_S = 0.85  # s over which post-buckling rest curvature relaxes in PhysX
OPEN_RAMP_S = 0.30  # s over which rest curvature relaxes to 0 when unretained
COIL_K = float(os.environ.get("SS_COIL_K", "1.80"))  # 3 * HOOP_K = 1.80 N m/rad (three bundled wire loops)
TW_SIGN_OVR = int(os.environ.get("SS_TWSIGN", "0"))
COIL_C = float(os.environ.get("SS_COIL_C", "0.012"))
COIL_ARM = 5e-4
TABLE_PRESS_TCP_Z = 0.045  # m: holding TCP height above table when pressing the coil flat onto the table
TABLE_PRESS_ZMIN = 0.020  # m: bottom of coil must touch table within 2.0 cm
TABLE_PRESS_ZMEAN = 0.070  # m: mean height of coiled hoop above table while pressed flat
OPEN_GRACE = 0.15  # s before an unretained coil springs open
FOLD_R = 0.10
CLEAR = 0.10
PARK_SPEED = 0.02
PARK_S = 0.5
OFF_TABLE_Z = TOP - 0.03


def _R(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _quat_xyzw(R) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    return Rotation.from_matrix(np.asarray(R, float)).as_quat()


def _rotvec(R) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    return Rotation.from_matrix(np.asarray(R, float)).as_rotvec()


def _finite(a) -> bool:
    return a is not None and bool(np.isfinite(np.asarray(a, float)).all())


def _min_rot_x(x_dst: np.ndarray) -> np.ndarray:
    x0 = np.array([1.0, 0.0, 0.0])
    ax = np.cross(x0, x_dst)
    s, c = float(np.linalg.norm(ax)), float(np.dot(x0, x_dst))
    if s < 1e-12:
        return np.eye(3)
    v = ax / s * math.atan2(s, c)
    th = float(np.linalg.norm(v))
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def lk0_coil_targets(s_ref: int = 0, tw_sign: int = 1) -> np.ndarray:
    """Exact joint rest angles (N_SEG, 3) = (rotX, rotY, rotZ) in radians for the uncut Lk=0 three-loop flat coil
    (Wr = -tw_sign*2, Tw = tw_sign*2), phased at segment ``s_ref``. Every joint angle is < 30 deg."""
    ch, sh = math.cos(-DTH / 2), math.sin(-DTH / 2)
    rz_nh = np.array([[ch, -sh, 0.0], [sh, ch, 0.0], [0.0, 0.0, 1.0]])
    n0 = np.array([0.0, 0.0, 1.0])
    rots = []
    for i in range(N_SEG):
        a = LOOPS * (i - s_ref) * DTH
        tw = tw_sign * (LOOPS - 1) * (i - s_ref) * DTH
        t = np.array([math.cos(a), math.sin(a), 0.0])
        ct, st = math.cos(tw), math.sin(tw)
        rx = np.array([[1.0, 0.0, 0.0], [0.0, ct, -st], [0.0, st, ct]])
        rots.append(np.stack([t, np.cross(n0, t), n0], axis=1) @ rx)
    qs = np.zeros((N_SEG, 3), dtype=np.float64)
    for i in range(N_SEG):
        Rj = rz_nh @ rots[i].T @ rots[(i + 1) % N_SEG] @ rz_nh
        # PhysX D6 twist-swing decomposition: Rj = R_swing([0, q1, q2]) @ Rx(q0)
        R_sw = _min_rot_x(Rj[:, 0])
        sv = _rotvec(R_sw)
        R_tw = R_sw.T @ Rj
        q0 = math.atan2(float(R_tw[2, 1]), float(R_tw[1, 1]))
        qs[i] = (q0, float(sv[1]), float(sv[2]))
    return qs


def folded(pts) -> bool:
    if not _finite(pts):
        return False
    pts = np.asarray(pts, float)
    return bool(np.linalg.norm(pts - pts.mean(0), axis=1).max() <= FOLD_R)


def to_pouch(pts, pouch_pos, pouch_R):
    return (np.asarray(pts, float) - np.asarray(pouch_pos, float)) @ np.asarray(pouch_R, float)


def seg_arc(a: int, b: int) -> int:
    return min((int(a) - int(b)) % N_SEG, (int(b) - int(a)) % N_SEG)


def snap_ready(s_left: int, s_right: int, tw_left: float, tw_right: float, gap: float) -> bool:
    if seg_arc(s_left, s_right) < SNAP_ARC or gap > SNAP_GAP:
        return False
    if tw_left * tw_right >= 0.0 or min(abs(tw_left), abs(tw_right)) < SNAP_EACH:
        return False
    return abs(tw_left - tw_right) >= SNAP_TWIST


def _multi_loop_held(entries: list[tuple[str, int]]) -> bool:
    segs = [seg for _, seg in entries]
    return len(segs) >= 2 and any(seg_arc(a, b) >= MIN_LOOP_SEP for i, a in enumerate(segs) for b in segs[i + 1:])


def loops_retained(welds: dict, tcp_z_local: tuple[float, float], pts_local, pouch_pos, pouch_R, gap: float) -> bool:
    """Whether the buckled 3-loop coil is physically kept from springing open."""
    if inside_pouch(pts_local, pouch_pos, pouch_R):
        return True
    if len(welds) == 2 and gap <= 0.22:
        s0, s1 = welds[0][0][1], welds[1][0][1]
        if seg_arc(s0, s1) >= SNAP_ARC // 2:
            return True
    if any(_multi_loop_held(entries) for entries in welds.values()):
        return True
    if welds and _finite(pts_local):
        zs = np.asarray(pts_local, float)[:, 2]
        low_hand = any(tcp_z_local[i] <= TOP + TABLE_PRESS_TCP_Z for i in welds)
        if low_hand and float(zs.min()) <= TOP + TABLE_PRESS_ZMIN and float(zs.mean()) <= TOP + TABLE_PRESS_ZMEAN:
            return True
    return False


def pouch_upright(pouch_pos, pouch_R) -> bool:
    if not (_finite(pouch_pos) and _finite(pouch_R)):
        return False
    z = float(np.asarray(pouch_pos, float)[2])
    return bool(float(np.asarray(pouch_R, float)[2, 2]) >= 0.95 and TOP - 0.005 <= z <= TOP + P_T + 0.010)


def inside_pouch(pts, pouch_pos, pouch_R, margin: float = WIRE_R) -> bool:
    if not (_finite(pts) and pouch_upright(pouch_pos, pouch_R)):
        return False
    q = to_pouch(pts, pouch_pos, pouch_R)
    return bool((q[:, 0] >= IN_X0 + margin).all() and (q[:, 0] <= IN_X1 - margin).all()
                and (np.abs(q[:, 1]) <= IN_Y - margin).all()
                and (q[:, 2] >= IN_Z0).all() and (q[:, 2] <= IN_Z1 - margin).all())


def pouch_box(pouch_pos, pouch_R):
    corners = np.array([[sx * P_LX / 2, sy * P_LY / 2, sz] for sx in (-1, 1) for sy in (-1, 1)
                        for sz in (-P_T / 2, P_T + P_H + P_T / 2)])
    w = np.asarray(pouch_pos, float) + corners @ np.asarray(pouch_R, float).T
    return w.min(0), w.max(0)


def clear_of(grip_pts, hoop_pts, pouch_pos, pouch_R, d: float = CLEAR) -> bool:
    if not grip_pts or not _finite(hoop_pts) or not _finite(pouch_pos):
        return False
    lo, hi = pouch_box(pouch_pos, pouch_R)
    hp = np.asarray(hoop_pts, float)
    for p in grip_pts:
        p = np.asarray(p, float)
        if not _finite(p):
            return False
        if float(np.linalg.norm(hp - p, axis=1).min()) < d:
            return False
        if float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0))) < d:
            return False
    return True


def evaluate(f: dict, pts, pouch_pos, pouch_R) -> dict:
    if f.get("dropped") or not _finite(pts) or not _finite(pouch_pos) or not _finite(pouch_R):
        return {"score": 0.0, "why": "dropped or invalid"}
    pts_a, pp_a = np.asarray(pts, float), np.asarray(pouch_pos, float)
    if (pts_a[:, 2] < OFF_TABLE_Z).any() or float(pp_a[2]) < OFF_TABLE_Z:
        return {"score": 0.0, "why": "dropped off table"}
    if not f.get("folded_ever"):
        return {"score": 0.0, "why": "never folded"}
    inside = folded(pts_a) and inside_pouch(pts_a, pp_a, pouch_R)
    if not inside:
        return {"score": 0.25, "why": "folded at some point, not folded inside the pouch now"}
    if f.get("still_s", 0.0) < PARK_S:
        return {"score": 0.5, "why": "folded inside the pouch, hands not released and parked clear"}
    return {"score": 1.0, "why": "folded inside the pouch, hands released and parked"}


_ST: dict = {"env": None, "root": None, "hoop": None}


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


def _enable_pad_contact_reports(stage, root: str) -> None:
    from pxr import PhysxSchema, UsdPhysics

    for p in stage.Traverse():
        ps = str(p.GetPath())
        if ps.startswith(root + "/") and ("/LeftArm/" in ps or "/RightArm/" in ps or "/left_robot/" in ps or "/right_robot/" in ps):
            if ps.endswith(("/tip_left", "/tip_right")) and p.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.0)


def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_sunshade", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != HOOP.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics, Vt

        _enable_contact_processing()
        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        _ST["root"], _ST["hoop"] = root, str(prim_path)
        _enable_pad_contact_reports(stage, root)
        for part in POUCH_PARTS:
            j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/{part.name}_weld")
            j.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{POUCH.name}")])
            j.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{part.name}")])
            j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, np.asarray(part.pos) - np.asarray(POUCH.pos))))
            j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateExcludeFromArticulationAttr(True)
        mesh = UsdGeom.Mesh.Define(stage, f"{root}/sunshade_panel")
        mesh.CreateSubdivisionSchemeAttr("none")
        pts = HOOP.start_points()
        allp = np.vstack([pts.mean(0) + np.array([0, 0, -0.0035]), pts + np.array([0, 0, -0.0035])])
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(allp.astype(np.float32)))
        mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, allp.min(0) - 0.01)),
                                             Gf.Vec3f(*map(float, allp.max(0) + 0.01))]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * N_SEG))
        idx = []
        for i in range(N_SEG):
            idx += [0, 1 + i, 1 + (i + 1) % N_SEG]
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.16, 0.16, 0.17)]))
        _ST["panel"] = f"{root}/sunshade_panel"
        return prim

    spawn._sunshade = True
    cablekit._spawn_cable = spawn


_install_spawn()


class _Episode:
    def __init__(self):
        self.welds: dict[int, list[tuple[str, int]]] = {}  # hand -> [(joint_path, seg_idx), ...]
        self.armed = {0: True, 1: True}
        self.twist = {}
        self.prevR = {}
        self.two_hand = False
        self.coiled = False
        self.coil_frac = 0.0
        self.coil_q = lk0_coil_targets(0, 1)
        self.unheld_s = 0.0
        self.folded_ever = False
        self.dropped = False
        self.snaps = 0
        self.opens = 0
        self.still_s = 0.0
        self.prev_grip = None
        self.steps = 0


_EP = _Episode()
_N = {"joint": 0}


def _stage():
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage()


def _origin(env) -> np.ndarray:
    return env.unwrapped.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)


def _t(x):
    return getattr(x, "torch", x)


def _hoop_poses(env):
    art = env.unwrapped.scene[HOOP.name]
    names = list(art.body_names)
    idx = [names.index(f"seg_{i}") for i in range(N_SEG)]
    pos = _t(art.data.body_pos_w)[0].detach().cpu().numpy().astype(np.float64)[idx]
    quat = _t(art.data.body_quat_w)[0].detach().cpu().numpy().astype(np.float64)[idx]
    return pos, np.array([_R(q) for q in quat])


def _pose_w(env, name):
    p = env.unwrapped.arena_world.get_pose_w(name)[0].detach().cpu().numpy().astype(np.float64)
    return p[:3], _R(p[3:])


def _gripper_paths():
    if _ST.get("gpaths"):
        return _ST["gpaths"]
    out = [None, None]
    for p in _stage().Traverse():
        if p.GetName() != "gripper" or "env_0" not in str(p.GetPath()):
            continue
        path = str(p.GetPath())
        for i, side in enumerate(("left", "right")):
            if out[i] is None and (f"/{side}_robot/" in path or f"/{side.capitalize()}Arm/" in path) \
                    and p.GetParent().GetName() != "gripper":
                out[i] = path
    _ST["gpaths"] = out
    return out


def _pad_forces(env) -> list[tuple[float, float]]:
    """Measured PhysX normal contact force (N) on (tip_left, tip_right) for each arm via yam_arena ContactSensor."""
    out = [(0.0, 0.0), (0.0, 0.0)]
    try:
        from yam_arena.runtime import contact_forces

        sensors = env.unwrapped.scene.sensors
        for i, side in enumerate(("left", "right")):
            key = f"{side}_contacts"
            if key in sensors:
                cf = contact_forces(sensors[key])
                out[i] = (float(cf.get("tip_left", 0.0)), float(cf.get("tip_right", 0.0)))
    except Exception:  # noqa: BLE001
        pass
    return out


def _hands(env):
    from yam_arena.runtime import link_poses

    base = env.unwrapped
    out = []
    for side in ("left", "right"):
        rob = base.scene.articulations[f"{side}_robot"]
        ids = rob.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        q = _t(rob.data.joint_pos)[0, ids].detach().cpu().numpy()
        tgt = _t(rob.data.joint_pos_target)[0, ids].detach().cpu().numpy()
        ap = float(abs(q[0]) + abs(q[1]))
        cmd = float(abs(tgt[0]) + abs(tgt[1]))
        lp = link_poses(rob)
        G = lp["gripper"]
        tcp = G[:3, :3] @ np.array([0.0, 0.0, 0.134]) + G[:3, 3]
        zc = G[:3, 2] / max(1e-9, float(np.linalg.norm(G[:3, 2])))
        xc = lp["tip_left"][:3, 3] - lp["tip_right"][:3, 3]
        xc = xc - zc * float(xc @ zc)
        xc = xc / float(np.linalg.norm(xc)) if float(np.linalg.norm(xc)) > 1e-4 else G[:3, 0]
        R_pad = np.stack([xc, np.cross(zc, xc), zc], axis=1)
        out.append((G, tcp, R_pad, ap, cmd))
    return out


def _segments_between_pads(pos: np.ndarray, tcp: np.ndarray, R_pad: np.ndarray, ap: float, multi_loop: bool) -> list[int]:
    """Segments whose wire cross-section physically lies inside the space between the two opposing finger pads."""
    loc = (pos - tcp) @ R_pad
    half_x = (max(ap / 2.0, 0.010) + WIRE_R + 0.008) if multi_loop else (max(ap / 2.0, 0.004) + WIRE_R + 0.004)
    pad_w = (PAD_HALF_W + 0.006) if multi_loop else PAD_HALF_W
    mask = (np.abs(loc[:, 0]) <= half_x) & (np.abs(loc[:, 1]) <= pad_w) & (loc[:, 2] >= PAD_Z_LO) & (loc[:, 2] <= PAD_Z_HI)
    cand = np.flatnonzero(mask)
    if len(cand) == 0:
        return []
    dists = np.linalg.norm(loc[cand], axis=1)
    order = cand[np.argsort(dists)]
    if not multi_loop:
        return [int(order[0])]
    chosen: list[int] = []
    for s in order:
        si = int(s)
        if all(seg_arc(si, prev) >= MIN_LOOP_SEP for prev in chosen):
            chosen.append(si)
            if len(chosen) == LOOPS:
                break
    return chosen


def _new_pinch_joint(gripper_path: str, G: np.ndarray, seg: int, seg_p: np.ndarray, seg_R: np.ndarray, tag: str,
                     axial_roll: bool = True) -> str:
    """Force-limited two-pad-contact compliant pad grip at the current relative pose (no rigid weld, zero snap).
    Engages only after verified two-pad PhysX contact (`pad_forces >= PAD_CONTACT_MIN_N`). Linear and tipping axes use
    compliant spring-damper drives capped at YAM rubber-pad limits (`HOLD_N = 50 N`, `TIP_NM = 1.50 N m`,
    `TWIST_NM = 0.50 N m`); on a single hem segment `rotX` allows damped axial wire roll inside the fabric hem."""
    from pxr import Gf, Sdf, UsdPhysics

    _N["joint"] += 1
    path = f"{_ST['root']}/shade_joints/{tag}_{_N['joint']}"
    T1 = np.eye(4)
    T1[:3, :3], T1[:3, 3] = seg_R, seg_p
    rel = np.linalg.inv(G) @ T1
    x, y, z, w = _quat_xyzw(rel[:3, :3])
    j = UsdPhysics.Joint.Define(_stage(), path)
    j.CreateBody0Rel().SetTargets([Sdf.Path(gripper_path)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(f"{_ST['hoop']}/seg_{seg}")])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, rel[:3, 3])))
    j.CreateLocalRot0Attr(Gf.Quatf(float(w), float(x), float(y), float(z)))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateExcludeFromArticulationAttr(True)
    prim = j.GetPrim()
    d2r = math.pi / 180.0
    for dof in ("transX", "transY", "transZ"):
        drv = UsdPhysics.DriveAPI.Apply(prim, dof)
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(PINCH_K_LIN)
        drv.CreateDampingAttr(PINCH_C_LIN)
        drv.CreateMaxForceAttr(HOLD_N)
        drv.CreateTargetPositionAttr(0.0)
    for dof in ("rotY", "rotZ"):
        drv = UsdPhysics.DriveAPI.Apply(prim, dof)
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(PINCH_K_ROT * d2r)
        drv.CreateDampingAttr(PINCH_C_ROT * d2r)
        drv.CreateMaxForceAttr(TIP_NM)
        drv.CreateTargetPositionAttr(0.0)
    drv_x = UsdPhysics.DriveAPI.Apply(prim, "rotX")
    drv_x.CreateTypeAttr("force")
    drv_x.CreateStiffnessAttr(0.0 if axial_roll else PINCH_K_ROT * d2r)
    drv_x.CreateDampingAttr((0.002 if axial_roll else PINCH_C_ROT) * d2r)
    drv_x.CreateMaxForceAttr(TWIST_NM)
    drv_x.CreateTargetPositionAttr(0.0)
    return path


def _drop_weld(i: int) -> None:
    entries = _EP.welds.pop(i, None)
    _EP.twist.pop(i, None)
    _EP.prevR.pop(i, None)
    if not entries:
        return
    for path, _ in entries:
        try:
            _stage().GetPrimAtPath(path).GetAttribute("physics:jointEnabled").Set(False)
        except Exception as e:  # noqa: BLE001
            print(f"[shade] weld drop failed: {e!r}", flush=True)


def _dof_map(art):
    names = list(art.joint_names)
    if "dof_map" not in _ST:
        idx = [[names.index(f"bend_{i}:{ax}") for ax in (0, 1, 2)] for i in range(1, N_SEG)]
        _ST["dof_map"] = np.array(idx, dtype=np.int64)
    return _ST["dof_map"]


def _apply_curvature_drive(env, frac: float) -> None:
    """Update ONLY PhysX joint spring rest-targets and stiffness/damping along the Lk=0 homotopy ``frac * coil_q``.
    Never writes root pose, velocity, or joint positions."""
    import torch

    base = env.unwrapped
    art = base.scene[HOOP.name]
    dev = base.device
    jp = _t(art.data.joint_pos)
    q = torch.zeros_like(jp)
    dm = _dof_map(art)
    qt = frac * _EP.coil_q[:N_SEG - 1]  # (47, 3) for bend_1 .. bend_47
    for ax in range(3):
        q[0, dm[:, ax]] = torch.tensor(qt[:, ax], dtype=q.dtype, device=dev)
    art.set_joint_position_target_index(target=q, full_data=True)

    k_b = HOOP_K + frac * (COIL_K - HOOP_K)
    k_t = HOOP_KT + frac * (COIL_K - HOOP_KT)
    kk = torch.full_like(q, float(k_b))
    kk[0, dm[:, 0]] = float(k_t)
    c_val = HOOP.bend_damping + frac * (COIL_C - HOOP.bend_damping)
    cc = torch.full_like(q, float(c_val))
    if hasattr(art, "actuators") and "bend" in art.actuators:
        try:
            art.actuators["bend"].stiffness[:] = kk
            art.actuators["bend"].damping[:] = cc
        except Exception:  # noqa: BLE001
            pass
    art.write_joint_stiffness_to_sim_index(stiffness=kk, full_data=True)
    art.write_joint_damping_to_sim_index(damping=cc, full_data=True)
    art.write_joint_armature_to_sim_index(armature=torch.full_like(q, float(COIL_ARM if frac > 0 else HOOP.armature)),
                                         full_data=True)
    try:
        cpath = _ST.get("close_path")
        if not cpath:
            for cand in (f"{_ST['hoop']}/seg_0/close", "/World/envs/env_0/sunshade/seg_0/close"):
                cp = _stage().GetPrimAtPath(cand)
                if cp and cp.IsValid():
                    cpath = cand
                    _ST["close_path"] = cpath
                    break
        if cpath:
            p = _stage().GetPrimAtPath(cpath)
            qc = frac * _EP.coil_q[N_SEG - 1]
            for ax, (dof, k) in enumerate((("rotX", k_t), ("rotY", k_b), ("rotZ", k_b))):
                p.GetAttribute(f"drive:{dof}:physics:targetPosition").Set(float(math.degrees(qc[ax])))
                p.GetAttribute(f"drive:{dof}:physics:stiffness").Set(float(k * math.pi / 180))
                p.GetAttribute(f"drive:{dof}:physics:damping").Set(float(c_val * math.pi / 180))
    except Exception as e:  # noqa: BLE001
        print(f"[shade] close-joint drive failed: {e!r}", flush=True)


def _update(env, dt: float) -> None:
    if _ST.get("root") is None:
        return
    _EP.steps += 1
    o = _origin(env)
    gp = _gripper_paths()
    hands = _hands(env)
    pforces = _pad_forces(env)
    pos, rots = _hoop_poses(env)
    if not _finite(pos):
        _EP.dropped = True
        return

    # 1. Two-pad PhysX contact-verified pad-pinch grasps
    for i in (0, 1):
        G, tcp, R_pad, ap, cmd = hands[i]
        f_l, f_r = pforces[i]
        if i in _EP.welds:
            if cmd > RELEASE_OPEN or ap > PINCH_AP_MAX + 0.008:
                _drop_weld(i)
                _EP.armed[i] = True
                print(f"[shade] {('left', 'right')[i]} hand released (ap={ap:.3f} cmd={cmd:.3f} step={_EP.steps})",
                      flush=True)
            continue
        if ap >= ARM_OPEN_AP:
            _EP.armed[i] = True
        if not _EP.armed[i] or gp[i] is None:
            continue
        two_pad_contact = (f_l >= PAD_CONTACT_MIN_N and f_r >= PAD_CONTACT_MIN_N)
        if cmd <= PINCH_CMD_MAX and EMPTY_CLOSE_AP < ap <= PINCH_AP_MAX and cmd < ap - 0.001 and two_pad_contact:
            segs = _segments_between_pads(pos, tcp, R_pad, ap, multi_loop=_EP.coiled)
            if segs:
                ax_roll = len(segs) == 1
                _EP.welds[i] = [(_new_pinch_joint(gp[i], G, s, pos[s], rots[s], f"grip_{i}", axial_roll=ax_roll), s)
                                for s in segs]
                _EP.armed[i] = False
                _EP.twist[i], _EP.prevR[i] = 0.0, G[:3, :3].copy()
                print(f"[shade] {('left', 'right')[i]} hand pinched segs {segs} on two-pad contact "
                      f"(f=({f_l:.2f},{f_r:.2f})N ap={ap:.3f} cmd={cmd:.3f} step={_EP.steps})", flush=True)
        elif ap <= EMPTY_CLOSE_AP:
            _EP.armed[i] = False

    # 2. Accumulate opposite twists while both hands hold opposite arcs of the open hoop
    u = hands[0][1] - hands[1][1]
    gap = float(np.linalg.norm(u))
    u = u / max(gap, 1e-9)
    if len(_EP.welds) == 2 and not _EP.coiled:
        if not _EP.two_hand:
            _EP.twist = {0: 0.0, 1: 0.0}
            _EP.prevR = {0: hands[0][0][:3, :3].copy(), 1: hands[1][0][:3, :3].copy()}
            _EP.two_hand = True
        else:
            for i in (0, 1):
                Rn = hands[i][0][:3, :3]
                dR = Rn @ _EP.prevR[i].T
                _EP.twist[i] += float(_rotvec(dR) @ u)
                _EP.prevR[i] = Rn.copy()
    else:
        _EP.two_hand = False

    tw0, tw1 = _EP.twist.get(0, 0.0), _EP.twist.get(1, 0.0)
    rel = (tw0 - tw1) if len(_EP.welds) == 2 else 0.0
    if _EP.steps % 10 == 0 and len(_EP.welds) == 2 and not _EP.coiled:
        print(f"[shade] step {_EP.steps}: gap={gap:.3f} tw=({math.degrees(tw0):.0f}, {math.degrees(tw1):.0f}) "
              f"rel={math.degrees(rel):.0f} deg", flush=True)

    if not _EP.coiled and len(_EP.welds) == 2:
        s0, s1 = _EP.welds[0][0][1], _EP.welds[1][0][1]
        if snap_ready(s0, s1, tw0, tw1, gap):
            tw_sign = TW_SIGN_OVR if TW_SIGN_OVR in (-1, 1) else (1 if rel < 0 else -1)
            _EP.coil_q = lk0_coil_targets(s1, tw_sign)
            _EP.coiled = True
            _EP.snaps += 1
            _EP.two_hand = False
            print(f"[shade] BUCKLE triggered at step {_EP.steps} (held segs {s0}, {s1}, tw_sign={tw_sign})", flush=True)

    # 3. Check physical loop retention while coiled
    pp, pR = _pose_w(env, POUCH.name)
    pouch_l = pp - o
    pl = pos - o
    tcp_z_local = (float(hands[0][1][2] - o[2]), float(hands[1][1][2] - o[2]))
    if _EP.coiled:
        if loops_retained(_EP.welds, tcp_z_local, pl, pouch_l, pR, gap):
            _EP.unheld_s = 0.0
        else:
            _EP.unheld_s += dt
            if _EP.unheld_s > OPEN_GRACE:
                _EP.coiled = False
                _EP.opens += 1
                _EP.unheld_s = 0.0
                print(f"[shade] coil sprang open at step {_EP.steps} (held={ {k: [s for _, s in v] for k, v in _EP.welds.items()} })",
                      flush=True)

    # 4. Smoothly ramp PhysX rest-curvature drives (zero pose/state writes; held segments never reassigned)
    target_frac = 1.0 if _EP.coiled else 0.0
    if abs(_EP.coil_frac - target_frac) > 1e-6:
        rate = (1.0 / SNAP_RAMP_S) if target_frac > _EP.coil_frac else (1.0 / OPEN_RAMP_S)
        if target_frac > _EP.coil_frac:
            _EP.coil_frac = min(1.0, _EP.coil_frac + dt * rate)
        else:
            _EP.coil_frac = max(0.0, _EP.coil_frac - dt * rate)
        _apply_curvature_drive(env, _EP.coil_frac)

    # 5. Latches & parked-clear timer
    if (pl[:, 2] < OFF_TABLE_Z).any() or float(pouch_l[2]) < OFF_TABLE_Z:
        _EP.dropped = True
    if folded(pl):
        _EP.folded_ever = True

    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    grip = []
    for G, tcp, _, _, _ in hands:
        tip = G[:3, :3] @ (np.array(TCP_OFFSET) + np.array([0.0, 0.0, PAD_TIP_PAST_TCP])) + G[:3, 3]
        grip.extend([tcp - o, tip - o])
    prev, _EP.prev_grip = _EP.prev_grip, grip
    if prev is None or _EP.welds or not all(_finite(g) for g in grip):
        _EP.still_s = 0.0
    else:
        speed = max(float(np.linalg.norm(a - b)) for a, b in zip(grip, prev)) / dt
        ok = speed < PARK_SPEED and clear_of(grip, pl, pouch_l, pR)
        _EP.still_s = _EP.still_s + dt if ok else 0.0
    _draw_panel(pl)


def _draw_panel(pl) -> None:
    try:
        from pxr import Gf, Vt

        p = _stage().GetPrimAtPath(_ST["panel"])
        c = pl.mean(0)
        allp = np.vstack([c, pl]).astype(np.float32)
        p.GetAttribute("points").Set(Vt.Vec3fArray.FromNumpy(allp))
        p.GetAttribute("extent").Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, allp.min(0) - 0.01)),
                                                    Gf.Vec3f(*map(float, allp.max(0) + 0.01))]))
    except Exception as e:  # noqa: BLE001
        if not _ST.get("panel_err"):
            _ST["panel_err"] = repr(e)
            print(f"[shade] panel draw failed: {e!r}", flush=True)


def _reset_episode() -> None:
    global _EP
    for i in list(_EP.welds):
        _drop_weld(i)
    was = _EP.coil_frac > 0.0 or _EP.opens
    _EP = _Episode()
    if was and _ST.get("env") is not None:
        try:
            _apply_curvature_drive(_ST["env"], 0.0)
        except Exception as e:  # noqa: BLE001
            print(f"[shade] reset targets failed: {e!r}", flush=True)


def _patch_embodiment(emb_mod) -> None:
    cls = emb_mod.BimanualYamEmbodiment
    if getattr(cls.__init__, "_sunshade_cs_patched", False):
        return
    orig_init = cls.__init__

    def __init__(self, *a, **kw):
        kw["contact_sensors"] = True
        orig_init(self, *a, **kw)

    __init__._sunshade_cs_patched = True
    cls.__init__ = __init__


class _EmbFinder(importlib.abc.MetaPathFinder):
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
            _patch_embodiment(module)

        spec.loader.exec_module = exec_module
        return spec


if "yam_arena.embodiment" in sys.modules:
    _patch_embodiment(sys.modules["yam_arena.embodiment"])
elif not any(getattr(f, "_sunshade_emb_finder", False) for f in sys.meta_path):
    _ef = _EmbFinder()
    _ef._sunshade_emb_finder = True
    sys.meta_path.insert(0, _ef)


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_shade_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ST["env"] = self._env
            try:
                _update(self._env, 1.0 / float(self.info.control_hz))
            except Exception as e:  # noqa: BLE001
                if not _ST.get("err"):
                    _ST["err"] = repr(e)
                    import traceback

                    traceback.print_exc()
                    print(f"[shade] model error: {e!r}", flush=True)
            return base(self, *a, **k)

        _task_step._shade_hooked = True
        cls._task_step = _task_step
    if not getattr(cls.reset, "_shade_hooked", False):
        base_reset = cls.reset

        def reset(self, *a, **k):
            _reset_episode()
            out = base_reset(self, *a, **k)
            _ST["env"] = self._env
            return out

        reset._shade_hooked = True
        cls.reset = reset


_install_hooks()


def _live(objects) -> dict:
    env = _ST.get("env")
    h = objects[HOOP.name]
    pts = np.asarray(h.points, float)
    if env is not None:
        o = _origin(env)
        pp, pR = _pose_w(env, POUCH.name)
        pp = pp - o
    else:
        s = objects[POUCH.name]
        pp, pR = np.asarray(s.pos, float), _R(s.quat_xyzw)
    f = {"dropped": _EP.dropped, "folded_ever": _EP.folded_ever or folded(pts), "still_s": _EP.still_s}
    return evaluate(f, pts, pp, pR)


def score(objects) -> float:
    return float(_live(objects)["score"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    return 1.0 - score(objects)


def progress(objects) -> dict:
    r = _live(objects)
    r.update(coiled=_EP.coiled, frac=round(_EP.coil_frac, 2), snaps=_EP.snaps, opens=_EP.opens,
             held={k: [s for _, s in v] for k, v in _EP.welds.items()})
    return r
