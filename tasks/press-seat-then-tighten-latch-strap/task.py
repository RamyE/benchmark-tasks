"""Child-seat fitting station. A grey vehicle-seat cushion block (28 x 50 x 6 cm) has a pit in its middle; in it a
blue child-seat shell (18 x 20 x 8 cm, 500 g) rides on a vertical spring (the cushion: 300 N/m, 4 N preload against a
top stop, 3 cm travel). A black 8 mm lower-anchor strap (40 cm) is hooked to the left lower anchor, runs up through the
shell's covered belt path (two ridges and a cover bar on the shell top, across its rear) and down into the connector
on the right lower anchor: a roofed channel in which the strap ends in a 25 g slider (the adjuster). The slider can
slide out freely but the strap cannot pull it back (one-way). Its orange tail tab comes out of the connector's end
wall and is joined to the slider by a breakaway link that counts as torn above 4 N. The strap starts with ~1 cm slack.

Pulling the tab alone only takes up the slack: past that, pressing the sprung seat down through the strap needs more
than the tab can take. One arm must press the seat into the cushion and hold it while the other draws the tab out;
when the press is let go the cushion pushes the seat up against the shortened strap, which holds it down.

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
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "The blue shell is an empty child car seat on a vehicle seat cushion. Tighten its lower-anchor strap so the car seat is fitted firmly, then let go."

GREY = (0.46, 0.47, 0.50)
DARK = (0.16, 0.17, 0.19)
BLUE = (0.16, 0.36, 0.78)
ORANGE = (0.95, 0.50, 0.08)

# -- vehicle seat cushion: a fixed frame round a pit; the child seat shell rides in the pit on a spring ----------------
CZ = TOP + 0.060  # cushion top
SH_X, SH_Y = 0.29, 0.0  # shell centre (top view)
SH_L, SH_W, SH_H = 0.18, 0.20, 0.08  # shell x, y, z
SH_Z0 = TOP + 0.082  # shell centre height at rest (top TOP + 0.122)
SH_TOP0 = SH_Z0 + SH_H / 2
SH_MASS = 0.5
CL = 0.003  # pit clearance round the shell
X0C, X1C, Y1C = 0.15, 0.43, 0.25
_hx0, _hx1, _hy = SH_X - SH_L / 2 - CL, SH_X + SH_L / 2 + CL, SH_W / 2 + CL
_cz = (TOP + CZ) / 2
CUSHION = [
    Box("cushion_front", pos=((X0C + _hx0) / 2, 0.0, _cz), size=(_hx0 - X0C, 2 * Y1C, CZ - TOP), color=GREY),
    Box("cushion_back", pos=((_hx1 + X1C) / 2, 0.0, _cz), size=(X1C - _hx1, 2 * Y1C, CZ - TOP), color=GREY),
    Box("cushion_left", pos=(SH_X, (_hy + Y1C) / 2, _cz), size=(_hx1 - _hx0, Y1C - _hy, CZ - TOP), color=GREY),
    Box("cushion_right", pos=(SH_X, -(_hy + Y1C) / 2, _cz), size=(_hx1 - _hx0, Y1C - _hy, CZ - TOP), color=GREY),
]

# shell spring (vertical prismatic joint world -> shell, see _install_spawn)
SPRING_K = 300.0  # N/m
SPRING_PRE = 4.0  # N net upward push against the top stop at rest (beyond the shell's weight)
SPRING_C = 12.0  # N s/m
TRAVEL = 0.030  # m the shell can be pushed down
SHELL = Box("seat_shell", pos=(SH_X, SH_Y, SH_Z0), size=(SH_L, SH_W, SH_H), mass=SH_MASS, color=BLUE)

# belt path: two ridges on the shell top across its rear, 2.6 cm apart
XB = 0.35  # belt line (strap runs along y here)
BP_GAP, RG_T, RG_H = 0.026, 0.004, 0.011
COVER_W = 0.12  # belt-path cover bar (y length) bridging the ridges over the middle of the shell
RIDGES = [Box(f"belt_ridge_{t}", pos=(XB + s * (BP_GAP / 2 + RG_T / 2), SH_Y, SH_TOP0 + RG_H / 2),
              size=(RG_T, SH_W, RG_H), mass=0.01, color=BLUE) for t, s in (("front", -1), ("back", 1))]
RIDGES.append(Box("belt_cover", pos=(XB, SH_Y, SH_TOP0 + RG_H + 0.002), size=(BP_GAP + 2 * RG_T, COVER_W, 0.004),
                  mass=0.01, color=BLUE))

# -- strap: hooked to the left lower anchor, over the shell in the belt path, into the right connector -----------------
R = 0.004
ZA = CZ + 0.008  # strap centre height at both anchors
ZT = SH_TOP0 + R  # strap centre on the shell top
PIVOT = (XB, 0.150, ZA)
ANCHOR_L = Box("anchor_left", pos=(XB, 0.1675, CZ + 0.006), size=(0.030, 0.015, 0.012), color=DARK)

# connector housing on the right anchor: a roofed channel along -y, entry lips (inner stop), slotted end wall
PLUG = Plug(size=(0.030, 0.020, 0.014), mass=0.025, color=DARK)
Y_LIP = -0.180  # lip outer face (plug inner stop)
Y_ENTRY = -0.140  # connector mouth (side walls start; roof starts 8 mm further in)
Y_P0 = Y_LIP - 0.002 - PLUG.size[0] / 2  # plug centre at start (against the lips)
Y_WALL = -0.268  # end wall inner face
Y_PMAX = Y_WALL - PLUG.size[0] / 2  # plug centre at the end wall
CH_HW = 0.012  # channel half width (x)
WT = 0.004
H_IN = 0.020  # channel inner height
_hy0, _hy1 = Y_ENTRY, Y_WALL - WT
_hyc, _hyl = (_hy0 + _hy1) / 2, _hy0 - _hy1
HOUSING = [
    Box("conn_side_a", pos=(XB - CH_HW - WT / 2, _hyc, CZ + H_IN / 2), size=(WT, _hyl, H_IN), color=DARK),
    Box("conn_side_b", pos=(XB + CH_HW + WT / 2, _hyc, CZ + H_IN / 2), size=(WT, _hyl, H_IN), color=DARK),
    Box("conn_roof", pos=(XB, (Y_ENTRY - 0.008 + _hy1) / 2, CZ + H_IN + WT / 2), size=(2 * CH_HW + 2 * WT, Y_ENTRY - 0.008 - _hy1, WT),
        color=DARK),
    Box("conn_lip_a", pos=(XB - 0.009, Y_LIP + 0.002, CZ + H_IN / 2), size=(0.006, 0.004, H_IN), color=DARK),
    Box("conn_lip_b", pos=(XB + 0.009, Y_LIP + 0.002, CZ + H_IN / 2), size=(0.006, 0.004, H_IN), color=DARK),
]
SLOT_HW, SLOT_HH = 0.009, 0.0065  # end-wall slot half width (x) / half height (z) round the tail
_wy = Y_WALL - WT / 2
_sw = CH_HW + WT - SLOT_HW
HOUSING += [
    Box("conn_wall_a", pos=(XB - SLOT_HW - _sw / 2, _wy, CZ + H_IN / 2), size=(_sw, WT, H_IN), color=DARK),
    Box("conn_wall_b", pos=(XB + SLOT_HW + _sw / 2, _wy, CZ + H_IN / 2), size=(_sw, WT, H_IN), color=DARK),
    Box("conn_wall_lo", pos=(XB, _wy, (CZ + ZA - SLOT_HH) / 2), size=(2 * SLOT_HW, WT, ZA - SLOT_HH - CZ), color=DARK),
    Box("conn_wall_hi", pos=(XB, _wy, (ZA + SLOT_HH + CZ + H_IN) / 2), size=(2 * SLOT_HW, WT, CZ + H_IN - ZA - SLOT_HH),
        color=DARK),
]


def _smooth(p: np.ndarray, it: int = 3) -> np.ndarray:
    """Chaikin corner cutting, ends kept."""
    for _ in range(it):
        q = [p[0]]
        for a, b in zip(p[:-1], p[1:]):
            q += [0.75 * a + 0.25 * b, 0.25 * a + 0.75 * b]
        q.append(p[-1])
        p = np.array(q)
    return p


SLACK_BOW = 0.014  # m: outward bow of the left leg at the start (the slack)


def strap_path() -> np.ndarray:
    """Start layout of the strap centreline (pivot -> left leg -> belt path -> right leg -> into the connector)."""
    ce = 0.106  # corner (centre line passes just outside the shell's top edges)
    zc = ZT + 0.0015
    yin = Y_ENTRY - 0.005  # strap level with the plug from here in

    def leg(a, b, sgn, k=1.0):
        a, b = np.asarray(a), np.asarray(b)
        m = (a + b) / 2
        d = b - a
        n = np.array([0.0, sgn * abs(d[2]), abs(d[1])])
        n /= np.linalg.norm(n)
        b_ = k * SLACK_BOW
        return [a + 0.25 * d + 0.7 * b_ * n, m + b_ * n, a + 0.75 * d + 0.7 * b_ * n]

    p0 = np.array(PIVOT)
    pts = [p0, p0 + [0.0, -0.004, 0.006]]
    pts += leg(pts[-1], (XB, ce, zc), +1, 1.45)
    pts += [(XB, ce, zc), (XB, ce - 0.012, zc), (XB, -ce + 0.012, zc), (XB, -ce, zc)]
    e0 = np.array((XB, yin + 0.012, ZA + 0.010))
    pts += [e0, (XB, yin, ZA + 0.001), (XB, yin - 0.010, ZA), (XB, yin - 0.08, ZA)]
    return _smooth(np.array(pts, dtype=float), 3)


def _cut(path: np.ndarray, y_end: float) -> float:
    """Arc length along ``path`` to where its horizontal run inside the connector reaches y = y_end."""
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    acc = 0.0
    for a, b, l in zip(path[:-1], path[1:], seg):
        if a[1] >= y_end >= b[1] and abs(a[2] - ZA) < 2e-3 and abs(b[2] - ZA) < 2e-3:
            return acc + l * (a[1] - y_end) / (a[1] - b[1])
        acc += l
    raise ValueError("path never reaches y_end")


_PATH = strap_path()
STRAP_LEN = _cut(_PATH, Y_P0 + PLUG.size[0] / 2)
# spawned straight along +x from the anchor pivot, clear of everything; reshaped at every reset (cableshape)
STRAP = cable("strap", start=PIVOT, yaw=0.0, length=STRAP_LEN, radius=R, end_plug=PLUG, cable_mass=0.05,
              color=(0.05, 0.05, 0.06), bend_stiffness=0.004, bend_damping=0.0008)

# adjuster tail with pull tab: a rigid strip through the end-wall slot, joined to the plug by a breakaway link
TAIL_L, TAIL_W, TAIL_T = 0.090, 0.014, 0.005
TEAR_N = 4.0  # N: the tab's breakaway link counts as torn above this pull
TAB_K, TAB_C, TAB_MAX = 400.0, 2.0, 0.025  # stretch link: N/m, N s/m, m (the tear is read from its stretch)
TEAR_EXT = TEAR_N / TAB_K
_spawn_plug = np.array(PIVOT) + (STRAP.seg_len * STRAP.segments + PLUG.size[0] / 2) * np.array([1.0, 0.0, 0.0])
TAIL = Box("tail_tab", pos=tuple(float(v) for v in _spawn_plug + [PLUG.size[0] / 2 + TAIL_L / 2, 0.0, 0.0]), size=(TAIL_W, TAIL_L, TAIL_T),
           mass=0.008, color=ORANGE, yaw=math.pi / 2)

OBJECTS = [*CUSHION, SHELL, *RIDGES, ANCHOR_L, *HOUSING, TAIL, STRAP]

TWIN_EXTRA = [
    {"name": "shell_spring", "kind": "joint", "real": "vertical linear slide + compression spring (300 N/m) under the shell",
     "note": "4 N preload against a top stop, 3 cm travel; the shell moves only up and down"},
    {"name": "anchor_hook", "kind": "joint", "real": "strap end hooked to the left anchor block", "note": "ball joint"},
    {"name": "one_way_adjuster", "kind": "joint", "real": "cam-buckle adjuster in the connector",
     "note": "slider slides out freely; resisted (stiff spring, ~2 mm give at 7 N) sliding back in"},
    {"name": "tab_link", "kind": "joint", "real": "magnetic breakaway tab, lets go at 4 N",
     "note": "sim: spring link (400 N/m) along the pull; torn when stretched over 10 mm"},
    {"name": "belt_path_welds", "kind": "joint", "real": "ridges and cover bar printed with the shell", "note": "one body"},
]

# the strap is laid by the reset and the shell is on its slide; nothing here is jittered
VARIATION = {}

# ratchet in the connector (one-way: the plug may slide out freely; sliding back in is resisted)
RATCHET_K, RATCHET_C, RATCHET_MAX = 1200.0, 2.5, 40.0  # N/m, N s/m, N
ADJ_DRAG = 0.5  # N s/m light sliding drag of the plug in the connector, both ways  # N/m, N s/m, N

_ENV: dict = {"env": None, "smax": None, "plug_idx": None, "step": 0}


# -- spawn hook: pivot, connector link, shell spring, ridge welds -----------------------------------------------------
def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_seat_task", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != STRAP.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        w = UsdGeom.Xformable(stage.GetPrimAtPath(root)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        L = STRAP.seg_len
        # strap start hooked to the left anchor: ball joint at the start tip
        j = UsdPhysics.Joint.Define(stage, f"{prim_path}/anchor_hook")
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        j.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + PIVOT[0]), float(w[1] + PIVOT[1]), float(w[2] + PIVOT[2])))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(-L / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        for dof in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        j.CreateExcludeFromArticulationAttr(True)
        # tail joined to the plug's outer face by the tab's stretch link (spring along the pull, all else locked)
        t = UsdPhysics.Joint.Define(stage, f"{prim_path}/tab_link")
        t.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path}/end_plug")])
        t.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{TAIL.name}")])
        t.CreateLocalPos0Attr(Gf.Vec3f(PLUG.size[0] / 2, 0.0, 0.0))
        t.CreateLocalRot0Attr(Gf.Quatf(math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)))
        t.CreateLocalPos1Attr(Gf.Vec3f(0.0, TAIL_L / 2, 0.0))
        t.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        for dof in ("transX", "transZ", "rotX", "rotY", "rotZ"):
            lim = UsdPhysics.LimitAPI.Apply(t.GetPrim(), dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        lim = UsdPhysics.LimitAPI.Apply(t.GetPrim(), "transY")
        lim.CreateLowAttr(-TAB_MAX)
        lim.CreateHighAttr(0.0)
        dv = UsdPhysics.DriveAPI.Apply(t.GetPrim(), "transY")
        dv.CreateTypeAttr("force")
        dv.CreateStiffnessAttr(TAB_K)
        dv.CreateDampingAttr(TAB_C)
        dv.CreateTargetPositionAttr(0.0)
        t.CreateExcludeFromArticulationAttr(True)
        fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"{root}/{TAIL.name}"))
        fp.CreateFilteredPairsRel().SetTargets([Sdf.Path(f"{prim_path}/end_plug")])
        # shell on a vertical spring
        s = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/shell_spring")
        s.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{SHELL.name}")])
        s.CreateAxisAttr("Z")
        s.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + SH_X), float(w[1] + SH_Y), float(w[2] + SH_Z0)))
        s.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        s.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        s.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        s.CreateLowerLimitAttr(-TRAVEL)
        s.CreateUpperLimitAttr(0.0)
        s.CreateExcludeFromArticulationAttr(True)
        drv = UsdPhysics.DriveAPI.Apply(s.GetPrim(), "linear")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(SPRING_K)
        drv.CreateDampingAttr(SPRING_C)
        drv.CreateTargetPositionAttr((SPRING_PRE + (SH_MASS + 0.02) * 9.81) / SPRING_K)
        # belt-path ridges welded to the shell
        parts = [f"{root}/{SHELL.name}"]
        for rg in RIDGES:
            f = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/weld_{rg.name}")
            f.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{SHELL.name}")])
            f.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{rg.name}")])
            f.CreateLocalPos0Attr(Gf.Vec3f(*map(float, np.asarray(rg.pos) - np.asarray(SHELL.pos))))
            f.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            f.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            f.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            f.CreateExcludeFromArticulationAttr(True)
            parts.append(f"{root}/{rg.name}")
        for pth in parts:
            fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(pth))
            fp.CreateFilteredPairsRel().SetTargets([Sdf.Path(q) for q in parts if q != pth])
        return prim

    spawn._seat_task = True
    cablekit._spawn_cable = spawn


_install_spawn()


# -- reset: lay the strap, put the tail on the plug, arm the ratchet ---------------------------------------------------
def _shape(emb) -> None:
    import torch

    _install_ratchet(emb._env)
    base = emb._env.unwrapped
    origin = cableshape._t(base.scene.env_origins)[0]
    sh = cableshape.shape_points(STRAP, _PATH)
    lift = np.array([0.0, 0.0, 0.0015])
    d = sh.dirs[-1]
    plug_c = sh.centres[-1] + d * (STRAP.seg_len / 2 + PLUG.size[0] / 2) + lift
    Fp = sh.frames[-1]  # plug frame = last segment frame
    tail_c = plug_c + Fp @ np.array([PLUG.size[0] / 2, 0.0, 0.0]) - Fp @ (
        cableshape._rotvec_to_mat(np.array([0, 0, math.pi / 2])) @ np.array([0.0, TAIL_L / 2, 0.0]))
    q_tail = cableshape._mat_to_quat_xyzw(Fp @ cableshape._rotvec_to_mat(np.array([0, 0, math.pi / 2])))
    tail = base.scene.rigid_objects[TAIL.name]
    pose = cableshape._t(tail.data.root_pose_w).clone().to(torch.float32)
    pose[0, :3] = origin + torch.tensor(tail_c, dtype=pose.dtype, device=base.device)
    pose[0, 3:7] = torch.tensor(q_tail, dtype=pose.dtype, device=base.device)
    tail.write_root_pose_to_sim_index(root_pose=pose)
    tail.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=base.device))
    cableshape.preshape(emb, STRAP, _PATH, settle_s=0, hold="ramp")
    _ENV["smax"] = None
    cableshape.settle(emb, 2.0)
    _MON.update(torn=False, out=False, step0=_ENV["step"])
    if os.environ.get("SEAT_DEBUG"):
        print(f"[seat] reset: plug_c={np.round(plug_c, 4)} tail_c={np.round(tail_c, 4)} q_tail={np.round(q_tail, 3)}",
              flush=True)


cableshape.on_reset(_shape)


def _ratchet(art, dt) -> None:
    """Before every physics step: resist the plug sliding back into the connector (one-way adjuster)."""
    import torch

    i = _ENV["plug_idx"]
    if i is None:
        i = list(art.body_names).index("end_plug")
        _ENV["plug_idx"] = i
    p = cableshape._t(art.data.body_link_pos_w)[0, i]
    v = cableshape._t(art.data.body_link_lin_vel_w)[0, i]
    y, vy = float(p[1]), float(v[1])
    if not (math.isfinite(y) and math.isfinite(vy)):
        return
    s = -y  # outward travel coordinate (env origin y = 0 for env 0)
    if _ENV["smax"] is None or s > _ENV["smax"]:
        _ENV["smax"] = s
    back = _ENV["smax"] - s
    f = -ADJ_DRAG * vy  # outward (-y) positive
    if back > 0.0:
        f += min(RATCHET_MAX, RATCHET_K * back + RATCHET_C * max(vy, 0.0))
    _ENV["rcalls"] = _ENV.get("rcalls", 0) + 1
    if os.environ.get("SEAT_DEBUG") and _ENV["rcalls"] % 240 == 0:
        print(f"[seat] ratchet: body={art.body_names[i]} y={y:.4f} smax={_ENV['smax']:.4f} f={f:.2f}", flush=True)
    t = torch.zeros((1, 1, 3), dtype=torch.float32, device=art.device)
    t[0, 0, 1] = -f
    art.instantaneous_wrench_composer.set_forces_and_torques_index(forces=t, body_ids=[i], is_global=True)


def _install_ratchet(env) -> None:
    base = env.unwrapped
    art = base.scene[STRAP.name]
    if getattr(art, "_seat_ratchet", False):
        return
    orig = art.write_data_to_sim
    dt = float(base.physics_dt)

    def write_data_to_sim():
        _ratchet(art, dt)
        return orig()

    art.write_data_to_sim = write_data_to_sim
    art._seat_ratchet = True


_MON: dict = {"torn": False, "out": False, "step0": 0, "last": -1}


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_seat_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        _install_ratchet(self._env)
        _ENV["step"] += 1
        return base(self, *a, **k)

    _task_step._seat_hooked = True
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


def _reset_episode() -> None:
    _ENV["smax"] = None
    _MON.update(torn=False, out=False, last=-1)


_install_env_hook()


def grippers() -> list[np.ndarray]:
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- geometry -----------------------------------------------------------------------------------------------------------
FULL, PART = 0.018, 0.010  # compression for 1.0 / 0.5
RELEASE = 0.03


def _finite(objects) -> bool:
    s, sh, t = objects["strap"], objects[SHELL.name], objects[TAIL.name]
    return bool(np.isfinite(s.points).all() and np.isfinite(s.end_plug).all() and np.isfinite(sh.pos).all()
                and np.isfinite(t.pos).all())


def compression(objects) -> float:
    return float(SH_Z0 - objects[SHELL.name].pos[2])


def _rot(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tab_gap(objects) -> float:
    """Distance between the tail's inner end and the plug's outer face (0 while the link holds)."""
    s, t = objects["strap"], objects[TAIL.name]
    inner = np.asarray(t.pos) + _rot(t.quat_xyzw) @ np.array([0.0, TAIL_L / 2, 0.0])
    face = np.asarray(s.end_plug) + PLUG.size[0] / 2 * np.asarray(s.end_dir)
    return float(np.linalg.norm(inner - face))


def plug_travel(objects) -> float:
    return float(Y_P0 - objects["strap"].end_plug[1])


def in_belt_path(objects) -> bool:
    p = objects["strap"].points
    top = float(objects[SHELL.name].pos[2]) + SH_H / 2
    m = np.abs(p[:, 1] - SH_Y) <= COVER_W / 2
    if not m.any():
        return False
    q = p[m]
    return bool((np.abs(q[:, 0] - XB) <= BP_GAP / 2).all() and (q[:, 2] <= top + RG_H + 0.004 + R).all())


def released(objects) -> bool:
    """Both grippers at least 3 cm from every strap point, the tail/tab and the shell (incl. its belt-path parts)."""
    pts = objects["strap"].polyline()
    t = objects[TAIL.name]
    tail_pts = np.asarray(t.pos) + np.outer(np.linspace(-0.5, 0.5, 7) * TAIL_L, _rot(t.quat_xyzw)[:, 1])
    sh = np.asarray(objects[SHELL.name].pos)
    half = np.array([SH_L / 2, SH_W / 2, SH_H / 2 + RG_H + 0.004])
    for g in grippers():
        if float(np.linalg.norm(np.vstack([pts, tail_pts]) - g, axis=1).min()) < RELEASE:
            return False
        if float(np.linalg.norm(np.maximum(np.abs(g - sh) - half, 0.0))) < RELEASE:
            return False
    return True


def _track(objects) -> None:
    """Latches, evaluated on every call (score() runs every control step): tab torn; strap out of the belt path."""
    if not _finite(objects):
        return
    if tab_gap(objects) > TEAR_EXT + 0.002:
        _MON["torn"] = True
    if _ENV["step"] - _MON.get("step0", 0) > 2 and not in_belt_path(objects):
        _MON["out"] = True


def score(objects) -> float:
    """0 if the state is non-finite, the tab's breakaway link was ever pulled past its tear force (4 N), the strap
    ever left the covered belt path, or a gripper is within 3 cm of the strap, the tab or the seat shell. Otherwise
    by how far the shell sits below its rest height (held there by the strap): >= 18 mm 1.0; >= 10 mm 0.5; less 0."""
    if not _finite(objects):
        return 0.0
    _track(objects)
    if _MON["torn"] or _MON["out"] or not released(objects):
        return 0.0
    c = compression(objects)
    return 1.0 if c >= FULL else 0.5 if c >= PART else 0.0


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and objects[SHELL.name].resting(0.01, 0.5) and objects["strap"].resting(0.04, 2.0))


def distance(objects) -> float:
    if not _finite(objects):
        return 1.0
    return float(max(0.0, FULL - compression(objects)) + (1.0 if (_MON["torn"] or _MON["out"]) else 0.0))


def progress(objects) -> dict:
    if not _finite(objects):
        return {"slack_taken": 0, "compressed_half": 0, "compressed_full": 0, "tab_intact": 0, "released": 0}
    c = compression(objects)
    return {"slack_taken": int(plug_travel(objects) >= 0.02), "compressed_half": int(c >= PART),
            "compressed_full": int(c >= FULL), "tab_intact": int(not _MON["torn"]), "released": int(released(objects))}


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
GRASP_OPEN = -0.016  # pads ~32 mm apart
PRESS_XY = (0.255, 0.04)  # left fingertips on the shell top, clear of the belt path
TAB_YAW = math.pi / 2  # right fingers close across the tail (along x); its wrist camera looks back at the connector


def _log(ctx, tag=""):
    st = ctx.states()
    if not _finite(st):
        print(f"[t={ctx.t:5.2f}] {tag}: NON-FINITE STATE", flush=True)
        return
    print(f"[t={ctx.t:5.2f}] {tag}: comp={compression(st) * 1000:.1f}mm travel={plug_travel(st) * 1000:.1f}mm "
          f"stretch={tab_gap(st) * 1000:.1f}mm inpath={in_belt_path(st)} torn={_MON['torn']} out={_MON['out']} "
          f"rel={released(st)} score={score(st):.2f}", flush=True)
    if os.environ.get("SEAT_DEBUG"):
        sh, sp = st[SHELL.name], st["strap"]
        print(f"   v: shell {np.linalg.norm(sh.lin_vel):.4f} {np.linalg.norm(sh.ang_vel):.3f} strap "
              f"{np.linalg.norm(sp.lin_vel):.4f} {np.linalg.norm(sp.ang_vel):.3f}", flush=True)


def _press(ctx, depth):
    """Left arm: closed fingers down on the shell top until it sits ``depth`` below rest (closed loop)."""
    from yam_arena.constants import FINGER_CLOSED

    L = ctx.left
    x, y = PRESS_XY
    yield from L.move((x, y, SH_TOP0 + 0.06), yaw=0.0, finger=FINGER_CLOSED, duration=1.5)
    yield from L.move((x, y, SH_TOP0 + 0.016), yaw=0.0, duration=0.8)
    z = SH_TOP0 + 0.013 - depth
    yield from L.move((x, y, z), yaw=0.0, duration=1.5)
    for k in range(5):
        yield from L.wait(0.3)
        c = compression(ctx.states())
        _log(ctx, f"press {k}")
        if c >= depth - 0.001:
            break
        z -= depth - c
        yield from L.move((x, y, z), yaw=0.0, duration=0.5)


def _take_tab(ctx):
    """Right arm: pinch the tail's pull tab (its outer end) across its width."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    t = np.asarray(ctx.states()[TAIL.name].pos, dtype=float)
    gy = t[1] - TAIL_L / 2 + 0.014
    yield from Rt.move((XB, gy, t[2] + 0.06), yaw=TAB_YAW, finger=GRASP_OPEN, duration=1.5)
    yield from Rt.move((XB, gy, t[2] + 0.010), yaw=TAB_YAW, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.6)


def _pull(ctx, step, dur, stop_stretch=None):
    """Right arm: draw the tab straight out in small steps; stop when the tab link starts to stretch (the strap is
    taut: the tension is felt), when the plug reaches the end wall, or when the tab tore."""
    Rt = ctx.right
    p0 = Rt.tcp().copy()
    y = p0[1]
    for k in range(int(0.09 / step)):
        y -= step
        yield from Rt.move((p0[0], y, p0[2]), yaw=TAB_YAW, duration=dur)
        st = ctx.states()
        if k % 5 == 0:
            _log(ctx, f"pull {k}")
        if _MON["torn"]:
            _log(ctx, "tab torn")
            return
        if stop_stretch is not None and tab_gap(st) > stop_stretch:
            _log(ctx, "tension felt")
            return
        if plug_travel(st) >= Y_P0 - Y_PMAX - 0.002:
            _log(ctx, "plug at end wall")
            return


def _let_go_right(ctx):
    """Open, lift straight up, sweep back toward the right arm's base and aside."""
    Rt = ctx.right
    p = Rt.tcp().copy()
    yield from Rt.grip(GRASP_OPEN, 0.5)
    yield from Rt.move((p[0], p[1], p[2] + 0.04), yaw=TAB_YAW, duration=0.6)
    yield from Rt.move((0.05, -0.36, TOP + 0.24), yaw=0.0, duration=1.0)


def _let_go_left(ctx):
    """Ease off the shell slowly (it rises against the strap) to just above it, then lift 4 cm straight up and sweep
    back toward the left arm's base and aside, quickly, so the top camera sees the fitted seat at the end."""
    L = ctx.left
    p = L.tcp().copy()
    yield from L.move((p[0], p[1], SH_TOP0 + 0.003), yaw=0.0, duration=1.5)
    yield from L.wait(0.5)
    yield from L.move((p[0], p[1], SH_TOP0 + 0.043), yaw=0.0, duration=0.4)
    yield from L.move((0.05, 0.36, TOP + 0.24), yaw=0.0, duration=0.7)


def _end(ctx):
    for _ in range(4):
        yield from ctx.left.wait(0.5)
        _log(ctx, "end")


def demo_script(ctx):
    """SEAT_DEMO=correct (default): left arm presses the shell 25 mm down and holds it; right arm draws the tab out
    until the strap is taut; right lets go, then left eases off and the spring pushes the shell up against the strap.
    Shortcut / check variants: none | pullonly (one arm: tab pulled until just under its tear force, seat never
    pressed) | yank (tab pulled hard and fast, seat never pressed) | pressonly (press and let go, tab never touched) |
    hold (correct, but the left arm keeps pressing at the end) | holdtab (correct, but the right arm keeps the tab) |
    pulltear (seat pressed, but the tab is pulled on after the strap is taut) | half (correct method, pressed only
    13 mm: partial-credit check, expect 0.5)."""
    mode = os.environ.get("SEAT_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    yield from L.wait(1.0)
    _log(ctx, "settled")
    if mode == "none":
        yield from L.wait(4.0)
        yield from _end(ctx)
        return
    if mode in ("pullonly", "yank"):
        yield from _take_tab(ctx)
        if mode == "pullonly":
            yield from _pull(ctx, 0.002, 0.2, stop_stretch=TEAR_EXT - 0.0005)
            yield from Rt.wait(1.0)
        else:
            yield from _pull(ctx, 0.01, 0.1)
        _log(ctx, "pulled")
        yield from _let_go_right(ctx)
        yield from _end(ctx)
        return
    if mode == "pressonly":
        yield from _press(ctx, 0.025)
        yield from L.wait(1.0)
        yield from _let_go_left(ctx)
        yield from _end(ctx)
        return
    depth = 0.013 if mode == "half" else 0.025
    yield from ctx.both(_press(ctx, depth), _take_tab(ctx))
    _log(ctx, "pressed, tab held")
    yield from _pull(ctx, 0.002, 0.15, stop_stretch=None if mode == "pulltear" else 0.006)
    yield from Rt.wait(0.5)
    _log(ctx, "slack taken up")
    if mode == "holdtab":
        yield from _let_go_left(ctx)
        yield from _end(ctx)
        return
    yield from _let_go_right(ctx)
    if mode == "hold":
        yield from _end(ctx)
        return
    yield from _let_go_left(ctx)
    yield from _end(ctx)
