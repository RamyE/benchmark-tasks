"""Seat a Cisco N9K-X9736C-FX line card in slot 4 of an unpowered Cisco Nexus 9504 switch chassis.

Scene (metres from the arm-base midpoint, x forward, y left; heights above the table). An unpowered Cisco Nexus 9504
chassis (44.5 cm wide along y, 31.1 cm / 7 RU tall, front face at x 0.360) stands on the bench, fixed, front toward
the robot. Line card slots 1-3 hold blank line cards; slot 4 (floor z 0.105) is empty but for the new line card. The
N9K-X9736C-FX 36-port 100G line card (43.18 cm wide, 41.91 cm deep inside the chassis, 4.4 cm tall, 6.62 kg) rests
in slot 4's side guides with its back 30 cm in and its faceplate 12.0 cm proud of the chassis front. Its two ejector
levers (one at each end of the faceplate) are rotated 45 deg open.

Cisco's Nexus 9504 guide: slide the card in until it stops with its front about 0.6 cm outside the chassis, then
rotate the ends of the two levers toward the chassis until they click; the levers seat the card on the fabric modules.
The levers are 43 cm apart, beyond one arm's reach of both; the card's 0.5 mm guide clearance limits how far one end
can get ahead of the other (~1 mm), so levering one end alone cannot seat the card.

Run it:
  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 600 \\
    --instruction "Install the line card in the switch's empty slot, then let go."
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Install the line card in the switch's empty slot, then let go."
RUN_LIMIT_STEPS = 600  # 30 s at 20 Hz

# -- colours (sRGB) -------------------------------------------------------------------------------------------------
CHASSIS_COL = (0.16, 0.17, 0.19)     # dark grey powder-coated steel
BLANK_COL = (0.24, 0.25, 0.27)       # blank line-card covers
FACE_COL = (0.55, 0.57, 0.60)        # line card faceplate (brushed steel)
PORT_COL = (0.05, 0.05, 0.06)        # QSFP28 port cages
LEVER_COL = (0.93, 0.93, 0.90)       # light ejector levers (high contrast on the dark chassis)
TRAY_COL = (0.10, 0.30, 0.16)        # green PCB / tray
LED_COL = (0.10, 0.85, 0.25)
SUP_COL = (0.12, 0.12, 0.13)

# -- chassis (Cisco Nexus 9504: 44.5 W x 84.20 D (with handles) x 31.1 H cm, 7 RU) ---------------------------------
X_F = 0.360                 # chassis front face (line card stop / flush reference)
CH_W = 0.445
CH_H = 0.311
CH_D = 0.780                # body depth modelled (handles omitted); the rear is far out of reach
# -- line card (N9K-X9736C-FX: 43.18 cm wide, 41.91 cm inside chassis, ejector levers 6.35 cm outside, 6.62 kg) -----
CARD_W = 0.4318
CARD_D = 0.4191
CARD_H = 0.044              # 1.75 in module height
CARD_M = 6.62
FACE_T = 0.0435             # faceplate + QSFP28 cage block in front of the chassis line (levers add 2.0 cm -> 6.35 cm)
CLEAR_SIDE = 0.0005         # guide clearance per side
SLOT_W = CARD_W + 2 * CLEAR_SIDE
Z_SLOT = 0.105              # slot-4 floor (top of supervisor / system-controller section)
Z_CEIL = Z_SLOT + CARD_H + 0.0015
WALL = (CH_W - SLOT_W) / 2  # 6.1 mm side walls
# -- ejector levers ---------------------------------------------------------------------------------------------------
LEV_L = 0.100               # pivot to tip
LEV_T = 0.020               # thickness along x (closed)
LEV_H = 0.030
LEV_M = 0.12
PIV_Y = CARD_W / 2 - 0.012  # pivot 12 mm in from each card end
OPEN_DEG = 45.0             # levers start rotated 45 deg open (Cisco: "rotate both levers 45 degrees")
LIMIT_DEG = 50.0
# -- start / stop / seat ----------------------------------------------------------------------------------------------
P0 = 0.120                  # faceplate starts 12.0 cm proud of the chassis front
P_STOP = 0.006              # open levers' knobs meet the chassis edge 0.6 cm proud (Cisco guide)
R_EFF = P_STOP / math.radians(OPEN_DEG)   # 7.64 mm: lever rotation 45 deg -> 6 mm of travel at its end
CAM_K = 1.0e5               # N/m knob-on-chassis contact stiffness
CAM_C = 600.0               # N s/m
CAM_MAX_N = 150.0           # most a lever's knob transmits to its end of the card
FABRIC_N = 50.0             # fabric connector resistance per card end over the last 6 mm (assumed)
FABRIC_RAMP = 0.0005        # the 50 N builds up over the first 0.5 mm past the stop
HOOK_MIN_DEG = 40.0         # a lever must be at least 40 deg open as its end reaches the stop for its knob to catch
LATCH_DEG = 2.0             # lever clicks when within 2 deg of closed with its end seated
LATCH_SEAT = 0.0015
LATCH_KR = 40.0             # N m/rad holding torque of a clicked lever
LATCH_CR = 0.4
PIVOT_C = 0.02              # N m s/rad pivot damping
DT_SUB = 1.0 / 240.0

CARD_X0 = X_F + CARD_D / 2 - P0
CARD_Z0 = Z_SLOT + CARD_H / 2


def _lever_pose(side: int, theta_deg: float, p: float = P0):
    """World (table-frame) centre and yaw of a lever. side +1 = left (+y) end, -1 = right end."""
    px = X_F - p - FACE_T - LEV_T / 2
    py = side * PIV_Y
    yaw = -side * math.radians(theta_deg)
    d = np.array([0.0, -side * LEV_L / 2])          # pivot -> centre when closed (toward the card centre)
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = c * d[0] - s * d[1], s * d[0] + c * d[1]
    return (float(px + dx), float(py + dy)), float(yaw)


CARD = Box("line_card", pos=(float(CARD_X0), 0.0, float(TOP + CARD_Z0)), size=(CARD_D, CARD_W, CARD_H), color=TRAY_COL, mass=CARD_M)
_lp, _ly = _lever_pose(+1, OPEN_DEG)
LEVER_L = Box("lever_left", pos=(_lp[0], _lp[1], float(TOP + CARD_Z0)), size=(LEV_T, LEV_L, LEV_H), color=LEVER_COL,
              mass=LEV_M, yaw=_ly)
_rp, _ry = _lever_pose(-1, OPEN_DEG)
LEVER_R = Box("lever_right", pos=(_rp[0], _rp[1], float(TOP + CARD_Z0)), size=(LEV_T, LEV_L, LEV_H), color=LEVER_COL,
              mass=LEV_M, yaw=_ry)


def _chassis_parts() -> list[Box]:
    xc = X_F + CH_D / 2
    parts = [
        # lower section (supervisors / system controllers / PSU area), its top is slot 4's floor
        Box("ch_lower", pos=(xc, 0.0, TOP + Z_SLOT / 2), size=(CH_D, CH_W, Z_SLOT), color=CHASSIS_COL),
        # upper section (slots 1-3 with blank covers), its underside is slot 4's ceiling
        Box("ch_upper", pos=(xc, 0.0, TOP + (Z_CEIL + CH_H) / 2), size=(CH_D, CH_W, CH_H - Z_CEIL), color=CHASSIS_COL),
        # slot 4 side walls (guides)
        Box("ch_wall_l", pos=(xc, SLOT_W / 2 + WALL / 2, TOP + (Z_SLOT + Z_CEIL) / 2), size=(CH_D, WALL, Z_CEIL - Z_SLOT),
            color=CHASSIS_COL),
        Box("ch_wall_r", pos=(xc, -SLOT_W / 2 - WALL / 2, TOP + (Z_SLOT + Z_CEIL) / 2),
            size=(CH_D, WALL, Z_CEIL - Z_SLOT), color=CHASSIS_COL),
        # fabric-module wall behind the seated card (3 mm behind its back edge)
        Box("ch_fabric", pos=(X_F + CARD_D + 0.003 + 0.01, 0.0, TOP + (Z_SLOT + Z_CEIL) / 2),
            size=(0.02, SLOT_W, Z_CEIL - Z_SLOT), color=SUP_COL),
    ]
    # blank covers on slots 1-3 and supervisor/controller faceplates (thin, flush with the front)
    for i, z0 in enumerate((Z_CEIL + 0.001, Z_CEIL + 0.0455, Z_CEIL + 0.091)):
        parts.append(Box(f"blank_{i + 1}", pos=(X_F - 0.002, 0.0, TOP + z0 + 0.021), size=(0.004, CARD_W, 0.042),
                         color=BLANK_COL))
    for i, (z0, h) in enumerate(((0.062, 0.040), (0.018, 0.040))):
        parts.append(Box(f"sup_face_{i + 1}", pos=(X_F - 0.002, 0.0, TOP + z0 + h / 2), size=(0.004, 0.40, h),
                         color=SUP_COL))
    return parts


OBJECTS = [*_chassis_parts(), CARD, LEVER_L, LEVER_R]

TWIN_EXTRA = [
    {"name": "lever_pivots", "kind": "joint", "real": "line card ejector lever pivot pins at each faceplate end",
     "note": "sim: revolute joints card->lever about z, 0-50 deg"},
    {"name": "lever_knobs", "kind": "force",
     "real": "lever knob bearing on the chassis edge: stop at 0.6 cm proud, levers the card home as it rotates",
     "note": "sim: stiff contact coupling lever angle to that card end's travel (6 mm over 45 deg), <=150 N"},
    {"name": "fabric_connectors", "kind": "force", "real": "line card to fabric module connectors",
     "note": "sim: 50 N push-back per card end over the last 6 mm, speed-independent"},
]

_P: dict = {}


# -- USD authoring: faceplate, ports, lever joints, physics materials ------------------------------------------------
def _author(root: str) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    card = f"{root}/{CARD.name}"
    _P.update(card=card, root=root, lever_left=f"{root}/{LEVER_L.name}", lever_right=f"{root}/{LEVER_R.name}")

    def look(name, rgb, rough=0.4, metal=0.3):
        path = f"{root}/Looks/{name}"
        if not stage.GetPrimAtPath(path).IsValid():
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(rgb), roughness=rough, metallic=metal)
            cfg.func(path, cfg)
        return path

    def box(parent, name, ctr, size, rgb, collide=True, rough=0.4, metal=0.3):
        path = f"{parent}/{name}"
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
        x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, ctr)))
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        bind_visual_material(path, look(name, rgb, rough, metal), stage=stage)
        if collide:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        return path

    # Card frame: origin at tray centre; -x toward the faceplate.  Child prims are in the (unscaled) body frame.
    fx = -CARD_D / 2 - FACE_T / 2
    box(card, "faceplate", (fx, 0.0, 0.0), (FACE_T, CARD_W, CARD_H), FACE_COL, collide=True, metal=0.6)
    # 36 QSFP28 cages in two rows (visual) + status LED
    for row, zz in enumerate((0.009, -0.009)):
        for k in range(18):
            yy = -0.150 + k * (0.300 / 17)
            box(card, f"qsfp_{row}_{k}", (-CARD_D / 2 - FACE_T - 0.0004, yy, zz), (0.002, 0.0150, 0.0110), PORT_COL,
                collide=False, rough=0.6, metal=0.2)
    box(card, "status_led", (-CARD_D / 2 - FACE_T - 0.0006, 0.170, 0.012), (0.002, 0.004, 0.004), LED_COL,
        collide=False, rough=0.3, metal=0.0)

    # Lever pivots: revolute joints about z between the card and each lever (collision between them disabled)
    for side, key in ((+1, "lever_left"), (-1, "lever_right")):
        jp = f"{root}/{key}_pivot"
        if stage.GetPrimAtPath(jp).IsValid():
            continue
        j = UsdPhysics.RevoluteJoint.Define(stage, jp)
        j.CreateBody0Rel().SetTargets([Sdf.Path(card)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(_P[key])])
        j.CreateAxisAttr("Z")
        j.CreateLocalPos0Attr(Gf.Vec3f(float(-CARD_D / 2 - FACE_T - LEV_T / 2), float(side * PIV_Y), 0.0))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, float(side * LEV_L / 2), 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        if side > 0:
            j.CreateLowerLimitAttr(-LIMIT_DEG); j.CreateUpperLimitAttr(0.0)
        else:
            j.CreateLowerLimitAttr(0.0); j.CreateUpperLimitAttr(LIMIT_DEG)
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateCollisionEnabledAttr(False)
        # grip ridge on the lever tip (visual)
        box(_P[key], "tip_ridge", (-LEV_T / 2 - 0.001, -side * (LEV_L / 2 - 0.010), 0.0), (0.002, 0.016, LEV_H - 0.004),
            PORT_COL, collide=False)

    # Physics materials: low-friction steel guides, card steel; levers moderate
    m_card = f"{root}/card_material"
    c1 = sim_utils.RigidBodyMaterialCfg(static_friction=0.20, dynamic_friction=0.15, restitution=0.0)
    c1.func(m_card, c1)
    m_lev = f"{root}/lever_material"
    c2 = sim_utils.RigidBodyMaterialCfg(static_friction=0.6, dynamic_friction=0.5, restitution=0.0)
    c2.func(m_lev, c2)
    for path, mat in ((card, m_card), (_P["lever_left"], m_lev), (_P["lever_right"], m_lev)):
        for p in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                bind_physics_material(str(p.GetPath()), mat, stage=stage)
    for o in OBJECTS:
        if o.name.startswith(("ch_", "blank_", "sup_")):
            q = stage.GetPrimAtPath(f"{root}/{o.name}")
            if q and q.IsValid():
                for c in Usd.PrimRange(q):
                    if c.HasAPI(UsdPhysics.CollisionAPI):
                        bind_physics_material(str(c.GetPath()), m_card, stage=stage)


def _wrap_cfg(spec, obj):
    from isaaclab.sim.utils import clone

    cfg = obj.spawner_cfg
    orig = cfg.func
    base = getattr(orig, "__wrapped__", orig)

    def spawn(prim_path, c, *a, **k):
        prim = base(prim_path, c, *a, **k)
        _author(str(prim_path).rsplit("/", 1)[0])
        return prim

    cfg.func = clone(spawn)
    obj.object_cfg = obj._init_object_cfg()
    return obj


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_lc_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        obj = orig(spec)
        if getattr(spec, "name", None) == OBJECTS[-1].name:
            _wrap_cfg(spec, obj)
        return obj

    task_object._lc_patched = True
    scene.task_object = task_object


class _SceneFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
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
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _SceneFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _SceneFinder())


# -- runtime state ------------------------------------------------------------------------------------------------------
_FRESH = dict(link=None, env=None, step=0, nan=False, pulled=False, latched_l=False, latched_r=False, views=None, ghist=None,
              g=None, why="", cam=(0.0, 0.0), subs=0, max_stop_n=0.0,
              hook_1=False, **{"hook_-1": False})
_M: dict = {}


def _reset_model() -> None:
    env = _M.get("env")
    _M.clear()
    _M.update(dict(_FRESH))
    _M["env"] = env


_reset_model()


def _qrot(q, v):
    """Rotate v by quaternion q given as (w, x, y, z)."""
    w, x, y, z = q
    u = np.array([x, y, z], dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def _yaw(q) -> float:
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


END_PTS = (np.array([-CARD_D / 2, +CARD_W / 2 - 0.01, 0.0]), np.array([-CARD_D / 2, -CARD_W / 2 + 0.01, 0.0]))


def geometry_from(cp, cq, lq_l, lq_r) -> dict:
    """cp: card centre (table frame), cq/lq_*: (w,x,y,z). Returns per-end proud distances and lever angles."""
    pl = cp + _qrot(cq, END_PTS[0])
    pr = cp + _qrot(cq, END_PTS[1])
    yc = _yaw(cq)
    th_l = -math.degrees(_wrap(_yaw(lq_l) - yc))
    th_r = math.degrees(_wrap(_yaw(lq_r) - yc))
    return dict(p=np.asarray(cp), q=np.asarray(cq), proud_l=float(X_F - pl[0]), proud_r=float(X_F - pr[0]),
                th_l=th_l, th_r=th_r, yaw_deg=math.degrees(yc))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _obj_pose(objects, name):
    o = objects[name]
    x, y, z, w = np.asarray(o.quat_xyzw, dtype=np.float64)
    return np.asarray(o.pos, dtype=np.float64) - np.array([0.0, 0.0, TOP]), np.array([w, x, y, z])


def _poses(objects):
    """Body-frame poses (table frame) of card, left lever, right lever: from the PhysX views when live (the object
    states report the card's compound centre of mass, which sits forward of the tray centre), else from ``objects``."""
    lk = _M.get("link")
    if lk is not None:
        return lk
    return tuple(_obj_pose(objects, n) for n in (CARD.name, LEVER_L.name, LEVER_R.name))


def geometry(objects) -> dict:
    (cp, cq), (_, ql), (_, qr) = _poses(objects)
    return geometry_from(cp, cq, ql, qr)


# -- PhysX substep model: lever knobs on the chassis edge, fabric connector resistance, lever click latch -------------
def _views(mgr):
    import warp as wp

    if _M.get("views") is None:
        sv = mgr.get_physics_sim_view()
        if sv is None or "card" not in _P:
            return None
        vs = {k: sv.create_rigid_body_view(_P[k]) for k in ("card", "lever_left", "lever_right")}
        raw = vs["card"].get_transforms()
        _M["dev"] = str(getattr(raw, "device", "cuda:0"))
        _M["idx"] = wp.array(np.array([0], dtype=np.int32), dtype=wp.int32, device=_M["dev"])
        _M["views"] = vs
    return _M["views"]


def _read(v):
    T = np.asarray(v.get_transforms().numpy(), dtype=np.float64).reshape(-1)
    V = np.asarray(v.get_velocities().numpy(), dtype=np.float64).reshape(-1)
    return T, V


def _apply(v, f, t):
    import warp as wp

    v.apply_forces_and_torques_at_position(
        wp.array(np.asarray([f], dtype=np.float32), dtype=wp.float32, device=_M["dev"]),
        wp.array(np.asarray([t], dtype=np.float32), dtype=wp.float32, device=_M["dev"]),
        None, _M["idx"], True,
    )


def _drive(mgr) -> None:
    try:
        vs = _views(mgr)
        if vs is None:
            return
        _M["subs"] += 1
        Tc, Vc = _read(vs["card"])
        Tl, Vl = _read(vs["lever_left"])
        Tr, Vr = _read(vs["lever_right"])
        if not all(np.isfinite(a).all() for a in (Tc, Vc, Tl, Vl, Tr, Vr)):
            _M["nan"] = True
            return
        org = np.zeros(3)
        if _M.get("env") is not None:
            org = _M["env"].unwrapped.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
        cq = np.array([Tc[6], Tc[3], Tc[4], Tc[5]])
        cpos = Tc[:3]
        ql = np.array([Tl[6], Tl[3], Tl[4], Tl[5]])
        qr = np.array([Tr[6], Tr[3], Tr[4], Tr[5]])
        off = org + np.array([0.0, 0.0, TOP])
        _M["link"] = ((cpos - off, cq), (Tl[:3] - off, ql), (Tr[:3] - off, qr))
        yc = _yaw(cq)
        f_tot, t_tot = np.zeros(3), np.zeros(3)
        cams = []
        for side, loc, qv, Vv, view, key in ((+1, END_PTS[0], ql, Vl, vs["lever_left"], "latched_l"),
                                              (-1, END_PTS[1], qr, Vr, vs["lever_right"], "latched_r")):
            r = _qrot(cq, loc)
            pt = cpos + r
            proud = X_F - (pt[0] - org[0])
            vel = Vc[:3] + np.cross(Vc[3:6], r)
            vx = float(vel[0])
            th = -side * _wrap(_yaw(qv) - yc)            # opening angle, rad (>= 0)
            om = -side * float(Vv[5] - Vc[5])            # opening rate
            fx = 0.0
            tau_open = 0.0
            if proud <= P_STOP + 0.0005 and th >= math.radians(HOOK_MIN_DEG):
                _M[f"hook_{side}"] = True                # card reached the stop with the lever open: knob catches the edge
            elif proud > P_STOP + 0.004:
                _M[f"hook_{side}"] = False
            if proud <= P_STOP + 0.002:
                # a hooked knob lets its end in as far as the lever angle allows; a lever that was not open when the
                # card arrived has its knob face against the chassis front, a hard stop at P_STOP (it cannot pull in)
                allow = R_EFF * max(th, 0.0) if _M.get(f"hook_{side}") else P_STOP
                e = proud - allow                         # >0: lever ahead of the card (knob pulls card in)
                if e < 0.0:
                    # card pushed deeper than the lever's knob allows: knob butts on the chassis edge (stop)
                    fx = -min(CAM_K * (-e) + CAM_C * max(vx, 0.0), 2.0 * CAM_MAX_N)
                    _M["max_stop_n"] = max(_M["max_stop_n"], -fx)
                elif _M.get(f"hook_{side}") and e > 0.0002:
                    # knob hooked behind the chassis edge: closing the lever levers this end in
                    fx = min(CAM_K * (e - 0.0002) + CAM_C * max(-vx, 0.0), CAM_MAX_N)
                    tau_open = fx * R_EFF                 # reaction on the lever resists closing
                if proud < P_STOP:
                    # connector mating resistance: a 50 N push-back on this end over the last 6 mm (ramped in over
                    # 0.5 mm), independent of speed, so even a slow close must hold 50 N through the lever
                    fx -= FABRIC_N * min(1.0, (P_STOP - proud) / FABRIC_RAMP)
            cams.append(fx)
            # lever torques (about world z): opening for the left lever is -z, for the right +z
            tz = 0.0
            if _M[key]:
                tz_open = -LATCH_KR * th - LATCH_CR * om
            else:
                tz_open = tau_open - PIVOT_C * om
                if th <= math.radians(LATCH_DEG) and proud <= LATCH_SEAT:
                    _M[key] = True
                    print(f"[lc] {'left' if side > 0 else 'right'} lever clicked at step {_M['step']} "
                          f"(proud {proud * 1000:.2f} mm)", flush=True)
            tz = -side * tz_open
            _apply(view, np.zeros(3), np.array([0.0, 0.0, tz]))
            # the lever torque reacts on the card through the pivot joint; the knob force acts on the card
            fv = np.array([fx, 0.0, 0.0])
            f_tot += fv
            t_tot += np.cross(r, fv)
        _M["cam"] = tuple(cams)
        _apply(vs["card"], f_tot, t_tot)
    except Exception as e:  # pragma: no cover
        if _M.get("err") is None:
            _M["err"] = repr(e)
            print(f"[lc] drive error: {e!r}", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_lc_hooked", False):
        return
    base_step = mgr.step.__func__

    def step(cls) -> None:
        base_step(cls)
        _drive(cls)

    step._lc_hooked = True
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


def _np(t):
    t = getattr(t, "torch", t)
    return np.asarray(t.detach().cpu().numpy() if hasattr(t, "detach") else t, dtype=np.float64)


# -- per-step tracking ------------------------------------------------------------------------------------------------
PULL_OUT = 0.300            # > 30 cm proud at either end (only 11.9 cm of card left in the guides) = pulled back out


def _track(objects) -> None:
    if _M.get("env") is not None and _M.get("link") is None:
        return  # no physics substep yet this episode
    try:
        g = geometry(objects)
    except Exception:  # noqa: BLE001
        return
    if not np.isfinite(g["p"]).all():
        _M["nan"] = True
        return
    _M["g"] = g
    if max(g["proud_l"], g["proud_r"]) > PULL_OUT and not _M["pulled"]:
        _M["pulled"] = True
        _M["why"] = "card pulled back out of its guides"
        print(f"[lc] {_M['why']} at step {_M['step']}", flush=True)


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
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._task_reset_hooks = []
        cls.reset = reset
    if _reset_model not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_model)
    if not getattr(cls._task_step, "_lc_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _M["env"] = self._env
            _M["step"] += 1
            return base(self, *a, **k)

        _task_step._lc_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- scoring ------------------------------------------------------------------------------------------------------------
AT_STOP = 0.008     # 0.3: faceplate <= 0.8 cm proud at both ends (pushed to the levers' stop)
SEATED = 0.001      # 0.7: faceplate flush within 1 mm at both ends
CLEAR = 0.10        # >= 10 cm from card faceplate and levers
STILL_STEPS = 40    # 2.0 s at 20 Hz
STILL_MOVE = 0.01   # <= 1.0 cm TCP excursion over 2.0 s
JAW_OPEN = 0.020    # finger-joint gap >= 20 mm


def grippers() -> list[np.ndarray]:
    env = _M.get("env")
    if env is None:
        return _M.get("mock_grippers") or []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin - np.array([0.0, 0.0, TOP])
            for k in ("left_robot", "right_robot")]


def jaws_open() -> bool:
    env = _M.get("env")
    if env is None:
        return bool(_M.get("mock_jaws_open", False))
    from yam_arena.constants import FINGER_JOINTS

    base = env.unwrapped
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        ids = rob.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        if float(np.abs(_np(rob.data.joint_pos[0])[ids]).sum()) < JAW_OPEN:
            return False
    return True


def _box_dist(pt, ctr, q, lo, hi) -> float:
    R = np.stack([_qrot(q, [1, 0, 0]), _qrot(q, [0, 1, 0]), _qrot(q, [0, 0, 1])], 1)
    loc = R.T @ (np.asarray(pt, dtype=np.float64) - ctr)
    d = np.maximum(np.maximum(lo - loc, loc - hi), 0.0)
    return float(np.linalg.norm(d))


def part_dist(pt, objects) -> float:
    """Distance from a TCP to the card (tray + faceplate) and both levers."""
    (cp, cq), lv_l, lv_r = _poses(objects)
    d = _box_dist(pt, cp, cq, np.array([-CARD_D / 2 - FACE_T, -CARD_W / 2, -CARD_H / 2]),
                  np.array([CARD_D / 2, CARD_W / 2, CARD_H / 2]))
    for lp, lq in (lv_l, lv_r):
        h = np.array([LEV_T / 2, LEV_L / 2, LEV_H / 2])
        d = min(d, _box_dist(pt, lp, lq, -h, h))
    return d


def hands_clear(objects) -> bool:
    g = grippers()
    if len(g) < 2 or not jaws_open():
        _M["ghist"] = []
        return False
    hist = _M["ghist"] if _M["ghist"] is not None else []
    _M["ghist"] = hist
    if not hist or hist[-1][0] != _M["step"]:
        hist.append((_M["step"], [p.copy() for p in g]))
        del hist[:-(STILL_STEPS + 1)]
    still = len(hist) > STILL_STEPS and all(
        max(float(np.linalg.norm(h[1][i] - g[i])) for h in hist) <= STILL_MOVE for i in range(len(g)))
    return still and all(part_dist(t, objects) >= CLEAR for t in g)


def at_stop(g) -> bool:
    return g["proud_l"] <= AT_STOP and g["proud_r"] <= AT_STOP


def seated(g) -> bool:
    return (g["proud_l"] <= SEATED and g["proud_r"] <= SEATED and _M["latched_l"] and _M["latched_r"]
            and g["th_l"] <= LATCH_DEG + 1.0 and g["th_r"] <= LATCH_DEG + 1.0)


def score(objects) -> float:
    """Cumulative task score in {0.0, 0.3, 0.7, 1.0}."""
    _track(objects)
    if _M["nan"] or _M["pulled"]:
        return 0.0
    g = _M.get("g")
    if g is None:
        return 0.0
    if seated(g):
        return 1.0 if hands_clear(objects) else 0.7
    if at_stop(g):
        return 0.3
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def progress(objects) -> dict:
    g = _M.get("g") or {}
    return {
        "at_stop": float(bool(g) and at_stop(g)),
        "seated": float(bool(g) and seated(g)),
        "latched_l": float(_M["latched_l"]), "latched_r": float(_M["latched_r"]),
        "proud_l_mm": round(g.get("proud_l", 0.0) * 1000, 2), "proud_r_mm": round(g.get("proud_r", 0.0) * 1000, 2),
        "th_l_deg": round(g.get("th_l", 0.0), 2), "th_r_deg": round(g.get("th_r", 0.0), 2),
        "pulled": float(_M["pulled"]),
    }


VARIATION = {"obj:line_card": {"dx": 0.01, "dy": 0.0, "yaw": 0.0}}
