"""Open a half-gallon gable-top milk carton's pour spout on its 'OPEN HERE' end, steadying the carton.
Scene (m, x forward, y left): 320x320x24 mm spill tray at (0.30, 0); 2.00 kg 1.89 L Pure-Pak carton (95x95x248 mm).
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
SUCCESS_HOLD_S = 0.05
RUN_LIMIT_STEPS = 600
INSTRUCTION = "Open the milk carton's spout."
CX, CY = 0.300, 0.000
MAT_W, MAT_H, RIM_H, RIM_T = 0.320, 0.002, 0.024, 0.005
SURFACE = TOP + MAT_H
CW, HW = 0.095, 0.0475
WALL_H, ROOF_H, FIN_H, FIN_T = 0.190, 0.045, 0.013, 0.0036
PLY_T, TOTAL_H, CARTON_M = FIN_T / 2, WALL_H + ROOF_H + FIN_H, 2.00
SLOPE_L, ROOF_ANG = math.hypot(HW, ROOF_H), math.atan2(ROOF_H, HW)
OPEN_S, Y_SPLIT = -1.0, 0.002
L_FRONT, Y_FRONT_C = Y_SPLIT + HW, 0.5 * (-HW + Y_SPLIT)
L_REAR, Y_REAR_C = HW - Y_SPLIT, 0.5 * (Y_SPLIT + HW)
SEAL_TQ_OPEN, SEAL_TQ_FAR = 0.075, 0.250
SEAL_BREAK_DEG, WING_OPEN_END_DEG = 25.0, 15.0
FOLD_TQ, WRONG_END_DEG, WING_LIM_DEG, WEDGE_MAX_DEG = 0.015, 15.0, 105.0, 30.0
SEAM_SEAL_TQ, SEAM_FOLD_TQ = 0.085, 0.025
SEAM_BREAK_DEG, SEAM_END_DEG, SEAM_LIM_DEG, THROAT_MIN = 12.0, 10.0, 25.0, 0.020
SPOUT_INSET, SPOUT_DROP, SPOUT_LIM_DEG, SPOUT_TQ = 0.016, 0.000, 70.0, 0.055
LIP_LOCAL = np.array([0.0, 0.000, 0.043])
BEAK_HALF_W, NOSE_T = 0.017, 0.0034

TILT_MAX, OFF_MAT_MAX, STEADY_MIN_N = 20.0, 0.080, 0.5
RELEASE_CLEAR, GRIP_CLEAR, GRIP_SPEED = 0.020, 0.100, 0.020
LIP_HALF, LIP_FULL, HOLD_STEPS = 0.010, 0.020, 20

WHITE, BLUE, RED = (0.96, 0.96, 0.95), (0.09, 0.27, 0.68), (0.82, 0.08, 0.07)
INNER, DARK, MAT_COL, MARK_COL = (0.84, 0.83, 0.78), (0.10, 0.10, 0.12), (0.24, 0.26, 0.29), (0.58, 0.76, 0.94)
_SMALL = (0.003, 0.003, 0.003)


def _wing_origin(s: float, k: float) -> tuple[float, float, float]: return (CX + k * HW, CY + s * HW, SURFACE + WALL_H)
def _seam_origin(k: float) -> tuple[float, float, float]: return (CX + k * HW, CY + Y_FRONT_C, SURFACE + WALL_H)

MAT = Box("drip_mat", pos=(CX, CY, TOP + MAT_H / 2), size=(MAT_W, MAT_W, MAT_H), color=MAT_COL)
CARTON = Box("milk_carton", pos=(CX, CY, SURFACE + WALL_H / 2), size=(CW, CW, WALL_H), mass=CARTON_M, color=WHITE)
SEAMS = {"seam_px": 1.0, "seam_nx": -1.0}
SEAM_OBJS = [Box(n, pos=_seam_origin(k), size=_SMALL, mass=0.02, color=WHITE) for n, k in SEAMS.items()]
WINGS = {"wing_open_px": (OPEN_S, 1.0), "wing_open_nx": (OPEN_S, -1.0), "wing_far_px": (-OPEN_S, 1.0), "wing_far_nx": (-OPEN_S, -1.0)}
WING_OBJS = [Box(n, pos=_wing_origin(s, k), size=_SMALL, mass=0.02, color=WHITE) for n, (s, k) in WINGS.items()]
SPOUT0 = np.array([CX, CY + OPEN_S * (HW - SPOUT_INSET), SURFACE + WALL_H - SPOUT_DROP])
SPOUT = Box("spout", pos=tuple(map(float, SPOUT0)), size=_SMALL, mass=0.025, color=WHITE)
OBJECTS = [MAT, CARTON, *SEAM_OBJS, *WING_OBJS, SPOUT]
DYN_NAMES = [CARTON.name, *(o.name for o in SEAM_OBJS), *(o.name for o in WING_OBJS), SPOUT.name]

TWIN_EXTRA = [{"name": n, "kind": "joint", "real": r, "note": m} for n, r, m in (
    ("top_seam", "front top-fin seam", "hinged + peel seal"), ("gable_wings", "gable wings", "revolute + seal"),
    ("pour_spout", "hinged V-spout", "revolute + 0.055 N m friction"))]
_P: dict = {}
GLYPHS = {"A": (14, 17, 17, 31, 17, 17, 17), "E": (31, 16, 16, 30, 16, 16, 31), "H": (17, 17, 17, 31, 17, 17, 17),
          "I": (31, 4, 4, 4, 4, 4, 31), "K": (17, 18, 20, 24, 20, 18, 17), "L": (16, 16, 16, 16, 16, 16, 31),
          "M": (17, 27, 21, 21, 17, 17, 17), "N": (17, 25, 21, 19, 17, 17, 17), "O": (14, 17, 17, 17, 17, 17, 14),
          "P": (30, 17, 17, 30, 16, 16, 16), "R": (30, 17, 17, 30, 20, 18, 17)}

def _glyph_rects(text: str, px: float) -> list[tuple[float, float, float, float]]:
    out, w = [], px * (6 * len(text) - 1)
    for i, ch in enumerate(text):
        for r, b in enumerate(GLYPHS[ch]):
            row, c = f"{b:05b}", 0
            while c < 5:
                if row[c] == "1":
                    c0 = c
                    while c < 5 and row[c] == "1": c += 1
                    out.append((-w / 2 + (6 * i + (c0 + c) / 2) * px, (3.0 - r) * px, (c - c0) * px, px))
                else: c += 1
    return out

def quat_to_mat(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

def _mat_to_quat(R) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(np.asarray(R, float)).as_quat()

def _qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])

def rel_angle(q_a, q_b, axis_local) -> float:
    qa = np.asarray(q_a, float)
    rel = _qmul(np.array([-qa[0], -qa[1], -qa[2], qa[3]]), np.asarray(q_b, float))
    return 2.0 * math.atan2(float(np.dot(rel[:3], axis_local)), float(rel[3]))

def tilt_deg(q) -> float:
    return math.degrees(math.acos(float(np.clip(quat_to_mat(q)[2, 2], -1.0, 1.0))))

def seam_axis(k: float) -> np.ndarray:
    return np.array([0.0, float(k), 0.0])

def wing_axis(end_s: float, k: float) -> np.ndarray:
    d = np.array([-k * HW, 0.0, ROOF_H]) / SLOPE_L
    return -d if np.cross(d, np.array([-k * HW, 0.0, 0.0]))[1] * (-end_s) < 0 else d

def wing_strips(k: float, n: int = 7) -> list[tuple[float, float, float, float]]:
    return [(-k * 0.5 * (HW * (0.5 * (i + i + 1) / n) + HW), ROOF_H * (i + 0.5) / n,
             HW * (1 - (i + 0.5) / n), ROOF_H / n) for i in range(n)]


def _author(root: str) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    _P["root"] = root
    for o in OBJECTS: _P[o.name] = f"{root}/{o.name}"

    def look(name, rgb, rough=0.55):
        p = f"{root}/Looks/mk_{name}"
        if not stage.GetPrimAtPath(p).IsValid():
            c = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(rgb), roughness=rough); c.func(p, c)
        return p

    lk = {"white": look("white", WHITE, 0.48), "blue": look("blue", BLUE), "red": look("red", RED),
          "inner": look("inner", INNER, 0.70), "dark": look("dark", DARK, 0.85),
          "mat": look("mat", MAT_COL, 0.75), "mark": look("mark", MARK_COL, 0.4)}
    mat_b, mat_g, mat_s = f"{root}/mk_board", f"{root}/mk_grip", f"{root}/mk_sil"
    for mp_p, sf, df in ((mat_b, 0.55, 0.45), (mat_g, 2.00, 1.65), (mat_s, 0.75, 0.65)):
        c = sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df); c.func(mp_p, c)
    cnt = [0]

    def box(body, pos, size, color, rot=None, col=False, pm=None):
        cnt[0] += 1; p = f"{body}/mk_{cnt[0]}"
        g = UsdGeom.Cube.Define(stage, p); g.CreateSizeAttr(1.0); x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        if rot is not None:
            q = _mat_to_quat(rot); x.AddOrientOp().Set(Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2])))
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, size))); bind_visual_material(p, lk[color], stage=stage)
        if col: UsdPhysics.CollisionAPI.Apply(g.GetPrim()); bind_physics_material(p, pm or mat_b, stage=stage)
        return p

    def hide_proxy(body):
        bp = stage.GetPrimAtPath(body)
        for pr in Usd.PrimRange(bp):
            if pr.HasAPI(UsdPhysics.CollisionAPI): UsdPhysics.CollisionAPI(pr).CreateCollisionEnabledAttr(False)
        for ch in bp.GetChildren():
            if img := UsdGeom.Imageable(ch): img.MakeInvisible()

    rot_y = lambda a: np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    mp = _P[MAT.name]
    for pr in Usd.PrimRange(stage.GetPrimAtPath(mp)):
        if pr.HasAPI(UsdPhysics.CollisionAPI): bind_physics_material(str(pr.GetPath()), mat_s, stage=stage)
    mw, rh = 0.0025, RIM_H - MAT_H
    for s in (-1.0, 1.0):
        box(mp, (s * (HW + mw / 2), 0, MAT_H / 2 + 0.0002), (mw, CW + 2 * mw, 0.0004), "mark")
        box(mp, (0, s * (HW + mw / 2), MAT_H / 2 + 0.0002), (CW + 2 * mw, mw, 0.0004), "mark")
        box(mp, (s * (MAT_W / 2 - RIM_T / 2), 0, MAT_H / 2 + rh / 2), (RIM_T, MAT_W, rh), "mat", col=True, pm=mat_s)
        box(mp, (0, s * (MAT_W / 2 - RIM_T / 2), MAT_H / 2 + rh / 2), (MAT_W - 2 * RIM_T, RIM_T, rh), "mat", col=True, pm=mat_s)

    car, zc = _P[CARTON.name], WALL_H / 2
    for pr in Usd.PrimRange(stage.GetPrimAtPath(car)):
        if pr.HasAPI(UsdPhysics.CollisionAPI): bind_physics_material(str(pr.GetPath()), mat_b, stage=stage)
    for k in (1.0, -1.0):
        box(car, (k * HW / 2, Y_REAR_C, zc + ROOF_H / 2), (SLOPE_L + 0.001, L_REAR, 0.002), "white", rot_y(k * ROOF_ANG), True)
    box(car, (0, Y_REAR_C, zc + ROOF_H + FIN_H / 2 - 0.001), (FIN_T, L_REAR, FIN_H + 0.002), "white", col=True, pm=mat_g)
    box(car, (0, Y_REAR_C, zc + ROOF_H + FIN_H - 0.0015), (FIN_T + 0.0004, L_REAR + 0.0004, 0.003), "blue")
    box(car, (0, Y_FRONT_C, zc - 0.002), (CW - 0.010, L_FRONT - 0.004, 0.006), "dark")
    for side in (-1.0, 1.0):
        for z0, hh in ((-zc + 0.018, 0.032), (zc - 0.012, 0.012)):
            box(car, (side * (HW + 0.0004), 0, z0), (0.0008, CW + 0.0012, hh), "blue")
            box(car, (0, side * (HW + 0.0004), z0), (CW + 0.0012, 0.0008, hh), "blue")
        for du, dv, ww, hh in _glyph_rects("MILK", 0.0035):
            box(car, (side * (HW + 0.0009), side * du, 0.030 + dv), (0.001, ww, hh), "blue")
    UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(car)).CreateCenterOfMassAttr(Gf.Vec3f(0, 0, -0.005))

    for n, k in SEAMS.items():
        sb = _P[n]; hide_proxy(sb); Rr = rot_y(k * ROOF_ANG)
        box(sb, (-k * HW / 2, 0, ROOF_H / 2), (SLOPE_L, L_FRONT, 0.002), "white", Rr, col=True)
        box(sb, (-k * (HW - PLY_T / 2), 0, ROOF_H + FIN_H / 2 - 0.001), (PLY_T, L_FRONT, FIN_H + 0.002), "white", col=True, pm=mat_g)
        box(sb, (-k * (HW - PLY_T / 2), 0, ROOF_H + FIN_H - 0.0015), (PLY_T + 0.0003, L_FRONT, 0.003), "red")
        nrm = Rr @ np.array([0.0, 0.0, 1.0]); nrm = -nrm if nrm[0] * k < 0 else nrm
        up = np.array([-k * HW, 0.0, ROOF_H]) / SLOPE_L; c_loc = np.array([-k * HW / 2, 0.0, ROOF_H / 2]) + nrm * 0.0013
        for word, ucen in (("OPEN", 0.012), ("HERE", -0.004)):
            for du, dv, ww, hh in _glyph_rects(word, 0.0013):
                box(sb, c_loc + up * (ucen + dv) + np.array([0.0, k * du, 0.0]), (hh, ww, 0.0008), "red", Rr)
        box(sb, c_loc + up * (-0.019), (0.004, 0.034, 0.0008), "red", Rr)

    for n, (s, k) in WINGS.items():
        bd = _P[n]; hide_proxy(bd)
        for xc, vz, ww, hh in wing_strips(k): box(bd, (xc, s * 0.0008, vz), (ww, 0.0016, hh + 0.0004), "white", col=True)
        if s == OPEN_S:
            for xc, vz, ww, hh in wing_strips(k, 7)[:5]: box(bd, (-k * (HW - 0.003), s * 0.0018, vz), (0.006, 0.0006, hh + 0.0004), "red")
        else: box(bd, (-k * HW * 0.6, s * 0.0018, 0.006), (HW * 0.7, 0.0006, 0.005), "blue")

    sp = _P[SPOUT.name]; hide_proxy(sp)
    u_r = LIP_LOCAL / float(np.linalg.norm(LIP_LOCAL)); u_b = np.array([0.0, u_r[2], -u_r[1]])
    R_sp = np.column_stack([np.array([1.0, 0.0, 0.0]), u_b, u_r]); nose_h, nose_d, cheek_d = 0.038, 0.012, 0.016
    box(sp, LIP_LOCAL - u_r * (nose_h / 2) + u_b * (nose_d / 2), (NOSE_T, nose_d, nose_h), "white", R_sp, True, mat_g)
    box(sp, LIP_LOCAL - u_r * 0.0014 + u_b * 0.0015, (NOSE_T + 0.0006, 0.003, 0.0028), "red", R_sp)
    plen = math.hypot(BEAK_HALF_W - NOSE_T / 2, cheek_d)
    for k in (-1.0, 1.0):
        ang = math.atan2(k * (BEAK_HALF_W - NOSE_T / 2), cheek_d)
        R_c = R_sp @ np.array([[math.cos(ang), math.sin(ang), 0.0], [-math.sin(ang), math.cos(ang), 0.0], [0, 0, 1]])
        c_pos = LIP_LOCAL - u_r * 0.011 + u_b * (nose_d + cheek_d * 0.48) + np.array([k * (NOSE_T / 2 + BEAK_HALF_W) * 0.5, 0, 0])
        box(sp, c_pos, (0.0018, plen, 0.022), "white", R_c, col=True); box(sp, c_pos + u_r * 0.0096, (0.0024, plen, 0.0028), "red", R_c)
    box(sp, LIP_LOCAL - u_r * 0.012 + u_b * (nose_d + cheek_d * 0.45), (BEAK_HALF_W, cheek_d * 0.72, 0.019), "dark", R_sp)

    for n in DYN_NAMES:
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(_P[n]))
        rb.CreateSolverPositionIterationCountAttr(20); rb.CreateSolverVelocityIterationCountAttr(6)
        if n != CARTON.name: rb.CreateAngularDampingAttr(0.4); rb.CreateLinearDampingAttr(0.4)
        if n in (CARTON.name, *SEAMS): PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(_P[n]))
    UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(sp)).CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    jroot, c0 = f"{root}/mk_joints", np.array(CARTON.pos)
    UsdGeom.Xform.Define(stage, jroot)
    gq = lambda R: Gf.Quatf(float(_mat_to_quat(R)[3]), *map(float, _mat_to_quat(R)[:3]))

    def add_rev(jname, p0_p, p1_p, pos0, q_rot, lim, damp, mf):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{jroot}/{jname}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(p0_p)]); j.CreateBody1Rel().SetTargets([Sdf.Path(p1_p)])
        j.CreateAxisAttr("X"); j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0))); j.CreateLocalRot0Attr(q_rot)
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0)); j.CreateLocalRot1Attr(q_rot)
        j.CreateLowerLimitAttr(0.0); j.CreateUpperLimitAttr(float(lim)); j.CreateExcludeFromArticulationAttr(True)
        dr = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        for fn, v in ((dr.CreateTypeAttr, "force"), (dr.CreateStiffnessAttr, 0.0), (dr.CreateDampingAttr, damp),
                      (dr.CreateTargetVelocityAttr, 0.0), (dr.CreateMaxForceAttr, float(mf))): fn(v)

    for n, k in SEAMS.items():
        d, y_ax = seam_axis(k), np.array([0.0, 0.0, 1.0])
        R = np.column_stack([d, y_ax, np.cross(d, y_ax)])
        add_rev(n, car, _P[n], np.array(_seam_origin(k)) - c0, gq(R), SEAM_LIM_DEG, 0.5, SEAM_FOLD_TQ)
    for n, (s, k) in WINGS.items():
        d = wing_axis(s, k)
        y_ax = np.cross(np.array([0.0, 1.0, 0.0]) if abs(d[1]) < 0.9 else np.array([1.0, 0.0, 0.0]), d)
        y_ax /= np.linalg.norm(y_ax)
        R = np.column_stack([d, y_ax, np.cross(d, y_ax)])
        par, p_orig = (_P["seam_px" if k > 0 else "seam_nx"], np.array(_seam_origin(k))) if s == OPEN_S else (car, c0)
        add_rev(n, par, _P[n], np.array(_wing_origin(s, k)) - p_orig, gq(R), WING_LIM_DEG, 0.5, FOLD_TQ)
    add_rev("spout_hinge", car, sp, SPOUT0 - c0, Gf.Quatf(1, 0, 0, 0), SPOUT_LIM_DEG, 0.8, SPOUT_TQ)

    flt = [(car, _P[n]) for n in DYN_NAMES[1:]] + [(mp, _P[n]) for n in DYN_NAMES[1:]]
    sn, wn = list(SEAMS), list(WINGS)
    flt += [(_P[sn[0]], _P[sn[1]]), (_P[wn[0]], _P[wn[1]]), (_P[wn[2]], _P[wn[3]]), (_P[wn[0]], sp), (_P[wn[1]], sp)]
    for s_n in sn:
        flt += [(_P[s_n], sp), *((_P[s_n], _P[w_n]) for w_n in wn)]
    for a, b in flt:
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a)).CreateFilteredPairsRel().AddTarget(Sdf.Path(b))
    print(f"[milk] authored under {root}", flush=True)


def _wrap_cfg(spec, obj):
    from isaaclab.sim.utils import clone
    cfg = obj.spawner_cfg
    base = getattr(cfg.func, "__wrapped__", cfg.func)
    cfg.func = clone(lambda p, c, *a, **k: (base(p, c, *a, **k), _author(str(p).rsplit("/", 1)[0]))[0])
    obj.object_cfg = obj._init_object_cfg()
    return obj

def _patch_scene(sc) -> None:
    if getattr(sc.task_object, "_milk_patched", False): return
    orig = sc.task_object
    def task_object(spec):
        obj = orig(spec)
        return _wrap_cfg(spec, obj) if getattr(spec, "name", None) == OBJECTS[-1].name else obj
    task_object._milk_patched = True
    sc.task_object = task_object

def _enable_contacts() -> None:
    for fn in (lambda: __import__("carb").settings.get_settings().set_bool("/physics/disableContactProcessing", False),
               lambda: __import__("isaaclab.app.settings_manager", fromlist=["x"]).get_settings_manager().set_bool(
                   "/physics/disableContactProcessing", False)):
        try: fn()
        except Exception: pass

class _SceneFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname not in ("yam_arena.scene", "isaaclab_physx.physics.physx_manager"): return None
        sys.meta_path.remove(self)
        try: spec = importlib.util.find_spec(fullname)
        finally: sys.meta_path.insert(0, self)
        if spec and spec.loader:
            oe = spec.loader.exec_module
            if fullname == "yam_arena.scene": spec.loader.exec_module = lambda m: (oe(m), _patch_scene(m))
            else:
                def _pm(m):
                    oe(m); mgr = m.PhysxManager
                    if not getattr(mgr._configure_physics, "_mk_c", False):
                        b = mgr._configure_physics.__func__
                        cp = lambda cls: (b(cls), _enable_contacts())
                        cp._mk_c = True; mgr._configure_physics = classmethod(cp)
                spec.loader.exec_module = _pm
        return spec

if "yam_arena.scene" in sys.modules: _patch_scene(sys.modules["yam_arena.scene"])
if "isaaclab_physx.physics.physx_manager" in sys.modules: _enable_contacts()
if not any(isinstance(f, _SceneFinder) for f in sys.meta_path): sys.meta_path.insert(0, _SceneFinder())

_FRESH = dict(env=None, step=0, nan=False, broken={}, wing={}, seam_broken={}, seam={}, seam_peak={}, throat=0.0,
              spout_deg=0.0, lip=-0.016, unlocked=False, wrong=False, unheld=False, start_xy=None, max_tilt=0.0,
              max_shift=0.0, prev_grip=None, grip_speed=99.0, hold=0, err=None)
_M: dict = {}


def _np(t) -> np.ndarray:
    x = getattr(t, "torch", t)
    if hasattr(x, "detach"): return np.asarray(x.detach().cpu().numpy(), dtype=np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)

def _set_wrench(env, name: str, torque_w: np.ndarray) -> None:
    import torch
    base = env.unwrapped; obj, dev = base.scene[name], base.device
    obj.permanent_wrench_composer.reset()
    obj.permanent_wrench_composer.add_forces_and_torques(
        forces=torch.zeros((1, 1, 3), dtype=torch.float32, device=dev),
        torques=torch.tensor(torque_w, dtype=torch.float32, device=dev).view(1, 1, 3), is_global=True)


def _apply_internal_torques(env, Rc: np.ndarray, poses: dict | None = None) -> None:
    tmap = {n: np.zeros(3, dtype=np.float64) for n in DYN_NAMES}
    s_wedge = min(16.0, 16.0 * max(0.0, _M["spout_deg"] - 15.0) / 45.0)
    for n, k in SEAMS.items():
        s_ax, th = Rc @ seam_axis(k), _M["seam"].get(n, 0.0)
        s_rest = max(min(16.0, 0.72 * _M["seam_peak"].get(n, 0.0)), s_wedge)
        mag = -SEAM_SEAL_TQ * min(1.0, max(0.20, th / 1.5)) if not _M["seam_broken"].get(n) else (
            0.055 * min(1.0, (s_rest - th) / 3.0) if th < s_rest else 0.0)
        tw = mag * s_ax; tmap[n] += tw; tmap[CARTON.name] -= tw
    w_rest = max(24.0, min(WEDGE_MAX_DEG, WEDGE_MAX_DEG * max(0.0, _M["spout_deg"] - 12.0) / 35.0))
    for n, (s, k) in WINGS.items():
        p_n = ("seam_px" if k > 0 else "seam_nx") if s == OPEN_S else CARTON.name
        Rp = quat_to_mat(poses[p_n][1]) if (poses is not None and p_n in poses) else Rc
        w_ax, th = Rp @ wing_axis(s, k), _M["wing"].get(n, 0.0)
        mag = -(SEAL_TQ_OPEN if s == OPEN_S else SEAL_TQ_FAR) * min(1.0, max(0.15, th / 1.5)) if not _M["broken"].get(n) else (
            0.045 * min(1.0, (w_rest - th) / 4.0) if (s == OPEN_S and th < w_rest) else 0.0)
        tw = mag * w_ax; tmap[n] += tw; tmap[p_n] -= tw
    ts = -min(1.5, 0.25 + 0.25 * max(0.0, _M["spout_deg"])) * (Rc @ np.array([1.0, 0.0, 0.0])) if not _M["unlocked"] else np.zeros(3)
    tmap[SPOUT.name] += ts; tmap[CARTON.name] -= ts
    for n, tw in tmap.items(): _set_wrench(env, n, tw)


def _reset_model() -> None:
    env = _M.get("env")
    _M.clear()
    _M.update({k: (v.copy() if isinstance(v, (dict, list)) else v) for k, v in _FRESH.items()})
    _M["env"] = env
    _M["lip"] = -float(SPOUT_INSET + LIP_LOCAL[1])


_reset_model()


def _origin() -> np.ndarray:
    try: return np.zeros(3) if _M.get("env") is None else _np(_M["env"].unwrapped.scene.env_origins[0]).reshape(-1)
    except Exception: return np.zeros(3)

def body(objects, name: str) -> tuple[np.ndarray, np.ndarray]:
    if _M.get("env") is not None:
        p = _np(_M["env"].unwrapped.arena_world.get_pose_w(name)[0]).reshape(-1)
        return p[:3] - _origin(), p[3:7].copy()
    o = objects[name]; return np.asarray(o.pos, float), np.asarray(o.quat_xyzw, float)

def gripper_state(side: str):
    from yam_arena.runtime import pad_tips_world
    rob = _M["env"].unwrapped.scene[f"{side}_robot"]; names, q = list(rob.joint_names), _np(rob.data.joint_pos[0]).reshape(-1)
    return np.asarray(pad_tips_world(rob), dtype=np.float64) - _origin(), abs(float(q[names.index("joint7")])) + abs(float(q[names.index("joint8")]))

def gripper_points() -> np.ndarray | None:
    if _M.get("env") is None: return _M.get("mock_grip_pts")
    try:
        from yam_arena.runtime import link_poses, pad_tips_world, tcp_world
        base, o, pts = _M["env"].unwrapped, _origin(), []
        for s in ("left_robot", "right_robot"):
            rob = base.scene[s]; lp, q, names = link_poses(rob), _np(rob.data.joint_pos[0]).reshape(-1), list(rob.joint_names)
            ap = abs(float(q[names.index("joint7")])) + abs(float(q[names.index("joint8")]))
            mid, half = np.asarray(pad_tips_world(rob), dtype=np.float64) - o, 0.5 * ap * lp["gripper"][:3, 0]
            pts.extend([np.asarray(lp["gripper"][:3, 3], dtype=np.float64) - o, np.asarray(tcp_world(rob), dtype=np.float64) - o, mid + half, mid - half])
        arr = np.asarray(pts, dtype=np.float64)
        return arr if arr.shape == (8, 3) and np.isfinite(arr).all() else None
    except Exception: return None

def seam_rim_point(p, q, k: float) -> np.ndarray:
    return np.asarray(p, float) + quat_to_mat(q) @ np.array([-k * HW, OPEN_S * (L_FRONT / 2), ROOF_H + FIN_H / 2])

def throat_gap_from_poses(cq, p_px, q_px, p_nx, q_nx) -> float:
    return max(0.0, float(np.dot(seam_rim_point(p_px, q_px, 1.0) - seam_rim_point(p_nx, q_nx, -1.0), quat_to_mat(cq) @ [1, 0, 0])))

def lip_point(sp, sq) -> np.ndarray: return np.asarray(sp, float) + quat_to_mat(sq) @ LIP_LOCAL

def nose_pinch_point(sp, sq) -> np.ndarray:
    u = LIP_LOCAL / float(np.linalg.norm(LIP_LOCAL))
    return np.asarray(sp, float) + quat_to_mat(sq) @ (LIP_LOCAL - u * 0.011 + np.array([0.0, u[2], -u[1]]) * 0.005)

def lip_past_face(cp, cq, sp, sq) -> float:
    n = quat_to_mat(cq) @ np.array([0.0, OPEN_S, 0.0])
    return float(np.dot(lip_point(sp, sq) - (np.asarray(cp, float) + n * HW), n))

def _seg_dist(p, a, b) -> float:
    ab = b - a
    return float(np.linalg.norm(p - (a + float(np.clip(np.dot(p - a, ab) / max(float(np.dot(ab, ab)), 1e-12), 0.0, 1.0)) * ab)))

def carton_clearance(p, cp, cq, sp, sq, seam_poses=None) -> float:
    pt, loc = np.asarray(p, float), quat_to_mat(cq).T @ (np.asarray(p, float) - cp)
    lo, hi = np.array([-HW, -HW, -WALL_H / 2]), np.array([HW, HW, WALL_H / 2 + ROOF_H + FIN_H])
    dm = min(float(np.linalg.norm(np.maximum(np.maximum(lo - loc, loc - hi), 0.0))),
             max(0.0, _seg_dist(pt, np.asarray(sp, float), lip_point(sp, sq)) - BEAK_HALF_W))
    for k, (s_p, s_q) in seam_poses or ():
        sl = quat_to_mat(s_q).T @ (pt - np.asarray(s_p, float))
        slo, shi = np.array([min(0.0, -k * HW), -L_FRONT / 2, 0.0]), np.array([max(0.0, -k * HW), L_FRONT / 2, ROOF_H + FIN_H])
        dm = min(dm, float(np.linalg.norm(np.maximum(np.maximum(slo - sl, sl - shi), 0.0))))
    return dm


_CV: dict = {}

def steadying_contact_n(side: str) -> float:
    if _M.get("env") is None: return float((_M.get("mock_steady_n") or {}).get(side, 0.0))
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab_physx.physics import PhysxManager
    from yam_arena.constants import PHYSICS_DT
    sv, st = PhysxManager.get_physics_sim_view(), get_current_stage()
    tgts = [_P[n] for n in (CARTON.name, *SEAMS) if n in _P]
    if sv is None or not tgts: return 0.0
    key = (id(sv), side, tuple(tgts))
    if key not in _CV:
        _enable_contacts(); arm = f"/{side.capitalize()}Arm/"
        lks = [s for p in st.Traverse() if "/env_0/" in (s := p.GetPath().pathString) and arm in s
               and s.endswith(("/gripper", "/tip_left", "/tip_right")) and "PhysxContactReportAPI" in p.GetAppliedSchemas()]
        if not lks: return 0.0
        _CV[key] = sv.create_rigid_contact_view(lks, filter_patterns=[tgts] * len(lks), max_contact_data_count=256)
    return float(np.linalg.norm(_np(_CV[key].get_contact_force_matrix(dt=PHYSICS_DT)).reshape(-1, len(tgts), 3), axis=-1).sum())

def _steady_ok(g: np.ndarray | None, cp: np.ndarray, Rc: np.ndarray) -> tuple[bool, float]:
    if g is None: return False, 0.0
    loc = (g - cp) @ Rc
    st_side = "left" if float(np.min(loc[4:, 1])) <= float(np.min(loc[:4, 1])) else "right"
    fn = steadying_contact_n(st_side)
    return fn >= STEADY_MIN_N, fn

def _step_sim(env) -> None:
    _M["env"] = env
    _M["step"] += 1
    poses = {n: body(None, n) for n in DYN_NAMES}
    if any(not (np.isfinite(p).all() and np.isfinite(q).all()) for p, q in poses.values()):
        _M["nan"] = True
        return
    cp, cq = poses[CARTON.name]
    sp, sq = poses[SPOUT.name]
    Rc = quat_to_mat(cq)
    g = gripper_points()
    held_ok, held_n = _steady_ok(g, cp, Rc)

    for n, (s, k) in WINGS.items():
        q_par = poses["seam_px" if k > 0 else "seam_nx"][1] if s == OPEN_S else cq
        th = math.degrees(rel_angle(q_par, poses[n][1], wing_axis(s, k))); _M["wing"][n] = th
        if s == OPEN_S and th >= SEAL_BREAK_DEG and not _M["broken"].get(n):
            _M["broken"][n] = True
            if not held_ok: _M["unheld"] = True
            print(f"[milk] wing {n} step {_M['step']} ({th:.1f} deg, F={held_n:.1f}N)", flush=True)
        if s != OPEN_S and th > WRONG_END_DEG and not _M["wrong"]: _M["wrong"] = True
    wings_ok = all(_M["broken"].get(n) for n, (s, _) in WINGS.items() if s == OPEN_S)
    for n, k in SEAMS.items():
        th = math.degrees(rel_angle(cq, poses[n][1], seam_axis(k))); _M["seam"][n] = th
        if wings_ok: _M["seam_peak"][n] = max(_M["seam_peak"].get(n, 0.0), th)
        if wings_ok and th >= SEAM_BREAK_DEG and not _M["seam_broken"].get(n):
            _M["seam_broken"][n] = True
            if not held_ok: _M["unheld"] = True
            print(f"[milk] seam {n} step {_M['step']} ({th:.1f} deg, F={held_n:.1f}N)", flush=True)
    _M["throat"] = throat_gap_from_poses(cq, *poses["seam_px"], *poses["seam_nx"])
    if not _M["unlocked"] and wings_ok and all(_M["seam_broken"].get(n) for n in SEAMS): _M["unlocked"] = True

    _M["spout_deg"] = math.degrees(rel_angle(cq, sq, np.array([1.0, 0.0, 0.0])))
    _M["lip"] = lip_past_face(cp, cq, sp, sq)
    _apply_internal_torques(env, Rc, poses)

    base_pt = cp - Rc @ np.array([0.0, 0.0, WALL_H / 2])
    if _M["start_xy"] is None: _M["start_xy"] = base_pt[:2].copy()
    _M["max_tilt"] = max(_M["max_tilt"], tilt_deg(cq))
    _M["max_shift"] = max(_M["max_shift"], float(np.linalg.norm(base_pt[:2] - _M["start_xy"])))
    if g is not None:
        _M["grip_speed"] = 0.0 if _M["prev_grip"] is None else float(np.max(np.linalg.norm(g - _M["prev_grip"], axis=1))) * 20.0
        _M["prev_grip"] = g.copy()
    else: _M["grip_speed"] = 99.0
    _M["hold"] = _M["hold"] + 1 if end_ready(None) else 0

def gate() -> str:
    if _M["nan"]: return "nan"
    if _M["wrong"]: return "far (+y) wing pushed back > 15 deg"
    if _M["unheld"]: return "unsealing without second-gripper steadying contact (>= 0.5 N)"
    if _M["max_tilt"] > TILT_MAX: return "carton tipped > 20 deg"
    if _M["max_shift"] > OFF_MAT_MAX: return "carton base shifted > 8.0 cm"
    return ""

def min_clearance(objects) -> float:
    g = gripper_points()
    if g is None: return -1.0
    cp, cq = body(objects, CARTON.name)
    sp, sq = body(objects, SPOUT.name)
    sposes = [(k, body(objects, n)) for n, k in SEAMS.items()] if (
        _M.get("env") is not None or (objects is not None and "seam_px" in objects)) else []
    return min(carton_clearance(p, cp, cq, sp, sq, sposes) for p in g)

def wings_unsealed() -> bool:
    return all(_M["broken"].get(n) and _M["wing"].get(n, 0.0) >= WING_OPEN_END_DEG for n, (s, _) in WINGS.items() if s == OPEN_S)

def current_throat(objects) -> float:
    if _M.get("env") is not None: return float(_M["throat"])
    return throat_gap_from_poses(body(objects, CARTON.name)[1], *body(objects, "seam_px"), *body(objects, "seam_nx"))

def seam_separated(objects) -> bool:
    return bool(wings_unsealed() and all(_M["seam_broken"].get(n) and _M["seam"].get(n, 0.0) >= SEAM_END_DEG for n in SEAMS)
                and current_throat(objects) >= THROAT_MIN)

def released(objects) -> bool:
    return bool(min_clearance(objects) >= RELEASE_CLEAR)

def current_lip(objects) -> float:
    return float(_M["lip"]) if _M.get("env") is not None else lip_past_face(*body(objects, CARTON.name), *body(objects, SPOUT.name))

def end_ready(objects) -> bool:
    return bool(not gate() and seam_separated(objects) and current_lip(objects) >= LIP_FULL
                and released(objects) and min_clearance(objects) >= GRIP_CLEAR and _M["grip_speed"] <= GRIP_SPEED)

def evaluate(objects) -> dict:
    g, lip, thr, rel = gate(), current_lip(objects), current_throat(objects), released(objects)
    w_ok, s_ok = wings_unsealed(), seam_separated(objects)
    s = 0.0 if (g or not w_ok or not rel) else (0.2 if not s_ok else (1.0 if (lip >= LIP_FULL and _M["hold"] >= HOLD_STEPS) else (0.7 if lip >= LIP_HALF else 0.4)))
    return dict(score=s, gate=g or ("not released" if not rel else "ok"), wings=w_ok, seam=s_ok,
                throat_mm=round(thr * 1000, 1), lip_mm=round(lip * 1000, 1), spout_deg=round(_M["spout_deg"], 1), hold=_M["hold"])

def score(objects) -> float:
    try: return float(evaluate(objects)["score"])
    except Exception as e:
        if _M["err"] is None: _M["err"] = repr(e); print(f"[milk] score error: {e!r}", flush=True)
        return 0.0

def success(objects) -> bool: return score(objects) >= 1.0
def distance(objects) -> float: return 1.0 - score(objects)


def progress(objects) -> dict:
    try: e = evaluate(objects)
    except Exception: return {}
    return {"wings": float(e["wings"]), "seam": float(e["seam"]), "throat_mm": e["throat_mm"], "lip_mm": e["lip_mm"],
            "spout_deg": e["spout_deg"], "seam_deg": {k[5:]: round(v, 1) for k, v in _M["seam"].items()},
            "wings_deg": {k[5:]: round(v, 1) for k, v in _M["wing"].items()}, "tilt_max": round(_M["max_tilt"], 1),
            "shift_max_mm": round(_M["max_shift"] * 1000, 1), "hold": _M["hold"], "gate": e["gate"]}

def _install_hooks() -> None:
    try: from yam_arena import inspect_embodiment as ie
    except Exception: return
    cls = ie.YamArenaEmbodiment
    is_ours = lambda s: getattr(getattr(getattr(s, "task", None), "module", None), "INSTRUCTION", None) == INSTRUCTION
    if not getattr(cls.reset, "_milk_hooked", False):
        orst = cls.reset
        def reset(self, *a, **k):
            _reset_model()
            obs = orst(self, *a, **k)
            if is_ours(self):
                _M["env"] = self._env
                poses = {n: body(None, n) for n in DYN_NAMES}
                _apply_internal_torques(self._env, quat_to_mat(poses[CARTON.name][1]), poses)
            return obs
        reset._milk_hooked = True; cls.reset = reset
    if not getattr(cls._task_step, "_milk_hooked", False):
        ostp = cls._task_step
        def _task_step(self, *a, **k):
            if is_ours(self):
                try: _step_sim(self._env)
                except Exception as exc:
                    if not _M.get("hook_err"): _M["hook_err"] = True; print(f"[milk] step err: {exc!r}", flush=True)
            return ostp(self, *a, **k)
        _task_step._milk_hooked = True; cls._task_step = _task_step


_install_hooks()
VARIATION = {"obj:milk_carton": {"dx": 0.01, "dy": 0.01, "yaw": 0.05}}
