"""Figure-of-eight elastic bandage on a sprained wrist (station table).

A rigid adult right forearm-and-hand form is clamped at its elbow end on a stand so the forearm, wrist and hand stick
straight out along +x over the table, palm down, axis 16.0 cm up, clear underneath. A coreless 7.5 cm x 1.66 m elastic
crepe bandage (marked with blue reset ticks every 20.0 cm relaxed, inner 4.0 cm hook-and-loop closure patch)
starts rolled at (24.0, +13.0) cm, axis along +x, with 20.0 cm of its outer end laid flat to (24.0, -8.2) cm.
"""
from __future__ import annotations

import math
import os
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

import bandage as bd  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
RUN_LIMIT = 2400
FULL_BANDAGE_M = 1.66
THUMB_BARE_FRAC = 0.40
PRESSURE_MIN_MMHG, PRESSURE_MAX_MMHG = bd.PRESSURE_MIN_MMHG, bd.PRESSURE_MAX_MMHG
WRIST_X_MAX, HAND_X_MIN, DIAG_SHIFT, UNDER_RAD = bd.X_W, bd.X_H, bd.DIAG_DX, bd.UNDER_DPHI
THUMB_CLEAR_M, TIP_CLEAR_M, TIP_BARE_LEN, TIP_X_MIN, FORM_TIP_X = 0.005, 0.010, 0.030, bd.TIP_X - 0.03, bd.TIP_X

INSTRUCTION = "Bandage the sprained wrist with the crepe bandage in figure-of-eight turns, fasten its end, then let go."

STEEL, DARK, RULE_WHITE = (0.55, 0.57, 0.60), (0.20, 0.21, 0.23), (0.94, 0.94, 0.95)
SKIN, CUFF_SLATE = (0.68, 0.48, 0.36), (0.26, 0.31, 0.37)
CREPE, SELVEDGE, PATCH = (0.87, 0.79, 0.64), (0.16, 0.30, 0.54), (0.20, 0.24, 0.36)

# elbow clamp stand + fixed 20.0 cm white calibration scale bar across y = -10..+10 cm at x = 14.5 cm
OBJECTS = [
    Box("stand_base", pos=(0.09, 0.0, TOP + 0.005), size=(0.14, 0.20, 0.01), color=DARK),
    Box("scale_bar", pos=(0.145, 0.0, TOP + 0.011), size=(0.018, 0.20, 0.002), color=RULE_WHITE),
    Box("stand_post", pos=(0.09, 0.0, TOP + 0.01 + 0.054), size=(0.03, 0.03, 0.108), color=STEEL),
    Box("stand_cradle", pos=(0.09, 0.0, TOP + 0.122), size=(0.07, 0.08, 0.012), color=STEEL),
    Box("stand_strap", pos=(0.09, 0.0, TOP + 0.196), size=(0.03, 0.072, 0.008), color=DARK),
]

TWIN_EXTRA = [
    {"name": "arm_form", "kind": "fixed", "real": "rigid adult right forearm-and-hand form, palm down, thumb abducted",
     "size_mm": [420, 85, 66], "colour": "skin tone", "pose": [0.26, 0.0, 0.16]},
    {"name": "bandage", "kind": "body", "real": "7.5 cm x 1.66 m coreless elastic crepe bandage with 4 cm hook patch",
     "size_mm": [75, int(round(bd.ROLL_D * 1000)), int(round(bd.ROLL_D * 1000))], "colour": "cream",
     "pose": [float(bd.ROLL0[0]), float(bd.ROLL0[1]), float(bd.ROLL_R)]},
]

_ENV: dict = {}


def _robots(env):
    return [env.unwrapped.scene.articulations[f"{s}_robot"] for s in ("left", "right")]


def _arm_state(env):
    from yam_arena.runtime import link_poses, tcp_world

    tcp, rot, ap = [], [], []
    for r in _robots(env):
        lp, q, names = link_poses(r), r.data.joint_pos.torch[0].cpu().numpy(), list(r.joint_names)
        tcp.append(tcp_world(r))
        rot.append(lp["gripper"][:3, :3])
        ap.append(abs(float(q[names.index("joint7")])) + abs(float(q[names.index("joint8")])))
    return np.array(tcp), np.array(rot), np.array(ap)


def _gripper_points(env):
    from yam_arena.runtime import link_poses

    out = []
    for r in _robots(env):
        g = link_poses(r)["gripper"]
        R, p = g[:3, :3], g[:3, 3]
        for loc in ((0, 0, 0.147), (0.03, 0, 0.13), (-0.03, 0, 0.13), (0, 0, 0.06), (0.05, 0, 0.03), (-0.05, 0, 0.03)):
            out.append(R @ np.array(loc) + p)
    return np.array(out)


def _apply_bandage_wrench(env, b: bd.Bandage):
    import torch

    f_vec, pos_vec = b.gripper_load()
    for i, r in enumerate(_robots(env)):
        bids, _ = r.find_bodies("gripper")
        f_t = torch.tensor(f_vec[i], dtype=torch.float32, device=r.device).view(1, 1, 3)
        tau = (np.cross(pos_vec[i] - r.data.body_com_pos_w.torch[0, bids[0]].cpu().numpy(), f_vec[i])
               if np.linalg.norm(f_vec[i]) > 1e-6 else np.zeros(3))
        tau_t = torch.tensor(tau, dtype=torch.float32, device=r.device).view(1, 1, 3)
        r.permanent_wrench_composer.set_forces_and_torques_index(forces=f_t, torques=tau_t, body_ids=bids, is_global=True)


def _render_grid(b: bd.Bandage):
    n, x = b.paid, b.x[:b.paid]
    g = np.stack([x[:, 0], 0.92 * x[:, 0] + 0.08 * x[:, 1], 0.5 * (x[:, 0] + x[:, 1]), x[:, 1],
                  0.5 * (x[:, 1] + x[:, 2]), 0.08 * x[:, 1] + 0.92 * x[:, 2], x[:, 2]], axis=1)
    flat = g.reshape(-1, 3)
    d, nrm = bd.arm_sdf(flat)
    loff = b._layer_off(n)
    loff7 = np.stack([loff[:, 0], 0.92 * loff[:, 0] + 0.08 * loff[:, 1], 0.5 * (loff[:, 0] + loff[:, 1]), loff[:, 1],
                      0.5 * (loff[:, 1] + loff[:, 2]), 0.08 * loff[:, 1] + 0.92 * loff[:, 2], loff[:, 2]], axis=1).reshape(-1)
    th = bd.THICK * 1.3 + loff7
    pen = d < th
    if pen.any():
        flat[pen] += (th[pen] - d[pen])[:, None] * nrm[pen]
    return flat.astype(np.float32), nrm.astype(np.float32)


def _quat_z_to(v):
    v = np.asarray(v, float) / np.linalg.norm(v)
    ax = np.cross([0.0, 0.0, 1.0], v)
    s, c = np.linalg.norm(ax), float(v[2])
    if s < 1e-9:
        return (1.0, 0.0, 0.0, 0.0) if c > 0 else (0.0, 1.0, 0.0, 0.0)
    ang = math.atan2(s, c)
    return (math.cos(ang / 2), *map(float, (ax / s) * math.sin(ang / 2)))


def _quat_from_R(R):
    from scipy.spatial.transform import Rotation

    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return (float(w), float(x), float(y), float(z))


class Tracker:
    def __init__(self):
        self.b = self.meshes = None
        self.tcp, self.ap = np.zeros((2, 3)) + [0, 0, 5], np.full(2, 0.09)
        self.gp = np.zeros((12, 3)) + [0, 0, 5]
        self.speed, self.steps, self.hold_steps = 1.0, 0, 0

    def reset(self):
        self.b = bd.Bandage()
        self.steps = self.hold_steps = 0
        self.tcp, self.ap = np.zeros((2, 3)) + [0, 0, 5], np.full(2, 0.09)
        self.gp, self.speed = np.zeros((12, 3)) + [0, 0, 5], 1.0
        if _ENV.get("env") is not None:
            _apply_bandage_wrench(_ENV["env"], self.b)

    def update_hold(self):
        b = self.b
        if b is None:
            self.hold_steps = 0
            return
        d_form = float(bd.arm_sdf(self.gp)[0].min())
        d_strip = float(np.min(np.linalg.norm(b.x[:b.paid].reshape(-1, 3)[None, :, :] - self.gp[:, None, :], axis=2)))
        pw, ph = bd.cuff_pressures_mmhg(b)
        p_ok = (PRESSURE_MIN_MMHG <= pw <= PRESSURE_MAX_MMHG) and (PRESSURE_MIN_MMHG <= ph <= PRESSURE_MAX_MMHG)
        if b.fastened and p_ok and not b.held_by() and (self.ap > JAW_OPEN).all() and min(d_form, d_strip) >= CLEAR and self.speed < STILL:
            self.hold_steps += 1
        else:
            self.hold_steps = 0

    def step(self, env):
        if self.b is None:
            self.reset()
        tcp, rot, ap = _arm_state(env)
        if self.steps:
            self.speed = float(np.linalg.norm(tcp - self.tcp, axis=1).max() / 0.05)
        self.tcp, self.ap, self.gp = tcp, ap, _gripper_points(env)
        self.b.step(tcp, rot, ap)
        _apply_bandage_wrench(env, self.b)
        self.steps += 1
        self.update_hold()
        self.render()
        if os.environ.get("BD_LOG") and self.steps % 20 == 0:
            b = self.b
            print(f"[bd] step {self.steps} paid {b.paid} laid {int(b.laid.sum())} holders {[h[0] for h in b.roll_holders]} "
                  f"pins {list(b.pins)} passes {b.passes}/{b.under_passes} ap {ap.round(3)} parts {parts()}", flush=True)

    def _material(self, stage, path, col, rough=0.8):
        from pxr import Gf, Sdf, UsdShade

        mat = UsdShade.Material.Define(stage, path)
        sh = UsdShade.Shader.Define(stage, path + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*col))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return mat

    def _build(self, stage):
        from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

        self.meshes = {}
        skin = self._material(stage, "/World/BdFormMat", SKIN, 0.6)
        cuff_mat = self._material(stage, "/World/BdCuffMat", CUFF_SLATE, 0.72)
        for k, part in enumerate(bd.PARTS):
            if part[0] == "cap":
                _, a, b, r = part
                g = UsdGeom.Capsule.Define(stage, f"/World/BdForm/c{k}")
                g.CreateAxisAttr("Z")
                g.CreateRadiusAttr(float(r))
                g.CreateHeightAttr(float(np.linalg.norm(b - a)))
                g.AddTranslateOp().Set(Gf.Vec3d(*map(float, (a + b) / 2)))
                g.AddOrientOp().Set(Gf.Quatf(*_quat_z_to(b - a)))
            else:
                _, c, half, _ = part
                g = UsdGeom.Cube.Define(stage, f"/World/BdForm/b{k}")
                g.CreateSizeAttr(1.0)
                g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
                g.AddScaleOp().Set(Gf.Vec3f(*map(float, 2 * half)))
            UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(skin)
            try:
                UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            except Exception:  # noqa: BLE001
                pass
        cw = UsdGeom.Cylinder.Define(stage, "/World/BdForm/cuff_wrist")
        cw.CreateAxisAttr("X")
        cw.CreateRadiusAttr(0.0308)
        cw.CreateHeightAttr(float(bd.CUFF_W_X[1] - bd.CUFF_W_X[0]))
        cw.AddTranslateOp().Set(Gf.Vec3d(0.5 * (bd.CUFF_W_X[0] + bd.CUFF_W_X[1]), 0.0, float(bd.AX_Z)))
        UsdShade.MaterialBindingAPI.Apply(cw.GetPrim()).Bind(cuff_mat)
        ch = UsdGeom.Cube.Define(stage, "/World/BdForm/cuff_palm")
        ch.CreateSizeAttr(1.0)
        ch.AddTranslateOp().Set(Gf.Vec3d(0.5 * (bd.CUFF_H_X[0] + bd.CUFF_H_X[1]), 0.0, float(bd.PALM[0][2])))
        ch.AddScaleOp().Set(Gf.Vec3f(float(bd.CUFF_H_X[1] - bd.CUFF_H_X[0]), float(2 * bd.PALM[1][1] + 0.0012), float(2 * bd.PALM[1][2] + 0.0012)))
        UsdShade.MaterialBindingAPI.Apply(ch.GetPrim()).Bind(cuff_mat)
        for name, col in (("Strip", CREPE), ("Edge", SELVEDGE), ("Patch", PATCH)):
            m = UsdGeom.Mesh.Define(stage, f"/World/Bd{name}")
            m.CreateDoubleSidedAttr(True)
            m.CreateSubdivisionSchemeAttr("none")
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(self._material(stage, f"/World/Bd{name}Mat", col, 0.92))
            self.meshes[name] = m
        cyl = UsdGeom.Cylinder.Define(stage, "/World/BdRoll")
        cyl.CreateAxisAttr("X")
        cyl.CreateRadiusAttr(1.0)
        cyl.CreateHeightAttr(float(bd.WIDTH))
        self._roll_xf = (cyl.AddTranslateOp(), cyl.AddOrientOp(), cyl.AddScaleOp())
        UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(self._material(stage, "/World/BdRollMat", CREPE, 0.92))
        self.meshes["roll"] = cyl

    def render(self):
        from pxr import Gf, UsdGeom, Vt
        import omni.usd

        if self.meshes is None or not self.meshes["Strip"].GetPrim().IsValid():
            self._build(omni.usd.get_context().get_stage())
        b = self.b
        n, (pts, nrm) = b.paid, _render_grid(b)
        pts_list, f_body, f_edge, f_patch = [pts], [], [], []
        patch_start = bd.NS - bd.PATCH_N
        for i in range(max(0, n - 1)):
            for c in range(6):
                quad = [7 * i + c, 7 * (i + 1) + c, 7 * (i + 1) + c + 1, 7 * i + c + 1]
                (f_patch if i >= patch_start else (f_edge if c in (0, 5) else f_body)).append(quad)
            if i % 10 == 0 and i <= 80 and i < patch_start:
                w_frac = 0.20
                base_idx = sum(len(arr) for arr in pts_list)
                lift = 0.00035 * nrm[7 * i + 1:7 * i + 6]
                p_a = pts[7 * i + 1:7 * i + 6] + lift
                p_b = (1.0 - w_frac) * pts[7 * i + 1:7 * i + 6] + w_frac * pts[7 * (i + 1) + 1:7 * (i + 1) + 6] + lift
                pts_list.append(np.concatenate([p_a, p_b], axis=0).astype(np.float32))
                for c in range(4):
                    f_edge.append([base_idx + c, base_idx + 5 + c, base_idx + 5 + c + 1, base_idx + c + 1])
        all_pts = np.concatenate(pts_list, axis=0) if len(pts_list) > 1 else pts
        for name, faces in (("Strip", f_body), ("Edge", f_edge), ("Patch", f_patch)):
            m, img = self.meshes[name], UsdGeom.Imageable(self.meshes[name])
            if faces:
                img.MakeVisible()
                m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(all_pts))
                m.GetFaceVertexCountsAttr().Set([4] * len(faces))
                m.GetFaceVertexIndicesAttr().Set(np.array(faces, np.int32).ravel().tolist())
            else:
                img.MakeInvisible()
                rc = b.roll_c.astype(np.float32)
                m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(np.array([rc, rc + [1e-3, 0, 0], rc + [1e-3, 1e-3, 0], rc + [0, 1e-3, 0]], np.float32)))
                m.GetFaceVertexCountsAttr().Set([4])
                m.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
        tr, ori, sc = self._roll_xf
        roll_img = UsdGeom.Imageable(self.meshes["roll"])
        if b.roll_empty():
            roll_img.MakeInvisible()
            sc.Set(Gf.Vec3f(1e-4, 1e-4, 1e-4))
        else:
            roll_img.MakeVisible()
            r = b.roll_r()
            tr.Set(Gf.Vec3d(*map(float, b.roll_c)))
            ori.Set(Gf.Quatf(*_quat_from_R(b.roll_R)))
            sc.Set(Gf.Vec3f(1.0, float(r), float(r)))


TRACKER = Tracker()


def reset_episode():
    TRACKER.reset()


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_bandage_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            obs = orig(self, *a, **k)
            if getattr(getattr(self, "task", None), "module", None) is None or \
                    getattr(self.task.module, "INSTRUCTION", None) != INSTRUCTION:
                return obs
            _ENV["env"] = self._env
            reset_episode()
            TRACKER.render()
            for _ in range(4):
                self._base.sim.render()
            for _ in range(2):
                self._env.step(self._ready)
                self._base.sim.render()
            return self._observation()

        reset._bandage_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_bandage_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            if getattr(self.task.module, "INSTRUCTION", None) == INSTRUCTION:
                _ENV["env"] = self._env
                TRACKER.step(self._env)
            return base(self, *a, **k)

        _task_step._bandage_hooked = True
        cls._task_step = _task_step


_install_hooks()

# -- scoring -------------------------------------------------------------------------------------------------------
CLEAR = 0.10           # every gripper point >= 10.0 cm from form and bandage
STILL = 0.02           # both TCPs < 2.0 cm/s
JAW_OPEN = 0.05        # both jaws open past 5.0 cm
HOLD_STEPS = 20        # 1.0 s continuous hold at 20 Hz


def parts() -> dict:
    b = TRACKER.b
    if b is None:
        return {"valid": False}
    out = {"valid": bool(not b.bad and np.isfinite(b.x).all())}
    pt = bd.pattern(b)
    out["wrist_turns"], out["loops"], out["total_revs"] = float(pt["wrist_turns"]), int(pt["loops"]), float(pt["total_revs"])
    out["passes"], out["under_passes"] = int(b.passes), int(b.under_passes)
    thumb, tips = bd.bare_thumb_and_tips(b)
    out["thumb_bare"], out["tips_bare"] = bool(thumb), bool(tips)
    out["loose"] = int(bd.loose(b))
    pw, ph = bd.cuff_pressures_mmhg(b)
    out["wrist_mmhg"], out["hand_mmhg"] = pw, ph
    out["pressure_ok"] = bool(PRESSURE_MIN_MMHG <= pw <= PRESSURE_MAX_MMHG
                              and PRESSURE_MIN_MMHG <= ph <= PRESSURE_MAX_MMHG)
    unwound_m = (b.paid - 1) * bd.H
    out["snug_complete"] = bool(b.roll_empty() and unwound_m >= FULL_BANDAGE_M - 1e-6
                                and bd.MIN_TOTAL_REVS <= out["total_revs"] <= bd.MAX_TOTAL_REVS
                                and bd.PATCH_LEN > 0 and out["loose"] == 0 and out["pressure_ok"])
    out["fastened"] = bool(b.fastened)
    d_form = float(bd.arm_sdf(TRACKER.gp)[0].min())
    d_strip = float(np.min(np.linalg.norm(b.x[:b.paid].reshape(-1, 3)[None, :, :] - TRACKER.gp[:, None, :], axis=2)))
    out["clear"] = round(min(d_form, d_strip), 3)
    out["released"] = bool(not b.held_by() and (TRACKER.ap > JAW_OPEN).all() and out["clear"] >= CLEAR)
    out["still"] = bool(TRACKER.speed < STILL)
    out["held_1s"] = bool(TRACKER.hold_steps >= HOLD_STEPS)
    return out


def score(objects=None) -> float:
    q = parts()
    if not q.get("valid"):
        return 0.0
    s = 0.0
    if q["wrist_turns"] >= 1.0:
        s = 0.25
        if q["loops"] >= 1:
            s = 0.5
            if q["loops"] >= 2 and q["thumb_bare"] and q["tips_bare"] and q["snug_complete"]:
                s = 0.75
                if q["fastened"] and q["released"] and q["still"] and q["held_1s"]:
                    s = 1.0
    return s


def success(objects=None) -> bool:
    return score(objects) >= 1.0


def progress(objects=None) -> dict:
    q = parts()
    if not q.get("valid"):
        return {"valid": 0}
    return {"wrist_turn": int(q["wrist_turns"] >= 1.0), "loops": q["loops"], "passes": q["passes"],
            "under_passes": q["under_passes"], "fastened": int(q["fastened"]), "released": int(q["released"])}


VARIATION = {"roll_x": {"dx": 0.01, "dy": 0.01}}
