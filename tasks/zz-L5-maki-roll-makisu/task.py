"""Form and seal a cucumber maki (hosomaki) with a cling-wrapped bamboo makisu, then unwrap the mat flat.

Scene (metres from the midpoint between the two arm bases, x forward, y left; heights above the station table TOP):
  - Cutting board (Winco CBBU-1218 blue NSF HDPE board, 30.5 x 45.7 x 1.27 cm, 1.60 kg): centred at (0.3475, 0.0),
    spanning x = 0.1950..0.5000 m, y = -0.2285..+0.2285 m, top at TOP + 0.0127 m.
  - Bamboo makisu (Joyce Chen J33-0022 9.5 in bamboo sushi rolling mat, 24.0 x 24.0 x 0.30 cm, 40 bamboo slats along y
    bound by cotton twine along x, wrapped in 1 layer of food-grade cling film, 85 g): centred on y = 0
    (y = -0.120..+0.120 m), spanning x = 0.180..0.420 m so its near edge (x = 0.180 m) overhangs the cutting board's
    near edge (x = 0.195 m) by 1.5 cm.
  - Half roasted nori sheet (Yamamotoyama Gold half-sheet, 10.5 x 19.0 cm, rough side up): on the mat at
    x = 0.195..0.300 m (1.5 cm in from the mat's near edge), y = -0.095..+0.095 m, spread evenly with a 6.0 mm (85 g)
    layer of room-temperature seasoned Calrose sushi rice over its near 9.0 cm (x = 0.195..0.285 m) and leaving its far
    1.5 cm strip (x = 0.285..0.300 m) bare and lightly water-misted.
  - Cucumber baton (seedless English cucumber stick, 1.0 x 19.0 x 1.0 cm, 19 g): across the middle of the rice bed at
    x = 0.230..0.240 m (centre x = 0.235 m, 4.0 cm from the nori's near edge), y = -0.095..+0.095 m.

Run it (600 steps = 30 s at 20 Hz):
  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 600 \\
    --instruction "Use the bamboo mat to form the nori, rice, and cucumber into a straight, closed sushi roll without touching the food directly or rolling the mat inside it, then open the mat flat with the sealed roll resting seam-down on it."
"""

from __future__ import annotations

import math
import os
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

import makisu as mk  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.5
RUN_LIMIT = 600
MAKI_TASK = True

INSTRUCTION = (
    "Use the bamboo mat to form the nori, rice, and cucumber into a straight, closed sushi roll without touching "
    "the food directly or rolling the mat inside it, then open the mat flat with the sealed roll resting seam-down on it."
)

BOARD_COL = (0.14, 0.36, 0.60)          # Winco CBBU-1218 HACCP blue HDPE
BAMBOO_EVEN = (0.84, 0.71, 0.43)        # Alternating natural bamboo slats
BAMBOO_ODD = (0.72, 0.57, 0.31)
TWINE_COL = (0.95, 0.94, 0.88)          # White cotton binding twine
NORI_COL = (0.05, 0.11, 0.06)           # Roasted dark-green/black sushi nori
RICE_COL = (0.96, 0.96, 0.93)           # Seasoned short-grain white sushi rice
CUC_SKIN_COL = (0.18, 0.64, 0.20)       # Cucumber outer green
CUC_FLESH_COL = (0.60, 0.88, 0.50)      # Pale-green cucumber core at the cut ends

BOARD = Box(
    "cutting_board",
    pos=(float(mk.BOARD_XY0[0]), float(mk.BOARD_XY0[1]), float(TOP + mk.BOARD_SIZE[2] / 2)),
    size=tuple(float(v) for v in mk.BOARD_SIZE),
    color=BOARD_COL,
    mass=mk.BOARD_MASS,
)
OBJECTS = [BOARD]

TWIN_EXTRA = [
    {
        "name": "bamboo_mat",
        "kind": "body",
        "real": "Joyce Chen J33-0022 9.5 in bamboo sushi rolling mat (240 x 240 x 3 mm, 40 slats on cotton twine, cling-wrapped)",
        "size_mm": [240, 240, 3],
        "colour": "natural bamboo",
        "pose": [0.300, 0.0, float(TOP + mk.BOARD_SIZE[2] + mk.MAT_T / 2)],
    },
    {
        "name": "nori_and_rice",
        "kind": "body",
        "real": "Yamamotoyama Gold half nori sheet (105 x 190 mm) with 6 mm (85 g) seasoned Calrose sushi rice over near 90 mm, far 15 mm bare misted",
        "size_mm": [105, 190, 6],
        "colour": "dark green nori / white rice",
        "pose": [0.2475, 0.0, float(TOP + mk.BOARD_SIZE[2] + mk.MAT_T + 0.003)],
    },
    {
        "name": "cucumber_stick",
        "kind": "body",
        "real": "Seedless English cucumber baton (10 x 190 x 10 mm, 19 g)",
        "size_mm": [10, 190, 10],
        "colour": "green",
        "pose": [float(mk.CUC_X0), 0.0, float(TOP + mk.BOARD_SIZE[2] + mk.MAT_T + mk.R_OPEN)],
    },
]

# -- Scoring thresholds (metres, seconds, m/s) -----------------------------------------------------------------------
BOARD_SHIFT_MAX = 0.020
OPEN_JAW_MIN = 0.020
CLEAR_MIN = 0.100
STILL_SPEED_MAX = 0.020
HOLD_MIN_S = 0.50

_BOXES_LOCAL = None
_GRID3 = np.array([[i, j, k] for i in (0.0, 0.5, 1.0) for j in (0.0, 1.0) for k in (0.0, 0.5, 1.0)])


def _gripper_boxes_local():
    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    text = pathlib.Path(YAM_URDF).read_text()
    root = pathlib.Path(YAM_URDF).parent

    def stl(p):
        b = p.read_bytes()
        if b[:5] == b"solid" and b"facet" in b[:300]:
            return np.array(re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", b), float)
        n = int.from_bytes(b[80:84], "little")
        a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
        return a["v"].reshape(-1, 3).astype(float)

    out = {}
    for link, split in (("gripper", None), ("tip_left", -0.044), ("tip_right", 0.044)):
        block = re.search(r'<link name="%s".*?</link>' % link, text, re.S).group(0)
        groups: dict[int, list] = {}
        for c in re.findall(r"<collision>.*?</collision>", block, re.S):
            xyz = np.array(re.search(r'xyz="([^"]+)"', c).group(1).split(), float)
            rpy = np.array(re.search(r'rpy="([^"]+)"', c).group(1).split(), float)
            v = stl(root / re.search(r'filename="([^"]+)"', c).group(1)) @ rpy_matrix(rpy).T + xyz
            key = 0 if split is None else int((v[:, 1].mean() < split) if split < 0 else (v[:, 1].mean() > split))
            groups.setdefault(key, []).append(v)
        out[link] = [(np.min(np.concatenate(g), 0), np.max(np.concatenate(g), 0)) for g in groups.values()]
    return out


def _arr(t):
    t = t.torch if hasattr(t, "torch") else t
    return t.detach().cpu().numpy().astype(np.float64)


def _rice_slab_faces(nu: int, ny: int) -> np.ndarray:
    """Closed triangle mesh indices for a slab of (nu, ny) bottom vertices + (nu, ny) top vertices."""
    n = nu * ny
    bot = np.arange(n).reshape(nu, ny)
    top = (n + np.arange(n)).reshape(nu, ny)
    f = []
    for i in range(nu - 1):
        for j in range(ny - 1):
            f.extend([(top[i, j], top[i + 1, j], top[i + 1, j + 1]), (top[i, j], top[i + 1, j + 1], top[i, j + 1])])
            f.extend([(bot[i, j], bot[i + 1, j + 1], bot[i + 1, j]), (bot[i, j], bot[i, j + 1], bot[i + 1, j + 1])])
    for j in range(ny - 1):
        f.extend([(bot[0, j], top[0, j], top[0, j + 1]), (bot[0, j], top[0, j + 1], bot[0, j + 1])])
        f.extend([(bot[-1, j], top[-1, j + 1], top[-1, j]), (bot[-1, j], bot[-1, j + 1], top[-1, j + 1])])
    for i in range(nu - 1):
        f.extend([(bot[i, 0], bot[i + 1, 0], top[i + 1, 0]), (bot[i, 0], top[i + 1, 0], top[i, 0])])
        f.extend([(bot[i, -1], top[i + 1, -1], bot[i + 1, -1]), (bot[i, -1], top[i, -1], top[i + 1, -1])])
    return np.asarray(f, dtype=np.int32)


def _twine_mesh_data(xm_grid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Three white cotton twine lines running along x across the 40 bamboo slats."""
    cols = (2, 5, 8)
    verts = []
    faces = []
    half_w = 0.0018
    for ci, col in enumerate(cols):
        base_v = ci * (2 * mk.NX_M)
        pts = xm_grid[:, col, :]
        du = np.empty_like(pts)
        du[:-1] = pts[1:] - pts[:-1]
        du[-1] = pts[-1] - pts[-2]
        nrm = np.c_[-du[:, 2], np.zeros(len(pts)), du[:, 0]]
        nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-9)
        p_lift = pts + 0.0006 * nrm
        for i in range(mk.NX_M):
            verts.append(p_lift[i] + np.array([0.0, -half_w, 0.0]))
            verts.append(p_lift[i] + np.array([0.0, +half_w, 0.0]))
        for i in range(mk.NX_M - 1):
            a, b, c, d = base_v + 2 * i, base_v + 2 * i + 1, base_v + 2 * (i + 1) + 1, base_v + 2 * (i + 1)
            faces.extend([(a, b, c), (a, c, d)])
    return np.asarray(verts, dtype=np.float32), np.asarray(faces, dtype=np.int32)


class Tracker:
    def __init__(self):
        self.m: mk.MakisuRoll | None = None
        self.meshes: dict | None = None
        self.rice_slab_faces = _rice_slab_faces(mk.NU_N - 3, mk.NY_R)
        self.steps = 0
        self.dt = 0.05
        self.tcp = np.zeros((2, 3)) + [0.0, 0.0, 5.0]
        self.aperture = np.full(2, 0.088)
        self.grip_pts = np.zeros((2, 3)) + [0.0, 0.0, 5.0]
        self.prev_jaw = None
        self.grip_speed = 0.0
        self.still_t = 0.0
        self.clear_dist = 1.0

    def _origin(self, base) -> np.ndarray:
        return _arr(base.scene.env_origins[0])

    def _board_pose(self, base):
        from yam_arena.scene import object_states

        s = object_states(self._env, [BOARD.name])[BOARD.name]
        qx, qy, qz, qw = s.quat_xyzw
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return np.asarray(s.pos, float), yaw

    def _hands(self, base):
        global _BOXES_LOCAL
        from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

        if _BOXES_LOCAL is None:
            _BOXES_LOCAL = _gripper_boxes_local()
        o = self._origin(base)
        tcp, rot, ap, boxes, jaw_pts, all_pts = [], [], [], [], [], []
        for i, side in enumerate(("left", "right")):
            rob = base.scene[f"{side}_robot"]
            lp = link_poses(rob)
            t_w = np.asarray(tcp_world(rob), float) - o
            p_w = np.asarray(pad_tips_world(rob), float) - o
            g = lp["gripper"].copy()
            g[:3, 3] -= o
            z = g[:3, 2] / max(np.linalg.norm(g[:3, 2]), 1e-9)
            xc = lp["tip_left"][:3, 3] - lp["tip_right"][:3, 3]
            xc = xc - z * float(xc @ z)
            xc = xc / np.linalg.norm(xc) if np.linalg.norm(xc) > 1e-4 else g[:3, 0]
            r_m = np.stack([xc, np.cross(z, xc), z], axis=1)
            ids = rob.find_joints(["joint7", "joint8"], preserve_order=True)[0]
            q = _arr(rob.data.joint_pos)[0, ids]
            a_val = float(abs(q[0]) + abs(q[1]))
            tcp.append(t_w)
            rot.append(r_m)
            ap.append(a_val)
            half = 0.5 * a_val * r_m[:, 0]
            j_arr = np.array([t_w, p_w, t_w + half, t_w - half, p_w + half, p_w - half])
            jaw_pts.append(j_arr)
            pts_arm = [j_arr]
            for link, bl in _BOXES_LOCAL.items():
                lw = lp[link].copy()
                lw[:3, 3] -= o
                for lo, hi in bl:
                    boxes.append((lw, lo, hi, i))
                    loc = lo + _GRID3 * (hi - lo)
                    pts_arm.append(loc @ lw[:3, :3].T + lw[:3, 3])
            all_pts.append(np.vstack(pts_arm))
        return np.array(tcp), np.array(rot), np.array(ap), boxes, jaw_pts, np.vstack(all_pts)

    def reset(self, env):
        self._env = env
        base = env.unwrapped
        self.dt = float(base.step_dt)
        self.m = mk.MakisuRoll(TOP)
        self.m.reset(self._board_pose(base))
        self.rice_slab_faces = _rice_slab_faces(self.m.rice_nu, mk.NY_R)
        self.steps = 0
        self.prev_jaw = None
        self.grip_speed = 0.0
        self.still_t = 0.0
        tcp, _rot, ap, _boxes, _jaw_pts, all_pts = self._hands(base)
        self.tcp, self.aperture, self.grip_pts = tcp, ap, all_pts
        self._update_clearance()
        self._render(base)

    def _update_clearance(self):
        if self.m is None or self.m.bad:
            self.clear_dist = 0.0
            return
        obj_pts = np.vstack([self.m.xm[::2], self.m.xn, self.m.cuc])
        g_pts = self.grip_pts[::3]
        self.clear_dist = float(np.linalg.norm(g_pts[:, None, :] - obj_pts[None, :, :], axis=2).min())

    def step(self, env):
        if self.m is None:
            self.reset(env)
        self._env = env
        base = env.unwrapped
        tcp, rot, ap, boxes, jaw_pts, all_pts = self._hands(base)
        bpose = self._board_pose(base)
        self.tcp, self.aperture, self.grip_pts = tcp, ap, all_pts
        self.m.step(tcp, rot, ap, boxes=boxes, board_pose=bpose, jaw_pts=jaw_pts, dt=self.dt)
        self.steps += 1

        jp = np.vstack(jaw_pts)
        if self.prev_jaw is not None:
            self.grip_speed = float(np.linalg.norm(jp - self.prev_jaw, axis=1).max()) / max(self.dt, 1e-6)
        self.prev_jaw = jp

        self._update_clearance()
        mt = self.m.metrics()
        released_clear = (
            mt.get("valid", False)
            and (not mt["held"])
            and bool((ap >= OPEN_JAW_MIN).all())
            and (self.clear_dist >= CLEAR_MIN)
        )
        is_still = released_clear and (self.grip_speed < STILL_SPEED_MAX) and (mt.get("roll_speed", 1.0) < STILL_SPEED_MAX)
        self.still_t = (self.still_t + self.dt) if is_still else 0.0

        self._render(base)
        if os.environ.get("MAKI_LOG") and self.steps % 20 == 0:
            print(
                f"[maki] step {self.steps:4d} score={self.score():.2f} pins={list(self.m.pins.keys())} "
                f"st={mt.get('stages')} peel={mt.get('peeled_free')} trap={mt.get('mat_trapped')} "
                f"sealed={mt.get('seam_sealed')} open={mt.get('roll_on_open_mat')} "
                f"skew={mt.get('skew_x', 0)*100:.2f}cm clear={self.clear_dist*100:.1f}cm still={self.still_t:.2f}s",
                flush=True,
            )

    def parts(self) -> dict:
        if self.m is None:
            return {"valid": False}
        mt = self.m.metrics()
        if not mt.get("valid", False):
            return {"valid": False}
        why = ""
        if mt["board_shift"] > BOARD_SHIFT_MAX:
            why = "board_shifted"
        elif not mt["food_on_board"] or not mt["mat_on_board"]:
            why = "off_board"
        elif mt["direct_food_touch"]:
            why = "direct_food_touch"
        elif mt["mat_trapped"]:
            why = "mat_trapped"
        jaws_open = bool((self.aperture >= OPEN_JAW_MIN).all())
        released = (
            (not mt["held"])
            and jaws_open
            and (self.clear_dist >= CLEAR_MIN)
            and (self.still_t >= HOLD_MIN_S - 1e-6)
        )
        return {
            **mt,
            "why": why,
            "jaws_open": jaws_open,
            "clear_dist": self.clear_dist,
            "grip_speed": self.grip_speed,
            "still_t": self.still_t,
            "released_parked": released,
        }

    def score(self) -> float:
        p = self.parts()
        if not p.get("valid", False) or p["why"]:
            return 0.0
        if not p["folded_over"]:
            return 0.0
        if not p["closed_tube"]:
            return 0.25
        if not (p["seam_sealed"] and p["straight"] and p["roll_on_open_mat"]):
            return 0.50
        return 1.0 if p["released_parked"] else 0.75

    def _render(self, base):
        try:
            from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
            from isaaclab.sim.utils.stage import get_current_stage

            stage = get_current_stage()
            o = self._origin(base)
            m = self.m

            def make_mesh(name, faces, rgb, rough=0.65):
                prim_p = f"/World/Maki_{name}"
                mesh = UsdGeom.Mesh.Define(stage, prim_p)
                mesh.CreateDoubleSidedAttr(True)
                mesh.CreateSubdivisionSchemeAttr("none")
                mesh.GetFaceVertexCountsAttr().Set([3] * len(faces))
                mesh.GetFaceVertexIndicesAttr().Set(faces.ravel().tolist())
                mat_p = f"/World/MakiLook_{name}"
                mat = UsdShade.Material.Define(stage, mat_p)
                sh = UsdShade.Shader.Define(stage, f"{mat_p}/Shader")
                sh.CreateIdAttr("UsdPreviewSurface")
                sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
                sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
                mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
                UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
                return mesh

            xm_w = (m.xm + o).astype(np.float32)
            tw_verts, tw_faces = _twine_mesh_data(m.xm.reshape(mk.NX_M, mk.NY_M, 3) + o)
            cuc_verts, cuc_faces = m.cucumber_box_mesh()
            cuc_verts = cuc_verts + o.astype(np.float32)

            if self.meshes is None or not self.meshes["slat_even"].GetPrim().IsValid():
                self.meshes = {
                    "slat_even": make_mesh("SlatsEven", m.mat_faces_even, BAMBOO_EVEN, 0.55),
                    "slat_odd": make_mesh("SlatsOdd", m.mat_faces_odd, BAMBOO_ODD, 0.60),
                    "twine": make_mesh("Twine", tw_faces, TWINE_COL, 0.80),
                    "nori": make_mesh("Nori", m.nori_faces, NORI_COL, 0.70),
                    "rice": make_mesh("Rice", self.rice_slab_faces, RICE_COL, 0.88),
                    "cuc_skin": make_mesh("CucSkin", cuc_faces[:8], CUC_SKIN_COL, 0.45),
                    "cuc_flesh": make_mesh("CucFlesh", cuc_faces[8:], CUC_FLESH_COL, 0.55),
                }

            pts_mat = Vt.Vec3fArray.FromNumpy(xm_w)
            self.meshes["slat_even"].GetPointsAttr().Set(pts_mat)
            self.meshes["slat_odd"].GetPointsAttr().Set(pts_mat)
            self.meshes["twine"].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(tw_verts))

            xn_w = (m.xn + o).astype(np.float32)
            self.meshes["nori"].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(xn_w))

            r_bot = m.xn.reshape(mk.NU_N, mk.NY_R, 3)[: m.rice_nu].reshape(-1, 3) + o
            r_top = m.rice_top_points() + o
            rice_w = np.vstack([r_bot, r_top]).astype(np.float32)
            self.meshes["rice"].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(rice_w))

            pts_cuc = Vt.Vec3fArray.FromNumpy(cuc_verts)
            self.meshes["cuc_skin"].GetPointsAttr().Set(pts_cuc)
            self.meshes["cuc_flesh"].GetPointsAttr().Set(pts_cuc)
        except Exception as e:  # noqa: BLE001
            if not getattr(self, "_render_warned", False):
                print(f"[maki] render warning: {e!r}", flush=True)
                self._render_warned = True


def tracker() -> Tracker | None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return None
    return getattr(ie.YamArenaEmbodiment, "_maki_tracker", None)


def _ours(emb) -> bool:
    task = getattr(emb, "task", None)
    return bool(task is not None and getattr(getattr(task, "module", None), "MAKI_TASK", False))


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_maki_hooked", False):
        return
    cls._maki_tracker = Tracker()
    base_reset, base_step = cls.reset, cls._task_step

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        if not _ours(self):
            return obs
        cls._maki_tracker.reset(self._env)
        for _ in range(2):
            self._env.step(self._ready)
            cls._maki_tracker.step(self._env)
        for _ in range(ie.RESET_RENDERS):
            self._base.sim.render()
        return self._observation()

    def _task_step(self, *a, **k):
        if _ours(self):
            cls._maki_tracker.step(self._env)
        return base_step(self, *a, **k)

    cls.reset, cls._task_step, cls._maki_hooked = reset, _task_step, True


_install_hooks()


# -- yam_arena task API ----------------------------------------------------------------------------------------------
def parts() -> dict:
    tr = tracker()
    return tr.parts() if tr is not None else {"valid": False}


def score(objects=None) -> float:
    tr = tracker()
    return tr.score() if tr is not None else 0.0


def success(objects=None) -> bool:
    if score(objects) < 1.0:
        return False
    if objects is not None and BOARD.name in objects:
        return bool(objects[BOARD.name].resting(max_speed=0.02))
    return True


def distance(objects=None) -> float:
    return 1.0 - score(objects)


def progress(objects=None) -> dict:
    p = parts()
    if not p.get("valid", False):
        return {"valid": 0}
    return {
        "bimanual_mat": int(p["bimanual_near_grasped"]),
        "folded_over": int(p["folded_over"]),
        "peeled_free": int(p["peeled_free"]),
        "closed_tube": int(p["closed_tube"]),
        "seam_sealed": int(p["seam_sealed"]),
        "straight": int(p["straight"]),
        "roll_on_open_mat": int(p["roll_on_open_mat"]),
        "released_parked": int(p["released_parked"]),
        "why": p["why"],
    }


VARIATION = {"obj:cutting_board": {"dx": 0.01, "dy": 0.01, "yaw": 0.05}}
