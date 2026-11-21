"""Take a punched handout from the 3-hole punch, thread all three binder rings and snap them shut.

Scene (station desk; metres from the arm-base midpoint, x forward, y left, heights above the desk top):
- Swingline 74063 12-sheet 3-hole punch (29.7 x 10.2 x 5.7 cm) taped on the left of the desk at (0.437, +0.155),
  29.7 cm side along y, 4.0 mm throat facing -x at x = 0.386 m (z = 2.8-3.2 cm), internal stop at x = 0.405 m.
- A 10-sheet US letter handout (21.6 x 27.9 cm, 20 lb bond, 46 g, 1.1 mm thick; three 9/32 in / 7.14 mm holes on
  4.25 in / 10.8 cm centres, 9.5 mm from the +x edge) sits in the throat at x = 0.404 m, overhanging toward -x.
- Taped open Avery 05714 navy-blue 1 in round-ring view binder (54.0 x 29.5 cm open) on the right at (0.295, -0.155),
  spine and 27.0 cm ring plate along y at x = 0.295 m, three 25.4 mm ID round rings (3.5 mm wire) at y = -0.263,
  -0.155, -0.047 m open 26 deg per half, with 1.8 cm end booster tabs at y = -0.290 and -0.020 m.

Score (end state, latched tear/pinch disqualifier over the whole trial; see score()):
- 0 if the handout is torn or pinched (threaded hole pulled > 6 mm past wire > 0.15 s, or rings shut onto unthreaded
  paper) or if any part of the handout remains inside the punch footprint (x in 0.386..0.488, y in +0.0065..+0.3035).
- 1.0 when out of the punch, all 3 holes are threaded onto the 3 distinct binder rings, all 3 rings are shut, the
  handout lies flat on one cover (centre >= 6 cm off x = 0.295 m, y centre in -0.2525..-0.0575 m, >= 95% of the points
  lying beyond +-2.5 cm of x = 0.295 m are within 3.5 cm of the desk), jaws are open (> 2.4 cm), and both arms are parked
  (every gripper collision box >= 10 cm from all binder parts and the handout, TCP speed < 2 cm/s, held >= 0.5 s).
- 0.6 when out of the punch and all 3 holes are threaded onto the 3 distinct rings (open or shut), without full 1.0.
- 0.3 when out of the punch and 1 or 2 holes are threaded onto distinct rings, with no tear/pinch; 0.0 otherwise.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --max-steps 900 \\
           --instruction "Put the punched handout into the binder and close the rings."
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

import paper as pp  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.5
RUN_LIMIT_STEPS = 900

INSTRUCTION = "Put the punched handout into the binder and close the rings."

# -- exact geometry (world m; local z above desk TOP) ---------------------------------------------------------------
# 1. Swingline 74063 3-hole punch: 29.7 cm (y) x 10.2 cm (x) x 5.7 cm (z), throat on -x face at x = 0.386
PUNCH_XC, PUNCH_YC = 0.437, 0.155
PUNCH_X0, PUNCH_X1 = PUNCH_XC - 0.051, PUNCH_XC + 0.051          # 0.386 .. 0.488 (10.2 cm)
PUNCH_Y0, PUNCH_Y1 = PUNCH_YC - 0.1485, PUNCH_YC + 0.1485        # +0.0065 .. +0.3035 (29.7 cm)
THROAT_STOP_X = 0.405                                            # 1.9 cm throat depth from x = 0.386
SLOT_Z0, SLOT_Z1 = 0.0280, 0.0320                                # 4.0 mm throat opening at 2.8 cm height
SHEET_EDGE0 = np.array([0.404, PUNCH_YC])                        # punched edge against stop (holes at x=0.3945)

PUNCH_BASE = ((PUNCH_X0, PUNCH_Y0, 0.000), (PUNCH_X1, PUNCH_Y1, SLOT_Z0))
PUNCH_BACK = ((THROAT_STOP_X, PUNCH_Y0, SLOT_Z0), (PUNCH_X1, PUNCH_Y1, 0.039))
PUNCH_JAW = ((PUNCH_X0, PUNCH_Y0, SLOT_Z1), (THROAT_STOP_X, PUNCH_Y1, 0.039))
PUNCH_HEADS = [
    ((0.391, PUNCH_YC + dy - 0.011, 0.039), (0.415, PUNCH_YC + dy + 0.011, 0.049))
    for dy in (-pp.HOLE_SPACE, 0.0, pp.HOLE_SPACE)
]
PUNCH_ARMS = [
    ((0.426, PUNCH_Y0 + 0.018, 0.039), (0.448, PUNCH_Y0 + 0.030, 0.052)),
    ((0.426, PUNCH_Y1 - 0.030, 0.039), (0.448, PUNCH_Y1 - 0.018, 0.052)),
]
PUNCH_HANDLE = ((0.424, PUNCH_Y0 + 0.016, 0.050), (0.448, PUNCH_Y1 - 0.016, 0.057))
PUNCH_DOT = ((PUNCH_X0 - 0.0005, PUNCH_YC - 0.006, 0.032), (PUNCH_X0 + 0.002, PUNCH_YC + 0.006, 0.037))

# 2. Avery 05714 navy-blue 1 in round-ring view binder, open flat (54.0 x 29.5 cm)
RING_X = 0.295
BINDER_YC = -0.155
BINDER_Y0, BINDER_Y1 = BINDER_YC - 0.1475, BINDER_YC + 0.1475    # -0.3025 .. -0.0075 (29.5 cm = 11.6 in)
RING_Y = np.array([BINDER_YC - pp.HOLE_SPACE, BINDER_YC, BINDER_YC + pp.HOLE_SPACE])  # -0.26295, -0.155, -0.04705

SPINE = ((RING_X - 0.018, BINDER_Y0, 0.000), (RING_X + 0.018, BINDER_Y1, 0.003))      # 3.6 cm (1.4 in) spine
FRONT = ((RING_X - 0.270, BINDER_Y0, 0.000), (RING_X - 0.018, BINDER_Y1, 0.003))      # 25.2 cm near cover
BACK = ((RING_X + 0.018, BINDER_Y0, 0.000), (RING_X + 0.270, BINDER_Y1, 0.003))       # 25.2 cm far cover
PLATE_Y0, PLATE_Y1 = BINDER_YC - 0.135, BINDER_YC + 0.135                              # 27.0 cm nickel spine plate
PLATE = ((RING_X - 0.011, PLATE_Y0, 0.003), (RING_X + 0.011, PLATE_Y1, 0.009))

COL_BINDER = (0.11, 0.20, 0.40)        # Avery navy blue polypropylene
COL_METAL = (0.76, 0.78, 0.81)         # nickel ring metal & punch deck
COL_DARK = (0.12, 0.12, 0.14)          # punch base, chip tray & soft-grip handle
COL_HEAD = (0.22, 0.23, 0.26)          # punch die heads
COL_GREEN = (0.15, 0.72, 0.28)         # EasyView alignment window
COL_PAPER = (0.96, 0.96, 0.94)         # 20 lb white bond
COL_INK = (0.18, 0.24, 0.36)           # printed handout header/text lines


def _box(name, lohi, color):
    lo, hi = np.array(lohi[0]), np.array(lohi[1])
    c = 0.5 * (lo + hi)
    return Box(name, pos=(float(c[0]), float(c[1]), TOP + float(c[2])), size=tuple(map(float, hi - lo)), color=color)


OBJECTS = [
    _box("punch_base", PUNCH_BASE, COL_DARK),
    _box("punch_back", PUNCH_BACK, COL_METAL),
    _box("punch_jaw", PUNCH_JAW, COL_METAL),
    *[_box(f"punch_head_{i}", b, COL_HEAD) for i, b in enumerate(PUNCH_HEADS)],
    *[_box(f"punch_arm_{i}", b, COL_METAL) for i, b in enumerate(PUNCH_ARMS)],
    _box("punch_handle", PUNCH_HANDLE, COL_DARK),
    _box("punch_guide", PUNCH_DOT, COL_GREEN),
    _box("binder_front", FRONT, COL_BINDER),
    _box("binder_spine", SPINE, COL_BINDER),
    _box("binder_back", BACK, COL_BINDER),
    _box("ring_plate", PLATE, COL_METAL),
]

TWIN_EXTRA = [
    {"name": "handout", "kind": "body",
     "real": "10 sheets 8.5 x 11 in 20 lb bond (46 g), 3-hole punched (9/32 in dia, 4.25 in spacing, 9.5 mm margin)",
     "size_mm": [216, 279, 1], "colour": "white", "pose": [0.294, PUNCH_YC, TOP + SLOT_Z0 + pp.THICK / 2]},
    {"name": "rings_and_boosters", "kind": "joint",
     "real": "Avery 05714 1 in round-ring mechanism (25.4 mm ID, 3.5 mm wire) with dual end booster tabs",
     "note": "bistable over-centre spring: open 26 deg per half, snaps shut past 12 deg on opposing ring pinch or inward booster push"},
]

_PUNCH_BOXES = (PUNCH_BASE, PUNCH_BACK, PUNCH_JAW, *PUNCH_HEADS, *PUNCH_ARMS, PUNCH_HANDLE)
_BINDER_BOXES = (FRONT, SPINE, BACK, PLATE)


def _fixed_boxes():
    return [(np.array(lo) + [0, 0, TOP], np.array(hi) + [0, 0, TOP]) for lo, hi in (*_PUNCH_BOXES, *_BINDER_BOXES)]


# -- gripper collision boxes from the YAM URDF ----------------------------------------------------------------------
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


_BOXES_LOCAL = None
_ENV: dict = {}


def _robots(env):
    base = env.unwrapped
    return [base.scene.articulations[f"{s}_robot"] for s in ("left", "right")]


def _arm_state(env):
    global _BOXES_LOCAL
    from yam_arena.runtime import link_poses, tcp_world

    if _BOXES_LOCAL is None:
        _BOXES_LOCAL = _gripper_boxes_local()
    tcp, rot, ap, boxes, pads = [], [], [], [], []
    for arm_idx, r in enumerate(_robots(env)):
        lp = link_poses(r)
        tcp.append(tcp_world(r))
        rot.append(lp["gripper"][:3, :3])
        q = r.data.joint_pos.torch[0].cpu().numpy()
        names = list(r.joint_names)
        ap.append(abs(float(q[names.index("joint7")])) + abs(float(q[names.index("joint8")])))
        for link, bl in _BOXES_LOCAL.items():
            for lo, hi in bl:
                boxes.append((lp[link], lo, hi))
                if link in ("tip_left", "tip_right") and lo[2] < -0.04:
                    pads.append((arm_idx, 0 if link == "tip_left" else 1, lp[link], lo, hi))
    return np.array(tcp), np.array(rot), np.array(ap), boxes, pads


def _box_sample_points(boxes):
    """Corners and face centres of every gripper collision box in world metres."""
    grid = np.array([[a, b, c] for a in (0.0, 0.5, 1.0) for b in (0.0, 0.5, 1.0) for c in (0.0, 0.5, 1.0)])
    out = []
    for world, lo, hi in boxes:
        loc = lo + grid * (hi - lo)
        out.append(loc @ world[:3, :3].T + world[:3, 3])
    return np.concatenate(out) if out else np.zeros((0, 3))


def _aabb_dist(pts: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """Euclidean distance from each point in ``pts`` (N, 3) to an axis-aligned box [lo, hi]."""
    d = np.maximum(0.0, np.maximum(lo[None, :] - pts, pts - hi[None, :]))
    return np.linalg.norm(d, axis=1)


# -- USD tube & tab mesh helpers ------------------------------------------------------------------------------------
def _tube(polys, r=pp.WIRE_R, sides=8):
    pts = []
    for q in polys:
        t = np.gradient(q, axis=0)
        t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-12)
        ref = np.array([0.0, 1.0, 0.0]) if abs(t[0, 1]) < 0.9 else np.array([1.0, 0.0, 0.0])
        a = np.cross(t, ref)
        a /= np.maximum(np.linalg.norm(a, axis=1, keepdims=True), 1e-12)
        b = np.cross(t, a)
        for k in range(sides):
            ang = 2 * math.pi * k / sides
            pts.append(q + r * (math.cos(ang) * a + math.sin(ang) * b))
    return np.concatenate(pts)


def _tube_faces(n_poly, n_pts, sides=8):
    f = []
    for p in range(n_poly):
        for k in range(sides):
            a0 = (p * sides + k) * n_pts
            b0 = (p * sides + (k + 1) % sides) * n_pts
            for j in range(n_pts - 1):
                f += [(a0 + j, b0 + j, b0 + j + 1), (a0 + j, b0 + j + 1, a0 + j + 1)]
    return np.array(f, np.int32)


def _booster_mesh(segs):
    """Points (8 per tab = 16) and triangles for the two end booster tabs."""
    pts, faces = [], []
    hw = pp.BOOSTER_W / 2
    ht = 0.0012
    for base, tip in segs:
        d = tip - base
        d /= max(np.linalg.norm(d), 1e-9)
        n = np.cross([1.0, 0.0, 0.0], d)
        b0 = len(pts)
        for end in (base, tip):
            for sx in (-hw, hw):
                for sn in (-ht, ht):
                    pts.append(end + np.array([sx, 0.0, 0.0]) + sn * n)
        # 8 box vertices: 0..3 at base, 4..7 at tip
        quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
        for a, b, c, dd in quads:
            faces += [(b0 + a, b0 + b, b0 + c), (b0 + a, b0 + c, b0 + dd)]
    return np.array(pts, np.float32), np.array(faces, np.int32)


class Tracker:
    def __init__(self):
        self.sheet: pp.Sheet | None = None
        self.rings: pp.Rings | None = None
        self.meshes: dict = {}
        self.tcp = np.zeros((2, 3)) + [0, 0, 5]
        self.aperture = np.full(2, 0.09)
        self.sample_pts = np.zeros((0, 3))
        self.steps = 0
        self.still_t = 0.0

    def reset(self):
        self.sheet = pp.Sheet()
        x0 = self.sheet.flat(SHEET_EDGE0, TOP + SLOT_Z0 + pp.THICK / 2 + 0.0005,
                             along_u=(0.0, 1.0), along_v=(-1.0, 0.0))
        self.sheet.place(x0, TOP, _fixed_boxes())
        self.rings = pp.Rings(RING_X, RING_Y, TOP + PLATE[1][2], plate_y_ends=(PLATE_Y0, PLATE_Y1))
        # Settle the overhanging handout naturally under gravity in the punch throat before step 0
        for _ in range(25):
            self._advance(np.zeros((2, 3)) + [0, 0, 5], np.tile(np.eye(3), (2, 1, 1)), np.full(2, 0.09), [], [])
        self.steps = 0
        self.tcp = np.zeros((2, 3)) + [0, 0, 5]
        self.aperture = np.full(2, 0.09)
        self.sample_pts = np.zeros((0, 3))
        self.still_t = 0.0

    def _advance(self, tcp, rot, ap, boxes, fingers):
        r = self.rings
        r.update(fingers, sheet=self.sheet)
        self.sheet.step(tcp, rot, ap, boxes=boxes, rings=r)
        r.track_tear(self.sheet)

    def step(self, env):
        if self.sheet is None:
            self.reset()
        tcp, rot, ap, boxes, fingers = _arm_state(env)
        self._advance(tcp, rot, ap, boxes, fingers)
        spd = float(np.max(np.linalg.norm(tcp - self.tcp, axis=1))) / 0.05 if self.steps > 0 else 9.0
        self.tcp, self.aperture = tcp, ap
        self.sample_pts = _box_sample_points(boxes)
        p = parts()
        if p.get("clear", 0.0) >= CLEAR and spd < STILL_V and p.get("released"):
            self.still_t += 0.05
        else:
            self.still_t = 0.0
        self.steps += 1
        self.render()
        if os.environ.get("BINDER_LOG") and self.steps % 20 == 0:
            r = self.rings
            print(f"[binder] step {self.steps} pins {list(self.sheet.pins)} ap {ap.round(3)} "
                  f"theta {math.degrees(r.theta):.1f} thread {r.thread} torn {r.torn} ({r.tear_why}) "
                  f"holes {np.round(self.sheet.holes(), 3).tolist()} p {p}", flush=True)

    def render(self):
        from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
        import omni.usd

        stage = omni.usd.get_context().get_stage()

        def mesh(name, faces, color, rough=0.6):
            m = self.meshes.get(name)
            if m is not None and m.GetPrim().IsValid():
                return m
            m = UsdGeom.Mesh.Define(stage, f"/World/{name}")
            m.CreateDoubleSidedAttr(True)
            m.CreateSubdivisionSchemeAttr("none")
            m.CreateFaceVertexCountsAttr([3] * len(faces))
            m.CreateFaceVertexIndicesAttr(np.asarray(faces).ravel().tolist())
            mat = UsdShade.Material.Define(stage, f"/World/{name}Mat")
            sh = UsdShade.Shader.Define(stage, f"/World/{name}Mat/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
            self.meshes[name] = m
            return m

        s = self.sheet
        m = mesh("Handout", s.render_faces, COL_PAPER, 0.82)
        m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(s.render_points().astype(np.float32)))

        mp = mesh("HandoutPrint", s.print_faces, COL_INK, 0.88)
        mp.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(s.print_points().astype(np.float32)))

        polys = list(self.rings.points().values())
        mr = mesh("BinderRings", _tube_faces(len(polys), pp.N_ARC, sides=8), COL_METAL, 0.25)
        mr.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(_tube(polys, sides=8).astype(np.float32)))

        bpts, bfaces = _booster_mesh(self.rings.booster_segments())
        mb = mesh("BinderBoosters", bfaces, COL_METAL, 0.28)
        mb.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(bpts))


TRACKER = Tracker()


def reset_episode():
    TRACKER.reset()


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001 (offline import)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_binder_handout_hooked", False):
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

        reset._binder_handout_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_binder_handout_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            if getattr(self.task.module, "INSTRUCTION", None) == INSTRUCTION:
                _ENV["env"] = self._env
                TRACKER.step(self._env)
            return base(self, *a, **k)

        _task_step._binder_handout_hooked = True
        cls._task_step = _task_step


_install_hooks()

# -- scoring (exact match to task.json & README.md) -----------------------------------------------------------------
CLEAR = 0.10              # 10 cm clearance from binder (covers, spine, plate, rings) and handout
STILL_V = 0.02            # 2 cm/s TCP speed limit
PARK_S = 0.5              # 0.5 s parked hold
COVER_OFFSET = 0.06       # handout centre >= 6 cm to one side of the ring line (on either open cover)
FLAT_ARCH_X = 0.025       # points more than 2.5 cm from the ring line must lie flat on the cover
FLAT_MAX_Z = 0.035        # <= 3.5 cm above the desk
FLAT_FRAC = 0.95          # >= 95% of those sheet points within FLAT_MAX_Z


def _binder_clearance(pts: np.ndarray, rings: pp.Rings) -> float:
    """Minimum 3D distance from gripper sample points ``pts`` to ALL binder parts (covers, spine, plate, rings, tabs)."""
    if not len(pts):
        return 0.0
    d_min = np.inf
    for lo, hi in _BINDER_BOXES:
        lo_w = np.array(lo) + [0.0, 0.0, TOP]
        hi_w = np.array(hi) + [0.0, 0.0, TOP]
        d_min = min(d_min, float(_aabb_dist(pts, lo_w, hi_w).min()))
    ring_pts = np.concatenate(list(rings.points().values()))
    for base, tip in rings.booster_segments():
        ring_pts = np.vstack([ring_pts, base + np.linspace(0.0, 1.0, 5)[:, None] * (tip - base)])
    d_rings = float(np.min(np.linalg.norm(pts[:, None, :] - ring_pts[None, :, :], axis=2))) - pp.WIRE_R
    return min(d_min, max(0.0, d_rings))


def parts() -> dict:
    s, r = TRACKER.sheet, TRACKER.rings
    if s is None or r is None:
        return {"valid": False}
    valid = bool(not s.bad and np.isfinite(s.x).all())
    in_punch = ((s.x[:, 0] > PUNCH_X0) & (s.x[:, 0] < PUNCH_X1) &
                (s.x[:, 1] > PUNCH_Y0) & (s.x[:, 1] < PUNCH_Y1))
    out_of_punch = bool(not in_punch.any())

    on = r.on_rings()
    n_on = int(sum(on))
    all_on = bool(all(on))
    shut = bool(r.closed)

    mid = s.x.mean(axis=0)
    on_one_cover = abs(float(mid[0]) - RING_X) >= COVER_OFFSET
    away_from_arch = np.abs(s.x[:, 0] - RING_X) > FLAT_ARCH_X
    flat_frac = float((s.x[away_from_arch, 2] - TOP <= FLAT_MAX_Z).mean()) if away_from_arch.any() else 0.0
    # Also ensure handout lies within the binder's y span (with 3 cm margin)
    in_binder_y = (float(mid[1]) >= BINDER_Y0 + 0.05) and (float(mid[1]) <= BINDER_Y1 - 0.05)
    flat = bool(on_one_cover and in_binder_y and flat_frac >= FLAT_FRAC)

    pts = TRACKER.sample_pts
    if len(pts):
        d_paper = float(np.min(np.linalg.norm(pts[:, None, :] - s.x[None, :, :], axis=2)))
        d_binder = _binder_clearance(pts, r)
        clear = min(d_paper, d_binder)
    else:
        clear = 0.0

    released = bool(not s.pins and (TRACKER.aperture > pp.PINCH_OPEN).all())
    parked = bool(released and clear >= CLEAR and TRACKER.still_t >= PARK_S)
    return {
        "valid": valid,
        "torn": bool(r.torn),
        "out_of_punch": out_of_punch,
        "on": n_on,
        "all_on": all_on,
        "shut": shut,
        "flat": flat,
        "clear": clear,
        "released": released,
        "parked": parked,
    }


def _ladder(p: dict) -> float:
    if not p.get("valid") or p["torn"] or not p["out_of_punch"]:
        return 0.0
    if p["all_on"] and p["shut"] and p["flat"] and p["released"] and p["parked"]:
        return 1.0
    if p["all_on"]:
        return 0.6
    if p["on"] >= 1:
        return 0.3
    return 0.0


def score(objects=None) -> float:
    return _ladder(parts())


def success(objects=None) -> bool:
    return score(objects) >= 1.0


def progress(objects=None) -> dict:
    p = parts()
    if not p.get("valid"):
        return {"valid": 0}
    return {k: (int(v) if isinstance(v, bool) else round(v, 3) if isinstance(v, float) else v) for k, v in p.items()}


VARIATION = {
    "punch_dy": 0.010,
    "binder_dx": 0.015,
    "binder_dy": 0.010,
}
