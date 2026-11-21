"""A home sewing machine sits sunk flush into a sewing table (its bed level with the table top), needle 33 cm in
front of the arm bases and 5 cm right of their midpoint, its arm and pillar running off to the right. Under the
lowered presser foot lie two basted pieces of cotton (24 x 20 cm, the near right corner rounded to a 6 cm radius),
the needle down in them at the chalked start mark, 15 mm in from the straight right edge, which lies on the throat
plate's 15 mm guide line. A second chalk mark on the near edge, 5 cm past the curve, ends the seam.

The robot presses the Start/Stop button on the front of the machine's head (the machine runs at its set, slow speed:
1 cm of fabric a second, 2.5 mm stitches), guides the fabric with both hands while the feed dogs draw it away,
turning it round the curve so the cut edge stays on the 15 mm line, presses Start/Stop again at the end mark, and
lets go.

The fabric is a reduced position-based cloth sheet (fabric_sheet.py) and the feed, foot and needle are a rule model
(seam_machine.py), stepped once per control step from the measured gripper poses, jaw openings and button travel;
score() reads the recorded stitches.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

import seam_machine as M  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # 0.7475
SCORE_OVER = "end"
SEAM_TASK = True  # marks this module for the embodiment hooks below

NX, NY = M.NEEDLE_XY
WHITE = (0.93, 0.93, 0.91)
GREY = (0.55, 0.57, 0.60)
STEEL = (0.78, 0.79, 0.81)
DARK = (0.15, 0.15, 0.17)
TABLE = (0.80, 0.70, 0.55)
GREEN = (0.10, 0.55, 0.25)

# -- sewing machine (Brother CS7000X-class computerized machine, 41 x 17 x 31 cm), set flush in a sewing table ------
# x: the bed runs from 5 cm in front of the needle to 12 cm behind it, the head's front face is 3 cm in front of it;
# y: the bed runs from 9 cm left of the needle to 32 cm right of it (the pillar at that end); the head's left face is
# level with the needle, with the needle-bar housing standing out of it.
MX0, MX1 = NX - 0.05, NX + 0.12  # 17 cm deep
HX0 = NX - 0.03  # front face of the head, 3 cm ahead of the needle
MY0, MY1 = NY - 0.32, NY + 0.09  # 41 cm wide
H_TOP = TOP + 0.31
BED = Box("machine_bed", pos=((MX0 + MX1) / 2, (MY0 + MY1) / 2, TOP + 0.0003), size=(MX1 - MX0, MY1 - MY0, 0.0006),
          color=WHITE)
THROAT = Box("throat_plate", pos=(NX + 0.005, NY - 0.005, TOP + 0.0004), size=(0.07, 0.10, 0.0008), color=STEEL)
YELLOW = (1.0, 0.82, 0.08)
GUIDES = [Box(f"guide_{int(d * 1000)}mm", pos=(NX + 0.01, NY - d, TOP + 0.00085), size=(0.05, 0.0012 if d != M.SA else 0.003, 0.0002),
              color=DARK if d != M.SA else YELLOW) for d in (0.010, M.SA, 0.020)]  # the 15 mm line taped in yellow
HEAD = Box("machine_head", pos=((HX0 + MX1) / 2, NY - 0.055, TOP + 0.20), size=(MX1 - HX0, 0.11, 0.22), color=WHITE)
NEEDLE_BAR = Box("needle_bar_housing", pos=(NX, NY, TOP + 0.07), size=(0.03, 0.025, 0.04), color=WHITE)
ARM = Box("machine_arm", pos=((HX0 + MX1) / 2, NY - 0.17, TOP + 0.235), size=(MX1 - HX0, 0.20, 0.15), color=WHITE)
PILLAR = Box("machine_pillar", pos=((MX0 + MX1) / 2, MY0 + 0.05, TOP + 0.155), size=(MX1 - MX0, 0.10, 0.31),
             color=WHITE)
FOOT_SHANK = Box("foot_shank", pos=(NX + 0.008, NY, TOP + 0.028), size=(0.006, 0.010, 0.044), color=STEEL)
FOOT = Box("presser_foot", pos=(NX, NY, TOP + 0.0042), size=(0.026, 0.018, 0.0035), color=STEEL)
# finger guard (generic low-shank needle/finger guard: a 4 mm wire loop clamped under the presser-foot screw, standing
# 2.2 cm in front of and 1.1 cm to each side of the needle, 1.2 cm above the bed)
GW, GX, GY, GZ = 0.004, 0.022, 0.011, 0.012
GUARD = [Box("finger_guard_front", pos=(NX - GX, NY, TOP + GZ), size=(GW, 2 * GY + GW, GW), color=STEEL),
         Box("finger_guard_left", pos=(NX - GX / 2 + 0.004, NY + GY, TOP + GZ), size=(GX + 0.008, GW, GW), color=STEEL),
         Box("finger_guard_right", pos=(NX - GX / 2 + 0.004, NY - GY, TOP + GZ), size=(GX + 0.008, GW, GW), color=STEEL)]


class SprungButton(Box):
    """The Start/Stop button; its spawner also authors its sprung slide."""


BTN_SIZE = (0.012, 0.018, 0.018)
BTN_PROUD = 0.005
BTN_TRAVEL = 0.004
BTN_ON = 0.0025  # pushed in this far = pressed (the switch's click point)
BTN_OFF = 0.0012  # back out past this = released (ready for the next press)
BTN_K = 400.0  # N/m, preloaded 1 mm: about 1.4 N at the click point
BTN_POS = (HX0 - BTN_PROUD + BTN_SIZE[0] / 2, NY - 0.085, TOP + 0.12)
BUTTON = SprungButton("start_stop_button", pos=BTN_POS, size=BTN_SIZE, mass=0.01, color=GREEN)

OBJECTS = [BED, THROAT, *GUIDES, HEAD, NEEDLE_BAR, ARM, PILLAR, FOOT_SHANK, FOOT, *GUARD, BUTTON]
VARIATION = {}
TWIN_EXTRA = [
    {"name": "fabric", "kind": "body", "size_mm": [240, 200], "colour": "navy quilting cotton (two basted layers)",
     "pose": [NX - M.START_UV[0] + 0.12, NY - M.SA + 0.10, 0.0]},
    {"name": "start_stop_slide", "kind": "joint", "real": "the machine's Start/Stop push switch",
     "note": "sim: sprung slider along x, 4 mm travel, clicks at 2.5 mm"},
    {"name": "needle", "kind": "body", "real": "HAX1 size 80/12 needle", "note": "visual; feed and stitching are a rule model"},
]

# -- scoring constants -----------------------------------------------------------------------------------------------
NEEDLE_CLEAR = 0.03  # no finger within 3 cm of the needle while the machine runs
PARK_CLEAR = 0.10  # parked: grippers 10 cm clear of the fabric
STILL_V = 0.02
PARK_S = 0.5
END_TOL = 0.01  # full/0.5 score: the seam ends within 1 cm of the end mark (and never >1 cm past it)
MIN_SEAM_S = 0.19  # partial score: seam reaches >= 19 cm along the chalked line (past halfway round the curve)
ST_MIN, ST_MAX = 0.001, 0.004  # stitch length band (2.5 mm nominal)
SA_FULL = (0.012, 0.018)  # 15 +- 3 mm curved-edge allowance
SA_MID = (0.009, 0.021)   # 15 +- 6 mm curved-edge allowance
SA_PART = (0.008, 0.022)  # 15 +- 7 mm curved-edge allowance


# -- Isaac side: the button's slide, authored when the button spawns --------------------------------------------------
_P: dict = {}


def _author_joints(stage, parent: str) -> None:
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    W = np.array([w[0], w[1], w[2]], dtype=float)
    j = UsdPhysics.PrismaticJoint.Define(stage, f"{parent}/{BUTTON.name}/slide")
    j.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in (W + np.array(BUTTON.pos)))))
    j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateBody1Rel().SetTargets([Sdf.Path(f"{parent}/{BUTTON.name}")])
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateAxisAttr("X")
    j.CreateLowerLimitAttr(0.0)
    j.CreateUpperLimitAttr(BTN_TRAVEL)
    j.CreateExcludeFromArticulationAttr(True)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(BTN_K)
    d.CreateDampingAttr(2.0)
    d.CreateTargetPositionAttr(-0.001)  # preloaded 1 mm against its outer stop
    fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"{parent}/{BUTTON.name}")).GetFilteredPairsRel()
    fp.AddTarget(Sdf.Path(f"{parent}/{HEAD.name}"))
    _P["slide"] = str(j.GetPath())


def _spawn_button(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_cuboid(prim_path, cfg, translation, orientation, **kwargs)
    _author_joints(get_current_stage(), str(prim_path).rsplit("/", 1)[0])
    return prim


def _button_object(spec: SprungButton):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import CollisionPropertiesCfg, MassPropertiesCfg, RigidBodyPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.constants import GRIPPY_FRICTION
    from yam_arena.station import linear_rgb

    _register_module()

    @configclass
    class ButtonCfg(sim_utils.CuboidCfg):
        func: object = clone(_spawn_button)

    cfg = ButtonCfg(size=tuple(spec.size), rigid_props=RigidBodyPropertiesCfg(),
                    mass_props=MassPropertiesCfg(mass=spec.mass), collision_props=CollisionPropertiesCfg(),
                    physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=GRIPPY_FRICTION[0],
                                                                    dynamic_friction=GRIPPY_FRICTION[1]),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.4))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _register_module():
    if __name__ not in sys.modules:
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _patch_scene(scene) -> None:
    if getattr(scene, "_seam_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, SprungButton):
            return _button_object(spec)
        return orig(spec)

    scene.task_object = task_object
    scene._seam_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _seam = True

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


def _install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(getattr(f, "_seam", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()


# -- fabric + machine tracker -----------------------------------------------------------------------------------------
def _box_dist(p, T, lo, hi):
    loc = (p - T[:3, 3]) @ T[:3, :3]
    d = np.maximum(np.maximum(lo - loc, loc - hi), 0.0)
    return np.linalg.norm(d, axis=1)


def finger_boxes(tcp, rot, aperture):
    """Both pads of one gripper as boxes in its TCP frame (x: closing axis, z: approach): 12 mm pads, 22 mm wide,
    running from the pad tips (13 mm past the TCP) 6 cm back toward the wrist."""
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rot, tcp
    a = aperture / 2
    return [(T, np.array([a, -0.011, -0.06]), np.array([a + 0.012, 0.011, 0.013])),
            (T, np.array([-a - 0.012, -0.011, -0.06]), np.array([-a, 0.011, 0.013]))]


def _arr(t):
    t = t.torch if hasattr(t, "torch") else t
    return t.detach().cpu().numpy().astype(np.float64)


def make_sheet():
    from fabric_sheet import Sheet

    ncx, ncy = round(M.FAB_L / M.CELL), round(M.FAB_H / M.CELL)
    x0, y0 = NX - M.START_UV[0], NY - M.START_UV[1]
    return Sheet(x0, y0, (M.FAB_L, M.FAB_H), TOP + 0.0006, table_z=TOP, cells=(ncx, ncy), mask=M.on_fabric,
                 bend_k=0.2, yield_strain=0.01, shear_k=0.5, mu=0.3, muc=0.0, kcomp=0.3, cmin=0.9, mut=0.3,
                 thickness=0.0012)


def needle_seg_dist(p):
    """Distance of points from the needle's vertical line (bed to needle-bar housing)."""
    p = np.atleast_2d(p)
    dz = np.maximum(np.maximum(TOP - p[:, 2], p[:, 2] - (TOP + 0.05)), 0.0)
    return np.hypot(np.hypot(p[:, 0] - NX, p[:, 1] - NY), dz)


class Tracker:
    def __init__(self):
        self._axis = {}
        self.S = None
        self.look = None

    def _origin(self, base):
        return _arr(base.scene.env_origins[0])

    def _hands(self, base):
        from yam_arena.runtime import link_poses, tcp_world

        o = self._origin(base)
        tcp, rot, ap = [], [], []
        for k in ("left_robot", "right_robot"):
            rob = base.scene[k]
            lp = link_poses(rob)
            g = lp["gripper"]
            z = g[:3, 2] / np.linalg.norm(g[:3, 2])
            if k not in self._axis:
                tl = lp["tip_left"][:3, 3] - lp["tip_right"][:3, 3]
                if np.linalg.norm(tl) > 0.03:
                    dots = [float(g[:3, c] @ tl) / np.linalg.norm(tl) for c in range(3)]
                    c = int(np.argmax(np.abs(dots)))
                    self._axis[k] = (c, 1.0 if dots[c] > 0 else -1.0)
            c, sg = self._axis.get(k, (0, 1.0))
            xc = sg * g[:3, c]
            xc = xc - z * (xc @ z)
            xc = xc / max(np.linalg.norm(xc), 1e-9)
            rot.append(np.stack([xc, np.cross(z, xc), z], 1))
            tcp.append(np.asarray(tcp_world(rob), float) - o)
            ids = rob.find_joints(["joint7", "joint8"], preserve_order=True)[0]
            q = _arr(rob.data.joint_pos)[0, ids]
            ap.append(float(abs(q[0]) + abs(q[1])))
        return np.array(tcp), np.array(rot), np.array(ap)

    def _button_depth(self, base):
        try:
            p = _arr(base.scene[BUTTON.name].data.root_pos_w)[0] - self._origin(base)
            return float(p[0] - BUTTON.pos[0])
        except Exception:  # noqa: BLE001
            return 0.0

    def reset(self, env):
        base = env.unwrapped
        self.dt = float(base.step_dt)
        self.S = make_sheet()
        self.mach = M.Machine(self.S, self.S.uv)
        tcp, rot, ap = self._hands(base)
        for _ in range(10):
            self.S.step(tcp, rot, ap, [b for k in range(2) for b in finger_boxes(tcp[k], rot[k], ap[k])], dt=self.dt)
        self.t = 0.0
        self.n = 0
        self.btn_down = False
        self.presses = 0
        self.needle_hit = False
        self.still_t = 0.0
        self._tcp_prev = None
        self.depth = 0.0
        self.last = {}
        self._render(base)

    def step(self, env):
        if self.S is None:
            return
        base = env.unwrapped
        tcp, rot, ap = self._hands(base)
        self.tcp = tcp
        boxes = [b for k in range(2) for b in finger_boxes(tcp[k], rot[k], ap[k])]
        # the Start/Stop switch toggles once per press (pushed past its click point, then let back out)
        self.depth = self._button_depth(base)
        if not self.btn_down and self.depth >= BTN_ON:
            self.btn_down = True
            self.presses += 1
            self.mach.set_running(not self.mach.running)
        elif self.btn_down and self.depth <= BTN_OFF:
            self.btn_down = False
        ext = self.mach.targets(self.dt)
        self.S.step(tcp, rot, ap, boxes, dt=self.dt, ext=ext)
        self.mach.after(self.dt)
        self.t += self.dt
        self.n += 1
        # fingers near the running needle (any part of either pad)
        if self.mach.running:
            for T, lo, hi in boxes:
                corners = np.array([[a, b, c] for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for c in (lo[2], hi[2])])
                pts = corners @ T[:3, :3].T + T[:3, 3]
                # sample the box densely enough (pads are 12 x 22 x 73 mm)
                g = np.linspace(0, 1, 4)
                samp = np.array([lo + (hi - lo) * np.array([a, b, c]) for a in g for b in g for c in g]) @ T[:3, :3].T + T[:3, 3]
                if float(needle_seg_dist(np.vstack([pts, samp])).min()) < NEEDLE_CLEAR:
                    self.needle_hit = True
        held = bool(self.S.pins)
        x = self.S.x
        dist = min(float(_box_dist(x, T, lo, hi).min()) for T, lo, hi in boxes)
        prev = self._tcp_prev
        speed = 0.0 if prev is None else float(np.linalg.norm(tcp - prev, axis=1).max() / self.dt)
        self._tcp_prev = tcp.copy()
        clear = (not held) and bool(np.all(ap >= 0.02)) and dist >= PARK_CLEAR and speed < STILL_V
        self.still_t = self.still_t + self.dt if clear else 0.0
        self.last = dict(held=held, hand_dist=dist, speed=speed, aperture=ap.tolist(), cax=[r[:, 0].round(2).tolist() for r in rot])
        self._render(base)

    # -- scoring ------------------------------------------------------------------------------------------------------
    def seam(self):
        st = np.array(self.mach.stitches) if self.mach.stitches else np.zeros((0, 2))
        return st

    def gate(self) -> str:
        if self.S is None:
            return "no fabric"
        if self.S.torn or self.S.bad:
            return "fabric torn"
        if self.needle_hit:
            return "finger within 3 cm of the running needle"
        if self.mach.running:
            return "machine still running"
        if self.last.get("held"):
            return "a gripper holds the fabric"
        if self.still_t < PARK_S - 1e-6:
            return "hands not parked (10 cm clear, open, still 0.5 s)"
        st = self.seam()
        if len(st) < 2:
            return "no seam"
        if self.mach.off_fabric:
            return "needle ran off the fabric"
        adv = np.array(self.mach.stitch_adv[1:])
        uv_step = np.linalg.norm(np.diff(st, axis=0), axis=1)
        if (len(adv) and (adv.min() < ST_MIN or adv.max() > ST_MAX)) or (uv_step.min() < ST_MIN or uv_step.max() > ST_MAX):
            return "stitch length out of 1-4 mm (fabric pushed or pulled)"
        s_all = np.array([M.seam_s(p) for p in st])
        s_end = float(s_all[-1])
        if float(s_all.min()) < -END_TOL or float(s_all.max()) > M.PATH_LEN + END_TOL or s_end < MIN_SEAM_S:
            return "seam ends before 19 cm along the chalked line or >1 cm past start/end mark"
        return ""

    def ladder(self) -> float:
        st = self.seam()
        a = M.allowance(st)
        lo, hi = float(a.min()), float(a.max())
        end_ok = abs(M.seam_s(st[-1]) - M.PATH_LEN) <= END_TOL
        if end_ok and lo >= SA_FULL[0] and hi <= SA_FULL[1]:
            return 1.0
        if end_ok and lo >= SA_MID[0] and hi <= SA_MID[1]:
            return 0.5
        if lo >= SA_PART[0] and hi <= SA_PART[1]:
            return 0.25
        return 0.0

    def score(self) -> float:
        if self.gate():
            return 0.0
        return self.ladder()

    def summary(self) -> str:
        st = self.seam()
        a = M.allowance(st) if len(st) else np.zeros(1)
        adv = np.array(self.mach.stitch_adv[1:]) if len(self.mach.stitch_adv) > 1 else np.zeros(1)
        end = float(np.hypot(*(st[-1] - np.array(M.END_UV)))) if len(st) else -1
        s_end = M.seam_s(st[-1]) if len(st) else 0.0
        return (f"gate={self.gate() or 'ok'} score={self.score()} run={self.mach.running} presses={self.presses} "
                f"depth={self.depth*1000:.1f}mm stitches={len(st)} sa={a.min()*1000:.1f}-{a.max()*1000:.1f}mm "
                f"len={adv.min()*1000:.2f}-{adv.max()*1000:.2f}mm s_end={s_end*100:.1f}cm end_d={end*1000:.1f}mm "
                f"pins={ {k: len(v) for k, v in self.S.pins.items()} } "
                f"w={self.mach.omega:+.2f} strain={self.S.max_strain:.2f} hand_d={self.last.get('hand_dist', 0):.3f} "
                f"still={self.still_t:.2f} needle_hit={self.needle_hit} ap={np.round(self.last.get('aperture', []), 4).tolist()} cax={self.last.get('cax')}")

    def _render(self, base):
        try:
            from fabric_look import render

            self.look = render(self.look, self, self._origin(base))
        except Exception as e:  # noqa: BLE001
            if not getattr(self, "_render_warned", False):
                print(f"[seam] render failed: {e!r}", flush=True)
                self._render_warned = True


# -- embodiment hooks -------------------------------------------------------------------------------------------------
def tracker():
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return None
    return getattr(ie.YamArenaEmbodiment, "_seam_tracker", None)


def _ours(emb) -> bool:
    task = getattr(emb, "task", None)
    return bool(task is not None and getattr(getattr(task, "module", None), "SEAM_TASK", False))


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_seam_hooked", False):
        return
    cls._seam_tracker = Tracker()
    base_reset, base_step = cls.reset, cls._task_step

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        if not _ours(self):
            return obs
        cls._seam_tracker.reset(self._env)
        for _ in range(2):
            self._env.step(self._ready)
            cls._seam_tracker.step(self._env)
        for _ in range(ie.RESET_RENDERS):
            self._base.sim.render()
        return self._observation()

    def _task_step(self, *a, **k):
        if _ours(self):
            cls._seam_tracker.step(self._env)
        return base_step(self, *a, **k)

    cls.reset, cls._task_step, cls._seam_hooked = reset, _task_step, True


_install_hooks()


def score(objects) -> float:
    tr = tracker()
    return tr.score() if tr is not None else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    return 1.0 - score(objects)


def progress(objects) -> dict:
    tr = tracker()
    if tr is None or tr.S is None:
        return {"fabric": 0}
    st = tr.seam()
    s = M.path_s(st[-1]) / M.PATH_LEN if len(st) else 0.0
    return {"started": float(tr.presses >= 1), "seam_progress": float(s), "stopped": float(not tr.mach.running),
            "released_parked": float(tr.still_t >= PARK_S), "needle_safe": float(not tr.needle_hit)}
