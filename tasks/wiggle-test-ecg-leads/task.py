"""On the station table, on the robot's left, stands an ECG lead-wire tester (a light grey block, 24 x 6 x 3 cm,
fixed). In front of it, on the table, are three connector seats (pairs of 4 mm rails 3 cm long), 6 cm apart, and
behind each seat, on the tester's top, a round status lamp (26 mm, green when lit). Three identical ECG lead wires
(grey 5 mm cords, 28 cm) each have a dark connector (30 x 20 x 15 mm, 15 g) resting in its seat and run straight
towards the robot's right, across the table, ending in a black snap clip (20 x 15 x 10 mm) that lies on a brass
shorting bar (20 x 11 cm, 2 mm). A lamp is lit while its lead's connector sits in its seat. One lead, drawn at random
on every reset from the trial seed and not marked in any way, has an intermittent break at its strain relief: its lamp
goes dark while the cord 5-7 cm from its connector is bent by more than 30 degrees. A red reject tray (22 x 28 cm
inside, 15 mm walls) stands on the robot's right. The robot must flex leads near their connectors and watch the lamps
until it knows which lead is faulty (its lamp went dark, or both other leads stayed lit), take that lead out of the
tester by its connector, put it in the tray, keep the good leads connected, and let go. Only the left arm reaches the
connector seats; only the right arm reaches the tray.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Three ECG lead wires are connected to this training lead tester, and one of them has an intermittent "
               "break. Find the faulty lead and put it in the red reject tray.")

# -- geometry (m, world frame: x away from the robot, y to its left) -------------------------------------------------
LEADS = ("a", "b", "c")
SEAT_X = {"a": 0.27, "b": 0.33, "c": 0.39}
SEAT_Y = 0.18  # connector centre
CL, CW, CH = 0.030, 0.020, 0.015  # connector: length along the cord, width, height
SL, SW, SH = 0.020, 0.015, 0.010  # snap clip
R = 0.0025  # 5 mm cord
CORD_LEN = 0.28
EXIT_Y = SEAT_Y - CL / 2  # where the cord leaves the connector
Z0 = TOP + CH / 2 + 0.0005  # connector (and cord) centre height at spawn
GREY_CORD = (0.62, 0.63, 0.64)
DARK = (0.12, 0.12, 0.13)
TESTER_GREY = (0.80, 0.80, 0.78)

TESTER_Y0, TESTER_Y1, TESTER_H = SEAT_Y + CL / 2 + 0.001, SEAT_Y + CL / 2 + 0.061, 0.030
TESTER = Box("tester", pos=(0.33, (TESTER_Y0 + TESTER_Y1) / 2, TOP + TESTER_H / 2),
             size=(0.24, TESTER_Y1 - TESTER_Y0, TESTER_H), color=TESTER_GREY)
LAMP_Y, LAMP_R = (TESTER_Y0 + TESTER_Y1) / 2, 0.013
LAMP_Z = TOP + TESTER_H  # lamps sit on the tester's top
RAIL_H, RAIL_T = 0.003, 0.004
RAIL_OFF = CW / 2 + 0.0015 + RAIL_T / 2


def _seat_parts(k: str) -> list:
    x = SEAT_X[k]
    return [Box(f"rail_{k}_{s}", pos=(x + sg * RAIL_OFF, SEAT_Y, TOP + RAIL_H / 2), size=(RAIL_T, CL, RAIL_H),
                color=DARK) for s, sg in (("n", -1), ("p", 1))] + [
        Cylinder(f"lamp_{k}", pos=(x, LAMP_Y, LAMP_Z + 0.002), radius=LAMP_R, height=0.004, color=(0.18, 0.20, 0.18))]


PLATE_X0, PLATE_X1, PLATE_Y0, PLATE_Y1, PLATE_H = 0.23, 0.43, -0.17, -0.06, 0.002
PLATE = Box("shorting_bar", pos=((PLATE_X0 + PLATE_X1) / 2, (PLATE_Y0 + PLATE_Y1) / 2, TOP + PLATE_H / 2),
            size=(PLATE_X1 - PLATE_X0, PLATE_Y1 - PLATE_Y0, PLATE_H), color=(0.78, 0.62, 0.25))

TRAY_X0, TRAY_X1, TRAY_Y0, TRAY_Y1 = 0.31, 0.53, -0.46, -0.18  # inner footprint (22 x 28 cm)
TRAY_WALL_T, TRAY_WALL_H = 0.004, 0.015
RED = (0.80, 0.10, 0.10)


def _tray() -> list[Box]:
    cx, cy = (TRAY_X0 + TRAY_X1) / 2, (TRAY_Y0 + TRAY_Y1) / 2
    w, d, t, h = TRAY_X1 - TRAY_X0, TRAY_Y1 - TRAY_Y0, TRAY_WALL_T, TRAY_WALL_H
    z = TOP + 0.002 + h / 2
    return [
        Box("tray_floor", pos=(cx, cy, TOP + 0.001), size=(w + 2 * t, d + 2 * t, 0.002), color=RED),
        Box("tray_xn", pos=(TRAY_X0 - t / 2, cy, z), size=(t, d + 2 * t, h), color=RED),
        Box("tray_xp", pos=(TRAY_X1 + t / 2, cy, z), size=(t, d + 2 * t, h), color=RED),
        Box("tray_yn", pos=(cx, TRAY_Y0 - t / 2, z), size=(w, t, h), color=RED),
        Box("tray_yp", pos=(cx, TRAY_Y1 + t / 2, z), size=(w, t, h), color=RED),
    ]


def _lead(k: str):
    return cable(f"lead_{k}", start=(SEAT_X[k], EXIT_Y, Z0), yaw=-math.pi / 2, length=CORD_LEN, radius=R,
                 start_plug=Plug(size=(CL, CW, CH), mass=0.015, color=DARK),
                 end_plug=Plug(size=(SL, SW, SH), mass=0.006, color=(0.05, 0.05, 0.05)),
                 color=GREY_CORD, bend_stiffness=0.01, bend_damping=0.001)


CORDS = {k: _lead(k) for k in LEADS}
OBJECTS = [TESTER, *[p for k in LEADS for p in _seat_parts(k)], PLATE, *_tray(), *CORDS.values()]

# -- lamps: a lit green lens (visual only) above each dark lamp, shown or hidden every control step -----------------
LIT = (0.15, 1.0, 0.25)


def _install_lenses() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_wiggle_lenses", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORDS[LEADS[-1]].name:
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        mat = f"{root}/lens_look"
        look = sim_utils.PreviewSurfaceCfg(diffuse_color=LIT, emissive_color=(0.1, 0.9, 0.2), roughness=0.3)
        look.func(mat, look)
        for k in LEADS:
            path = f"{root}/lens_{k}"
            g = UsdGeom.Cylinder.Define(stage, path)
            g.CreateRadiusAttr(LAMP_R - 0.001)
            g.CreateHeightAttr(0.002)
            g.CreateAxisAttr("Z")
            g.AddTranslateOp().Set(Gf.Vec3d(SEAT_X[k], LAMP_Y, LAMP_Z + 0.0045))
            bind_visual_material(path, mat)
            _LENS[k] = path
        return prim

    spawn._wiggle_lenses = True
    cablekit._spawn_cable = spawn


_LENS: dict[str, str] = {}
_install_lenses()

# -- tester logic and trial history -----------------------------------------------------------------------------------
SEAT_TOL = 0.004  # connector centre within 4 mm of its seat (top view)
SEAT_DZ = 0.005  # ... and within 5 mm of its resting height
SEAT_ANG = math.radians(12)  # ... and its axis within 12 deg of the seat axis
BEND_FAULT = math.radians(30)  # the break opens when the cord 6 cm from the connector is bent more than this
RELEASE_DIST = 0.035
PULL_DIST = 0.01  # a connector leaving its seat by more than 1 cm ...
HAND_DIST = 0.025  # ... must have a gripper TCP within 2.5 cm of its centre (taken out by the connector, not by the cord)
OUT_DIST = 0.03  # "removed": connector >= 3 cm from its seat and snap off the bar


def draw_fault(seed: int | None) -> str:
    """The faulty lead for one trial: uniform over a, b, c from the reset seed (the runner passes one per trial).

    Unseeded resets (the local demo runner) use WIGGLE_SEED if set, else fresh entropy. WIGGLE_FAULT=a|b|c forces the
    fault for scripted test runs only; a policy cannot set it."""
    forced = os.environ.get("WIGGLE_FAULT", "")
    if forced in LEADS:
        return forced
    if seed is None and os.environ.get("WIGGLE_SEED", ""):
        seed = int(os.environ["WIGGLE_SEED"])
    return LEADS[int(np.random.default_rng(seed).integers(len(LEADS)))]


def _ok(v) -> bool:
    return v is not None and bool(np.all(np.isfinite(v)))


def conn_axis(c) -> np.ndarray:
    a = np.asarray(c.start_plug) - c.points[0]
    return a / (np.linalg.norm(a) + 1e-12)


def bend(c) -> float:
    """Angle (rad) between the connector's axis and the cord 5-7 cm from the cord exit."""
    a = conn_axis(c)
    u = c.points[2] - c.points[3]
    u = u / (np.linalg.norm(u) + 1e-12)
    return float(math.acos(float(np.clip(a @ u, -1.0, 1.0))))


def seated(c, k: str) -> bool:
    p = c.start_plug
    if not _ok(p):
        return False
    off = math.hypot(p[0] - SEAT_X[k], p[1] - SEAT_Y)
    ang = math.acos(float(np.clip(conn_axis(c) @ np.array([0.0, 1.0, 0.0]), -1.0, 1.0)))
    return off <= SEAT_TOL and abs(p[2] - Z0) <= SEAT_DZ and ang <= SEAT_ANG


def on_bar(c) -> bool:
    p = c.end_plug
    return _ok(p) and PLATE_X0 <= p[0] <= PLATE_X1 and PLATE_Y0 <= p[1] <= PLATE_Y1 and p[2] < TOP + 0.02


class _Episode:
    def __init__(self, faulty: str):
        self.faulty = faulty  # hidden: drawn per reset, never shown in the scene
        self.good_dark: set[str] = set()  # good leads whose lamp went dark at some step (latched)
        self.tested: set[str] = set()  # leads flexed past the threshold while seated (latched)
        self.max_bend = {k: 0.0 for k in LEADS}
        self.left_seat: set[str] = set()  # leads whose connector has gone more than 1 cm from its seat
        self.cord_pulled: set[str] = set()  # ... with no gripper within 2.5 cm of the connector at that moment
        self.evidence_at_removal: bool | None = None  # was the evidence sufficient when the faulty lead left its seat


_EP = _Episode(draw_fault(None))
_ENV: dict = {"env": None}
_LAMP_SHOWN: dict[str, bool] = {}


def faulty() -> str:
    return _EP.faulty


def reset_episode(seed: int | None = None) -> None:
    global _EP
    _EP = _Episode(draw_fault(seed))


def evidence_sufficient(tested: set[str]) -> bool:
    """The lamps have shown which lead is faulty: the faulty lead was flexed while seated (its lamp went dark), or
    both good leads were flexed while seated and stayed lit (so only the third can be faulty)."""
    return _EP.faulty in tested or set(LEADS) - {_EP.faulty} <= tested


def lamp_lit(objects, k: str) -> bool:
    c = objects[CORDS[k].name]
    if not seated(c, k):
        return False
    return not (k == _EP.faulty and bend(c) > BEND_FAULT)


def _track(objects) -> None:
    for k in LEADS:
        c = objects[CORDS[k].name]
        if not _ok(c.points):
            if k != _EP.faulty:
                _EP.good_dark.add(k)
            continue
        s, b = seated(c, k), bend(c)
        if s:
            _EP.max_bend[k] = max(_EP.max_bend[k], b)
            if b > BEND_FAULT:
                _EP.tested.add(k)
        if k != _EP.faulty and not lamp_lit(objects, k):
            _EP.good_dark.add(k)
        p = c.start_plug
        if k not in _EP.left_seat and _ok(p) and math.hypot(p[0] - SEAT_X[k], p[1] - SEAT_Y) > PULL_DIST:
            _EP.left_seat.add(k)
            if k == _EP.faulty:
                _EP.evidence_at_removal = evidence_sufficient(_EP.tested)
            gs = grippers()
            if gs and min(float(np.linalg.norm(g - p)) for g in gs) > HAND_DIST:
                _EP.cord_pulled.add(k)


def _show_lamps(objects) -> None:
    if not _LENS:
        return
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    stage = get_current_stage()
    for k in LEADS:
        on = lamp_lit(objects, k)
        if _LAMP_SHOWN.get(k) is on:
            continue
        img = UsdGeom.Imageable(stage.GetPrimAtPath(_LENS[k]))
        img.MakeVisible() if on else img.MakeInvisible()
        _LAMP_SHOWN[k] = on


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_wiggle_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            seed = k.get("seed")
            reset_episode(seed)  # draw the fault before the settle steps light the lamps
            out = orig(self, *a, **k)
            reset_episode(seed)  # same seed, same fault; clears anything the settle steps recorded
            print(f"[wiggle] reset seed={seed}", flush=True)
            return out

        reset._wiggle_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_wiggle_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            from yam_arena.scene import object_states

            states = object_states(self._env, [c.name for c in CORDS.values()])
            _track(states)
            _show_lamps(states)
            return base(self, *a, **k)

        _task_step._wiggle_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[np.ndarray]:
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.linalg.norm(b - a) / step))
        out += [a + (b - a) * (i / n) for i in range(1, n + 1)]
    return np.asarray(out)


def _lead_pts(c) -> np.ndarray:
    return np.vstack([_dense(c.polyline())] + [p[None] for p in (c.start_plug, c.end_plug) if p is not None])


def released(objects) -> bool:
    pts = np.vstack([_lead_pts(objects[c.name]) for c in CORDS.values()])
    if not np.isfinite(pts).all():
        return False
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def in_tray(c) -> bool:
    pts = _lead_pts(c)
    m = 0.003
    return bool(np.isfinite(pts).all() and (pts[:, 0] >= TRAY_X0 - m).all() and (pts[:, 0] <= TRAY_X1 + m).all()
                and (pts[:, 1] >= TRAY_Y0 - m).all() and (pts[:, 1] <= TRAY_Y1 + m).all()
                and (pts[:, 2] <= TOP + 0.04).all())


def removed(c, k: str) -> bool:
    p = c.start_plug
    return _ok(p) and math.hypot(p[0] - SEAT_X[k], p[1] - SEAT_Y) >= OUT_DIST and not on_bar(c)


def score(objects) -> float:
    """Score 0 if: a good lead's lamp ever went dark (its connector moved > 4 mm, rose > 5 mm or turned > 12 deg in
    its seat; taking a good lead out does this); any connector left its seat (> 1 cm) with no gripper within 2.5 cm of
    it (pulled out by the cord); or a gripper TCP is within 3.5 cm of any lead at the end.
    "Evidence" = the faulty lead was flexed past 30 deg (cord 5-7 cm from the connector vs the connector's axis) while
    seated, or both good leads were. Otherwise:
    1.0  faulty lead entirely in the tray, and the evidence was complete before its connector left its seat;
    0.5  faulty lead out of the tester (connector >= 3 cm from its seat, snap off the bar) but not entirely in the tray,
         evidence complete before it left its seat;
    0.25 evidence complete, faulty lead never taken out of its seat (diagnosed, not rejected);
    0    anything else (in particular, a faulty lead taken out before the evidence was complete scores 0)."""
    _track(objects)
    if _EP.good_dark or _EP.cord_pulled or not released(objects):
        return 0.0
    k = _EP.faulty
    f = objects[CORDS[k].name]
    if k in _EP.left_seat:
        if not _EP.evidence_at_removal:
            return 0.0
        if in_tray(f):
            return 1.0
        return 0.5 if removed(f, k) else 0.0
    return 0.25 if evidence_sufficient(_EP.tested) else 0.0


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and all(objects[c.name].resting(max_speed=0.02) for c in CORDS.values()))


def distance(objects) -> float:
    if _EP.good_dark or _EP.cord_pulled or (_EP.faulty in _EP.left_seat and not _EP.evidence_at_removal):
        return 1.0
    p = objects[CORDS[_EP.faulty].name].start_plug
    if not _ok(p):
        return 1.0
    c = np.array([(TRAY_X0 + TRAY_X1) / 2, (TRAY_Y0 + TRAY_Y1) / 2])
    return float(np.linalg.norm(p[:2] - c))


def progress(objects) -> dict:
    _track(objects)
    f = objects[CORDS[_EP.faulty].name]
    return {"good_leads_kept": float(not _EP.good_dark), "no_cord_pull": float(not _EP.cord_pulled),
            "leads_tested": len(_EP.tested) / 3.0, "evidence": float(evidence_sufficient(_EP.tested)),
            "evidence_before_removal": float(bool(_EP.evidence_at_removal)),
            "faulty_removed": float(removed(f, _EP.faulty) or in_tray(f)),
            "faulty_in_tray": float(in_tray(f)), "released": float(released(objects))}


TWIN_EXTRA = [
    {"name": f"lens_{k}", "kind": "decal", "size_mm": [24, 24], "colour": "green (LED behind a diffuser, lit while the"
     " lead conducts)", "pose": [SEAT_X[k], LAMP_Y, 0.035]} for k in LEADS
]

VARIATION = {
    "fault": {"lead": list(LEADS), "dist": "uniform, drawn per reset from the trial seed"},
    "cable:lead_a": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},  # connectors sit in fixed seats: no start offset
    "cable:lead_b": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},
    "cable:lead_c": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},
}


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
# Chooses every motion from the fixed layout, the arms' own poses and the camera images (ctx.obs); ctx.states() is
# used only in _log() for the debug log. The faulty lead is unknown to the script.
# Test: the left arm hovers over the lead's connector with its wrist camera on that lead's lamp; the right arm takes
# the cord 20 cm from the connector and pushes it 5 cm towards the tester while lifting 5 cm, so the cord humps and
# bends sharply where it leaves the connector, then lays it back. The lamp is read in the left wrist image (share of
# bright green pixels against the share before the flex). Stop as soon as one lamp went dark, or when two leads stayed
# lit (then the third is the faulty one).
# Reject: the left arm lifts that connector 2 cm up out of its seat, draws it 3 cm back and holds it; the right arm takes the cord
# near the middle of the lead, lifts it, the left lets go, and the right arm carries the lead hanging from that point
# over the tray, waits for the swing to stop, lowers it slowly to the floor and lets go.
FINGER_PINCH = -0.020
GRASP_OPEN = -0.014  # pads ~28 mm apart for the bare cord
FLEX_S = 0.20  # right arm takes the cord this far (m, along it) from the cord exit
FEED, LIFT = 0.05, 0.05
HI = TOP + 0.07
L_PARK = (0.22, 0.26, TOP + 0.12)
R_PARK = (0.22, -0.26, TOP + 0.12)
GRAB_Y = 0.0  # right arm takes the faulty cord here while the left holds its connector up
CONN_LIFT = 0.02
CONN_BACK = 0.03  # ... and draws it 3 cm out of the seat towards the robot before the right arm takes the cord
CORD_Z = TOP + R  # centre of a cord lying on the table
_TRAY_MID = ((TRAY_X0 + TRAY_X1) / 2, (TRAY_Y0 + TRAY_Y1) / 2)
CARRY_Z = TOP + 0.17  # carry height: each half of the lead hangs about 16 cm, so it just clears the table
VIA = (0.33, -0.10)  # carry route, reachable at CARRY_Z
TRAY_DROP = (0.42, -0.325)  # over the tray's middle: lower slowly to the floor, let go
CORD_YAW = -math.pi / 2  # gripper yaw across a cord running along y (fingers close across it)


def _log(ctx, tag=""):
    """Debug log only (sim state); never used to choose a motion."""
    st = ctx.states()
    info = []
    for k in LEADS:
        c = st[CORDS[k].name]
        p = c.start_plug
        info.append(f"{k}:off={1000 * math.hypot(p[0] - SEAT_X[k], p[1] - SEAT_Y):.1f} "
                    f"bend={math.degrees(bend(c)):.0f} lit={int(lamp_lit(st, k))} bar={int(on_bar(c))} "
                    f"tray={int(in_tray(c))}")
    print(f"[t={ctx.t:6.2f}] {tag:10s} {' | '.join(info)} fault={_EP.faulty} dark={sorted(_EP.good_dark)} "
          f"tested={sorted(_EP.tested)} pulled={sorted(_EP.cord_pulled)} ev_rm={_EP.evidence_at_removal} "
          f"rel={int(released(st))} score={score(st)}", flush=True)
    q = _lead_pts(st[CORDS[_EP.faulty].name])
    print(f"           faulty lead x {q[:, 0].min():.3f}..{q[:, 0].max():.3f} y {q[:, 1].min():.3f}..{q[:, 1].max():.3f} "
          f"z<= {q[:, 2].max() - TOP:.3f} (tray x {TRAY_X0}..{TRAY_X1} y {TRAY_Y0}..{TRAY_Y1})", flush=True)


def green_share(img) -> float:
    """Share of bright green (lit lamp) pixels in the centre of an RGB image (the watched lamp; neighbours lie at the
    image edges)."""
    im = np.asarray(img, dtype=np.int16)
    h, w = im.shape[:2]
    im = im[int(0.1 * h):int(0.9 * h), int(0.3 * w):int(0.7 * w)]
    r, g, b = im[..., 0], im[..., 1], im[..., 2]
    return float(((g > 110) & (g > r + 50) & (g > b + 40)).mean())


def _lamp(ctx) -> float:
    return green_share(ctx.obs.images["left_cam"])


def _watch(L, k):
    """Left arm hovers over lead k's connector, wrist camera on its lamp."""
    yield from L.move((SEAT_X[k], SEAT_Y, TOP + 0.075), yaw=math.pi / 2, finger=FINGER_PINCH, duration=1.2)
    yield from L.wait(0.3)


def _flex(Rt, ctx, k):
    """Right arm flexes lead k at its strain relief; returns True if the lamp in the left wrist view went dark."""
    from yam_arena.constants import FINGER_CLOSED

    base = _lamp(ctx)
    g = np.array([SEAT_X[k], EXIT_Y - FLEX_S, CORD_Z + 0.010])
    yield from Rt.move((g[0], g[1], HI), yaw=CORD_YAW, finger=GRASP_OPEN, duration=1.2)
    yield from Rt.move(g, yaw=CORD_YAW, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    low = base
    for f in np.linspace(0.0, 1.0, 7)[1:]:
        yield from Rt.move((g[0], g[1] + f * FEED, g[2] + f * LIFT), yaw=CORD_YAW, duration=0.4)
        low = min(low, _lamp(ctx))
    for _ in range(3):
        yield from Rt.wait(0.2)
        low = min(low, _lamp(ctx))
    dark = base > 0.01 and low < 0.3 * base
    print(f"[demo] flex {k}: green before={base:.4f} lowest while flexed={low:.4f} -> {'DARK' if dark else 'lit'}",
          flush=True)
    _log(ctx, f"flex_{k}")
    yield from Rt.move(g + np.array([0.0, 0.0, 0.002]), yaw=CORD_YAW, duration=1.2)
    yield from Rt.grip(GRASP_OPEN, 0.4)
    yield from Rt.move((g[0], g[1], HI), yaw=CORD_YAW, duration=0.7)
    return dark


def _diagnose(L, Rt, ctx, order=LEADS, stop_early=True):
    """Flex leads in turn; return the faulty lead the lamps point to (None if they never do)."""
    clean, dark = [], []
    for k in order:
        yield from _watch(L, k)
        if (yield from _flex(Rt, ctx, k)):
            print(f"[demo] lamp {k} went dark while flexed -> {k} is faulty", flush=True)
            dark.append(k)
            if stop_early:
                return k
        else:
            clean.append(k)
        if stop_early and len(clean) == 2:
            rest = [x for x in LEADS if x not in clean][0]
            print(f"[demo] leads {clean} stayed lit -> {rest} is faulty (untested)", flush=True)
            return rest
    return dark[0] if dark else None


def _reject(L, Rt, ctx, k, dest=None, *, z_lo=TOP + 0.02, release=True):
    """Left lifts lead k's connector out of its seat; right takes the cord near the lead's middle, carries the lead
    hanging from that point and lowers it slowly into the tray (or at ``dest``)."""
    from yam_arena.constants import FINGER_CLOSED

    x = SEAT_X[k]
    gz = TOP + CH + 0.003
    yield from L.move((x, SEAT_Y, HI), yaw=math.pi / 2, finger=FINGER_PINCH, duration=1.0)
    yield from L.move((x, SEAT_Y, gz), yaw=math.pi / 2, duration=0.9)
    yield from L.grip(FINGER_CLOSED, 0.6)
    yield from L.move((x, SEAT_Y, gz + CONN_LIFT), yaw=math.pi / 2, duration=1.0)
    yield from L.move((x, SEAT_Y - CONN_BACK, gz + CONN_LIFT), yaw=math.pi / 2, duration=0.8)
    _log(ctx, "lifted")
    g = (x, GRAB_Y, CORD_Z + 0.010)
    yield from Rt.move((x, GRAB_Y, HI), yaw=CORD_YAW, finger=GRASP_OPEN, duration=1.5)
    yield from Rt.move(g, yaw=CORD_YAW, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((x, GRAB_Y - 0.02, TOP + 0.06), yaw=CORD_YAW, duration=1.0)
    yield from L.grip(FINGER_PINCH, 0.5)
    yield from L.move((x, SEAT_Y - CONN_BACK, HI + 0.03), yaw=math.pi / 2, duration=0.6)
    yield from L.move(L_PARK, yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
    yield from Rt.move((x, GRAB_Y - 0.05, TOP + 0.11), yaw=CORD_YAW, duration=1.2)
    a = TRAY_DROP if dest is None else dest
    yield from Rt.move((VIA[0], VIA[1], CARRY_Z), yaw=CORD_YAW, duration=1.2)
    yield from Rt.move((a[0], a[1], CARRY_Z), yaw=CORD_YAW, duration=1.8)
    yield from Rt.wait(1.2)  # let the hanging lead stop swinging
    yield from Rt.move((a[0], a[1], z_lo), yaw=CORD_YAW, duration=3.0)
    end = a
    if dest is None:  # drag 5 cm sideways along the floor so the folded lead lies down across the tray, not along it
        end = (a[0] - 0.05, a[1])
        yield from Rt.move((end[0], end[1], z_lo), yaw=CORD_YAW, duration=1.2)
    yield from Rt.wait(0.5)
    _log(ctx, "lowered")
    if not release:  # "hold" shortcut: lead lowered into the tray but never let go
        yield from Rt.move((end[0], end[1], TOP + 0.02), yaw=CORD_YAW, duration=1.0)
        return
    yield from Rt.grip(GRASP_OPEN, 0.4)
    yield from Rt.move((end[0], end[1], TOP + 0.12), yaw=CORD_YAW, duration=0.8)


def _end(L, Rt, ctx):
    from yam_arena.constants import FINGER_CLOSED

    yield from Rt.move(R_PARK, yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
    for _ in range(3):
        yield from L.wait(0.5)
    _log(ctx, "end")


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED

    # test only: correct | none | alwaysa | randompick | flexalla | flexonly | cordpull | hold | tier05
    mode = os.environ.get("WIGGLE_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "none":
        for _ in range(4):
            yield from L.wait(1.0)
        _log(ctx, "end")
        return
    if mode in ("alwaysa", "randompick"):  # no test at all: reject a fixed lead / a random lead
        k = "a" if mode == "alwaysa" else LEADS[int.from_bytes(os.urandom(2), "little") % 3]
        print(f"[demo] {mode}: rejecting {k} without testing", flush=True)
        yield from _reject(L, Rt, ctx, k)
        yield from _end(L, Rt, ctx)
        return
    if mode in ("flexalla", "flexonly"):  # test all three, then ignore the lamps
        found = yield from _diagnose(L, Rt, ctx, stop_early=False)
        print(f"[demo] {mode}: lamps said {found}", flush=True)
        yield from L.move(L_PARK, yaw=0.0, duration=1.2)
        if mode == "flexalla":
            yield from _reject(L, Rt, ctx, "a")
        yield from _end(L, Rt, ctx)
        return
    k = yield from _diagnose(L, Rt, ctx)
    if k is None:
        print("[demo] no lamp went dark; nothing rejected", flush=True)
        yield from L.move(L_PARK, yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
        yield from _end(L, Rt, ctx)
        return
    yield from L.move(L_PARK, yaw=0.0, duration=1.2)
    if mode == "cordpull":  # right arm alone pulls the lead out by its cord as close to the connector as it reaches
        x = SEAT_X[k]
        yield from Rt.move((x, 0.13, HI), yaw=CORD_YAW, finger=GRASP_OPEN, duration=1.2)
        yield from Rt.move((x, 0.13, CORD_Z + 0.010), yaw=CORD_YAW, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED, 0.5)
        yield from Rt.move((x, 0.03, Z0 + 0.03), yaw=CORD_YAW, duration=2.5)
        yield from Rt.move((x, -0.03, TOP + 0.15), yaw=CORD_YAW, duration=1.5)
        yield from Rt.move((_TRAY_MID[0], _TRAY_MID[1], TOP + 0.15), yaw=CORD_YAW, duration=2.0)
        yield from Rt.move((_TRAY_MID[0], _TRAY_MID[1], TOP + 0.05), yaw=CORD_YAW, duration=1.5)
        yield from Rt.grip(GRASP_OPEN, 0.4)
        yield from Rt.move((_TRAY_MID[0], _TRAY_MID[1], TOP + 0.12), yaw=CORD_YAW, duration=0.8)
        yield from _end(L, Rt, ctx)
        return
    if mode == "hold":
        yield from _reject(L, Rt, ctx, k, release=False)
        for _ in range(4):
            yield from Rt.wait(1.0)
        _log(ctx, "end")
        return
    if mode == "tier05":  # out of the tester, laid on the table beside the tray
        yield from _reject(L, Rt, ctx, k, dest=(0.22, -0.20), z_lo=TOP + 0.03)
    else:
        yield from _reject(L, Rt, ctx, k)
    yield from _end(L, Rt, ctx)
