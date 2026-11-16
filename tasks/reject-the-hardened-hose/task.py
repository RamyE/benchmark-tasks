"""A hose-inspection bench on the station table. Three identical black 10 mm hydraulic hoses (60 cm between their
fittings, 45 g) lie straight and parallel across the robot's midline, along y, at x 0.28, 0.35 and 0.42 m, each with
a brass end fitting (30 x 14 x 14 mm, 8 g) on both ends (fitting centres at y +-0.315). Two hoses are supple; one has
hardened: its bending stiffness is five times theirs. Which one is drawn afresh at every reset (uniform over the three
positions, from the trial seed), and nothing marks it: at rest all three look the same. A red reject tray (inside
10 x 70 cm, 2 cm walls) lies along y nearest the robot (inside x 0.115-0.215).

Inspection practice is to flex a hose and judge its stiffness. A hose counts as flexed when a gripper holds one of its
fittings at least 6 cm above the table and at least 10 cm in from that fitting's rest place towards the hose's other
end (an end-on push that loads the hose in bending), while two parts of the hose point at least 30 degrees apart; it
counts as tested once it has been flexed and then let go (no gripper within 3.5 cm of it). Flexed by lifting one
fitting 10 cm and drawing it 14 cm towards the other end, a supple hose folds over, comes down to the table about 6 cm
from the raised fitting and its far fitting slides about 3 cm; the hardened hose makes a ramp about 16 cm long and
pushes its far fitting about 10 cm along the table. The robot must test every hose, then lay the hardened one in the
tray with both arms (each fitting, and each end of the tray, is reachable only by the arm on its side), keep the good
hoses out of the tray, and let go.

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
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("One of these hydraulic hoses has hardened and must be taken out of service. Put it in the red tray "
               "and leave the good ones on the bench, then let go.")

# -- geometry (m, world frame: x away from the robot, y to its left) -------------------------------------------------
HOSES_K = ("a", "b", "c")
HOSE_X = {"a": 0.28, "b": 0.35, "c": 0.42}
R = 0.005  # 10 mm hose
L = 0.60  # hose length between the fittings
HOSE_MASS = 0.045
FL, FW, FH = 0.030, 0.014, 0.014  # fitting: length along the hose, width, height
FIT_MASS = 0.008
K_GOOD, K_HARD = 0.006, 0.03  # bending stiffness, N m/rad per joint (30 segments of 2 cm)
ZC = TOP + FH / 2 + 0.0005
BLACK = (0.06, 0.06, 0.06)
BRASS = (0.78, 0.60, 0.24)
RED = (0.80, 0.10, 0.10)

TRAY_X0, TRAY_X1, TRAY_Y0, TRAY_Y1 = 0.115, 0.215, -0.35, 0.35  # inner footprint
TRAY_T, TRAY_H = 0.004, 0.020
TRAY_CX = (TRAY_X0 + TRAY_X1) / 2


def _tray() -> list[Box]:
    cy = (TRAY_Y0 + TRAY_Y1) / 2
    w, d, t, h = TRAY_X1 - TRAY_X0, TRAY_Y1 - TRAY_Y0, TRAY_T, TRAY_H
    z = TOP + 0.002 + h / 2
    return [
        Box("tray_floor", pos=(TRAY_CX, cy, TOP + 0.001), size=(w + 2 * t, d + 2 * t, 0.002), color=RED),
        Box("tray_xn", pos=(TRAY_X0 - t / 2, cy, z), size=(t, d + 2 * t, h), color=RED),
        Box("tray_xp", pos=(TRAY_X1 + t / 2, cy, z), size=(t, d + 2 * t, h), color=RED),
        Box("tray_yn", pos=(TRAY_CX, TRAY_Y0 - t / 2, z), size=(w, t, h), color=RED),
        Box("tray_yp", pos=(TRAY_CX, TRAY_Y1 + t / 2, z), size=(w, t, h), color=RED),
    ]


def _hose(k: str):
    return cable(f"hose_{k}", start=(HOSE_X[k], -L / 2, ZC), yaw=math.pi / 2, length=L, radius=R, cable_mass=HOSE_MASS,
                 start_plug=Plug(size=(FL, FW, FH), mass=FIT_MASS, color=BRASS),
                 end_plug=Plug(size=(FL, FW, FH), mass=FIT_MASS, color=BRASS),
                 bend_stiffness=K_GOOD, bend_damping=0.001, color=BLACK)  # the hardened one is set at reset


HOSES = {k: _hose(k) for k in HOSES_K}
OBJECTS = [*_tray(), *HOSES.values()]
# rest places of the fittings (start = right end, y -0.315; end = left end, y +0.315); the layout is not jittered
REST = {k: {"start": np.array([HOSE_X[k], -(L / 2 + FL / 2)]), "end": np.array([HOSE_X[k], L / 2 + FL / 2])}
        for k in HOSES_K}


def draw_hard(seed: int | None, n_reset: int = 0) -> str:
    """The hardened hose for one trial: uniform over a, b, c from the trial seed the runner passes to reset().

    Unseeded resets use HOSE_SEED + the reset count if HOSE_SEED is set, else fresh entropy. HOSE_HARD=a|b|c forces
    the hardened hose for scripted test runs only (a policy cannot set it)."""
    forced = os.environ.get("HOSE_HARD", "")
    if forced in HOSES_K:
        return forced
    if seed is None and os.environ.get("HOSE_SEED", ""):
        seed = int(os.environ["HOSE_SEED"]) + n_reset
    rng = np.random.default_rng(None if seed is None else [int(seed), 4057])
    return HOSES_K[int(rng.integers(len(HOSES_K)))]

# -- inspection history and scoring -----------------------------------------------------------------------------------
HELD_DIST = 0.03  # gripper TCP within 3 cm of a fitting centre = holding it
FLEX_LIFT = 0.06  # held fitting at least 6 cm above the table ...
FLEX_DRAW = 0.10  # ... and at least 10 cm in from its rest place towards the hose's other end (end-on push) ...
FLEX_BEND = math.radians(30)  # ... while two parts of the hose point at least 30 deg apart (it is bent)
ENTER_Z = TOP + 0.06  # a hose point over the tray's footprint and below this has entered the tray
IN_Z = TOP + 0.03  # "in the tray": below this (wall top 2.2 cm)
MARGIN = 0.003
RELEASE_DIST = 0.035
FLOOR_Z = TOP - 0.05  # any hose point below this has fallen off the table


def _ok(v) -> bool:
    return v is not None and bool(np.all(np.isfinite(v)))


class _Episode:
    def __init__(self, hard: str):
        self.hard = hard  # the hardened hose this trial (hidden from the policy)
        self.flexed: set[str] = set()  # flexed at some step (latched)
        self.tested: set[str] = set()  # flexed and afterwards let go (no gripper within 3.5 cm of it) (latched)
        self.hard_entered = False  # hardened hose has entered the tray at some step
        self.untested_entry = False  # ... and at that first step the tests had not identified it (latched)
        self.fell = False  # a hose went off the table (latched)
        self.broken = False  # non-finite hose state (latched)


_ENV: dict = {"env": None, "resets": 0}
_EP = _Episode(draw_hard(None))


def reset_episode(seed: int | None = None) -> None:
    global _EP
    _EP = _Episode(draw_hard(seed, _ENV["resets"]))


def apply_stiffness(env) -> None:
    """Write each hose's bending stiffness into the sim: K_HARD for this trial's hardened hose, K_GOOD for the rest."""
    base = env.unwrapped
    for k in HOSES_K:
        base.scene[HOSES[k].name].write_joint_stiffness_to_sim_index(stiffness=float(K_HARD if k == _EP.hard
                                                                                    else K_GOOD))


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


def hose_pts(c) -> np.ndarray:
    return np.vstack([_dense(c.polyline())] + [p[None] for p in (c.start_plug, c.end_plug) if p is not None])


def _over_tray(pts: np.ndarray, zmax: float) -> np.ndarray:
    return ((pts[:, 0] >= TRAY_X0 - MARGIN) & (pts[:, 0] <= TRAY_X1 + MARGIN) & (pts[:, 1] >= TRAY_Y0 - MARGIN)
            & (pts[:, 1] <= TRAY_Y1 + MARGIN) & (pts[:, 2] <= zmax))


def _inside(xy: np.ndarray, inset: float) -> bool:
    """Every xy point at least ``inset`` inside the tray's inner outline."""
    return bool(((xy[:, 0] >= TRAY_X0 + inset) & (xy[:, 0] <= TRAY_X1 - inset) & (xy[:, 1] >= TRAY_Y0 + inset)
                 & (xy[:, 1] <= TRAY_Y1 - inset)).all())


def fitting_corners(p, d) -> np.ndarray:
    """xy corners of a fitting's footprint: centre ``p``, long axis along the hose end direction ``d``."""
    u = np.asarray(d[:2], dtype=float)
    n = float(np.linalg.norm(u))
    if n < 0.2:  # fitting nearly upright: its footprint is about FW x FH
        u, half = np.array([1.0, 0.0]), FH / 2
    else:
        u, half = u / n, FL / 2
    v = np.array([-u[1], u[0]])
    return np.array([p[:2] + a * half * u + b * FW / 2 * v for a in (-1, 1) for b in (-1, 1)])


def fittings_in_tray(c) -> bool:
    """Both whole fittings (their footprints) inside the tray's inner outline, centres below 3 cm."""
    if c.start_plug is None or c.end_plug is None:
        return False
    if max(c.start_plug[2], c.end_plug[2]) > IN_Z:
        return False
    return _inside(np.vstack([fitting_corners(c.start_plug, c.start_dir), fitting_corners(c.end_plug, c.end_dir)]),
                   0.0)


def in_tray(c) -> bool:
    """The whole hose inside the tray: its centreline at least its radius inside the inner outline and below 3 cm,
    and both whole fittings inside (fittings_in_tray)."""
    pts = _dense(c.polyline())
    return bool((pts[:, 2] <= IN_Z).all() and _inside(pts[:, :2], R) and fittings_in_tray(c))


def good_in_tray(objects) -> list[str]:
    """Good hoses with any point over the tray's inner outline (3 mm margin) below 6 cm."""
    return [k for k in HOSES_K if k != _EP.hard and _over_tray(hose_pts(objects[HOSES[k].name]), ENTER_Z).any()]


def bend(c) -> float:
    """Largest angle (rad) between the directions of any two parts of the hose (tip-to-centre and centre-to-centre
    pieces of its centreline): 0 for a straight hose."""
    d = np.diff(c.polyline(), axis=0)
    d = d / (np.linalg.norm(d, axis=1, keepdims=True) + 1e-12)
    return float(np.arccos(np.clip((d @ d.T).min(), -1.0, 1.0)))


def drawn_in(k: str, end: str, p) -> float:
    """How far (m) fitting ``end`` of hose k is, horizontally, from its rest place towards the other fitting's."""
    r0, r1 = REST[k][end], REST[k]["end" if end == "start" else "start"]
    u = (r1 - r0) / np.linalg.norm(r1 - r0)
    return float((np.asarray(p[:2]) - r0) @ u)


def flexed_now(k: str, c, gs) -> bool:
    if not gs or bend(c) < FLEX_BEND:
        return False
    return any(p[2] >= TOP + FLEX_LIFT and drawn_in(k, end, p) >= FLEX_DRAW
               and min(float(np.linalg.norm(g - p)) for g in gs) <= HELD_DIST
               for end, p in (("start", c.start_plug), ("end", c.end_plug)))


def _track(objects) -> None:
    gs = grippers()
    for k in HOSES_K:
        c = objects[HOSES[k].name]
        pts = hose_pts(c)
        if not np.isfinite(pts).all():
            _EP.broken = True
            continue
        if (pts[:, 2] < FLOOR_Z).any():
            _EP.fell = True
        if flexed_now(k, c, gs):
            _EP.flexed.add(k)
        elif k in _EP.flexed and k not in _EP.tested and gs and all(
                float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in gs):
            _EP.tested.add(k)
        if k == _EP.hard and not _EP.hard_entered and _over_tray(pts, ENTER_Z).any():
            _EP.hard_entered = True
            _EP.untested_entry = not identified()


def identified() -> bool:
    """The tests so far identify the hardened hose: it was tested itself, or both good hoses were (elimination)."""
    good = {k for k in HOSES_K if k != _EP.hard}
    return _EP.hard in _EP.tested or good <= _EP.tested


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_hose_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            seed = k.get("seed")
            out = orig(self, *a, **k)
            reset_episode(seed)
            _ENV["resets"] += 1
            _ENV["env"] = self._env
            apply_stiffness(self._env)  # the hoses lie straight at rest, so the new stiffness does not move them
            print(f"[hose] reset seed={seed}", flush=True)
            return out

        reset._hose_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_hose_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            from yam_arena.scene import object_states

            _track(object_states(self._env, [c.name for c in HOSES.values()]))
            return base(self, *a, **k)

        _task_step._hose_hooked = True
        cls._task_step = _task_step


_install_hooks()


def released(objects) -> bool:
    pts = np.vstack([hose_pts(objects[c.name]) for c in HOSES.values()])
    if not np.isfinite(pts).all():
        return False
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """Tested = flexed (held by a fitting at least 6 cm above the table and at least 10 cm in from its rest place
    towards the other end, while two parts of it pointed at least 30 deg apart) and afterwards let go (no gripper TCP
    within 3.5 cm of it). Score 0 if: at the end a good hose has any point over the tray's inner outline below 6 cm;
    when the hardened hose first entered the tray (any point over the outline below 6 cm) neither it nor both good
    hoses had been tested; a hose went off the table; or a gripper TCP is within 3.5 cm of any hose at the end.
    Otherwise: the whole hardened hose inside the tray (in_tray) -> 1.0; both whole fittings inside, the hose partly
    outside -> 0.5; else 0."""
    _track(objects)
    if (_EP.broken or _EP.fell or good_in_tray(objects) or _EP.untested_entry or not _EP.hard_entered
            or not released(objects)):
        return 0.0
    h = objects[HOSES[_EP.hard].name]
    if in_tray(h):
        return 1.0
    return 0.5 if fittings_in_tray(h) else 0.0


CLEAR_DIST = 0.10  # success also needs both gripper centres this far from the hardened hose (arms clear of the tray)


def success(objects) -> bool:
    """Full score, every hose at rest, and both grippers at least 10 cm from the hardened hose, so the episode ends
    on a view of the tray with the whole hose in it."""
    if not (score(objects) >= 1.0 and all(objects[c.name].resting(max_speed=0.02) for c in HOSES.values())):
        return False
    pts = hose_pts(objects[HOSES[_EP.hard].name])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= CLEAR_DIST for g in grippers())


def distance(objects) -> float:
    h = objects[HOSES[_EP.hard].name]
    if not _ok(h.points):
        return 1.0
    return float(np.abs(h.points[:, 0] - TRAY_CX).mean())


def progress(objects) -> dict:
    _track(objects)
    h = objects[HOSES[_EP.hard].name]
    return {"flexed": len(_EP.flexed) / 3.0, "tested": len(_EP.tested) / 3.0, "good_kept_out": float(not good_in_tray(objects)),
            "identified_before_tray": float(_EP.hard_entered and not _EP.untested_entry),
            "fittings_in_tray": float(fittings_in_tray(h)), "hose_in_tray": float(in_tray(h)),
            "released": float(released(objects))}


VARIATION = {
    "hardened": {"hose": list(HOSES_K), "dist": "uniform, drawn per reset from the trial seed"},
    **{f"cable:hose_{k}": {"dx": 0.0, "dy": 0.0, "yaw": 0.0} for k in HOSES_K},  # hoses on fixed tape lines
}
