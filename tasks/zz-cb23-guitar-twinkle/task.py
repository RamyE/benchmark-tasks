"""Play the first verse of Twinkle Twinkle Little Star on a classical guitar: one hand frets, the other plucks.

Scene (metres from the arm-base midpoint, x forward, y left, z up from the table top). A Yamaha C40 classical guitar
(650 mm scale, nylon strings, standard tuning E A D G B E) stands on its side as it is held to play, in two foam
cradle blocks taped to the table: soundboard vertical and facing the robot (front face at x = 0.400), neck along y with
the nut at y = -0.220 (headstock to the robot's right), the body's centre line 0.197 above the table. So the strings
run along +y from the nut to the saddle at y = +0.430; the low E string is the top one, the high E the bottom one.

Notes of the verse (C major): C4 = B string fret 1, D4 = B fret 3, E4 = high E open, F4 = E fret 1, G4 = E fret 3,
A4 = E fret 5.  C C G G A A G (half) | F F E E D D C (half).

Model. Each of the B and high E strings is split, between the nut and the 7th fret, into one rigid segment per fret
space on a sprung slider toward the fingerboard (stiffness = string tension x (1/a + 1/b), a, b = distances to nut and
saddle; travel = the string's height over the fret crowns there). Pressed to the end of its travel = the string is
fretted in that space. Every string also has a 30 mm segment over the soundhole (480 mm from the nut) on a sprung
slider toward the soundboard: a pluck is that segment pushed >= 2.0 mm and let go so it springs back below 0.5 mm within
0.1 s. The pitch of a pluck is the open string raised by the highest fretted space on that string at that moment.

Run it:
  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 900 --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Play the first verse of Twinkle Twinkle Little Star on the guitar (C C G G A A G, F F E E D D C) in a "
               "steady beat, then move both hands away from the guitar.")
RUN_LIMIT_STEPS = 900

# -- guitar geometry (Yamaha C40: 650 mm scale, 52 mm nut, body 485 mm long, lower bout 370 mm, upper bout 280 mm) ----
SCALE = 0.650
XS = 0.400                    # soundboard front face
BODY_D = 0.100                # body depth (soundboard to back)
FB_PROUD = 0.006              # fingerboard face this far in front of the soundboard
X_FB = XS - FB_PROUD          # fingerboard face
FRET_H = 0.0012               # fret crown above the fingerboard
Y_NUT = -0.220
Y_SADDLE = Y_NUT + SCALE
ZC = 0.197                    # body / neck centre line above the table
SP_NUT, SP_SADDLE = 0.0086, 0.0116    # string spacing at nut and saddle (52 mm nut, 58 mm bridge spacing)
ACT_NUT, ACT_12 = 0.0005, 0.0030      # string bottom over the fret crowns at the nut and at the 12th fret
STR_T = 0.0010                # string collision/visual thickness (real nylon B 0.82 mm, high E 0.71 mm)
TENSION = [72.0, 69.0, 63.0, 51.0, 54.0, 72.0]   # N, normal-tension nylon set, low E .. high E (typ. 6.5-7.4 kg)
OPEN_MIDI = [40, 45, 50, 55, 59, 64]             # E2 A2 D3 G3 B3 E4
NAMES = ["E6", "A5", "D4", "G3", "B2", "E1"]      # string names (number = string number)
FRETTED = [3, 4, 5]           # G, B and high E strings have per-fret segments
N_FRETS = 7
PLUCK_U, PLUCK_L = 0.480, 0.030
PLUCK_TRAVEL = 0.0045
SEG_M = 0.002                 # kg per string segment
SEG_C = 1.5                   # N s/m slider damping


def fret_u(n: int) -> float:
    return SCALE * (1.0 - 2.0 ** (-n / 12.0))


def spacing(u: float) -> float:
    return SP_NUT + (SP_SADDLE - SP_NUT) * u / SCALE


def action(u: float) -> float:
    return ACT_NUT + (ACT_12 - ACT_NUT) * u / fret_u(12)


def string_z(i: int, u: float) -> float:
    return ZC + (2.5 - i) * spacing(u)


def string_x(u: float) -> float:
    """String centre x (its distance in front of the fingerboard face) at u metres from the nut."""
    return X_FB - FRET_H - action(u) - STR_T / 2


def stiffness(i: int, u: float) -> float:
    return TENSION[i] * (1.0 / u + 1.0 / (SCALE - u))


WOOD_TOP = (0.86, 0.70, 0.46)
WOOD_SIDE = (0.36, 0.19, 0.10)
FB_COL = (0.10, 0.07, 0.05)
NYLON = (0.93, 0.92, 0.86)
BASS = (0.80, 0.72, 0.55)
FRET_COL = (0.78, 0.78, 0.74)
HOLE = (0.03, 0.02, 0.02)
FOAM = (0.10, 0.10, 0.11)
BONE = (0.95, 0.94, 0.88)

UB_R, LB_R = 0.140, 0.185
Y_JOIN = Y_NUT + fret_u(12)            # neck meets the body at the 12th fret
Y_TAIL = Y_JOIN + 0.485
Y_UB = Y_JOIN + UB_R
Y_LB = Y_TAIL - LB_R
XB = XS + BODY_D / 2

FIXED: list = []
FIXED.append(Cylinder("body_upper", pos=(XB, Y_UB, TOP + ZC), radius=UB_R, height=BODY_D, color=WOOD_SIDE))
FIXED.append(Cylinder("body_lower", pos=(XB, Y_LB, TOP + ZC), radius=LB_R, height=BODY_D, color=WOOD_SIDE))
FIXED.append(Cylinder("top_upper", pos=(XS - 0.0005, Y_UB, TOP + ZC), radius=UB_R - 0.003, height=0.002,
                      color=WOOD_TOP))
FIXED.append(Cylinder("top_lower", pos=(XS - 0.0005, Y_LB, TOP + ZC), radius=LB_R - 0.003, height=0.002,
                      color=WOOD_TOP))
SOUNDHOLE_Y = Y_JOIN + 0.130
FIXED.append(Cylinder("rosette", pos=(XS - 0.0012, SOUNDHOLE_Y, TOP + ZC), radius=0.050, height=0.0006,
                      color=(0.55, 0.35, 0.18)))
FIXED.append(Cylinder("soundhole", pos=(XS - 0.0016, SOUNDHOLE_Y, TOP + ZC), radius=0.043, height=0.0006, color=HOLE))
FIXED.append(Box("bridge", pos=(XS - 0.004, Y_SADDLE + 0.008, TOP + ZC), size=(0.008, 0.028, 0.185), color=WOOD_SIDE))
FIXED.append(Box("saddle", pos=(XS - 0.0085, Y_SADDLE, TOP + ZC), size=(0.004, 0.003, 0.075), color=BONE))
_FB_END = Y_NUT + fret_u(19)
FIXED.append(Box("fingerboard", pos=(X_FB + 0.003, (Y_NUT + _FB_END) / 2, TOP + ZC),
                 size=(0.006, _FB_END - Y_NUT, 0.056), color=FB_COL))
FIXED.append(Box("neck", pos=(X_FB + 0.006 + 0.012, (Y_NUT + Y_JOIN) / 2, TOP + ZC),
                 size=(0.024, Y_JOIN - Y_NUT, 0.052), color=WOOD_SIDE))
FIXED.append(Box("headstock", pos=(X_FB + 0.012, Y_NUT - 0.005 - 0.095, TOP + ZC),
                 size=(0.020, 0.190, 0.058), color=WOOD_SIDE))
FIXED.append(Box("nut", pos=(X_FB - 0.002, Y_NUT - 0.0015, TOP + ZC), size=(0.004, 0.003, 0.052), color=BONE))
for k in range(3):
    for s in (-1, 1):
        FIXED.append(Cylinder(f"tuner_{k}_{'t' if s > 0 else 'b'}", pos=(X_FB + 0.012, Y_NUT - 0.045 - 0.05 * k,
                              TOP + ZC + s * 0.040), radius=0.008, height=0.016, color=BONE))
for n in range(1, 13):
    FIXED.append(Box(f"fret_{n}", pos=(X_FB - FRET_H / 2, Y_NUT + fret_u(n), TOP + ZC),
                     size=(FRET_H, 0.002, 0.054), color=FRET_COL))
for n in (5, 7, 9):   # side dots, as on the C40's neck edge
    yy = Y_NUT + (fret_u(n - 1) + fret_u(n)) / 2
    FIXED.append(Box(f"dot_{n}", pos=(X_FB + 0.002, yy, TOP + ZC + 0.0285), size=(0.003, 0.003, 0.001),
                     color=(0.95, 0.95, 0.95)))
# foam cradle blocks (black closed-cell EVA): one under the lower bout, one under the neck at the 9th-10th frets
FIXED.append(Box("foam_body", pos=(XB, Y_LB, TOP + 0.006), size=(0.140, 0.080, 0.012), color=FOAM))
_NECK_LO = ZC - 0.026
FIXED.append(Box("foam_neck", pos=(X_FB + 0.015, Y_NUT + 0.300, TOP + _NECK_LO / 2), size=(0.060, 0.050, _NECK_LO),
                 color=FOAM))

# -- strings --------------------------------------------------------------------------------------------------------
GAP = 0.0012   # clearance between adjacent string pieces


def _seg(name, i, u0, u1, mass=None):
    um = (u0 + u1) / 2
    return Box(name, pos=(string_x(um), Y_NUT + um, TOP + string_z(i, um)), size=(STR_T, u1 - u0, STR_T),
               mass=mass, color=BASS if i < 3 else NYLON)


SEGS: dict = {}       # dynamic segment name -> (string, kind, fret or None, u0, u1)
STRINGS: list = []
for i in range(6):
    pu0, pu1 = PLUCK_U - PLUCK_L / 2, PLUCK_U + PLUCK_L / 2
    if i in FRETTED:
        for n in range(1, N_FRETS + 1):
            u0, u1 = fret_u(n - 1) + GAP, fret_u(n) - GAP
            nm = f"str_{NAMES[i]}_f{n}"
            SEGS[nm] = (i, "fret", n, u0, u1)
            STRINGS.append(_seg(nm, i, u0, u1, SEG_M))
        STRINGS.append(_seg(f"str_{NAMES[i]}_mid", i, fret_u(N_FRETS) + GAP, pu0 - GAP))
    else:
        STRINGS.append(_seg(f"str_{NAMES[i]}_neck", i, GAP, pu0 - GAP))
    nm = f"str_{NAMES[i]}_pluck"
    SEGS[nm] = (i, "pluck", None, pu0, pu1)
    STRINGS.append(_seg(nm, i, pu0, pu1, SEG_M))
    STRINGS.append(_seg(f"str_{NAMES[i]}_tail", i, pu1 + GAP, SCALE - GAP))

OBJECTS = [*FIXED, *STRINGS]

for nm, (i, kind, n, u0, u1) in SEGS.items():
    pass


def travel(nm: str) -> float:
    i, kind, n, u0, u1 = SEGS[nm]
    if kind == "pluck":
        return PLUCK_TRAVEL
    return action((u0 + u1) / 2)


TWIN_EXTRA = [
    {"name": "string_segments", "kind": "joint", "real": "nylon strings under tension",
     "note": "B and high E: one sprung slider per fret space (frets 1-7); all six: a sprung slider over the soundhole"},
]

_P: dict = {}


# -- spawning: cylinders along x; string segments get their sprung sliders --------------------------------------------
def _make_slider(nm: str, prim_path: str) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    stage = get_current_stage()
    w = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path)).ComputeLocalToWorldTransform(0)
    p = w.ExtractTranslation()
    i, kind, n, u0, u1 = SEGS[nm]
    um = (u0 + u1) / 2
    j = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}_slide")
    j.CreateBody1Rel().SetTargets([Sdf.Path(prim_path)])
    j.CreateAxisAttr("X")
    j.CreateLocalPos0Attr(Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
    j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateLowerLimitAttr(0.0)
    j.CreateUpperLimitAttr(float(travel(nm)))
    j.CreateExcludeFromArticulationAttr(True)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(float(stiffness(i, um)))
    d.CreateDampingAttr(SEG_C)
    d.CreateTargetPositionAttr(0.0)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(prim_path))
    rb.CreateSolverPositionIterationCountAttr(16)
    rb.CreateDisableGravityAttr(True)
    _P[nm] = prim_path


def _wrap(spec, obj):
    from isaaclab.sim.utils import clone

    cfg = obj.spawner_cfg
    if isinstance(spec, Cylinder):
        cfg.axis = "X"
    orig = cfg.func
    base = getattr(orig, "__wrapped__", orig)
    name = spec.name

    def spawn(prim_path, cc, *a, **k):
        prim = base(prim_path, cc, *a, **k)
        p = str(prim_path)
        _P["root"] = p.rsplit("/", 1)[0]
        if name in SEGS:
            try:
                _make_slider(name, p)
            except Exception as e:  # noqa: BLE001
                print(f"[guitar] slider for {name} failed: {e!r}", flush=True)
        return prim

    cfg.func = clone(spawn)
    obj.object_cfg = obj._init_object_cfg()
    return obj


_WRAP = {o.name for o in OBJECTS if isinstance(o, Cylinder)} | set(SEGS)


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_guitar_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        obj = orig(spec)
        if getattr(spec, "name", None) in _WRAP:
            _wrap(spec, obj)
        return obj

    task_object._guitar_patched = True
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


# -- the tune ---------------------------------------------------------------------------------------------------------
# (string index, fret) and the beat each note falls on (quarter notes; notes 7 and 14 are half notes)
TUNE = [(4, 1), (4, 1), (5, 3), (5, 3), (5, 5), (5, 5), (5, 3),
        (5, 1), (5, 1), (5, 0), (5, 0), (4, 3), (4, 3), (4, 1)]
BEATS = [0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14]
N_NOTES = len(TUNE)

# -- scoring constants ------------------------------------------------------------------------------------------------
HZ = 20.0
PLUCK_IN = 0.0020        # segment pushed >= 2.0 mm ...
PLUCK_OUT = 0.0005       # ... then back below 0.5 mm ...
PLUCK_STEPS = 2          # ... within 2 control steps (0.1 s) ...
SWEEP = 0.002            # ... while the fingertip pushing it moved >= 2 mm across the strings (it slid off the string)
RETREAT = 0.001          # ... or the string sprang back while that fingertip was still pressed in (pulled back < 1 mm)
FRET_FULL = 0.00015      # fretted = within 0.15 mm of the end of travel (string on the fret)
FRET_PART = 0.00020      # pressed more than 0.20 mm but not fretted = a dead (buzzing) note
BPM_LO, BPM_HI = 60.0, 100.0
TIME_TOL = 0.150         # s, a note this far off its fitted beat ends the count
PART_W = 0.9             # counted notes / 14 x 0.9
CLEAR = 0.05             # both fingertips >= 5 cm in front of the strings at the end ...
STILL_STEPS = 20         # ... and still (<= 1 cm) over 1.0 s
STILL_MOVE = 0.01
STRING_FRONT = min(string_x(u) for u in (0.0, SCALE)) - STR_T / 2

_FRESH = dict(env=None, step=0, notes=None, hist=None, ghist=None, hook_err=False, last_disp=None, pstart=None)
_M: dict = {}


def _reset_model() -> None:
    env = _M.get("env")
    _M.clear()
    _M.update(dict(_FRESH))
    _M["env"] = env
    _M["notes"] = []
    _M["hist"] = {nm: [] for nm in SEGS}
    _M["pstart"] = {}


_reset_model()


def note_name(midi: int) -> str:
    return ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"][midi % 12] + str(midi // 12 - 1)


def fret_state(disp: dict, i: int):
    """(highest fretted space or 0, dead) for string i from slider displacements (m)."""
    if i not in FRETTED:
        return 0, False
    top = 0
    for n in range(1, N_FRETS + 1):
        nm = f"str_{NAMES[i]}_f{n}"
        if disp.get(nm, 0.0) >= travel(nm) - FRET_FULL:
            top = n
    dead = False
    for n in range(top + 1, N_FRETS + 1):
        nm = f"str_{NAMES[i]}_f{n}"
        if disp.get(nm, 0.0) > FRET_PART:
            dead = True
    return top, dead


def _swept(i: int, s_push: int, s_rel: int) -> bool:
    """Did the string slip off the fingertip?  The fingertip nearest the plucked point either moved >= SWEEP across
    the strings (z) from the step the string was first pushed >= PLUCK_IN to the release step, or was pulled back out
    (x) by < RETREAT over that span, so the string sprang back while the fingertip was still pressed in.  A string
    pressed and let back out by pulling the fingertip straight away does not sound."""
    if _M.get("mock_sweep") is not None:
        return bool(_M["mock_sweep"])
    h = {st: g for st, g in (_M.get("ghist") or [])}
    if s_push not in h or s_rel not in h:
        return False
    pt = np.array([string_x(PLUCK_U), Y_NUT + PLUCK_U, string_z(i, PLUCK_U)])
    best = None
    for a, b in zip(h[s_push], h[s_rel]):
        pa, pb = np.atleast_2d(a), np.atleast_2d(b)
        k = int(np.argmin(np.linalg.norm(pa - pt, axis=1)))
        dist = float(np.linalg.norm(pa[k] - pt))
        if best is None or dist < best[0]:
            best = (dist, abs(float(pb[k, 2] - pa[k, 2])), float(pa[k, 0] - pb[k, 0]))
    if best is None:
        return False
    print(f"[guitar-dbg] slip push={s_push} rel={s_rel} dz={best[1]*1e3:.2f}mm back={best[2]*1e3:.2f}mm", flush=True)
    return best[1] >= SWEEP or best[2] < RETREAT


def _update(disp: dict, step: int) -> None:
    """Feed one control step of slider displacements; records plucks."""
    for i in range(6):
        nm = f"str_{NAMES[i]}_pluck"
        h = _M["hist"][nm]
        h.append((step, disp.get(nm, 0.0), fret_state(disp, i)))
        del h[:-(PLUCK_STEPS + 2)]
        d_now = h[-1][1]
        if d_now >= PLUCK_IN:
            _M["pstart"].setdefault(i, step)      # first step of this push
        if d_now > PLUCK_OUT:
            continue
        p0 = _M["pstart"].pop(i, None)            # released: this push episode is over
        if len(h) < 2:
            continue
        # released now: was it pushed >= PLUCK_IN within the last PLUCK_STEPS steps?
        for (s, d, fs) in reversed(h[:-1]):
            if step - s > PLUCK_STEPS:
                break
            if d >= PLUCK_IN:
                if not _swept(i, s if p0 is None else p0, step):
                    print(f"[guitar] string {NAMES[i]} let go without a sweep (pressed and pulled back): no note",
                          flush=True)
                    h.clear()
                    h.append((step, d_now, fret_state(disp, i)))
                    break
                fret, dead = fs   # fret state while the string was still pushed (the moment it was let go)
                _M["notes"].append(dict(step=step, t=step / HZ, string=i, fret=fret, dead=dead,
                                        midi=OPEN_MIDI[i] + fret))
                print(f"[guitar] pluck step {step}: string {NAMES[i]} fret {fret}{' DEAD' if dead else ''} "
                      f"-> {note_name(OPEN_MIDI[i] + fret)}", flush=True)
                h.clear()
                h.append((step, d_now, fret_state(disp, i)))
                break


def count_notes(notes: list) -> tuple[int, dict]:
    """Notes played in order (first wrong/dead/extra note ends the count), then the timing check."""
    k = 0
    for nt in notes:
        if k >= N_NOTES:
            break
        want = TUNE[k]
        if nt["dead"] or (nt["string"], nt["fret"]) != want:
            break
        k += 1
    info = dict(in_order=k, extra=max(0, len(notes) - N_NOTES), bpm=None, worst_ms=None)
    if k >= 3:
        t = np.array([n["t"] for n in notes[:k]])
        b = np.array(BEATS[:k], float)
        A = np.stack([b, np.ones_like(b)], 1)
        (per, t0), *_ = np.linalg.lstsq(A, t, rcond=None)
        bpm = 60.0 / per if per > 1e-6 else 1e9
        res = np.abs(t - (t0 + per * b))
        info["bpm"] = round(float(bpm), 1)
        info["worst_ms"] = round(float(res.max() * 1000), 1)
        if not (BPM_LO <= bpm <= BPM_HI):
            return 0, info
        bad = np.nonzero(res > TIME_TOL)[0]
        if len(bad):
            k = int(bad[0])
    return k, info


def _np(t):
    try:
        return t.detach().cpu().numpy()
    except AttributeError:
        return np.asarray(t)


def _env_disp() -> dict:
    base = _M["env"].unwrapped
    origin = _np(base.scene.env_origins[0]).astype(np.float64)
    out = {}
    for nm, (i, kind, n, u0, u1) in SEGS.items():
        pose = _np(base.arena_world.get_pose_w(nm)[0]).astype(np.float64)
        out[nm] = float(pose[0] - origin[0] - string_x((u0 + u1) / 2))
    return out


def grippers() -> list:
    env = _M.get("env")
    if env is None:
        return _M.get("mock_grippers") or []
    from yam_arena.runtime import pad_tips_world

    base = env.unwrapped
    origin = _np(base.scene.env_origins[0]).astype(np.float64)
    return [np.asarray(pad_tips_world(base.scene[k]), dtype=np.float64) - origin - np.array([0.0, 0.0, TOP])
            for k in ("left_robot", "right_robot")]


def _track_hands() -> None:
    g = grippers()
    if len(g) < 2:
        return
    h = _M["ghist"] if _M["ghist"] is not None else []
    _M["ghist"] = h
    h.append((_M["step"], [p.copy() for p in g]))
    del h[:-(STILL_STEPS + 1)]


def hands_clear() -> bool:
    if _M.get("mock_clear") is not None:
        return bool(_M["mock_clear"])
    h = _M.get("ghist") or []
    if len(h) <= STILL_STEPS:
        return False
    g = h[-1][1]
    if any(float(np.max(np.atleast_2d(p)[:, 0])) > STRING_FRONT - CLEAR for p in g):
        return False
    return all(max(float(np.linalg.norm(np.atleast_2d(x[1][i]) - np.atleast_2d(g[i]), axis=1).max()) for x in h)
               <= STILL_MOVE for i in range(len(g)))


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline import)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_guitar_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _M["env"] = self._env
            out = base(self, *a, **k)
            _M["step"] += 1
            try:
                _track_hands()
                d = _env_disp()
                _M["last_disp"] = d
                _update(d, _M["step"])
            except Exception as e:  # noqa: BLE001
                if not _M.get("hook_err"):
                    _M["hook_err"] = True
                    print(f"[guitar] step tracking failed: {e!r}", flush=True)
            return out

        _task_step._guitar_hooked = True
        cls._task_step = _task_step
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


_install_hooks()


# -- scoring ----------------------------------------------------------------------------------------------------------
def score(objects=None) -> float:
    """counted notes / 14 x 0.9; 1.0 = all 14 counted, no extra pluck, both hands >= 5 cm clear of the strings, still."""
    k, info = count_notes(_M["notes"])
    if k >= N_NOTES and info["extra"] == 0 and hands_clear():
        return 1.0
    return round(PART_W * k / N_NOTES, 4)


def success(objects=None) -> bool:
    return score(objects) >= 1.0


def progress(objects=None) -> dict:
    k, info = count_notes(_M["notes"])
    return {"counted": k, **info, "plucks": [f"{NAMES[n['string']]}{n['fret']}{'x' if n['dead'] else ''}@{n['t']:.2f}"
                                             for n in _M["notes"]], "clear": float(hands_clear())}


VARIATION = {}
