"""On the station table a free grey patient-monitor lead (6 mm PVC, 60 cm) lies straight, left to right, 40 cm in
front of the robot: a grey plug (35 x 16 x 12 mm) on its left end in front of the left arm, a red patient clip
(25 x 15 x 15 mm) on its right end in front of the right arm. A 2 cm blue band is painted on the cord 14-16 cm from
the plug. A clean white drape (76 x 18 cm, 3 mm thick) lies on the table nearer the robot, parallel to the lead.
Both grippers' finger pads are covered with disinfectant wipe (real trial; bare pads in sim).

The whole lead is wiped outward from the blue band, every part once and never wiped back over: the cord up to the
clip face, then the clip's sides, cord face, top and tip face while the plug is held; the cord up to the plug face,
then the plug's sides, cord face, top and tip face while the clip is held; then each end piece in turn is held up
rolled a quarter turn about the lead while the other hand wipes the strip of its cord face under the cord and its
underside; then the lead is laid on the drape and let go. No surface of the lead is left unwiped.

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
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Wipe the whole grey monitor lead with the wipe pads, the cord from end to end and every face of both end "
               "pieces, undersides included, wiping outward from the blue band and never back over any part already "
               "wiped, then lay the lead on the white drape and let go.")

# -- scene -------------------------------------------------------------------------------------------------------
R = 0.003                      # lead radius (6 mm lead)
D = 2 * R
LEAD_LEN = 0.60
LEAD_X = 0.40
PLUG = Plug(size=(0.035, 0.016, 0.012), mass=0.012, color=(0.30, 0.31, 0.33))
CLIP = Plug(size=(0.025, 0.015, 0.015), mass=0.010, color=(0.80, 0.10, 0.08))
PLUG_Y = 0.08                  # plug centre
CORD_Y0 = PLUG_Y - PLUG.size[0] / 2   # cord start (plug face), y 0.0625; the cord runs toward -y
LEAD = cable("lead", start=(LEAD_X, CORD_Y0, TOP + CLIP.size[2] / 2 + 0.0005),
             yaw=-math.pi / 2, length=LEAD_LEN, radius=R, start_plug=PLUG, end_plug=CLIP,
             cable_mass=0.033, color=(0.62, 0.63, 0.65), bend_stiffness=0.01, bend_damping=0.001)
SEG = LEAD.seg_len             # 0.02
S_BLUE = (0.14, 0.16)          # band extent, m from the plug end of the cord
BLUE_SEG = round(S_BLUE[0] / SEG)   # 7
BLUE = (0.10, 0.30, 0.95)

DRAPE_X = (0.12, 0.30)
DRAPE_Y = (-0.56, 0.20)
DRAPE_T = 0.003
DRAPE = Box("drape", pos=(sum(DRAPE_X) / 2, sum(DRAPE_Y) / 2, TOP + DRAPE_T / 2),
            size=(DRAPE_X[1] - DRAPE_X[0], DRAPE_Y[1] - DRAPE_Y[0], DRAPE_T), color=(0.96, 0.96, 0.96))

OBJECTS = [DRAPE, LEAD]
TWIN_EXTRA = [
    {"name": "blue_band", "kind": "decal", "size_mm": [20, 6], "colour": "blue", "pose": [0.40, -0.0875, 0.003]},
    {"name": "finger_wipes", "kind": "wipe pad on each finger pad", "size_mm": [40, 40, 1.5], "colour": "white"},
]


def _install_bands() -> None:
    """cablekit has no per-segment colour: paint the blue band segment (visual only)."""
    orig = cablekit._spawn_cable
    if getattr(orig, "_wipe_bands", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != LEAD.name:
            return prim
        try:
            import isaaclab.sim as sim_utils
            from isaaclab.sim.utils import bind_visual_material
            from isaaclab.sim.utils.stage import get_current_stage
            from yam_arena.station import linear_rgb

            look = f"{prim_path}/Looks/band_blue"
            mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BLUE), roughness=0.5)
            mat.func(look, mat)
            bind_visual_material(f"{prim_path}/seg_{BLUE_SEG}/geom", look, stage=get_current_stage())
        except Exception as e:  # pragma: no cover - never break the scene for a colour
            print(f"[wipe task] band colour not applied: {e}", flush=True)
        return prim

    spawn._wipe_bands = True
    cablekit._spawn_cable = spawn


_install_bands()

# -- scoring -----------------------------------------------------------------------------------------------------
CLOSED_GAP = D + 0.001         # finger gap that counts as closed on the cord (pads within 0.5 mm of it)
PAD_HALF_W = 0.010             # half pad width along the lead
PAD_UP, PAD_DOWN = 0.015, 0.013  # pad extent above / below the TCP along the approach axis
BACK_TOL = 0.03                # backward tolerance for a hand closed on the cord
MIN_TRAVEL = 0.02              # a closed cord slide counts once it has slid 2 cm along the cord
REWIPE_MAX = 4                 # marks already wiped that a later slide may pass over again (stroke overlap)
HOLD_DIST = 0.025              # end-piece centre to TCP for "holds"
RELEASE_DIST = 0.03
MARK_TOL = 0.005
MARKS = np.round(np.arange(0.005, LEAD_LEN, 0.01), 3)   # 0.005 .. 0.595: 60 marks, one per cm of cord, end to end
SPLIT_LO, SPLIT_HI = S_BLUE[0] - 0.005, S_BLUE[1] + 0.005   # clip-side strokes credit marks >= 0.135, plug-side <= 0.165
C_MID, C_LOW = 0.90, 0.50
ENDS = {"plug": PLUG, "clip": CLIP}
OTHER_END = {"plug": "clip", "clip": "plug"}
BODY_GAP_LO, BODY_GAP_HI = 0.002, 0.007   # pads on a body: gap from its smaller cross size - 2 mm to its larger + 7 mm
BODY_LAT = 0.005               # body centre within 5 mm of the pads' mid-plane (it is between the pads)
BODY_BACK_TOL = 0.01           # a pass on a body may not go back toward the cord by more than 1 cm
BODY_REWIPE = 0.01             # a new pass sliding 1 cm along a body already wiped = wiping back over it
BODY_IN = 0.008                # a pass must close with the pad centre at most 8 mm onto the body (pads straddle the face)
REACH = 0.013                  # a cord stroke reaches an end piece: pad centre within 13 mm of its cord face, measured
                               # along the table (the bare finger's leading edge, 7.7 mm from the centre, within 5.3 mm;
                               # the wipe pad, standing out ~1 cm past that edge, is pressed to the face)
SWEEP_MOVE = 0.005             # the hand opens at the cord face (TCP moves at most 5 mm) ...
SWEEP_GAP = 0.006              # ... to at least the body's larger cross size + 6 mm (pad edges swept past the face's sides)
TOP_GAP = 0.003                # top-and-tip pass: fingers closed (gap at most 3 mm)
TIP_PAST = 0.013               # finger tips 13 mm past the TCP along the approach axis
FINGER_T = 0.004               # each closed finger is 4 mm thick at its tip (outer face 4 mm from the TCP)
TOP_LAT = 0.007                # TCP within 7 mm of the body's centre line: the 15 mm finger tip's wipe pad stands out
                               # about 1 cm past each side edge (a ~35 mm footprint across), so it covers a 16 mm top
TOP_H = (-0.001, 0.006)        # fingertip 0-6 mm above the top face (sim, bare; the 6 mm pad touches or presses)
FACE_CLR = 0.006               # bare finger's outer face within 6 mm of an end face (0-6 mm; the 6 mm pad touches or presses)
LOW_H = 0.004                  # fingertip down to within 4 mm of the body's bottom edge (the table)
TOP_BACK_TOL = 0.01            # a top pass may not go back toward the cord by more than 1 cm
ALIGN = math.cos(math.radians(30))   # top pass: fingers close along the body's axis (within 30 deg)
FINGER_HW = 0.0075             # half the closed fingers' width (15 mm) along the end piece in the underside pass
EDGE_CLR = 0.008               # underside pass: the 15 mm tip's leading edge up to 8 mm behind the cord face, as the
#                                fingers widen 14 mm above their tips and that part stops against the face first; the
#                                pad standing out ~1 cm past the edge still sweeps the face
ROOT_CLR = 0.004               # underside pass starts with the finger's outer face within 4 mm of the cord (sim, bare;
                               # the 6 mm pad presses on it)
PAD_T = 0.003                  # strip sweep ends with the bare finger's outer face 3 mm past the underside's plane
TIP_BELOW = 0.006              # ... and the fingertip at most 6 mm past the face's lower edge (the pad wraps 2.5 cm up
                               # the finger's outer face, so it still covers the 15-16 mm face)
PARTS = ("reach", "sweep", "sides", "face_up", "top", "tip", "face_down", "under")

_ENV: dict = {"env": None}
_ST: dict = {}


def _reset_episode() -> None:
    _ST.clear()
    _ST.update(wiped=np.zeros(len(MARKS), bool), back=False, back_info="",
               cl={"left": None, "right": None}, ec={"left": None, "right": None},
               ends={"plug": False, "clip": False}, tp={"left": None, "right": None},
               sw={"left": None, "right": None}, un={"left": None, "right": None},
               parts={n: {q: False for q in PARTS} for n in ("plug", "clip")})


_reset_episode()


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_wipe_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._wipe_hooked = True
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


_install_hooks()


def _t(x):
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy()


def hands() -> dict:
    """{arm: (tcp, approach axis, closing axis, width axis, finger gap, gripper-frame closing axis)} in env
    coordinates; {} offline. The third entry comes from the two finger tips' link origins, which also differ along
    the finger width, so with the fingers shut it leans ~28 deg off the closing direction; the last is exact."""
    env = _ENV["env"]
    if env is None:
        return {}
    from yam_arena.analysis import pose7_to_matrix
    from yam_arena.constants import TCP_OFFSET

    base = env.unwrapped
    origin = _t(base.scene.env_origins)[0].astype(np.float64)
    out = {}
    for arm in ("left", "right"):
        art = base.scene[f"{arm}_robot"]
        names = list(art.body_names)
        poses = _t(art.data.body_link_pose_w)[0]
        g = pose7_to_matrix(poses[names.index("gripper")])
        tcp = g[:3, :3] @ np.array(TCP_OFFSET) + g[:3, 3] - origin
        a = g[:3, 2]
        tl, tr = poses[names.index("tip_left")][:3], poses[names.index("tip_right")][:3]
        c = (tl - tr).astype(np.float64)
        c -= a * (c @ a)
        c /= np.linalg.norm(c) + 1e-12
        w = np.cross(a, c)
        jn = list(art.joint_names)
        q = _t(art.data.joint_pos)[0]
        gap = -float(q[jn.index("joint7")] + q[jn.index("joint8")])
        # the fingers' true closing axis: the gripper frame's x or y axis (the two arms' frames differ by a quarter
        # turn), whichever the tip-to-tip direction leans toward
        ex = max((g[:3, 0], g[:3, 1]), key=lambda e: abs(float(e @ c))).astype(np.float64)
        out[arm] = (tcp, a, c, w, gap, ex)
    return out


def piece_rots() -> dict:
    """{name: 3x3 rotation} of the plug and the clip bodies (columns: along the lead, across it, up through the top
    face when lying as laid out); {} offline."""
    env = _ENV["env"]
    if env is None:
        return {}
    art = env.unwrapped.scene[LEAD.name]
    names = list(art.body_names)
    qs = _t(art.data.body_quat_w)[0].astype(np.float64)
    out = {}
    for name, key in (("plug", "start_plug"), ("clip", "end_plug")):
        if key not in names:
            continue
        x, y, z, w = qs[names.index(key)]                 # Isaac Lab body quaternions are (x, y, z, w)
        out[name] = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                              [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                              [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return out


def _dense_s(c, step: float = 0.002):
    """Resampled cord polyline (tips + segment centres) with arclength from the plug end."""
    poly = c.polyline()
    s_v = np.r_[0.0, (np.arange(len(poly) - 2) + 0.5) * c.seg_len, LEAD_LEN]
    pts, ss = [poly[0]], [0.0]
    for i in range(len(poly) - 1):
        k = max(1, int(round((s_v[i + 1] - s_v[i]) / step)))
        for j in range(1, k + 1):
            f = j / k
            pts.append(poly[i] + (poly[i + 1] - poly[i]) * f)
            ss.append(s_v[i] + (s_v[i + 1] - s_v[i]) * f)
    return np.array(pts), np.array(ss)


def _pinch(h, pts, ss):
    """(s_lo, s_mid, s_hi) of the cord inside this gripper's closed pinch, or None."""
    tcp, a, cax, w, gap = h[:5]
    if gap > CLOSED_GAP:
        return None
    d = pts - tcp
    da, dc, dw = d @ a, d @ cax, d @ w
    m = (da >= -PAD_UP) & (da <= PAD_DOWN) & (np.abs(dc) <= gap / 2 + 0.001) & (np.abs(dw) <= PAD_HALF_W)
    if not m.any():
        return None
    return float(ss[m].min()), float(ss[m].mean()), float(ss[m].max())


def _end_pose(c, name):
    """(centre, outward unit axis) of an end piece; the axis points from the cord toward the free tip."""
    if name == "plug":
        ctr, tip = c.start_plug, c.polyline()[0]
    else:
        ctr, tip = c.end_plug, c.polyline()[-1]
    if ctr is None or not np.isfinite(ctr).all():
        return None, None
    ax = np.asarray(ctr, float) - tip
    return np.asarray(ctr, float), ax / (np.linalg.norm(ax) + 1e-12)


def _holds(h, c, name) -> bool:
    """Gripper holds this end piece: finger gap at most its larger cross dimension + 3 mm, centre within 2.5 cm."""
    ctr, _ = _end_pose(c, name)
    p = ENDS[name]
    return ctr is not None and h[4] <= max(p.size[1], p.size[2]) + 0.003 and \
        float(np.linalg.norm(ctr - h[0])) <= HOLD_DIST


def _violate(msg: str) -> None:
    if not _ST["back"]:
        _ST["back"] = True
        _ST["back_info"] = msg


def _track_cord(arm, other, h, ho, c, pts, ss) -> None:
    p = _pinch(h, pts, ss)
    if p is None:
        k = _ST["cl"][arm]
        if k is not None and k.get("reach"):
            # the stroke ended at the cord face: it now has to open there (pads sweep across the face beside the cord)
            _ST["sw"][arm] = dict(name=k["reach"], tcp=k["tcp"])
        _ST["cl"][arm] = None
        return
    lo, mid, hi = p
    k = _ST["cl"][arm]
    if k is None:
        k = _ST["cl"][arm] = dict(s0=mid, smin=mid, smax=mid, dir=0, dmax=0.0, a=None, b=None,
                                  snap=_ST["wiped"].copy())
    if k["dir"] == 0 and abs(mid - k["s0"]) >= 0.01:
        k["dir"] = 1 if mid > k["s0"] else -1
    if k["dir"] > 0 and mid < k["smax"] - BACK_TOL:
        _violate(f"{arm} slid back toward the plug: s={mid:.3f} after {k['smax']:.3f}")
    if k["dir"] < 0 and mid > k["smin"] + BACK_TOL:
        _violate(f"{arm} slid back toward the clip: s={mid:.3f} after {k['smin']:.3f}")
    k["smin"], k["smax"] = min(k["smin"], mid), max(k["smax"], mid)
    if k["dir"]:
        behind = "plug" if k["dir"] > 0 else "clip"
        ctr, _ = _end_pose(c, behind)
        if ctr is not None:
            # a closed stroke back toward the anchored end pushes the cord instead of sliding on it
            dist = float(np.linalg.norm(h[0] - ctr))
            if dist < k["dmax"] - BACK_TOL:
                _violate(f"{arm} hand moved back toward the {behind}: {dist:.3f} after {k['dmax']:.3f}")
            k["dmax"] = max(k["dmax"], dist)
        anchored = ho is not None and _holds(ho, c, behind)
    else:
        anchored = ho is not None and (_holds(ho, c, "plug") or _holds(ho, c, "clip"))
    k["tcp"] = h[0].copy()
    if anchored:
        k["a"] = lo if k["a"] is None else min(k["a"], lo)
        k["b"] = hi if k["b"] is None else max(k["b"], hi)
        if k["dir"] and k["smax"] - k["smin"] >= MIN_TRAVEL:
            ahead = "clip" if k["dir"] > 0 else "plug"
            ctr, ax = _end_pose(c, ahead)
            axh = None if ctr is None else np.array([ax[0], ax[1], 0.0]) / (np.linalg.norm(ax[:2]) + 1e-12)
            if ctr is not None and -ENDS[ahead].size[0] / 2 - float((h[0] - ctr) @ axh) <= REACH:
                k["reach"] = ahead
                _ST["parts"][ahead]["reach"] = True
    if k["dir"] and k["a"] is not None and k["smax"] - k["smin"] >= MIN_TRAVEL:
        new = (MARKS >= k["a"] - MARK_TOL - 1e-9) & (MARKS <= k["b"] + MARK_TOL + 1e-9)
        new &= (MARKS >= SPLIT_LO - 1e-9) if k["dir"] > 0 else (MARKS <= SPLIT_HI + 1e-9)
        rew = int((new & k["snap"]).sum())
        if rew > REWIPE_MAX:
            _violate(f"{arm} wiped back over {rew} cm of already-wiped cord")
        _ST["wiped"] |= new


def _on_body(h, c, name):
    """u = TCP position along the body's outward axis (from its centre, + toward the free tip) if this gripper's pads
    are closed around the body (gap fits the body, body centre between the pads and within the pads' height,
    pads overlapping the body's length); else None."""
    tcp, a, cax, _, gap = h[:5]
    p = ENDS[name]
    if not (min(p.size[1], p.size[2]) - BODY_GAP_LO <= gap <= max(p.size[1], p.size[2]) + BODY_GAP_HI):
        return None
    ctr, ax = _end_pose(c, name)
    if ctr is None:
        return None
    d = ctr - tcp
    u = float(-d @ ax)
    c2 = cax - ax * (cax @ ax)
    c2 /= np.linalg.norm(c2) + 1e-12
    if not (-PAD_UP <= d @ a <= PAD_DOWN and abs(d @ c2) <= BODY_LAT and abs(u) <= p.size[0] / 2 + PAD_HALF_W):
        return None
    return u


def _track_ends(arm, ho, h, c) -> None:
    """One closed pass per body: anchored (other hand holds the other end piece) the whole time, closing with the pads
    straddling the cord face (pad centre u <= -L/2 + 8 mm) and sliding outward until the pad centre reaches the free
    tip (u >= +L/2); never back by more than 1 cm."""
    hit = None
    for name in ENDS:
        u = _on_body(h, c, name)
        if u is not None:
            hit = (name, u)
            break
    k = _ST["ec"][arm]
    if hit is None:
        _ST["ec"][arm] = None
        return
    name, u = hit
    half = ENDS[name].size[0] / 2
    if k is None or k["name"] != name:
        k = _ST["ec"][arm] = dict(name=name, u0=u, umin=u, umax=u, ok=True, done=_ST["ends"][name])
    if k["u0"] <= -half + BODY_IN and u < k["umax"] - BODY_BACK_TOL:
        _violate(f"{arm} went back along the {name} body toward the cord: u={u:.3f} after {k['umax']:.3f}")
    if k["done"] and max(k["umax"] - k["u0"], k["u0"] - k["umin"]) >= BODY_REWIPE:
        _violate(f"{arm} wiped the {name} body again")
    k["umin"], k["umax"] = min(k["umin"], u), max(k["umax"], u)
    if not (ho is not None and _holds(ho, c, OTHER_END[name])):
        k["ok"] = False
    if k["ok"] and k["u0"] <= -half + BODY_IN and k["umax"] >= half:
        _ST["ends"][name] = True
        _ST["parts"][name]["sides"] = True


def _track_sweep(arm, h) -> None:
    """After a stroke that reached a cord face ends, the hand must open there: the pads' leading edges, pressed to the
    face, sweep across it from the cord out past its sides."""
    k = _ST["sw"][arm]
    if k is None:
        return
    if float(np.linalg.norm(h[0] - k["tcp"])) > SWEEP_MOVE:
        _ST["sw"][arm] = None
    elif h[4] >= max(ENDS[k["name"]].size[1:]) + SWEEP_GAP:
        _ST["parts"][k["name"]]["sweep"] = True
        _ST["sw"][arm] = None


def _top_geom(h, c, name):
    """(u, lat, z) if this gripper is closed with its fingers lined up along the body's axis near it: u = TCP along the
    body's outward axis from its centre, lat = TCP across it, z = fingertip height above the body's centre."""
    tcp, a, _, _, gap = h[:5]
    cax = h[5] if len(h) > 5 else h[2]
    if gap > TOP_GAP:
        return None
    ctr, ax = _end_pose(c, name)
    if ctr is None:
        return None
    axh = np.array([ax[0], ax[1], 0.0])
    axh /= np.linalg.norm(axh) + 1e-12
    if abs(float(cax @ axh)) < ALIGN:
        return None
    d = tcp - ctr
    u, lat = float(d @ axh), float(d @ np.array([-axh[1], axh[0], 0.0]))
    if abs(lat) > 0.02 or abs(u) > ENDS[name].size[0] / 2 + 0.03:
        return None
    return u, lat, float((tcp + a * TIP_PAST)[2] - ctr[2])


def _track_top(arm, ho, h, c) -> None:
    """Top-and-tip pass of the closed finger pads, the fingers lined up along the body and the other hand holding the
    other end piece throughout: down the cord face above the cord (leading finger's outer face 0-6 mm behind the face,
    fingertip within 4 mm of the cord's top), then over the whole top face outward (fingertip 0-6 mm above it, the
    4 mm-thick fingers' footprint covering it from the cord face to the tip), then down the tip face (trailing finger's
    outer face 0-6 mm past it, fingertip to within 4 mm of the table). Never back toward the cord by over 1 cm."""
    hit = None
    for name in ENDS:
        g = _top_geom(h, c, name)
        if g is not None:
            hit = (name, g)
            break
    if hit is None:
        _ST["tp"][arm] = None
        return
    name, (u, lat, z) = hit
    P = ENDS[name]
    half, hh = P.size[0] / 2, P.size[2] / 2
    k = _ST["tp"][arm]
    if k is None or k["name"] != name:
        k = _ST["tp"][arm] = dict(name=name, ok=True, lo=None, hi=None, umax=None,
                                  done=_ST["parts"][name]["top"])
    if not (ho is not None and _holds(ho, c, OTHER_END[name])):
        k["ok"] = False
    if abs(lat) > TOP_LAT:
        return
    pr = _ST["parts"][name]
    if -half - FINGER_T - FACE_CLR <= u <= -half - FINGER_T and z <= R + LOW_H and k["ok"]:
        pr["face_up"] = True
    if hh + TOP_H[0] <= z <= hh + TOP_H[1]:
        if k["umax"] is not None and u < k["umax"] - TOP_BACK_TOL:
            _violate(f"{arm} went back along the {name} top toward the cord: u={u:.3f} after {k['umax']:.3f}")
        k["umax"] = u if k["umax"] is None else max(k["umax"], u)
        k["lo"] = u if k["lo"] is None else min(k["lo"], u)
        k["hi"] = u if k["hi"] is None else max(k["hi"], u)
        if k["done"] and k["hi"] - k["lo"] >= BODY_REWIPE:
            _violate(f"{arm} wiped the {name} top again")
        if k["ok"] and k["lo"] <= -half + FINGER_T and k["hi"] >= half - FINGER_T:
            k["top"] = True
            pr["top"] = True
    if k.get("top") and k["ok"] and half + FINGER_T <= u <= half + FINGER_T + FACE_CLR and z <= -hh + LOW_H:
        pr["tip"] = True


def _track_under(arm, ho, h, c, rots) -> None:
    """Underside pass on an end piece turned a quarter turn so its underside faces sideways (either tipped end-down,
    turned about the axis across the lead, or rolled about the lead), the other hand holding this end piece or the
    other one (the anchor, as for the other passes) throughout, the wiping hand's fingers
    shut and closing along the underside's normal (within 30 deg). Two ways in, by how the turned piece lies under the
    downward-pointing fingers:
    - end-down (the fingers point along the piece): the fingertip, 0-6 mm above the cord face and the fingers' 15 mm
      width centred on the piece (within 7 mm), first sweeps across the strip of that face under the cord root (the
      fingers' outer face goes from within 4 mm of the cord to 3 mm past the underside's plane, so the 6 mm pad
      spans the face's edge); then, the outer face 0-6 mm
      off the underside, the fingertip slides down along it from the cord face to the tip;
    - rolled (the fingers point across the underside, the fingertip within 4 mm of its lower edge or up to 6 mm past
      it): the fingers' leading edge, 0-6 mm behind the cord face, sweeps the strip the same way; then the outer face
      slides along the underside until the 15 mm width has covered it from the cord face to the tip.
    Never back toward the cord by over 1 cm; a later closure sliding 1 cm over an underside already wiped wipes it
    again."""
    tcp, a, _, _, gap = h[:5]
    cax = h[5] if len(h) > 5 else h[2]
    hit = None
    if gap <= TOP_GAP:
        for name in ENDS:
            if name not in rots:
                continue
            ctr, ax = _end_pose(c, name)
            if ctr is None:
                continue
            P = ENDS[name]
            n, wv = -rots[name][:, 2], rots[name][:, 1]       # underside normal; the face's width direction
            if abs(float(cax @ n)) < ALIGN:
                continue
            d = tcp - ctr
            e = float(d @ n)
            if not (0.0 < e <= P.size[2] / 2 + 0.03):
                continue
            tipp = tcp + a * TIP_PAST - ctr                    # fingertip from the piece's centre
            if abs(float(a @ ax)) >= ALIGN:                    # end-down: t = fingertip along the piece
                t = float(tipp @ ax)
                if abs(float(d @ wv)) <= TOP_LAT and abs(t) <= P.size[0] / 2 + 0.03:
                    hit = (name, "end", t, e - FINGER_T)
            elif abs(float(a @ wv)) >= ALIGN:                  # rolled: t = TCP along the piece
                t = float(d @ ax)
                tip = float(tipp @ a)
                if abs(t) <= P.size[0] / 2 + 0.03 and P.size[1] / 2 - LOW_H <= tip <= P.size[1] / 2 + TIP_BELOW:
                    hit = (name, "roll", t, e - FINGER_T)
            if hit:
                break
    _ST.setdefault("un_dbg", {})[arm] = None if hit is None else tuple(round(x, 4) if isinstance(x, float) else x
                                                                        for x in hit)
    if hit is None:
        _ST["un"][arm] = None
        return
    name, mode, t, o = hit
    P = ENDS[name]
    half, hh = P.size[0] / 2, P.size[2] / 2
    k = _ST["un"][arm]
    if k is None or k["name"] != name:
        k = _ST["un"][arm] = dict(name=name, ok=True, root0=False, lo=None, hi=None, tmax=None,
                                  done=_ST["parts"][name]["under"])
    if not (ho is not None and (_holds(ho, c, name) or _holds(ho, c, OTHER_END[name]))):
        k["ok"] = False
    pr = _ST["parts"][name]
    if mode == "end":
        at_face = -half - FACE_CLR <= t <= -half + 0.001            # fingertip 0-6 mm above the cord face
        lo_need, hi_need = -half + 0.004, half                      # fingertip from the cord face to the tip
    else:
        at_face = -half - EDGE_CLR <= t + FINGER_HW <= -half        # leading edge 0-8 mm behind the cord face
        lo_need, hi_need = -half + FINGER_HW, half - FINGER_HW      # 15 mm width over the whole length
    if at_face:
        if o <= R + ROOT_CLR:
            k["root0"] = True
        if k["root0"] and o >= hh + PAD_T and k["ok"]:
            pr["face_down"] = True
    if hh <= o <= hh + FACE_CLR:
        if k["tmax"] is not None and t < k["tmax"] - TOP_BACK_TOL:
            _violate(f"{arm} went back along the {name} underside toward the cord: t={t:.3f} after {k['tmax']:.3f}")
        k["tmax"] = t if k["tmax"] is None else max(k["tmax"], t)
        k["lo"] = t if k["lo"] is None else min(k["lo"], t)
        k["hi"] = t if k["hi"] is None else max(k["hi"], t)
        if k["done"] and k["hi"] - k["lo"] >= BODY_REWIPE:
            _violate(f"{arm} wiped the {name} underside again")
        if k["ok"] and k["lo"] <= lo_need and k["hi"] >= hi_need:
            pr["under"] = True


def _track(objects) -> None:
    """Latch wiped marks, wiped end pieces and the backward rules (idempotent per step)."""
    c = objects["lead"]
    hs = hands()
    if not hs or not np.isfinite(c.points).all():
        return
    pts, ss = _dense_s(c)
    rots = piece_rots()
    for arm, other in (("left", "right"), ("right", "left")):
        _track_cord(arm, other, hs[arm], hs.get(other), c, pts, ss)
        _track_ends(arm, hs.get(other), hs[arm], c)
        _track_sweep(arm, hs[arm])
        _track_top(arm, hs.get(other), hs[arm], c)
        _track_under(arm, hs.get(other), hs[arm], c, rots)


def coverage(objects) -> float:
    _track(objects)
    return float(_ST["wiped"].mean())


def ends_wiped() -> int:
    """Number of end-piece parts wiped (8 per end piece: cord up to its face, cord-face sweep, both long sides, cord
    face above the cord, top, tip face, cord face under the cord, underside)."""
    return sum(int(v) for pr in _ST["parts"].values() for v in pr.values())


def ends_done() -> bool:
    return all(all(pr.values()) for pr in _ST["parts"].values())


def _on_drape(p) -> bool:
    return bool(DRAPE_X[0] <= p[0] <= DRAPE_X[1] and DRAPE_Y[0] <= p[1] <= DRAPE_Y[1] and p[2] <= TOP + 0.03)


def plug_on_drape(objects) -> bool:
    p = objects["lead"].start_plug
    return p is not None and np.isfinite(p).all() and _on_drape(p)


def lead_on_drape(objects) -> bool:
    c = objects["lead"]
    pts, _ = _dense_s(c)
    if c.start_plug is None or c.end_plug is None:
        return False
    allp = np.vstack([pts, c.start_plug[None], c.end_plug[None]])
    return bool(np.isfinite(allp).all() and all(_on_drape(q) for q in allp))


def released(objects) -> bool:
    """Every gripper at least 3 cm from every point of the cord, plug and clip; False without gripper state."""
    c = objects["lead"]
    hs = hands()
    if len(hs) != 2:
        return False
    pts, _ = _dense_s(c)
    allp = np.vstack([pts] + [q[None] for q in (c.start_plug, c.end_plug) if q is not None])
    return all(float(np.linalg.norm(allp - h[0], axis=1).min()) >= RELEASE_DIST for h in hs.values())


def score(objects) -> float:
    """0 on any backward wipe (latched), if the plug is not on the drape, or a gripper is within 3 cm of the lead at
    the end. Else by the fraction c of the 60 1 cm cord marks (0.5 to 59.5 cm) wiped: 0.3 if c >= 0.5, 0.6 if c >= 0.9,
    1.0 if every mark is wiped, all eight parts of both end pieces are wiped (cord up to the face, cord-face sweep, both
    long sides, cord face above the cord, top, tip face, cord face under the cord, underside) and the whole lead
    (cord, plug and clip) lies on the drape."""
    c = coverage(objects)
    if _ST["back"] or not plug_on_drape(objects) or not released(objects):
        return 0.0
    if _ST["wiped"].all() and ends_done() and lead_on_drape(objects):
        return 1.0
    if c >= C_MID:
        return 0.6
    if c >= C_LOW:
        return 0.3
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["lead"].resting(max_speed=0.02)


def distance(objects) -> float:
    """1 - (wiped marks + wiped end-piece parts) / 76 (1.0 once a backward rule fired)."""
    c = coverage(objects)
    return 1.0 if _ST["back"] else 1.0 - (c * len(MARKS) + ends_wiped()) / (len(MARKS) + 2 * len(PARTS))


def progress(objects) -> dict:
    hs = hands()
    c = objects["lead"]
    held = any(_holds(h, c, "plug") for h in hs.values())
    return {"plug_held": int(held), "coverage": round(coverage(objects), 3),
            "clip": "".join(str(int(v)) for v in _ST["parts"]["clip"].values()),
            "plug": "".join(str(int(v)) for v in _ST["parts"]["plug"].values()), "no_backwipe": int(not _ST["back"]),
            "plug_on_drape": int(plug_on_drape(objects)), "lead_on_drape": int(lead_on_drape(objects)),
            "released": int(released(objects))}


VARIATION = {"cable:lead": {"dx": 0.015, "dy": 0.015, "yaw": 0.05}}

# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------
# Actions come only from the fixed known layout and the robot's own measured joint state (TCP from the joint
# angles in ctx.obs.state). ctx.states() is used only by log() for the run log.
WIPE_F = -0.0025               # finger joint target: pads 5 mm apart -> light squeeze on the 6 mm cord
LIFT = TOP + 0.025
WIPE_Z = TOP + 0.0115          # TCP height while wiping the cord
OVER = 0.010                   # TCP above an end piece's centre when gripping it
WIPE_Z2 = TOP + 0.0095         # stroke 2: the left arm sits ~2 mm higher than the right
S_START = S_BLUE[0] + 0.005    # stroke 1 closes on the near half of the blue band
S2_START = S_BLUE[1] - 0.005   # stroke 2 closes on the far half
PLUG_TOP_OFF = np.array([-0.004, -0.0095, 0.0])   # measured: turned along the lead over the plug, the left hand
#                                                  settles about 9.5 mm outward along it and 4 mm forward of target
STOP_FACE = 0.0115             # measured: a stroke's pad centre stops this far from the end piece's cord face
PINCH_OFF = 0.018              # measured: pinch arclength = TCP distance to the held plug centre - 1.8 cm
SHORT = os.environ.get("WIPE_CHEAT", "") == "shortend"   # test only: the plug-side stroke stops 3 cm short of the plug
WIPE_F_END = -0.0033           # last 2 cm before an end piece: pads 6.6 mm apart, just round the 6 mm cord (the 6 mm
                               # pads press on it), so they slide on to the face without dragging the end piece
END_LIGHT = 0.02
WIPE_Z_END = TOP + 0.022       # TCP for that last stretch toward the clip: fingertips ~8 mm up, so the fingers' wider part
                               # (14 mm above the tip) clears the 15 mm clip; the pads still reach the cord's centre line
                               # (the real 6 mm pad folded under the tip covers the rest)
S1_END = 0.590   # stop the cord stroke here: pad centre 10 mm short of the clip face, the finger's
S2_END = 0.040 if SHORT else 0.010   # leading edge at the face (closed pads must not run onto an end piece)
STEP = 0.015
END_TO_END = PLUG.size[0] / 2 + LEAD_LEN + CLIP.size[0] / 2   # plug centre to clip centre on a straight lead, 0.63
LEAD_TAUT = 0.628              # measured plug centre to clip centre after a stroke has run up to the clip
CLIP_F = float(os.environ.get("WIPE_CF", -0.0105))   # bare fingers 21 mm apart on the 15 mm clip (two 6 mm wipe pads press on it)
PLUG_F = float(os.environ.get("WIPE_PF", -0.011))    # bare fingers 22 mm apart on the 16 mm plug
OFF_TIP = 0.022                # a body pass ends with the pad centre 2.2 cm past the tip (pads fully off it)
BODY_T = float(os.environ.get("WIPE_BT", 4.0))       # body pass duration, s
CLIP_DZ = float(os.environ.get("WIPE_BZC", 0.0085))  # body pass TCP above the cord-wiping height: clip, TCP about
PLUG_DZ = float(os.environ.get("WIPE_BZP", 0.004))   # 3 mm above the body's top (the space between the fingers ends there)


def meas_tcp(ctx, side: str) -> np.ndarray:
    """TCP from the measured joint angles (robot state), not the command."""
    from yam_arena import ik

    q = np.asarray(ctx.obs.state["joint_pos"], dtype=float).reshape(-1)
    arm = ctx.left if side == "left" else ctx.right
    qa = q[0:6] if side == "left" else q[7:13]
    return ik.tcp_pose(qa, arm.base)[0]


def settle(ctx, side, xy, tol=0.003, tmax=2.5):
    """Wait (at least 0.4 s) until the measured TCP of this arm is within tol of xy, at most tmax: the hand pulling the
    lead straight can lag while it drags the far end piece, and the far end piece is located from it."""
    arm = ctx.left if side == "left" else ctx.right
    yield from arm.wait(0.4)
    t = 0.4
    while t < tmax and float(np.linalg.norm(meas_tcp(ctx, side)[:2] - np.asarray(xy))) > tol:
        yield from arm.wait(0.2)
        t += 0.2


def _cl_dbg():
    return {k: (None if v is None else (round(v['s0'], 3), v['dir'], v['a'] and round(v['a'], 3),
                                        v['b'] and round(v['b'], 3))) for k, v in _ST['cl'].items()}


def _ec_dbg():
    return {k: (None if v is None else (v['name'], round(v['u0'], 3), round(v['umax'], 3), v['ok']))
            for k, v in _ST['ec'].items()}


def body_pass(ctx, arm, f, ctr, length, v, z, log, tag, inward=False, start=-0.012):
    """Wipe an end piece's body (centre estimate ctr, outward direction v). The hand is open and lifted. Set down wide
    open at `start` from the body's face (negative: on the cord short of it; positive: pads straddling the face, so
    closing centres the body between them), close the pads to just over the body's width (f), then slide outward
    along v in one pass over the body and off past its tip. inward (test only): start past the tip and pass back
    toward the cord."""
    yaw = -math.pi / 2
    face = np.array([ctr[0], ctr[1], z]) - v * length / 2
    p0 = face + v * start
    p1 = face + v * (length + OFF_TIP)
    if inward:
        p0, p1 = p1, p0
    # near the edge of its reach the hand settles a few mm off its target: feed back the offset measured at the hover
    # point (joint angles) before setting down
    side = "left" if arm is ctx.left else "right"
    hov = np.array([p0[0], p0[1], p0[2] + 0.03])
    corr = np.zeros(3)
    yield from arm.move(tuple(hov), yaw=yaw, duration=0.6)
    for _ in range(0 if os.environ.get("WIPE_NOCORR") else 2):
        yield from arm.wait(0.8)
        corr = np.clip(corr + hov - meas_tcp(ctx, side), -0.008, 0.008)
        yield from arm.move(tuple(hov + corr), yaw=yaw, duration=0.3)
    p0, p1 = p0 + corr, p1 + corr
    log(f"{tag} side corr={np.round(corr * 1000, 1).tolist()}")
    yield from arm.move(tuple(p0), yaw=yaw, duration=0.5)
    yield from arm.grip(f, 0.5)
    log(f"{tag} set")
    n = 3
    for i in range(1, n + 1):
        yield from arm.move(tuple(p0 + (p1 - p0) * i / n), yaw=yaw, duration=BODY_T / n)
        log(f"{tag} pass {i}/{n}")
    return p1


def centre_piece(ctx, arm, ctr, z, piece, log, tag):
    """Close the wide-open fingers to 2 mm wider than the end piece over its middle and open again (no sliding): the
    piece is pushed to lie centred under the hand, so its position across the lead is known for the passes that
    follow (when a stroke's pads let go, the cord pushed up against the piece can spring it a few cm sideways)."""
    from yam_arena.constants import FINGER_OPEN

    yaw = -math.pi / 2
    yield from arm.move((ctr[0], ctr[1], z + 0.03), yaw=yaw, finger=FINGER_OPEN, duration=0.6)
    yield from arm.move((ctr[0], ctr[1], z), yaw=yaw, duration=0.4)
    yield from arm.grip(-(piece.size[1] + 0.002) / 2, 0.5)
    log(f"{tag} centred")
    yield from arm.grip(FINGER_OPEN, 0.3)
    yield from arm.move((ctr[0], ctr[1], z + 0.03), yaw=yaw, duration=0.3)


def top_pass(ctx, arm, ctr, length, height, v, log, tag, back_clr=0.003, tip_clr=0.003):
    """Top-and-tip pass with the closed finger pads, the fingers turned to line up along the lead (yaw 0): from above,
    come down just behind the cord face (leading finger's outer face 3 mm from it) to 2 mm above the cord, rise to
    7 mm above the top face (bare fingers 3-5 mm clear of it once the hand settles; any closer and they catch it),
    slide outward over the whole top, then come down just past the tip face (trailing
    finger's outer face 3 mm from it) to 2 mm above the table, and move off outward and up. In this turned pose the
    hand settles a few mm off its target, so the offset measured at the hover point (joint angles) is fed back."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    yaw0 = 0.0
    side = "left" if arm is ctx.left else "right"
    zc = ctr[2]
    back = ctr - v * (length / 2 + FINGER_T + back_clr)
    fwd = ctr + v * (length / 2 + FINGER_T + tip_clr)
    top_h, cord_h, low_h = zc + height / 2 + 0.007, zc + R + 0.002, zc - height / 2 + 0.002
    corr = np.zeros(3)

    def go(p, hgt, dur, **kw):
        tgt = np.array([p[0], p[1], hgt + TIP_PAST])      # TCP for the fingertip at height hgt
        yield from arm.move(tuple(tgt + corr), yaw=yaw0, duration=dur, **kw)

    hover = top_h + 0.03
    yield from arm.grip(FINGER_CLOSED, 0.4)              # shut the fingers up clear of the piece, then turn and come over
    yield from go(back, hover, 0.9)
    for _ in range(0 if os.environ.get("WIPE_NOCORR") else 2):
        yield from arm.wait(0.8)
        corr[:] = np.clip(corr + np.array([back[0], back[1], hover + TIP_PAST]) - meas_tcp(ctx, side), -0.008, 0.008)
        yield from go(back, hover, 0.3)
    log(f"{tag} top corr={np.round(corr * 1000, 1).tolist()}")
    yield from go(back, cord_h, 0.7)
    log(f"{tag} top: down the cord face")
    yield from go(back, top_h, 0.4)
    n = 3
    for i in range(1, n + 1):
        yield from go(back + (fwd - back) * i / n, top_h, 0.8)
    log(f"{tag} top: over the top")
    yield from go(fwd, low_h, 0.8)
    log(f"{tag} top: down the tip face")
    out = fwd + v * 0.02
    yield from go(out, low_h, 0.4)
    yield from go(out, hover, 0.4)
    yield from arm.grip(FINGER_OPEN, 0.3)


def demo_script(ctx):
    """Stroke 1: left holds the plug, right closes lightly on the cord at the blue band and slides toward the clip
    (closed loop on the measured hand distance) until its pads are 1.5 cm short of the clip face. Clip body: right
    opens and lifts, left pulls the lead straight, right sets down astride the clip's cord face, closes to just over
    the clip's width and slides over the clip and off its tip; then it takes the clip by its centre. Both carry the
    lead 8 cm further to the left; the left puts the plug down. Stroke 2: left closes on the cord at the blue band and
    slides toward the plug. Plug body: left opens, right pulls the lead straight, left sets down short of the plug
    face, closes to just over the plug's width and slides over the plug and off its tip. Left re-takes the plug;
    undersides(): each end piece is held up rolled a quarter turn while the other hand wipes its underside; both lay
    the lead on the drape and let go."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("WIPE_CHEAT", "")
    # test only: none|hover|onearm|hops|scrub|back|hold|midonly|short2|noclip|noplug|clipin|shortend|notop|nounder|nostrip|noslide
    yaw = -math.pi / 2       # fingers close along x (across the lead)
    hi = TOP + 0.07
    wf = FINGER_OPEN if cheat == "hover" else WIPE_F
    cf = FINGER_OPEN if cheat == "hover" else CLIP_F
    pf = FINGER_OPEN if cheat == "hover" else PLUG_F

    def log(tag):
        st = ctx.states()
        hs = hands()
        hh = {k: (np.round(v[0], 3).tolist(), round(v[4] * 1000, 1)) for k, v in hs.items()}
        pts, ss = _dense_s(st["lead"])
        pr = {k: (None if _pinch(v, pts, ss) is None else round(_pinch(v, pts, ss)[1], 3)) for k, v in hs.items()}
        print(f"[demo t={ctx.t:5.1f}] {tag}: score={score(st):.2f} {progress(st)} pinch={pr} "
              f"back={_ST['back_info']} hands={hh} plug={np.round(st['lead'].start_plug, 3).tolist()} "
              f"clip={np.round(st['lead'].end_plug, 3).tolist()} cl={_cl_dbg()} ec={_ec_dbg()} un={_ST.get('un_dbg')} "
              f"up={ {k: np.round(r[:, 2], 2).tolist() for k, r in piece_rots().items()} }",
              flush=True)

    log("start")
    if cheat == "none":
        yield from L.wait(3.0)
        log("end")
        return
    pl = np.array([LEAD_X, PLUG_Y, TOP + PLUG.size[2] / 2])        # known layout
    b = np.array([LEAD_X, CORD_Y0 - S_START, WIPE_Z])
    take_plug = cheat != "onearm"
    # 1) left takes the plug, right closes lightly around the cord at the blue band
    if take_plug:
        yield from ctx.both(L.move((pl[0], pl[1], hi), yaw=yaw, finger=FINGER_OPEN, duration=1.5),
                            Rt.move((b[0], b[1], hi), yaw=yaw, finger=FINGER_OPEN, duration=1.5))
        yield from ctx.both(L.move((pl[0], pl[1], pl[2] + OVER), yaw=yaw, duration=1.0),
                            Rt.move(tuple(b), yaw=yaw, duration=1.0))
        yield from ctx.both(L.grip(FINGER_CLOSED), Rt.grip(wf))
        yield from ctx.both(L.move((pl[0], pl[1], LIFT), yaw=yaw, duration=1.0), Rt.wait(1.0))
    else:
        yield from Rt.move((b[0], b[1], hi), yaw=yaw, finger=FINGER_OPEN, duration=1.5)
        yield from Rt.move(tuple(b), yaw=yaw, duration=1.0)
        yield from Rt.grip(wf)
    log("pinched")

    def s_right():
        lt = meas_tcp(ctx, "left") if take_plug else pl
        return float(np.linalg.norm((meas_tcp(ctx, "right") - lt)[:2])) - PINCH_OFF

    # 2) stroke 1: toward the clip
    pos = b.copy()
    s1_end = 0.55 if cheat == "midonly" else S1_END
    if cheat == "hops":
        hop = float(os.environ.get("WIPE_HOP", "0.10"))
        while pos[1] > CORD_Y0 - s1_end:
            yield from Rt.grip(FINGER_OPEN, 0.4)
            yield from Rt.move((pos[0], pos[1], WIPE_Z + 0.03), yaw=yaw, duration=0.5)
            pos[1] -= hop
            yield from Rt.move((pos[0], pos[1], WIPE_Z + 0.03), yaw=yaw, duration=0.6)
            yield from Rt.move(tuple(pos), yaw=yaw, duration=0.5)
            yield from Rt.grip(WIPE_F, 0.4)
            yield from Rt.move(tuple(pos), yaw=yaw, duration=0.4)
            log("hop")
    else:
        light, stall = False, 0
        for k in range(90):
            s = s_right()
            if s >= s1_end:
                break
            if s >= s1_end - END_LIGHT and not light and cheat != "hover":
                # the last 2 cm of cord is stiff against the clip: pads eased to just round it, so they slide on to
                # the face instead of dragging the clip (and the lead out of the left hand)
                yield from Rt.grip(WIPE_F_END, 0.3)
                pos[2] = WIPE_Z_END
                yield from Rt.move(tuple(pos), yaw=yaw, duration=0.4)
                light = True
                s = s_right()
            fine = s >= s1_end - 0.03
            pos[1] -= min(0.003 if light else 0.005 if fine else STEP, s1_end - s + 0.002)
            yield from Rt.move(tuple(pos), yaw=yaw, duration=0.3)
            if fine:
                yield from Rt.wait(0.25)
            if light:
                log(f"wipe1 light s={s:.3f}")
            if light:   # the hand was sent on but the slide did not advance, twice: it is blocked or dragging; stop
                stall = stall + 1 if s_right() - s < 0.001 else 0
                if stall >= 2:
                    log("wipe1 stopped: dragging")
                    break
            if k % 6 == 0:
                log(f"wipe1 k={k} s={s:.3f}")
            if cheat == "scrub" and s > 0.35:   # closed stroke back 8 cm and forward again
                yield from Rt.move(tuple(pos + np.array([0.0, 0.08, 0.0])), yaw=yaw, duration=1.0)
                yield from Rt.move(tuple(pos), yaw=yaw, duration=1.0)
                log("scrubbed back")
                cheat = "scrubbed"
            if cheat == "back" and s > 0.35:    # open, go back 8 cm along the cord, close and wipe on
                yield from Rt.grip(FINGER_OPEN, 0.4)
                pos[1] += 0.08
                yield from Rt.move((pos[0], pos[1], WIPE_Z + 0.03), yaw=yaw, duration=0.8)
                yield from Rt.move(tuple(pos), yaw=yaw, duration=0.5)
                yield from Rt.grip(WIPE_F, 0.4)
                log("went back 8 cm")
                cheat = "went_back"
    log("wiped1")
    lay_y = 0.16
    if cheat == "midonly" or not take_plug:
        # old procedure: only the band-to-55 cm stretch, then lay the lead down
        yield from ctx.both(L.wait(0.1), Rt.grip(WIPE_F, 0.2))
        dist = float(np.linalg.norm((meas_tcp(ctx, "right") - meas_tcp(ctx, "left"))[:2]))
        yield from lay(ctx, yaw, lay_y, dist, take_plug)
        return
    # 3) open where the stroke stopped (the pads' edges sweep the clip's cord face) and lift.
    z_clip = TOP + CLIP.size[2] / 2 + OVER
    #    The left hand has been pulled a little toward the clip by the stroke; it is re-sent to where it is first, so
    #    it does not spring back and drag the lead when the right hand lets go.
    lt = meas_tcp(ctx, "left")
    yield from L.move(tuple(lt), yaw=yaw, duration=0.3)
    yield from L.wait(0.2)
    rt = meas_tcp(ctx, "right")
    yield from Rt.grip(FINGER_OPEN, 0.4)
    yield from Rt.move((rt[0], rt[1] + 0.03, z_clip + 0.02), yaw=yaw, duration=0.6)
    yield from L.wait(0.4)
    log("opened1")
    # the left hand draws the lead 5 cm toward the plug side, so the clip lies where the right hand works accurately
    # (further out, at this wrist turn, it settles off target). The stroke has left the lead taut and straight, so
    # the clip centre is located from the left hand: 62.8 cm away (63 cm on a fully straight lead).
    lt0 = meas_tcp(ctx, "left")
    yield from L.move((lt0[0], lt0[1] + 0.05, lt0[2]), yaw=yaw, duration=0.8)
    yield from settle(ctx, "left", (lt0[0], lt0[1] + 0.05))
    lt = meas_tcp(ctx, "left")
    clip_est = np.array([rt[0], lt[1] - LEAD_TAUT, z_clip])   # x: in line with the right hand's pads at the stop
    log("drawn")

    def clip_from_left():
        return clip_est.copy()

    # 3b) clip body, the plug still held: one pass from the cord over the clip and off past its tip
    yield from centre_piece(ctx, Rt, clip_est, z_clip, CLIP, log, "clip")
    if cheat != "notop":
        clip_c = clip_from_left()
        clip_c[2] = TOP + CLIP.size[2] / 2
        # aimed 0.5 mm off the cord face and 4.5 mm off the tip face: here the right hand settles ~4 mm toward the plug
        yield from top_pass(ctx, Rt, clip_c, CLIP.size[0], CLIP.size[2], np.array([0.0, -1.0, 0.0]), log, "clip",
                            back_clr=0.0005, tip_clr=0.0045)
    if cheat != "noclip":
        yield from body_pass(ctx, Rt, cf, clip_from_left(), CLIP.size[0], np.array([0.0, -1.0, 0.0]),
                             WIPE_Z + CLIP_DZ, log, "clip", inward=(cheat == "clipin"), start=-0.004)
        yield from Rt.grip(FINGER_OPEN, 0.4)
        rt = meas_tcp(ctx, "right")
        yield from Rt.move((rt[0], rt[1], z_clip + 0.05), yaw=yaw, duration=0.5)
        yield from L.wait(0.3)
    log("clip wiped")
    # 3c) take the clip by its centre as the anchor for stroke 2
    clip_c = clip_from_left()
    yield from Rt.move((clip_c[0], clip_c[1], z_clip + 0.02), yaw=yaw, duration=0.6)
    yield from Rt.move(tuple(clip_c), yaw=yaw, duration=0.4)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    u_r = 0.0
    log("clip held")
    # 4) carry the lead 8 cm further toward +y, put the plug down, left lets go
    lt, rt = meas_tcp(ctx, "left"), meas_tcp(ctx, "right")
    shift = 0.08
    yield from ctx.both(L.move((LEAD_X, lt[1] + shift, LIFT), yaw=yaw, duration=1.5),
                        Rt.move((LEAD_X, rt[1] + shift, z_clip), yaw=yaw, duration=1.5))
    yield from L.move((LEAD_X, lt[1] + shift, pl[2] + OVER), yaw=yaw, duration=0.6)
    yield from L.grip(FINGER_OPEN, 0.5)
    yield from L.move((LEAD_X, lt[1] + shift, hi), yaw=yaw, duration=0.6)
    log("plug down")
    # 5) stroke 2: close at the blue band, slide toward the plug
    rt = meas_tcp(ctx, "right")
    s_r = LEAD_LEN + CLIP.size[0] / 2 + u_r       # arclength of the right TCP (on the clip)
    v = np.array([0.0, 1.0, 0.0])

    def s_left():
        return s_r - float(np.linalg.norm((meas_tcp(ctx, "left") - meas_tcp(ctx, "right"))[:2])) + PINCH_OFF - 0.0175

    yield from Rt.move((rt[0], rt[1], LIFT), yaw=yaw, duration=0.6)
    rt = meas_tcp(ctx, "right")
    p2 = rt + v * (s_r - S2_START)
    pos = np.array([LEAD_X, p2[1], WIPE_Z2])
    yield from L.move((pos[0], pos[1], hi), yaw=yaw, duration=1.0)
    yield from L.move(tuple(pos), yaw=yaw, duration=0.8)
    yield from L.grip(wf, 0.6)
    log("pinched2")
    s2_end = 0.065 if cheat == "short2" else S2_END    # short2: stop 6.5 cm from the plug (tier check)
    light, stall = False, 0
    for k in range(50):
        s = s_left()
        if s <= s2_end:
            break
        if s <= s2_end + END_LIGHT and not light and cheat != "hover":
            yield from L.grip(WIPE_F_END, 0.3)
            light = True
            s = s_left()
        fine = s <= s2_end + 0.03
        pos[1] += min(0.003 if light else 0.005 if fine else STEP, s - s2_end + 0.002)
        yield from L.move(tuple(pos), yaw=yaw, duration=0.3)
        if fine:
            yield from L.wait(0.25)
        if light:
            stall = stall + 1 if s - s_left() < 0.001 else 0
            if stall >= 2:
                log("wipe2 stopped: dragging")
                break
        if k % 4 == 0:
            log(f"wipe2 k={k} s={s:.3f}")
    log("wiped2")
    # 6) open where the stroke stopped (the pads' edges sweep the plug's cord face) and lift. The right hand, pulled a
    #    little toward the plug by the stroke, is re-sent to where it is first so it does not spring back. The left
    #    hand has stopped with its fingers against the plug's cord face, pad centre 11.5 mm from it (measured), so the
    #    plug centre lies that far plus half the plug's length ahead of it, in line with its pads.
    z_plug = TOP + PLUG.size[2] / 2 + OVER
    rt = meas_tcp(ctx, "right")
    yield from Rt.move(tuple(rt), yaw=yaw, duration=0.3)
    yield from Rt.wait(0.2)
    lt, rt = meas_tcp(ctx, "left"), meas_tcp(ctx, "right")
    plug_est = np.array([lt[0], lt[1] + STOP_FACE + PLUG.size[0] / 2 + (S2_END - 0.010), z_plug])  # shortend: 3 cm
    yield from L.grip(FINGER_OPEN, 0.4)
    yield from L.move((lt[0], lt[1] - 0.02, z_plug + 0.02), yaw=yaw, duration=0.6)
    yield from Rt.wait(0.4)
    log("opened2")

    def plug_est_c():
        return plug_est.copy()

    # 6b) plug body, the clip still held: one pass from the cord over the plug and off past its tip
    yield from centre_piece(ctx, L, plug_est, z_plug, PLUG, log, "plug")
    plug_c = plug_est_c() + PLUG_TOP_OFF
    plug_c[2] = TOP + PLUG.size[2] / 2
    # aimed 6.5 mm off the cord face and 0.5 mm inside the tip face: here the left hand settles a further ~3.5 mm out
    yield from top_pass(ctx, L, plug_c, PLUG.size[0], PLUG.size[2], v, log, "plug", back_clr=0.0065, tip_clr=-0.0005)
    if cheat != "noplug":
        yield from body_pass(ctx, L, pf, plug_est_c(), PLUG.size[0], v, WIPE_Z2 + PLUG_DZ, log, "plug",
                             start=-0.004)
        yield from L.grip(FINGER_OPEN, 0.4)
        lt = meas_tcp(ctx, "left")
        yield from L.move((lt[0], lt[1], z_plug + 0.05), yaw=yaw, duration=0.5)
        yield from L.wait(0.3)
    log("plug wiped")

    # 7) undersides: the right rolls the clip a quarter turn (the plug rolls with it); the left wipes both; then the lay
    yield from undersides(ctx, log, cheat, plug_est_c() + PLUG_TOP_OFF)


def tmove(arm, pos, rot, duration, finger=None):
    """Generator: move the TCP to ``pos`` with any grip rotation ``rot`` (slerped from the current pose)."""
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN

    p0, r0 = ik.tcp_pose(arm.q, arm.base)
    f0 = arm.finger
    f1 = f0 if finger is None else finger
    pos = np.asarray(pos, float)
    n = max(1, round(duration * 20))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        q, _, _ = ik.solve(p0 + (pos - p0) * s, ik.matrix_to_quat_xyzw(ik.slerp_matrix(r0, rot, s)), arm.q,
                           base_pos=arm.base, iters=60)
        arm.q = q
        arm.cmd = np.r_[q, float(np.clip((f0 + (f1 - f0) * s) / FINGER_OPEN, 0.0, 1.0))]
        yield
    arm.finger = f1


def turned_grip():
    """The grip holding an end piece by its long sides (fingers closing across the lead), rolled a quarter turn about
    the lead: the fingers point away from the robot and close vertically, so the piece's underside faces away."""
    from yam_arena.demo import grip_rotation

    ry = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])   # -90 deg about y
    return ry @ grip_rotation(-math.pi / 2)


def meas_pose(ctx, side: str):
    from yam_arena import ik

    q = np.asarray(ctx.obs.state["joint_pos"], dtype=float).reshape(-1)
    arm = ctx.left if side == "left" else ctx.right
    return ik.tcp_pose(q[0:6] if side == "left" else q[7:13], arm.base)


def under_pass(ctx, arm, ctr, n, v, piece, log, tag, cheat=""):
    """Underside pass with the shut fingers (closing across the lead, the yaw used on the cord) on an end piece held
    rolled on its side, its underside facing n (horizontal), its tip along v, ctr its estimated centre: from a hover
    point 4.5 cm out past the underside, come straight in beside the cord about 3.5 cm short of the cord face (the
    fingers' outer face ~3 mm off the cord, fingertip 1 mm above the piece's lower edge so the fingers' wider part
    about clears it), creep toward the face until it is felt (measured TCP lagging the command), sweep straight out
    across the strip of the face under the cord root until the outer face is 9 mm past the underside's plane, come
    back to 5 mm past it, slide along the underside until the fingers are past the tip, and move off outward and up."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    yaw = -math.pi / 2
    side = "left" if arm is ctx.left else "right"
    half, wh, hh = piece.size[0] / 2, piece.size[1] / 2, piece.size[2] / 2
    z = max(ctr[2] - wh + 0.001, TOP + 0.002) + TIP_PAST   # fingertip 1 mm above the lower edge: the wider part
    #                                                      of the fingers (14 mm up) then about clears the piece

    def P(u, e):
        p = ctr + v * u + n * e
        return np.array([p[0], p[1], z])

    # the fingers' outer face 2.8 mm off the cord while finding the face, then 5 mm out past the underside's plane
    u0, u1 = -half - FINGER_HW + 0.002, half + FINGER_HW + 0.008
    ec, e1 = R + 0.0028 + FINGER_T, hh + 0.005 + FINGER_T
    yield from arm.grip(FINGER_OPEN if cheat == "hover" else FINGER_CLOSED, 0.4)
    hov = P(u0 - 0.035, ec) + n * 0.045   # straight out from where the pass starts (no move back along the piece)
    corr = np.zeros(3)
    yield from arm.move(tuple(hov + np.array([0.0, 0.0, 0.07])), yaw=yaw, duration=0.9)   # over it, clear of the holder
    yield from arm.move(tuple(hov + np.array([0.0, 0.0, 0.03])), yaw=yaw, duration=0.4)
    yield from arm.move(tuple(hov), yaw=yaw, duration=0.5)
    for _ in range(0 if os.environ.get("WIPE_NOCORR") else 2):
        yield from arm.wait(0.6)
        corr = np.clip(corr + hov - meas_tcp(ctx, side), -0.008, 0.008)
        yield from arm.move(tuple(hov + corr), yaw=yaw, duration=0.3)
    log(f"{tag} under corr={np.round(corr * 1000, 1).tolist()}")
    if cheat == "nostrip":   # test: skip the strip under the cord root, come in level with the underside instead
        yield from arm.move(tuple(P(u0 - 0.015, e1) + corr), yaw=yaw, duration=0.6)
        yield from arm.move(tuple(P(u0, e1) + corr), yaw=yaw, duration=0.4)
    else:
        # creep along the lead toward the cord face beside the cord until the fingers stop against it (felt as the
        # measured TCP lagging the command): the piece may sit up to a centimetre off centre in the holding hand
        us = u0 - 0.035
        yield from arm.move(tuple(P(us, ec) + corr), yaw=yaw, duration=0.6)
        yield from arm.wait(0.3)
        base_lag = us - float((meas_tcp(ctx, side) - P(0.0, ec)) @ v)
        uc = u0
        for k in range(1, 61):
            u = us + 0.001 * k
            yield from arm.move(tuple(P(u, ec) + corr), yaw=yaw, duration=0.1)
            lag = u - float((meas_tcp(ctx, side) - P(0.0, ec)) @ v) - base_lag
            if lag > 0.004:
                uc = u - lag + 0.001          # just touching the face, free to slide out along it
                break
        log(f"{tag} under: face found {(uc - u0) * 1000:+.1f} mm from the hold estimate")
        yield from arm.move(tuple(P(uc, ec) + corr), yaw=yaw, duration=0.3)
        log(f"{tag} under: by the cord root")
        yield from arm.move(tuple(P(uc, e1 + 0.004) + corr), yaw=yaw, duration=0.6)
        yield from arm.wait(0.2)
        log(f"{tag} under: across the root strip")
        yield from arm.move(tuple(P(uc, e1) + corr), yaw=yaw, duration=0.3)
        u0, u1 = uc, u1 + (uc - u0)
    for i in range(1, 1 if cheat == "noslide" else 5):   # noslide (test): the strip only, never along the underside
        yield from arm.move(tuple(P(u0 + (u1 - u0) * i / 4, e1) + corr), yaw=yaw, duration=0.5)
    yield from arm.wait(0.3)
    log(f"{tag} under: along the underside")
    out = P(u0 if cheat == "noslide" else u1, e1 + 0.02) + corr
    yield from arm.move(tuple(out), yaw=yaw, duration=0.4)
    yield from arm.move(tuple(out + np.array([0.0, 0.0, 0.05])), yaw=yaw, duration=0.5)
    yield from arm.grip(FINGER_OPEN, 0.3)


TURN_IN = 0.0055        # held rolled, an end piece's centre sits 5.5 mm out along the fingers from the TCP
HOLD_P = np.array([0.375, 0.005, TOP + 0.04])   # left hand's TCP holding the plug rolled, 4 cm up (low, so the cord
#                                                 runs off its face onto the table instead of looping up)
PLUG_DR = np.array([0.15, 0.12])                # where the plug waits on the drape (as far from the clip as can be)
CLIP_WAIT = np.array([0.22, -0.20])            # the right hand holds the clip here while the left takes the plug
CLIP_SHALLOW = 0.0025                           # rolled, the clip sits this much less far along the fingers
CLIP_IN = 0.012                                 # taken near its tip, the clip's centre ends ~1 cm toward the cord
GRIP_SPAN = 0.094                               # finger gap when wide open (the gripper state reads 0 shut, 1 open)
HOLD_C = np.array([0.31, 0.03, TOP + 0.04])     # right hand's TCP holding the clip rolled


def undersides(ctx, log, cheat, plug_c):
    """The plug lies on the table where it was wiped (plug_c: its centre), the right hand holds the clip. The left
    picks up the plug; the right sets the clip down where it is. The left brings the plug to the middle 4 cm up and
    rolls its wrist a quarter turn about the lead, so the plug's underside faces away from the robot; the right wipes
    the strip of its cord face under the cord and its underside. The left rolls back and puts the plug on the drape.
    The right finds the clip again by feel, takes it near its tip and does the same with it, the left wiping. The right
    rolls back, the left takes the plug again, and both lay the lead straight on the drape and let go."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    yaw = -math.pi / 2
    hi = TOP + 0.07
    z_tab = TOP + CLIP.size[2] / 2 + OVER
    z_plug = TOP + PLUG.size[2] / 2 + OVER
    z_pd = TOP + DRAPE_T + PLUG.size[2] / 2 + OVER
    rt = meas_tcp(ctx, "right")
    clip_t = np.array([rt[0], rt[1]])
    # the left takes the plug; then the clip is set down where it is; the plug lifted and rolled
    yield from L.move((plug_c[0], plug_c[1], z_plug + 0.03), yaw=yaw, finger=FINGER_OPEN, duration=1.0)
    yield from L.move((plug_c[0], plug_c[1], z_plug), yaw=yaw, duration=0.5)
    yield from L.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((clip_t[0], clip_t[1], z_tab), yaw=yaw, duration=1.0)
    yield from Rt.grip(FINGER_OPEN, 0.4)
    yield from Rt.move((clip_t[0], clip_t[1], hi), yaw=yaw, duration=0.5)
    yield from L.move((plug_c[0], plug_c[1], LIFT + 0.02), yaw=yaw, duration=0.4)
    yield from L.move(tuple(HOLD_P + np.array([0.0, 0.0, 0.02])), yaw=yaw, duration=1.2)
    yield from L.move(tuple(HOLD_P), yaw=yaw, duration=0.4)
    yield from tmove(L, HOLD_P, turned_grip(), 1.2)
    yield from L.wait(0.6)
    log("plug turned")
    if cheat != "nounder":
        ph, rh = meas_pose(ctx, "left")
        n = rh[:, 2] / np.linalg.norm(rh[:, 2])          # the holder's fingers' direction = the underside's normal
        yield from under_pass(ctx, Rt, ph + n * TURN_IN, np.array([n[0], n[1], 0.0]), np.array([0.0, 1.0, 0.0]),
                              PLUG, log, "plug", cheat)
    log("plug underside")
    yield from Rt.move((clip_t[0], clip_t[1], hi), yaw=yaw, duration=1.0)
    yield from L.move(tuple(HOLD_P + np.array([0.0, 0.0, 0.02])), yaw=yaw, duration=1.0)
    yield from L.move((PLUG_DR[0], PLUG_DR[1], LIFT + 0.02), yaw=yaw, duration=1.2)
    yield from L.move((PLUG_DR[0], PLUG_DR[1], z_pd), yaw=yaw, duration=0.5)
    yield from L.grip(FINGER_OPEN, 0.4)
    yield from L.move((PLUG_DR[0], PLUG_DR[1], hi), yaw=yaw, duration=0.5)
    log("plug on drape")
    # the clip, let go of, may have sprung up to ~3 cm along itself: the right finds it again by feel and takes it near
    # its tip, so its cord face stands well clear of the right's fingers; it is rolled and the left wipes (the left
    # waits off to its own side, clear of the right arm)
    yield from L.move((0.22, 0.26, hi), yaw=yaw, duration=0.8)
    gy = yield from find_tip(ctx, Rt, clip_t, z_tab, CLIP, -1.0, log, "clip", 0.006)
    yield from Rt.move((clip_t[0], gy, LIFT + 0.02), yaw=yaw, duration=0.4)
    log("clip taken")
    yield from Rt.move(tuple(HOLD_C + np.array([0.0, 0.0, 0.02])), yaw=yaw, duration=1.5)
    yield from Rt.move(tuple(HOLD_C), yaw=yaw, duration=0.4)
    yield from tmove(Rt, HOLD_C, turned_grip(), 1.2)
    yield from Rt.wait(0.6)
    log("clip turned")
    if cheat != "nounder":
        ph, rh = meas_pose(ctx, "right")
        n = rh[:, 2] / np.linalg.norm(rh[:, 2])
        # taken near its tip: its centre lies toward the cord; it also sits 2.5 mm less far along the fingers than the
        # plug does (measured in the sim; the creep below finds the face along the lead, not this offset)
        c_est = ph + n * (TURN_IN - CLIP_SHALLOW) + np.array([0.0, CLIP_IN, 0.0])
        yield from under_pass(ctx, L, c_est, np.array([n[0], n[1], 0.0]), np.array([0.0, -1.0, 0.0]), CLIP, log,
                              "clip", cheat)
    log("clip underside")
    yield from L.move((0.22, 0.26, hi), yaw=yaw, duration=1.0)
    # the right rolls back and carries the clip toward its end of the drape, out of the left arm's way
    yield from Rt.move(tuple(HOLD_C + np.array([0.0, 0.0, 0.02])), yaw=yaw, duration=1.0)
    yield from Rt.move((CLIP_WAIT[0], CLIP_WAIT[1], LIFT + 0.02), yaw=yaw, duration=1.2)
    # the plug, let go of on the drape, may since have sprung or been drawn a few cm along itself: found by feel too
    gy = yield from find_tip(ctx, L, PLUG_DR, z_pd, PLUG, 1.0, log, "plug", 0.015)
    yield from L.move((PLUG_DR[0], gy, LIFT), yaw=yaw, duration=0.4)
    log("ends held")
    yield from lay(ctx, yaw, 0.16, 0.625, True, hold=(cheat == "hold"))


def find_tip(ctx, arm, xy, z, piece, sgn, log, tag, take_in):
    """Generator returning the y to take an end piece at, by feel. The piece lies along y about xy (where it was let
    go), its free tip toward sgn * y, no cord on that side. Close the fingers (across the lead) at points stepping in
    from 2.25 cm past where its tip should be, reading the finger gap: shut means nothing between them; the first
    point where they stop at the piece's width has the fingers' 15 mm tip on the piece's tip end. The piece is then
    taken take_in further in."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    yaw = -math.pi / 2
    y0 = xy[1] + sgn * (piece.size[0] / 2 + 0.0225)
    y = y0
    for _ in range(8):
        yield from arm.move((xy[0], y, z + 0.02), yaw=yaw, finger=FINGER_OPEN, duration=0.4)
        yield from arm.move((xy[0], y, z), yaw=yaw, duration=0.3)
        yield from arm.grip(FINGER_CLOSED, 0.4)
        q = np.asarray(ctx.obs.state["joint_pos"], dtype=float).reshape(-1)
        if float(q[13 if arm is ctx.right else 6]) * GRIP_SPAN >= piece.size[1] / 2:
            break
        yield from arm.grip(FINGER_OPEN, 0.3)
        y -= sgn * 0.01
    log(f"{tag} tip felt {(y - y0) * 1000:+.0f} mm in from the first probe")
    yield from arm.grip(FINGER_OPEN, 0.3)
    y -= sgn * take_in
    yield from arm.move((xy[0], y, z), yaw=yaw, duration=0.3)
    yield from arm.grip(FINGER_CLOSED, 0.5)
    return y


def lay(ctx, yaw, lay_y, dist, take_plug, hold=False):
    from yam_arena.constants import FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    hi = TOP + 0.07
    lay_l = np.array([0.19, lay_y])
    lay_r = np.array([0.19, lay_y - min(dist, 0.66)])
    down = TOP + DRAPE_T + R + OVER + 0.004
    if take_plug:
        yield from ctx.both(L.move((lay_l[0], lay_l[1], LIFT), yaw=yaw, duration=2.0),
                            Rt.move((lay_r[0], lay_r[1], LIFT), yaw=yaw, duration=2.0))
        yield from ctx.both(L.move((lay_l[0], lay_l[1], down + 0.004), yaw=yaw, duration=1.0),
                            Rt.move((lay_r[0], lay_r[1], down + 0.004), yaw=yaw, duration=1.0))
    else:
        yield from Rt.move((lay_r[0], lay_r[1], LIFT), yaw=yaw, duration=2.0)
        yield from Rt.move((lay_r[0], lay_r[1], down), yaw=yaw, duration=1.0)
    st = ctx.states()
    print(f"[demo t={ctx.t:5.1f}] laid: score={score(st):.2f} {progress(st)} back={_ST['back_info']}", flush=True)
    if hold:
        yield from L.wait(4.0)
        st = ctx.states()
        print(f"[demo t={ctx.t:5.1f}] end (held): score={score(st):.2f} {progress(st)}", flush=True)
        return
    yield from ctx.both(L.grip(FINGER_OPEN), Rt.grip(FINGER_OPEN))
    yield from ctx.both(L.move((lay_l[0], lay_l[1], hi + 0.02), yaw=yaw, duration=0.8),
                        Rt.move((lay_r[0], lay_r[1], hi + 0.02), yaw=yaw, duration=0.8))
    for _ in range(4):
        yield from L.wait(0.5)
    st = ctx.states()
    print(f"[demo t={ctx.t:5.1f}] end: score={score(st):.2f} {progress(st)} back={_ST['back_info']} "
          f"unwiped={MARKS[~_ST['wiped']].tolist()}", flush=True)
