"""Camera turntable on the station table. A grey turntable disc (120 mm diameter, 20 mm thick, 100 g) turns on a fixed
vertical axle at x 30 cm, y -7 cm, resting on the table (light bearing drag). A red arrow on its top (where the camera
would face) points away from the robot (+x); a grey knob (20 mm diameter, 25 mm tall, on its own pin) stands on the
disc 4 cm from the axle on the right (-y). A green tape mark lies on the table at the disc's robot-side edge, opposite
the arrow. The black 6 mm camera cable (56 cm) leaves a swivelling strain relief on the disc's far rim (x 36.35, y -7)
tangentially to the left (+y), passes 3 cm beyond a fixed grey post (25 mm diameter, 45 mm tall, at x 33.3, y 7) and
runs through a fixed cable clip (18 x 16 mm opening) at y 18; beyond the clip it lies loose to y 49. Turning the disc
half a turn clockwise winds the cable round the disc; turning it anticlockwise unwinds it but sweeps it into the post,
where it hooks round the post's far side unless it is first moved over the post to the robot's side.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants: TT_DEMO=correct (default) | nothing | ccw_no_reroute | cw | reroute_only | hold;
scene check TT_CLIP=mouth (clip turned beside the cable: the cable lies along its mouth, not through it).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Turn the camera turntable half a turn so its red arrow points at the green mark, then let go. Its "
               "cable must not end up wound round the turntable or wrapped round the grey post.")

GREY = (0.55, 0.56, 0.58)
DARK = (0.30, 0.31, 0.33)
RED = (0.85, 0.10, 0.08)
GREEN = (0.15, 0.65, 0.25)
BLACK = (0.04, 0.04, 0.04)

# -- turntable: free disc on a vertical D6 axle (see _install_spawn_hook) ------------------------------------------
C = np.array([0.30, -0.07])  # axle
DISC_R, DISC_H = 0.06, 0.020
DISC_Z = TOP + 0.0005 + DISC_H / 2  # settles onto the table: its weight gives bearing friction
DISC = Cylinder("turntable", pos=(float(C[0]), float(C[1]), DISC_Z), radius=DISC_R, height=DISC_H, mass=0.10,
                color=GREY)
DISC_TOP = DISC_Z + DISC_H / 2
KNOB_RK = 0.040  # knob axis distance from the axle
KNOB_A0 = -math.pi / 2  # knob starts on the right (-y) side
KNOB_R, KNOB_H = 0.010, 0.025
KNOB_P0 = np.array([C[0] + KNOB_RK * math.cos(KNOB_A0), C[1] + KNOB_RK * math.sin(KNOB_A0), DISC_TOP + KNOB_H / 2 + 0.0003])
KNOB = Cylinder("knob", pos=tuple(float(v) for v in KNOB_P0), radius=KNOB_R, height=KNOB_H, mass=0.01, color=DARK)
DAMPING = 0.004  # N m s/rad on the axle (light bearing friction)

# -- fixed post, cable clip and green mark ----------------------------------------------------------------------------
P = C + np.array([0.033, 0.14])
POST_R, POST_H = 0.0125, 0.045
POST = Cylinder("post", pos=(float(P[0]), float(P[1]), TOP + POST_H / 2), radius=POST_R, height=POST_H, color=DARK)

R = 0.003  # 6 mm cable
CORD_X = C[0] + DISC_R + R + 0.0005  # 0.3635: leaves the far rim tangentially
CORD_Z = TOP + R + 0.0005
CORD_START = (float(CORD_X), float(C[1]), float(CORD_Z))
CORD_YAW = math.pi / 2
CORD_LEN = 0.56
CORD = cable("camera_cable", start=CORD_START, yaw=CORD_YAW, length=CORD_LEN, radius=R, anchor_start=False,
             bend_stiffness=0.01, bend_damping=0.001, color=BLACK)

CLIP_Y = float(C[1]) + 0.25
CLIP_W, CLIP_HGT, CLIP_T, CLIP_L = 0.018, 0.016, 0.006, 0.020
# Test-only scene variant TT_CLIP=mouth (scorer check, never used for a real trial): the clip is turned 90 deg and
# set beside the cable with its mouth 1 mm from it, so the cable lies along the clip's mouth (its centre 14 mm from
# the clip's centre, inside a 15 mm top-view proximity radius) but does not pass through the jaw.
CLIP_MOUTH = os.environ.get("TT_CLIP", "") == "mouth"
B = np.array([CORD_X + (CLIP_L / 2 + R + 0.001 if CLIP_MOUTH else 0.0), CLIP_Y])  # clip centre (top view)
CLIP_AXIS = np.array([1.0, 0.0]) if CLIP_MOUTH else np.array([0.0, 1.0])  # direction of the clip's tunnel


def _clip() -> list[Box]:
    x, y = B
    wx = CLIP_W / 2 + CLIP_T / 2
    if CLIP_MOUTH:  # same clip, tunnel along x
        return [
            Box("clip_left", pos=(x, y + wx, TOP + CLIP_HGT / 2), size=(CLIP_L, CLIP_T, CLIP_HGT), color=DARK),
            Box("clip_right", pos=(x, y - wx, TOP + CLIP_HGT / 2), size=(CLIP_L, CLIP_T, CLIP_HGT), color=DARK),
            Box("clip_top", pos=(x, y, TOP + CLIP_HGT + 0.002), size=(CLIP_L, CLIP_W + 2 * CLIP_T, 0.004),
                color=DARK),
        ]
    return [
        Box("clip_left", pos=(x - wx, y, TOP + CLIP_HGT / 2), size=(CLIP_T, CLIP_L, CLIP_HGT), color=DARK),
        Box("clip_right", pos=(x + wx, y, TOP + CLIP_HGT / 2), size=(CLIP_T, CLIP_L, CLIP_HGT), color=DARK),
        Box("clip_top", pos=(x, y, TOP + CLIP_HGT + 0.002), size=(CLIP_W + 2 * CLIP_T, CLIP_L, 0.004), color=DARK),
    ]


MARK = Box("green_mark", pos=(float(C[0] - DISC_R - 0.012), float(C[1]), TOP + 0.0005), size=(0.014, 0.024, 0.001),
           color=GREEN)

OBJECTS = [DISC, KNOB, POST, *_clip(), MARK, CORD]


# -- axle, knob pin, cable weld and the arrow / connector decals ------------------------------------------------------
# cablekit can only weld a cable end to the world, so the cable's spawn function is wrapped: right after the cable is
# authored (it is the last object), the disc gets its vertical axle to the world (D6 joint, yaw free), the knob its
# own vertical pin on the disc (a firm pinch must not lock the disc), and the cable's first segment is pinned to the
# disc's rim by a vertical swivel (revolute, -200..+20 deg). All are excluded from the cable articulation (PhysX solves them as external constraints).

def _install_spawn_hook() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_turntable_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORD.name:
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        disc, knob = f"{parent}/{DISC.name}", f"{parent}/{KNOB.name}"
        w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()

        # axle: a D6 joint free in yaw and in height (+/-3 mm) only, so the disc's weight rests on the table and table
        # friction (a turntable's bearing drag) holds it wherever it is left; plus light viscous damping
        ax = UsdPhysics.Joint.Define(stage, f"{prim_path}/turntable_axle")
        ax.CreateBody1Rel().SetTargets([Sdf.Path(disc)])
        ax.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + C[0]), float(w[1] + C[1]), float(w[2] + DISC_Z)))
        ax.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        ax.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        ax.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        ax.CreateExcludeFromArticulationAttr(True)
        for dof in ("transX", "transY", "rotX", "rotY"):
            lim = UsdPhysics.LimitAPI.Apply(ax.GetPrim(), dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        lim = UsdPhysics.LimitAPI.Apply(ax.GetPrim(), "transZ")
        lim.CreateLowAttr(-0.003)
        lim.CreateHighAttr(0.003)
        drv = UsdPhysics.DriveAPI.Apply(ax.GetPrim(), "rotZ")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(0.0)
        drv.CreateDampingAttr(DAMPING * math.pi / 180)  # USD angular gains are per degree

        pin = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/knob_pin")
        pin.CreateBody0Rel().SetTargets([Sdf.Path(disc)])
        pin.CreateBody1Rel().SetTargets([Sdf.Path(knob)])
        pin.CreateAxisAttr("Z")
        off = KNOB_P0 - np.array([C[0], C[1], DISC_Z])
        pin.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in off)))
        pin.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        pin.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        pin.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        pin.CreateExcludeFromArticulationAttr(True)

        # the cable leaves the rim through a swivelling strain relief: a vertical pin that lets the cable exit turn
        # from tangential (as spawned) through radial to the other tangent, never into the disc
        weld = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/disc_weld")
        weld.CreateAxisAttr("Z")
        weld.CreateLowerLimitAttr(-200.0)
        weld.CreateUpperLimitAttr(20.0)
        weld.CreateBody0Rel().SetTargets([Sdf.Path(disc)])
        weld.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        off = np.asarray(CORD_START) - np.array([C[0], C[1], DISC_Z])
        weld.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in off)))
        weld.CreateLocalRot0Attr(Gf.Quatf(math.cos(CORD_YAW / 2), 0.0, 0.0, math.sin(CORD_YAW / 2)))
        weld.CreateLocalPos1Attr(Gf.Vec3f(-CORD.seg_len / 2, 0.0, 0.0))
        weld.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        weld.CreateExcludeFromArticulationAttr(True)
        fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(disc))
        fp.CreateFilteredPairsRel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])

        # decals: visual-only children of the disc body (no collider), so they turn with it
        for nm, pos, size, col in (
            ("arrow", (DISC_R * 0.45, 0.0, DISC_H / 2 + 0.001), (DISC_R * 0.9, 0.010, 0.002), RED),
            ("arrow_tip", (DISC_R * 0.80, 0.0, DISC_H / 2 + 0.0015), (0.014, 0.022, 0.002), RED),
            ("connector", (DISC_R - 0.002, 0.0, -0.004), (0.010, 0.014, 0.010), BLACK),
        ):
            cp = f"{disc}/{nm}"
            cube = UsdGeom.Cube.Define(stage, cp)
            cube.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(cube.GetPrim())
            xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
            xf.AddScaleOp().Set(Gf.Vec3f(*size))
            look = f"{disc}/Looks_{nm}"
            mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(col), roughness=0.5)
            mat.func(look, mat)
            bind_visual_material(cp, look, stage=stage)
        return prim

    spawn._turntable_hook = True
    cablekit._spawn_cable = spawn


_install_spawn_hook()

# -- grippers (score() needs them for the release check) --------------------------------------------------------------
_ENV: dict = {"env": None}


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (no embodiment: offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_turntable_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._turntable_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[np.ndarray] | None:
    """World positions of both grippers' TCP (0.134 m along the gripper axis); None if unknown (then the release
    check fails: a missing gripper reading never counts as let go)."""
    env = _ENV["env"]
    if env is None:
        return None
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        a = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + a * 0.134)
    return out


# -- scoring ----------------------------------------------------------------------------------------------------------
# All checks use the cable's 3D centre line (segment centres and both tips, resampled every 4 mm).
FULL_DEG = 10.0  # arrow within 10 deg of the mark: 1.0
HALF_DEG = 30.0  # within 30 deg: 0.5
WIND_MAX = 180.0  # cable swept half a turn or more round the turntable axle (connector to clip): wound, 0
POST_WRAP_MAX = 180.0  # cable winds half a turn or more round the post's axis (connector to clip): wrapped, 0
RELEASE_DIST = 0.03
DENSE = 0.004  # m between resampled centre-line points
CLIP_EXIT = 0.02  # the cable must go on at least 2 cm beyond both ends of the clip's jaw


def disc_deg(objects) -> float:
    """Turntable yaw (deg, -180..180; 0 = arrow away from the robot, 180 = arrow at the green mark)."""
    x, y, z, w = (float(v) for v in objects[DISC.name].quat_xyzw)
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def arrow_error(objects) -> float:
    return abs(((disc_deg(objects) - 180.0) + 180.0) % 360.0 - 180.0)


def cable_line(objects) -> np.ndarray:
    """(N, 3) cable centre line from the connector to the free end, resampled every DENSE m."""
    poly = np.asarray(objects[CORD.name].polyline(), dtype=float)
    if not np.isfinite(poly).all():
        return np.zeros((0, 3))
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))])
    t = np.arange(0.0, s[-1] + 1e-9, DENSE)
    return np.stack([np.interp(t, s, poly[:, k]) for k in range(3)], axis=1)


def in_jaw(pts: np.ndarray) -> np.ndarray:
    """True for centre-line points inside the clip's jaw: the 18 mm wide, 16 mm high, 20 mm long tunnel between its
    two side walls, under its top bar and above the table (height included, so a cable lying over the clip is out)."""
    d = pts[:, :2] - B
    along = d @ CLIP_AXIS
    across = d @ np.array([-CLIP_AXIS[1], CLIP_AXIS[0]])
    return ((np.abs(along) <= CLIP_L / 2) & (np.abs(across) <= CLIP_W / 2) & (pts[:, 2] >= TOP)
            & (pts[:, 2] <= TOP + CLIP_HGT))


def clip_entry(pts: np.ndarray) -> int | None:
    """Index of the centre-line point where the cable enters the clip's jaw, or None if it does not pass through it:
    a run of consecutive points inside the jaw that spans its length (>= 11 mm along the tunnel), with the points just
    before and after the run on opposite ends of the clip, and at least 2 cm of cable beyond each end."""
    inside = in_jaw(pts)
    along = (pts[:, :2] - B) @ CLIP_AXIS
    n, i = len(pts), 0
    while i < n:
        if not inside[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and inside[j + 1]:
            j += 1
        if (0 < i and j < n - 1 and along[i - 1] * along[j + 1] < 0
                and float(along[i:j + 1].max() - along[i:j + 1].min()) >= CLIP_L - 2 * DENSE - 0.001
                and i * DENSE >= CLIP_EXIT and (n - 1 - j) * DENSE >= CLIP_EXIT):
            return i
        i = j + 1
    return None


def winding_deg(pts2: np.ndarray, q: np.ndarray) -> float:
    """Winding angle (deg) of a top-view path about the vertical axis through ``q``: the sum of the signed angles
    between consecutive points seen from ``q`` (+ = anticlockwise from above). Half a turn or more = the path goes
    round the axis rather than past it."""
    v = pts2 - q
    a = np.arctan2(v[:, 1], v[:, 0])
    d = (np.diff(a) + np.pi) % (2 * np.pi) - np.pi
    return float(np.degrees(d.sum()))


def to_clip(objects) -> np.ndarray | None:
    """Top-view centre line from the connector to where the cable enters the clip; None if not through the clip."""
    pts = cable_line(objects)
    i = clip_entry(pts)
    return None if i is None else pts[: i + 1, :2]


def wind_deg(objects) -> float | None:
    """Winding of the cable (connector to clip) about the turntable axle. About +76 at the start; about +250 after a
    clockwise half turn; about -97 after the demo's anticlockwise turn."""
    p = to_clip(objects)
    return None if p is None else winding_deg(p, C)


def post_wrap_deg(objects) -> float | None:
    """Winding of the cable (connector to clip) about the post's axis. Passing the post on one side gives under half
    a turn (about +150 at the start, about -160 when it passes on the robot's side after the half turn); going round
    the post's far side from the robot-side connector gives about +200; each extra loop adds 360."""
    p = to_clip(objects)
    return None if p is None else winding_deg(p, P)


def released(objects) -> bool:
    gs = grippers()
    if gs is None:  # no reading: not let go
        return False
    pts = objects[CORD.name].polyline()
    for g in gs:
        if float(np.min(np.linalg.norm(pts - g, axis=1))) < RELEASE_DIST:
            return False
        for o in (KNOB, DISC):
            st = objects[o.name]
            if float(np.linalg.norm(np.maximum(np.maximum(st.low - g, g - st.high), 0.0))) < RELEASE_DIST:
                return False
    return True


def _finite(objects) -> bool:
    return all(np.isfinite(objects[o.name].pos).all() for o in (DISC, KNOB)) and bool(
        np.isfinite(objects[CORD.name].points).all())


def score(objects) -> float:
    """0 unless, at the end: no NaN; both grippers at least 3 cm from the cable, disc and knob; the cable passes
    through the clip's jaw (3D); from the connector to the clip it winds less than half a turn about the turntable
    axle and less than half a turn about the post's axis. Then 1.0 if the arrow is within 10 deg of the green mark,
    0.5 if within 30 deg, else 0."""
    if not _finite(objects) or not released(objects):
        return 0.0
    w, pw = wind_deg(objects), post_wrap_deg(objects)
    if w is None or pw is None or abs(w) >= WIND_MAX or abs(pw) >= POST_WRAP_MAX:
        return 0.0
    e = arrow_error(objects)
    return 1.0 if e <= FULL_DEG else 0.5 if e <= HALF_DEG else 0.0


def success(objects) -> bool:
    return bool(score(objects) == 1.0 and objects[DISC.name].resting() and objects[KNOB.name].resting())


def distance(objects) -> float:
    """Arrow error (in half turns), plus 3 if out of the clip, else +1 if wound and +1 if wrapped round the post."""
    if not _finite(objects):
        return 10.0
    w, pw = wind_deg(objects), post_wrap_deg(objects)
    if w is None:
        return arrow_error(objects) / 180.0 + 3.0
    return arrow_error(objects) / 180.0 + float(abs(w) >= WIND_MAX) + float(abs(pw) >= POST_WRAP_MAX)


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    if not _finite(objects):
        return {"finite": 0.0}
    w, pw = wind_deg(objects), post_wrap_deg(objects)
    return {
        "through_clip": float(w is not None),
        "turned": float(max(0.0, 1.0 - arrow_error(objects) / 180.0)),
        "not_wound": float(w is not None and abs(w) < WIND_MAX),
        "post_clear": float(pw is not None and abs(pw) < POST_WRAP_MAX),
        "released": float(released(objects)),
    }


def topview_near_clip(objects) -> bool:
    """Diagnostic only (plain top-view proximity): some cable point within 1.5 cm of the clip centre in top
    view. Logged by the TT_CLIP=mouth check to show that a proximity test alone would pass that variant."""
    return bool(np.linalg.norm(objects[CORD.name].polyline()[:, :2] - B, axis=1).min() <= 0.015)


VARIATION = {
    "cable:camera_cable": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},  # swivel-pinned to the disc rim: moves with the turntable
    "obj:post": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},  # fixed layout (always run with object_jitter=0)
}

# -- camera-based scripted demo (cabledemo.py) ------------------------------------------------------------------------
# Every motion is chosen from the top camera image (ctx.obs, via vision.py), the arms' own TCP poses and the fixed,
# known layout (axle, knob start position, post, clip). Object poses from the simulator are read only by _log(),
# which prints diagnostics and never feeds a motion.
# The right arm pinches the knob at its known start position and swings it anticlockwise round the axle, 12 deg per
# move, to 60 deg: the connector swings towards the clip and the cable goes slack. It holds the turntable there while
# the left arm takes the cable (found in the top image) 3 cm past the post, lifts it over the post, lays it down on
# the robot's side well clear of the post and lets go. The right arm finishes the half turn and lets go; both arms
# park out of the camera's view, the arrow is measured in the top image, and if it is more than 4 deg off the mark
# the right arm re-takes the knob where the image puts it and corrects the turn.

FINGER_KNOB = -0.030
RYAW = math.pi / 2
GRAB_Y = float(P[1]) + 0.035  # take the cable ~3 cm past the post (fingers clear of it)
LAY_X, LAY_Y = 0.215, float(P[1]) + 0.045  # robot side, well clear of the post
LIFT_Z = TOP + 0.075
PRE_TURN = 66.0  # commanded; the turntable lags the gripper by a few degrees
KNOB_GZ = DISC_TOP + KNOB_H - 0.006  # pads pinch most of the knob height
PARK_L = (0.10, 0.44, TOP + 0.10)  # out of the top camera's view of the turntable and cable
PARK_R = (0.10, -0.44, TOP + 0.10)
ARROW_TOL = 4.0
ARROW_Z = DISC_TOP + 0.002


def _log(ctx, tag=""):
    """Diagnostics only (prints sim state; never used to choose a motion)."""
    st = ctx.states()
    print(f"[t={ctx.t:5.2f}] {tag} disc={disc_deg(st):7.1f} err={arrow_error(st):5.1f} wind={wind_deg(st)} "
          f"post_wrap={post_wrap_deg(st)} near_clip={topview_near_clip(st)} rel={released(st)} score={score(st)} "
          f"prog={progress(st)}", flush=True)
    d = os.environ.get("TT_DUMP")
    if d:
        os.makedirs(d, exist_ok=True)
        tg = tag.split()[0] if tag else "x"
        np.save(os.path.join(d, f"cable_{ctx.t:06.2f}_{tg}.npy"), np.asarray(st[CORD.name].polyline()))
        np.save(os.path.join(d, f"top_{ctx.t:06.2f}_{tg}.npy"), np.asarray(ctx.obs.images["top_cam"]))


# --- camera estimates
def _arrow_deg(ctx) -> float | None:
    """Turntable angle (deg, 0 = arrow away from the robot) from the red arrow in the top image: the direction from
    the known axle to the red pixels on the disc's top face."""
    import vision

    img = vision.top_image(ctx)
    m = vision.hsv_mask(img, 0.0, 20.0, min_sat=0.55, min_val=60)
    pts = vision.top_camera(img.shape).to_plane(m, ARROW_Z)
    v = pts - C
    r = np.linalg.norm(v, axis=1)
    v = v[(r > 0.010) & (r < DISC_R + 0.004)]
    if len(v) < 6:
        return None
    mv = v.mean(axis=0)
    return math.degrees(math.atan2(mv[1], mv[0]))


def _cable_at(ctx, y0: float):
    """Where the black cable crosses the line y = y0 near the post (top image, cable plane): (x, heading) of a line
    fitted to the dark grey pixels within 3 cm of y0, or None. The post (dark grey, taller) is masked out."""
    import vision

    img = np.asarray(vision.top_image(ctx), dtype=float)
    mx, mn = img.max(axis=-1), img.min(axis=-1)
    m = (mx < 140) & ((mx - mn) / (mx + 1e-9) < 0.3)  # dark and grey: the 6 mm cable is ~1 px, blended with the table
    pts = vision.top_camera(img.shape).to_plane(m, CORD_Z)
    post_seen = P + np.array([0.022, 0.0])  # the post's top face lands ~2 cm further out on the cable plane
    k = ((np.abs(pts[:, 1] - y0) < 0.03) & (pts[:, 0] > 0.30) & (pts[:, 0] < 0.47)
         & (np.linalg.norm(pts - post_seen, axis=1) > 0.03) & (np.linalg.norm(pts - P, axis=1) > 0.025))
    q = pts[k]
    if len(q) < 2 or np.ptp(q[:, 1]) < 0.008:
        return None
    A = np.stack([np.ones(len(q)), q[:, 1] - y0], axis=1)
    (x0, b), *_ = np.linalg.lstsq(A, q[:, 0], rcond=None)
    if abs(b) > 2.0:
        b = 0.0
    return float(x0), math.atan2(1.0, float(b))  # heading along the cable towards the clip (+y)


def _look(ctx, fn, *a, tries=4):
    for _ in range(tries):
        yield from ctx.left.wait(0.3)
        r = fn(ctx, *a)
        if r is not None:
            return r
    return None


# --- moves
def _knob_xy(disc_angle_deg: float) -> np.ndarray:
    """Knob centre (top view) when the turntable is at ``disc_angle_deg`` (layout: 4 cm from the axle, on the right
    at 0 deg)."""
    a = math.radians(disc_angle_deg) + KNOB_A0
    return C + KNOB_RK * np.array([math.cos(a), math.sin(a)])


def _tcp_disc_deg(ctx) -> float:
    """Turntable angle implied by the right gripper holding the knob (its own TCP, about the known axle)."""
    t = ctx.right.tcp()
    return math.degrees(math.atan2(t[1] - C[1], t[0] - C[0]) - KNOB_A0)


def _take_knob(ctx, angle_deg: float = 0.0):
    """Right arm pinches the knob from above where the layout (and the measured turntable angle) puts it."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    k = _knob_xy(angle_deg)
    yield from Rt.move((k[0], k[1], KNOB_GZ + 0.05), yaw=RYAW, finger=FINGER_KNOB, duration=1.4)
    yield from Rt.move((k[0], k[1], KNOB_GZ), yaw=RYAW, duration=0.8)
    yield from Rt.wait(0.2)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.wait(0.2)


def _turn(ctx, sign: float, total: float, step=12.0, dur=0.35):
    """Swing the held knob ``total`` deg round the axle (sign +1 = anticlockwise), each move rotating the gripper's own
    current position about the axle by up to ``step`` deg (so it follows the knob, not a fixed plan)."""
    Rt = ctx.right
    start = _tcp_disc_deg(ctx)
    done, last = 0.0, start
    for _ in range(int(total / step) + 8):
        rem = total - sign * done
        if rem < 1.0:
            break
        a = last + sign * min(step, rem)
        k = _knob_xy(a)
        yield from Rt.move((k[0], k[1], KNOB_GZ), yaw=RYAW, duration=dur)
        now = _tcp_disc_deg(ctx)
        done += ((now - last + 180.0) % 360.0) - 180.0
        last = now
    yield from Rt.wait(0.3)


def _let_go_right(ctx):
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    yield from Rt.grip(FINGER_KNOB, 0.4)
    p = Rt.tcp()
    yield from Rt.move((p[0], p[1], p[2] + 0.07), yaw=RYAW, duration=0.8)
    yield from Rt.move(PARK_R, yaw=0.0, finger=FINGER_CLOSED, duration=1.4)


def _reroute(ctx):
    """Left arm: find the cable 3 cm past the post in the top image, take it, lift it over the post and lay it down
    on the robot's side, let go and park."""
    seen = yield from _look(ctx, _cable_at, GRAB_Y)
    if seen is None:
        print("[demo] cable not seen; using the layout line", flush=True)
        seen = (float(CORD_X) + 0.02, math.pi / 2)
    for _ in range(3):  # closed loop: re-look after each try; retry while the cable is still beyond the post
        yield from _carry_over_post(ctx, seen)
        again = yield from _look(ctx, _cable_at, GRAB_Y, tries=3)
        if again is None or again[0] < P[0] + POST_R + 0.01:
            break
        print(f"[demo] cable still beyond the post (x={again[0]:.3f}); trying again", flush=True)
        seen = again
    _log(ctx, "rerouted")


def _carry_over_post(ctx, seen):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L = ctx.left
    x0, hd = seen
    gy = hd - math.pi  # fingers across the cable, wrist camera looking back along it at the post and turntable
    while gy < -math.pi:
        gy += 2 * math.pi
    print(f"[demo] cable seen at x={x0:.4f} y={GRAB_Y:.3f} heading={math.degrees(hd):.0f}", flush=True)
    yield from L.move((x0, GRAB_Y, TOP + 0.06), yaw=gy, finger=FINGER_OPEN, duration=1.4)
    yield from L.move((x0, GRAB_Y, CORD_Z + 0.009), yaw=gy, duration=0.8)
    yield from L.wait(0.2)
    yield from L.grip(FINGER_CLOSED, 0.8)
    yield from L.wait(0.3)
    yield from L.move((x0 - 0.01, GRAB_Y, LIFT_Z), yaw=gy, duration=1.4)
    yield from L.move((LAY_X, LAY_Y, LIFT_Z), yaw=gy, duration=2.2)
    yield from L.move((LAY_X, LAY_Y, CORD_Z + 0.012), yaw=gy, duration=1.0)
    yield from L.wait(0.3)
    yield from L.grip(FINGER_OPEN, 0.4)
    yield from L.move((LAY_X, LAY_Y, TOP + 0.09), yaw=gy, duration=0.6)
    yield from L.move(PARK_L, yaw=0.0, finger=FINGER_CLOSED, duration=1.4)


def _correct_arrow(ctx, tries: int = 2):
    """Both arms parked: measure the arrow in the top image; if it is more than ARROW_TOL off the mark, re-take the
    knob where that angle puts it, turn the rest of the way and let go again."""
    for _ in range(tries):
        a = yield from _look(ctx, _arrow_deg)
        if a is None:
            print("[demo] arrow not seen", flush=True)
            return
        err = ((180.0 - a) + 180.0) % 360.0 - 180.0  # + = still short of the mark, anticlockwise
        print(f"[demo] arrow seen at {a:.1f} deg (off the mark by {err:+.1f})", flush=True)
        if abs(err) <= ARROW_TOL:
            return
        yield from _take_knob(ctx, a)
        yield from _turn(ctx, 1.0 if err > 0 else -1.0, abs(err), step=6.0, dur=0.5)
        yield from _let_go_right(ctx)


def demo_script(ctx):
    mode = os.environ.get("TT_DEMO", "correct")  # test only: correct|nothing|ccw_no_reroute|cw|reroute_only|hold
    _log(ctx, f"start ({mode})")
    if mode == "nothing":
        for _ in range(5):
            yield from ctx.left.wait(1.0)
        _log(ctx, "end")
        return
    yield from ctx.left.move(PARK_L, yaw=0.0, duration=1.4)  # out of the camera's view
    if mode == "reroute_only":
        yield from _reroute(ctx)
    elif mode in ("correct", "hold"):
        # turn a third of the way first (the connector swings towards the clip, so the cable goes slack), hold the
        # turntable there while the left arm lifts the slack cable over the post, then finish the turn
        yield from _take_knob(ctx)
        yield from _turn(ctx, 1.0, PRE_TURN)
        _log(ctx, "pre-turned")
        yield from _reroute(ctx)
        yield from _turn(ctx, 1.0, 180.0 - PRE_TURN)
        _log(ctx, "turned")
        if mode == "hold":
            yield from ctx.left.wait(3.0)
            _log(ctx, "end")
            return
        yield from _let_go_right(ctx)
        yield from _correct_arrow(ctx)
    else:  # ccw_no_reroute | cw: the half turn without touching the cable
        yield from _take_knob(ctx)
        yield from _turn(ctx, -1.0 if mode == "cw" else 1.0, 180.0)
        _log(ctx, "turned")
        yield from _let_go_right(ctx)
        yield from _correct_arrow(ctx)
    for _ in range(4):
        yield from ctx.left.wait(0.5)
    _log(ctx, "end")
