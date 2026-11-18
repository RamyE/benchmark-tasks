"""A dairy-case shelf deck at the far side of the station table: a 2.5 cm front price-channel lip, a 5 cm rear lip,
the case's back wall behind it and 4 cm wire lane dividers making five 8 cm lanes, 18 cm deep. The middle lane holds
two chocolate milk pints and the lanes beside it two skim pints each. Each of the two 2% lanes (y +16 and -16 cm, next to the end panels) holds one
older 2% pint (code date 18) at the front; its back slot is empty. Two new 2% pints (code date 27) stand on the
stocking cart's deck in front of the case. The robot must put the new cartons on the shelf so that in each 2% lane
the older carton stands at the front and the new one behind it, all upright, then let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "Put the two new cartons of milk on the shelf."
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

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # 0.7475
SCORE_OVER = "end"

# -- shelf (m; x forward from the arm-base line, y left) ---------------------------------------------------------
DECK_T = 0.005
DECK = TOP + DECK_T  # shelf deck surface
LIP_X0, LIP_X1, LIP_H = 0.205, 0.215, 0.025  # front price channel; inner face at LIP_X1
LANE_X0, LANE_X1 = LIP_X1, 0.395  # 18 cm deep lanes
RLIP_X1, RLIP_H = 0.405, 0.05  # rear lip
WALL_X0, WALL_X1, WALL_H = 0.52, 0.53, 0.30  # case back wall (behind the air-return gap)
DIV_T, DIV_H = 0.003, 0.04
DIV_Y = (-0.2845, -0.2015, -0.115, -0.04, 0.04, 0.115, 0.2015, 0.2845)  # wire lane dividers (outermost: section edge)
CASE_HALF = 0.293  # deck, lips and back wall run y -29.3 to +29.3 (one section of a case lineup: no end panels)
TWO_Y = 0.15825  # 2% lanes: between the dividers at 11.5 and 20.15 cm (8.35 cm inside)
WHOLE_Y = 0.243  # whole-milk lanes: between the dividers at 20.15 and 28.45 cm (8 cm inside)
SKIM_Y = 0.0775  # skim lanes: between the dividers at 4 and 11.5 cm (7.2 cm)
LANE_Y = {"whole_r": -WHOLE_Y, "two_r": -TWO_Y, "skim_r": -SKIM_Y, "choc": 0.0, "skim_l": SKIM_Y, "two_l": TWO_Y,
          "whole_l": WHOLE_Y}
LANE_W = {"whole_r": 0.08, "two_r": 0.0835, "skim_r": 0.072, "choc": 0.077, "skim_l": 0.072, "two_l": 0.0835,
          "whole_l": 0.08}  # inside widths

WHITE = (0.94, 0.94, 0.92)
STEEL = (0.80, 0.81, 0.83)
GREY = (0.45, 0.46, 0.48)
BROWN = (0.45, 0.27, 0.14)
SKIM = (0.55, 0.70, 0.88)
WHOLE = (0.80, 0.22, 0.20)
CART = (0.35, 0.36, 0.38)


def _fixed():
    xc = (LIP_X0 + WALL_X1) / 2
    out = [Box("shelf_deck", pos=(xc, 0.0, TOP + DECK_T / 2), size=(WALL_X1 - LIP_X0, 2 * CASE_HALF, DECK_T), color=WHITE),
           Box("price_lip", pos=((LIP_X0 + LIP_X1) / 2, 0.0, DECK + LIP_H / 2), size=(LIP_X1 - LIP_X0, 2 * CASE_HALF, LIP_H),
               color=GREY),
           Box("rear_lip", pos=((LANE_X1 + RLIP_X1) / 2, 0.0, DECK + RLIP_H / 2), size=(RLIP_X1 - LANE_X1, 2 * CASE_HALF, RLIP_H),
               color=STEEL),
           Box("case_wall", pos=((WALL_X0 + WALL_X1) / 2, 0.0, DECK + WALL_H / 2), size=(WALL_X1 - WALL_X0, 2 * CASE_HALF, WALL_H),
               color=WHITE)]
    for i, y in enumerate(DIV_Y):
        out.append(Box(f"divider_{i}", pos=((LANE_X0 + LANE_X1) / 2, y, DECK + DIV_H / 2),
                       size=(LANE_X1 - LANE_X0, DIV_T, DIV_H), color=STEEL))
    return out


# -- cartons ------------------------------------------------------------------------------------------------------
CW, CH = 0.069, 0.125  # pint gable-top carton: 6.9 x 6.9 cm, 12.5 cm to the fin top (modelled as a box)
CMASS = 0.5
FRONT_X = LANE_X0 + 0.002 + CW / 2  # 0.2515: front face 2 mm behind the lip
BACK_X = LANE_X1 - 0.005 - CW / 2  # 0.3555: back face 5 mm before the rear lip
CZ = DECK + CH / 2 + 0.0005
CART_X0, CART_X1, CART_T = 0.06, 0.20, 0.02
CART_HALF_Y = 0.30
CART_TOP = TOP + CART_T
NEW_X = 0.15
NEW_Y = {"new_l": 0.08, "new_r": -0.08}

OLD = {"old_l": (FRONT_X, LANE_Y["two_l"]), "old_r": (FRONT_X, LANE_Y["two_r"])}
STOCK = {"choc_f": (FRONT_X, 0.0, BROWN), "choc_b": (BACK_X, 0.0, BROWN),
         "skim_lf": (FRONT_X, SKIM_Y, SKIM), "skim_lb": (BACK_X, SKIM_Y, SKIM),
         "skim_rf": (FRONT_X, -SKIM_Y, SKIM), "skim_rb": (BACK_X, -SKIM_Y, SKIM),
         "whole_lb": (BACK_X, WHOLE_Y, WHOLE), "whole_rb": (BACK_X, -WHOLE_Y, WHOLE)}  # whole lanes: front sold through
CODE = {"old_l": "18", "old_r": "18", "new_l": "27", "new_r": "27"}
ANG_DAMP = 20.0  # 1/s, the four 2% cartons only (see _patch_scene)


def _cartons():
    out = []
    for n, (x, y) in OLD.items():
        out.append(Box(n, pos=(x, y, CZ), size=(CW, CW, CH), color=WHITE, mass=CMASS))
    for n, y in NEW_Y.items():
        out.append(Box(n, pos=(NEW_X, y, CART_TOP + CH / 2 + 0.0005), size=(CW, CW, CH), color=WHITE, mass=CMASS))
    for n, (x, y, c) in STOCK.items():
        out.append(Box(n, pos=(x, y, CZ), size=(CW, CW, CH), color=c, mass=CMASS))
    return out


CART_DECK = Box("cart_deck", pos=((CART_X0 + CART_X1) / 2, 0.0, TOP + CART_T / 2),
                size=(CART_X1 - CART_X0, 2 * CART_HALF_Y, CART_T), color=CART)
OBJECTS = [*_fixed(), CART_DECK, *_cartons()]

# -- code dates: black 5x7 digits on each 2% carton's top, read from the robot side (visual only, move with it) -----
GLYPHS = {
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
}
PX = 0.005


def _rects(rows):
    out = []
    for r, row in enumerate(rows):
        c = 0
        while c < len(row):
            if row[c] == "1":
                c0 = c
                while c < len(row) and row[c] == "1":
                    c += 1
                out.append((c0, r, c - c0, 1))
            else:
                c += 1
    return out


def _digit_boxes(text):
    """(centre, size) of each pixel run of ``text`` in the carton frame (top face, letter tops to +x)."""
    w = PX * (5 * len(text) + (len(text) - 1))
    h = 7 * PX
    out = []
    for i, ch in enumerate(text):
        y_left = w / 2 - i * 6 * PX
        for c0, r0, nc, nr in _rects(GLYPHS[ch]):
            out.append(((h / 2 - (r0 + nr / 2) * PX, y_left - (c0 + nc / 2) * PX, CH / 2 + 0.0006),
                        (nr * PX, nc * PX, 0.001)))
    return out


def _add_decals(prim, text):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdGeom

    stage = get_current_stage()
    path = str(prim.GetPath())
    look = f"{path}/Looks/code_ink"
    cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 0.0), roughness=0.6)
    cfg.func(look, cfg)
    for k, (c, s) in enumerate(_digit_boxes(text)):
        cube = UsdGeom.Cube.Define(stage, f"{path}/code_{k}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*c))
        cube.AddScaleOp().Set(Gf.Vec3f(*s))
        bind_visual_material(str(cube.GetPath()), look, stage=stage)


def _patch_scene(scene) -> None:
    if getattr(scene, "_milk_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        obj = orig(spec)
        if getattr(spec, "name", "") in CODE:
            # held cartons: rubber pads pinch a real carton without letting it pivot about the pinch line; the
            # simulated pads touch at a few points, so the four 2% cartons get angular damping instead
            for cfg in (obj.spawner_cfg, getattr(getattr(obj, "object_cfg", None), "spawn", None)):
                if cfg is not None and getattr(cfg, "rigid_props", None) is not None:
                    cfg.rigid_props.angular_damping = ANG_DAMP
        return obj

    scene.task_object = task_object
    scene._milk_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _milk = True

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
    elif not any(getattr(f, "_milk", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()

# -- grippers (release check) --------------------------------------------------------------------------------------
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_milk_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        type(self)._milk_env = self._env  # shared: the task module can be loaded more than once
        return base(self, *a, **k)

    _task_step._milk_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def _runtime_decals(env) -> None:
    """Author the code-date digits under each 2% carton's prim once the scene exists (visual only)."""
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    base = env.unwrapped
    for name, text in CODE.items():
        path = base.scene.rigid_objects[name].cfg.prim_path.replace("env_[^/]+", "env_0").replace("env_.*", "env_0")
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid() and not stage.GetPrimAtPath(f"{path}/code_0").IsValid():
            _add_decals(prim, text)


def _env():
    env = _ENV["env"]
    if env is None:
        try:
            from yam_arena import inspect_embodiment as ie

            env = getattr(ie.YamArenaEmbodiment, "_milk_env", None)
        except Exception:  # noqa: BLE001
            env = None
    return env


def grippers() -> list[np.ndarray] | None:
    env = _env()
    if env is None:
        return None
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- scoring --------------------------------------------------------------------------------------------------------
TILT_MAX = 10.0  # deg
LANE_HALF = 0.04 - DIV_T / 2  # 3.85 cm from lane centre to a divider face
FRONT_MAX = 0.03  # old carton's front face within 3 cm of the lip
STOCK_MOVE = 0.01
RELEASE_DIST = 0.03
TWO_LANES = {"two_l": ("old_l", "new_l"), "two_r": ("old_r", "new_r")}
OLDS, NEWS = ("old_l", "old_r"), ("new_l", "new_r")


def _tilt_deg(q) -> float:
    x, y, z, w = q
    up_z = 1 - 2 * (x * x + y * y)  # z component of the body's z axis
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def _upright_on_deck(o) -> bool:
    p = np.asarray(o.pos)
    return bool(np.isfinite(p).all() and _tilt_deg(o.quat_xyzw) <= TILT_MAX and o.low[2] <= DECK + 0.01
                and LANE_X0 - 0.005 <= p[0] <= LANE_X1 + 0.005 and abs(p[1]) <= CASE_HALF)


def _upright_on_cart(o) -> bool:
    p = np.asarray(o.pos)
    return bool(np.isfinite(p).all() and _tilt_deg(o.quat_xyzw) <= TILT_MAX and o.low[2] <= CART_TOP + 0.01
                and o.low[2] >= CART_TOP - 0.01 and CART_X0 <= p[0] <= CART_X1 and abs(p[1]) <= CART_HALF_Y)


def _lane_of(o):
    """Lane name of an upright carton on the deck (centre between that lane's dividers), else None."""
    if not _upright_on_deck(o):
        return None
    for n, y in LANE_Y.items():
        if abs(o.pos[1] - y) <= (LANE_W[n] - CW) / 2 + 0.004:
            return n
    return None


def _gate(objects) -> str:
    """'' if no score-0 condition holds at the end, else the reason."""
    for n in (*OLDS, *NEWS, *STOCK):
        o = objects[n]
        if not np.isfinite(np.asarray(o.pos)).all():
            return f"{n} unreadable"
        # a 2% carton may stand on the deck or still on the cart (an untouched lane); anything else is a drop
        if not (_upright_on_deck(o) or (n in (*OLDS, *NEWS) and _upright_on_cart(o))):
            return f"{n} not standing on the deck or the cart"
    for n, (x, y, _) in STOCK.items():
        o = objects[n]
        if math.hypot(o.pos[0] - x, o.pos[1] - y) > STOCK_MOVE:
            return f"{n} moved"
    gs = grippers()
    if gs is None or len(gs) != 2:
        return "grippers unreadable"  # fail closed: no end score without the release check
    for g in gs:
        if not np.isfinite(g).all():
            return "gripper unreadable"
        for n in (*OLDS, *NEWS, *STOCK):
            o = objects[n]
            d = np.maximum(np.maximum(o.low - g, g - o.high), 0.0)
            if float(np.linalg.norm(d)) < RELEASE_DIST:
                return f"gripper near {n}"
    return ""


def lane_correct(objects, lane: str) -> bool:
    """One old and one new 2% carton in the lane, old in front, its front face within 2 cm of the lip."""
    inside = [n for n in (*OLDS, *NEWS) if _lane_of(objects[n]) == lane]
    old = [n for n in inside if n in OLDS]
    new = [n for n in inside if n in NEWS]
    if len(old) != 1 or len(new) != 1:
        return False
    o, w = objects[old[0]], objects[new[0]]
    return bool(o.pos[0] < w.pos[0] and o.low[0] - LANE_X0 <= FRONT_MAX)


def score(objects) -> float:
    env = _env()
    if env is not None and not getattr(env, "_milk_decals", False):
        env._milk_decals = True
        try:
            _runtime_decals(env)
        except Exception as e:  # noqa: BLE001
            print(f"[milk] runtime decals failed: {e!r}", flush=True)
    if _gate(objects):
        return 0.0
    return 0.5 * sum(lane_correct(objects, ln) for ln in TWO_LANES)


def success(objects) -> bool:
    return score(objects) >= 1.0 and all(objects[n].resting(max_speed=0.02) for n in (*OLDS, *NEWS))


def distance(objects) -> float:
    return 1.0 - score(objects)


def progress(objects) -> dict:
    return {"gate": _gate(objects) or "ok", **{ln: lane_correct(objects, ln) for ln in TWO_LANES},
            "lanes": {n: _lane_of(objects[n]) for n in (*OLDS, *NEWS)}}
