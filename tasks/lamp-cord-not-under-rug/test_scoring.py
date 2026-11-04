"""Offline checks of score() on synthetic cord/rug states (no simulator): run `python test_scoring.py`.

Covers the outcomes a scripted robot can't easily produce (cord tucked under the rug, rug lifted onto the cord) next
to the ideal layout, so the zero-score gates are documented and tested."""

import math

import numpy as np

import task
from cablekit import CableState
from yam_arena.task_file import ObjectState

TOP = task.TOP
Z0 = np.zeros(3)


def rug(dx=0.0, dz=0.0):
    c = np.array([task.RUG_XY[0] + dx, task.RUG_XY[1], TOP + task.RUG_SIZE[2] / 2 + dz])
    h = np.array(task.RUG_SIZE) / 2
    return ObjectState(pos=c, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=Z0, ang_vel=Z0, low=c - h, high=c + h)


def cord(path_xy, z=None, plug_depth=0.0225):
    """A cord along the polyline ``path_xy`` (from the lamp) resampled to the kit's segment count, with the plug
    pointing -y from its end and pushed ``plug_depth`` into the socket."""
    path_xy = np.asarray(path_xy, float)
    seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path_xy, axis=0), axis=1))]
    n = task.CORD.segments
    s = (np.arange(n) + 0.5) * task.CORD.seg_len
    s = s * seg[-1] / task.CORD_LEN
    pts = np.stack([np.interp(s, seg, path_xy[:, 0]), np.interp(s, seg, path_xy[:, 1]),
                    np.full(n, TOP + task.R if z is None else z)], axis=1)
    plug = np.array([task.XR, task.FACE_Y + task.PL / 2 - plug_depth, TOP + task.PH / 2])
    d0 = pts[0] - pts[1]
    d1 = plug - pts[-1]
    return CableState(pos=pts.mean(0), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=Z0, ang_vel=Z0, low=pts.min(0),
                      high=pts.max(0), points=pts, end_plug=plug, start_dir=d0 / np.linalg.norm(d0),
                      end_dir=d1 / np.linalg.norm(d1), seg_len=task.CORD.seg_len)


LAMP = (task.CORD_START[0], task.CORD_START[1])
PLUG_BACK = (task.XR, task.FACE_Y + task.PL - 0.0225)
ROUTED = [LAMP, (task.XR, task.RW_Y0 + 0.01), (task.XR, task.RW_Y1), PLUG_BACK]
ACROSS = [LAMP, (0.24, 0.0), (0.30, -0.12), PLUG_BACK]  # plugged in, cord across the walkway
UNDER = [LAMP, (0.25, 0.10), (0.15, 0.0), (0.30, -0.12), PLUG_BACK]  # loop under the rug


def check(name, objs, want):
    got = task.score(objs)
    print(f"{name:34s} score={got:.3f} gates={task.gates(objs)}")
    assert math.isclose(got, want, abs_tol=1e-9), (name, got, want)


if __name__ == "__main__":
    check("routed + plugged", {"cord": cord(ROUTED), "rug": rug()}, 1.0)
    check("routed, plug not in", {"cord": cord(ROUTED, plug_depth=-0.01), "rug": rug()}, 0.5)
    check("routed + plugged, rug moved 4 cm", {"cord": cord(ROUTED), "rug": rug(dx=0.04)}, 0.0)
    check("plugged, cord across walkway", {"cord": cord(ACROSS), "rug": rug()}, 0.0)
    check("plugged, cord loop under rug", {"cord": cord(UNDER, z=TOP + 0.002), "rug": rug(dz=0.004)}, 0.0)
    under_only = {"cord": cord(UNDER, z=TOP + 0.002), "rug": rug(dz=0.004)}
    assert task.cord_under_rug(under_only)
    check("routed, cord on table behind rail", {"cord": cord([LAMP, (0.33, 0.15), (0.33, -0.12), PLUG_BACK]),
                                                 "rug": rug()}, 0.0)
    print("all scorer checks passed")
