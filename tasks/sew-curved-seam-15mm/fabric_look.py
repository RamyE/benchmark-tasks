"""Look of the fabric, its chalk marks, the sewn thread and the needle (visual only)."""

from __future__ import annotations

import math

import numpy as np

import seam_machine as M

LIFT = 0.0004  # marks sit this far over the fabric surface; thread sits higher so it never z-fights with chalk
THREAD_LIFT = 0.0009
THREAD_HALF = 0.0016  # 3.2 mm wide contrasting topstitching ribbon so the sewn seam reads clearly on camera
MAX_ST = 300  # pre-allocated fixed mesh topology so Omniverse RTX Hydra streams stitch updates every step
FABRIC_RGB = (0.07, 0.11, 0.26)  # navy quilting cotton
CHALK_RGB = (0.97, 0.97, 0.95)
THREAD_RGB = (1.00, 0.45, 0.05)  # orange topstitching thread


def bil(S, uv):
    uv = np.asarray(uv, float).reshape(-1, 2)
    fi = np.clip(uv[:, 0] / S.dx, 0, S.nx - 1 - 1e-9)
    fj = np.clip(uv[:, 1] / S.dy, 0, S.ny - 1 - 1e-9)
    i0 = np.clip(np.floor(fi).astype(int), 0, S.nx - 2)
    j0 = np.clip(np.floor(fj).astype(int), 0, S.ny - 2)
    a, b = fi - i0, fj - j0
    I = S.ids
    corners = [I[i0, j0], I[i0 + 1, j0], I[i0, j0 + 1], I[i0 + 1, j0 + 1]]
    W = [(1 - a) * (1 - b), a * (1 - b), (1 - a) * b, a * b]
    X = S.x
    num = np.zeros((len(uv), 3))
    den = np.zeros(len(uv))
    for c, w in zip(corners, W):  # cells cut by the rounded corner: use the nodes that exist
        ok = c >= 0
        num[ok] += X[c[ok]] * w[ok, None]
        den[ok] += w[ok]
    den = np.maximum(den, 1e-9)
    p = num / den[:, None]
    p[:, 2] += LIFT
    return p


def ribbon(uv_line, half):
    """Quad strip in flat coordinates along a polyline: returns (uv points, faces)."""
    P = np.asarray(uv_line, float)
    if len(P) < 2:
        return np.zeros((0, 2)), np.zeros((0, 3), np.int32)
    t = np.gradient(P, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True) + 1e-12
    n = np.stack([-t[:, 1], t[:, 0]], 1)
    A, B = P + n * half, P - n * half
    pts = np.empty((2 * len(P), 2))
    pts[0::2], pts[1::2] = A, B
    f = []
    for k in range(len(P) - 1):
        a, b, c, d = 2 * k, 2 * k + 1, 2 * k + 2, 2 * k + 3
        f += [(a, b, d), (a, d, c)]
    return pts, np.array(f, np.int32)


def _material(stage, path, rgb, rough=0.8):
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(float(c) for c in rgb)))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _mesh(stage, path, pts, faces, mat):
    from pxr import UsdGeom, UsdShade, Vt

    m = UsdGeom.Mesh.Define(stage, path)
    m.CreateDoubleSidedAttr(True)
    m.CreateSubdivisionSchemeAttr("none")
    _set(m, pts, faces)
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return m


def _set(m, pts, faces):
    from pxr import Vt

    if len(faces) == 0:  # keep an invisible degenerate triangle
        pts = np.zeros((3, 3), np.float32) - 10.0
        faces = np.array([[0, 1, 2]], np.int32)
    m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(np.asarray(pts, np.float32)))
    m.GetFaceVertexCountsAttr().Set([3] * len(faces))
    m.GetFaceVertexIndicesAttr().Set(np.asarray(faces).ravel().tolist())


def _seam_line():
    """The chalked 15 mm sewing line from the start mark to the end mark (flat coordinates)."""
    n = int(M.PATH_LEN / 0.004) + 1
    return np.array([M.path_point(s)[0] for s in np.linspace(0.0, M.PATH_LEN, n)])


def _marks():
    """Chalk marks across the seam line at the start and the end (flat coordinates, 3 cm in from the edge)."""
    su, sv = M.START_UV
    eu, ev = M.END_UV
    return [np.array([[su, 0.002], [su, 0.032]]), np.array([[0.002, ev], [0.032, ev]])]


def render(state, tr, origin):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdGeom

    S = tr.S
    if state is None or not state["cloth"].GetPrim().IsValid():
        stage = get_current_stage()
        cloth = _mesh(stage, "/World/Seam_fabric", S.x + origin, S.faces,
                      _material(stage, "/World/SeamLook_fabric", FABRIC_RGB, 0.95))
        mk_uv, mk_f = [], []
        for line in _marks():
            p, f = ribbon(np.linspace(line[0], line[1], 6), 0.0025)
            mk_f.append(f + sum(len(q) for q in mk_uv))
            mk_uv.append(p)
        p, f = ribbon(_seam_line(), 0.0008)  # the chalked sewing line, 1.6 mm wide
        mk_f.append(f + sum(len(q) for q in mk_uv))
        mk_uv.append(p)
        mk_uv, mk_f = np.vstack(mk_uv), np.vstack(mk_f)
        chalk = _mesh(stage, "/World/Seam_chalk", bil(S, mk_uv) + origin, mk_f,
                      _material(stage, "/World/SeamLook_chalk", CHALK_RGB, 0.9))
        th_faces = np.array([(a, b, d) if q == 0 else (a, d, c)
                             for k in range(MAX_ST - 1)
                             for a, b, c, d in [(2 * k, 2 * k + 1, 2 * k + 2, 2 * k + 3)]
                             for q in (0, 1)], np.int32)
        th_init = np.full((2 * MAX_ST, 3), -10.0, np.float32)
        thread = _mesh(stage, "/World/Seam_thread", th_init, th_faces,
                       _material(stage, "/World/SeamLook_thread", THREAD_RGB, 0.5))
        nd = UsdGeom.Cylinder.Define(stage, "/World/Seam_needle")
        nd.CreateRadiusAttr(0.0009)
        nd.CreateHeightAttr(0.05)
        nd.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.86, 0.88)])
        op = UsdGeom.Xformable(nd).AddTranslateOp()
        state = dict(cloth=cloth, chalk=chalk, chalk_uv=mk_uv, chalk_f=mk_f, thread=thread, needle_op=op, n_st=-1)
    from pxr import Vt

    state["cloth"].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy((S.x + origin).astype(np.float32)))
    state["chalk"].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy((bil(S, state["chalk_uv"]) + origin).astype(np.float32)))
    st = tr.mach.stitches
    if len(st) >= 2:
        line = np.asarray(st[:MAX_ST], float)
        p, _ = ribbon(line, THREAD_HALF)
        used = (bil(S, p) + origin + np.array([0.0, 0.0, THREAD_LIFT - LIFT])).astype(np.float32)
        th_pts = np.empty((2 * MAX_ST, 3), np.float32)
        th_pts[:len(used)] = used
        th_pts[len(used):] = used[-1]
        state["thread"].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(th_pts))
    else:
        state["thread"].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(np.full((2 * MAX_ST, 3), -10.0, np.float32)))
    # needle: bobs one stroke per stitch while running, rests down in the fabric when stopped
    m = tr.mach
    z = 0.024 if not m.running else 0.024 + 0.008 * (0.5 - 0.5 * math.cos(2 * math.pi * m.t_run / M.STITCH_DT))
    N = M.NEEDLE_XY
    state["needle_op"].Set(Gf.Vec3d(float(N[0] + origin[0]), float(N[1] + origin[1]), float(0.7475 + z + origin[2])))
    return state
