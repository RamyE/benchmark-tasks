"""Reduced physical model of forming and sealing a cucumber hosomaki with a bamboo makisu (no demo-phase inputs).

Every 20 Hz control step the rigid simulator supplies the measured TCP poses, jaw openings, gripper collision boxes
and the cutting-board pose. From those alone this module advances:

  1. Cling-wrapped bamboo makisu (Joyce Chen J33-0022, 24 x 24 x 0.3 cm, 40 bamboo slats along y on cotton twine):
     a 41 x 11 position-based grid. Cotton twine along x is inextensible and bends freely so the mat rolls; each
     row along y is projected onto a stiff straight bamboo slat so symmetric two-hand presses distribute evenly across
     the roll while a one-hand corner lift tilts the slats and skews the roll. Board contact with friction; gripper
     collision boxes push free nodes. Contact-triggered grasp rule: closing a jaw (< 15 mm) on mat slats pins up to 6
     nodes to the pads; they release when the jaw opens (> 24 mm) or slips (> 4.2 cm).
  2. Cucumber baton (1.0 x 19.0 x 1.0 cm, 19 g): a rigid straight core across 9 slices, nestled in the rice bed before
     tucking and driven along the board by the net horizontal normal reaction of contacting mat plies once rolling
     exceeds the sticky-rice rolling resistance.
  3. Nori + seasoned sushi rice (Yamamotoyama Gold half sheet 10.5 x 19.0 cm, 6 mm rice over the near 9.0 cm, misted
     1.5 cm bare far strip): an elastic reduced sheet carried by the mat until tucked.
     - Tuck bond: requires carrying the nori's near edge over the cucumber and down into the rice bed ahead of it
       AND cinching the mat down over the front cheek (>= 1.4 mm radial compression for >= 0.15 s). Without pressing,
       the roasted nori springs back flat when the mat opens.
     - Anti-entrapment (peel lip clear): after tucking, the mat's near lip lies down in the front nip against the rice.
       Advancing the bundle > 1.2 cm without first peeling the mat lip upward clear of the nip (>= zc + 0.55 R), or
       rolling the bundle over the mat's near lip under an upper mat ply, traps the bamboo mat inside the sushi roll
       (mat_trapped = True).
     - Seam bond: once the peeled mat bight rolls the bundle across the full 10.5 cm sheet, a second front-cinched
       squeeze (>= 1.4 mm for >= 0.15 s) seals the misted 1.5 cm bare strip onto the lower half of the roll; an
       unpressed bare strip springs back open as soon as the mat lifts off it.
"""

from __future__ import annotations

import math

import numpy as np

# -- Nominal 1:1 product geometry (metres, station-table frame) -----------------------------------------------------
BOARD_SIZE = np.array([0.305, 0.457, 0.0127])   # Winco CBBU-1218: 12 x 18 x 1/2 in (30.5 x 45.7 x 1.27 cm)
BOARD_XY0 = np.array([0.3475, 0.0])             # spans x = 0.195..0.500 m, y = -0.2285..+0.2285 m
BOARD_MASS = 1.60

MAT_LX, MAT_LY, MAT_T = 0.240, 0.240, 0.0030    # Joyce Chen J33-0022: 24.0 x 24.0 cm, slats 3.0 mm thick
MAT_X0 = 0.180                                  # near edge x = 18.0 cm: 1.5 cm overhang past board near edge (19.5 cm)
MAT_MASS = 0.085
NX_M = 41                                       # 41 nodes along x = 40 slat pitches of 6.0 mm
DX_M = MAT_LX / (NX_M - 1)

NORI_LX, NORI_LY = 0.105, 0.190                 # half roasted nori sheet: 10.5 cm (x) x 19.0 cm (y)
NORI_X0 = 0.195                                 # nori near edge 1.5 cm in from mat near edge (aligned with x = 19.5 cm)
RICE_LX = 0.090                                 # 6 mm rice over u = 0..9.0 cm; misted bare strip u = 9.0..10.5 cm
RICE_T = 0.0060                                 # 6.0 mm levelled seasoned sushi rice layer (85 g)
RICE_PACK = 0.85                                # compacted rice area fraction inside pressed hosomaki
NU_N = 22                                       # 5.0 mm sheet stations along u
DU_N = NORI_LX / (NU_N - 1)
NY_R = 9                                        # 9 lateral slices across y = -9.5..+9.5 cm
Y_ROLL = np.linspace(-NORI_LY / 2, NORI_LY / 2, NY_R)
Y_MAT = np.r_[-MAT_LY / 2, Y_ROLL, MAT_LY / 2]  # mat columns 1..9 align with the 9 sushi slices
NY_M = len(Y_MAT)

CUC_W, CUC_H, CUC_L = 0.010, 0.010, 0.190       # English cucumber baton: 1.0 x 19.0 x 1.0 cm
CUC_X0 = 0.235                                  # centre x = 23.5 cm (4.0 cm from nori near edge, middle of rice bed)
CUC_MASS = 0.019
NORI_GAP = 0.0006                               # nori sits 0.6 mm above mat midline + half thickness

# Grasp, contact, compression-bond and anti-entrapment thresholds
PINCH_CLOSE = 0.015
PINCH_OPEN = 0.024
PINCH_SLIP = 0.042
PINCH_RECENT = 1.2
DIRECT_FOOD_DIST = 0.0045
MAT_SHIELD_DIST = 0.022
PRESS_COMP_MIN = 0.0014       # >= 1.4 mm radial mat compression (with front cinch) to bond rice tuck or nori seam
PRESS_HOLD_S = 0.15
TUCK_AHEAD = 0.003            # nori near edge >= 3 mm ahead of core axis ...
TUCK_BELOW = 0.004            # ... and <= 4 mm above axis (carried over crown and down into front rice bed)
SEAM_WOUND = NORI_LX - 0.003  # full sheet wound when bundle reaches within 3 mm of far bare edge
PEEL_CLEAR_FRAC = 0.55        # mat near lip must rise above zc + 0.55*R to clear the front nip after tucking
TRAP_ADVANCE_MAX = 0.012      # advancing > 1.2 cm from tuck without peeling lip clear traps the mat in the nip


def bundle_radius(u_b):
    """Outer nori radius of a hosomaki bundle wound to sheet station u_b."""
    area = CUC_W * CUC_H + RICE_PACK * RICE_T * np.minimum(np.maximum(u_b, 0.040), RICE_LX)
    return np.sqrt(area / math.pi) + 0.0004


R_OPEN = float(bundle_radius(0.040))            # ~10.9 mm initial effective core radius on rice bed
R_FULL = float(bundle_radius(NORI_LX))          # ~13.7 mm -> ~2.75 cm finished hosomaki diameter


def _straighten_slats(xm_grid: np.ndarray, pg: np.ndarray) -> None:
    """Project each row along y toward a straight rigid bamboo slat of length MAT_LY (weighted toward pinned nodes)."""
    w = np.where(pg, 12.0, 1.0)
    sw = w.sum(axis=1, keepdims=True)
    y_bar = (w * Y_MAT[None, :]).sum(axis=1, keepdims=True) / sw
    dy = Y_MAT[None, :] - y_bar
    denom = (w * dy * dy).sum(axis=1, keepdims=True)
    p_bar = (w[:, :, None] * xm_grid).sum(axis=1) / sw
    slope = (w[:, :, None] * dy[:, :, None] * xm_grid).sum(axis=1) / np.maximum(denom, 1e-9)
    sl_len = np.linalg.norm(slope, axis=1, keepdims=True)
    slope_unit = slope / np.maximum(sl_len, 1e-9)
    fit = p_bar[:, None, :] + dy[:, :, None] * slope_unit[:, None, :]
    free = ~pg
    xm_grid[free] += 0.80 * (fit[free] - xm_grid[free])


class MakisuRoll:
    def __init__(self, table_z: float):
        self.table_z = float(table_z)
        self._build_topology()
        self.reset()

    # ------------------------------------------------------------------ topology
    def _build_topology(self):
        self.s_mat = np.arange(NX_M, dtype=float) * DX_M
        S, Ym = np.meshgrid(self.s_mat, Y_MAT, indexing="ij")
        self.mat_rest_xy = np.stack([MAT_X0 + S, Ym], axis=-1)
        self.n_mat = NX_M * NY_M
        self.mat_idx = np.arange(self.n_mat).reshape(NX_M, NY_M)
        ma, mb, mrest, mk = [], [], [], []

        def add_m(ia, ib, k):
            a, b = ia.ravel(), ib.ravel()
            p0 = self.mat_rest_xy.reshape(-1, 2)
            ma.append(a)
            mb.append(b)
            mrest.append(np.linalg.norm(p0[b] - p0[a], axis=1))
            mk.append(np.full(len(a), k))

        add_m(self.mat_idx[:-1, :], self.mat_idx[1:, :], 0.95)    # cotton twine: inextensible along x
        add_m(self.mat_idx[:-2, :], self.mat_idx[2:, :], 0.06)    # cotton twine: soft bending
        add_m(self.mat_idx[:, :-1], self.mat_idx[:, 1:], 0.95)    # bamboo slats: axial spacing along y
        add_m(self.mat_idx[:, :-2], self.mat_idx[:, 2:], 0.80)
        add_m(self.mat_idx[:, :-4], self.mat_idx[:, 4:], 0.65)
        add_m(self.mat_idx[:-1, :-1], self.mat_idx[1:, 1:], 0.32)  # twine-slat shear coupling
        add_m(self.mat_idx[:-1, 1:], self.mat_idx[1:, :-1], 0.32)
        self.ma, self.mb = np.concatenate(ma), np.concatenate(mb)
        self.mrest, self.mk = np.concatenate(mrest), np.concatenate(mk)
        self.mcolors = self._color_edges(self.n_mat, self.ma, self.mb)

        self.u_nori = np.arange(NU_N, dtype=float) * DU_N
        self.nori_idx = np.arange(NU_N * NY_R).reshape(NU_N, NY_R)
        self.has_rice = self.u_nori <= RICE_LX + 1e-6
        self.rice_nu = int(self.has_rice.sum())
        s_n = (NORI_X0 - MAT_X0) + self.u_nori
        self._carry_m = np.clip((s_n / DX_M).astype(int), 0, NX_M - 2)
        self._carry_a = (s_n - self._carry_m * DX_M) / DX_M

        self.mat_faces_even, self.mat_faces_odd = self._build_slat_faces()
        self.nori_faces = self._grid_faces(NU_N, NY_R, self.nori_idx)

    @staticmethod
    def _color_edges(n_nodes, ea, eb):
        col = -np.ones(len(ea), dtype=int)
        used = [set() for _ in range(n_nodes)]
        for idx, (a, b) in enumerate(zip(ea, eb)):
            c = 0
            while c in used[a] or c in used[b]:
                c += 1
            col[idx] = c
            used[a].add(c)
            used[b].add(c)
        return [np.flatnonzero(col == c) for c in range(col.max() + 1)]

    @staticmethod
    def _grid_faces(nx, ny, idx):
        f = []
        for i in range(nx - 1):
            for j in range(ny - 1):
                a, b, c, d = idx[i, j], idx[i + 1, j], idx[i + 1, j + 1], idx[i, j + 1]
                f.extend([(a, b, c), (a, c, d)])
        return np.asarray(f, dtype=np.int32)

    def _build_slat_faces(self):
        even, odd = [], []
        for i in range(NX_M - 1):
            dst = even if i % 2 == 0 else odd
            for j in range(NY_M - 1):
                a, b = self.mat_idx[i, j], self.mat_idx[i + 1, j]
                c, d = self.mat_idx[i + 1, j + 1], self.mat_idx[i, j + 1]
                dst.extend([(a, b, c), (a, c, d)])
        return np.asarray(even, dtype=np.int32), np.asarray(odd, dtype=np.int32)

    # ------------------------------------------------------------------ reset
    def reset(self, board_pose=None):
        if board_pose is None:
            bpos, byaw = np.array([BOARD_XY0[0], BOARD_XY0[1], self.table_z + BOARD_SIZE[2] / 2]), 0.0
        else:
            bpos, byaw = np.asarray(board_pose[0], float), float(board_pose[1])
        self.board_pos0 = bpos.copy()
        self.board_pos = bpos.copy()
        self.board_yaw = byaw
        btop = bpos[2] + BOARD_SIZE[2] / 2
        c, s = math.cos(byaw), math.sin(byaw)
        d = self.mat_rest_xy - BOARD_XY0
        xm = np.zeros((NX_M, NY_M, 3))
        xm[..., 0] = bpos[0] + c * d[..., 0] - s * d[..., 1]
        xm[..., 1] = bpos[1] + s * d[..., 0] + c * d[..., 1]
        xm[..., 2] = btop + MAT_T / 2
        self.xm = xm.reshape(-1, 3)
        self.vm = np.zeros_like(self.xm)

        self.xc = np.full(NY_R, CUC_X0 - BOARD_XY0[0])
        self.xc0 = self.xc.copy()
        self.zc = np.full(NY_R, MAT_T + R_OPEN)
        self.stage = np.zeros(NY_R, dtype=int)        # 0 open sheet, 1 tucked bundle, 2 seam sealed
        self.u_b = np.zeros(NY_R)
        self.xc_tuck = np.zeros(NY_R)
        self.ub_tuck = np.zeros(NY_R)
        self.tuck_t = np.zeros(NY_R)
        self.seam_t = np.zeros(NY_R)
        self.squeeze = np.zeros(NY_R)
        self.front_cinch = np.zeros(NY_R, dtype=bool)
        self.top_ply = np.zeros(NY_R, dtype=bool)
        self.peeled_free = np.zeros(NY_R, dtype=bool)
        self.mat_trapped = False
        self.time = 0.0
        self.open_t = np.zeros(2)
        self.pins: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        self.bimanual_near_grasped = False
        self.direct_food_touch = False
        self.bad = False
        self.vcore = np.zeros(NY_R)
        self._update_sheet()

    # ------------------------------------------------------------------ frames
    def _fwd(self):
        return np.array([math.cos(self.board_yaw), math.sin(self.board_yaw), 0.0])

    def _to_board(self, pts):
        c, s = math.cos(self.board_yaw), math.sin(self.board_yaw)
        d = pts - self.board_pos
        return np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]]

    def _from_board(self, loc):
        c, s = math.cos(self.board_yaw), math.sin(self.board_yaw)
        out = np.empty_like(loc)
        out[..., 0] = self.board_pos[0] + c * loc[..., 0] - s * loc[..., 1]
        out[..., 1] = self.board_pos[1] + s * loc[..., 0] + c * loc[..., 1]
        out[..., 2] = self.board_pos[2] + loc[..., 2]
        return out

    def _from_board_dir(self, v):
        c, s = math.cos(self.board_yaw), math.sin(self.board_yaw)
        return np.c_[c * v[:, 0] - s * v[:, 1], s * v[:, 0] + c * v[:, 1], v[:, 2]]

    @property
    def btop_loc(self):
        return BOARD_SIZE[2] / 2

    @property
    def radius(self):
        return np.where(self.stage > 0, bundle_radius(self.u_b), R_OPEN)

    @property
    def cuc(self):
        """Cucumber baton slice centres (NY_R, 3) in env frame."""
        loc = np.c_[self.xc, Y_ROLL, self.btop_loc + self.zc]
        return self._from_board(loc)

    def _fit_rigid_core(self):
        """Cucumber baton is rigid: slice centres lie on a straight line across y."""
        A = np.c_[np.ones(NY_R), Y_ROLL]
        coef, *_ = np.linalg.lstsq(A, self.xc, rcond=None)
        self.xc[:] = A @ coef

    # ------------------------------------------------------------------ grasp pins & direct-touch check
    def _update_pins(self, tcp, rot, aperture):
        for arm in range(2):
            if aperture[arm] > PINCH_CLOSE:
                self.open_t[arm] = self.time
            if arm in self.pins:
                nodes, off = self.pins[arm]
                tgt = tcp[arm] + off @ rot[arm].T
                slip = float(np.linalg.norm(self.xm[nodes] - tgt, axis=1).mean())
                if aperture[arm] > PINCH_OPEN or slip > PINCH_SLIP:
                    del self.pins[arm]
                continue
            if aperture[arm] > PINCH_CLOSE or (self.time - self.open_t[arm]) > PINCH_RECENT:
                continue
            loc = (self.xm - tcp[arm]) @ rot[arm]
            ok = (np.abs(loc[:, 0]) < 0.020) & (np.abs(loc[:, 1]) < 0.022) & (loc[:, 2] > -0.018) & (loc[:, 2] < 0.022)
            for other, _ in self.pins.values():
                ok[other] = False
            if not ok.any():
                continue
            cand = np.flatnonzero(ok)
            sel = cand[np.argsort(np.linalg.norm(loc[cand], axis=1))[:6]]
            offs = loc[sel].copy()
            offs[:, 0] = np.clip(offs[:, 0], -0.002, 0.002)
            self.pins[arm] = (sel, offs)
        if 0 in self.pins and 1 in self.pins:
            r0, c0 = np.divmod(self.pins[0][0], NY_M)
            r1, c1 = np.divmod(self.pins[1][0], NY_M)
            near0 = r0 * DX_M <= 0.030 + 1e-6
            near1 = r1 * DX_M <= 0.030 + 1e-6
            y0, y1 = Y_MAT[c0], Y_MAT[c1]
            if ((near0 & (y0 >= 0.045)).any() and (near1 & (y1 <= -0.045)).any()) or (
                (near0 & (y0 <= -0.045)).any() and (near1 & (y1 >= 0.045)).any()
            ):
                self.bimanual_near_grasped = True

    def food_points(self):
        return np.vstack([self.xn, self.rice_top_points(), self.cuc])

    def _check_direct_food_touch(self, jaw_pts, boxes):
        if self.direct_food_touch:
            return
        food = self.food_points()
        lo, hi = food.min(axis=0) - 0.03, food.max(axis=0) + 0.03
        g = [p for p in np.vstack(jaw_pts) if np.all((p >= lo) & (p <= hi))]
        for world, blo, bhi, _arm in boxes:
            for u in (0.0, 0.5, 1.0):
                for v in (0.0, 1.0):
                    for w in (0.0, 0.5, 1.0):
                        p = (blo + np.array([u, v, w]) * (bhi - blo)) @ world[:3, :3].T + world[:3, 3]
                        if np.all((p >= lo) & (p <= hi)):
                            g.append(p)
        if not g:
            return
        G = np.asarray(g)
        dd = np.linalg.norm(G[:, None, :] - food[None, :, :], axis=2)
        gi, fi = np.where(dd < DIRECT_FOOD_DIST)
        for a, b in zip(gi, fi):
            mid = 0.5 * (G[a] + food[b])
            if float(np.min(np.linalg.norm(self.xm - mid, axis=1))) > MAT_SHIELD_DIST:
                self.direct_food_touch = True
                return

    # ------------------------------------------------------------------ physics step
    def step(self, tcp, rot, aperture, boxes=(), board_pose=None, jaw_pts=None, dt: float = 0.05,
             substeps: int = 8, iters: int = 6):
        tcp, rot, aperture = np.asarray(tcp, float), np.asarray(rot, float), np.asarray(aperture, float)
        if board_pose is not None:
            self.board_pos = np.asarray(board_pose[0], float)
            self.board_yaw = float(board_pose[1])
        self.time += dt
        self._update_pins(tcp, rot, aperture)
        hx, hy, hz = BOARD_SIZE / 2
        btop = self.board_pos[2] + hz
        h = dt / substeps

        pin_list = []
        pinned = np.zeros(self.n_mat, dtype=bool)
        for arm, (nodes, off) in self.pins.items():
            pin_list.append((nodes, self.xm[nodes].copy(), tcp[arm] + off @ rot[arm].T))
            pinned[nodes] = True
        w_mat = (~pinned).astype(float)
        pg = pinned.reshape(NX_M, NY_M)
        lo_ws, hi_ws = self.xm.min(axis=0) - 0.12, self.xm.max(axis=0) + 0.12
        act_boxes = [b for b in boxes if np.all((b[0][:3, 3] > lo_ws) & (b[0][:3, 3] < hi_ws))]

        squeeze_sum = np.zeros(NY_R)
        top_ply = np.zeros(NY_R, dtype=bool)
        xc_start = self.xc.copy()
        for sub in range(substeps):
            fr = (sub + 1) / substeps
            xm_prev = self.xm.copy()
            self.vm *= 0.92
            self.vm[:, 2] -= 9.81 * h
            self.xm += self.vm * h
            for nodes, p0, p1 in pin_list:
                self.xm[nodes] = p0 + fr * (p1 - p0)
            sub_sq = np.zeros(NY_R)
            for it in range(iters):
                for cg in self.mcolors:
                    a, b = self.ma[cg], self.mb[cg]
                    dv = self.xm[b] - self.xm[a]
                    ln = np.linalg.norm(dv, axis=1)
                    wa, wb = w_mat[a], w_mat[b]
                    ws = wa + wb
                    ok = (ws > 0) & (ln > 1e-9)
                    cs = np.zeros_like(ln)
                    cs[ok] = (ln[ok] - self.mrest[cg][ok]) / ln[ok] * self.mk[cg][ok] / ws[ok]
                    corr = dv * cs[:, None]
                    self.xm[a] += corr * wa[:, None]
                    self.xm[b] -= corr * wb[:, None]
                _straighten_slats(self.xm.reshape(NX_M, NY_M, 3), pg)
                for nodes, p0, p1 in pin_list:
                    self.xm[nodes] = p0 + fr * (p1 - p0)
                # Cutting board and table support
                loc = self._to_board(self.xm)
                on_b = (np.abs(loc[:, 0]) <= hx + 0.002) & (np.abs(loc[:, 1]) <= hy + 0.002)
                low = on_b & (loc[:, 2] < hz + MAT_T / 2) & ~pinned
                if low.any():
                    idx = np.flatnonzero(low)
                    front = loc[idx, 0] < -hx + 0.008
                    zfix = hz + MAT_T / 2 - loc[idx, 2]
                    top = idx[~front | (zfix < 0.008)]
                    self.xm[top, 2] = btop + MAT_T / 2
                    edge = idx[front & (zfix >= 0.008)]
                    if len(edge):
                        dxo = (-hx - 0.002) - loc[edge, 0]
                        self.xm[edge] += self._from_board_dir(np.c_[dxo, np.zeros(len(edge)), np.zeros(len(edge))])
                lt = (self.xm[:, 2] < self.table_z + MAT_T / 2) & ~pinned
                self.xm[lt, 2] = self.table_z + MAT_T / 2
                # Gripper boxes push free mat nodes (for jaws not already pinching the mat)
                for world, blo, bhi, arm_id in act_boxes:
                    if arm_id in self.pins:
                        continue
                    Rm, t = world[:3, :3], world[:3, 3]
                    q = (self.xm - t) @ Rm
                    lp, hp = blo - 0.0015, bhi + 0.0015
                    ins = np.all((q > lp) & (q < hp), axis=1) & ~pinned
                    if ins.any():
                        ii = np.flatnonzero(ins)
                        qi = q[ii]
                        dlo, dhi = qi - lp, hp - qi
                        ax = np.argmin(np.minimum(dlo, dhi), axis=1)
                        r = np.arange(len(ii))
                        qi[r, ax] = np.where(dlo[r, ax] < dhi[r, ax], lp[ax], hp[ax])
                        self.xm[ii] = qi @ Rm.T + t
                sq, tp = self._mat_core_contact(pinned, first=(it == 0))
                if it == 0:
                    sub_sq = sq
                top_ply |= tp
            squeeze_sum += sub_sq
            loc = self._to_board(self.xm)
            rest = (
                (np.abs(loc[:, 0]) <= hx + 0.002)
                & (np.abs(loc[:, 1]) <= hy + 0.002)
                & (self.xm[:, 2] <= btop + MAT_T / 2 + 0.0015)
                & ~pinned
            )
            if rest.any():
                sl = self.xm[rest] - xm_prev[rest]
                sl[:, 2] = 0.0
                self.xm[rest] -= 0.92 * sl
            self.vm = np.clip((self.xm - xm_prev) / h, -2.0, 2.0)

        self.squeeze = squeeze_sum / substeps
        self.top_ply = top_ply
        self._fit_rigid_core()
        self._support_core(btop)
        self._advance_roll(dt, xc_start)
        self._update_sheet()
        if jaw_pts is not None:
            self._check_direct_food_touch(jaw_pts, act_boxes)
        self.vcore = np.abs(self.xc - xc_start) / dt
        if not (np.isfinite(self.xm).all() and np.isfinite(self.xn).all() and np.isfinite(self.xc).all()):
            self.bad = True

    def _mat_core_contact(self, pinned, first=False):
        """Resolve mat <-> roll contact; apply symmetric rear-vs-front horizontal reaction above sticky-rice threshold."""
        grid = self.xm.reshape(NX_M, NY_M, 3)[:, 1:1 + NY_R, :]
        loc = self._to_board(grid.reshape(-1, 3)).reshape(NX_M, NY_R, 3)
        dx = loc[..., 0] - self.xc[None, :]
        dz = loc[..., 2] - (self.btop_loc + self.zc)[None, :]
        dist = np.hypot(dx, dz)
        r_req = (self.radius + MAT_T / 2)[None, :]
        pen = r_req - dist
        hit = (pen > 0) & (dist > 1e-9)
        sq = np.zeros(NY_R)
        tp = np.zeros(NY_R, dtype=bool)
        if not hit.any():
            return sq, tp
        nx, nz = dx / np.maximum(dist, 1e-9), dz / np.maximum(dist, 1e-9)
        upper = hit & (nz > 0.15)
        tp = upper.any(axis=0)
        sq = np.where(upper, pen, 0.0).max(axis=0)
        p_grid = pinned.reshape(NX_M, NY_M)[:, 1:1 + NY_R]
        w_m = np.where(p_grid, 0.0, 0.85)
        w_c = np.where(self.stage > 0, 0.18, 0.04)[None, :]
        f_m = np.where(hit, pen * w_m / np.maximum(w_m + w_c, 1e-9), 0.0)
        corr_loc = np.stack([f_m * nx, np.zeros_like(f_m), f_m * nz], axis=-1)
        corr = self._from_board_dir(corr_loc.reshape(-1, 3)).reshape(NX_M, NY_R, 3)
        self.xm.reshape(NX_M, NY_M, 3)[:, 1:1 + NY_R, :] += corr
        if first:
            active = hit & (~p_grid) & (nz > -0.45) & (nz < 0.85)
            rear_pen = np.where(active & (nx < -0.20), pen * (-nx), 0.0).max(axis=0)
            front_pen = np.where(active & (nx > +0.20), pen * nx, 0.0).max(axis=0)
            net = rear_pen - front_pen
            thresh = 0.0007 + 0.30 * sq
            drive = np.where(np.abs(net) > thresh, net - np.sign(net) * thresh, 0.0)
            scale = np.where(self.stage > 0, 0.88, 0.18)
            self.xc += np.clip(drive * scale, -0.0018, 0.0018)
            s0 = self.stage == 0
            if s0.any():
                self.xc[s0] = np.clip(self.xc[s0], self.xc0[s0] - 0.004, self.xc0[s0] + 0.008)
        return sq, tp

    def _support_core(self, btop):
        """Core rests on the supporting mat ply beneath it (or the cutting board)."""
        grid = self.xm.reshape(NX_M, NY_M, 3)[:, 1:1 + NY_R, :]
        loc = self._to_board(grid.reshape(-1, 3)).reshape(NX_M, NY_R, 3)
        under = np.abs(loc[..., 0] - self.xc[None, :]) < 0.009
        z_low = np.where(under, loc[..., 2], np.inf).min(axis=0)
        z_low = np.where(np.isfinite(z_low), z_low, self.btop_loc + MAT_T / 2)
        self.zc = np.maximum(z_low - self.btop_loc, MAT_T / 2) + MAT_T / 2 + self.radius

    def _near_edge_rel(self):
        """Nori near edge relative to core axis: (ahead, above) per slice in board frame."""
        xn = self.xn.reshape(NU_N, NY_R, 3)
        loc = self._to_board(xn[0])
        return loc[:, 0] - self.xc, loc[:, 2] - (self.btop_loc + self.zc)

    def _advance_roll(self, dt, xc_start):
        ahead, above = self._near_edge_rel()
        grid = self.xm.reshape(NX_M, NY_M, 3)[:, 1:1 + NY_R, :]
        loc_all = self._to_board(grid.reshape(-1, 3)).reshape(NX_M, NY_R, 3)
        lip_z_above_board = loc_all[:3, :, 2].max(axis=0) - self.btop_loc
        lip_x = loc_all[0, :, 0]
        # True radial cinch requires both upper-half compression >= PRESS_COMP_MIN AND the front mat lip drawn down
        # below zc + 0.50*R ahead of the roll (draping/pulling high at z > zc + 0.8*R cannot compress the front seam!)
        front_cinched = (lip_x > self.xc + 0.5 * self.radius) & (lip_z_above_board <= self.zc + 0.50 * self.radius)
        self.front_cinch = front_cinched
        pressed = (self.squeeze >= PRESS_COMP_MIN) & front_cinched
        over_roll = (
            (np.abs(loc_all[4:, :, 0] - self.xc[None, :]) < self.radius[None, :] + 0.004)
            & (loc_all[4:, :, 2] - self.btop_loc > self.zc[None, :] + 0.4 * self.radius[None, :])
        ).any(axis=0)

        for k in range(NY_R):
            if self.stage[k] == 0:
                ok = pressed[k] and ahead[k] >= TUCK_AHEAD and above[k] <= TUCK_BELOW
                self.tuck_t[k] = self.tuck_t[k] + dt if ok else max(0.0, self.tuck_t[k] - 0.5 * dt)
                if self.tuck_t[k] >= PRESS_HOLD_S:
                    self.stage[k] = 1
                    self.xc_tuck[k] = self.xc[k]
                    self.ub_tuck[k] = self.xc[k] - (NORI_X0 - BOARD_XY0[0])
                    self.u_b[k] = self.ub_tuck[k]
                    self.peeled_free[k] = False
            else:
                if lip_z_above_board[k] >= self.zc[k] + PEEL_CLEAR_FRAC * self.radius[k]:
                    self.peeled_free[k] = True
                if (not self.peeled_free[k] and (self.xc[k] - self.xc_tuck[k]) > TRAP_ADVANCE_MAX) or (
                    abs(lip_x[k] - self.xc[k]) < self.radius[k] + 0.005
                    and lip_z_above_board[k] < self.zc[k]
                    and over_roll[k]
                ):
                    self.mat_trapped = True
                if self.stage[k] == 1:
                    want = self.ub_tuck[k] + (self.xc[k] - self.xc_tuck[k])
                    lim = NORI_LX if self.top_ply[k] else min(NORI_LX, RICE_LX)
                    self.u_b[k] = float(np.clip(want, self.ub_tuck[k], lim))
                    if self.u_b[k] >= SEAM_WOUND and pressed[k] and self.peeled_free[k]:
                        self.seam_t[k] += dt
                        if self.seam_t[k] >= PRESS_HOLD_S:
                            self.stage[k] = 2
                            self.u_b[k] = NORI_LX
                    elif not pressed[k]:
                        self.seam_t[k] = max(0.0, self.seam_t[k] - 0.5 * dt)
                else:
                    self.u_b[k] = NORI_LX

    # ------------------------------------------------------------------ sheet geometry & meshes
    def _mat_point(self, u):
        grid = self.xm.reshape(NX_M, NY_M, 3)[:, 1:1 + NY_R, :]
        mi, al = self._carry_m, self._carry_a[:, None, None]
        p = (1 - al) * grid[mi] + al * grid[mi + 1]
        t = grid[np.minimum(NX_M - 1, mi + 1)] - grid[np.maximum(0, mi - 1)]
        t /= np.maximum(np.linalg.norm(t, axis=2, keepdims=True), 1e-9)
        sy = grid[:, -1, :] - grid[:, 0, :]
        sy /= np.maximum(np.linalg.norm(sy, axis=1, keepdims=True), 1e-9)
        n = np.cross(t, sy[mi][:, None, :])
        n /= np.maximum(np.linalg.norm(n, axis=2, keepdims=True), 1e-9)
        return p, n

    def _update_sheet(self):
        p, n = self._mat_point(self.u_nori)
        xn = p + n * (MAT_T / 2 + NORI_GAP)
        if (self.stage > 0).any():
            fwd = self._fwd()
            up = np.array([0.0, 0.0, 1.0])
            ax = self.cuc
            R = self.radius
            for k in np.flatnonzero(self.stage > 0):
                ub = self.u_b[k]
                u = self.u_nori
                wound = u <= ub + 1e-9
                back = ub - u[wound]
                rr = np.where(back > 2 * math.pi * R[k] - 0.002, R[k] - 0.0025, R[k])
                th = -0.5 * math.pi - back / R[k]
                rr = np.where(u[wound] > RICE_LX, R[k] + 0.0007, rr)
                pts = ax[k][None, :] + rr[:, None] * (
                    np.cos(th)[:, None] * fwd[None, :] + np.sin(th)[:, None] * up[None, :]
                )
                xn[wound, k] = pts
                nw = ~wound
                if nw.any():
                    contact = ax[k] - R[k] * up
                    xn[nw, k] = p[nw, k] + n[nw, k] * (MAT_T / 2 + NORI_GAP)
                    shift = contact + fwd * (u[nw][0] - ub) - xn[nw, k][0]
                    shift[2] = 0.0
                    xn[nw, k] += shift
        self.xn = xn.reshape(-1, 3)

    def rice_top_points(self):
        g = self.xn.reshape(NU_N, NY_R, 3)[: self.rice_nu]
        du = np.empty_like(g)
        du[:-1] = g[1:] - g[:-1]
        du[-1] = g[-1] - g[-2]
        dv = np.empty_like(g)
        dv[:, :-1] = g[:, 1:] - g[:, :-1]
        dv[:, -1] = g[:, -1] - g[:, -2]
        nrm = np.cross(du, dv)
        nrm /= np.maximum(np.linalg.norm(nrm, axis=2, keepdims=True), 1e-9)
        return (g + 0.0048 * nrm).reshape(-1, 3)

    def cucumber_box_mesh(self):
        c = self.cuc
        p0, p1 = c[0], c[-1]
        a = p1 - p0
        u = a / max(float(np.linalg.norm(a)), 1e-9)
        v = np.cross([0.0, 0.0, 1.0], u)
        v /= max(float(np.linalg.norm(v)), 1e-9)
        w = np.cross(u, v)
        hw = CUC_W / 2
        verts = [e + cv * v + cw * w for e in (p0, p1) for cv, cw in ((-hw, -hw), (hw, -hw), (hw, hw), (-hw, hw))]
        faces = [
            (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
            (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7),
            (0, 3, 2), (0, 2, 1), (4, 5, 6), (4, 6, 7),
        ]
        return np.asarray(verts, dtype=np.float32), np.asarray(faces, dtype=np.int32)

    # ------------------------------------------------------------------ geometric read-out for scoring
    def metrics(self):
        if self.bad:
            return {"valid": False}
        hx, hy, hz = BOARD_SIZE / 2
        btop = self.board_pos[2] + hz
        fwd = self._fwd()
        board_shift = float(np.linalg.norm((self.board_pos - self.board_pos0)[:2]))
        cuc = self.cuc
        lc, ln_, lm = self._to_board(cuc), self._to_board(self.xn), self._to_board(self.xm)
        food_on_board = bool(
            (np.abs(lc[:, 0]).max() <= hx)
            & (np.abs(lc[:, 1]).max() <= hy)
            & (np.abs(ln_[:, 0]).max() <= hx + 0.010)
            & (np.abs(ln_[:, 1]).max() <= hy)
            & (lc[:, 2].min() >= hz - 0.002)
        )
        mat_on_board = bool(
            lm[:, 0].min() >= -hx - 0.045
            and lm[:, 0].max() <= hx + 0.025
            and np.abs(lm[:, 1]).max() <= hy + 0.025
        )

        xn = self.xn.reshape(NU_N, NY_R, 3)
        wrap = np.zeros(NY_R)
        enclosed = np.zeros(NY_R, dtype=bool)
        seam_gap = np.zeros(NY_R)
        seam_down = np.zeros(NY_R, dtype=bool)
        for k in range(NY_R):
            rel = xn[:, k, :] - cuc[k]
            rx = rel @ fwd
            rz = rel[:, 2]
            ang = np.unwrap(np.arctan2(rz[: self.rice_nu], rx[: self.rice_nu]))
            wrap[k] = math.degrees(abs(ang[-1] - ang[0]))
            # Enclosed within <= 3.5 cm outer diameter (radius <= 1.75 cm from core axis)
            wrapped_pts = np.linalg.norm(rel[: self.rice_nu], axis=1)
            enclosed[k] = bool(wrapped_pts.max() <= 0.0175)
            d = np.linalg.norm(xn[-1, k] - xn[:12, k], axis=1)
            seam_gap[k] = float(d.min())
            seam_down[k] = bool(xn[-1, k, 2] <= cuc[k, 2] + 0.002)

        mid = slice(1, NY_R - 1)
        ahead, _above = self._near_edge_rel()
        folded = (self.stage > 0) | (ahead > 0.003)
        folded_over = bool(folded[mid].mean() >= 0.8)
        closed = (self.stage > 0) & (self.u_b >= RICE_LX - 1e-6) & enclosed & (wrap >= 330.0)
        closed_tube = bool(closed[mid].mean() >= 0.8)
        sealed = (self.stage == 2) & (seam_gap <= 0.004) & seam_down
        seam_sealed = bool(closed_tube and sealed[mid].mean() >= 0.8)
        # Lateral straightness across the roll (between y = -7.1 cm and y = +7.1 cm, ~2.4 cm in from each end)
        skew_x = abs(float(self.xc[7] - self.xc[1]))
        straight = bool(skew_x <= 0.015)

        grid = self.xm.reshape(NX_M, NY_M, 3)
        mat_span_x = float((grid[-1, NY_M // 2] - grid[0, NY_M // 2]) @ fwd)
        mat_max_h = float(self.xm[:, 2].max() - btop)
        mat_open = bool(mat_span_x >= 0.200 and mat_max_h <= 0.020)
        near_x = float((grid[0, NY_M // 2] - self.board_pos) @ fwd)
        far_x = float((grid[-1, NY_M // 2] - self.board_pos) @ fwd)
        roll_x = float(self.xc[NY_R // 2])
        roll_z = float(self.zc[NY_R // 2])
        roll_on_open_mat = bool(
            mat_open
            and min(near_x, far_x) + 0.015 <= roll_x <= max(near_x, far_x) - 0.015
            and roll_z <= MAT_T + R_FULL + 0.004
        )
        return {
            "valid": True,
            "board_shift": board_shift,
            "food_on_board": food_on_board,
            "mat_on_board": mat_on_board,
            "direct_food_touch": bool(self.direct_food_touch),
            "mat_trapped": bool(self.mat_trapped),
            "peeled_free": bool(self.peeled_free[mid].mean() >= 0.8),
            "bimanual_near_grasped": bool(self.bimanual_near_grasped),
            "wrap_deg_med": float(np.median(wrap[mid])),
            "wrap_deg_min": float(np.min(wrap[mid])),
            "seam_gap_max": float(np.max(seam_gap[mid])),
            "stages": "".join(str(int(s)) for s in self.stage),
            "folded_over": folded_over,
            "closed_tube": closed_tube,
            "seam_sealed": seam_sealed,
            "skew_x": skew_x,
            "straight": straight,
            "mat_span_x": mat_span_x,
            "mat_max_h": mat_max_h,
            "mat_open": mat_open,
            "roll_on_open_mat": roll_on_open_mat,
            "roll_speed": float(self.vcore.max()),
            "held": bool(len(self.pins) > 0),
        }
