"""Cling film (one 26 x 26 cm square of Glad ClingWrap, LDPE ~12.5 um) stretched as a diamond over a Corelle 28-oz
soup bowl, modelled as a reduced position-based (XPBD) sheet. NumPy only, no task-phase inputs.

The rigid sim supplies, every control step: both TCP poses, both jaw openings, the gripper collision boxes and the
bowl's pose. The film only reacts to those measured quantities:
  * contact: gripper boxes, table, bowl, and static fixture boxes push film points out of themselves and carry touching
    points along by friction;
  * a pinch starts when a jaw closes (aperture falls below PINCH_CLOSE, having been open within PINCH_RECENT) with
    untacked film points between its pad tips; those points are then held; it lets go when the jaw opens (> PINCH_OPEN)
    or the held film is dragged > PINCH_SLIP off the pads;
  * cling: film points that lie on the bowl's rim and upper outer wall are tacked to the bowl. A tack pulls its point
    back to where it clung with a stiff spring and gives way (for good) when the film pulls it off the glaze harder
    than PEEL_N straight out from the surface, or harder than SHEAR_N along it. Cling film is weak in peel and strong
    in shear: lift a corner up off the glaze and tacks release progressively, while tugging without steadying the bowl
    transmits enough horizontal pull through intact tacks to slide the bowl on the damp mat. Every intact tack's pull
    is returned to the sim as a wrench on the rigid bowl.

Material coordinates (a, b) in [-L/2, L/2]^2 are rotated 45 deg when draped as a diamond over the bowl, placing the
four triangular corners on the cardinal axes: +x (far), -x (near), +y (left), -y (right).
"""
from __future__ import annotations

import math

import numpy as np

L = 0.26                 # 26 x 26 cm square cut from a 30.5 cm (12 in) Glad ClingWrap roll
H = 0.013                # lattice spacing (21 x 21 = 441 nodes)
GSM = 0.0115             # kg/m2: 12.5 um LDPE at 0.92 g/cm3
FILM_K = 6.0e3           # N/m per lattice edge, tension only
SHEAR_K = 120.0
BEND_K = 0.16            # light LDPE flex resistance so loose corner tabs flare slightly off the bowl wall
THICK = 0.0015           # contact skin (m)
MU_TABLE, MU_BOWL, MU_PAD, MU_BIN = 0.6, 0.55, 0.9, 0.45
PINCH_CLOSE, PINCH_OPEN, PINCH_SLIP = 0.016, 0.023, 0.065
PINCH_RECENT = 1.2
PIN_BOX = np.array([0.024, 0.024, 0.026])      # half extents in gripper frame around PIN_C
PIN_C = np.array([0.0, 0.0, 0.136])            # between the finger pads (TCP at 0.134, tips at 0.147)
DAMP = 0.94

# Corelle Winter Frost White 28-oz soup bowl: 15.9 cm outer rim dia, 7.6 cm tall, tapering toward a 9.6 cm base
BOWL_R = 0.0795
BOWL_FOOT_R = 0.048
BOWL_H = 0.076
BOWL_RR = 0.004           # rim rounding
CLING_DEPTH = 0.025       # film clings to the rim and 2.5 cm down the upper outer glaze
TACK_K = 440.0            # N/m per tack spring
PEEL_N = 0.30             # N normal pull off the glaze per 1.3 cm tack (~0.23 N/cm, low cling-film peel cling)
SHEAR_N = 0.75            # N tangential pull along the glaze: tack releases (tugging unbraced drags the bowl)

C45 = math.sqrt(0.5)

# In diamond layout: fx = [C45*(a - b), C45*(a + b)]
#   far (+x):  a = +L/2, b = -L/2
#   near (-x): a = -L/2, b = +L/2
#   left (+y): a = +L/2, b = +L/2
#   right (-y):a = -L/2, b = -L/2
TAB_AB = {
    "far": (1, -1),
    "near": (-1, 1),
    "left": (1, 1),
    "right": (-1, -1),
    "far_left": (1, 0),
    "far_right": (0, -1),
    "near_left": (0, 1),
    "near_right": (-1, 0),
}


def quat_to_R(q_xyzw):
    x, y, z, w = q_xyzw
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Film:
    def __init__(self, table_z: float, static_aabbs: list[tuple[np.ndarray, np.ndarray, float]] | None = None):
        """static_aabbs: list of (lo_xyz, hi_xyz, mu) world axis-aligned boxes (bin walls/floor, trivet, etc.)."""
        self.table_z = float(table_z)
        self.static_aabbs = [(np.asarray(lo, float), np.asarray(hi, float), float(mu))
                             for lo, hi, mu in (static_aabbs or ())]
        m = int(round(L / H)) + 1
        self.m = m
        g = -L / 2 + np.arange(m) * H
        A, B = np.meshgrid(g, g, indexing="ij")
        self.ab = np.c_[A.ravel(), B.ravel()]
        self.diamond_xy = np.c_[C45 * (self.ab[:, 0] - self.ab[:, 1]), C45 * (self.ab[:, 0] + self.ab[:, 1])]
        n = m * m
        self.n = n
        idx = np.arange(n).reshape(m, m)
        e, kind = [], []
        for i in range(m):
            for j in range(m):
                if i + 1 < m:
                    e.append((idx[i, j], idx[i + 1, j])); kind.append(0)
                if j + 1 < m:
                    e.append((idx[i, j], idx[i, j + 1])); kind.append(0)
                if i + 1 < m and j + 1 < m:
                    e.append((idx[i, j], idx[i + 1, j + 1])); kind.append(1)
                    e.append((idx[i + 1, j], idx[i, j + 1])); kind.append(1)
                if i + 2 < m:
                    e.append((idx[i, j], idx[i + 2, j])); kind.append(2)
                if j + 2 < m:
                    e.append((idx[i, j], idx[i, j + 2])); kind.append(2)
                if i + 2 < m and j + 2 < m:
                    e.append((idx[i, j], idx[i + 2, j + 2])); kind.append(2)
                    e.append((idx[i + 2, j], idx[i, j + 2])); kind.append(2)
        self.edges = np.array(e)
        self.kind = np.array(kind)
        flat = np.c_[self.ab, np.zeros(n)]
        self.rest_flat = np.linalg.norm(flat[self.edges[:, 1]] - flat[self.edges[:, 0]], axis=1)
        self.rest = self.rest_flat.copy()
        k = np.where(self.kind == 0, FILM_K, np.where(self.kind == 1, SHEAR_K, BEND_K))
        self.compliance = 1.0 / k
        self._tension_only = self.kind <= 1
        self.inv_m = np.full(n, 1.0 / (GSM * H * H))
        self.colors = self._color()
        f = []
        for i in range(m - 1):
            for j in range(m - 1):
                p, q, r, s = idx[i, j], idx[i + 1, j], idx[i + 1, j + 1], idx[i, j + 1]
                f += [(p, q, r), (p, r, s)]
        self.faces = np.array(f, np.int32)
        mid = m // 2
        self.tabs = {}
        for name, (sa, sb) in TAB_AB.items():
            ia = mid if sa == 0 else (0 if sa < 0 else m - 1)
            ib = mid if sb == 0 else (0 if sb < 0 else m - 1)
            self.tabs[name] = int(idx[ia, ib])
        self.corner = self.tabs  # compatibility alias
        self.edge_ring = np.flatnonzero((np.abs(self.ab) > L / 2 - 1e-6).any(axis=1))
        self.border_mask = (np.abs(self.ab) > L / 2 - 0.015).any(axis=1)
        self.fog_mask = np.linalg.norm(self.diamond_xy, axis=1) <= 0.062
        self.x = np.zeros((n, 3)); self.v = np.zeros((n, 3))
        self.pins = {}
        self.prev_boxes = None
        self.time = 0.0
        self.open_t = np.zeros(2)
        self.bowl_T = np.eye(4)
        self.tack = np.zeros(n, bool)
        self.tack_loc = np.zeros((n, 3))
        self.tack_nrm = np.zeros((n, 3))
        self.tack0 = np.zeros(n, bool)
        self.broke_t = np.full(n, np.inf)
        self.broke_seq = []
        self.bowl_force = np.zeros(3)
        self.bowl_torque = np.zeros(3)
        self.touched = np.zeros(n, bool)
        self.peak_strain = 1.0
        self.bad = False

    def _color(self):
        col = -np.ones(len(self.edges), int)
        used = [set() for _ in range(self.n)]
        for i, (a, b) in enumerate(self.edges):
            c = 0
            while c in used[a] or c in used[b]:
                c += 1
            col[i] = c; used[a].add(c); used[b].add(c)
        return [np.flatnonzero(col == c) for c in range(col.max() + 1)]

    # -- bowl frame ------------------------------------------------------------------------------------------------
    def set_bowl(self, base_pos, R):
        """base_pos: world point at the centre of the bowl's foot; R: its rotation."""
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = base_pos
        self.bowl_T = T

    def to_bowl(self, x):
        T = self.bowl_T
        return (x - T[:3, 3]) @ T[:3, :3]

    def from_bowl(self, p, vec=False):
        T = self.bowl_T
        return p @ T[:3, :3].T + (0 if vec else T[:3, 3])

    @staticmethod
    def bowl_outer_r(z_local):
        """Flared Corelle bowl outer radius at height z_local in [0, BOWL_H]."""
        t = np.clip(z_local / BOWL_H, 0.0, 1.0)
        return BOWL_FOOT_R + (BOWL_R - BOWL_FOOT_R) * (t ** 0.72)

    def _bowl_sdf(self, x):
        """Signed distance and outward normal (world) of the flared bowl solid (preventing film passing through)."""
        p = self.to_bowl(x)
        r = np.hypot(p[:, 0], p[:, 1])
        z = p[:, 2]
        rw = self.bowl_outer_r(z)
        # Signed distance to capped flared frustum [0..BOWL_H] with outer radius rw(z)
        dr = r - rw
        dz_top = z - BOWL_H
        dz_bot = -z
        dz = np.maximum(dz_top, dz_bot)
        outside_r = np.maximum(dr, 0.0)
        outside_z = np.maximum(dz, 0.0)
        d_out = np.hypot(outside_r, outside_z)
        d_in = np.maximum(dr, dz)
        d = np.where((dr <= 0.0) & (dz <= 0.0), d_in, d_out)
        radial = p[:, :2] / np.maximum(r, 1e-9)[:, None]
        # Slope drw/dz ~ 0.72*(BOWL_R - BOWL_FOOT_R)/BOWL_H * t^-0.28 -> outward wall normal tilts slightly down (-z)
        slope = 0.30
        nw_xy = radial / math.hypot(1.0, slope)
        nw_z = np.full(len(x), -slope / math.hypot(1.0, slope))
        top_dom = (z > BOWL_H - 0.004) & (dr < 0.004)
        nl = np.c_[nw_xy, nw_z]
        nl[top_dom] = [0.0, 0.0, 1.0]
        bot_dom = (z < 0.002) & (dr < 0.0)
        nl[bot_dom] = [0.0, 0.0, -1.0]
        return d, self.from_bowl(nl, vec=True)

    # -- state -----------------------------------------------------------------------------------------------------
    def drape(self, base_pos, R=np.eye(3)):
        """Stretched as a diamond over the rim (corners +-x, +-y), clinging CLING_DEPTH down the upper outer glaze,
        with the 4 triangular corner tails flaring cleanly above the counter."""
        self.set_bowl(np.asarray(base_pos, float), R)
        fx = self.diamond_xy.copy()
        d = np.linalg.norm(fx, axis=1)
        u = fx / np.maximum(d, 1e-9)[:, None]
        p = np.zeros((self.n, 3))
        top = BOWL_H + THICK
        on_top = d <= BOWL_R
        p[on_top, :2] = fx[on_top]
        p[on_top, 2] = top
        s = d - BOWL_R
        clung = (~on_top) & (s <= CLING_DEPTH)
        # Step down along glaze so euclidean step <= s (zero pre-tension on tacks at spawn)
        zc = top - 0.94 * s[clung]
        rc = self.bowl_outer_r(zc) + THICK + 0.0005
        p[clung, :2] = u[clung] * rc[:, None]
        p[clung, 2] = zc
        skirt = (~on_top) & (~clung)
        ds = s[skirt] - CLING_DEPTH
        z0 = top - 0.94 * CLING_DEPTH
        r0 = float(self.bowl_outer_r(z0)) + THICK + 0.0005
        # LDPE corner tabs flare outward at ~42 deg from vertical, hanging ~1.2 cm clear above the counter
        flare_sin, flare_cos = math.sin(math.radians(42.0)), math.cos(math.radians(42.0))
        z_raw = np.maximum(z0 - ds * flare_cos, 0.012)
        r_raw = r0 + ds * flare_sin
        p[skirt, :2] = u[skirt] * r_raw[:, None]
        p[skirt, 2] = z_raw
        self.x = self.from_bowl(p)
        self.x[:, 2] = np.maximum(self.x[:, 2], self.table_z + THICK)
        # Bend-edge rest lengths capture the draped crease while intact, then relax toward flat once unsealed
        self.rest = self.rest_flat.copy()
        bend = self.kind == 2
        self.rest[bend] = np.linalg.norm(p[self.edges[bend, 1]] - p[self.edges[bend, 0]], axis=1)
        self.v[:] = 0; self.pins = {}; self.prev_boxes = None
        cling = (on_top & (d >= BOWL_R - 0.012)) | clung
        self.tack = cling.copy(); self.tack0 = cling.copy()
        loc = p.copy()
        nl = np.zeros((self.n, 3))
        nl[on_top] = [0.0, 0.0, 1.0]
        nl[clung] = np.c_[u[clung], np.zeros(clung.sum())]
        self.tack_loc = loc; self.tack_nrm = nl
        self.broke_t[:] = np.inf; self.broke_seq = []
        self.touched[:] = False
        self.peak_strain = 1.0; self.bad = False; self.time = 0.0; self.open_t[:] = 0.0
        self.bowl_force[:] = 0; self.bowl_torque[:] = 0

    # -- pinches ---------------------------------------------------------------------------------------------------
    def _update_pins(self, tcp, rot, aperture):
        for arm in range(len(tcp)):
            if aperture[arm] > PINCH_CLOSE:
                self.open_t[arm] = self.time
            if arm in self.pins:
                nodes, off = self.pins[arm]
                target = tcp[arm] + off @ rot[arm].T
                slip = np.linalg.norm(self.x[nodes] - target, axis=1).mean()
                if aperture[arm] > PINCH_OPEN or slip > PINCH_SLIP:
                    del self.pins[arm]
                continue
            if aperture[arm] > PINCH_CLOSE or self.time - self.open_t[arm] > PINCH_RECENT:
                continue
            local = (self.x - tcp[arm]) @ rot[arm] - (PIN_C - [0.0, 0.0, 0.134])
            ok = np.all(np.abs(local) < PIN_BOX, axis=1)
            for other in self.pins.values():
                ok[other[0]] = False
            ok &= ~self.tack
            if not ok.any():
                continue
            nodes = np.flatnonzero(ok)
            self.pins[arm] = (nodes, (self.x[nodes] - tcp[arm]) @ rot[arm])
            self.touched[nodes] = True

    # -- contacts --------------------------------------------------------------------------------------------------
    def _add(self, idx, dn, nrm, mu, mv=None):
        self._c.append((np.asarray(idx), np.asarray(dn, float), np.asarray(nrm, float), np.full(len(idx), mu),
                        np.zeros((len(idx), 3)) if mv is None else mv))

    def _collide(self, pinned, boxes, prevb):
        x = self.x
        free = ~pinned
        low = (x[:, 2] < self.table_z + THICK) & free
        if low.any():
            i = np.flatnonzero(low)
            self._add(i, self.table_z + THICK - x[i, 2], np.tile([0, 0, 1.0], (len(i), 1)), MU_TABLE)
            x[i, 2] = self.table_z + THICK
        d, nrm = self._bowl_sdf(x)
        hit = (d < THICK) & free
        if hit.any():
            i = np.flatnonzero(hit)
            dep = THICK - d[i]
            x[i] += dep[:, None] * nrm[i]
            self._add(i, dep, nrm[i], MU_BOWL)
        for lo, hi, mu_s in self.static_aabbs:
            ins = np.all((x > lo - THICK) & (x < hi + THICK), axis=1) & free
            if not ins.any():
                continue
            i = np.flatnonzero(ins)
            xi = x[i]
            aa, bb = xi - (lo - THICK), (hi + THICK) - xi
            mm = np.minimum(aa, bb)
            ax = np.argmin(mm, axis=1)
            r = np.arange(len(i))
            lower = aa[r, ax] < bb[r, ax]
            newx = xi.copy()
            newx[r, ax] = np.where(lower, lo[ax] - THICK, hi[ax] + THICK)
            delta = newx - xi
            x[i] = newx
            nloc = np.zeros_like(newx)
            nloc[r, ax] = np.where(lower, -1.0, 1.0)
            self._add(i, np.linalg.norm(delta, axis=1) + 1e-4, nloc, mu_s)
        for bi, (world, lo, hi, arm) in enumerate(boxes):
            Rm, t = world[:3, :3], world[:3, 3]
            l = (x - t) @ Rm
            ins = np.all((l > lo - THICK) & (l < hi + THICK), axis=1) & free & (~self.tack)
            if not ins.any():
                continue
            i = np.flatnonzero(ins)
            li = l[i]
            pw = prevb[bi] if prevb is not None else world
            pl = (x[i] - pw[:3, 3]) @ pw[:3, :3]
            out_lo = (lo - THICK) - pl; out_hi = pl - (hi + THICK)
            outv = np.maximum(out_lo, out_hi)
            aa, bb = li - (lo - THICK), (hi + THICK) - li
            mm = np.minimum(aa, bb); ax = np.argmin(mm, axis=1); r = np.arange(len(i))
            lower = aa[r, ax] < bb[r, ax]
            was = outv.max(axis=1) > 0
            if was.any():
                k2 = np.argmax(outv[was], axis=1)
                ax[was] = k2
                rr = np.flatnonzero(was)
                lower[was] = out_lo[rr, k2] > out_hi[rr, k2]
            newl = li.copy()
            newl[r, ax] = np.where(lower, lo[ax] - THICK, hi[ax] + THICK)
            delta = (newl - li) @ Rm.T
            x[i] += delta
            nloc = np.zeros_like(newl); nloc[r, ax] = np.where(lower, -1.0, 1.0)
            mv = (newl @ Rm.T + t) - (newl @ pw[:3, :3].T + pw[:3, 3])
            self._add(i, np.linalg.norm(delta, axis=1) + 1e-4, nloc @ Rm.T, MU_PAD, mv / self._sub)
            self.touched[i] = True

    def _friction(self, prev, pinned):
        if not self._c:
            return
        idx = np.concatenate([c[0] for c in self._c]); dn = np.concatenate([c[1] for c in self._c])
        n = np.concatenate([c[2] for c in self._c]); mu = np.concatenate([c[3] for c in self._c])
        mv = np.concatenate([c[4] for c in self._c])
        keep = ~pinned[idx]
        idx, dn, n, mu, mv = idx[keep], dn[keep], n[keep], mu[keep], mv[keep]
        if not len(idx):
            return
        order = np.lexsort((-(mu * dn), idx))
        idx, dn, n, mu, mv = idx[order], dn[order], n[order], mu[order], mv[order]
        first = np.r_[True, idx[1:] != idx[:-1]]
        tot = np.zeros(self.n); np.add.at(tot, idx, dn)
        idx, n, mu, mv = idx[first], n[first], mu[first], mv[first]
        dn = tot[idx]
        dx = (self.x[idx] - prev[idx]) - mv
        t = dx - (dx * n).sum(1, keepdims=True) * n
        tl = np.linalg.norm(t, axis=1)
        lim = mu * dn
        f = np.where(tl <= lim, 1.0, lim / np.maximum(tl, 1e-12))
        self.x[idx] -= t * f[:, None]

    def _tacks(self, w, a_tack, lam):
        i = np.flatnonzero(self.tack)
        if not len(i):
            return
        anc = self.from_bowl(self.tack_loc[i])
        d = self.x[i] - anc
        ln = np.linalg.norm(d, axis=1)
        dl = (-ln - a_tack * lam[i]) / np.maximum(w[i] + a_tack, 1e-12)
        lam[i] += dl
        g = d / np.maximum(ln, 1e-12)[:, None]
        self.x[i] += (w[i] * dl)[:, None] * g
        self._tdir[i] = g

    # -- main step -------------------------------------------------------------------------------------------------
    def step(self, tcp, rot, aperture, boxes=(), bowl=None, dt=0.05, substeps=8, iters=10):
        tcp = np.asarray(tcp, float); rot = np.asarray(rot, float)
        if bowl is not None:
            self.set_bowl(np.asarray(bowl[0], float), np.asarray(bowl[1], float))
        self.time += dt
        self._update_pins(tcp, rot, aperture)
        # Once all tacks break, relax bend rest lengths smoothly so the loose sheet crumples naturally in the bin
        if not self.tack.any():
            bend = self.kind == 2
            self.rest[bend] += 0.15 * (0.65 * self.rest_flat[bend] - self.rest[bend])
        h = dt / substeps
        self._sub = substeps
        a_edge = self.compliance / (h * h)
        a_tack = 1.0 / (TACK_K * h * h)
        E, R = self.edges, self.rest
        w = self.inv_m.copy()
        pinned = np.zeros(self.n, bool)
        targets = []
        for arm, (nodes, off) in self.pins.items():
            targets.append((nodes, self.x[nodes].copy(), tcp[arm] + off @ rot[arm].T))
            pinned[nodes] = True; w[nodes] = 0.0
        cur = [bx[0] for bx in boxes]
        prevb = self.prev_boxes if (self.prev_boxes is not None and len(self.prev_boxes) == len(cur)) else None
        for s in range(substeps):
            fr = (s + 1) / substeps
            prev = self.x.copy()
            self.v *= DAMP
            self.v[:, 2] -= 9.81 * h
            self.x += self.v * h
            for nodes, start, tgt in targets:
                self.x[nodes] = start + fr * (tgt - start)
            bx = []
            for bi, (world, lo, hi, arm) in enumerate(boxes):
                wd = world
                if prevb is not None:
                    wd = world.copy(); wd[:3, 3] = prevb[bi][:3, 3] + fr * (world[:3, 3] - prevb[bi][:3, 3])
                bx.append((wd, lo, hi, arm))
            last = prevb if s == 0 else last_bx
            last_bx = [b[0] for b in bx]
            lam = np.zeros(len(E))
            lam_t = np.zeros(self.n)
            self._tdir = np.zeros((self.n, 3))
            self._c = []
            for it in range(iters):
                self._tacks(w, a_tack, lam_t)
                for c in self.colors:
                    a, b = E[c, 0], E[c, 1]
                    d = self.x[b] - self.x[a]
                    ln = np.linalg.norm(d, axis=1)
                    C = ln - R[c]
                    dl = (-C - a_edge[c] * lam[c]) / np.maximum(w[a] + w[b] + a_edge[c], 1e-12)
                    tens = self._tension_only[c]
                    dl = np.where(tens, np.minimum(lam[c] + dl, 0.0) - lam[c], dl)
                    lam[c] += dl
                    g = d / np.maximum(ln, 1e-12)[:, None]
                    self.x[a] -= (w[a] * dl)[:, None] * g
                    self.x[b] += (w[b] * dl)[:, None] * g
                self._collide(pinned, bx, last)
            self._friction(prev, pinned)
            self._lam_t, self._h = lam_t, h
            self.v = (self.x - prev) / h
            sp = np.linalg.norm(self.v, axis=1)
            self.v *= np.minimum(1.0, 2.5 / np.maximum(sp, 1e-9))[:, None]
        self.prev_boxes = cur
        self._tack_forces()
        st = self.strain(98)
        self.peak_strain = max(self.peak_strain, st)
        if not np.isfinite(self.x).all() or st > 4.5:
            self.bad = True
            self.x = np.nan_to_num(self.x)
        return self.x

    def _tack_forces(self):
        """Each intact tack's spring pull (film on bowl), its break test, and the total wrench on the bowl."""
        i = np.flatnonzero(self.tack)
        self.bowl_force = np.zeros(3); self.bowl_torque = np.zeros(3)
        if not len(i):
            return
        anc = self.from_bowl(self.tack_loc[i])
        F = TACK_K * (self.x[i] - anc)
        nw = self.from_bowl(self.tack_nrm[i], vec=True)
        fn = (F * nw).sum(1)
        ft = np.linalg.norm(F - fn[:, None] * nw, axis=1)
        brk = (fn > PEEL_N) | (ft > SHEAR_N)
        if brk.any():
            for k in i[brk]:
                p = self.tack_loc[k]
                self.broke_seq.append((self.time, math.atan2(p[1], p[0]), int(k)))
            self.tack[i[brk]] = False
            self.broke_t[i[brk]] = self.time
        keep = ~brk
        if keep.any():
            Fk = F[keep]
            bowl_c = self.bowl_T[:3, 3] + self.bowl_T[:3, :3] @ np.array([0.0, 0.0, BOWL_H / 2])
            self.bowl_force = Fk.sum(0)
            self.bowl_torque = np.cross(anc[keep] - bowl_c, Fk).sum(0)

    def strain(self, pct=None):
        s = self._tension_only
        ln = np.linalg.norm(self.x[self.edges[s, 1]] - self.x[self.edges[s, 0]], axis=1)
        r = ln / self.rest_flat[s]
        return float(np.max(r) if pct is None else np.percentile(r, pct))
