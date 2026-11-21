"""First-party reduced position-based model of a flat-laid piece of fabric, written in Warp for this task (no
third-party assets). It takes no phase or goal inputs; the sewing machine's foot patch is passed in as prescribed
node targets (``ext``).

The rigid simulator supplies, every control step, the measured TCP poses and jaw openings and the finger boxes; this
module moves the cloth and nothing else (it exerts no force on the rigid bodies).
- Grid of nodes over the flat fabric outline (a rectangular grid with the cells off the outline removed; the cell
  size is set by the caller: 8 x 8 mm here). In-plane springs (edges and diagonals) are stiff in tension and soft in compression (cloth buckles out of
  plane rather than shortening).
- Bending: skip-one springs along rows, columns and both diagonals. Fold memory: when a bending spring's strain passes
  a yield band its rest length follows, so a laid fold stays folded after release.
- Contact: table plane and the finger boxes, with friction on contact nodes.
- Layers: no cloth-on-cloth collision. Each node gets a layer index (the fold in which it first left the table) and
  rests that many cloth thicknesses above its support, so later folds lie over earlier ones.
- Grasp rule: a jaw that closes its pads onto the cloth (pad gap down to the cloth thickness + 1 mm, after being
  open) grips the cloth EDGES its pads closed across: edge nodes inside the pads' footprint (22 mm wide, 20 mm up from
  the pad tips, 1 mm of pad give at the tips) on an edge running between the pad faces. That is one edge, or several
  edges stacked by earlier folds. A jaw that is already shut (pressing) picks nothing up. A gripped node lets go when
  the jaw opens past 23 mm or the node is pulled 3.5 cm from the fingers.
- Tear: stretch beyond 1.8x for 0.25 s (or 3x at once) latches the cloth as torn.
Serial Gauss-Seidel in a Warp CPU kernel. Uncalibrated stand-in, not a cloth FEM.
"""
from __future__ import annotations

import numpy as np
import warp as wp

wp.config.quiet = True
_DEV = "cpu"


@wp.kernel
def substeps_kernel(x: wp.array(dtype=wp.vec3), v: wp.array(dtype=wp.vec3), w: wp.array(dtype=float),
                    target: wp.array(dtype=wp.vec3), ca: wp.array(dtype=int), cb: wp.array(dtype=int),
                    crest: wp.array(dtype=float), cflat: wp.array(dtype=float), ck: wp.array(dtype=float),
                    cpl: wp.array(dtype=int), layer_off: wp.array(dtype=float), contact: wp.array(dtype=int),
                    old: wp.array(dtype=wp.vec3), cnrm: wp.array(dtype=wp.vec3), pred: wp.array(dtype=wp.vec3), muc: float, mut: float,
                    epos: wp.vec3, erot: wp.mat33,
                    pl_c: wp.array(dtype=wp.vec3), pl_h: wp.array(dtype=wp.vec3), pl_ax: wp.array(dtype=int), npl: int,
                    table_z: float,
                    box_T: wp.array(dtype=wp.mat44), box_t0: wp.array(dtype=wp.vec3), box_lo: wp.array(dtype=wp.vec3), box_hi: wp.array(dtype=wp.vec3),
                    nbox: int, h: float, nsub: int, iters: int, mu: float, ey: float, damp: float,
                    kcomp: float, eyc: float, cmin: float):
    n = x.shape[0]
    nc = ca.shape[0]
    ert = wp.transpose(erot)
    for s in range(nsub):
        for i in range(n):
            old[i] = x[i]
            contact[i] = 0
            if w[i] > 0.0:
                vi = v[i] * damp
                vi = vi + wp.vec3(0.0, 0.0, -9.81 * h)
                v[i] = vi
                x[i] = x[i] + vi * h
            else:
                x[i] = target[i]
            pred[i] = x[i]
        for it in range(iters):
            for c in range(nc):
                a = ca[c]
                b = cb[c]
                wa = w[a]
                wb = w[b]
                ws = wa + wb
                if ws > 0.0:
                    d = x[b] - x[a]
                    ln = wp.length(d)
                    if ln > 1.0e-9:
                        kk = ck[c]
                        if cpl[c] == 2 and ln < crest[c]:
                            kk = kk * kcomp
                        corr = d * ((ln - crest[c]) / ln * kk / ws)
                        x[a] = x[a] + corr * wa
                        x[b] = x[b] - corr * wb
            # collisions
            for i in range(n):
                if w[i] > 0.0:
                    p = x[i]
                    off = layer_off[i]
                    # fixed thin plates, hit from the side the node came from (none in this task)
                    q = ert * (p - epos)
                    qo = ert * (old[i] - epos)
                    for k in range(npl):
                        pc = pl_c[k]
                        hh = pl_h[k]
                        ax = pl_ax[k]
                        r = q - pc
                        ro = qo - pc
                        m = 0.0003 + off
                        inside = True
                        for j in range(3):
                            if j != ax:
                                if wp.abs(r[j]) > hh[j] + 0.0002:
                                    inside = False
                        if inside:
                            rn = r[ax]
                            rno = ro[ax]
                            lim = hh[ax] + m
                            crossed = (rn > -lim and rn < lim) or (rn * rno < 0.0)
                            if crossed:
                                sg = 1.0
                                if rno < 0.0:
                                    sg = -1.0
                                if ax == 0:
                                    r = wp.vec3(sg * lim, r[1], r[2])
                                elif ax == 1:
                                    r = wp.vec3(r[0], sg * lim, r[2])
                                else:
                                    r = wp.vec3(r[0], r[1], sg * lim)
                                q = r + pc
                                nl = wp.vec3(0.0, 0.0, 0.0)
                                if ax == 0:
                                    nl = wp.vec3(sg, 0.0, 0.0)
                                elif ax == 1:
                                    nl = wp.vec3(0.0, sg, 0.0)
                                else:
                                    nl = wp.vec3(0.0, 0.0, sg)
                                contact[i] = 1
                                cnrm[i] = erot * nl
                    p = erot * q + epos
                    if p[2] < table_z + 0.0006 + off:
                        p = wp.vec3(p[0], p[1], table_z + 0.0006 + off)
                        contact[i] = 1
                        if off < 1.0e-6:
                            contact[i] = 2  # cloth directly on the bed (not on an earlier layer)
                        cnrm[i] = wp.vec3(0.0, 0.0, 1.0)
                    # finger boxes
                    for k in range(nbox):
                        T = box_T[k]
                        R = wp.mat33(T[0, 0], T[0, 1], T[0, 2], T[1, 0], T[1, 1], T[1, 2], T[2, 0], T[2, 1], T[2, 2])
                        t1 = wp.vec3(T[0, 3], T[1, 3], T[2, 3])
                        fr = float(s + 1) / float(nsub)
                        t = box_t0[k] + (t1 - box_t0[k]) * fr  # the fingers sweep through the control step
                        qq = wp.transpose(R) * (p - t)
                        lo = box_lo[k] - wp.vec3(0.0008, 0.0008, 0.0008)
                        hi = box_hi[k] + wp.vec3(0.0008, 0.0008, 0.0008)
                        if qq[0] > lo[0] and qq[0] < hi[0] and qq[1] > lo[1] and qq[1] < hi[1] and qq[2] > lo[2] and qq[2] < hi[2]:
                            best = 1.0e9
                            bx = 0
                            side = 0.0
                            for j in range(3):
                                dl = qq[j] - lo[j]
                                dh = hi[j] - qq[j]
                                if dl < best:
                                    best = dl
                                    bx = j
                                    side = lo[j]
                                if dh < best:
                                    best = dh
                                    bx = j
                                    side = hi[j]
                            if bx == 0:
                                qq = wp.vec3(side, qq[1], qq[2])
                            elif bx == 1:
                                qq = wp.vec3(qq[0], side, qq[2])
                            else:
                                qq = wp.vec3(qq[0], qq[1], side)
                            p = R * qq + t
                            contact[i] = 1
                            cnrm[i] = R * wp.vec3(0.0, 0.0, -1.0)
                    x[i] = p
        # friction: Coulomb (tangential slip removed up to muc x the normal push-out this substep) plus a small
        # viscous share mu
        for i in range(n):
            if contact[i] > 0 and w[i] > 0.0:
                mui = mu
                if contact[i] == 2:
                    mui = mut
                dxs = x[i] - old[i]
                nn = cnrm[i]
                dn = wp.dot(dxs, nn)
                tang = dxs - nn * dn
                push = wp.max(wp.dot(x[i] - pred[i], nn), 0.0)
                tl = wp.length(tang)
                if tl > 1.0e-12:
                    cut = wp.min(tl, muc * push + mui * tl)
                    x[i] = x[i] - tang * (cut / tl)
        # fold memory
        for c in range(nc):
            if cpl[c] == 1:
                ln = wp.length(x[cb[c]] - x[ca[c]])
                r0 = crest[c]
                if ln < r0 * (1.0 - ey):
                    crest[c] = ln / (1.0 - ey)
                elif ln > r0 * (1.0 + ey):
                    crest[c] = wp.min(cflat[c], ln / (1.0 + ey))
            elif cpl[c] == 2:
                ln = wp.length(x[cb[c]] - x[ca[c]])
                r0 = crest[c]
                if ln < r0 * (1.0 - eyc):
                    crest[c] = wp.max(cflat[c] * cmin, ln / (1.0 - eyc))
                elif ln > r0:
                    crest[c] = wp.min(cflat[c], ln)
        for i in range(n):
            vi = (x[i] - old[i]) / h
            sp = wp.length(vi)
            if sp > 2.0:
                vi = vi * (2.0 / sp)
            v[i] = vi


class Sheet:
    def __init__(self, x0, y0, size, z, *, table_z, cells, mask=None, bend_k=0.5, yield_strain=0.02, shear_k=0.8,
                 iters=12, substeps=10, thickness=0.0015, kcomp=0.6, eyc=0.08, cmin=0.9, mu=0.8, muc=0.0, mut=None):
        """x0, y0: corner of the grid's bounding rectangle with the lowest x and y; size (sx, sy); cells (ncx, ncy);
        mask(u, v) -> bool keeps a grid node (u, v metres from the x0/y0 corner) when it lies on the garment."""
        sx, sy = size
        ncx, ncy = cells
        self.nx, self.ny = gx, gy = ncx + 1, ncy + 1
        self.dx, self.dy = dx, dy = sx / ncx, sy / ncy
        self.size = (sx, sy)
        U, V = np.meshgrid(np.arange(gx) * dx, np.arange(gy) * dy, indexing="ij")
        keep = np.ones((gx, gy), bool) if mask is None else np.vectorize(mask)(U, V)
        ids = np.full((gx, gy), -1)
        ids[keep] = np.arange(int(keep.sum()))
        self.ids = ids
        self.uv = np.stack([U[keep], V[keep]], 1)
        self.x = np.stack([x0 + U[keep], y0 + V[keep], np.full(int(keep.sum()), z)], 1)
        self.v = np.zeros_like(self.x)
        self.iters, self.substeps, self.th = iters, substeps, thickness
        self.kcomp, self.eyc, self.cmin = kcomp, eyc, cmin
        self.table_z, self.mu, self.ey, self.muc = table_z, mu, yield_strain, muc
        self.mut = float(mu if mut is None else mut)  # friction of cloth lying directly on the table
        self.mut_folds = 99
        self.pinch_gap = thickness + 0.001  # the pads must close on the cloth: gap down to its thickness + 1 mm
        self.pl_c = np.zeros((1, 3), np.float32); self.pl_h = np.zeros((1, 3), np.float32)
        self.pl_ax = np.zeros(1, np.int32)
        A, B, R0, K, P = [], [], [], [], []

        def add(a, b, rest, k, pl=0):
            a = np.asarray(a).ravel(); b = np.asarray(b).ravel()
            m = (a >= 0) & (b >= 0)
            A.append(a[m]); B.append(b[m]); R0.append(np.full(int(m.sum()), rest)); K.append(np.full(int(m.sum()), k))
            P.append(np.full(int(m.sum()), pl))

        cp = 2
        dg = float(np.hypot(dx, dy))
        add(ids[:, :-1], ids[:, 1:], dy, 1.0, cp)
        add(ids[:-1, :], ids[1:, :], dx, 1.0, cp)
        add(ids[:-1, :-1], ids[1:, 1:], dg, shear_k, cp)
        add(ids[:-1, 1:], ids[1:, :-1], dg, shear_k, cp)
        add(ids[:, :-2], ids[:, 2:], 2 * dy, bend_k, 1)
        add(ids[:-2, :], ids[2:, :], 2 * dx, bend_k, 1)
        add(ids[:-2, :-2], ids[2:, 2:], 2 * dg, bend_k, 1)
        add(ids[:-2, 2:], ids[2:, :-2], 2 * dg, bend_k, 1)
        self.ca, self.cb = np.concatenate(A), np.concatenate(B)
        self.crest0 = np.concatenate(R0)
        self.crest = self.crest0.copy()
        self.ck, self.cpl = np.concatenate(K), np.concatenate(P)
        f = []
        for i in range(gx - 1):
            for j in range(gy - 1):
                a, b, c, d = ids[i, j], ids[i + 1, j], ids[i + 1, j + 1], ids[i, j + 1]
                if min(a, b, c, d) >= 0:
                    f.extend([(a, b, c), (a, c, d)])
        self.faces = np.asarray(f, np.int32)
        # edge nodes: on the garment with a missing 4-neighbour; their two neighbours along the edge (to find which
        # way it runs), corners keep (-1, -1)
        pad = np.pad(ids, 1, constant_values=-1)
        edge = keep & ((pad[:-2, 1:-1] < 0) | (pad[2:, 1:-1] < 0) | (pad[1:-1, :-2] < 0) | (pad[1:-1, 2:] < 0))
        self.edge_nodes = ids[edge]
        nbl = []
        for i, j in zip(*np.nonzero(edge)):
            nb = [ids[i + a, j + b] for a, b in ((1, 0), (-1, 0), (0, 1), (0, -1))
                  if 0 <= i + a < gx and 0 <= j + b < gy and edge[i + a, j + b]]
            nbl.append(nb[:2] if len(nb) == 2 else [-1, -1])
        self.edge_nb = np.array(nbl, int)
        self.pins, self.armed, self.sweep = {}, {}, {}
        self.overstrain = 0.0
        self.torn = self.bad = False
        N = len(self.x)
        self.layer = np.full(N, -1)
        self.fold = 0
        self.frozen = False
        self._pinned_before = False
        self.max_strain = 1.0
        d = _DEV
        self._x = wp.array(self.x, dtype=wp.vec3, device=d)
        self._v = wp.array(self.v, dtype=wp.vec3, device=d)
        self._ca = wp.array(self.ca.astype(np.int32), dtype=int, device=d)
        self._cb = wp.array(self.cb.astype(np.int32), dtype=int, device=d)
        self._crest = wp.array(self.crest, dtype=float, device=d)
        self._cflat = wp.array(self.crest0, dtype=float, device=d)
        self._ck = wp.array(self.ck, dtype=float, device=d)
        self._cpl = wp.array(self.cpl.astype(np.int32), dtype=int, device=d)
        self._contact = wp.zeros(N, dtype=int, device=d)
        self._old = wp.zeros(N, dtype=wp.vec3, device=d)
        self._cn = wp.zeros(N, dtype=wp.vec3, device=d)
        self._pred = wp.zeros(N, dtype=wp.vec3, device=d)
        self._plc = wp.array(self.pl_c, dtype=wp.vec3, device=d)
        self._plh = wp.array(self.pl_h, dtype=wp.vec3, device=d)
        self._pla = wp.array(self.pl_ax, dtype=int, device=d)
        self.contact = np.zeros(N, bool)

    # ------------------------------------------------------------------ grasp rule
    def update_pins(self, tcp, rot, aperture):
        """pins[arm] = list of (node, offset in TCP frame). A closing jaw grips every sheet edge lying between its
        fingertips (one edge, or several edges stacked by earlier folds)."""
        for arm in range(len(tcp)):
            if arm in self.pins:
                keep = []
                for node, off in self.pins[arm]:
                    target = tcp[arm] + rot[arm] @ off
                    if aperture[arm] <= 0.023 and np.linalg.norm(self.x[node] - target) <= 0.035:
                        keep.append((node, off))
                if keep:
                    self.pins[arm] = keep
                else:
                    del self.pins[arm]
                continue
            hist = self.sweep.setdefault(arm, [])
            hist.append(float(aperture[arm]))
            del hist[:-20]
            if aperture[arm] > 0.020:
                self.armed[arm] = True
            if aperture[arm] > self.pinch_gap or not self.armed.get(arm, False):  # pads (almost) shut on it
                continue
            self.armed[arm] = False
            # Two-pad contact + closing jaws: require cloth inside both closed pads' 12 x 22 x 20 mm contact zones
            # (never proximity outside the pads) and an edge crossing between the two pads.
            pad_outer = self.pinch_gap / 2 + 0.012
            all_loc = (self.x - tcp[arm]) @ rot[arm]
            in_yz = (np.abs(all_loc[:, 1]) < 0.011) & (all_loc[:, 2] > -0.007) & (all_loc[:, 2] < 0.014)
            two_pad = (bool(np.any(in_yz & (all_loc[:, 0] >= -0.004) & (all_loc[:, 0] <= pad_outer)))
                       and bool(np.any(in_yz & (all_loc[:, 0] <= 0.004) & (all_loc[:, 0] >= -pad_outer))))
            if not two_pad:
                continue
            cand = self.edge_nodes
            local = (self.x[cand] - tcp[arm]) @ rot[arm]  # TCP frame: x closing axis, z approach
            # inside the closed pads' footprint: across their 22 mm width and 20 mm length up from the pad tips
            good = (np.abs(local[:, 0]) <= pad_outer) & (np.abs(local[:, 1]) < 0.011) & (local[:, 2] > -0.007) \
                & (local[:, 2] < 0.014)
            # only an edge that crosses the gap between the pads is pinched; one running along the closing axis is
            # pushed aside by the pads, not gripped
            nbr = self.edge_nb
            tan = np.where((nbr[:, :1] >= 0), self.x[np.maximum(nbr[:, 1], 0)] - self.x[np.maximum(nbr[:, 0], 0)], 0.0)
            tl = np.linalg.norm(tan, axis=1)
            along = np.abs(tan @ rot[arm][:, 0]) / np.maximum(tl, 1e-9)
            good &= (tl < 1e-9) | (along < 0.7)
            taken = {nd for lst in self.pins.values() for nd, _ in lst}
            sel = [c for c in cand[good][np.argsort(np.linalg.norm(local[good], axis=1))] if c not in taken][:16]
            if sel:
                offs = [(self.x[c] - tcp[arm]) @ rot[arm] for c in sel]
                for o in offs:
                    o[0] = np.clip(o[0], -self.pinch_gap / 2, self.pinch_gap / 2)  # squeezed between the pads
                self.pins[arm] = [(int(c), o) for c, o in zip(sel, offs)]
        if self.pins and not self._pinned_before:
            self.fold += 1
        self._pinned_before = bool(self.pins)

    # ------------------------------------------------------------------ step
    def step(self, tcp, rot, aperture, finger_boxes=(), dt=0.05, ext=None):
        env_pos, env_rot = np.zeros(3), np.eye(3)
        self.update_pins(tcp, rot, aperture)
        x = self.x
        ep, eR = np.asarray(env_pos, float), np.asarray(env_rot, float)
        N = len(x)
        w = np.ones(N, np.float32)
        target = x.copy()
        for k, lst in self.pins.items():
            for node, off in lst:
                target[node] = tcp[k] + rot[k] @ off
                w[node] = 0.0
        if ext:
            for node, tgt in ext.items():
                target[node] = tgt
                w[node] = 0.0
        layer_off = (np.maximum(self.layer, 0) * self.th).astype(np.float32)
        T = np.zeros((max(1, len(finger_boxes)), 4, 4), np.float32)
        lo = np.zeros((max(1, len(finger_boxes)), 3), np.float32)
        hi = np.zeros_like(lo)
        for k, (Tk, l, hgh) in enumerate(finger_boxes):
            T[k], lo[k], hi[k] = Tk, l, hgh
        t0 = T[:, :3, 3].copy()
        prev = getattr(self, "_box_prev", None)
        if prev is not None and len(prev) == len(t0):
            t0 = prev.astype(np.float32)
        self._box_prev = T[:, :3, 3].copy()
        d = _DEV
        self._x.assign(x.astype(np.float32))
        self._v.assign(self.v.astype(np.float32))
        wp.launch(substeps_kernel, dim=1, device=d, inputs=[
            self._x, self._v, wp.array(w, dtype=float, device=d),
            wp.array(target.astype(np.float32), dtype=wp.vec3, device=d),
            self._ca, self._cb, self._crest, self._cflat, self._ck, self._cpl,
            wp.array(layer_off, dtype=float, device=d), self._contact, self._old, self._cn, self._pred, self.muc, (self.mut if self.fold <= self.mut_folds else self.mu),
            wp.vec3(*ep), wp.mat33(*eR.ravel()), self._plc, self._plh, self._pla, 0,
            float(self.table_z),
            wp.array(T, dtype=wp.mat44, device=d), wp.array(t0, dtype=wp.vec3, device=d), wp.array(lo, dtype=wp.vec3, device=d),
            wp.array(hi, dtype=wp.vec3, device=d), len(finger_boxes),
            dt / self.substeps, self.substeps, self.iters, self.mu, self.ey, 0.985,
            self.kcomp, self.eyc, self.cmin])
        self.x = x = self._x.numpy().astype(np.float64)
        self.v = self._v.numpy().astype(np.float64)
        self.contact = self._contact.numpy().astype(bool)
        lifted = (x[:, 2] - self.table_z) > 0.006
        if not self.frozen:
            if lifted.mean() > 0.6:  # the whole sheet is being moved: no new layers from now on
                self.frozen = True
                self.layer[self.layer < 0] = 0
            else:
                new = lifted & (self.layer < 0)
                self.layer[new] = max(self.fold, 1)
        st = np.linalg.norm(x[self.cb] - x[self.ca], axis=1)
        m = self.cpl != 1
        self.max_strain = float((st[m] / self.crest0[m]).max())
        self.overstrain = self.overstrain + dt if self.max_strain > 1.8 else max(0.0, self.overstrain - 0.1 * dt)
        if not np.isfinite(x).all():
            self.bad = True
        self.torn |= self.bad or self.max_strain > 3.0 or self.overstrain > 0.25
        return x

    def triangles(self):
        return self.x[self.faces]

    def at(self, u, v):
        """Bilinear point of the sheet at flat-sheet coordinates (u along x rows, v along y columns, metres from the
        x0/y0 corner)."""
        fi, fj = u / self.dx, v / self.dy
        i0 = int(np.clip(np.floor(fi), 0, self.nx - 2)); j0 = int(np.clip(np.floor(fj), 0, self.ny - 2))
        a, b = fi - i0, fj - j0
        I = self.ids
        X = self.x
        return ((1 - a) * (1 - b) * X[I[i0, j0]] + a * (1 - b) * X[I[i0 + 1, j0]] + (1 - a) * b * X[I[i0, j0 + 1]]
                + a * b * X[I[i0 + 1, j0 + 1]])
