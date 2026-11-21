"""A 10-sheet 3-hole-punched letter handout as a reduced position-based (XPBD) sheet, and a 1 in round-ring binder's
over-centre ring mechanism with end booster tabs. Pure NumPy; driven only by measured robot state (TCP poses, jaw
openings and gripper collision boxes).

Sheet: 22 x 15 lattice over 279.4 x 215.9 mm (11 x 8.5 in, ~1.35 cm spacing), mass 46 g (ten sheets of 20 lb / 75 g/m2
bond), thickness 1.1 mm, bending stiffness 3.3e-3 N m (ten unbonded sheets). Inextensible stretch/shear + tension-only
long-range tethers, gravity, light air drag, and Coulomb friction against the desk, punch, binder covers, ring wires
and gripper boxes.
Pinch: closing a jaw (aperture <= PINCH_CLOSE) with a sheet border node inside the finger-pad pinch box clamps the
patch between the pads in that gripper's frame; opening (> PINCH_OPEN) or slipping (> PINCH_SLIP) releases it.
Holes: three 9/32 in (7.14 mm dia, R = 3.57 mm) holes on 4.25 in (107.95 mm) centres (215.9 mm outer spacing), centred
9.525 mm (3/8 in) in from the 279.4 mm punched edge. Each hole sits strictly inside one lattice quad and is rendered as
an actual 8-sided cut-out opening through the mesh.
Rings: three 1 in round rings (25.4 mm inside dia, 3.5 mm wire) on one 27.0 cm spine plate with two angled booster tabs
at its ends. All six half-rings share one opening angle (26 deg each side when open) held by an over-centre toggle
spring. Closing requires either pinching both halves of a ring inward at once with finger pads or pushing an end
booster tab inward (>= 8 mm); passing 12 deg snaps all three rings shut. An open ring's rounded tip enters the sheet
only within CAPTURE_R (3.2 mm) of a hole centre; closing the rings onto unthreaded paper or pulling a threaded hole
more than TEAR_D off its wire marks the handout torn/pinched.
"""
from __future__ import annotations

import math

import numpy as np

# -- 10-sheet US letter handout (m) ---------------------------------------------------------------------------------
W_U = 0.2794                  # 11.0 in along the punched edge
L_V = 0.2159                  # 8.5 in perpendicular to the punched edge
NU, NV = 22, 15               # hu = 13.30 mm, hv = 15.42 mm; each hole lies strictly inside quads (0, 2/10/18)
MASS = 0.046                  # 10 sheets of 20 lb (75 g/m2) letter bond
THICK = 0.0011                # 10 sheets (~0.11 mm each)
STRETCH_C = 1e-8              # XPBD compliance (m/N)
SHEAR_C = 2e-7
BEND_D = 3.3e-3               # N m: 10 loose sheets of 20 lb bond
AIR_DRAG = 3.0
MU_SURF = 0.42                # paper on desk / polypropylene binder / metal punch
MU_PAD = 1.0                  # rubber finger pads on paper

HOLE_V = 0.009525             # 3/8 in backgauge from the punched edge (v = 0)
HOLE_SPACE = 0.10795          # 4.25 in adjacent centre-to-centre (215.9 mm first-to-last)
HOLE_U = np.array([W_U / 2 - HOLE_SPACE, W_U / 2, W_U / 2 + HOLE_SPACE])
HOLE_R = 0.00357              # 9/32 in diameter (7.14 mm)
HOLE_SIDES = 8

PINCH_CLOSE = 0.004           # jaw opening (m) at or below which both pads are closed on the 1.1 mm packet and clamp it
PINCH_OPEN = 0.024            # jaw opening (m) above which a pinch lets go
PINCH_SLIP = 0.045            # m: excessive pull tears/slips out of the pads
PIN_BOX = np.array([0.012, 0.018, 0.016])   # half-extents (m) around PIN_C in gripper frame
PIN_C = np.array([0.0, 0.0, 0.0005])
PINCH_RECENT = 0.8

TEAR_D = 0.006                # m beyond wire slack before hole edge tears
TEAR_TIME = 0.15              # s sustained pull to tear

# -- 1 in round-ring mechanism (ring bar along world y at x = ring_x) -----------------------------------------------
WIRE_R = 0.00175              # 3.5 mm wire diameter
RING_IN = 0.0127              # 1.0 in (25.4 mm) inside radius -> 25.4 mm inside diameter
RC = RING_IN + WIRE_R         # 14.45 mm wire centreline radius
CAPTURE_R = 0.0032            # rounded/bevelled ring tip enters 3.57 mm hole within 3.2 mm of hole centre
OPEN_DEG = 26.0               # open half-angle per side (tips ~24 mm apart)
SNAP_DEG = 12.0               # bistable over-centre toggle angle
N_ARC = 16
BOOSTER_LEN = 0.018           # 18 mm end booster tabs at both ends of the 27 cm spine plate
BOOSTER_W = 0.014


class Sheet:
    def __init__(self):
        self.hu = W_U / (NU - 1)
        self.hv = L_V / (NV - 1)
        n = NU * NV
        self.n = n
        idx = np.arange(n).reshape(NV, NU)      # idx[v, u]; v=0 is the punched edge
        self.idx = idx
        self.uv = np.stack(np.meshgrid(np.arange(NU) * self.hu, np.arange(NV) * self.hv), -1).reshape(-1, 2)

        e, kind = [], []

        def add(a, b, k):
            e.append((int(a), int(b)))
            kind.append(k)

        for v in range(NV):
            for u in range(NU):
                if u + 1 < NU:
                    add(idx[v, u], idx[v, u + 1], 0)
                if v + 1 < NV:
                    add(idx[v, u], idx[v + 1, u], 0)
                if u + 1 < NU and v + 1 < NV:
                    add(idx[v, u], idx[v + 1, u + 1], 1)
                    add(idx[v, u + 1], idx[v + 1, u], 1)
        # Tension-only long-range tethers keep paper inextensible while allowing natural bending/droop
        for v in range(NV):
            for step in (4, 10, NU - 1):
                for u in range(0, NU - step):
                    add(idx[v, u], idx[v, u + step], 3)
        for u in range(NU):
            for step in (4, NV - 1):
                for v in range(0, NV - step):
                    add(idx[v, u], idx[v + step, u], 3)

        self.edges = np.array(e)
        self.kind = np.array(kind)
        d = self.uv[self.edges[:, 1]] - self.uv[self.edges[:, 0]]
        self.rest = np.linalg.norm(d, axis=1)
        self.compl = np.where(self.kind == 1, SHEAR_C, STRETCH_C)

        self.tri_groups = []
        for axis in (0, 1):
            for g in range(3):
                t = []
                for v in range(NV):
                    for u in range(NU):
                        if axis == 0 and 1 <= u < NU - 1 and u % 3 == g:
                            t.append((idx[v, u - 1], idx[v, u], idx[v, u + 1]))
                        if axis == 1 and 1 <= v < NV - 1 and v % 3 == g:
                            t.append((idx[v - 1, u], idx[v, u], idx[v + 1, u]))
                h, wd = (self.hu, self.hv) if axis == 0 else (self.hv, self.hu)
                self.tri_groups.append((np.array(t), h ** 3 / (BEND_D * wd)))

        self.inv_m = np.full(n, n / MASS)
        self.colors = self._color()

        # Collision faces (full lattice triangulation for fast point-to-surface checks)
        f = []
        for v in range(NV - 1):
            for u in range(NU - 1):
                a, b, c, dd = idx[v, u], idx[v, u + 1], idx[v + 1, u + 1], idx[v + 1, u]
                f += [(a, b, c), (a, c, dd)]
        self.faces = np.array(f, np.int32)

        self.border = np.zeros(n, bool)
        self.border[idx[0]] = self.border[idx[-1]] = self.border[idx[:, 0]] = self.border[idx[:, -1]] = True

        # Bilinear embedding of the 3 hole centres + real cut-out render mesh topology
        self.hole_nodes, self.hole_w, self.hole_quads = [], [], []
        for hu in HOLE_U:
            iu = min(NU - 2, max(0, int(hu // self.hu)))
            iv = min(NV - 2, max(0, int(HOLE_V // self.hv)))
            fu, fv = hu / self.hu - iu, HOLE_V / self.hv - iv
            self.hole_quads.append((iv, iu))
            self.hole_nodes.append([idx[iv, iu], idx[iv, iu + 1], idx[iv + 1, iu], idx[iv + 1, iu + 1]])
            self.hole_w.append([(1 - fu) * (1 - fv), fu * (1 - fv), (1 - fu) * fv, fu * fv])
        self.hole_nodes = np.array(self.hole_nodes)
        self.hole_w = np.array(self.hole_w)
        self.hole_uv = np.c_[HOLE_U, np.full(3, HOLE_V)]
        self._build_cutout_render_mesh()
        self._build_print_lines()

        self.x = np.zeros((n, 3))
        self.v = np.zeros((n, 3))
        self.pins: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        self.open_t = [0.0, 0.0]
        self.time = 0.0
        self.boxes_fixed = []
        self.table_z = 0.0
        self.prev_boxes = None
        self.bad = False

    def _color(self):
        col = -np.ones(len(self.edges), int)
        used = [set() for _ in range(self.n)]
        for i, (a, b) in enumerate(self.edges):
            c = 0
            while c in used[a] or c in used[b]:
                c += 1
            col[i] = c
            used[a].add(c)
            used[b].add(c)
        return [np.flatnonzero(col == c) for c in range(col.max() + 1)]

    def _build_cutout_render_mesh(self):
        """Build render vertices & faces with real 8-sided cut-out holes through the 3 punched quads."""
        idx = self.idx
        hole_map = {q: hi for hi, q in enumerate(self.hole_quads)}
        extra_nodes, extra_w = [], []
        rfaces = []
        for v in range(NV - 1):
            for u in range(NU - 1):
                c00, c10, c11, c01 = idx[v, u], idx[v, u + 1], idx[v + 1, u + 1], idx[v + 1, u]
                if (v, u) not in hole_map:
                    rfaces += [(c00, c10, c11), (c00, c11, c01)]
                    continue
                hi = hole_map[(v, u)]
                hu, hv = HOLE_U[hi], HOLE_V
                base = self.n + len(extra_nodes)
                for k in range(HOLE_SIDES):
                    ang = 2 * math.pi * k / HOLE_SIDES - 3 * math.pi / 4  # k=0 faces (-u, -v) -> corner c00
                    pu = hu + HOLE_R * math.cos(ang)
                    pv = hv + HOLE_R * math.sin(ang)
                    fu = np.clip(pu / self.hu - u, 0.02, 0.98)
                    fv = np.clip(pv / self.hv - v, 0.02, 0.98)
                    extra_nodes.append([c00, c10, c01, c11])
                    extra_w.append([(1 - fu) * (1 - fv), fu * (1 - fv), (1 - fu) * fv, fu * fv])
                # Connect 4 quad corners to the 8-gon ring (2 octants per side, counter-clockwise)
                corners = [c00, c10, c11, c01]
                # Ring angles: k=0 (-135 deg, near c00), k=1 (-90), k=2 (-45, near c10), k=3 (0),
                # k=4 (+45, near c11), k=5 (+90), k=6 (+135, near c01), k=7 (180)
                for side_i in range(4):
                    ca = corners[side_i]
                    cb = corners[(side_i + 1) % 4]
                    k0 = (2 * side_i) % HOLE_SIDES
                    k1 = (2 * side_i + 1) % HOLE_SIDES
                    k2 = (2 * side_i + 2) % HOLE_SIDES
                    rfaces.append((ca, base + k1, base + k0))
                    rfaces.append((ca, cb, base + k1))
                    rfaces.append((cb, base + k2, base + k1))
        self.extra_nodes = np.array(extra_nodes, np.int32)
        self.extra_w = np.array(extra_w, float)
        self.render_faces = np.array(rfaces, np.int32)

    def _build_print_lines(self):
        """Printed handout header + text bars in (u, v) coords so the page stack is immediately recognizable."""
        # Bars as (u0, u1, v0, v1) on the page face (away from the v=0..18mm punched margin)
        bars = [
            (0.032, 0.195, 0.030, 0.042),   # title bar
            (0.032, 0.245, 0.056, 0.062),   # text line 1
            (0.032, 0.238, 0.074, 0.080),   # text line 2
            (0.032, 0.210, 0.092, 0.098),   # text line 3
            (0.032, 0.245, 0.118, 0.124),   # text line 4
            (0.032, 0.232, 0.136, 0.142),   # text line 5
            (0.032, 0.180, 0.154, 0.160),   # text line 6
        ]
        pnodes, pw, pfaces = [], [], []
        for u0, u1, v0, v1 in bars:
            # Split wide bars along u into segments so they curve smoothly with the sheet
            us = np.linspace(u0, u1, 9)
            base = len(pnodes)
            for uu in us:
                for vv in (v0, v1):
                    iu = min(NU - 2, max(0, int(uu // self.hu)))
                    iv = min(NV - 2, max(0, int(vv // self.hv)))
                    fu, fv = uu / self.hu - iu, vv / self.hv - iv
                    pnodes.append([self.idx[iv, iu], self.idx[iv, iu + 1], self.idx[iv + 1, iu], self.idx[iv + 1, iu + 1]])
                    pw.append([(1 - fu) * (1 - fv), fu * (1 - fv), (1 - fu) * fv, fu * fv])
            for j in range(len(us) - 1):
                a, b, c, dd = base + 2 * j, base + 2 * j + 2, base + 2 * j + 3, base + 2 * j + 1
                pfaces += [(a, b, c), (a, c, dd)]
        self.print_nodes = np.array(pnodes, np.int32)
        self.print_w = np.array(pw, float)
        self.print_faces = np.array(pfaces, np.int32)

    def render_points(self) -> np.ndarray:
        extra = np.einsum("ek,ekd->ed", self.extra_w, self.x[self.extra_nodes])
        return np.vstack([self.x, extra])

    def print_points(self) -> np.ndarray:
        nrm = self.normals()
        pts = np.einsum("pk,pkd->pd", self.print_w, self.x[self.print_nodes])
        pn = np.einsum("pk,pkd->pd", self.print_w, nrm[self.print_nodes])
        pn /= np.maximum(np.linalg.norm(pn, axis=1, keepdims=True), 1e-12)
        # Ensure normal points generally +z when sheet is upright/flat
        sgn = np.where(pn[:, 2:3] >= 0, 1.0, -1.0)
        return pts + sgn * pn * (THICK / 2 + 0.0005)

    def flat(self, edge_mid, z, along_u=(0.0, 1.0), along_v=(-1.0, 0.0)):
        """Flat sheet: punched-edge midpoint at ``edge_mid`` (xy), height ``z``."""
        au = np.r_[np.asarray(along_u, float), 0.0]
        av = np.r_[np.asarray(along_v, float), 0.0]
        o = np.r_[np.asarray(edge_mid, float)[:2], z] - au * (W_U / 2)
        return o + self.uv[:, :1] * au + self.uv[:, 1:] * av

    def place(self, x, table_z, boxes_fixed):
        self.x = np.array(x, float)
        self.v[:] = 0.0
        self.pins = {}
        self.table_z = float(table_z)
        self.boxes_fixed = [(np.asarray(lo, float), np.asarray(hi, float)) for lo, hi in boxes_fixed]
        self.time = 0.0
        self.open_t = [0.0, 0.0]
        self.prev_boxes = None
        self.bad = False

    def holes(self) -> np.ndarray:
        return np.einsum("hk,hkd->hd", self.hole_w, self.x[self.hole_nodes])

    def normals(self) -> np.ndarray:
        f = self.faces
        fn = np.cross(self.x[f[:, 1]] - self.x[f[:, 0]], self.x[f[:, 2]] - self.x[f[:, 0]])
        nn = np.zeros((self.n, 3))
        for i in range(3):
            np.add.at(nn, f[:, i], fn)
        return nn / np.maximum(np.linalg.norm(nn, axis=1, keepdims=True), 1e-12)

    # -- contact-triggered edge pinches ---------------------------------------------------------------------------
    def update_pins(self, tcp, rot, aperture):
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
            local = (self.x - tcp[arm]) @ rot[arm] - PIN_C
            inside = np.all(np.abs(local) < PIN_BOX, axis=1)
            for other in self.pins.values():
                inside[other[0]] = False
            if (inside & self.border).any():
                nodes = np.flatnonzero(inside)
                self.pins[arm] = (nodes, (self.x[nodes] - tcp[arm]) @ rot[arm])

    # -- collision & friction -------------------------------------------------------------------------------------
    def _box_push(self, idx_mask, lo, hi, world=None, mu=MU_SURF, mv=None):
        x = self.x
        l = x if world is None else (x - world[:3, 3]) @ world[:3, :3]
        t = THICK / 2
        ins = np.all((l > lo - t) & (l < hi + t), axis=1) & idx_mask
        if not ins.any():
            return
        i = np.flatnonzero(ins)
        aa, bb = l[i] - (lo - t), (hi + t) - l[i]
        mm = np.minimum(aa, bb)
        ax = np.argmin(mm, axis=1)
        r = np.arange(len(i))
        depth = mm[r, ax]
        sgn = np.where(aa[r, ax] < bb[r, ax], -1.0, 1.0)
        l_out = l[i].copy()
        l_out[r, ax] = np.where(sgn < 0, lo[ax] - t, hi[ax] + t)
        nrm = np.zeros((len(i), 3))
        nrm[r, ax] = sgn
        if world is not None:
            l_out = l_out @ world[:3, :3].T + world[:3, 3]
            nrm = nrm @ world[:3, :3].T
        x[i] = l_out
        self._contact(i, depth, nrm, mu, None if mv is None else mv[i] if mv.ndim == 2 else mv)

    def _contact(self, idx, depth, nrm, mu, mv=None):
        self._c.append((np.asarray(idx), np.asarray(depth, float), np.asarray(nrm, float), mu,
                        np.zeros((len(idx), 3)) if mv is None else np.broadcast_to(mv, (len(idx), 3))))

    def _friction(self, prev, free):
        for idx, dn, n, mu, mv in self._c:
            k = free[idx]
            idx, dn, n, mv = idx[k], dn[k], n[k], mv[k]
            if not len(idx):
                continue
            dx = (self.x[idx] - prev[idx]) - mv
            t = dx - (dx * n).sum(1, keepdims=True) * n
            tl = np.linalg.norm(t, axis=1)
            lim = mu * dn
            f = np.where(tl <= lim, 1.0, lim / np.maximum(tl, 1e-12))
            self.x[idx] -= t * f[:, None]

    def collide(self, free, boxes, prevb, rings):
        x = self.x
        tz = self.table_z + THICK / 2
        low = (x[:, 2] < tz) & free
        if low.any():
            i = np.flatnonzero(low)
            self._contact(i, tz - x[i, 2], np.tile([0, 0, 1.0], (len(i), 1)), MU_SURF)
            x[i, 2] = tz
        for lo, hi in self.boxes_fixed:
            self._box_push(free, lo, hi)
        for bi, (world, lo, hi) in enumerate(boxes):
            mv = None
            if prevb is not None:
                l = (x - world[:3, 3]) @ world[:3, :3]
                mv = x - (l @ prevb[bi][:3, :3].T + prevb[bi][:3, 3])
            self._box_push(free, lo, hi, world, MU_PAD, mv)
        if rings is not None:
            rings.push_sheet(self, free)

    def step(self, tcp, rot, aperture, boxes=(), rings=None, dt=0.05, substeps=10, iters=6):
        tcp = np.asarray(tcp, float)
        rot = np.asarray(rot, float)
        self.time += dt
        self.update_pins(tcp, rot, aperture)
        h = dt / substeps
        a_edge = self.compl / (h * h)
        E, R = self.edges, self.rest
        w = self.inv_m.copy()
        pinned = np.zeros(self.n, bool)
        targets = []
        for arm, (nodes, off) in self.pins.items():
            tgt = tcp[arm] + off @ rot[arm].T
            targets.append((nodes, self.x[nodes].copy(), tgt))
            pinned[nodes] = True
            w[nodes] = 0.0
        self._w = w
        free = ~pinned
        cur = [b[0] for b in boxes]
        prevb = self.prev_boxes if (self.prev_boxes is not None and len(self.prev_boxes) == len(cur)) else None

        for s in range(substeps):
            fr = (s + 1) / substeps
            if prevb is None:
                bx = list(boxes)
                pw = None
            else:
                bx, pw = [], []
                for bi, (world, lo, hi) in enumerate(boxes):
                    wd = world.copy()
                    wd[:3, 3] = prevb[bi][:3, 3] + fr * (world[:3, 3] - prevb[bi][:3, 3])
                    bx.append((wd, lo, hi))
                    wd0 = world.copy()
                    wd0[:3, 3] = prevb[bi][:3, 3] + (fr - 1 / substeps) * (world[:3, 3] - prevb[bi][:3, 3])
                    pw.append(wd0)
            prev = self.x.copy()
            self.v *= math.exp(-AIR_DRAG * h)
            self.v[:, 2] -= 9.81 * h
            self.x += self.v * h
            for nodes, start, tgt in targets:
                self.x[nodes] = start + fr * (tgt - start)
            lam = np.zeros(len(E))
            for it in range(iters):
                for c in self.colors:
                    a, b = E[c, 0], E[c, 1]
                    d = self.x[b] - self.x[a]
                    ln = np.linalg.norm(d, axis=1)
                    C = ln - R[c]
                    ws = w[a] + w[b]
                    dl = (-C - a_edge[c] * lam[c]) / np.maximum(ws + a_edge[c], 1e-12)
                    dl = np.where(self.kind[c] == 3, np.minimum(lam[c] + dl, 0.0) - lam[c], dl)
                    lam[c] += dl
                    g = d / np.maximum(ln, 1e-12)[:, None]
                    self.x[a] -= (w[a] * dl)[:, None] * g
                    self.x[b] += (w[b] * dl)[:, None] * g
                for tri, comp in self.tri_groups:
                    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
                    C = self.x[a] - 2 * self.x[b] + self.x[c]
                    den = w[a] + 4 * w[b] + w[c] + comp / (h * h)
                    dl = -C / np.maximum(den, 1e-12)[:, None]
                    self.x[a] += w[a][:, None] * dl
                    self.x[b] -= 2 * w[b][:, None] * dl
                    self.x[c] += w[c][:, None] * dl
                if rings is not None:
                    rings.hole_constraints(self, w)
                    if it >= iters - 2:
                        rings.tip_constraints(self, w)
                self._c = []
                self.collide(free, bx, pw, rings)
            self._friction(prev, free)
            self.v = (self.x - prev) / h
            sp = np.linalg.norm(self.v, axis=1)
            self.v *= np.minimum(1.0, 2.0 / np.maximum(sp, 1e-9))[:, None]
        self.prev_boxes = cur
        if not np.isfinite(self.x).all():
            self.bad = True
            self.x = np.nan_to_num(self.x)


class Rings:
    """Over-centre 3-ring metal along world y at x = ring_x, rings at ring_y, with end booster tabs at plate_y0/y1."""

    def __init__(self, ring_x, ring_y, plate_top, plate_y_ends=(-0.290, -0.020)):
        self.rx = float(ring_x)
        self.ry = np.asarray(ring_y, float)
        self.pt = float(plate_top)
        self.py0, self.py1 = float(plate_y_ends[0]), float(plate_y_ends[1])
        self.zc = self.pt + 0.75 * RC
        self.hinge_z = self.pt - 0.002
        self.theta = math.radians(OPEN_DEG)
        self.closed = False
        self.s_min = self._s_min()
        self.thread: list[tuple[int, int, float] | None] = [None, None, None]
        self.side: dict[tuple[int, int], int | None] = {}
        self.torn = False
        self.tear_why = ""
        self.tear_t = 0.0
        self.max_off = 0.0
        self.moving = False
        self.log: list = []

    def _s_min(self) -> float:
        c = (self.pt - self.zc) / RC
        return float(math.acos(max(-1.0, min(1.0, -c)))) if abs(c) < 1 else 0.0

    def arc(self, ring: int, half: int, s, theta=None) -> np.ndarray:
        """World point(s) on half-ring (+1 far / -1 near) at arc parameter s in [s_min, pi] (pi = free tip)."""
        th = self.theta if theta is None else theta
        s = np.asarray(s, float)
        px = half * RC * np.sin(s)
        pz = self.zc - RC * np.cos(s) - self.hinge_z
        ph = half * th
        x = px * math.cos(ph) + pz * math.sin(ph)
        z = -px * math.sin(ph) + pz * math.cos(ph)
        return np.stack([self.rx + x, np.full_like(x, self.ry[ring]), self.hinge_z + z], -1)

    def tangent(self, ring: int, half: int, s, theta=None) -> np.ndarray:
        a = self.arc(ring, half, s + 1e-4, theta)
        b = self.arc(ring, half, s - 1e-4, theta)
        d = a - b
        return d / np.maximum(np.linalg.norm(d, axis=-1, keepdims=True), 1e-12)

    def points(self, theta=None) -> dict[tuple[int, int], np.ndarray]:
        s = np.linspace(self.s_min, math.pi, N_ARC)
        return {(r, h): self.arc(r, h, s, theta) for r in range(3) for h in (1, -1)}

    def booster_segments(self, theta=None) -> list[tuple[np.ndarray, np.ndarray]]:
        """Two end booster tabs at py0 (-y end) and py1 (+y end): tilt outward when open, upright when shut."""
        th = self.theta if theta is None else theta
        frac = th / max(math.radians(OPEN_DEG), 1e-6)
        ang = math.radians(15.0 + 38.0 * frac)  # 53 deg outward from vertical when open, 15 deg when shut
        segs = []
        for sgn, y0 in ((-1.0, self.py0), (1.0, self.py1)):
            base = np.array([self.rx, y0, self.pt])
            tip = base + np.array([0.0, sgn * BOOSTER_LEN * math.sin(ang), BOOSTER_LEN * math.cos(ang)])
            segs.append((base, tip))
        return segs

    def _closest_s(self, ring: int, half: int, p: np.ndarray):
        s = np.linspace(self.s_min, math.pi, 40)
        q = self.arc(ring, half, s)
        k = int(np.argmin(np.linalg.norm(q - p, axis=1)))
        lo, hi = s[max(k - 1, 0)], s[min(k + 1, len(s) - 1)]
        s2 = np.linspace(lo, hi, 12)
        q2 = self.arc(ring, half, s2)
        k2 = int(np.argmin(np.linalg.norm(q2 - p, axis=1)))
        return float(s2[k2]), q2[k2]

    def hole_constraints(self, sheet: Sheet, w: np.ndarray):
        P = sheet.holes()
        slack = HOLE_R - WIRE_R
        for i, th in enumerate(self.thread):
            if th is None:
                continue
            r, h, _ = th
            s, q = self._closest_s(r, h, P[i])
            if not self.closed and s >= math.pi - 1e-3:
                tip = self.arc(r, h, math.pi)
                t = self.tangent(r, h, math.pi)
                if float((P[i] - tip) @ t) > slack:
                    self.thread[i] = None
                    self.log.append(("off", i, r, h))
                    self.side[(r, h)] = None
                    continue
            # When closed, the hole can slide freely across the top join (s = pi) from one half to the other
            if self.closed and s >= math.pi - 1e-3:
                s_other, q_other = self._closest_s(r, -h, P[i])
                if np.linalg.norm(q_other - P[i]) < np.linalg.norm(q - P[i]):
                    h, s, q = -h, s_other, q_other
            self.thread[i] = (r, h, s)
            d = q - P[i]
            dl = float(np.linalg.norm(d))
            if dl <= slack:
                continue
            delta = d * (1 - slack / dl)
            nodes = sheet.hole_nodes[i]
            wt = sheet.hole_w[i] * (w[nodes] > 0)
            ssum = float((wt ** 2).sum())
            if ssum < 1e-9:
                continue
            sheet.x[nodes] += (wt / ssum)[:, None] * delta

    def push_sheet(self, sheet: Sheet, free: np.ndarray):
        x = sheet.x
        threaded_nodes = set()
        for i, th in enumerate(self.thread):
            if th is not None:
                threaded_nodes.update(sheet.hole_nodes[i].tolist())
        mask = free.copy()
        if threaded_nodes:
            mask[list(threaded_nodes)] = False
        near = mask & (np.abs(x[:, 0] - self.rx) < 0.04) & (x[:, 2] < sheet.table_z + self.zc + RC + 0.02)
        if not near.any():
            return
        idx = np.flatnonzero(near)
        rad = WIRE_R + THICK / 2
        for (r, h), q in self.points().items():
            yy = np.abs(x[idx, 1] - self.ry[r]) < 0.010
            if not yy.any():
                continue
            j = idx[yy]
            a, b = q[:-1], q[1:]
            ab = b - a
            pa = x[j][:, None, :] - a[None]
            t = np.clip((pa * ab[None]).sum(-1) / np.maximum((ab ** 2).sum(-1), 1e-12)[None], 0, 1)
            cp = a[None] + t[..., None] * ab[None]
            dv = x[j][:, None, :] - cp
            dd = np.linalg.norm(dv, axis=-1)
            k = np.argmin(dd, axis=1)
            dmin = dd[np.arange(len(j)), k]
            hit = dmin < rad
            if hit.any():
                jj = j[hit]
                nv = dv[np.flatnonzero(hit), k[hit]] / np.maximum(dmin[hit], 1e-9)[:, None]
                x[jj] = cp[np.flatnonzero(hit), k[hit]] + nv * rad
                sheet._contact(jj, rad - dmin[hit], nv, MU_SURF)

    def _tip_side(self, sheet: Sheet, p: np.ndarray):
        f = sheet.faces
        a, b, c = sheet.x[f[:, 0]], sheet.x[f[:, 1]], sheet.x[f[:, 2]]
        n = np.cross(b - a, c - a)
        nl = np.linalg.norm(n, axis=1)
        n = n / np.maximum(nl, 1e-12)[:, None]
        d = ((p - a) * n).sum(1)
        q = p - d[:, None] * n
        v0, v1, v2 = b - a, c - a, q - a
        d00 = (v0 * v0).sum(1)
        d01 = (v0 * v1).sum(1)
        d11 = (v1 * v1).sum(1)
        d20 = (v2 * v0).sum(1)
        d21 = (v2 * v1).sum(1)
        den = np.maximum(d00 * d11 - d01 * d01, 1e-18)
        bv = (d11 * d20 - d01 * d21) / den
        bw = (d00 * d21 - d01 * d20) / den
        bu = 1 - bv - bw
        e = -0.02
        ok = (bu > e) & (bv > e) & (bw > e)
        if not ok.any():
            return None
        k = np.flatnonzero(ok)[np.argmin(np.abs(d[ok]))]
        uv = bu[k] * sheet.uv[f[k, 0]] + bv[k] * sheet.uv[f[k, 1]] + bw[k] * sheet.uv[f[k, 2]]
        return float(d[k]), uv, k, np.array([bu[k], bv[k], bw[k]]), n[k]

    def tip_constraints(self, sheet: Sheet, w: np.ndarray):
        """An open ring's rounded tip crosses the sheet only within CAPTURE_R of a hole centre; else blocks."""
        if self.closed:
            return
        busy_ring = {t[0] for t in self.thread if t is not None}
        P = sheet.holes()
        for r in range(3):
            if r in busy_ring:
                continue
            for h in (1, -1):
                key = (r, h)
                tip = self.arc(r, h, math.pi)
                # Direct 3D proximity check for hole capture when hole passes right over the bevelled tip
                for i in range(3):
                    if self.thread[i] is not None:
                        continue
                    dp = P[i] - tip
                    tang = self.tangent(r, h, math.pi)
                    along = float(dp @ tang)
                    perp = float(np.linalg.norm(dp - along * tang))
                    if perp <= CAPTURE_R and -0.006 <= along <= 0.0025:
                        self.thread[i] = (r, h, math.pi - 0.03)
                        self.log.append(("on", i, r, h))
                        busy_ring.add(r)
                        break
                if r in busy_ring:
                    break
                res = self._tip_side(sheet, tip)
                if res is None:
                    continue
                d, uv, k, bary, nrm = res
                old = self.side.get(key)
                if old is None:
                    if abs(d) > 2 * THICK:
                        self.side[key] = 1 if d >= 0 else -1
                    continue
                if abs(d) > 0.022 or old * d >= THICK / 2:
                    continue
                dist = np.linalg.norm(sheet.hole_uv - uv, axis=1)
                i = int(np.argmin(dist))
                if dist[i] <= CAPTURE_R and self.thread[i] is None and old * d < THICK:
                    self.thread[i] = (r, h, math.pi - 0.03)
                    self.log.append(("on", i, r, h))
                    busy_ring.add(r)
                    break
                corr = (d - old * THICK / 2) * nrm
                nodes = sheet.faces[k]
                wt = bary * (w[nodes] > 0)
                ssum = float((wt ** 2).sum())
                if ssum > 1e-9:
                    sheet.x[nodes] += (wt / ssum)[:, None] * corr[None, :]

    def update(self, pad_boxes, sheet: Sheet | None = None, dt: float = 0.05):
        """Over-centre closure: requires EITHER one gripper's two opposing finger pads pinching BOTH halves of a ring
        inward at once, OR a finger pad pushing one of the end booster tabs inward (>= 8 mm) toward the rings."""
        self.moving = False
        if self.closed:
            return
        th = self.theta
        target_th = None

        # 1. Opposing pinch on any ring r by the two finger pads (side 0 vs side 1) of the SAME arm
        for r in range(3):
            q_near = self.arc(r, -1, np.linspace(self.s_min + 0.3, math.pi, 10), th)
            q_far = self.arc(r, +1, np.linspace(self.s_min + 0.3, math.pi, 10), th)
            for arm_idx in (0, 1):
                near_hit, far_hit = [], []
                for item in pad_boxes:
                    a_i, s_i, world, lo, hi = item
                    if a_i != arm_idx:
                        continue
                    bc = world[:3, 3] + world[:3, :3] @ (0.5 * (lo + hi))
                    if abs(bc[1] - self.ry[r]) > 0.022:
                        continue
                    ln = (q_near - world[:3, 3]) @ world[:3, :3]
                    lf = (q_far - world[:3, 3]) @ world[:3, :3]
                    if bc[0] < self.rx and np.any(np.all((ln > lo - WIRE_R) & (ln < hi + WIRE_R), axis=1)):
                        near_hit.append((s_i, world, lo, hi))
                    if bc[0] > self.rx and np.any(np.all((lf > lo - WIRE_R) & (lf < hi + WIRE_R), axis=1)):
                        far_hit.append((s_i, world, lo, hi))
                if near_hit and far_hit and any(sn != sf for sn, *_ in near_hit for sf, *_ in far_hit):
                    for t in np.linspace(th, 0.0, 50)[1:]:
                        qn = self.arc(r, -1, np.linspace(self.s_min + 0.3, math.pi, 10), t)
                        qf = self.arc(r, +1, np.linspace(self.s_min + 0.3, math.pi, 10), t)
                        still_n = any(np.any(np.all(((qn - w[:3, 3]) @ w[:3, :3] > lo - WIRE_R) &
                                                    ((qn - w[:3, 3]) @ w[:3, :3] < hi + WIRE_R), axis=1))
                                      for _, w, lo, hi in near_hit)
                        still_f = any(np.any(np.all(((qf - w[:3, 3]) @ w[:3, :3] > lo - WIRE_R) &
                                                    ((qf - w[:3, 3]) @ w[:3, :3] < hi + WIRE_R), axis=1))
                                      for _, w, lo, hi in far_hit)
                        if not (still_n and still_f):
                            target_th = t if target_th is None else min(target_th, t)
                            break
                    else:
                        target_th = 0.0

        # 2. End booster tabs pushed inward toward the ring bar (coupled to opening angle th)
        for sgn, (base0, tip0) in zip((-1.0, 1.0), self.booster_segments(th)):
            hit_boxes = []
            pts_open = base0 + np.linspace(0.45, 1.0, 6)[:, None] * (tip0 - base0)
            for _, _, world, lo, hi in pad_boxes:
                bc = world[:3, 3] + world[:3, :3] @ (0.5 * (lo + hi))
                if abs(bc[0] - self.rx) > 0.020:
                    continue
                if sgn < 0 and bc[1] > base0[1] - 0.002:
                    continue
                if sgn > 0 and bc[1] < base0[1] + 0.002:
                    continue
                lt = (pts_open - world[:3, 3]) @ world[:3, :3]
                if np.any(np.all((lt > lo - 0.002) & (lt < hi + 0.002), axis=1)):
                    hit_boxes.append((world, lo, hi))
            if hit_boxes:
                idx_end = 0 if sgn < 0 else 1
                for t in np.linspace(th, 0.0, 50)[1:]:
                    b_t, p_t = self.booster_segments(t)[idx_end]
                    pts_t = b_t + np.linspace(0.45, 1.0, 6)[:, None] * (p_t - b_t)
                    still = any(np.any(np.all(((pts_t - w[:3, 3]) @ w[:3, :3] > lo - 0.002) &
                                              ((pts_t - w[:3, 3]) @ w[:3, :3] < hi + 0.002), axis=1))
                                for w, lo, hi in hit_boxes)
                    if not still:
                        target_th = t if target_th is None else min(target_th, t)
                        break
                else:
                    target_th = 0.0

        if target_th is not None:
            th = float(target_th)
        elif th < math.radians(OPEN_DEG):
            th = min(math.radians(OPEN_DEG), th + math.radians(45.0) * dt)

        if th < math.radians(SNAP_DEG):
            th = 0.0
            self.closed = True
            self.log.append(("shut",))
            if sheet is not None:
                self._check_pinch_unthreaded(sheet)

        self.moving = abs(th - self.theta) > 1e-6
        self.theta = th

    def _check_pinch_unthreaded(self, sheet: Sheet):
        """If the rings snap shut while paper sits across a ring whose hole is NOT threaded on it, the closing jaws
        pinch/tear the sheet."""
        on = self.on_rings()
        x = sheet.x
        for r in range(3):
            if on[r]:
                continue
            # Any sheet surface crossing this ring's closed wire circle?
            in_ring = ((np.abs(x[:, 0] - self.rx) < RC + 0.008) &
                       (np.abs(x[:, 1] - self.ry[r]) < 0.012) &
                       (x[:, 2] < self.zc + RC + 0.010))
            if in_ring.any():
                self.torn = True
                self.tear_why = f"ring {r} closed onto unthreaded paper"
                self.log.append(("pinched", r))
                return
            tip = self.arc(r, 1, math.pi, theta=0.0)
            res = self._tip_side(sheet, tip)
            if res is not None and abs(res[0]) < 0.018:
                self.torn = True
                self.tear_why = f"ring {r} tips snapped onto unthreaded paper"
                self.log.append(("pinched", r))
                return

    def track_tear(self, sheet: Sheet, dt: float = 0.05):
        P = sheet.holes()
        off = 0.0
        for i, th in enumerate(self.thread):
            if th is None:
                continue
            r, h, s = th
            off = max(off, float(np.linalg.norm(self.arc(r, h, s) - P[i])) - (HOLE_R - WIRE_R))
        self.max_off = max(self.max_off, off)
        self.tear_t = self.tear_t + dt if off > TEAR_D else 0.0
        if self.tear_t > TEAR_TIME and not self.torn:
            self.torn = True
            self.tear_why = "threaded hole pulled past wire tear threshold"

    def on_rings(self) -> list[bool]:
        """Per-ring threaded flags: True for ring r if a distinct hole sits on ring r (supports either cover side)."""
        engaged = {t[0] for t in self.thread if t is not None}
        return [r in engaged for r in range(3)]
