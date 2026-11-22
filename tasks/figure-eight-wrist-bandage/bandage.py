"""Elastic crepe bandage on a fixed forearm-and-hand form, as a reduced position-based strip (NumPy only).

Every step receives only measured robot state: both TCP poses (position, rotation) and jaw openings.
  * Strip: 7.5 cm x 1.66 m relaxed, 3 width columns at 2.0 cm spacing (84 stations; 20.0 cm reset marks).
  * Coreless roll: 4.0 cm dia x 7.5 cm cylinder wound onto its 4.0 cm inner hook patch, unwinding past PAYOUT_RATIO.
  * Grasps: roll and strip grasps engage only on simultaneous two-pad contact while jaws actively close.
  * Fastening: all 3 inner-patch stations must contact laid wrap under >= 1.5 mm inward pad compression for 0.25 s.
"""
from __future__ import annotations

import numpy as np

TOP = 0.7475
AX_Z = TOP + 0.16
ELBOW_X, WRIST_X, TIP_X = 0.05, 0.29, 0.47
FOREARM = [
    (np.array([ELBOW_X, 0.0, AX_Z]), np.array([0.13, 0.0, AX_Z]), 0.033),
    (np.array([0.13, 0.0, AX_Z]), np.array([0.21, 0.0, AX_Z]), 0.031),
    (np.array([0.21, 0.0, AX_Z]), np.array([WRIST_X, 0.0, AX_Z]), 0.0285),
]
# rounded boxes (true box centre, half_extents, corner_radius): continuous hand x=0.295..0.470 m (5 mm overlap)
PALM = (np.array([0.3475, 0.0, AX_Z - 0.003]), np.array([0.0525, 0.0425, 0.014]), 0.010)     # x=0.295..0.400 m
FINGERS = (np.array([0.4325, 0.0, AX_Z - 0.004]), np.array([0.0375, 0.038, 0.009]), 0.008)   # x=0.395..0.470 m
THUMB = (np.array([0.300, 0.036, AX_Z - 0.008]), np.array([0.322, 0.095, AX_Z - 0.022]), 0.0105)
THUMB_BARE_FRAC = 0.40
PARTS = [("cap", a, b, r) for a, b, r in FOREARM] + [("box", *PALM), ("box", *FINGERS), ("cap", *THUMB)]

WIDTH, NW, HW = 0.075, 3, 0.0375
LENGTH, H, PATCH_LEN = 1.66, 0.02, 0.04
NS = int(round(LENGTH / H)) + 1   # 84 stations (0..83); reset length marks every 20.0 cm
THICK, CORE_R, PLY = 0.0012, 0.007, 0.00074
PATCH_N, FREE_END = 3, 10
ROLL_D = round(2.0 * float(np.sqrt(CORE_R ** 2 + (LENGTH - FREE_END * H) * PLY / np.pi)), 3)
ROLL_R = ROLL_D / 2.0
ROLL0 = np.array([0.24, 0.13, TOP + ROLL_R])
MASS_PER_M, ROLL_MASS, BAND_K, WRENCH_K = 0.020, 0.035, 10.0, 6.0

PAYOUT_RATIO, TAUT_RATIO = 1.42, 1.05
MIN_TOTAL_REVS, MAX_TOTAL_REVS = 6.0, 9.0
CUFF_W_X, CUFF_W_R = (0.200, 0.280), 0.0295
CUFF_H_X, CUFF_H_R = (0.310, 0.395), 0.0333
PRESSURE_MIN_MMHG, PRESSURE_MAX_MMHG, PA_PER_MMHG = 12.0, 55.0, 133.322
GRIP_PAD_TOL, GRIP_COMP_MAX, GRIP_RELEASE = 0.0035, 0.006, 0.008
PINCH_CLOSE, PINCH_OPEN, PINCH_SLIP = 0.014, 0.024, 0.040
PRESS_S, PATCH_CONTACT_TOL, PATCH_PRESS_MIN = 0.25, 0.0020, 0.0015
X_W, X_H, TOP_HALF, DIAG_DX, UNDER_DPHI = 0.300, 0.335, np.pi / 2, 0.03, 1.2


def cap_sdf(p, a, b, r):
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0.0, 1.0)
    d = p - (a + t[:, None] * ab)
    n = np.linalg.norm(d, axis=1)
    return n - r, d / np.maximum(n, 1e-9)[:, None]


def rbox_sdf(p, c, half, rr):
    q = p - c
    s = np.where(q == 0, 1.0, np.sign(q))
    a = np.abs(q) - (half - rr)
    out = np.maximum(a, 0.0)
    no = np.linalg.norm(out, axis=1)
    d = no + np.minimum(a.max(axis=1), 0.0) - rr
    g = np.where(no[:, None] > 1e-9, out / np.maximum(no, 1e-9)[:, None], 0.0)
    ins = no <= 1e-9
    if ins.any():
        k = np.argmax(a[ins], axis=1)
        gi = np.zeros((int(ins.sum()), 3))
        gi[np.arange(len(k)), k] = 1.0
        g[ins] = gi
    return d, g * s


def arm_sdf(p, skip_thumb=False):
    p = np.atleast_2d(np.asarray(p, float))
    best_d = best_n = None
    for k, part in enumerate(PARTS):
        if skip_thumb and k == len(PARTS) - 1:
            continue
        d, n = cap_sdf(p, *part[1:]) if part[0] == "cap" else rbox_sdf(p, *part[1:])
        if best_d is None:
            best_d, best_n = d, n
        else:
            m = d < best_d
            best_d = np.where(m, d, best_d)
            best_n = np.where(m[:, None], n, best_n)
    return best_d, best_n


def thumb_dist(p):
    a, b, r = THUMB
    return cap_sdf(np.atleast_2d(p), a + THUMB_BARE_FRAC * (b - a), b, r)[0]


def roll_axis_of(R):
    return R[:, 0]


# -- scoring geometry & metrology helpers -------------------------------------------------------------------------
def runs(b):
    idx = np.flatnonzero(b.laid[:b.paid])
    if len(idx) < 2:
        return []
    m = b.mid(idx)
    th = np.arctan2(m[:, 1], m[:, 2] - AX_Z)
    phi, top = np.unwrap(th), np.abs(th) < TOP_HALF
    raw, s = [], 0
    for k in range(1, len(idx) + 1):
        if k == len(idx) or top[k] != top[s] or idx[k] > idx[k - 1] + 5:
            raw.append((s, k, bool(top[s])))
            s = k
    out = []
    for s, k, is_top in raw:
        x0, x1, xm = float(m[s, 0]), float(m[k - 1, 0]), float(m[s:k, 0].mean())
        s_ext, k_ext = max(0, s - 1), min(len(idx), k + 1)
        dphi_ext, pts = float(phi[k_ext - 1] - phi[s_ext]), m[s_ext:k_ext, :2].copy()
        if is_top:
            kind = ("up" if (pts[-1, 0] - pts[0, 0] >= DIAG_DX or x1 - x0 >= DIAG_DX)
                    else ("down" if (pts[0, 0] - pts[-1, 0] >= DIAG_DX or x0 - x1 >= DIAG_DX)
                          else ("Wt" if xm < X_W else ("Ht" if xm > X_H else "Mt"))))
        else:
            kind = "dip" if abs(dphi_ext) < UNDER_DPHI else ("Wb" if xm < X_W else ("Hb" if xm > X_H else "Mb"))
        out.append(dict(kind=kind, i0=int(idx[s]), i1=int(idx[k - 1]), x0=x0, x1=x1, xm=xm, dphi_ext=dphi_ext, pts=pts))
    return out


def _cross(p, q):
    for a0, a1 in zip(p[:-1], p[1:]):
        for b0, b1 in zip(q[:-1], q[1:]):
            d1, d2 = a1 - a0, b1 - b0
            den = d1[0] * d2[1] - d1[1] * d2[0]
            if abs(den) < 1e-12:
                continue
            w = b0 - a0
            s, t = (w[0] * d2[1] - w[1] * d2[0]) / den, (w[0] * d1[1] - w[1] * d1[0]) / den
            if 0 <= s <= 1 and 0 <= t <= 1:
                return True
    return False


def pattern(b):
    idx_all = np.flatnonzero(b.laid[:b.paid])
    total_revs = 0.0
    if len(idx_all) >= 4:
        phi_all = np.unwrap(np.arctan2(b.mid(idx_all)[:, 1], b.mid(idx_all)[:, 2] - AX_Z))
        total_revs = round(abs(float(phi_all[-1] - phi_all[0])) / (2 * np.pi), 2)
    rs = [r for r in runs(b) if r["kind"] != "dip"]
    out = {"wrist_turns": 0.0, "loops": 0, "total_revs": total_revs, "seq": [r["kind"] for r in rs]}
    if not rs:
        return out
    first_diag = next((k for k, r in enumerate(rs) if r["kind"] in ("up", "down")), len(rs))
    pre = rs[:first_diag]
    if pre:
        i_start, i_end = pre[0]["i0"], pre[-1]["i1"]
        w_idx = [i for i in (np.flatnonzero(b.laid[i_start:i_end + 1]) + i_start) if b.mid(i)[0] < X_W]
        if len(w_idx) >= 3:
            phi = np.unwrap(np.arctan2(b.mid(w_idx)[:, 1], b.mid(w_idx)[:, 2] - AX_Z))
            out["wrist_turns"] = round(abs(float(phi[-1] - phi[0])) / (2 * np.pi), 2)
    loops = k = 0
    while k + 3 < len(rs):
        a, bb, c, d = rs[k:k + 4]
        if (a["kind"] == "up" and bb["kind"] == "Hb" and c["kind"] == "down" and d["kind"] == "Wb"
                and a["dphi_ext"] * bb["dphi_ext"] > 0 and bb["dphi_ext"] * c["dphi_ext"] > 0
                and c["dphi_ext"] * d["dphi_ext"] > 0 and _cross(a["pts"], c["pts"])):
            loops += 1
            k += 4
        else:
            k += 1
    out["loops"] = loops
    return out


def bare_thumb_and_tips(b):
    pts = b.x[:b.paid].reshape(-1, 3)
    on_thumb = bool((thumb_dist(pts) < 0.005).any())
    far = pts[:, 0] > TIP_X - 0.03
    on_tips = bool((arm_sdf(pts[far])[0] < 0.010).any()) if far.any() else False
    return (not on_thumb), (not on_tips)


def loose(b):
    n = b.paid
    upto = n if b.fastened else max(0, n - PATCH_N)
    if upto == 0:
        return 0
    d = arm_sdf(b.x[:upto].reshape(-1, 3))[0].reshape(upto, NW) - b._layer_off(n)[:upto]
    return int((d.max(axis=1) > 0.010).sum())


def cuff_pressures_mmhg(b):
    """Laplace sub-bandage radial pressure (mmHg) on the flush wrist (x=20..28 cm) and palm (x=31..39.5 cm) bladders."""
    upto = min(b.paid, NS - PATCH_N)
    if upto < 2 or not b.laid[:upto].any():
        return 0.0, 0.0
    m = b.mid()
    fw = fh = 0.0
    for i in np.flatnonzero(b.laid[:upto]):
        i = int(i)
        if i > 0 and b.laid[i - 1]:
            ds = float(np.clip(np.linalg.norm(m[i] - m[i - 1]), 0.5 * H, 3.0 * H))
        elif i + 1 < upto and b.laid[i + 1]:
            ds = float(np.clip(np.linalg.norm(m[i + 1] - m[i]), 0.5 * H, 3.0 * H))
        else:
            ds = H
        tens = BAND_K * max(0.0, float(b.lay_strain[i]) - 1.0)
        xs = b.x[i, :, 0]
        w_frac = float(((xs >= CUFF_W_X[0]) & (xs <= CUFF_W_X[1])).mean())
        h_frac = float(((xs >= CUFF_H_X[0]) & (xs <= CUFF_H_X[1])).mean())
        fw += w_frac * tens * ds / CUFF_W_R
        fh += h_frac * tens * ds / CUFF_H_R
    pw = fw / (2.0 * np.pi * CUFF_W_R * (CUFF_W_X[1] - CUFF_W_X[0]) * PA_PER_MMHG)
    ph = fh / (2.0 * np.pi * CUFF_H_R * (CUFF_H_X[1] - CUFF_H_X[0]) * PA_PER_MMHG)
    return round(float(pw), 1), round(float(ph), 1)


# -- bandage state, contact grasps, pressure-latched patch & PBD strip -------------------------------------------
class Bandage:
    def __init__(self):
        self.n, self.paid = NS, FREE_END + 1
        self.x = np.zeros((NS, NW, 3))
        self.v = np.zeros((NS, NW, 3))
        self.laid = np.zeros(NS, bool)
        self.lay_strain = np.ones(NS)
        self.lay_order = np.full(NS, -1)
        self._lay_count = 0
        self.pins = {}
        self.roll_c = ROLL0.copy()
        self.roll_R = np.eye(3)
        self.roll_v = np.zeros(3)
        self.roll_holders = []
        self.prev_ap = np.full(2, 0.09)
        self.passes = self.under_passes = 0
        self._pass_from = None
        self._pass_under = self._went_under_limb = False
        self.fastened = self.bad = False
        self.press_s = self.press_depth = 0.0
        self.patch_holder = None
        self.unheld_steps = 0
        self.stretch_ratio = 1.0
        self._init_layout()

    def _init_layout(self):
        self.roll_R = np.eye(3)
        r = self.roll_r()
        self.roll_c = np.array([ROLL0[0], ROLL0[1], TOP + r])
        for i in range(self.n):
            yy = (self.roll_c[1] - (self.paid - 1 - i) * H - 0.012) if i < self.paid else self.roll_c[1]
            z = (TOP + THICK) if i < self.paid else self.roll_c[2]
            for c, sx in enumerate((-1.0, 0.0, 1.0)):
                self.x[i, c] = [self.roll_c[0] + sx * HW, yy, z]
        self.v[:] = 0

    def mid(self, idx=None):
        return self.x[:, 1, :] if idx is None else self.x[idx, 1, :]

    def roll_left(self):
        return max(0.0, LENGTH - (self.paid - 1) * H)

    def roll_r(self):
        return float(np.sqrt(CORE_R ** 2 + self.roll_left() * PLY / np.pi))

    def roll_empty(self):
        return self.paid >= self.n

    def _aligned_axis(self, ref_idx):
        ax = roll_axis_of(self.roll_R)
        return -ax if float(ax @ (self.x[ref_idx, 2] - self.x[ref_idx, 0])) < 0.0 else ax

    def anchor(self):
        ax = self._aligned_axis(max(0, self.paid - 1))
        d = self.x[self.paid - 1, 1] - self.roll_c
        d = d - (d @ ax) * ax
        nd = np.linalg.norm(d)
        d = d / nd if nd > 1e-6 else -self.roll_R[:, 1]
        base = self.roll_c + self.roll_r() * d
        return np.array([base - ax * HW, base, base + ax * HW])

    def _two_pad_roll_contact(self, tcp, rot, ap, prev_ap):
        """Engage only when jaws actively close AND both finger pads contact opposite sides of the roll."""
        if ap >= prev_ap - 1e-4:
            return None
        loc = rot.T @ (self.roll_c - tcp)
        ax = roll_axis_of(self.roll_R)
        ax_loc = rot.T @ ax
        r = self.roll_r()
        if abs(ax_loc[0]) > 0.88:
            if np.hypot(loc[1], loc[2]) > max(r + 0.004, 0.018):
                return None
            half_span, kind, size = 0.5 * WIDTH, "faces", WIDTH
        elif abs(ax_loc[0]) < 0.38:
            along = abs(float(ax @ (self.roll_c - tcp)))
            perp = (self.roll_c - tcp) - float(ax @ (self.roll_c - tcp)) * ax
            if along > HW - 0.008 or np.linalg.norm(perp) > 0.024:
                return None
            half_span, kind, size = r, "body", 2.0 * r
        else:
            return None
        gap_pos, gap_neg = (0.5 * ap) - (loc[0] + half_span), (loc[0] - half_span) - (-0.5 * ap)
        if (-GRIP_COMP_MAX <= gap_pos <= GRIP_PAD_TOL and -GRIP_COMP_MAX <= gap_neg <= GRIP_PAD_TOL
                and size - GRIP_COMP_MAX <= ap <= size + GRIP_PAD_TOL):
            return kind
        return None

    def update_grasps(self, tcp, rot, ap):
        keep = [(arm, rp, rR, sz) for arm, rp, rR, sz in self.roll_holders if ap[arm] <= sz + GRIP_RELEASE]
        if len(keep) < len(self.roll_holders) and keep and self._pass_from is not None:
            if self._pass_from not in [h[0] for h in keep]:
                self.passes += 1
                if self._pass_under and self.roll_c[2] < AX_Z:
                    self.under_passes += 1
                    self._went_under_limb = False
            self._pass_from, self._pass_under = None, False
        self.roll_holders = keep
        holders = [h[0] for h in self.roll_holders]
        if not self.roll_empty():
            for arm in (0, 1):
                if arm in holders or arm in self.pins:
                    continue
                kind = self._two_pad_roll_contact(tcp[arm], rot[arm], ap[arm], self.prev_ap[arm])
                if kind is not None:
                    rp, rR = rot[arm].T @ (self.roll_c - tcp[arm]), rot[arm].T @ self.roll_R
                    self.roll_holders.append((arm, rp, rR, WIDTH if kind == "faces" else 2.0 * self.roll_r()))
                    if len(self.roll_holders) > 1:
                        self._pass_from = self.roll_holders[0][0]
                        self._pass_under = bool(self._went_under_limb and self.roll_c[2] < AX_Z)
        for arm in (0, 1):
            if arm in self.pins:
                slipped = any(np.linalg.norm(self.x[i, c] - (tcp[arm] + rot[arm] @ off)) > PINCH_SLIP
                              for i, c, off in self.pins[arm])
                if ap[arm] > PINCH_OPEN or slipped:
                    del self.pins[arm]
                    if self.laid.any() and self.roll_holders:
                        i_first = int(np.flatnonzero(self.laid[:self.paid])[0])
                        loff = self._layer_off(self.paid)
                        for i in range(i_first - 1, -1, -1):
                            if not self.laid[i] and float((arm_sdf(self.x[i])[0] - loff[i]).min()) < 0.009:
                                self._lay(i)
                continue
            if ap[arm] > PINCH_CLOSE or ap[arm] >= self.prev_ap[arm] - 1e-4 or arm in [h[0] for h in self.roll_holders]:
                continue
            cand = [(i, c, (rot[arm].T @ (self.x[i, c] - tcp[arm])).copy())
                    for i in range(self.paid) if not self.laid[i] for c in range(NW)
                    if abs((loc := rot[arm].T @ (self.x[i, c] - tcp[arm]))[0]) <= 0.5 * ap[arm] + 0.008
                    and abs(loc[1]) < 0.020 and abs(loc[2]) < 0.024]
            if cand:
                self.pins[arm] = cand
        self.prev_ap = np.array(ap, float)

    def _fasten(self, dt):
        """Latch hook patch only when all 3 patch stations contact laid wrap under >= 1.5 mm inward pad compression."""
        if self.fastened or not self.roll_empty() or self.patch_holder is None:
            if not self.fastened:
                self.press_s = 0.0
            return
        n = self.paid
        L = np.flatnonzero(self.laid[:n - PATCH_N])
        if not len(L):
            self.press_s = 0.0
            return
        patch = self.x[n - PATCH_N:n].reshape(-1, 3)
        la, lb, lc = self.x[L, 0], self.x[L, 1], self.x[L, 2]
        surf = np.concatenate([la, 0.5 * (la + lb), lb, 0.5 * (lb + lc), lc], axis=0)
        dd = np.linalg.norm(patch[:, None, :] - surf[None, :, :], axis=2).min(axis=1)
        da, _ = arm_sdf(patch)
        loff_p = self._layer_off(n)[n - PATCH_N:n].reshape(-1)
        in_contact = bool((dd <= 0.016).all() and ((da - (THICK + loff_p)) <= PATCH_CONTACT_TOL).all())
        self.press_s = (self.press_s + dt) if (in_contact and self.press_depth >= PATCH_PRESS_MIN) else 0.0
        if self.press_s >= PRESS_S:
            self.fastened = True
            for i in range(n - PATCH_N, n):
                self._lay(i)

    def _move_roll(self, tcp, rot, dt):
        if self.roll_holders:
            arm, rp, rR, _ = self.roll_holders[0]
            c = tcp[arm] + rot[arm] @ rp
            self.roll_v = (c - self.roll_c) / dt
            self.roll_c, self.roll_R = c, rot[arm] @ rR
            return
        self.roll_v[2] -= 9.81 * dt
        self.roll_v *= 0.98
        c, rr = self.roll_c + self.roll_v * dt, self.roll_r()
        if c[2] < TOP + rr:
            c[2], self.roll_v[:] = TOP + rr, 0
        d, nrm = arm_sdf(c[None])
        if d[0] < rr:
            c = c + (rr - d[0]) * nrm[0]
            self.roll_v = 0.5 * (self.roll_v - (self.roll_v @ nrm[0]) * nrm[0])
        self.roll_c = c
        ax = np.array([self.roll_R[0, 0], self.roll_R[1, 0], 0.0])
        ax = ax / np.linalg.norm(ax) if np.linalg.norm(ax) >= 1e-6 else np.array([1.0, 0.0, 0.0])
        self.roll_R = np.column_stack([ax, np.cross([0.0, 0.0, 1.0], ax), [0.0, 0.0, 1.0]])

    def _free_span_ratio(self):
        n = self.paid
        laid = np.flatnonzero(self.laid[:n])
        pinned = [i for held in self.pins.values() for i, _, _ in held]
        i0 = int(laid[-1]) if len(laid) else (min(pinned) if pinned else 0)
        pts = (np.concatenate([self.x[i0:n], self.anchor()[None, :, :]], axis=0) if not self.roll_empty()
               else (self.x[i0:n] if n - 1 > i0 else None))
        if pts is None or len(pts) < 2:
            return 1.0
        return float(np.linalg.norm(np.diff(pts, axis=0), axis=2).sum(axis=0).max() / ((len(pts) - 1) * H))

    def _payout(self, tcp, rot, ap):
        unwound = False
        clamp = max((max(0.0, sz - float(ap[arm]) - 0.0035) for arm, _, _, sz in self.roll_holders), default=0.0)
        payout_lim = PAYOUT_RATIO + min(1.05, 130.0 * clamp)
        for _ in range(2):
            if self.roll_empty():
                break
            self.stretch_ratio = self._free_span_ratio()
            if self.stretch_ratio <= payout_lim:
                break
            n = self.paid
            self.x[n], self.v[n] = 0.75 * self.anchor() + 0.25 * self.x[n - 1], 0
            self.paid += 1
            unwound = True
            if self.roll_empty() and self.patch_holder is None and self.roll_holders:
                self._transfer_to_patch(tcp, rot, ap)
        if unwound:
            n = self.paid
            L = np.flatnonzero(self.laid[:n])
            i0 = int(L[-1]) if len(L) else 0
            if n - i0 >= 2:
                xs, ws = self.x[i0:n], np.ones((n - i0, NW))
                ws[0] = 0.0
                an = self.anchor() if not self.roll_empty() else None
                for _ in range(4):
                    for col in range(NW):
                        self._dist(xs[:-1, col], xs[1:, col], ws[:-1, col], ws[1:, col], H, True, 0.55)
                    if an is not None:
                        for col in range(NW):
                            d = an[col] - xs[-1, col]
                            ln = np.linalg.norm(d)
                            if ln > H and ws[-1, col] > 0:
                                xs[-1, col] += d / ln * (ln - H) * 0.55
        self.stretch_ratio = self._free_span_ratio()

    def _transfer_to_patch(self, tcp, rot, ap):
        arm, _, _, _ = self.roll_holders[0]
        ref = max(0, self.n - 1 - PATCH_N)
        ax = self._aligned_axis(ref)
        raw_t = (rot[arm][:, 2] - float(rot[arm][:, 2] @ ax) * ax
                 if abs(float(rot[arm][:, 0] @ ax)) > 0.7 else np.cross(rot[arm][:, 2], ax))
        nt = float(np.linalg.norm(raw_t))
        tang = raw_t / nt if nt > 1e-6 else rot[arm][:, 1]
        fwd_hint = (tcp[arm] + 0.018 * rot[arm][:, 2]) - self.x[ref, 1]
        if float(tang @ fwd_hint) < 0.0:
            tang = -tang
        offs = []
        for idx_s, t_off in zip(range(self.n - PATCH_N, self.n), (-0.018, 0.0, 0.018)):
            c_pt = tcp[arm] + 0.018 * rot[arm][:, 2] + t_off * tang
            for col, sx in enumerate((-1.0, 0.0, 1.0)):
                pt = c_pt + sx * HW * ax
                self.x[idx_s, col], self.v[idx_s, col] = pt, 0.0
                offs.append((idx_s, col, rot[arm].T @ (pt - tcp[arm])))
        self.patch_holder, self.roll_holders = [arm, float(ap[arm]), offs], []

    def step(self, tcp, rot, ap, dt=0.05, substeps=8, iters=6):
        tcp, rot, ap = np.asarray(tcp, float), np.asarray(rot, float), np.asarray(ap, float)
        self.update_grasps(tcp, rot, ap)
        self._move_roll(tcp, rot, dt)
        if not self.roll_holders:
            self._went_under_limb = False
        elif not self.roll_empty() and 0.16 <= self.roll_c[0] <= TIP_X and abs(self.roll_c[1]) <= 0.06 and self.roll_c[2] <= AX_Z - 0.03:
            self._went_under_limb = True
        if self.roll_empty():
            if self.patch_holder is None and self.roll_holders:
                self._transfer_to_patch(tcp, rot, ap)
            elif self.patch_holder is not None:
                arm, ap_hold, _ = self.patch_holder
                if ap[arm] < ap_hold:
                    self.patch_holder[1] = float(ap[arm])
                elif ap[arm] > max(ap_hold + GRIP_RELEASE, PINCH_OPEN):
                    self.patch_holder = None
        if self.laid.any() and not self.fastened:
            held = bool(self.roll_holders) or (self.roll_empty() and self.patch_holder is not None) or bool(self.pins)
            if not held:
                self.unheld_steps += 1
                L_idx = np.flatnonzero(self.laid[:self.paid])
                if len(L_idx):
                    ip = int(L_idx[-1])
                    self.laid[ip], self.lay_order[ip], self.lay_strain[ip] = False, -1, 1.0
                    self.x[ip] += 0.014 * arm_sdf(self.x[ip])[1]
            else:
                self.unheld_steps = 0

        h, n = dt / substeps, self.paid
        x, v = self.x[:n], self.v[:n]
        w = np.ones((n, NW))
        w[self.laid[:n]] = 0.0
        pin_targets = []
        for arm, held in self.pins.items():
            for i, col, off in held:
                if i < n and not self.laid[i]:
                    pin_targets.append((i, col, tcp[arm] + rot[arm] @ off))
                    w[i, col] = 0.0
        loff = self._layer_off(n)
        th_flat = (THICK + loff).reshape(-1)
        patch_targets = []
        self.press_depth = 0.0
        if self.patch_holder is not None and not self.fastened:
            arm, _, offs = self.patch_holder
            raw_pts = [(i, col, tcp[arm] + rot[arm] @ off) for i, col, off in offs if i < n]
            if raw_pts:
                da_t, nrm_t = arm_sdf(np.array([pt for _, _, pt in raw_pts]))
                surf_th = np.array([THICK + loff[i, col] for i, col, _ in raw_pts])
                self.press_depth = float(np.maximum(0.0, surf_th - da_t).mean())
                for idx_p, (i, col, pt) in enumerate(raw_pts):
                    if da_t[idx_p] < surf_th[idx_p]:
                        pt = pt + (surf_th[idx_p] - da_t[idx_p]) * nrm_t[idx_p]
                    patch_targets.append((i, col, pt))
                    w[i, col] = 0.0
        if self.fastened:
            w[n - PATCH_N:] = 0.0

        targets, tie, diag = pin_targets + patch_targets, not self.roll_empty(), np.hypot(H, HW)
        for _ in range(substeps):
            prev = x.copy()
            v *= 0.98
            v[..., 2] -= 9.81 * h
            x += v * h * (w[..., None] > 0)
            for i, col, t in targets:
                x[i, col] = t
            an = self.anchor() if tie else None
            for _it in range(iters):
                for col in range(NW):
                    self._dist(x[:-1, col], x[1:, col], w[:-1, col], w[1:, col], H, True, 0.42)
                self._dist(x[:, 0], x[:, 1], w[:, 0], w[:, 1], HW, False, 0.95)
                self._dist(x[:, 1], x[:, 2], w[:, 1], w[:, 2], HW, False, 0.95)
                for c0, c1 in ((0, 1), (1, 0), (1, 2), (2, 1)):
                    self._dist(x[:-1, c0], x[1:, c1], w[:-1, c0], w[1:, c1], diag, False, 0.25)
                if tie:
                    for col in range(NW):
                        d = an[col] - x[n - 1, col]
                        ln = np.linalg.norm(d)
                        if ln > H and w[n - 1, col] > 0:
                            x[n - 1, col] += d / ln * (ln - H) * 0.55
                self._collide(x, w, th_flat)
            dx = x - prev
            d, nrm = arm_sdf(x.reshape(-1, 3))
            con = (d < th_flat + THICK).reshape(n, NW) & (w > 0)
            if con.any():
                nrm_c = nrm.reshape(n, NW, 3)[con]
                x[con] -= (dx[con] - (dx[con] * nrm_c).sum(-1, keepdims=True) * nrm_c) * 0.85
            v[:] = (x - prev) / h
            sp = np.linalg.norm(v, axis=-1, keepdims=True)
            v *= np.minimum(1.0, 3.0 / np.maximum(sp, 1e-9))
            for i, col, t in targets:
                x[i, col] = t
        self.stretch_ratio = self._free_span_ratio()
        self._tack()
        self._payout(tcp, rot, ap)
        self._tack()
        self._fasten(dt)
        if not np.isfinite(self.x).all():
            self.bad, self.x = True, np.nan_to_num(self.x)

    @staticmethod
    def _dist(a, b, wa, wb, rest, tension_only, stiff=1.0):
        d = b - a
        ln = np.linalg.norm(d, axis=-1)
        C = np.maximum(ln - rest, 0.0) if tension_only else (ln - rest)
        ws = wa + wb
        k = np.where(ws > 0, stiff * C / np.maximum(ws, 1e-12), 0.0)
        g = d / np.maximum(ln, 1e-12)[..., None]
        a += (wa * k)[..., None] * g
        b -= (wb * k)[..., None] * g

    def _layer_off(self, n):
        L = np.flatnonzero(self.laid[:n])
        if not len(L):
            return np.zeros((n, NW))
        dd = np.linalg.norm(self.x[:n].reshape(-1, 3)[:, None, :] - self.x[L].reshape(-1, 3)[None, :, :], axis=2)
        earlier = np.repeat(L, NW)[None, :] < (np.repeat(np.arange(n), NW)[:, None] - 4)
        return (np.minimum(((dd < 0.020) & earlier).sum(axis=1) / float(NW), 4.0) * THICK).reshape(n, NW)

    @staticmethod
    def _collide(x, w, th_flat):
        n, flat, free = len(x), x.reshape(-1, 3), w.reshape(-1) > 0
        d, nrm = arm_sdf(flat)
        m = (d < th_flat) & free
        if m.any():
            flat[m] += (th_flat[m] - d[m])[:, None] * nrm[m]
        low = (flat[:, 2] < TOP + THICK) & free
        flat[low, 2] = TOP + THICK
        x[:] = flat.reshape(n, NW, 3)

    def taut(self):
        return bool(self.roll_holders or self.patch_holder is not None) and self.stretch_ratio >= TAUT_RATIO

    def _tack(self):
        n = self.paid
        pinned = {i for held in self.pins.values() for i, _, _ in held}
        loff = self._layer_off(n)
        d = arm_sdf(self.x[:n].reshape(-1, 3))[0].reshape(n, NW) - loff
        touch = (d[:, 1] < THICK * 2.8) | ((d < THICK * 2.8).sum(axis=1) >= 2)
        last_ok = n - 1 if not self.roll_empty() else max(0, n - PATCH_N)
        if self.taut():
            if not self.laid.any():
                for i in range(last_ok):
                    if i not in pinned and touch[i]:
                        self._lay(i)
                        break
            if self.laid.any():
                progressed = True
                while progressed:
                    progressed = False
                    L = np.flatnonzero(self.laid[:last_ok])
                    if not len(L):
                        break
                    i_hi = int(L[-1])
                    for cand in (i_hi + 1, i_hi + 2):
                        if cand < last_ok and not self.laid[cand] and cand not in pinned and touch[cand]:
                            if cand == i_hi + 2 and (i_hi + 1) not in pinned and d[i_hi + 1].min() < 0.006:
                                self._lay(i_hi + 1)
                                self._lay(cand)
                                progressed = True
                                break
                            elif cand == i_hi + 1:
                                self._lay(cand)
                                progressed = True
                                break
                    i_lo = int(L[0])
                    if i_lo - 1 >= 0 and not self.laid[i_lo - 1] and (i_lo - 1) not in pinned and touch[i_lo - 1]:
                        self._lay(i_lo - 1)
                        progressed = True
        if self.laid.any() and (self.roll_holders or self.patch_holder is not None or self.fastened):
            L = np.flatnonzero(self.laid[:n])
            lm, lown = self.x[L].reshape(-1, 3), np.repeat(L, NW)
            for i in range(last_ok):
                if self.laid[i] or i in pinned or d[i].min() > 0.022:
                    continue
                later = lown > i + 3
                if later.any() and np.linalg.norm(self.x[i][:, None, :] - lm[None, later, :], axis=2).min() < 0.024:
                    self._lay(i)

    def _lay(self, i):
        loff = self._layer_off(self.paid)[i]
        for _ in range(3):
            d, nrm = arm_sdf(self.x[i])
            self.x[i] -= np.clip(d - (THICK + loff), -0.002, 0.020)[:, None] * nrm
            for col in (0, 2):
                dv = self.x[i, col] - self.x[i, 1]
                ln = float(np.linalg.norm(dv))
                if ln > 1e-6:
                    self.x[i, col] = self.x[i, 1] + dv * (HW / ln)
        d, nrm = arm_sdf(self.x[i])
        self.x[i] -= np.clip(d - (THICK + loff), 0.0, 0.022)[:, None] * nrm
        d, nrm = arm_sdf(self.x[i])
        pen = d < (THICK + loff)
        if pen.any():
            self.x[i, pen] += ((THICK + loff)[pen] - d[pen])[:, None] * nrm[pen]
        self.laid[i], self.lay_order[i], self.v[i] = True, self._lay_count, 0
        self.lay_strain[i] = 1.0 if i >= self.n - PATCH_N else max(TAUT_RATIO, float(self.stretch_ratio))
        self._lay_count += 1

    def held_by(self):
        return set(h[0] for h in self.roll_holders) | set(self.pins) | ({self.patch_holder[0]} if self.patch_holder else set())

    def gripper_load(self):
        f, pos = np.zeros((2, 3)), np.zeros((2, 3))
        n = self.paid
        laid, pinned = np.flatnonzero(self.laid[:n]), [i for held in self.pins.values() for i, _, _ in held]
        i0 = int(laid[-1]) if len(laid) else (min(pinned) if pinned else -1)
        pull_mag = min(6.0, WRENCH_K * max(0.0, self.stretch_ratio - 1.0)) if i0 >= 0 else 0.0
        anchor_pt = self.x[i0, 1] if i0 >= 0 else self.roll_c
        if self.roll_holders and not self.roll_empty():
            w_roll = np.array([0.0, 0.0, -9.81 * (0.005 + MASS_PER_M * self.roll_left())])
            d = anchor_pt - self.roll_c
            nd = float(np.linalg.norm(d))
            t_vec = (pull_mag * d / nd) if nd > 1e-6 else np.zeros(3)
            for arm, _, _, _ in self.roll_holders:
                f[arm] += (w_roll + t_vec) / len(self.roll_holders)
                pos[arm] = self.roll_c
        elif self.patch_holder is not None and not self.fastened:
            arm, p_end = self.patch_holder[0], self.x[self.n - 1, 1]
            d = anchor_pt - p_end
            nd = float(np.linalg.norm(d))
            if nd > 1e-6:
                f[arm] += pull_mag * d / nd
            pos[arm] = p_end
        return f, pos
