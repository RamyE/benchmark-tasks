"""Sewing machine feed and stitch model for the fabric sheet (no Isaac imports; also runs offline).

The machine runs at a fixed speed: the feed dogs draw the fabric under the presser foot away from the operator
(+x) at FEED_V, and the needle makes one stitch every STITCH_DT. Under the foot the fabric cannot slide sideways;
it turns about the needle when the fabric outside the foot pulls it round (a torque above a small static
threshold), and it slips along the feed (longer or shorter stitches) when it is pulled or held back hard.

Model of the foot patch: the fabric nodes within FOOT_R of the needle are moved rigidly each control step
(rotation about the needle + feed), and the in-plane tension in the sheet edges that join them to the rest of the
fabric gives the torque and the pull along the feed for the next step. Stitches are recorded where the needle
enters the fabric, in flat-fabric coordinates.
"""
from __future__ import annotations

import math

import numpy as np

# -- fabric (flat coordinates u along x, v along y at the start, metres from the rounded corner's square corner) ----
FAB_L, FAB_H = 0.24, 0.20  # 24 x 20 cm
FAB_RC = 0.06  # the near right corner is rounded to a 6 cm radius
SA = 0.015  # seam allowance, 15 mm
CELL = 0.008
START_UV = (FAB_L - 0.03, SA)  # chalk start mark: 3 cm in from the far end of the straight right edge
END_UV = (SA, FAB_RC + 0.05)  # chalk end mark: 5 cm past the curve on the near edge

# -- machine --------------------------------------------------------------------------------------------------------
NEEDLE_XY = (0.33, -0.05)
FEED_V = 0.010  # m/s: about 240 stitches a minute at 2.5 mm
STITCH_DT = 0.25  # s per stitch -> 2.5 mm stitches
FOOT_R = 0.015  # fabric clamped between foot and feed dogs within 15 mm of the needle
TAU0 = 0.0005  # static torque threshold (strain-units x m)
G_ROT = 300.0  # rad/s per unit of torque above the threshold
W_MAX = 1.2  # rad/s cap on turning
F0 = 0.25  # pull along the feed tolerated before the fabric slips over the feed dogs (strain units)
G_SLIP = 0.02  # m/s of slip per unit pull above F0
SLIP_MAX = 0.02


def corner_center():
    return np.array([FAB_RC, FAB_RC])


def allowance(uv) -> np.ndarray:
    """Distance (m) of flat-fabric points from the sewn curved edge (right edge, rounded corner, near edge)."""
    uv = np.atleast_2d(np.asarray(uv, float))
    u, v = uv[:, 0], uv[:, 1]
    d = np.minimum(u, v)
    inc = (u < FAB_RC) & (v < FAB_RC)
    dc = FAB_RC - np.hypot(FAB_RC - u, FAB_RC - v)
    return np.where(inc, dc, d)


def on_fabric(u, v) -> bool:
    if u < FAB_RC - 1e-9 and v < FAB_RC - 1e-9:
        return math.hypot(FAB_RC - u, FAB_RC - v) <= FAB_RC + 1e-9
    return True


# ideal seam path (for progress along the seam), from the start mark to the end mark
_R = FAB_RC - SA
SEG1 = START_UV[0] - FAB_RC  # straight run along the right edge
SEG2 = 0.5 * math.pi * _R  # the curve
SEG3 = END_UV[1] - FAB_RC  # straight run along the near edge
PATH_LEN = SEG1 + SEG2 + SEG3


def path_point(s: float) -> tuple[np.ndarray, np.ndarray]:
    """Point and unit tangent (direction of sewing) of the ideal 15 mm seam at arc length s."""
    if s <= SEG1:
        return np.array([START_UV[0] - s, SA]), np.array([-1.0, 0.0])
    if s <= SEG1 + SEG2:
        a = -0.5 * math.pi - (s - SEG1) / _R  # from straight down (-90 deg) round to 180 deg
        c = corner_center()
        p = c + _R * np.array([math.cos(a), math.sin(a)])
        t = np.array([math.sin(a), -math.cos(a)])
        return p, t
    s3 = s - SEG1 - SEG2
    return np.array([SA, FAB_RC + s3]), np.array([0.0, 1.0])


def seam_s(uv) -> float:
    """Signed arc-length position (m) along the extended 15 mm seam line (0 at start mark, PATH_LEN at end mark)."""
    u, v = float(uv[0]), float(uv[1])
    u1 = max(u, FAB_RC)
    d1, s1 = math.hypot(u - u1, v - SA), START_UV[0] - u1
    ang = math.atan2(v - FAB_RC, u - FAB_RC)
    if ang > 0.25 * math.pi:
        ang -= 2 * math.pi
    ac = min(max(ang, -math.pi), -0.5 * math.pi)
    d2 = math.hypot(u - (FAB_RC + _R * math.cos(ac)), v - (FAB_RC + _R * math.sin(ac)))
    s2 = SEG1 + (-0.5 * math.pi - ac) * _R
    v3 = max(v, FAB_RC)
    d3, s3 = math.hypot(u - SA, v - v3), SEG1 + SEG2 + (v3 - FAB_RC)
    return float(min((d1, s1), (d2, s2), (d3, s3))[1])


def path_s(uv) -> float:
    """Arc length along the ideal seam of the point nearest to uv."""
    return float(np.clip(seam_s(uv), 0.0, PATH_LEN))


def _rot2(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


class Machine:
    """Feed, needle and stitch record. step() is called once per control step with the sheet, before the sheet
    moves; after() with the sheet after it moved (to read the tension on the foot patch)."""

    def __init__(self, S, uv):
        self.S = S
        self.uv = uv  # flat coordinates of every sheet node
        self.N = np.array(NEEDLE_XY, float)
        self.running = False
        self.t_run = 0.0
        self.next_stitch = STITCH_DT * 0.5
        self.stitches = []  # (u, v) where the needle went in
        self.stitch_adv = []  # fabric advance (m) over each stitch
        self.off_fabric = False  # the needle came down with no fabric under it
        self._adv = 0.0
        self.omega = 0.0
        self.slip = 0.0
        self.zone = np.zeros(0, int)
        self.max_w = 0.0
        self.tau = 0.0
        self.fx = 0.0
        # structural (stretch) constraints only
        m = S.cpl == 2
        self.ca, self.cb, self.rest = S.ca[m], S.cb[m], S.crest0[m]

    def set_running(self, on: bool):
        self.running = bool(on)

    def _zone(self):
        x = self.S.x
        d = np.hypot(x[:, 0] - self.N[0], x[:, 1] - self.N[1])
        return np.nonzero(d < FOOT_R)[0]

    def targets(self, dt):
        """Prescribed positions of the foot-patch nodes for this control step (the feed + turn), or None."""
        self.zone = self._zone()
        if len(self.zone) == 0:
            return None
        x = self.S.x
        if not self.running:
            # stopped with the needle down: the patch stays put (it can still be turned about the needle)
            adv = 0.0
        else:
            adv = FEED_V * dt + self.slip * dt
        th = self.omega * dt
        R = _rot2(th)
        out = {}
        for i in self.zone:
            p = x[i]
            q = R @ (p[:2] - self.N) + self.N
            q[0] += adv
            out[int(i)] = np.array([q[0], q[1], self.S.table_z + 0.0006])
        if self.running:
            self._adv += adv
            self.t_run += dt
        return out

    def after(self, dt):
        """Read the tension on the foot patch: torque about the needle -> turning rate; pull along x -> slip."""
        x = self.S.x
        inz = np.zeros(len(x), bool)
        inz[self.zone] = True
        a, b = self.ca, self.cb
        sel = inz[a] ^ inz[b]
        if not sel.any():
            self.omega = 0.0
            self.slip = 0.0
            return
        za = np.where(inz[a[sel]], a[sel], b[sel])
        fr = np.where(inz[a[sel]], b[sel], a[sel])
        d = x[fr, :2] - x[za, :2]
        ln = np.linalg.norm(d, axis=1) + 1e-12
        rest = self.rest[sel]
        ten = np.maximum(ln - rest, 0.0) / rest  # tension only: fabric doesn't push
        f = d / ln[:, None] * ten[:, None]
        r = x[za, :2] - self.N
        tau = float(np.sum(r[:, 0] * f[:, 1] - r[:, 1] * f[:, 0]))
        fx = float(np.sum(f[:, 0]))
        self.tau, self.fx = tau, fx
        if abs(tau) > TAU0:
            self.omega = float(np.clip(G_ROT * (abs(tau) - TAU0) * np.sign(tau), -W_MAX, W_MAX))
        else:
            self.omega = 0.0
        self.max_w = max(self.max_w, abs(self.omega))
        if self.running and abs(fx) > F0:
            self.slip = float(np.clip(G_SLIP * (abs(fx) - F0) * np.sign(fx), -SLIP_MAX, SLIP_MAX))
        else:
            self.slip = 0.0
        if self.running and self.t_run >= self.next_stitch:
            self.next_stitch += STITCH_DT
            uv = self.needle_uv()
            if uv is not None:
                self.stitches.append(uv)
                self.stitch_adv.append(self._adv)
            else:
                self.off_fabric = True
            self._adv = 0.0

    def needle_uv(self):
        """Flat-fabric coordinates of the point under the needle (barycentric in the sheet's triangles), or None
        when no fabric is under it."""
        S = self.S
        x = S.x
        f = S.faces
        P = self.N
        A, B, C = x[f[:, 0], :2], x[f[:, 1], :2], x[f[:, 2], :2]
        v0, v1, v2 = B - A, C - A, P - A
        den = v0[:, 0] * v1[:, 1] - v1[:, 0] * v0[:, 1]
        ok = np.abs(den) > 1e-12
        den = np.where(ok, den, 1.0)
        l1 = (v2[:, 0] * v1[:, 1] - v1[:, 0] * v2[:, 1]) / den
        l2 = (v0[:, 0] * v2[:, 1] - v2[:, 0] * v0[:, 1]) / den
        l0 = 1 - l1 - l2
        inside = ok & (l0 >= -1e-6) & (l1 >= -1e-6) & (l2 >= -1e-6)
        k = np.nonzero(inside)[0]
        if len(k) == 0:
            return None
        k = k[0]
        uv = self.uv
        return tuple(l0[k] * uv[f[k, 0]] + l1[k] * uv[f[k, 1]] + l2[k] * uv[f[k, 2]])
