"""Reduced XPBD multi-strand rooted hair lock with three sequential entwined crossings and comb-tooth interaction.

Pure NumPy (no simulator imports, no task-phase flags). Driven every 20 Hz control step solely by measured simulator
state: both gripper TCP poses, orientations and jaw apertures, gripper collision boxes, and the rigid wide-tooth
comb's world pose.

Physical model:
- 6 synthetic Kanekalon strand bundles (S0..S5, 38 cm long, 20 nodes at 2.0 cm spacing, 25 mm wide at the scalp slot,
  8.0 g total lock mass) exit the right-temple scalp eyelet at (0.420, y_i, TOP + 0.120) and drape over the rounded
  right shoulder onto the white chest towel (top at TOP + 0.055).
- Three sequential entwined crossings along the matted zone (10-22 cm from the tips):
    C1 (upper)  at s = 16 cm (node 8, 22 cm from tips)
    C2 (middle) at s = 22 cm (node 11, 16 cm from tips)
    C3 (distal) at s = 28 cm (node 14, 10 cm from tips)
- Closing a free gripper's jaws on the lock pinches the strand nodes between its finger pads until the jaws reopen.
- Drawing the comb's 10 rounded teeth down-lock through an active crossing whose downstream hair is clear resolves that
  crossing over 5.0 cm of comb travel while exerting 3.2 N combing drag on the strands at the crossing:
    * With the other gripper pinching the lock >= 2.0 cm nearer the scalp than that crossing, the holding hand absorbs
      the drag and root tension stays below 0.5 N (well under the 2.0 N scalp limit).
    * Without an above-knot hold, the combing pull transmits along the taut strands to the scalp weft, exceeding the
      2.0 N root-pull limit on the 5 N peak-hold spring scale.
    * Raking into an upper crossing while downstream crossings remain unworked jams the crossings together; forcing
      the comb > 1.8 cm into a multi-crossing jam exceeds the 6.5 N single-bundle pull-out limit and detaches a strand.
"""

from __future__ import annotations

import math

import numpy as np

N_STRANDS = 6
N_NODES = 20
SEG_L = 0.020
LOCK_LEN = (N_NODES - 1) * SEG_L  # 0.380 m
WEFT_W = 0.025
Y_ROOT = np.linspace(-WEFT_W / 2, WEFT_W / 2, N_STRANDS)

ROOT_X = 0.420
ROOT_DZ = 0.120      # scalp eyelet height above table top
SHOULDER_DZ = 0.114  # shoulder crest height above table top (x ~ 0.410..0.425)
TOWEL_DZ = 0.055     # chest towel top height above table top (x = 0.045..0.380, |y| <= 0.085)
THICK = 0.0028       # strand contact radius on towel/shoulder

LOCK_MASS = 0.008    # 8.0 g Kanekalon weft lock
NODE_M = 2.5e-4      # effective XPBD nodal mass for substep stability
STRETCH_K = 650.0    # N/m axial stiffness per strand segment
BEND_K = 1.6         # N/m smoothing stiffness across j -> j+2
CROSS_K = 260.0      # N/m entwining stiffness of an active crossing
DAMP = 0.72          # velocity damping per substep
MU_TOWEL = 0.22      # Coulomb friction of synthetic hair on cotton terry towel

ROOT_LIMIT_N = 2.0   # N: scalp weft pull limit (red mark on the 5 N peak-hold spring scale)
STRAND_BREAK_N = 6.5 # N: single-strand root pull-out / snap threshold in a jammed mat
TANGLE_DRAG_N = 3.2  # N: comb drag while working through one active crossing
CLEAN_DRAG_N = 0.30  # N: comb drag through smooth, detangled hair
CLEAR_TRAVEL = 0.050 # m of engaged comb travel toward the tips to resolve one crossing
JAM_TRAVEL = 0.018   # m of forced comb travel into >=2 stacked crossings before strand pull-out
HOLD_MARGIN = 0.020  # m: holding pinch must be at least 2.0 cm nearer the scalp than the active crossing

# Pinch thresholds (jaw aperture in m, summed across both fingers)
PINCH_CLOSE = 0.020
PINCH_OPEN = 0.028
PINCH_SLIP = 0.050
PINCH_RECENT = 1.5
PIN_LO = np.array([-0.024, -0.022, -0.020])
PIN_HI = np.array([+0.024, +0.022, +0.020])

# Comb local geometry (origin at centre of 19.6 cm comb; local +y from handle butt toward blade tip; teeth along -x)
COMB_LEN = 0.196
COMB_GRIP_LOCAL = np.array([0.0, -0.062, 0.0])
N_TEETH = 10
TOOTH_Y = -0.001 + np.arange(N_TEETH) * 0.0105  # y_local in [-0.001, +0.0935]
TOOTH_X_BASE = 0.007
TOOTH_X_TIP = -0.027                            # 3.4 cm tooth length along -x_local

# Three sequential crossings: (name, centre node index j, arc-length s_m, ((strand_a, strand_b, over_sign), ...))
CROSSINGS = (
    ("C1_upper", 8, 8 * SEG_L, ((0, 4, +1.0), (1, 5, -1.0), (2, 3, +1.0))),
    ("C2_mid", 11, 11 * SEG_L, ((0, 5, -1.0), (1, 3, +1.0), (2, 4, -1.0))),
    ("C3_distal", 14, 14 * SEG_L, ((0, 3, +1.0), (2, 5, -1.0), (1, 4, +1.0))),
)


def support_dz(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Height (m above table top) of the draped shoulder and chest-towel support under (x, y)."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    on_towel = (x >= 0.040) & (x <= 0.435) & (np.abs(y) <= 0.085)
    t_sh = np.clip((x - 0.365) / 0.045, 0.0, 1.0)
    sh_profile = TOWEL_DZ + (SHOULDER_DZ - TOWEL_DZ) * (0.5 - 0.5 * np.cos(math.pi * t_sh))
    return np.where(on_towel, sh_profile, 0.002)


class HairLock:
    """6-bundle XPBD hair lock with 3 entwined crossings, gripper pinches, and comb-tooth mechanics."""

    def __init__(self, table_z: float):
        self.table_z = float(table_z)
        self.s_nodes = np.arange(N_NODES, dtype=float) * SEG_L
        self.anchors0 = np.column_stack([
            np.full(N_STRANDS, ROOT_X),
            Y_ROOT.copy(),
            np.full(N_STRANDS, self.table_z + ROOT_DZ),
        ])
        self._build_cross_arrays()
        self._build_tube_topology()
        self.reset()

    def _build_cross_arrays(self) -> None:
        ck, sa, sb, jj, tgt_y, tgt_z = [], [], [], [], [], []
        for k, (_n, jk, _sk, pairs) in enumerate(CROSSINGS):
            for dj, sc in ((0, 1.0), (-1, 0.55), (+1, 0.55)):
                for a, b, sgn in pairs:
                    ck.append(k)
                    sa.append(a)
                    sb.append(b)
                    jj.append(jk + dj)
                    tgt_y.append(sgn * 0.026 * sc)
                    tgt_z.append(sgn * 0.009 * sc)
        self._cr_k = np.asarray(ck, dtype=np.int32)
        self._cr_a = np.asarray(sa, dtype=np.int32)
        self._cr_b = np.asarray(sb, dtype=np.int32)
        self._cr_j = np.asarray(jj, dtype=np.int32)
        self._cr_ty = np.asarray(tgt_y, dtype=float)
        self._cr_tz = np.asarray(tgt_z, dtype=float)

    def _build_tube_topology(self, sides: int = 6) -> None:
        self.tube_sides = sides
        ang = np.linspace(0.0, 2.0 * math.pi, sides, endpoint=False)
        self._ring_cos = np.cos(ang)
        self._ring_sin = np.sin(ang)
        faces = []
        per_strand = N_NODES * sides + 2
        for si in range(N_STRANDS):
            base = si * per_strand
            cap0 = base + N_NODES * sides
            cap1 = cap0 + 1
            for k in range(sides):
                kn = (k + 1) % sides
                faces.append((cap0, base + kn, base + k))
                r_last = base + (N_NODES - 1) * sides
                faces.append((cap1, r_last + k, r_last + kn))
            for j in range(N_NODES - 1):
                r0 = base + j * sides
                r1 = base + (j + 1) * sides
                for k in range(sides):
                    kn = (k + 1) % sides
                    faces.append((r0 + k, r0 + kn, r1 + kn))
                    faces.append((r0 + k, r1 + kn, r1 + k))
        self.faces = np.asarray(faces, dtype=np.int32)
        palette = (
            (0.42, 0.21, 0.10),
            (0.56, 0.31, 0.15),
            (0.32, 0.15, 0.07),
            (0.60, 0.35, 0.18),
            (0.46, 0.24, 0.12),
            (0.36, 0.18, 0.08),
        )
        vcols = np.zeros((N_STRANDS * per_strand, 3), dtype=np.float32)
        for si in range(N_STRANDS):
            vcols[si * per_strand:(si + 1) * per_strand] = palette[si]
        self.vertex_colors = vcols

    def tube_vertices(self) -> np.ndarray:
        sides = self.tube_sides
        per_strand = N_NODES * sides + 2
        out = np.zeros((N_STRANDS * per_strand, 3), dtype=np.float32)
        r_y, r_z = 0.0025, 0.0021
        for si in range(N_STRANDS):
            pts = self.x[si]
            tang = np.zeros_like(pts)
            tang[1:-1] = pts[2:] - pts[:-2]
            tang[0] = pts[1] - pts[0]
            tang[-1] = pts[-1] - pts[-2]
            tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-9)
            up = np.array([0.0, 0.0, 1.0])
            n1 = np.cross(tang, up)
            n1_norm = np.linalg.norm(n1, axis=1, keepdims=True)
            n1 = np.where(n1_norm > 1e-6, n1 / np.maximum(n1_norm, 1e-9), np.array([0.0, 1.0, 0.0]))
            n2 = np.cross(tang, n1)
            rings = (pts[:, None, :]
                     + r_y * self._ring_cos[None, :, None] * n1[:, None, :]
                     + r_z * self._ring_sin[None, :, None] * n2[:, None, :])
            base = si * per_strand
            out[base:base + N_NODES * sides] = rings.reshape(-1, 3)
            out[base + N_NODES * sides] = pts[0]
            out[base + N_NODES * sides + 1] = pts[-1]
        return out

    def reset(self) -> None:
        self.time = 0.0
        self.anchors = self.anchors0.copy()
        self.root_attached = np.ones(N_STRANDS, dtype=bool)
        self.u = np.zeros(len(CROSSINGS), dtype=float)
        self.jam_disp = 0.0

        self.x = np.zeros((N_STRANDS, N_NODES, 3), dtype=float)
        for si in range(N_STRANDS):
            for j in range(N_NODES):
                s = j * SEG_L
                x_j = ROOT_X - (s if s <= 0.14 else 0.14 + 0.92 * (s - 0.14))
                y_j = Y_ROOT[si] * (1.0 + 0.45 * max(0.0, (s - 0.28) / 0.10))
                z_j = self.table_z + float(support_dz(x_j, y_j)) + THICK + 0.001
                if 0.04 <= s <= 0.15:
                    t = (s - 0.04) / 0.11
                    z_j = max(z_j, self.table_z + (1.0 - t) * SHOULDER_DZ + t * (TOWEL_DZ + 0.005))
                self.x[si, j] = (x_j, y_j, z_j)
        self.v = np.zeros_like(self.x)

        self.pins: dict[int, tuple[np.ndarray, np.ndarray, float]] = {}
        self.open_t = np.zeros(2, dtype=float)
        self.prev_teeth_x: float | None = None
        self.comb_engaged = False
        self.stroke_active = False
        self.stroke_start_s = 1.0
        self.stroke_enter_x = -1.0
        self.stroke_caught: list[int] = []
        self.stroke_max_s = 0.0
        self.stroke_all_clear = False
        self.stroke_supported_above = False

        self.cur_root_pull_N = 0.0
        self.max_root_pull_N = 0.0
        self.root_overpull = False
        self.strand_broken = False
        self.unheld_tangle_comb = False
        self.supported_tangle_comb = False
        self.distal_clear_stroke = False
        self.mid_clear_stroke = False
        self.final_clean_stroke = False
        self.completed_strokes: list[tuple[float, float, bool]] = []
        self.bad = False

        dummy_tcp = np.array([[0.0, 0.4, self.table_z + 0.5], [0.0, -0.4, self.table_z + 0.5]])
        dummy_rot = np.stack([np.eye(3), np.eye(3)])
        dummy_ap = np.array([0.08, 0.08])
        for _ in range(25):
            self.step(dummy_tcp, dummy_rot, dummy_ap, comb_pos=np.array([0.24, -0.24, self.table_z + 0.02]),
                      comb_rot=np.eye(3), comb_holder=None, settle_only=True)
        self.v[:] = 0.0
        self.cur_root_pull_N = 0.12
        self.max_root_pull_N = 0.12
        self.time = 0.0

    def comb_teeth_world(self, comb_pos: np.ndarray, comb_rot: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        bases_loc = np.column_stack([np.full(N_TEETH, TOOTH_X_BASE), TOOTH_Y, np.zeros(N_TEETH)])
        tips_loc = np.column_stack([np.full(N_TEETH, TOOTH_X_TIP), TOOTH_Y, np.zeros(N_TEETH)])
        return comb_pos + bases_loc @ comb_rot.T, comb_pos + tips_loc @ comb_rot.T

    def s_at_x(self, x_val: float) -> float:
        mean_x = self.x[:, :, 0].mean(axis=0)
        if x_val >= mean_x[0]:
            return 0.0
        if x_val <= mean_x[-1]:
            return LOCK_LEN
        for j in range(N_NODES - 1):
            if mean_x[j] >= x_val >= mean_x[j + 1]:
                denom = max(mean_x[j] - mean_x[j + 1], 1e-6)
                return float((j + (mean_x[j] - x_val) / denom) * SEG_L)
        idx = int(np.argmin(np.abs(mean_x - x_val)))
        return float(idx * SEG_L)

    def crossing_world_pos(self, k: int) -> np.ndarray:
        _, j, _, _ = CROSSINGS[k]
        return self.x[:, j, :].mean(axis=0)

    def held_s_max(self, exclude_arm: int | None = None) -> float | None:
        vals = [s_pin for arm, (_m, _o, s_pin, _jm) in self.pins.items() if arm != exclude_arm]
        return min(vals) if vals else None

    def _update_pins(self, tcp: np.ndarray, rot: np.ndarray, aperture: np.ndarray, comb_holder: int | None) -> None:
        for arm in range(2):
            if arm == comb_holder:
                self.pins.pop(arm, None)
                continue
            if aperture[arm] > PINCH_CLOSE:
                self.open_t[arm] = self.time
            if arm in self.pins:
                mask, off, _spin, _jm = self.pins[arm]
                tgt = tcp[arm] + off @ rot[arm].T
                slip = float(np.linalg.norm(self.x[mask] - tgt, axis=1).mean())
                if aperture[arm] > PINCH_OPEN or slip > PINCH_SLIP:
                    del self.pins[arm]
                continue
            if aperture[arm] > PINCH_CLOSE or (self.time - self.open_t[arm]) > PINCH_RECENT:
                continue
            loc = (self.x - tcp[arm]) @ rot[arm]
            inside = np.all((loc >= PIN_LO) & (loc <= PIN_HI), axis=2)
            inside[:, 0] = False
            for other_mask, _o, _s, _jm in self.pins.values():
                inside &= ~other_mask
            if int(inside.any(axis=1).sum()) < 3:
                continue
            j_hits = np.flatnonzero(inside.any(axis=0))
            j_mid = int(round(float(j_hits.mean())))
            mask = np.zeros((N_STRANDS, N_NODES), dtype=bool)
            for j in range(max(1, j_mid - 1), min(N_NODES, j_mid + 2)):
                mask[:, j] = True
            off = (self.x[mask] - tcp[arm]) @ rot[arm]
            s_pin = float(j_mid * SEG_L)
            self.pins[arm] = (mask, off, s_pin, j_mid)

    def step(
        self,
        tcp: np.ndarray,
        rot: np.ndarray,
        aperture: np.ndarray,
        comb_pos: np.ndarray,
        comb_rot: np.ndarray,
        comb_holder: int | None,
        boxes: list = (),
        dt: float = 0.05,
        substeps: int = 8,
        iters: int = 20,
        settle_only: bool = False,
    ) -> None:
        tcp = np.asarray(tcp, float)
        rot = np.asarray(rot, float)
        aperture = np.asarray(aperture, float)
        comb_pos = np.asarray(comb_pos, float)
        comb_rot = np.asarray(comb_rot, float)
        self.time += dt

        if not settle_only:
            self._update_pins(tcp, rot, aperture, comb_holder)

        tb, tt = self.comb_teeth_world(comb_pos, comb_rot)
        t_mid = 0.5 * (tb + tt)
        tx = float(t_mid[:, 0].mean())
        ty_min, ty_max = float(min(tb[:, 1].min(), tt[:, 1].min())), float(max(tb[:, 1].max(), tt[:, 1].max()))
        tz_min, tz_max = float(min(tb[:, 2].min(), tt[:, 2].min())), float(max(tb[:, 2].max(), tt[:, 2].max()))

        x_tips = float(self.x[:, -1, 0].mean())
        x_root = float(self.x[:, 0, 0].mean())
        s_comb = self.s_at_x(tx)
        j_comb = int(np.clip(round(s_comb / SEG_L), 1, N_NODES - 1))
        lock_y = self.x[:, j_comb, 1]
        lock_z = float(self.x[:, j_comb, 2].mean())
        in_y = int(((lock_y >= ty_min - 0.010) & (lock_y <= ty_max + 0.010)).sum())

        in_plane = (
            not settle_only
            and comb_holder is not None
            and in_y >= 3
            and tz_min <= lock_z + 0.016
            and tz_max >= lock_z - 0.016
        )
        engaged = in_plane and (x_tips - 0.020 <= tx <= x_root + 0.010)

        # A closed gripper pinching the lock downstream of where the comb entered physically blocks the comb pass
        blocked_s = [
            s_p for arm, (_m, _o, s_p, _jm) in self.pins.items()
            if arm != comb_holder and self.stroke_active and s_p >= self.stroke_start_s - 0.005
        ]
        pin_block = bool(blocked_s and s_comb >= min(blocked_s) - 0.010)

        ext_force = np.zeros_like(self.x)
        if engaged and not pin_block:
            if not self.stroke_active:
                self.stroke_active = True
                self.stroke_start_s = s_comb
                self.stroke_enter_x = tx
                self.stroke_max_s = s_comb
                self.stroke_all_clear = bool(np.all(self.u >= 1.0 - 1e-6))
                self.stroke_supported_above = False
                self.stroke_caught = [
                    k for k, (_n, jk, _sk, _p) in enumerate(CROSSINGS)
                    if self.u[k] < 1.0 and float(self.x[:, jk, 0].mean()) <= tx + 0.008
                ]
            self.stroke_max_s = max(self.stroke_max_s, s_comb)

            dx = 0.0 if self.prev_teeth_x is None else (tx - self.prev_teeth_x)
            ds = max(0.0, -dx)
            if ds > 1e-4:
                still_active = [k for k in self.stroke_caught if self.u[k] < 1.0]
                reached = [
                    k for k in still_active
                    if tx <= float(self.x[:, CROSSINGS[k][1], 0].mean()) + 0.016
                ]
                if reached:
                    k_act = min(reached, key=lambda k: CROSSINGS[k][2])
                    _, j_act, s_act, _ = CROSSINGS[k_act]
                    s_hold = self.held_s_max(exclude_arm=comb_holder)
                    held_above = s_hold is not None and s_hold <= s_act - HOLD_MARGIN + 1e-6
                    if not held_above:
                        self.unheld_tangle_comb = True
                    else:
                        self.supported_tangle_comb = True
                        self.stroke_supported_above = True

                    # Any uncleared crossing at or downstream of k_act stacks into a jam when >=2 stages remain
                    downstream_active = [
                        k for k in range(k_act, len(CROSSINGS)) if self.u[k] < 1.0
                    ]
                    if len(still_active) >= 2 or len(downstream_active) >= 2:
                        self.jam_disp += ds
                        f_jam = min(8.5, TANGLE_DRAG_N + (STRAND_BREAK_N - TANGLE_DRAG_N) * (self.jam_disp / JAM_TRAVEL))
                        ext_force[:, j_act, 0] -= f_jam / N_STRANDS
                        if self.jam_disp >= JAM_TRAVEL:
                            self.strand_broken = True
                            self.root_attached[2] = False
                    else:
                        self.jam_disp = max(0.0, self.jam_disp - 0.5 * ds)
                        ext_force[:, j_act, 0] -= TANGLE_DRAG_N / N_STRANDS
                        if held_above:
                            self.u[k_act] = min(1.0, self.u[k_act] + ds / CLEAR_TRAVEL)
                else:
                    ext_force[:, j_comb, 0] -= CLEAN_DRAG_N / N_STRANDS

            if tx <= x_tips and self.stroke_max_s >= LOCK_LEN - 1e-6:
                self._record_completed_stroke()
            self.prev_teeth_x = tx
        else:
            disengaged = (
                comb_holder is None
                or tz_min > lock_z + 0.022
                or tx < x_tips - 0.015
                or in_y == 0
                or pin_block
            )
            if disengaged:
                if self.stroke_active and not pin_block and self.prev_teeth_x is not None and self.prev_teeth_x <= x_tips:
                    self._record_completed_stroke()
                self.stroke_active = False
                self.stroke_caught = []
                self.prev_teeth_x = None
                self.jam_disp = max(0.0, self.jam_disp - 0.005)

        self.comb_engaged = engaged

        # -- Vectorized XPBD substeps ---------------------------------------------------------------------------------
        h = dt / substeps
        a_stretch = (1.0 / STRETCH_K) / (h * h)
        a_bend = (1.0 / BEND_K) / (h * h)

        inv_m = np.full((N_STRANDS, N_NODES), 1.0 / NODE_M)
        pin_targets = []
        pin_excl = {}
        for _arm, (mask, off, _spin, j_mid) in self.pins.items():
            tgt = tcp[_arm] + off @ rot[_arm].T
            pin_targets.append((mask, self.x[mask].copy(), tgt))
            inv_m[mask] = 0.0
            excl = np.zeros((N_STRANDS, N_NODES), dtype=bool)
            excl[:, max(0, j_mid - 3):min(N_NODES, j_mid + 4)] = True
            pin_excl[_arm] = excl

        even_j = np.arange(0, N_NODES - 1, 2)
        odd_j = np.arange(1, N_NODES - 1, 2)
        root_force_peak_step = 0.0

        rest_seg = np.full((N_STRANDS, N_NODES - 1), SEG_L)
        for k, (_n, jk, _sk, _pairs) in enumerate(CROSSINGS):
            w_k = 1.0 - self.u[k]
            if w_k > 0.0:
                rest_seg[:, max(0, jk - 1):min(N_NODES - 1, jk + 1)] *= (1.0 - 0.14 * w_k)

        w_cr = 1.0 - self.u[self._cr_k]
        active_cr = w_cr > 1e-4

        for sub in range(substeps):
            frac = (sub + 1) / substeps
            prev = self.x.copy()
            self.v *= DAMP
            self.v[:, :, 2] -= 9.81 * h
            self.v += (ext_force * inv_m[:, :, None]) * h
            self.x += self.v * h

            for mask, p0, p1 in pin_targets:
                self.x[mask] = p0 + frac * (p1 - p0)

            if engaged:
                free_j = inv_m[:, j_comb] > 0
                self.x[free_j, j_comb, 1] += 0.30 * (Y_ROOT[free_j] - self.x[free_j, j_comb, 1])
                self.x[free_j, j_comb, 2] = np.minimum(self.x[free_j, j_comb, 2], tz_max - 0.003)

            lam_root = np.zeros(N_STRANDS)
            lam_seg = np.zeros((N_STRANDS, N_NODES - 1))

            for _it in range(iters):
                att = self.root_attached & (inv_m[:, 0] > 0.0)
                if att.any():
                    d0 = self.x[att, 0] - self.anchors[att]
                    ln0 = np.linalg.norm(d0, axis=1)
                    nz = ln0 > 1e-9
                    if nz.any():
                        idx_att = np.flatnonzero(att)[nz]
                        dl0 = (-ln0[nz] - a_stretch * lam_root[idx_att]) / (inv_m[idx_att, 0] + a_stretch)
                        lam_root[idx_att] += dl0
                        self.x[idx_att, 0] += (inv_m[idx_att, 0] * dl0)[:, None] * (d0[nz] / ln0[nz, None])

                for js in (even_j, odd_j):
                    d = self.x[:, js + 1, :] - self.x[:, js, :]
                    ln = np.linalg.norm(d, axis=2)
                    C = ln - rest_seg[:, js]
                    wa, wb = inv_m[:, js], inv_m[:, js + 1]
                    dl = (-C - a_stretch * lam_seg[:, js]) / np.maximum(wa + wb + a_stretch, 1e-12)
                    dl = np.minimum(lam_seg[:, js] + dl, 0.0) - lam_seg[:, js]
                    lam_seg[:, js] += dl
                    g = d / np.maximum(ln, 1e-12)[:, :, None]
                    self.x[:, js, :] -= (wa * dl)[:, :, None] * g
                    self.x[:, js + 1, :] += (wb * dl)[:, :, None] * g

                db = self.x[:, 2:, :] - self.x[:, :-2, :]
                lb = np.linalg.norm(db, axis=2)
                over = lb < 2.0 * SEG_L * 0.96
                if over.any():
                    Cb = lb - 2.0 * SEG_L * 0.98
                    w0, w2 = inv_m[:, :-2], inv_m[:, 2:]
                    dlb = -Cb / np.maximum(w0 + w2 + a_bend, 1e-12)
                    gb = db / np.maximum(lb, 1e-12)[:, :, None]
                    self.x[:, :-2, :] -= (w0 * dlb * over)[:, :, None] * gb
                    self.x[:, 2:, :] += (w2 * dlb * over)[:, :, None] * gb

                if active_cr.any():
                    idx_c = np.flatnonzero(active_cr)
                    ca, cb, cj = self._cr_a[idx_c], self._cr_b[idx_c], self._cr_j[idx_c]
                    wc = w_cr[idx_c]
                    wa_c, wb_c = inv_m[ca, cj], inv_m[cb, cj]
                    a_cr = (1.0 / (CROSS_K * wc)) / (h * h)
                    tgt = np.column_stack([np.zeros(len(idx_c)), self._cr_ty[idx_c] * wc, self._cr_tz[idx_c] * wc])
                    err = (self.x[ca, cj] - self.x[cb, cj]) - tgt
                    corr = -err / np.maximum(wa_c + wb_c + a_cr, 1e-12)[:, None]
                    np.add.at(self.x, (ca, cj), wa_c[:, None] * corr)
                    np.add.at(self.x, (cb, cj), -wb_c[:, None] * corr)

                sup_z = self.table_z + support_dz(self.x[:, :, 0], self.x[:, :, 1]) + THICK
                for k, (_n, jk, _sk, _pairs) in enumerate(CROSSINGS):
                    wk = 1.0 - self.u[k]
                    if wk > 1e-3:
                        sup_z[:, jk] += 0.008 * wk
                low = (self.x[:, :, 2] < sup_z) & (inv_m > 0.0)
                if low.any():
                    self.x[:, :, 2] = np.where(low, sup_z, self.x[:, :, 2])

            sup_z = self.table_z + support_dz(self.x[:, :, 0], self.x[:, :, 1]) + THICK
            on_sup = (self.x[:, :, 2] <= sup_z + 5e-4) & (inv_m > 0.0)
            if on_sup.any():
                dx_xy = self.x[:, :, :2] - prev[:, :, :2]
                dl_xy = np.linalg.norm(dx_xy, axis=2)
                max_fric = MU_TOWEL * 9.81 * h * h
                sc = np.where(dl_xy <= max_fric, 1.0, max_fric / np.maximum(dl_xy, 1e-12))
                self.x[:, :, :2] = np.where(on_sup[:, :, None], self.x[:, :, :2] - dx_xy * sc[:, :, None], self.x[:, :, :2])

            for world_m, lo, hi, _arm in boxes:
                Rm, t_b = world_m[:3, :3], world_m[:3, 3]
                loc = (self.x - t_b) @ Rm
                free_mask = (inv_m > 0.0) & (~pin_excl[_arm] if _arm in pin_excl else True)
                hit = np.all((loc > lo - THICK) & (loc < hi + THICK), axis=2) & free_mask
                if hit.any():
                    d_lo = loc[hit] - (lo - THICK)
                    d_hi = (hi + THICK) - loc[hit]
                    ax = np.argmin(np.minimum(d_lo, d_hi), axis=1)
                    rows = np.arange(len(ax))
                    new_loc = loc[hit].copy()
                    new_loc[rows, ax] = np.where(d_lo[rows, ax] < d_hi[rows, ax], lo[ax] - THICK, hi[ax] + THICK)
                    self.x[hit] = new_loc @ Rm.T + t_b

            f_root_sub = float(np.sum(np.abs(lam_root[self.root_attached]) / (h * h)))
            f_seg0_sub = float(np.sum(np.maximum(0.0, -lam_seg[self.root_attached, 0]) / (h * h)))
            root_force_peak_step = max(root_force_peak_step, max(f_root_sub, f_seg0_sub))

            self.v = (self.x - prev) / h
            sp = np.linalg.norm(self.v, axis=2, keepdims=True)
            self.v *= np.minimum(1.0, 2.0 / np.maximum(sp, 1e-9))

        if not settle_only:
            self.cur_root_pull_N = root_force_peak_step
            self.max_root_pull_N = max(self.max_root_pull_N, root_force_peak_step)
            if self.max_root_pull_N > ROOT_LIMIT_N:
                self.root_overpull = True

        if not np.isfinite(self.x).all():
            self.bad = True
            self.x = np.nan_to_num(self.x, nan=self.table_z + TOWEL_DZ)

    def _record_completed_stroke(self) -> None:
        self.completed_strokes.append((self.stroke_start_s, LOCK_LEN, self.stroke_all_clear))
        if self.u[2] >= 1.0 - 1e-6 and self.stroke_start_s <= 0.285:
            self.distal_clear_stroke = True
        if self.u[2] >= 1.0 - 1e-6 and self.u[1] >= 1.0 - 1e-6 and self.stroke_start_s <= 0.210:
            self.mid_clear_stroke = True
        if np.all(self.u >= 1.0 - 1e-6) and self.stroke_all_clear and self.stroke_start_s <= 0.030:
            self.final_clean_stroke = True
        self.stroke_active = False
