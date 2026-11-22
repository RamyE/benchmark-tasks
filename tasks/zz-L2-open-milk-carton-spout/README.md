# Open a half-gallon milk carton's gable-top spout on its 'open here' end, holding the carton

Instruction: `Open the milk carton's spout.`

A full, sealed half-gallon (1.89 L) gable-top milk carton stands upright in a rimmed silicone spill tray. One hand
steadies the 2.00 kg carton by its walls or sealed rear 4-ply top fin while the other pushes both `'OPEN HERE'` wings
inward, peels the front top-fin seam into an open pouring passage, and pinches and unfolds the V-spout past the end wall.

## Why it matters and what makes it hard

- **Bimanual stabilization.** Unsealing wings (`~3.2 N`), peeling the front seam (`~6.8 N`), and pulling the spout at
  ~20-23 cm height apply ~1.3-1.6 N m (> 0.93 N m tipping moment); shift `<= 8.0 cm` and physical steadying contact (`>= 0.5 N`) rule out rim bracing.
- **Three-stage opening on the `'OPEN HERE'` end (`-y`).** `+y` is permanently sealed; both `-y` wings (`>= 25 deg`) and
  both front seam halves (`>= 12 deg`/side, `>= 2.0 cm` throat) must unseal before the `3.4 mm` 4-ply V-spout unlocks and unfolds.

## Real-world setup and waterproof rig protection

Positions in cm from arm-base midpoint (`x` forward, `y` left; bases `(0, +-30.5)`). Counter `z = 74.75 cm`; tray floor `z = 74.95 cm`.

| Item | Commercial product / material | Dimensions and mass | Placement |
|---|---|---|---|
| Spill tray & protection | Fixed rimmed grey silicone tray (2.2 cm interior depth = **2.11 L > 1.89 L**) with 9.5 cm mark; poly counter drape; nitrile finger sleeves and waterproof wrist wraps on every trial | 32.0 x 32.0 x 2.4 cm (`31.0 x 31.0 x 2.2 cm` = 2.11 L), fixed (`mu_s = 0.75`, `mu_d = 0.65`) | `(30.0, 0.0)` |
| Milk carton | Commercial half-gallon (1.89 L) chilled (2-4 C) whole-milk Pure-Pak / Pactiv Evergreen 95 mm flat-gable carton (`0.85 mm` PE-coated board/ply, no cap; 2-ply `1.8 mm`, 4-ply `3.4-3.6 mm`), wiped dry | 9.5 x 9.5 cm base, 19.0 cm walls, 4.5 cm roof (43.5 deg), 1.3 x 0.36 cm 4-ply top fin (`0.18 cm`/side on split seam), 24.8 cm tall; 2.00 kg | Upright in mark `(30.0, 0.0)`, `'OPEN HERE'` `-y` |
| Front seam & wings | Rear (`+y`, `4.55 cm`) roof/fin sealed; front (`-y`, `4.95 cm`) split into two hinged roof/2-ply-fin halves (`1.8 mm`/side) sealed at `x = 0`, plus two wings/end (`4.75 x 4.5 cm`) on roof edges | Seam: `4.95 cm`, `1.8 mm`/side; wings: `4.75 x 4.5 cm` | Sealed flat; `-y` red `'OPEN HERE'`, `+y` blue |
| Pour spout | Integral folded V-beak (`3.4 mm` 4-ply front crease up to pouring-lip apex, no tab ahead of lip) | 3.4 cm V-beak, 4.3 cm crease from lower hinge | Hinge 1.6 cm inside `-y`; unfolds up to 70 deg (`+2.44 cm`) |

**Reset:** Fit fresh nitrile finger sleeves, check wrist wraps and drapes, wipe the 2.11 L tray dry, and set a fresh
sealed commercial 1.89 L (2.00 kg) chilled (2-4 C) whole-milk Pure-Pak 95 x 95 mm carton (wiped dry) upright inside the
9.5 cm mark at `(30.0, 0.0)` with `'OPEN HERE'` facing `-y`.

## Scoring

Evaluated at the end of the 600-step (30 s at 20 Hz) trial, with whole-trial spill, damage, and hold latches:

- **Score 0** if ever: (1) `+y` wing folds `> 15 deg`, (2) carton tips `> 20 deg`, (3) base shifts `> 8.0 cm` (keeping
  the carton clear of the 31.0 cm tray rim, ruling out rim bracing), or (4) a `-y` wing or front seam unseals without the
  other gripper in physical steadying contact with the carton (`>= 0.5 N` PhysX pad contact force in sim, `0 N` when
  hovering; visible finger-pad pinch/bracing contact on the carton on video in real trials); **or** at end either gripper
  has any point `< 2.0 cm` from carton/seam/spout.
- **0.2** when both `-y` wings unsealed (`>= 25 deg` peak, `>= 15 deg` end).
- **0.4** when `0.2` holds and both front seam halves separated (`>= 12 deg` peak, `>= 10 deg` end, `>= 2.0 cm` throat).
- **0.7** when `0.4` holds and the frontmost pouring-lip vertex projects `>= 1.0 cm` past the `-y` wall plane.
- **1.0** when `0.7` holds, throat is `>= 2.0 cm`, lip projects `>= 2.0 cm` past `-y`, and both grippers are `>= 10.0 cm`
  clear and `< 2.0 cm/s` continuously for `1.0 s` (`20` steps).

**Real-trial check:** End photo with ruler/caliper + video: (1) open pouring throat `>= 2.0 cm` between front top-fin
rims, (2) lip projection `>= 1.0` / `2.0 cm` past `-y`, and (3) visible finger-pad pinch/bracing contact steadying the
carton throughout wing and seam unsealing (hovering scores 0), tilt `<= 20 deg`, `+y` wings `<= 15 deg`, shift
`<= 8.0 cm`, and grippers `>= 10.0 cm` clear and still for `1.0 s`.

## Simulation fidelity

- **Pure contact grasping and multi-ply board (no welds):** Rear `3.6 mm` 4-ply top-fin hold (`4 x 0.85 mm` PE-coated
  board + bead; `1.8 mm`/side on split seam) and `3.4 mm` 4-ply spout pinch use native PhysX contact/friction (`mu_s = 2.00`,
  up to 50 N; zero `FixedJoint` or welds). Steadying is verified via PhysX pad contact force (`>= 0.5 N`, `0 N` hovering).
- **Wings, peel-sealed seam, and passive V-spout:** Rigid 2.00 kg carton in fixed `mu_s = 0.75` tray (liquid ballast;
  flex/slosh unmodelled). Hinged wings (`0.075 N m` `-y`, `0.250 N m` `+y`, ISO 17480 range) break at `25 deg` into
  `0.015 N m` friction and `24 deg` rest. Front roof/fin halves (`0.085 N m`/side peel seal, `~6.8 N` total) break at
  `12 deg` into `0.025 N m` friction and `<=72%` peak plastic rest (cap `16 deg`, zero self-opening). The V-spout
  (`1.6 cm` inside `-y`) unlocks after wings + seam unseal and unfolds up to `70 deg` (`+2.44 cm`) against `0.055 N m` friction.

## Running a trial and demo verification

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 600 --instruction "Open the milk carton's spout."
```

A scripted policy steadies the rear `3.6 mm` fin with the left hand while the right unseals both `-y` wings, peels the
front seam (`3.8 cm` throat), pinches the `3.4 mm` spout crease, pulls the lip `2.4 cm` past `-y`, releases, and parks
(`1.0` in 328 steps). Ablations: idle (`0.0`), one-arm (`0.0`), `+y` (`0.0`), keep-hold (`0.0`), wings-only (`0.2`),
wings + seam (`0.4`), and half-spout (`0.7`).

## Nearest tasks and sources

- **Nearest tasks:** Board egg-carton/box tasks fold lids or flaps; none unseals gable wings, peels a top-fin seam, and
  unfolds a V-spout on a 2 kg carton. External `PickPlaceMilk` (robosuite) and `PnPMilkToMicrowaveClose` (RoboCasa) only move a rigid carton.
- **Sources:** Elopak Pure-Pak 95 x 95 mm gable-top (https://www.elopak.com/); Pactiv Evergreen half-gallon cartons
  (https://www.pactivevergreen.com/); ISO 17480:2015 ease-of-opening (https://www.iso.org/standard/59895.html).
