# Comb a three-stage sleep mat out of a care manikin's long hair ends-to-roots without scalp pull

Instruction: `Comb the tangle out of the lock of hair lying over her shoulder so the comb runs from scalp to tip without pulling on her scalp, then put the comb back on its tray.`

## Occupational context

Nursing assistants (O*NET 31-1131.00 core task: *"Wash, groom, shave, or drape patients to prepare them for surgery, treatment, or examination"*; 1,351,760 US workers, BLS OEWS May 2023) and home health and personal care aides (O*NET 31-1122.00 core personal hygiene assistance; 3,689,350 US workers) clear localized side-sleeping pillow mats that form behind a bedbound resident's ear. On a reclined care-training manikin, standard nurse-aide (CNA) skills checklists teach aides to drape a towel over the shoulder, sweep the matted hair forward onto the towel, grip firmly between the scalp and the active snarl so combing drag lands on the caregiver's holding hand rather than pulling painful roots, comb the distal ends free first and step upward through stacked knots, verify with one smooth root-to-tip stroke, and return the comb to its tray.

## Why two hands and progressive ends-first combing are required

- **Scalp force isolation (two hands):** Working a wide-tooth comb through one snarl stage generates `3.2 N` of combing drag (`3.2 +- 0.5 N` bench calibration). Combing one-handed transmits that pull straight to the roots, driving the peak-hold root spring scale past the `2.0 N` comfort limit (disqualified, score 0). Pinching the hair with the other gripper at least `2.0 cm` nearer the scalp clamps the proximal hair slack (`< 0.5 N` at the roots) while the holding hand absorbs the comb's drag.
- **Ends-first order:** The matted region (`10-22 cm` from the tips) has three sequential entwined snarl stages (`C3` near the ends, `C2` in the middle, `C1` higher up). Raking from above two or more uncleared stages compacts the upper snarl into the lower one, jamming above `6.5 N` and detaching a magnetic root breakaway clip (score 0).
- **Natural hand spacing:** One hand holds the proximal hair on the chest/shoulder (`x = 0.27-0.37 m, y ~ 0`) while the other holds the `19.6 cm` comb by its handle (`y = -0.156 m`) and sweeps its toothed blade down-lock (`x = 0.40 -> 0.04 m`), keeping the two wrist housings `> 15 cm` apart throughout.

## Real-world setup and repeatable build / tangle reset

Origin at the midpoint between the two arm bases on the station table (`z = 0.7475 m`): `+x` forward away from the robot, `+y` to its left, heights above the table top. Arm bases sit at `(0, +-0.305 m)`.

| Item | Commercial part & mechanical build | Dimensions & mass | Placement |
|---|---|---|---|
| Care-training manikin & sprung neck | Diane D150 adult female PVC care-training head-and-shoulders manikin on a `35 deg` medical foam backrest wedge; neck flange mounted on a vertical `M10 x 50 mm` shoulder-bolt pivot (`McMaster 91259A`) with two opposed steel torsion springs (`0.075 N m/deg` each, `0.15 N m/deg` net centring rate), `+-45 deg` Delrin stop pins, and aligned `8 mm` white vinyl witness stripes across the `52 mm`-radius collar seam (`6.0 deg = 5.5 mm` stripe offset) | Wedge `19 x 36 x 13 cm` (`x 0.44-0.63, y -0.15..+0.21`); upper torso `11 x 34 x 10.8 cm` (`x 0.41-0.52`); head `15.6 x 18.6 x 21.0 cm`, ballasted to `4.0 kg` | Wedge bolted to table; head centred at `(0.495, 0.035, 0.185)` |
| Chest bolster & shoulder towel | Folded white cotton terry hand towel over closed-cell EVA foam chest pad and shoulder pad | Shoulder pad `2.8 x 16.0 x 7.6 cm` at `(0.396, 0, 0.038)` (drape crest `z = 0.114 m`); bolster `33.7 x 15.6 x 4.8 cm`; towel `33.7 x 16.0 x 0.7 cm`, top at `z = 0.055 m` | Centred along `y = 0`, `x = 0.045..0.382 m`. Stitched marks on right towel edge (`y = -0.070`): green scalp-zone mark at `x = 0.394` (`s = 3 cm`), blue midpoint mark at `x = 0.246` (`s = 21 cm`) |
| Loose-fibre hair weft & 6.5 N breakaway root clips | Dry, unconditioned Kanekalon jumbo synthetic hair weft in multi-tone chestnut/auburn shades, parted only at the `25 mm` root header into 6 **loose, unbanded free-fibre sections** `S0..S5` (~330 free fibres per section, ~2,000 loose fibres total — **no bands or ties anywhere along the `38 cm` hair shaft**). Each root sub-tab mates to the collector bar behind the scalp slot via a `6 x 3 mm` N52 neodymium magnet pair shimmed with `0.25 mm` Kapton tape and calibrated with a handheld `10 N` spring scale to release cleanly at `6.5 +- 0.4 N` (snaps back on reset) | `38.0 cm` free length (`s = 0..38 cm`), `25 mm` wide at weft (`5 mm` spacing), `8.0 g` total (`1.33 g` per section) | Exits a `34 x 12 mm` PTFE-lined scalp slot behind right ear at `(0.420, 0.0, 0.120)`, drapes over shoulder onto towel along `y = 0` to tips at `x ~ 0.09 m` |
| 2.0 N root force gauge & cord routing | Ohaus `8003-MN` `5 N` (`500 g x 10 g`) pull-type tubular spring scale fitted with a sliding `1.5 mm` red Buna-N O-ring peak-hold telltale (`< 0.08 N` ring slide friction); collector bar tied with `0.8 mm` braided Dacron cord round a `12 mm` Delrin guide pulley at `(0.432, -0.010, 0.122)` straight along `-y` to the scale hook (`0.1 N` / `10 g` rest preload) | Barrel `12.0 cm x 1.8 cm` (`20 mm/N` plunger travel), mounted horizontally behind right shoulder at `(0.432, -0.130, 0.122)` | Zero tick at `y = -0.075`, red `2.0 N` limit band `40 mm` along barrel at `y = -0.115` |
| Comb & tray | Goody Super Comb wide-tooth detangler in a rimmed polypropylene instrument tray | Comb `19.6 cm` long (`9.0 x 2.2 x 0.9 cm` handle, `10.6 cm` spine, 10 rounded `3.4 cm` teeth at `10.5 mm` pitch), `32 g`; tray `14.0 x 24.0 x 2.0 cm` (`12.4 x 22.4 cm` inside, `0.6 cm` base) | Tray fixed at `(0.24, -0.24)`; comb flat inside at `(0.24, -0.235, 0.012)`, handle toward `-y` (white grip band at `y = -0.309`), teeth toward `-x` |

### Tensioned reset procedure and dry-weft resistance calibration

**Pre-reset bench calibration & clean-gripper rule (before a session and every 10 trials, *prior* to forming the trial reset):** Verify the Ohaus `5 N` peak-hold telltale ring stops within `+-1 mm` of the red `2.0 N` (`40 mm`) band under a `200 g` (`1.96 N`) check pull, verify each magnetic root clip releases at **`6.5 +- 0.4 N`** on a `10 N` scale, and draw the Goody comb hooked to a handheld `5 N` scale through a test `C3` crossing over `5 cm` while pinching at `s = 20 cm` (peak pull **`3.2 +- 0.5 N`**, verified repeatable across 10 independent card-indexed resets). Because that check clears `C3`, **always execute the full Steps 1–3 reset below afterward before starting any scored trial**. **Never apply silicone spray or conditioner** (which could transfer to shared gripper pads); replace the dry weft whenever drag drifts outside `2.7-3.7 N`.

Using a laminated 6-slot indexing card (`5 mm` slot pitch) and a `100 g` (`1.0 N`) clip-on calibration weight:
1. Comb all loose fibres straight into the 6 root-header sections `S0..S5` (`-y` to `+y`) and clip the card at `s = 16 cm`: cross free-fibre sections `(S0 over S4)`, `(S1 under S5)`, `(S2 over S3)` with one `180 deg` inter-section wrap plus `2` light `2 cm` back-comb strokes with a rat-tail comb tip to entwine surface fibres, hang the `100 g` (`1.0 N`) weight across the ends for `2 s`, and unclip to form **`C1`** (`s = 16 cm`, `22 cm` from tips).
2. Repeat at `s = 22 cm` with `(S0 under S5)`, `(S1 over S3)`, `(S2 under S4)` to form **`C2`** (`s = 22 cm`, `16 cm` from tips), and at `s = 28 cm` with `(S0 over S3)`, `(S2 under S5)`, `(S1 over S4)` to form **`C3`** (`s = 28 cm`, `10 cm` from tips).
3. Lay the hair along `y = 0` on the towel, verify all 6 magnetic root clips are seated, slide the red O-ring peak telltale back to the `0.1 N` stop, align the `8 mm` white neck witness stripes, and set the comb flat in its tray.

## Scoring (real trial and simulation)

All disqualifiers latch over the whole trial (every check uses macro camera-observable geometry and the mounted `5 N` peak-hold root scale — no comb force sensor and no individual-fibre or colour counting is required):
- **Score 0** if:
  1. Peak root pull exceeds **2.0 N** (red peak-hold O-ring telltale on the Ohaus `5 N` spring scale moves past the red `2.0 N` band at `40 mm` travel).
  2. Any section's `6.5 N` magnetic root breakaway clip detaches (`> 6.5 N` jam load).
  3. The comb's teeth rake into an uncleared mat stage without the other gripper closed across the lock body (`aperture <= 2.0 cm`, clamping `>= half` of the `25 mm` lock width `>= 12 mm` between its jaws so the proximal hair stays slack above the grip) with its pinch centre at least **2.0 cm** nearer the scalp (`+x`) than that stage (`s_pin <= s_stage - 2.0 cm`).
  4. The comb is dropped (released outside the tray interior `12.4 x 22.4 cm` footprint or from `> 3.5 cm` above the table).
  5. The manikin's head is knocked or turned more than **6.0 deg** (`> 5.5 mm` collar witness-stripe offset) or **1.5 cm** from its initial pose.
- **Observable stage-clearance rule (snag-free stroke + knot check; harmless fibre mixing allowed):** Stage `C_k` (`s = 25-31 cm` for `C3`, `19-25 cm` for `C2`, `13-19 cm` for `C1`) is cleared when **(a)** no raised matted knot lump remains in its `6 cm` zone and **(b)** a continuous wide-tooth comb stroke entering at or above that zone passes smoothly out past the hair tips (`s = 38.0 cm`) with its teeth spanning the lock and without stalling, bunching hair ahead of the comb spine, or pulling the root scale past `2.0 N` (ordinary intermingling of loose smooth fibres or permutation from still-uncleared upstream stages does not penalize clearance). During the `0.75`/`1.00` unpinched scalp-to-tip verification stroke (`s <= 3.0 cm -> 38.0 cm`), neither gripper pinches the hair, so the entire lock pulls directly on the Ohaus `5 N` root scale (`<= 2.0 N` red band).
- **Partial and full credit** (mutually ordered tiers when no disqualifier fires):
  - **0.25:** Hair clamped (`>= half` lock width) `>= 2.0 cm` above `C3` while a comb stroke entering above `C3` (`s <= 28.5 cm`) clears distal stage `C3` (`s = 28 cm`) and passes smoothly out past the tips (`s = 38.0 cm`) without bunching.
  - **0.50:** Both `C3` and `C2` (`s = 22 cm`) cleared and a comb stroke entering at or above the blue midpoint mark (`s <= 21.0 cm`) passes smoothly out past the tips (`s = 38.0 cm`) without bunching, even if `C1` upstream remains matted.
  - **0.75:** All three stages (`C3, C2, C1`) cleared and one continuous unpinched verification comb stroke enters at or behind the green scalp mark (`s <= 3.0 cm` from the scalp slot, unobstructed by any downstream grip so the lock hangs directly on the `5 N` root scale `<= 2.0 N`) and passes smoothly out past the tips (`s = 38.0 cm`).
  - **1.00:** All `0.75` requirements met, the untangled hair lies extended on the chest towel (`z >= 4.0 cm` above table, `tips at x <= 0.12 m`, within `+-5.0 cm` of `y = 0`), the comb is released flat inside its tray (`within 26 deg` of horizontal, centre `z <= 2.2 cm` above table), and both grippers are open (`> 2.8 cm`), at least **10.0 cm** clear of the head, hair, comb and tray, and still (`< 2.0 cm/s`) for **0.5 s**.

## Simulation model and fidelity

- **Hair lock (`hair.py`):** 6 reduced XPBD strand bundles (20 nodes per bundle, `2.0 cm` segments, `8.0 g` total mass, `650 N/m` axial stretch stiffness, tension-only buckling, towel/shoulder contact and `mu = 0.22` Coulomb friction) standing in for the ~2,000 loose Kanekalon fibres, rendered every control step as a 6-bundle chestnut/auburn mesh with 3D snarl markers on active stages.
- **Crossings and calibration:** Each stage (`C1, C2, C3`) couples its paired bundles laterally and vertically (`260 N/m`) until resolved by `5.0 cm` of engaged comb-tooth travel (`3.2 N` combing drag, matching the `3.2 +- 0.5 N` dry Kanekalon bench calibration and published tangled wig-brushing loads of `2.5-4.5 N`). Entering above `>= 2` uncleared stages jams them (`> 6.5 N` over `1.8 cm` travel), detaching a root anchor. Bare gripper jaws clamp all bundles together and cannot advance stage resolution without the comb's 10 teeth, while any closed pinch downstream of where a comb stroke enters physically blocks the comb from passing.
- **Pinches and grasp-weld (sim approximations):** Closing a free gripper (`aperture <= 2.0 cm`) across `>= half` the lock width (`>= 3` of the 6 equal-width strand bundles between its finger pads) pins that hair cross-section to the pads until `aperture > 2.8 cm` or `5.0 cm` slip. As a simulation stand-in for elastomer pad friction on the comb's `2.2 x 0.9 cm` handle, a fixed grasp joint **engages only on two-pad contact** (`>= 0.02 N` measured PhysX normal contact force on both `tip_left` and `tip_right` simultaneously while the jaws are commanded closed `<= 1.6 cm` across opposite sides of the handle, `0.8 <= measured <= 2.7 cm`) and releases immediately when the jaws reopen (`> 3.0 cm`).
- **Root gauge and head:** Summed XPBD tension at the 6 scalp root tethers drives the live red peak-hold ring on the `5 N` spring scale (`20 mm/N`). The `4.0 kg` manikin head rides a PhysX revolute neck joint (`0.15 N m/deg` spring, `0.02 N m s/deg` damping). All geometry is procedural USD primitives (no external meshes or third-party asset licences).

## Requirements & Run a trial

`yam-isaac-lab-arena` on Isaac Lab-Arena 0.3.1 with Inspect Robots (`yam_arena` embodiment); `hair.py` ships in this folder.

```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 1200 \
  --instruction "Comb the tangle out of the lock of hair lying over her shoulder so the comb runs from scalp to tip without pulling on her scalp, then put the comb back on its tray."
```

Seeds are deterministic with `object_jitter=0`; a 1,200-step trial (60 s at 20 Hz) runs in ~2 minutes.

## Demo video

A scripted policy plans joint-space motions from live simulator object poses and strand positions and completes in **779 steps (39.0 s at 20 Hz)**, well within the 1,200-step limit. The right hand physically closes onto the wide-tooth comb's handle and pitches the teeth downward; the left hand pinches the hair above `C3`, steps upward above `C2`, and steps near the shoulder above `C1` while the right hand combs each stage out past the tips, keeping peak root tension under `0.4 N`. Shortcut policies (idle, combing one-handed without holding the hair, raking top-down across multiple stages at once, pulling with bare fingers without the comb, dropping the comb outside its tray, and bumping the head) all score `0.0`, while clearing only `C3` scores `0.25`, clearing `C3` and `C2` scores `0.50`, and omitting comb return scores `0.75`. After the left hand releases and clears, the right hand runs one continuous verification stroke from `2 cm` of the scalp past the tips, lays the comb flat in its tray, and both arms swing wide outward (`> 20 cm` clear) and hold still to finish at `1.0`. Scores: see `task.json`.

## Sources

- O*NET 31-1131.00 Nursing Assistants & 31-1122.00 Home Health and Personal Care Aides; BLS OEWS May 2023 national employment estimates: https://www.bls.gov/oes/
- OpenRN *Nursing Assistant*, Ch. 5 Personal Care Skills (hair care: part hair into sections, hold hair above the tangle so combing does not pull at the scalp, start at the ends and work upward): https://wtcs.pressbooks.pub/nurseassist/
- Goody Super Comb wide-tooth detangler (7.75 in / 19.6 cm); Ohaus 8003-MN 5 N pull-type spring scale; Diane D150 care-training mannequin head with shoulders.
