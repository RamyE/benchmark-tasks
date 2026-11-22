# Shuck an ear of sweet corn: peel the husk back in two halves, then snap it off with the stalk end

Instruction: `Shuck the ear of corn and leave the bare cob on the board and the husk in the bowl.`

A fresh ear of yellow sweet corn lies in its green husk on an acacia cutting board beside a stainless steel scrap bowl.
The robot steadies the ear, peels the husk and silk tuft back past the cob's base in two halves, holds the exposed cob
gently without rupturing tender milk-stage kernels while bending and snapping the stalk stub (shank) off with the
folded-back husk bundle, drops the husk and shank into the bowl, lays the bare cob on the board, and parks both open
grippers clear.

## Why it matters and why it needs two hands

Shucking sweet corn at the counter, sink or picnic table is an immediately recognizable household food-preparation
chore. Because the husk leaves grow concentrically from the tough nodal ring on the shank and close tightly over the
ear's tip, pulling a husk tip with one hand alone drags and spins the 0.34 kg ear across the cutting board (~0.7 N board
sliding resistance versus ~2 N husk-peeling pull). One hand must hold the ear steady by its shank while the other grasps
each flag tip and peels that half all the way past the cob's base; then one hand must hold the bare kernelled cob gently
(milk-stage sweet-corn pericarps rupture above ~35 N of pad clamping force) while the other bends the shank (~3.07 N m
at 32 deg) until it snaps free carrying the creased husk bundle.

## Real-world setup and repeatable sourcing

Coordinates in cm from the table top directly under the arm-base midpoint (`x` forward, `y` left, `z` height above the
table top; arm bases at `x = 0, y = +-30.5 cm`).

| Item | Product / standard | Dimensions & mass | Placement |
|---|---|---|---|
| Cutting board | OXO Good Grips Acacia Wood Utility Cutting Board (SKU 11443000), clamped/non-slip on table | 27.0 (`x`) x 37.6 (`y`) x 2.8 (`z`) cm | Centre `(27.0, 11.0)` (`x: 13.5..40.5`, `y: -7.8..29.8`); white 4.4 x 0.5 cm tape lines at `x = 27.0`, `y = 0.0` (shank butt) and `y = 23.0` (cob tip) |
| Scrap bowl | Winco MXB-500Q 5-Quart Stainless Steel Mixing Bowl | 29.2 cm outer rim dia, 27.0 cm inner opening dia, 8.9 cm tall, flat base | Centre `(29.0, -26.5)` (`y: -41.1..-11.9`, 4.1 cm clear of board) |
| Sweet corn ear | USDA U.S. Fancy fresh yellow sweet corn in the husk (7 CFR 51.835 & 51.843), kept at 1-4 C (34-40 F) | 28.5 +- 2.0 cm overall, 0.34 +- 0.05 kg: shank 4.5 +- 0.8 cm x 2.4 +- 0.3 cm dia (0.03 kg); kernelled cob 18.5 +- 1.5 cm x 4.4 +- 0.4 cm dia (0.25 kg); 6-8 green husk leaves + silk tuft extending 5.5 +- 1.5 cm past cob tip (0.06 kg) | Along `x = 27.0`: shank butt at `y = 0.0` (`-y`, toward right arm), cob `y = 4.5..23.0`, flag tips parted into `+x` (far) and `-x` (near) halves ~3.8 cm apart at `y = 23.0..28.5` (`+y`, toward left arm) |
| Gripper covers | Disposable food-grade textured nitrile finger sleeves on both parallel grippers | Thin textured nitrile over ~20 x 30 mm finger pad contact faces | Fitted before trials and wiped/replaced between runs to keep corn milk, moisture and silk off shared grippers |

### Reset protocol

Select refrigerated (1-4 C) USDA U.S. Fancy ears within the table tolerances above. Strip any loose torn wrapper leaf,
trim the shank to 4.5 cm below the husk base, part the protruding flag leaves cleanly into two symmetric lateral bundles
(`+x` and `-x`, ~3.8 cm apart) enclosing the silk tuft, and lay the ear along `x = 27.0 cm` between the `y = 0.0` and
`y = 23.0 cm` white alignment marks with the shank toward `-y`. Empty the bowl and wipe or replace the nitrile finger
sleeves.

## Scoring

Evaluated every control step for latched disqualifiers and at the end of the 1,200-step (60 s at 20 Hz) limit for
milestones. Precedence: any latched disqualifier returns `0.0`; otherwise the highest milestone whose conditions all
hold is awarded (`0.0` if none).

- **Score 0.0 (latched throughout the trial)** if:
  1. the cob falls off the table or touches the table off the cutting board (`cob` centre outside the `27.0 x 37.6 cm`
     board while within `3.7 cm` of the table top, or below `table + 1.5 cm`), or any husk/shank piece falls off the
     table; or
  2. any kernel on the exposed cob is crushed: in a real trial, **1 or more kernels with a split pericarp, expressed
     milky juice, or a dent > 2 mm**; in simulation, clamping the exposed cob with **> 35 N** pad force for >= 2 steps.
- **0.25**: one husk half peeled clear of the cob (before snap: base hinge >= 100 deg and all 3 of its shell segments
  behind `cob_base + 2.0 cm` along the cob axis; after snap: all 3 segments >= 3.0 cm clear of the cob surface),
  exposing >= 50% of the cob circumference along its full length.
- **0.50**: both husk halves (and their attached silk tufts) peeled clear behind `cob_base + 2.0 cm`, exposing 100% of
  the kernelled cob.
- **0.75**: shank and all husk segments detached >= 3.0 cm clear of the cob, and the bare cob resting level (axis tilt
  < 14.5 deg, speed < 3 cm/s) on the cutting board with both cob ends at least 1.0 cm inside the board edges. In a real
  trial, "bare cob" means zero husk leaves and no silk tuft left at the tip (up to 5 loose single silk hairs clinging
  between kernel rows are ignored).
- **1.0**: all `0.75` conditions hold, the detached shank and every husk segment (including flag tips and silk tufts,
  checked across all bounding points, speed < 5 cm/s) lie inside the bowl's 27.0 cm inner opening below the 8.9 cm rim,
  and both grippers are open (aperture >= 5.0 cm) and parked (every gripper/finger point >= 10.0 cm from all ear/husk
  parts and from the bowl, TCP speed and 1.0 s drift <= 2.0 cm/s, held continuously for >= 1.0 s; `SUCCESS_HOLD_S = 2.0`
  holds the finished state for an additional 2.0 s so the video clearly shows >= 3.0 s parked).

## Requirements and running a trial

Requires `yam-isaac-lab-arena` on Isaac Lab-Arena 0.3.1 (`inspect-robots` with the `yam_arena` embodiment).

```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 1200 \
  --instruction "Shuck the ear of corn and leave the bare cob on the board and the husk in the bowl."
```

## Simulation model and fidelity

- **Force- and torque-limited pad grip.** Husk and flag grasps use PhysX contact and friction (mu 1.50-1.65). The cob and shank use a simulated stand-in for pad friction: a compliant spring-damper engages only on two-pad PhysX contact and commanded squeezing, anchored at the current pose. For the shank, caps are 50 N pull (two pads, mu 0.5, 50 N squeeze), 0.50 N m twist, and 1.50 N m tipping. For the exposed cob, caps use a conservative 25 N squeeze: 25 N pull, 0.25 N m twist and 0.75 N m tipping. These use the YAM gripper effort limit and a 20 x 30 mm pad patch. It slips above a cap and releases on opening or loss of two-pad contact for five steps. The pad collisions also act directly on the corn. The ~3.07 N m snap is not carried through the friction caps: in the recorded run both capped grips stayed at <= 6.3 N load and logged 17 slips, while the rigid jaw pads pushed the stub sideways by direct contact (as real fingers do).
- **Cob and crush check.** Rigid 18.5 x 4.4 cm cylinder (0.250 kg) with longitudinal kernel ridges. Direct pad clamping
  force on the exposed cob is computed each step from jaw compression and gripper stiffness (`5,000 N/m`, `50 N` max);
  exceeding `35 N` (~6 N/kernel across a ~6-kernel pad patch) for >= 2 steps latches `crushed` and marks a bruised patch
  on the cob, whereas a gentle ~22-26 N hold keeps kernels intact during shank snapping.
- **Husk and silk.** Two lateral halves (0.060 kg total), each 3 curved rigid shell segments joined by spring-damped
  Z-hinges (`0.70 N m/rad`, `38 deg` curl range), carrying an inward tip shoulder (`3 mm` past the cob tip), a
  10 mm gathered flag-leaf tab, and 5 solid colliding silk-tuft strands (`4.6 cm`); stray single silk threads are not
  modelled (aligned with the real-trial rule ignoring up to 5 loose single silk hairs). Base hinges on the shank have
  initial wrap retention (`0.45 N m/rad` to 18 deg), inelastic peel set (`0.28 N m/rad`), and crease memory past 90 deg
  (`0.48 N m/rad` toward 140 deg folded back, with mid-hinges gathering 36 deg inward once snapped free). Base and mid
  shell segments filter cob-lip overlap to stand in for transverse leaf flex, while the tip shoulders collide with the
  cob so an unpeeled husk cannot slide off over the tip.
- **Shank snap.** Rigid 4.5 x 2.4 cm stalk stub (0.030 kg total with internal gimbal) joined to the cob base by a
  2-axis (`X`/`Z`) revolute bend spring (`5.5 N m/rad`, `~3.07 N m` at `32 deg`) that disables permanently once bent
  past `32 deg`.

## Demo video

A scripted policy plans joint trajectories from live object poses without modifying simulator state. Its intended sequence is: the right hand
lifts and steadies the ear by the shank at one angle while the left hand peels the far
(`+x`) and near (`-x`) husk halves past the cob base in smooth continuous arcs; the left hand then holds the bare cob
gently (peak 24.7 N in the recorded run, < 35 N crush limit) while the right hand bends and snaps the shank horizontally past 32 deg
(~3.07 N m), centres the detached shank and creased husk bundle over the mixing bowl and drops it inside, lays the cob
flat on the board, and parks both wide-open grippers well clear for ~3.0 s. The recorded scripted demo scores 1.0 in 858 steps. Idle and one-arm shortcuts score 0.0, omitting the snap scores 0.5, and crushing the cob scores 0.0.

## Sources

- USDA Agricultural Marketing Service, *United States Standards for Grades of Sweet Corn*, 7 CFR 51.835-51.843 (U.S.
  Fancy cob >= 6 in / 15.2 cm, fresh green husks, trimmed shank <= 2 in / 5.1 cm):
  https://www.ecfr.gov/current/title-7/subtitle-B/chapter-I/subchapter-C/part-51/subpart-United-States-Standards-for-Grades-of-Sweet-Corn
- OXO Good Grips Acacia Wood Utility Cutting Board, SKU 11443000 (14.8 x 10.65 x 1.1 in / 37.6 x 27.0 x 2.8 cm):
  https://www.oxo.com/
- Winco MXB-500Q 5-Quart Stainless Steel Mixing Bowl (11.5 in / 29.2 cm dia, 3.5 in / 8.9 cm deep):
  https://www.wincous.com/
