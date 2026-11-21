# Form a cucumber maki with a bamboo mat: tuck-press, peel the mat lip free, cinch shut, and unwrap

Instruction: `Use the bamboo mat to form the nori, rice, and cucumber into a straight, closed sushi roll without touching the food directly or rolling the mat inside it, then open the mat flat with the sealed roll resting seam-down on it.`

## Physical skill tested

Forming hosomaki (kappamaki) with a bamboo makisu tests **tool-sheet decoupling** between a reusable slatted outer
forming mat and an inner adhesive food sheet:

1. **Indirect handling through the mat:** Neither gripper may touch the sticky rice, nori, or cucumber directly; both
   hands manipulate the overhanging near corners of the bamboo mat (~19 cm apart) so the stiff lateral bamboo slats
   distribute pressure evenly without skewing.
2. **First compression bond (tuck press):** Both hands fold the mat forward over the cucumber baton so the nori's near
   edge reaches into the rice bed ahead of the baton, then cinch the mat downward (>= 1.4 mm radial squeeze held
   >= 0.15 s) to bond the inner tuck. Without downward compression, the elastic roasted nori springs back flat when the
   mat reopens.
3. **Peel-back anti-entrapment:** Because the bamboo mat is an external forming tool rather than a permanent wrapper,
   winding the mat continuously forward rolls the bamboo mat's near lip inside the sushi spiral between the inner rice
   turn and the outer nori. After tacking the tuck, both hands must **peel the mat's near lip upward clear of the front
   nip** before drawing the upper mat bight forward to roll the tucked cylinder onto the moistened 1.5 cm bare nori
   strip.
4. **Second compression bond (seam-seal cinch) and tool removal:** Drawing the mat taut ahead of the closed cylinder
   presses the moistened bare nori strip onto the lower half of the roll (seam-down), after which both hands lift and
   lay the bamboo mat flat open while leaving the straight, closed roll resting intact on the open mat.

## Scene and repeatable real-trial setup

Coordinates in cm from the midpoint between the two arm bases (`(0, +-30.5)` cm) on the station table top (`z = 74.75`
cm above floor); `x` forward, `y` left, heights above the table top.

| Object | Commercial item / specification | Dimensions (x x y x z) | Mass | Placement |
|---|---|---|---|---|
| Cutting board | Winco CBBU-1218 blue NSF HDPE cutting board | `30.5 x 45.7 x 1.27 cm` | `1.60 kg` | Centred at `(34.75, 0.0)`, spanning `x = 19.5..50.0 cm`, `y = -22.85..+22.85 cm` |
| Bamboo makisu | Joyce Chen J33-0022 `9.5 in` natural bamboo sushi mat (40 slats at `6 mm` pitch along `y` on 3 cotton twine rows along `x`), wrapped smoothly in 1 layer of food-grade PE cling film | `24.0 x 24.0 x 0.30 cm` | `85 g` | Centred on `y = 0` (`y = -12.0..+12.0 cm`), spanning `x = 18.0..42.0 cm` (`1.5 cm` overhang past the board's near edge at `x = 19.5 cm`) |
| Nori + sushi rice | Yamamotoyama Gold roasted half nori sheet (rough side up) with levelled `6.0 mm` layer of room-temperature (`22-26 C`) seasoned short-grain Calrose sushi rice over near `9.0 cm`; far `1.5 cm` strip bare and misted once (`~0.3 mL` water) | `10.5 x 19.0 x 0.60 cm` | `85 g` | On the mat at `x = 19.5..30.0 cm` (`1.5 cm` inside the mat's near edge), `y = -9.5..+9.5 cm`; rice on `x = 19.5..28.5 cm`, bare strip on `x = 28.5..30.0 cm` |
| Cucumber baton | Seedless English cucumber baton, cut square with a `10 mm` baton cutter, chilled (`4-8 C`) and blotted dry | `1.0 x 19.0 x 1.0 cm` | `19 g` | Across the middle of the rice bed at centre `x = 23.5 cm` (`x = 23.0..24.0 cm`, `4.0 cm` from nori near edge), `y = -9.5..+9.5 cm` |

### Shared-rig protection and reset protocol (< 2 min per trial)

- **Rig protection:** Fit replaceable food-safe disposable nitrile finger sleeves (or washable silicone finger boots)
  over the parallel gripper jaws and wipe them clean between trials. Wrapping the bamboo makisu in food-grade PE cling
  film is standard sushi-counter practice so rice never adheres to bamboo slats or cotton twine, and direct gripper
  contact with the food scores `0`.
- **Standardized food preparation:** Cook short-grain Calrose rice (`1 : 1.1` dry rice to water by weight), fold in
  seasoned rice vinegar (`15 mL` per `100 g` cooked rice), and hold covered in an insulated container at `22-26 C`.
  Between trials: wipe the cling-wrapped mat, align its near edge to the `x = 18.0 cm` mark (`1.5 cm` overhang), place a
  fresh `10.5 x 19.0 cm` half nori sheet at `x = 19.5 cm`, level `85 g` of rice inside a `9.0 x 19.0 x 0.6 cm` acrylic
  levelling frame over `x = 19.5..28.5 cm`, mist the bare `1.5 cm` strip (`x = 28.5..30.0 cm`) with one pump of water,
  and set a blotted `1.0 x 1.0 x 19.0 cm` cucumber baton at `x = 23.5 cm`.

## Run a trial

Requires `yam-isaac-lab-arena` on Isaac Lab-Arena 0.3.1. Run limit: **600 steps** (30 seconds at 20 Hz; layout fixed
with `object_jitter=0`).

```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 600 \
  --instruction "Use the bamboo mat to form the nori, rice, and cucumber into a straight, closed sushi roll without touching the food directly or rolling the mat inside it, then open the mat flat with the sealed roll resting seam-down on it."
```

## Scoring (matching `score()` in `task.py` and real-trial ruler + photo/video checks)

Definition of a **closed tube**: the rice-covered nori wraps `>= 330 deg` around the cucumber baton within an outer
diameter `<= 3.5 cm` (radius `<= 1.75 cm` from the core axis) across `>= 80%` of the interior slices.

- **Score `0.0`** if at any point a gripper directly touches the nori, rice, or cucumber without the bamboo mat between
  them (`< 4.5 mm`), or if the bamboo mat's near lip is rolled inside/under the sushi bundle (`mat_trapped`: advancing
  the tucked roll `> 1.2 cm` without peeling the mat lip upward clear of the nip, or trapping the near lip beneath the
  roll under an upper mat ply); or if at the end the roll/filling or mat lies off the cutting board, the board shifted
  `> 2.0 cm`, or the nori was not folded forward over the cucumber across `>= 80%` of the roll's width (all other
  unlisted failed states also score `0.0`).
- **Score `0.25`** if the nori's near edge was folded forward over the cucumber across `>= 80%` of the roll's width,
  but the Closed Tube condition above is not yet met.
- **Score `0.50`** if the Closed Tube condition holds, but the `1.5 cm` bare far strip is not sealed flush on the lower
  half (`<= 4.0 mm` from the inner turn and at or below core-axis height across `>= 80%` of the roll, checked in a real
  trial by the roll remaining closed seam-down without springing open once the mat is unwrapped), **or** the roll is
  skewed (`x` offset between `y = -7.1 cm` and `y = +7.1 cm` exceeds `1.5 cm`), **or** the bamboo mat is not laid flat
  open (`x` span `>= 20.0 cm`, max height `<= 2.0 cm` above the board) with the roll resting on it (`>= 1.5 cm` inside
  both its near and far edges).
- **Score `0.75`** if the closed, sealed, straight (`<= 1.5 cm` skew) roll rests seam-down on the open bamboo mat on
  the board, but the grippers have not yet released and parked clear.
- **Score `1.0`** when all `0.75` conditions hold AND nothing is held, both gripper jaws are open `>= 2.0 cm`, all
  gripper geometry is `>= 10.0 cm` clear of the mat and roll, and both grippers and the roll move at `< 2.0 cm/s` for
  at least `0.5 s`.

## Simulation model and fidelity

- **Cutting board:** Free `1.60 kg` rigid box (`Winco CBBU-1218` dimensions).
- **Bamboo makisu:** `41 x 11` position-based grid (`6 mm` slat pitch along `x`, `11` stations across `y`), inextensible
  cotton twine along `x` with soft bending, stiff straight-rod projection along each bamboo slat (`y`), weak diagonal
  shear, board contact with friction, and gripper collision-box pushing. A contact-triggered pinch holds up to 6 mat
  nodes when a jaw closes (`< 15 mm`) around them and releases when the jaw opens (`> 24 mm`) or slips (`> 4.2 cm`).
- **Cucumber and nori + rice:** Rigid 9-slice cucumber baton; reduced elastic nori + rice sheet. Both the inner rice
  tuck and the misted `1.5 cm` bare-nori outer seam require a genuine front-cinched radial squeeze (`>= 1.4 mm` mat
  compression with the front lip drawn down across the front cheek for `>= 0.15 s`) to bond; unpressed folds or seams
  spring back open as soon as the mat lifts off them. Individual loose rice grains are not simulated.
- **Demo video:** A scripted policy plans joint-space motions from simulator state (pinching both corners of the
  overhanging mat, folding and pressing the tuck, peeling the mat lip upward clear of the nip, traction-rolling onto the
  bare strip, cinching the seam shut, unwrapping the mat flat, and parking both arms clear). Scripted ablation policies
  verify that idle, direct food contact, continuous rolling without peeling the mat lip (`mat_trapped`), unpressed
  draping, single-arm corner rolling, partial folding, leaving the mat wrapped, and holding at the end all score below
  `1.0`.

## Assets and licences

All scene geometry (cutting board, bamboo slats, cotton twine, nori sheet, sushi rice layer, and cucumber baton) is
generated procedurally in Python/USD with no external mesh, texture, or third-party asset files.
