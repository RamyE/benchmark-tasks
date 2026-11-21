# Peel one soda can free of its tight six-pack rings and stand it on a coaster, holding the pack down

Instruction: `Take one can of soda out of the six-pack and stand it on the coaster.`

A six-pack of unopened 12 fl oz aluminium soda cans stands on the kitchen counter inside its flexible LDPE six-ring
carrier, with a round cork coaster nearby. To serve one can without dragging the 2.3 kg pack across the counter or
tearing the carrier, one hand holds the pack down while the other grips a can around its upper wall and top seam, tips
it outward so the stretched plastic ring peels cleanly over the top double-seam rim, lifts the freed can clear, stands
it upright on the coaster, and parks both arms away. The remaining five cans stay upright in their intact rings.

## What makes it hard

- **Hold down while peeling.** Each LDPE ring has a `4.40 cm` unstretched inner opening pre-stretched around its can's
  `5.24 cm` neck groove beneath the `5.48 cm` top seam rim. Stretching a ring over that rim requires outward peel
  tension exceeding the `~5.3-6.6 N` counter sliding resistance of the other five cans, so pulling with one arm alone
  drags the pack across the counter unless the second hand holds the pack down (`~15-20 cm` away).
- **Peel by tipping and stretching, not yanking.** Pulling straight up catches the ring all round under the `5.48 cm`
  chime overhang and lifts neighbours or tears the carrier. Yanking sideways without tipping (`>= 18 deg`) tears the
  `0.38 mm` LDPE web instead of peeling the ring over the rim.
- **Keep the carrier and five cans intact.** Tearing any ring or inter-ring web, popping a second can loose, or
  tipping, lifting, or dragging any remaining ringed can scores `0`.

## Scene and reproducible setup

Origin on the counter surface directly below the midpoint between the two arm bases (`x` forward, `y` left in cm;
heights `z` above the counter top). Arm bases sit at `(0, +30.5)` and `(0, -30.5)` cm, `0.25` cm above the counter.

| Item | Real product / specification | Dimensions & mass | Placement & orientation |
|---|---|---|---|
| Counter | Station laminate countertop | `120.0 x 80.0 cm` flat rigid top at `z = 74.75 cm` above floor | Fixed horizontal work surface |
| Six soda cans | Standard US 12 fl oz (355 mL) sleek-neck aluminium beverage cans (CMI `211 x 413` body with `202` end, e.g. Coca-Cola Classic 12 fl oz) | Body `6.60 cm` dia (`0-10.60 cm`), tapered shoulder `5.96 cm` dia (`10.60-11.30 cm`), neck groove `5.24 cm` dia (`11.30-11.90 cm`), `202` top double-seam rim `5.48 cm` dia (`11.90-12.22 cm`), `383 g` each (`2.30 kg` pack) | Upright `2 x 3` grid centred at `(34.0, +4.0)` cm, `6.70 cm` centre pitch (`13.30 x 20.00 cm` pack): rows `x = 30.65, 37.35` cm; cols `y = -2.70, +4.00, +10.70` cm; stay-on pull-tabs pointing `+x`. Any one of the six cans may be removed |
| Ring carrier | Commercial Hi-Cone `202`-end `2 x 3` LDPE six-pack ring carrier (`0.38 mm` / `15 mil` low-density polyethylene sheet, `~4.5 g`) | Unstretched flat sheet (`~12.6 x 19.4 cm`): six circular apertures of `4.40 cm` inner dia and `0.80 cm` band width (`6.00 cm` outer ring dia), joined by seven `1.00 cm` wide inter-ring webs (four along `y`, three along `x`) forming two central finger holes | Pre-stretched onto the six cans' `5.24 cm` neck grooves at `z = 11.60 cm`, directly beneath each `5.48 cm` top double-seam rim |
| Pack outline | `2.2 mm` wide dark grey tape/printed rectangle on counter | `13.70 x 20.40 cm` (`2 mm` clearance round the `2 x 3` pack) | Centred at `(34.0, +4.0)` cm for repeatable reset |
| Cork coaster | IKEA 365+ round natural cork coaster | `10.00 cm` diameter, `0.50 cm` thick | Fixed on counter, centred at `(20.5, -10.5)` cm |

### Reset between trials

1. Inspect the LDPE six-pack carrier closely; if any ring or inter-ring web has a visible tear, cut, or permanent necked
   stretch that no longer grips under a can's `5.48 cm` seam, replace it with a fresh commercial `202`-end LDPE carrier
   (about `$0.15` bulk or `$4` for a retail six-pack).
2. Snap all six unopened `383 g` cans into the six rings so every ring seats evenly in the `5.24 cm` neck groove under
   the `5.48 cm` top seam, turn each can's pull-tab forward (`+x`), and stand the `2 x 3` pack upright inside the
   `13.70 x 20.40 cm` outline mark centred at `(34.0, +4.0)` cm. Verify the cork coaster is centred at `(20.5, -10.5)` cm.

## Scoring

Score-0 disqualifiers are checked continuously over the whole trial (latched if triggered at any time; in a real trial,
checked from the trial video, ruler measurements against the counter outline, and post-trial visual inspection of the
carrier). When no disqualifier fires, the trial receives the **highest tier satisfied at the end of the episode**
(`SCORE_OVER = "end"`, `SUCCESS_HOLD_S = 1.0`):

- **Score 0** if at any time:
  - any ring or inter-ring web of the LDPE carrier is visibly torn or severed (in simulation: ring tension reaches
    `>= 28.0 N` before peel release);
  - more than one can's ring comes off its neck/rim;
  - any remaining ringed can (any can that stays in its ring rather than being freed) tips `> 20.0 deg` from vertical,
    lifts its base `> 1.5 cm` above the counter, or slides its centre `> 3.0 cm` horizontally from its starting mark
    (`3.0 cm` is more than twice the `~1.2 cm` elastic shift during a held peel, catching unheld pulls that drag the
    pack across the counter); or
  - a freed can tips `> 45.0 deg` from vertical while unheld (dropped or knocked over).
- **0.3**: Exactly one can's ring is off its rim, and the other five cans remain in their rings.
- **0.6**: That freed can is horizontally `>= 2.0 cm` clear (wall-to-wall and wall-to-ring) of the other five cans and
  the carrier, and all five remaining cans stand upright in their rings (`tilt <= 10.0 deg`, base within `0.5 cm` of
  the counter).
- **1.0**: All `0.6` conditions hold, the freed can stands released and at rest (linear speed `<= 2.0 cm/s`, angular
  speed `<= 0.20 rad/s`, `tilt <= 10.0 deg`) on the cork coaster (base within `0.5 cm` vertically of the coaster top,
  base centre within `1.7 cm` horizontally of `(20.5, -10.5)` cm so the `6.60 cm` base rests wholly inside the `10.00 cm`
  coaster), nothing is held, and both grippers (TCP and finger pads) remain `>= 10.0 cm` from all cans, the carrier, and
  the coaster while moving at `<= 2.0 cm/s` continuously for `1.0 s`.

## Simulation model and fidelity

- **Cans, coaster, and pure contact grasping:** Six free rigid bodies (`383 g` each; lower aluminium-on-laminate
  counter friction `0.35` static / `0.28` dynamic with `min` combine mode; upper `3.6 cm` grip zone, `5.96 cm`
  shoulder, `5.24 cm` neck groove, and `5.48 cm` `202` double-seam rim colliders with rubber-pad-on-aluminium friction
  `1.15` static / `1.00` dynamic, `max` combine mode; rigid-body damping `1.0 /s` linear, `4.0 /s` angular) plus visual
  lid panel, `+x` pull-tab, and white label band. Coaster: fixed rigid cylinder (`0.75 / 0.65` cork friction, `max`
  combine mode). Both YAM parallel grippers (`50 N` jaw clamp limit) grasp, hold down, and peel cans strictly through
  `240 Hz` PhysX rigid-body contact and finger-pad friction—no `FixedJoint` or grasp-weld attachment is used. No
  third-party mesh assets are used (all procedural USD primitives and meshes).
- **Calibrated commercial LDPE carrier, peel, and tear model:** Commercial `0.38 mm` (`15 mil`) Hi-Cone LDPE sheet
  (`E ~ 220-280 MPa`, tensile yield/break `~10-13 MPa` across `8-10 mm x 0.38 mm` bands `-> ~30-40 N` tensile break) is
  pre-stretched `19%` from its `4.40 cm` aperture onto each `5.24 cm` neck under the `5.48 cm` (`1.2 mm` radial) seam
  overhang. At every `240 Hz` physics substep, the `4.5 g` sheet solves its stiffness-weighted horizontal force-and-yaw
  equilibrium across the engaged cans' necks (`z = 11.60 cm`), applying calibrated 3D restoring forces (`950 N/m` taut
  web + upright seated ring stiffness for `tilt <= 4 deg`, blending linearly to `440 N/m` local `8 mm` ring-band stretch
  by `tilt >= 14 deg` as the outer arc rides onto the bevelled seam ramp, `550 N/m` axial chime retention, and
  `14 N s/m` internal relative carrier damping, capped at `32 N`) and driving a live deformable 3D USD mesh (`six
  4.40 cm` relaxed holes pre-stretched to `5.28 cm` on the necks, `8 mm` bands, seven `10 mm` webs, visual-only mesh). A
  ring peels free when its can tilts `>= 18.0 deg` relative to the carrier while stretching the ring `>= 32.0 mm`
  radially (`>= 14.1 N` peel tension, matching hand-peel load on a commercial `202` ring and exceeding twice the
  `~5.3-6.6 N` counter slip resistance of five unheld `383 g` cans) and visually snaps back to its `4.40 cm` aperture.
  Any ring load `>= 28.0 N` latches a carrier tear (`Score 0`, visually splitting the web), conservatively matching the
  onset of necking/tear in an `8-10 mm x 0.38 mm` LDPE band.

## Run a trial

Requirements: `yam-isaac-lab-arena` on `Isaac Lab-Arena 0.3.1` (`inspect-robots` with the `yam_arena` embodiment).
Layout is deterministic with `-E object_jitter=0` (any seed produces the nominal scene; `VARIATION` in `task.py`
defines optional perturbation ranges). Typical runtime: ~30-60 seconds for a 600-step trial.

```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 600 \
  --instruction "Take one can of soda out of the six-pack and stand it on the coaster."
```

## Demo

A scripted policy plans joint-space motions from measured simulator object poses: the left arm clamps the far-left can's
upper body and top seam to hold the six-pack steady on its outline while the right arm clamps the near-right corner
can's upper body and seam, pivots it outward around its outer base chime past `18 deg` until its stretched LDPE ring
peels cleanly over the `5.48 cm` rim, lifts the freed can clear, levels and sets it upright on the cork coaster,
releases both cans cleanly, and parks both grippers more than `15 cm` clear for over `1 s`, scoring `1.0` in ~353 steps
(within the 600-step / 30 s limit).
Scripted shortcut policies—idle (`0.0`), one-arm peel without holding the pack down (`0.0`, drags the five cans past
`3.0 cm`), straight vertical pull without tipping (`0.0`, lifts ringed cans past `1.5 cm`), sideways yank without
tipping (`0.0`, drags ringed cans past `3.0 cm` / tears the carrier), stopping after lifting clear (`0.6`), keeping hold
on the coaster (`0.6`), and leaving a hand hovering near the coaster (`0.6`)—confirm that only the coordinated hold-down
peel and clean parked placement achieves `1.0`.
