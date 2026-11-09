# Burton a crate over the hatch coaming on two married falls

A ship's-hatch mock-up on the table. A small orange crate sits on the grey hold pad on one side of a grey coaming
wall; the green pier pad is on the other side. The crate hangs from two cargo falls shackled to its lifting eye: a
blue fall that runs up over a head sheave on the hold side, and a yellow fall that runs up over a head sheave on the
pier side. Each fall drops from its head sheave to a heel sheave near the deck, passes under it, and runs along the
deck towards the robot to a red toggle. This is the burton (married falls, yard-and-stay, union purchase) rig of
conventional cargo gear, at toy scale: one fall plumbs the hatch, the other plumbs the pier.

The robot must move the crate to the green pad by the falls alone: haul both a little to lift it, then haul the
yellow while easing the blue in step, so the crate travels sideways and up over the coaming, then ease both to land
it, and let go. Hands stay out of the hatch square and off the crate.

The tested ability is coordinating two lines that share one load: the crate's position is set by the two hook-side
lengths together, so the hands must change them at a ratio (haul one, ease the other) that keeps the crate's path
above the coaming. Neither line alone can do it:
- the crate hangs from both falls at once; let either go and it swings or drops under the other sheave;
- hauling one fall alone drags the crate across the deck into the coaming and over its top (the pull from a head
  sheave 30 cm up and 30 cm away is mostly sideways), which the scoring rejects;
- the falls end 46 cm apart and only meet at the crate (inside the hatch square), so one gripper cannot take both.

## How this differs from similar tasks

- Lift a tube level on two hand lines onto its stands (our tandem-lift task, the nearest): there each hand holds a
  short line from one end of a long tube and both lift and carry it together; the lines are just handles of fixed
  length, and the skill is keeping the tube level while the arms carry it. Here the hands never carry anything: the
  load hangs from two long lines that run over fixed sheaves far from the hands, both lines meet at one point on the
  load, and the load is moved sideways, over an obstacle, purely by changing the two line lengths in opposite
  directions (haul one, ease the other). The end state is a lateral transfer of a suspended load across a wall,
  not a level placement.
- Lower a hanging load into a slot using a tag line (ours): one lifting line held at the hook plus a tag line to turn
  the load. Here there is no hook to hold and no turning: two falls, both over sheaves, set the position.
- Plumb a guyed mast (ours): two guy lengths set and left; the load does not move between two regions.
- Board tasks with a lifted load lift a rigid object directly or by a single line; none moves a load between two
  regions by the length difference of two converging lines. No public set (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR,
  DataCenterBench, RoboDojo) has a two-fall cargo transfer.

## Scene (station table; cm from the midpoint between the arm bases, x forward, y left, heights above the table)

| object | type | size | material (real rebuild) | position |
|---|---|---|---|---|
| hatch square | fixed mat | 22 x 34 x 0.2 cm | dark grey board | x 27-49, y -17 to +17 |
| coaming | fixed wall | 16 x 3 x 7 cm | grey painted wood/PLA, smooth | on the mat, x 30-46, y -1.5 to +1.5 |
| hold pad | fixed plate | 8 x 8 x 0.3 cm | grey PLA | centre (38, +10) |
| pier pad | fixed plate | 8 x 8 x 0.3 cm | green PLA | centre (38, -10) |
| crate | free box, 80 g | 4 x 4 x 4 cm | orange PLA box with a lifting eye on the lid | on the hold pad, centred |
| head sheaves | 2 fixed wheels, axle along x | 5 cm groove diameter, 12 mm groove, 8.4 cm cheeks | nylon sheave on an M5 axle | centres 30 cm up at (38, +20) and (38, -20) |
| boom posts | 2 fixed round posts | 2 cm diameter, 30 cm tall | dark painted dowel; the sheave axle is cantilevered 6 cm from it | (44, +20) and (44, -20), behind the sheaves |
| heel sheaves | 2 fixed wheels, axle along y | same sheave | nylon | centres 8 cm up at (34.9, +23.1) and (34.9, -23.1) |
| heel blocks | 2 fixed blocks | 3 x 1.2 x 9.2 cm | dark PLA, carry the heel axles | (34.9, +27.9) and (34.9, -27.9) |
| blue fall | rope | 6 mm, 86 cm, 22 g | blue braided cord | shackled to the crate's eye, up over the +y head sheave, down to and under the +y heel sheave, along the deck towards the robot at y +23.1 |
| yellow fall | rope | 6 mm, 96 cm, 24 g | yellow braided cord | the same on the pier side (y -20 head sheave, y -23.1 run) |
| toggles | one on each fall's end, 6 g | 2.4 x 2 x 1.2 cm | red PLA | on the deck at about x 11, y +-23 |

At the start both falls hang just taut from the crate to their head sheaves (blue hook side 27 cm from the eye to the
sheave axle, yellow 39 cm), so the crate rests on the hold pad.

## Sources (each opened; quotes verbatim)

- OSHA 29 CFR 1918 Appendix III, "The Mechanics of Conventional Cargo Gear",
  https://www.osha.gov/laws-regs/regulations/standardnumber/1918/1918AppIII: "It is important to understand that under
  the Burton System of cargo handling (conventional gear consisting of two cargo derricks with married falls), the
  midships or up-and-down boom should be spotted as close to the fore and aft centerline of the hatch as operationally
  possible." and "While under a load, the cargo falls (running rigging) must not be permitted to chafe on any standing
  or other running gear." and "Special attention must be paid to ensure that cargo runners work freely through the heel
  block, without chafing the cheek of the block."
- US Navy shiploading picture dictionary, https://ibiblio.org/hyperwar/USN/ref/Shiploading/index.html: MARRY "To join
  any two objects, usually falls. The Married Fall system consists of handling cargo by two connected falls. Also
  called Yard and Stay method, or Union Purchase method."; BOOM "1. HATCH BOOM -- The boom spotted over the hatch, also
  called the inboard boom, amidship boom, or up-and-down boom." and "2. OUTBOARD BOOM -- The boom spotted over the side
  of the ship, also called the burton boom or yard boom."; FALL "a. The Burton fall, also called outboard fall, is the
  cargo fall suspended over the side of the ship." and "b. The hatch fall, also called the up and down fall, or inboard
  fall, is the fall suspended over the hatch."
- O*NET 49-9096.00 Riggers, https://www.onetonline.org/link/summary/49-9096.00: "Tilt, dip, and turn suspended loads to
  maneuver over, under, or around obstacles, using multi-point suspension techniques." and "Manipulate rigging lines,
  hoists, and pulling gear to move or support materials, such as heavy equipment, ships, or theatrical sets."

What the sim takes from them: a hatch fall plumbing the hatch (here the blue fall's head sheave over the hold side)
and a burton fall plumbing the pier (the yellow), both joined at the load; the falls lead from the head blocks through
heel blocks to the hands (the winches of a real rig); the load is moved over the coaming by working both falls at
once. The opened pages name the rig and its parts but do not spell out the haul-and-ease sequence in a quotable
sentence; it follows from the geometry (the load hangs where both falls meet).

## Why two arms (simultaneous)

The crate hangs on both falls whenever it is off the deck, so both falls must be held, and their lengths changed in
opposite directions, for the whole transfer. Evidence from runs of `demo_script` variants (table below):
- `onearm_yellow` (right arm hauls yellow, blue left alone): score 0. With the blue fall left lying, hauling the yellow drags the crate across the deck into the coaming (contact latched at y +4 cm), then it is pulled up over it and falls off the far side (0.70 m/s); it ends on the deck at y -17 cm.
- `onearm_blue` (left arm hauls blue alone): score 0. Hauling the blue alone lifts the crate towards the hold-side sheave; it never crosses the coaming, and it drops (0.57 m/s down) back onto the hold side at y +16 cm.
- `drop` (blue let go halfway, while loaded): score 0. When the left hand lets go of the blue halfway over, the crate swings down on the yellow into the coaming (contact at y -2.7 cm, 9.4 cm up) and ends on the deck at y -16.5 cm.
- `gather` (one hand takes both falls): score 0. Carrying the yellow across to the blue side hauls it with the blue still slack: the crate is dragged into the coaming (latched at y +4 cm, 2.6 cm up) and falls off the far side to y -13 cm before the hand reaches the blue; the hand must open to pick the blue up, so it never holds both.
- Nothing in the scene can hold a fall for a hand: the sheaves, posts, heel blocks and coaming are smooth (friction
  0.1), so a round turn on any of them holds about e^(0.1 x 2 pi) = 1.9 times the drag of a tail lying on the deck
  (about 0.05 N), far below the 0.4-0.7 N each fall carries; the toggles are smaller than nothing they could jam in
  within the working lengths. A hand-free fall therefore runs.
- One fixed fall cannot land it either: with the blue held at any fixed length, hauling the yellow swings the crate
  on a circle round the hold-side sheave axle; at the start length (27 cm from the eye to the axle) that circle puts
  the crate's bottom 5 cm below the coaming top where the crate meets the coaming, and any length short enough to clear
  the coaming (23 cm or less) never reaches down to the deck (the eye starts 25 cm below the axle), so the crate
  cannot be set down on the pier side without easing the blue.

Offline IK (gripper pointing down, `scratch/ik_run.py`, output `scratch/ik_run.out`): each arm reaches its own fall's
hauling run at 6.2 cm height from x 6 to x 30 cm (y +-23.1) with 0.0-0.1 mm error. The task is not reach-forced:
both runs are within the other arm's reach too; the two-arm need comes from the simultaneous hold.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (rope simulation, the reset that lays the falls
  over the sheaves, scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 900 \
    --instruction "This crate has to go from the hold side to the pier: use the two cargo falls to lift it over the coaming and land it on the green pad, keeping your hands off the load and out of the dark hatch square. Then let go of both falls."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 900
```

Closed loop on the live crate. Each arm pinches its own fall where it runs along the deck towards the robot (left:
blue at x 15 cm; right: yellow at x 29 cm, just past its heel sheave), fingers across the rope, and holds it 6 cm up
in line with the heel sheave. A target path for the crate's lifting eye is then followed in 3 mm steps: up 4 cm off
the hold pad, over the coaming with the eye at 14.5-15.5 cm (crate bottom about 8-9 cm, 1-2 cm above the coaming top),
down to the pier pad. Every 0.1 s each hand hauls or eases its fall (moving along the run, at most 6 mm per step) by
0.6 times the error between the eye's measured distance from that fall's head sheave and the target's: the right
hand hauls the yellow about 23 cm in all while the left hauls the blue 9 cm for the lift and then eases it about 20
cm. After the crate lands both ease their falls slack, wait for it to be still, open, lift 4 cm straight up and sweep back
beside their own bases, out of the top camera's view.

Result (`demo/result.json`, recorded with all three cameras): score 1.0, success at step 388 (19.4 s; run limit
900). Crate centre 0.7 cm from the pier pad's centre, tilt 0 deg; peak speed 0.17 m/s (0.11 m/s vertical); no coaming
contact (the 3 mm latch never set). End frame `scratch/end.png`: the crate sits on the green pad, both falls
lie slack from their sheaves to the deck, both arms are back beside their bases. Wrist cameras stay bright (mean
luma 87-171 per second, except 2 s at 16-18 on the right while it passes over the heel block).

## Scoring (`score()`, evaluated at the end; `success()` = score 1.0 with the crate still, held 1 s)

Score 0 if any of these held at any time (latched for the trial):
- the crate came within 3 mm of the coaming (dragged against it, or set on it);
- the crate moved faster than 0.6 m/s, or faster than 0.4 m/s up or down (dropped, jerked, or swung loose after a
  fall was let go);
- the crate's lid came within 8 cm of a head sheave's axle (two-blocked);
- a robot part touched the crate, or a gripper (TCP or pad tip) came within 3 cm of it or entered the hatch square;
- one gripper was within 2.5 cm of both falls at once;
- a state went non-finite.

Also 0 unless at the end both grippers are at least 4 cm from the crate and from both falls (let go), and the crate
rests on the deck (lowest corner within 6 mm of the pad top), upright (tilt under 15 deg), with its centre on the pier
side (y at most -4 cm). Then:

| score | end state |
|---|---|
| 1.0 | crate centre within 1.5 cm of the green pad's centre, tilt under 10 deg |
| 0.6 | crate centre within 3 cm of the green pad's centre |
| 0.3 | crate landed on the pier side of the coaming, off the pad |
| 0 | anything else, including the crate still on the hold side |

The lowest tier needs the crate lifted on both falls and carried over the coaming without touching it: touching or
hauling a fall earns nothing.

## Shortcuts and tiers (each run as a `demo_script` variant after the last edit of `task.py`; evidence `scratch/sc_<mode>.json`)

| mode | what it does | score | end state / latch |
|---|---|---|---|
| (default) correct | haul yellow / ease blue closed loop, release, retreat | **1.0** (step 388) | crate 0.7 cm from pad centre, tilt 0 |
| `none` | arms stay at ready | 0 | crate untouched on hold pad |
| `hold` | correct transfer, never lets go | 0 | crate on pad, grippers still on the falls (not released) |
| `onearm_yellow` | right hauls yellow, left idle | 0 | coaming contact, then jerk 0.70 m/s; crate at y -17 |
| `onearm_blue` | left hauls blue, right idle | 0 | crate never crosses; dropped at 0.57 m/s on the hold side |
| `drop` | blue let go at 45 % of the path | 0 | coaming contact; crate at y -16.5 |
| `low` | peak eye height 10 cm instead of 15 | 0 | coaming contact at y +4 cm, then drop/jerk; crate lands on the pad but still 0 |
| `twoblock` | both falls hauled to the hands' limit | 0 | eye stops 22 cm from either sheave (hands at their reach limit, latch not reached); released there, crate falls onto the coaming |
| `replace` | lifts and sets the crate back on the hold pad | 0 | crate on the hold side (y +10.6) |
| `carry` | left hand reaches in and pushes the crate over | 0 | hand in the hatch square, within 3 cm of the crate, touches it; crate tipped 93 deg against the coaming |
| `gather` | right hand carries the yellow over to take the blue too | 0 | coaming contact while the yellow is carried across; crate dropped off to y -13 |
| `tier03` | correct transfer, landed at y -5.6 | **0.3** | crate 4.4 cm from pad centre, tilt 8 deg |
| `tier06` | correct transfer, landed at y -12.3 | **0.6** | crate 3.0 cm (2.99) from pad centre, tilt 0 |

All runs ended with finite states (no NaN in any log). The `tier06` landing (2.99 cm) is on the 3 cm edge of the 0.6 band: it shows the band is reachable, not that a
landing in the middle of it (about 2 cm) is; the tier boundaries themselves are exact in `score()`. A landing within
1.5 cm but tilted 10-15 deg also scores 0.6.

The two-block latch (lid within 8 cm of a head-sheave axle) was not reached by any run: the closest the eye came to an
axle was 19 cm (`onearm_blue`). A policy that re-grips hand over hand can haul further than the demo's hands, so the
latch stays as a guard; it is not evidence-tested.

## Limit: blind replay

The layout is fixed (nothing is randomised), so a policy that replays the demo's arm motions open loop would also
score; the task tests the two-line coordination, not perception of a varying scene.

## Runtime

One trial: 900 control steps at 20 Hz (45 s of sim time). Seedless: no randomisation, `object_jitter=0`.

## Paper fields

- Skills: hanging/sag (suspended load), constraint-keeping (coaming clearance, hands off the load), placement.
- Industry: maritime cargo handling (stevedoring, rigging).
- Two arms: simultaneous-hold (both falls carry the load during the whole transfer).
- Flaws found and fixed during the build: a fall pinched where it hangs steeply below a high sheave slipped out of the
  fingers (the gripper's palm pushes a near-vertical rope aside), so heel sheaves were added to lead each fall
  horizontally to the hand; a one-arm haul jammed the crate against a high-friction coaming until the sim went
  non-finite, so the coaming is smooth (the crate now slides over it, finite, and the contact latch scores it 0);
  joint break limits on the shackles were tried to cap tension and dropped (they parted during the reset).
