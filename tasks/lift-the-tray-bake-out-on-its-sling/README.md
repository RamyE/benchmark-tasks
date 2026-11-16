# Lift a slab cake out of its deep tin by both ends of its parchment sling

A baking bench on the table. A dark-grey square tin holds a brown cake slab that fills it almost wall to wall. Under
the cake lies a white parchment sling: one strip across the tin floor that runs up two opposite inner walls and stands
4 cm up out of the tin on each side. In front of the tin stands a wire cooling rack. The robot must lift the cake out
of the tin by the two ends of the sling, carry it over, set it flat on the rack and let go.

## Why it matters (societal relatability: home baking)

Lifting a pan of brownies, bars or a snack cake out of its tin on a parchment sling is an everyday home-baking step.
Baking sites and cookbooks tell home cooks to line the tin with a strip whose ends hang over two sides, then lift the
finished bake out by those ends (sources below). Most people who bake at home have done it or watched it done, and
they know what goes wrong: tug one end and the paper slides out from under the cake; let one side sag and the cake
slides off. A household robot that helps in the kitchen has to do this the way people do, with both hands rising
together. The scene is the home version: one tin, one slab, one cooling rack.

## What is tested

The tested ability is lifting a body that cannot be gripped with an untied flat strip, using both hands at once. The
cake is 11.6 x 14.0 cm, wider than the gripper opens (9.4 cm) in both horizontal directions, and it sits low in a tin 5 cm deep, so it cannot be pinched.
So it can only come out on the sling. The sling is not tied to anything. It is a basket that holds the cake only while
both ends rise together and stay level:
- **Lift one end and the strip runs out.** It slides out from under the cake, which stays on the non-stick floor. The
  other end is drawn down into the tin. Nothing rises, with either hand.
- **Let one end go mid-carry and the cake spills out of the sling and drops.**
- One gripper cannot take both ends: they stand 16 cm apart on opposite sides of the cake.

## How this differs from similar tasks

- **Lift a tube level on two hand lines onto its stands** (the nearest). That task has two separate short
  lines, each tied to one end of the tube, and its one-line failure is a tilt. Here there is one untied strip under
  the body, and lifting one end alone carries nothing: the strip just runs.
- **Sling an off-centre beam with two chain legs** That task is about choosing leg lengths for a hoist. The
  robot never lifts.
- **Other lift-out tasks:** lifting a basket out of a well by its rigid handle with one hand, turning a cake out of
  its pan by clamping and inverting it, and lifting a pot by its two rigid handles. In each the robot holds a rigid
  grip point. Here the only grip is a flexible strip that holds the cake by friction and geometry alone.

## Scene (station table; cm from the midpoint between the arm bases, x forward, y left, heights above the table)

| object | type | size | material (real rebuild) | position |
|---|---|---|---|---|
| tin | fixed open box (floor + 4 walls) | inner 12.8 x 16.3 cm (x by y), 5 cm deep; walls 0.4 cm; floor 0.3 cm | dark-grey non-stick metal baking tin (friction 0.05) | centred (32, 0); floor top 0.3, rim 5.3 |
| cake | free box, 400 g | 11.6 x 14.0 x 4.0 cm | brown foam block standing in for a baked slab | centred in the tin on the sling; 0.6 cm from the x walls, 1.15 cm from the y walls |
| sling | flat flexible strip, 18 g | 34 x 9 cm; 0.2 cm thick in the sim (see below) | white silicone-coated baking parchment, about 0.05 mm thick | across the tin floor along y under the cake; up the +y and -y inner walls; both ends stand about 4.1 cm above the rim (tops at about 9.4) |
| cooling rack | fixed | 14 x 18 cm, 2 cm tall; 4 feet, 2 side rails, 8 wires along y | pale steel wire rack | centred (16, 0), x 9-23 |

Nothing else is on the table. The layout is fixed (no randomisation).

**Sling thickness.** Real baking parchment is about 0.04-0.06 mm thick. The sim strip is 2 mm thick for two reasons:
the grippers pinch it across its thickness, and the 400 g cake presses it onto the tin floor. Both are rigid-box
contacts in PhysX (the strip's contact offset is 2 mm), so the pieces are kept millimetres thick so that neither the
closed fingers nor the cake's weight push through them. The extra thickness takes 2 mm of the 11.5 mm gap beside the
cake and lifts the cake 2 mm; neither changes how the sling runs or carries. The sim strip is also stiffer in bending
than paper, so its ends stand up on their own. A real rebuild uses an ordinary sheet of baking parchment cut to
34 x 9 cm, creased at the floor and wall corners, with its 4 cm ends creased upright above the rim (folded double
if they flop).

## Sources (each opened; quotes verbatim)

- King Arthur Baking, "Baking with a parchment paper sling",
  https://www.kingarthurbaking.com/blog/2017/03/27/parchment-paper-sling:
  - "A parchment paper sling doesn't just cover the bottom of the pan, it also goes up and over the sides, too."
  - "The overhanging edges serve as handy little tabs to easily and cleanly lift delicate baked goods out of the pan."
  - "Use metal (oven-safe) binder clips to prevent the sides from folding downward."
  - "Gently pull on the overhanging edges to lift your perfectly baked treat out of the pan."
- America's Test Kitchen, "Making a Foil Sling",
  https://www.americastestkitchen.com/how_tos/6374-making-a-foil-sling:
  - "Lining the pan with an aluminum foil or parchment paper "sling" before baking prevents any casualties."
  - "brownies and bar cookies and often snack cakes can be nearly impossible to remove from their baking pans—no
    matter how well the pan is greased."

What the sim takes from them:
- a single strip that lines the pan bottom and goes up two sides;
- its ends stand up as tabs (the source clips them so they do not fold down);
- the baked slab is lifted out by pulling on both overhanging ends.

## Why two arms (simultaneous hold)

Two arms are needed because of the physics, not reach. Evidence comes from `demo_script` variants (table below):
- `one_left` and `one_right`: one hand pinches its end and lifts it 12 cm straight up (to 20 cm). The strip slides out
  from under the cake. The far end is drawn down into the tin and under the cake, and the cake stays on the floor
  (lowest point 0.5 cm, tilt at most 2.8 deg). Score 0.
- `one_then_other`: lift one end and let it go, then lift the other. Nothing rises. Score 0.
- `gather`: carry one end across toward the other end. The strip runs, and the other end goes into the tin. Score 0.
- `drop`: let one end go mid-carry. The cake spills and drops (latched when it falls faster than 0.35 m/s). Score 0.

The statics at the near cake edge:
- Lifting that edge needs about half the cake's weight, W/2 = 2.0 N.
- The strip slides out at about μW·e^(μπ/2) = 0.15 x 3.9 x 1.27 = 0.75 N, plus the strip's own friction on the floor
  (0.05 x 3.9 = 0.2 N).
- So the strip slides first, with about a 2x margin.

The ends are bare (no tabs or beads): a thickened end would catch at a wall and anchor the strip.

Offline IK (gripper pointing down, any yaw; `scratch/ik.py`): each arm reaches its own end at (32, ±8) from 9 to
17.5 cm height. It also reaches the carry path (24, ±8, 17.5) and (16, ±8, 17.5) and the set-down point (16, ±8, 11),
all within 0.1 mm. The task is not reach-forced. Each arm could reach the other end with a tilted gripper, but a gripper opens 9.4 cm
and the ends stand 16 cm apart, so one gripper cannot hold both. The one-arm routes tried (lift one end, lift them in
turn, carry one end across) all score 0; no one-arm route is claimed impossible beyond those.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder. They provide the strip simulation, the reset
  that lays the sling in the tin, and the scripted-demo runner.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 600 \
    --instruction "A brown cake slab sits in the deep grey tin on a white parchment sling whose two ends stand up out of the tin. Lift the cake out by the sling, set it flat on the wire rack and let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 600
```

The demo runs closed loop on the live strip:
1. Each arm pinches its own standing end across the strip's thickness (fingers closing along y), 2.5 cm above the
   rim. Both do this at the same time.
2. Both rise together to 17.5 cm in small level steps over 6 s. The cake's lowest point ends about 3 cm above the
   rim.
3. Both carry together to x 16 over 6 s, then wait for the swing to die.
4. Both lower in closed-loop steps until the cake's lowest corner is 2 mm above the rack wires, then go 1.2 cm
   lower.
5. Both open, lift 4 cm straight up and sweep back beside their own bases, out of the top camera's view.

Result (`demo/`, log `demo/run.log`): **score 1.0, success at step 391**. The recording was made before the last
`task.py` edit (which only narrowed two failure rules); the demo was re-run on the final code with an identical result,
1.0 with success at step 391 (`scratch/verify_final.json`). During the lift the
cake's lowest point reached 8.6 cm (rim 5.3) with a peak tilt of 9 deg. It came to rest on the rack 1.2 cm from the
rack centre, tilted 0.6 deg. The last top-camera frame shows the tin empty, the cake flat on the sling on the rack
with the sling ends lying out to each side, and both arms back beside their bases. `demo/setup.png` shows the start.

## Scoring (`score()`, evaluated at the end; `success()` = score 1.0 with the cake moving under 1 cm/s; the harness ends the episode as a success once `success()` has held for 1 s in a row)

Score 0 if any of these held at any time (latched for the trial):
- a robot part touched the cake (PhysX contact query on every arm and gripper link; a contact point reported at 1 mm
  separation or less counts, because PhysX reports contacts slightly before the surfaces meet; a pass farther than that
  is not penalised);
- the cake fell faster than 0.35 m/s (dropped). That is the speed of a 6 mm free fall and 5x the demo's peak vertical
  speed (0.07 m/s). Only falling counts: lifting the cake briskly is not penalised, while a drop that would crack a
  real bake is;
- the cake tilted more than 35 deg (spilled or tipped);
- a state went non-finite.

Also 0 unless all of these hold at the end:
- both grippers (TCP and pad tips) are at least 4 cm from the cake and from every point of the sling, measured to its full 9 cm width and 2 mm thickness (each 1 cm piece as a box), so a hand still holding an edge does not count (let go);
- the cake is still (under 2 cm/s);
- the cake is wholly out of the tin: every corner of the cake is above the rim or outside the tin's outline.

Then:

| score | end state |
|---|---|
| 1.0 | cake rests on the rack (lowest corner within 8 mm of the wires, centre over the rack), centre within 3 cm of the rack centre, tilt under 5 deg |
| 0.6 | cake out of the tin, resting upright (tilt under 10 deg) anywhere else, e.g. flat on the rack about 4 cm off centre |
| 0.3 | cake out of the tin, resting tilted 10-35 deg, e.g. set down hanging off the rack's right (-y) edge so one end rests on the table |
| 0 | anything else, including the cake still in the tin |

The lowest tier needs the cake lifted out of the tin on the sling with both hands. Pinching, lifting or pulling one
end earns nothing.

## Shortcuts and tiers (each run as a `demo_script` variant after the last edit of `task.py`, `SLING_DEMO=<mode>`; evidence `scratch/sc_<mode>.json` and `.log`)

All runs went the full 600 steps with finite states. The last edit of `task.py` only narrowed two failure rules (the
drop rule now counts falling only; a near miss without contact no longer fails). `push`, `tilt`, `uneven` and `drop`,
the runs where a failure rule fired, were rerun after it; the other runs fired no failure rule, so that edit cannot
change their scores (the scorer does not act on the physics).

| variant | what the arms do | end state | score |
|---|---|---|---|
| `none` | nothing | cake untouched in the tin | 0 |
| `one_left` / `one_right` | one hand pinches its end and lifts it to 20 cm, holds 3 s, lets go | the strip slides out from under the cake; the cake stays on the tin floor (lowest point 0.5 cm, tilt at most 2.4 / 2.8 deg) | 0 (in tin) |
| `one_then_other` | lift the left end and let go, then the right end | the cake stays on the floor (peak tilt 3.7 deg) | 0 (in tin) |
| `gather` | the left hand carries its end across toward the right end | the strip runs; the cake rocks (peak 18 deg) and settles back on the floor | 0 (in tin) |
| `push` | a closed bare gripper prods the cake and tries to lever it against the wall | the cake shifts 5 mm and stays in the tin | 0 (touch latch) |
| `drag` | both ends pinched and drawn toward the robot at pinch height | the pinches slip off and both ends stay standing at the tin walls; the cake moves 2 mm and stays in the tin | 0 (in tin) |
| `tilt` | left end lifted to 22 cm, right end raised only 4 cm (to about 11.8 cm) | the cake tips over sideways at 35 deg and ends on edge in the tin | 0 (tilt latch, touch) |
| `uneven` | both lift together, left 6 cm ahead | the cake tips past 35 deg and falls on edge in the tin | 0 (tilt, drop and touch latches) |
| `drop` | correct lift; the left hand lets go mid-carry at x 24 | the cake spills out and drops (0.38 m/s), ends tilted 27 deg between tin and rack | 0 (drop latch) |
| `hold` | correct lift, carry and set-down, never lets go | the cake sits on the rack but both hands still hold the sling | 0 (not released) |
| `tier06` | correct lift; set down 4.5 cm right of centre | flat on the rack, 4.0 cm from the rack centre, tilt 0.1 deg | **0.6** |
| `tier03` | correct lift; set down 9.5 cm right of centre | one end on the rack, the other on the table, tilt 10.4 deg | **0.3** |

The tier03 tilt (10.4 deg) is only just past the 10 deg line. It came out at 10.4 in the logged run. It is set by geometry
(one end on the table, the other on the 2 cm rack rail), so a cake resting further onto the rack lands in the 0.6 tier.

## Limit: blind replay

The layout is fixed (nothing is randomised). A policy that replays the demo's arm motions open loop would also score.
The task tests the coordinated two-hand sling lift, not perception of a varying scene.

## Paper fields

- Skill tags: extraction, placement/pinning, constraint-keeping (level, simultaneous hold).
- Domain: societal relatability (home baking).
- Two-arm need: simultaneous hold.

## Runtime

One trial: 600 control steps at 20 Hz (30 s of sim time). Seedless: no randomisation, `object_jitter=0`.
