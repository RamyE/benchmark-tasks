# Fit a child seat tightly by pressing it down while the other hand takes up the lower-anchor strap

A child-seat fitting station on the table. A grey vehicle-seat cushion block has a blue child-seat shell sitting in a
pit in its middle; the shell rides on a spring (the cushion) and sits at its rest height. A black lower-anchor strap
(an 8 mm webbing stand-in) is hooked to the left lower anchor, runs up and through the shell's covered belt path
across its rear, and down into the connector on the right lower anchor. Inside the connector the strap ends in a
slider (the adjuster); its tail comes out of the connector's end as an orange pull tab. The adjuster is one-way: the
tail can be pulled out to shorten the strap, but the strap cannot pull it back. At the start the strap is loose (about
1 cm of slack).

Instruction: "The blue shell is an empty child car seat on a vehicle seat cushion. Tighten its lower-anchor strap so the car seat is fitted firmly, then let go."

The tested ability is making a line tight by loading a spring before fastening it, which needs two simultaneous acts:
- pulling the tail alone only takes up the slack. Once the strap is taut, the tension needed to press the sprung seat
  down any further is more than the pull tab's breakaway link holds (4 N), so a one-arm pull leaves the seat at its
  rest height (0 mm, `pullonly` below) or tears the tab (`yank`);
- the fit comes from body weight: one hand presses the seat 25 mm into the cushion and holds it there, which makes
  about 4 cm of new slack, and the other hand takes it up at low force; when the press is let go, the cushion pushes
  the seat up against the now-short strap, and the strap holds it down 22-23 mm;
- pressing alone does nothing lasting: the seat springs back when let go (`pressonly`).
This is the documented way to install a child seat on lower anchors ("Press down firmly on the car seat and tighten
the lower anchor straps").

## How this differs from similar tasks

- Hold a sprung belt tensioner at its mark with one hand while the other turns its lock knob (TK-B5F4YZ, nearest on
  the board): one hand holds a rigid lever against its spring, the other turns a rigid knob, and the end state is a
  lever angle. Here the spring is the load itself (the seat on its cushion), the second hand shortens a flexible strap
  through a one-way adjuster, and the score is how tight the strap ends, read from how far it holds the seat down
  after both hands let go. Holding is not the point; what matters is that the strap is shortened while the load is
  pressed, and how much.
- Hold a reel's spring-loaded cord at a length mark while the other hand sets the reel lock (our nearest): a cord is
  held at a mark against a reel spring and a lock is set; the cord's length is the result. Here no mark exists, the
  strap is never held against a spring, and the result is a load the strap keeps compressed.
- Compress a spring plunger and slide a lock bar (TK-6JB9Z5), or hold a sprung drive down while turning its latch
  (TK-M6GCBU): a rigid part is pushed to a fixed stop and a rigid catch is moved; the end state is binary (latched or
  not). Here the catch is a flexible strap drawn through an adjuster, the end depth is continuous and set by how much
  strap is taken up while the load is pressed, and pulling the strap is itself the tightening.
- Ratchet a tie-down strap taut (TK-AUMWWL): one actor works a ratchet on a fixed load; nothing is pressed.
- Strap a load over its top to two anchors (ours): the strap is laid over a carton into cups, with no tension. Buckle a
  doll into an infant car seat (TK-P5DQA7): harness buckling, no strap tension or seat fitting.
- No board task or public set (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench, RoboDojo) tightens a strap by
  preloading the thing it holds.

## Scene (station table; cm from the midpoint between the arm bases, x forward, y left, heights above the table)

| object | type, size, material | position |
|---|---|---|
| vehicle-seat cushion | grey fixed block 28 x 50 x 6 cm (rigid foam), with a pit 18.6 x 20.6 cm through it | x 15-43, y -25..+25; pit x 19.7-38.3, y -10.3..+10.3 |
| child-seat shell | blue box 18 x 20 x 8 cm, 500 g (printed shell or a wooden block) | centre x 29, y 0; top 12.2 cm high (6.2 cm above the cushion) at rest |
| seat spring | vertical slide + spring under the shell: 300 N/m, 4 N preload against a top stop, 3 cm travel | under the shell |
| belt path | two blue ridges 0.4 x 20 x 1.1 cm on the shell top 2.6 cm apart, bridged by a blue cover bar 3.4 x 12 x 0.4 cm | ridges at x 33.3 and 36.7, cover over y -6..+6 |
| lower-anchor strap | black 8 mm round webbing stand-in, 40 cm, 50 g | hooked at (35, +15, 6.8 high), up over the belt path, into the connector |
| left lower anchor | dark block 3 x 1.5 x 1.2 cm (the strap hooks to it) | x 35, y +16.75, on the cushion |
| right connector | dark roofed channel along y, inner 2.4 wide x 2 cm high, entry lips (inner stop), end wall with a 1.8 x 1.3 cm slot | x 35, y -14 to -27.2, on the cushion |
| adjuster slider | dark block 3 x 2 x 1.4 cm, 25 g, on the strap end inside the connector | starts at y -20 |
| tail with pull tab | orange strip 1.4 x 9 x 0.5 cm, 8 g, out through the slot; joined to the slider by a breakaway link (4 N) | from y -21.5 to -30.5 |

## Sources (each opened; quotes verbatim)

- Ohio State University BuckleUp program, child car seat installation, https://buckleup.osu.edu/child-car-seat-installation/ :
  "Press down firmly on the car seat and tighten the lower anchor straps." and "Grab the car seat at the belt path and
  pull on it firmly; it should not move side-to-side or front-to-back more than 1 inch."
- Children's Hospital of Philadelphia, What is LATCH?, https://www.chop.edu/pages/what-latch : "In order to get it to
  fit tightly, you need to press the seat down and back while tightening the straps." and "use your upper body's
  weight to press down the seat".
- O*NET 33-9094.00 School Bus Monitors, https://www.onetonline.org/link/summary/33-9094.00 : "Buckle seatbelts or
  fasten wheelchair tie-down straps to secure passengers for transportation."

## Why two arms

Simultaneous, not reach. Offline position-only IK (`scratch/ik.py`, gripper down at any yaw): the press point
(25.5, +4) at 11.0-13.5 cm is reachable by either arm (0.0 mm error); the pull tab (35, -29..-34, 7.8 cm, where it is pinched and drawn to) by the right
arm (0.0 mm) and not by the left arm with the gripper pointing down (no solution). The second arm is needed because the
slack the press makes exists only while the seat is held down, and the tail must be drawn out during that time:
- `pullonly` (right arm only, pulling the tab until its link is stretched to just under the tear force): the strap goes
  taut after 1 cm and the seat does not move (0.0 mm compression), score 0;
- `pressonly` (left arm presses 25 mm, lets go, tab never touched): the seat springs back to 0 mm, score 0;
- the demo (press and pull together): 22-23 mm held after both let go, score 1.0.
One arm cannot press and pull at once (the press point and the tab are about 34 cm apart).

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (strap simulation, the reset that lays the strap
  through the belt path, scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "The blue shell is an empty child car seat on a vehicle seat cushion. Tighten its lower-anchor strap so the car seat is fitted firmly, then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop on the live shell height, the slider and the tab link. The left arm closes its fingers and presses the
shell top at (25.5, +4), clear of the belt path, stepping down until the shell is 25 mm below rest (about 11 N). At the
same time the right arm pinches the tab's outer end across its width (fingers closing along x, wrist camera toward the
connector). The right arm then draws the tab straight out in 2 mm steps; the slider follows freely through about
5 cm of slack, and the arm stops as soon as the tab link starts to stretch (the strap is taut: about 2 N felt). The
right arm opens, lifts 4 cm and sweeps back to its side; then the left arm eases off over 1.5 s, the shell rises a
few mm against the strap, and the left arm lifts and sweeps back to its side, so the top camera sees the seat and
strap.
Result (`demo/result.json`, recorded with all three cameras, final code): score 1.0, success at step 379 (19.0 s).
While held the shell is 25-26 mm down; the slider is drawn out 49 mm before the strap goes taut. After release the
strap holds the shell 22.5-22.6 mm down in development runs of this script (success needs >= 18 mm held for 1 s);
the final-code variants that do the same press and pull end at 23.4 mm (`holdtab`) and 23.7 mm (`pulltear`, scored
0 for the torn tab).

## Scoring (`score()`, at the end; `success()` holds it 1 s with the shell and strap at rest)

Score 0 if, at any moment of the trial, the tab's breakaway link was pulled past its 4 N tear force (read from the
link's stretch, latched) or the strap left the belt path under the cover (latched); or if, at the end, a gripper is
within 3 cm of the strap, the tail or the shell. Otherwise, by the shell's compression (how far its top sits below
its rest height, held there by the strap; a ruler against a mark on the shell side): >= 18 mm 1.0; >= 10 mm 0.5;
under 10 mm 0. Tiers as end states: 0.5 = the strap was taken up while the seat was pressed only part way (the
`half` variant presses 13 mm and ends at 12.0 mm); 1.0 = taken up while pressed firmly (22-23 mm). Pulling the tail
alone ends at 0 mm (strap taut, seat not moved), so the lowest tier needs the press and the pull together.

## Shortcuts (each run as a `demo_script` variant, `SEAT_DEMO=<name>`, after the last edit of `task.py`; evidence `scratch/sc_<name>.json`)

| variant | what it does | end state | score |
|---|---|---|---|
| `none` | waits | shell 0 mm down, strap slack | 0.0 |
| `pullonly` | one arm: pinches the tab and draws it out in 2 mm steps until the tab link carries about 3 N (just under its 4 N tear force), holds 1 s, lets go; the seat is never pressed | strap taut after 11 mm of tail, shell 0.0 mm down | 0.0 |
| `yank` | one arm: pinches the tab and pulls it out 1 cm per 0.1 s | tab torn (link stretched 15 mm), shell 0.9 mm | 0.0 |
| `pressonly` | left arm presses the shell 25 mm and lets go; tab never touched | shell back at 0 mm | 0.0 |
| `hold` | full method, but the left arm keeps pressing at the end | shell 25.8 mm down under the gripper; not released | 0.0 |
| `holdtab` | full method, but the right arm keeps the tab pinched | shell 23.4 mm down; tab still held | 0.0 |
| `pulltear` | seat pressed, but the tab is pulled on after the strap is taut | tab torn (latched); shell 23.7 mm | 0.0 |
| `half` (tier check) | full method, pressed only 13 mm | shell 11.8 mm down, released | 0.5 |

Evidence: `scratch/sc_<variant>.json` and `.log`, all run after the last edit of `task.py` (`scratch/seq_local.sh`).
The strap cannot be lifted out of the belt path: the cover bar closes it (the latch exists but cannot fire without
unhooking the strap), and the shell slides only up and down, so it cannot be pushed or tipped.

## Feasibility checks

- No NaN in any run (probes, demo, every variant). At the end the strap holds the shell against about 11 N of cushion
  push, with one end on a ball joint and the other held by the adjuster (a stiff one-way spring), not by a gripper.
- The left arm pressing at about 11.5 N (4 N preload + 300 N/m x 25 mm) holds the shell 25-26 mm down (measured).
- The one-way adjuster lets the slider back in about 2.5 mm under the final strap tension (53.1 mm out, then 50.6 mm).
- Pulling the tab alone: the strap is taut after 11 mm and the shell does not move at 3 N on the tab; the cushion's
  4 N preload and the strap's friction over the shell's edges take it all (`pullonly`).

## Limit: blind replay

The layout is fixed (`object_jitter=0`, nothing randomised), so a policy replaying the demo's arm motions open loop
would also score 1.0; the task does not test perception of a varying scene. It tests the coordinated press-and-pull
and knowing when to stop pulling.

## Runtime

One trial: 1,200 control steps at 20 Hz (60 s of sim time); the scripted demo finishes at about 21 s. Seedless.

## Paper fields

- Skills: tension and length (taking up slack), constraint-keeping (tab not torn, strap in its path), sequencing
  (press, take up, release in that order), two-hand coordination.
- Industry: child passenger safety (child-seat fitting), passenger transport.
- Two arms: simultaneous (press while pull); reach is not the reason.
