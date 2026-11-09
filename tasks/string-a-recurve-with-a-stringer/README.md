# String a model recurve bow with its bow stringer, then let go

A model recurve bow lies flat on the table with its back away from the robot. Its two limbs are sprung straight (an
unstrung bow). The bowstring's bottom eye is already in the right-hand limb's nock; its top eye sits on the left-hand
limb 1.8 cm short of the nock, where an unbent bow leaves it: the string is straight between the eyes, so the eye cannot
go further without bending the limbs. A bow stringer is fitted: two blue cords run from pockets on the two limb tips
to a yellow handle lying on the table in front of the bow.

To string the bow the robot has to do what an archer does: draw the stringer's handle back so the stringer bends both
limbs, and while it holds them bent, slide the top eye up the limb, over the nock lip and down into the nock; then
ease the handle forward so the string takes the limbs' load, and let go. The handle must be held the whole time the
eye is moved (the limbs spring back in under a second when it is let go), and the eye is 23 cm away from the handle,
so one arm holds the stringer while the other seats the string (simultaneous hold).

The tested ability is using a line (the stringer) as a tool to store a spring load in a part, and seating another
line's (the string's) loop while that load is held, in the right order: bend first, seat, then hand the load from
the stringer to the string. The string is the deformable object that ends up loaded; the end state is a braced bow.
The score rejects the two ways round the stringer: bending a limb by hand (any gripper contact with a limb) and
forcing the string loop up an unbent limb.

## How this differs from similar tasks

- Hold a sprung belt tensioner at its mark with one hand while the other turns its lock knob (board, TK-B5F4YZ, the
  nearest): one hand holds a sprung rigid lever against its spring while the other locks it. Here the spring load is
  stored and held through a line (the stringer pulls both limbs at once from one handle 25 cm away), the second hand
  seats a line's loop, not a knob, and the job ends with the load handed over to that line (the string carries the
  limbs and the stringer goes slack); a released stringer before the eye is seated undoes everything.
- Hold a bag's pocket flap open and stow a charger inside (ours, `stow-charger-in-bag-pocket`): one arm holds a
  part open by gravity while the other packs a cable. Here nothing is held open; the held part is a line in tension,
  the seated item is the other line's end, and the order (bend, seat, ease) decides the outcome.
- Pull a reel cord to a mark and lock it (ours): a single line under a reel spring. No line-as-tool, no load
  transfer.
- No board task or public set (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench, RoboDojo) strings a bow or
  uses a cord to bend a sprung part.

## Scene (station table; cm from the midpoint between the arm bases, x forward, y left, heights above the table)

| object | type | size | material (real rebuild) | position |
|---|---|---|---|---|
| riser | fixed block | 3 x 9.2 x 2.5 cm | wood or brown PLA | x 32.5-35.5, y -4.6 to +4.6 |
| limbs (2) | rigid bars, 20 g each, pinned about vertical axes at the riser ends, torsion spring 0.15 N m/rad with 3 deg preload against the straight rest stop, bend 0-50 deg | 0.6 x 15 x 2.5 cm | black PLA on M4 shoulder bolts with small torsion springs | near face at x 32.5, pivots at (32.8, +-5), 0.5 cm above the table, tips at y +-20 |
| nock lips (2) | part of each limb | 0.6 x 0.4 x 1.5 cm | black PLA | on each limb's far face, 13.3 cm from the pivot, 1.2-2.7 cm above the table |
| bottom eye | rigid loop fixed in the right-hand limb's nock | 1 x 0.6 x 2.4 cm | red PLA | far face of the right-hand limb, 13.8 cm from the pivot (y -18.8) |
| top eye | rigid loop, 4 g, slides along the left-hand limb's far top edge, can lift 12 mm to pass over the lip | 1 x 0.6 x 2.4 cm | red PLA on a PTFE-taped edge | 12.0 cm from the pivot (y +17); the nock is at 13.8 cm |
| bowstring | cord, 6 g, tied eye to eye | 4 mm, 35.4 cm | white braided cord | straight along the limbs' far faces, x 33.7 |
| stringer cords (2) | cords, 8 g each | 6 mm, 19.4 cm each | blue braided cord | from pockets on each limb tip's near face (32.1, +-19.7) to the handle's front face |
| stringer handle | free block, 20 g | 5 x 1.6 x 3.5 cm | yellow PLA | centre (26.5, 0), long side along x |

Brace: with the eye in the nock and the stringer slack, the string holds both limbs bent about 22 deg (the string then runs
about 4 cm in front of the riser's near face); unstrung they rest at 0 deg.

## Sources (each opened; quotes verbatim)

- Archery360, "How to String a Bow", https://archery360.com/2019/06/19/how-to-string-a-bow/: "Stringing a bow is a
  fundamental skill of recurve and longbow archery." "You can use other methods, but a bow-stringer provides the
  safest and only method bow manufacturers recommend." "The bow-stringer supports the bow's draw weight as you slide
  the top string loop into the limb tip's string grooves."
- O*NET 39-3091.00 Amusement and Recreation Attendants, https://www.onetonline.org/link/summary/39-3091.00: "Maintain
  inventories of equipment, storing and retrieving items and assembling and disassembling equipment as necessary."

What the sim takes from them: the limbs are bent only through the stringer (pockets on both tips, pulled from its
middle), the top loop is slid up the limb into the tip groove while the stringer carries the load, and the stringer
is then let off so the string carries it. An archer stands on the stringer cord and lifts the bow; here the bow is
fixed and the robot draws the stringer's middle instead, the same load path.

## Why two arms

Simultaneous hold. The stringer handle must stay drawn back (to x 12 cm, 15 cm behind its start) while the top eye is
moved, 23 cm away; let go of the handle and the limbs spring back to 0 deg in under a second, which pulls the string
straight and leaves no room for the eye. Both points are in reach of the arm that works them (offline IK,
`scratch/ik.py`, output `scratch/ik.out`, gripper pointing down):

| point (x, y, height) | arm | error |
|---|---|---|
| handle grip from start (25.3, 0, 2.8) to full draw (10.8, 0, 2.8), yaw 0 | right | 0.0 mm |
| top eye pinch (26.5-27.9, 15.2-17.8, 6), yaw 0.52 rad (along the bent limb) | left | 0.0 mm |
| retreat points (8, +-52, 10) | left / right | 0.0 mm |

Each arm can also reach the other's point; the need is holding two places 23 cm apart at once, not reach.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (cord simulation, scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 900 \
    --instruction "String the bow with its bow stringer, then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 900
```

Closed loop on the live limb angles and the live eye. The right arm takes the stringer handle by its rear end
(fingers across it) and draws it straight back in 4 mm steps until both limbs are bent 29 deg or more (handle from
x 26.5 to 11.8 cm; limbs 29.4 and 30.3 deg, at 9 s). Holding it there, the left arm pinches the top eye along the
limb, lifts it 9-10 mm (over the height of the nock lip), and steps it up the bent limb, following the limb's live
angle and never running the command more than 15 mm ahead of the eye; once the eye is past the nock it is pressed
down onto the limb edge behind the lip (seated at about 16 s). The left arm opens, lifts 4 cm straight up and sweeps
back to its side. The right arm then eases the handle forward in 4 mm steps; the limbs follow it back until the
string takes them at about 22 deg, and the handle goes slack. The right arm opens, lifts 4 cm and sweeps back to
its side, so the last frames show the braced bow with the stringer slack and both arms clear.
Result (`demo/result.json`, recorded with all three cameras): score 1.0, success at step 452 (22.6 s): eye in the
nock, pulled back against its lip by the string, limbs held at 22.2 and 22.3 deg by the string, stringer slack,
no gripper or arm contact with a limb, nock lip or the bottom eye.

## Scoring (`score()`, evaluated at the end; `success()` = score 1.0 held for 1 s)

Score 0 if any of these holds (the first two latched for the whole trial):
- a gripper or arm link touches either limb, a nock lip or the bottom eye (contact read every physics substep);
- the top eye goes more than 3 mm up its limb from where it starts (past about 12.35 cm from the pivot) while the stringer
  handle is less than 8 cm behind its start position (the string forced up an unbent limb); checked each time the eye
  passes that mark;
- at the end the top eye is not seated: 1.5 mm or more short of the nock (the slider end stop is 5 mm past it; code accepts any position from 1.5 mm short to the stop) or still lifted 3 mm or more (not down
  behind the lip);
- at the end either limb is bent less than 15 deg (the string is not carrying the limbs);
- at the end a gripper TCP is within 4 cm of the top eye (not let go).

Otherwise:

| score | end state |
|---|---|
| 1.0 | bow strung (eye in the nock, both limbs held bent by the string), eye let go, and no gripper within 5 cm of the stringer handle (stringer let go) |
| 0.5 | eye seated in the nock and let go, limbs bent, but a gripper still holds the stringer handle (within 5 cm) |
| 0 | anything else |

The lowest tier needs the limbs bent with the stringer, the eye carried 1.8 cm up the limb, over the lip and into
the nock, and let go; drawing the stringer or touching the eye alone earns nothing.

## Shortcuts (each run as a `demo_script` variant after the last edit of `task.py`; evidence `scratch/sc_<mode>.json`)

| variant (`BOW_DEMO=`) | what it does | outcome | score |
|---|---|---|---|
| `none` | touches nothing | limbs straight, eye 1.8 cm short of the nock | 0 |
| `draw_only` | draws the stringer (limbs 29 / 30 deg), never seats the eye, eases back and lets go | limbs back to 1.8 / 2.3 deg, eye 2.1 cm short | 0 |
| `push_eye` | no stringer: lifts the top eye and drives it up the unbent limb | latched at about 4.5 s (eye 3-5 mm up with the handle not drawn); the string, held at both eyes, then goes unstable in the sim (5.9 s) | 0 |
| `onearm` | left arm alone: draws the stringer to 30 deg, lets go of the handle, then tries to seat the eye | limbs back to 0.0 deg and the handle pulled back to x 28.6 within 1.8 s; forcing the eye up is latched (then unstable, as `push_eye`) | 0 |
| `limb` | left arm pushes on the left limb's back face, 9 cm out (bent 24 deg), then lifts off and tries to seat the eye | gripper contact with the limb latched at 4.6 s; the eye forced up the unbent limb is latched too | 0 |
| `hold` | correct, but the left arm never lets go of the eye | eye seated, gripper still on it at the end | 0 |
| `tier05` | correct up to the seat, but the right arm keeps holding the stringer handle drawn | eye seated and let go, limbs held 29.5 / 30.9 deg by the stringer, handle still in the gripper | 0.5 |
| correct demo | | strung and braced at 22.2 / 22.3 deg, both grippers clear | 1.0 |

Found and fixed during the build: with the first limb spring (0.05 N m/rad), the one-arm variant scored 1.0, since a
dropped handle stayed put on the table and kept the limbs bent (`scratch/weak_springs/`); the springs were made three
times stiffer. A latch threshold of 6 mm let the forced eye stop short of it (the stiffer string jams at about 5 mm
before the sim goes unstable); the code latches at 3 mm from the start (the logs show 3.2 mm). The tier-0.5 wording reflects that in that state the
stringer, not the string, is still holding the limbs.

## Feasibility checks

- Draw: the stringer bends both limbs evenly through a 15 cm draw (29.4 / 30.3 deg in the demo; 29.9 / 31.0 deg with
  no tie stretch over 2 mm at the first spring rate, `scratch/probe7.log`). An earlier layout had the riser ends
  touching the limbs at the pivots; the limbs jammed against the riser and a 21 cm draw bent them only 14 / 17 deg,
  the handle slipping 4 cm in the gripper (`scratch/probe6.log`). The riser is now 4 mm short of each pivot.
- Seat and brace: the eye slides into the nock with the stringer drawn, and with the stringer eased the string holds
  the limbs at about 22 deg (demo).
- Load hand-over: let go of the handle without the eye seated and the limbs are back at 0 deg within about a second
  (`onearm`: 0.0 deg 2 s after the handle was dropped, the handle pulled back to x 28.6). With the first spring rate
  tried (0.05 N m/rad) this was not so: a dropped handle stayed where it was, held by its own and the cords' table
  friction, the limbs stayed bent 27 deg, and the one-arm variant scored 1.0 (`scratch/weak_springs/sc_onearm.log`).
  The springs were made three times stiffer (and the string's end ties stiffer, so the brace stays near 22 deg).
- Wrist cameras: mean brightness 114-184 (left) and 131-192 (right) through the demo; the left camera looks down on
  the eye and the nock lip while seating, the right on the handle and both stringer cords while drawing.

## Limit: blind replay

The layout is fixed (nothing is randomised), so a policy that replays the demo's arm motions open loop would also
score 1.0; the task does not test perception of a varying scene. What it tests is the order and the hold: the eye can
only be seated while the stringer is held drawn, and the stringer can only be let go after the eye is in.

## Runtime

One trial: 900 control steps at 20 Hz (45 s of sim time); the scripted demo succeeds at about 23 s. Seedless: no
randomisation, `object_jitter=0`.

## Paper fields

- Skills: tool-use (a cord as a tool to bend a sprung part), routing/seating (string loop into the nock),
  constraint-keeping (no hand on a limb), sequencing (bend, seat, ease).
- Industry: sport and recreation (archery range set-up, equipment preparation).
- Two arms: simultaneous hold (stringer held drawn while the other arm seats the string).
