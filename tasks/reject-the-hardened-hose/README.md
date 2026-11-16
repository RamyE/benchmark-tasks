# Find the hardened hydraulic hose by flexing each one, and lay it in the reject tray

A hose-inspection bench on the table. Three black hydraulic hoses with brass fittings on both ends lie straight and
parallel across the robot's midline. They look the same at rest. One of them has hardened: it bends five times less
easily than the other two. Which one is drawn at random for every trial (any of the three positions), and nothing in
the scene shows it. A red reject tray lies along the near edge. The robot must find the hardened hose, lay it in the
tray, keep the good hoses out of the tray, and let go.

The robot can only find the hardened hose by flexing the hoses and reading their response. Lifted by one fitting 10 cm
and pushed 14 cm towards the other end, a good hose folds over at once and comes down to the table near the raised
fitting, and its far fitting barely moves. The hardened hose stands up in a long ramp and shoves its far fitting along
the table. The hardened hose must be identified by flexing (a measured push, defined below: the hose itself, or both good
hoses so the third follows by elimination) before it goes into the tray, so a policy that skips the test scores 0,
and a policy that tests but ignores the result is right only one trial in three.

Instruction: `One of these hydraulic hoses has hardened and must be taken out of service. Put it in the red tray and leave the good ones on the bench, then let go.`

## Objects

| object | type | size | material / colour |
|---|---|---|---|
| hoses a, b, c | deformable hose | 60 cm between fittings, 10 mm outside diameter, 45 g | black rubber hose (spec below) |
| fittings (2 per hose) | rigid, on the hose ends | 30 x 14 x 14 mm, 8 g each | brass |
| reject tray | fixed open box | inside 10 x 70 cm, 4 mm walls 2 cm tall, 2 mm floor | red |

Positions (cm from the midpoint between the arm bases, x forward, y left): hoses straight along y from y -30 to +30
on tape lines at x 28 (a), 35 (b) and 42 (c); fitting centres at y -31.5 and +31.5. Tray inside x 11.5-21.5,
y -35 to +35. One of a, b, c is the hardened hose, drawn per trial.

## Real-world hoses and the flex test

The task needs three hoses that look the same and a hardened one whose stiffness shows clearly in the flex test.

- **Hose stock.** Cut all three 60 cm lengths from one roll of the same black 10 mm OD rubber hose and crimp the same
  brass end fittings on both ends (fitting body about 30 x 14 x 14 mm). Weigh them: each hose with fittings should be
  within 10% of the others.
- **Making the hardened hose.** Heat-age one length in an air-circulating oven, the standard way rubber is aged on
  purpose (air-oven ageing in the style of ASTM D573, which uses a 70 h exposure and leaves the temperature to the user; e.g. 70 h at 125 C, ageing further in 24 h steps, our own choice, until it passes the check below).
  Heat ageing hardens rubber hose; this is the "hard and brittle" failure in the sources. If ageing alone cannot
  reach the target, use a length of the same-OD hose with a stiffer reinforcement under the same black cover. Mark it
  with a small dot under one fitting, which the cameras cannot see.
- **Stiffness check (tape measure).** Lay the hose flat on the bench with one end overhanging the edge: 6 cm of hose
  plus its fitting, the rest held down flat. Measure how far the fitting's centre drops below the bench top. Accept
  the set only if each good hose drops at least 5 cm (it hangs nearly straight down), the hardened hose drops at most
  3.5 cm, and the two good hoses agree within 1 cm. The simulated hoses have bending stiffness EI = 1.2 x 10^-4 N m^2
  (good) and 6.0 x 10^-4 N m^2 (hardened); a simple cantilever estimate with these values gives about 5.7 cm (good) and 1.1 cm (hardened) over 6 cm. This check was not run in the sim.
- **Flex-test check (the response the robot reads).** On the bench, lift one fitting of each hose 10 cm and push it
  14 cm towards the other end, hold 1 s. Accept the set only if each good hose touches the table within 10 cm of the
  raised fitting and its far fitting moves less than 5 cm, and the hardened hose stays off the table for at least
  12 cm and pushes its far fitting at least 7 cm. Measured in the simulation: in the three demo runs the good hoses' far fittings slid 2.1-3.7 cm (bend 105-129 degrees, touching down about 6 cm from the raised fitting) and the hardened hose's 9.2-10.8 cm (bend about 50 degrees, a ramp about 16 cm long).

Repeat both checks before each session (ageing continues and hoses relax), and replace a hose that no longer passes.

## Reset (every trial)

1. Draw the hardened hose's position from the trial seed:
   `["a", "b", "c"][numpy.random.default_rng([seed, 4057]).integers(3)]`, the same draw the simulation uses.
2. Lay the hardened hose straight on the drawn tape line and the two good hoses on the others, fittings centred at
   y -31.5 and +31.5 cm. Tray at its marks, empty. Arms at their ready pose.

## Scoring (`score()`, at the end; `success()` holds 1 s)

Definitions, all read from the hose, fitting and gripper positions (in a real trial: an overhead camera, a tape
measure, and the operator's note of which hose is hardened):

- **Flexed:** at some moment a gripper holds one of the hose's fittings (gripper centre within 3 cm of it) with that
  fitting at least 6 cm above the table and at least 10 cm in from its rest place towards the hose's other end (an
  end-on push that loads the hose in bending), while two parts of the hose point at least 30 degrees apart (largest
  angle between any two 2 cm pieces of its centreline; a straight hose is about 5 degrees). A straight lift of one
  fitting, or carrying a hose by both fittings, is not a flex.
- **Tested:** flexed, and afterwards let go (no gripper centre within 3.5 cm of any part of it). Both are latched.
- **Entered the tray:** any point of the hose or its fittings over the tray's inner outline (3 mm margin) and below
  6 cm.

- **Identified:** the hardened hose has been tested, or both good hoses have (the untested third is then the
  hardened one by elimination).

Score 0 if any of these holds:

- at the end, any point of a good hose lies over the tray's inner outline below 6 cm (a wrong hose, or all of them,
  in the tray); a good hose carried over the tray and taken away again is not penalised;
- when the hardened hose first entered the tray, it was not yet identified (latched), so a pick without testing
  scores 0 even when it happens to be right;
- a hose went off the table; or at the end a gripper centre is within 3.5 cm of any hose (not let go).

Otherwise:

- **1.0:** the whole hardened hose inside the tray: its centreline at least its 5 mm radius inside the tray's inner
  outline, both fittings' whole 30 x 14 mm footprints inside the outline, and all of it below 3 cm.
- **0.5:** both whole fittings inside the outline and below 3 cm, but some of the hose outside (draped over a wall).
- **0:** anything else (for example the hose laid on the tray wall or beside the tray).

`success()` also needs every hose at rest and both gripper centres at least 10 cm from the hardened hose, so a
successful episode ends on a view of the tray with the whole hose in it.

Chance: testing can be done without using the result, so a policy that tests and then lays a fixed or random hose
scores 1.0 only on the one trial in three where it happens to be right (the hardened hose is drawn uniformly per
trial), and 0 otherwise. Testing one hose and laying another untested one scores 0.

## Why two arms (offline IK, gripper pointing down)

- Each hose's two fittings are 63 cm apart, one on each side. The left arm cannot reach the right-hand fittings:
  best position error 73.5 / 91.7 / 141 mm at x 28 / 35 / 42 cm, y -31.5 cm, fitting height. The right arm misses
  the left-hand fittings by 119.8 / 157 / 156 mm. Each arm reaches its own fittings exactly, also lifted 10 cm and
  pushed 14 cm in.
- Near the tray's ends at the tested points (tray x 15-17 cm), each end is reachable only by its own arm (the other arm misses by 72-86 mm); at the tray's far edge (x about 19-21 cm) the other arm comes within about 1-2 cm. Laid in the tray the
  hose (66 cm with its fittings) reaches past y -20 and +20 cm, so one end lies where only the other arm reaches. One
  arm alone dragging the hose by one fitting or by its middle left the fittings outside the tray in earlier tests.

## Demo video

The scripted demo does not know which hose is hardened. It chooses every motion from the fixed layout, the arms' own
commanded poses and the top-camera image; the simulator's object state is used only in debug logging.

1. Both arms move aside so the top camera sees all six fittings.
2. For each hose a, b, c: the camera locates the hose's two brass fittings. The left arm takes the left fitting,
   lifts it 10 cm while pushing it 14 cm towards the far end, and holds it 1 s. The camera locates the far fitting
   again; how far it slid is the reading. The left arm lays the hose back and lets go.
3. The hose whose far fitting slid furthest is the hardened one.
4. Both arms move aside, the camera locates that hose's fittings, both arms take one each, lift the hose 14 cm with
   2 cm of slack, carry it over the tray, lower it, let go 4.5 cm above the floor and move out to the sides.

The demo reaches 1.0 with the hardened hose at each of the three positions, 772 steps each (38.6 s at 20 Hz, of
which about 25 s are the three flex tests). Far-fitting slide seen by the camera (good / hardened):

| hardened hose | slide a | slide b | slide c | chosen | score |
|---|---|---|---|---|---|
| a | 10.5 cm | 2.1 cm | 3.7 cm | a | 1.0 |
| b | 3.1 cm | 9.2 cm | 2.6 cm | b | 1.0 |
| c (the video) | 3.7 cm | 2.3 cm | 10.8 cm | c | 1.0 |

The video shows the trial with hose c hardened. It ends when `success()` holds: the whole hose and both fittings
lie in the tray, released, with both arms out to the sides.

## Shortcut and tier checks

Scripted policies run on the final code and scored by the scorer; the hardened hose was fixed to the position shown
for each run, so that each run tests the loophole it names.

| policy | hardened | what it does | score |
|---|---|---|---|
| nothing | b | no motion | 0 |
| no test, fixed pick a | a | lays hose a in the tray without flexing anything (it happens to be the hardened one; it lies wholly in the tray) | 0 (hardened hose entered the tray untested) |
| no test, fixed pick b | b | the same with hose b | 0 (same) |
| no test, fixed pick c | c | the same with hose c | 0 (same) |
| straight-lift "test" | c | lifts one fitting of each hose straight up 12 cm (no push), sets it down, then lays the hose whose far fitting moved most; the far fittings moved alike (-2.1 to -2.4 cm) | 0 (a straight lift is not a flex, so nothing was tested) |
| wrong hose | b | flexes all three (b's far fitting slides 9.2 cm), then lays the good hose c | 0 (good hose in the tray) |
| not released | a | full test, lays hose a in the tray, keeps both grippers on its fittings | 0 (not let go) |
| draped | c | full test, then lowers hose c with its fittings 18 cm apart near the far wall: both fittings in the tray, the loop of hose over the wall | 0.5 |
| test two, infer the third | b | flexes a and c (both slide about 3 cm), then lays the untested hose b | 1.0 (the two good hoses were tested, so b is known by elimination) |
| test one, guess another | b | flexes only a (slides 3.1 cm), then lays b, the right hose, without testing it | 0 (b entered the tray before it or the other good hose was tested) |
| missed the tray | a | full test, then lays hose a alongside the tray, 3 cm beyond its wall, on the bench | 0 (hardened hose never in the tray) |
| full demo | a / b / c | as described above | 1.0 each |

## How this differs

- Nearest existing task: finding the thin-walled tube among three look-alike cylinders. Same "one of three
  look-alikes" structure, but those objects are rigid and the test is different. Here the property is the stiffness
  of a deformable line, the test is bending it, and the result shows only in the hose's own shape and in how far it
  pushes its far end; laying the 60 cm hose in the tray then takes both arms.
- Wiggling ECG leads (bend each lead and watch a tester lamp): there an instrument reports an electrical fault; here
  there is no instrument, the hose's mechanical response is the reading.

## Paper fields

- Skill tags: measurement (interactive perception of stiffness), placement, sequencing.
- Industry: industrial and mobile-equipment maintenance (hydraulics workshop).
- Two-arm need: reach-forced (each fitting and each end of the tray only by the arm on its side), with a simultaneous
  two-arm carry for the lay.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "One of these hydraulic hoses has hardened and must be taken out of service. Put it in the red tray and leave the good ones on the bench, then let go."
```

Seeds: the hardened hose is drawn on every embodiment reset, uniformly over a, b, c, from the seed the runner passes
to `reset()` (each trial gets its own seed, so trials vary and a given seed reproduces). Without a seed, `HOSE_SEED`
plus the reset count seeds the draw, or fresh entropy is used; `HOSE_HARD=a|b|c` forces it for test runs only. The
draw only sets the hoses' bending stiffness; nothing visible changes. The layout is otherwise fixed
(`object_jitter=0`; the hoses are not jittered). Runtime: the demo takes 772 steps; a 1,200-step trial takes several minutes on one GPU.

## Simulation notes (fidelity)

- Hoses: 30 rigid 2 cm segments with bending springs (no twist, stretch or plasticity); hardening is modelled only
  as 5 times the bending stiffness (a real aged hose also cracks when flexed). Hoses weigh 75 g/m, lighter than most
  real 10 mm hose, so the sim stays stable; the real set is specified by the stiffness and flex-test checks above, not
  by weight alone.
- Fittings are rigid blocks (real: crimped). Flexing and release are judged geometrically from hose and gripper
  positions.

## Sources (opened; quotes verbatim)

- FAA Advisory Circular AC 43.13-1B, par. 9-29 d, "Inspections"
  (www.faa.gov/documentLibrary/media/Advisory_Circular/AC_43.13-1B_w-chg1.pdf): "Hydraulic and pneumatic systems are
  inspected for leakage, worn or damaged tubing, worn or damaged hoses, wear of moving parts, security of mounting for
  all units, safetying, and any other condition specified by the maintenance manual. A complete inspection includes
  considering the age, cure date, stiffness of the hose, and an operational check of all subsystems."
- Power Motion Tech, "A Guide to Recognizing the Causes of Hose Failure"
  (www.powermotiontech.com/hose-tubing/guide-recognizing-causes-hose-failure), Problem 2, Hard and Brittle Hose: "If
  hose becomes hard and brittle, and its cover cracks when flexed at room temperature, several issues can occur." and
  "If ambient heat is the cause, install heat guards or shields as required, and replace the hose assembly."
- IBT Inc., "When to Replace Your Hydraulic Hose: Warning Signs & Best Practices"
  (ibtinc.com/learn/when-to-replace-your-hydraulic-hose-warning-signs-best-practices/): "Old or heat-damaged hoses
  often become rigid or brittle." and "This makes them more prone to cracking and limits their ability to flex under
  system pressure."
- O*NET 49-9041.00 Industrial Machinery Mechanics, task: "Examine parts for defects, such as breakage or excessive
  wear."

None of these prescribes a numeric flex test; the push test, its thresholds and the acceptance checks are ours.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` is in this folder (hose simulation).
