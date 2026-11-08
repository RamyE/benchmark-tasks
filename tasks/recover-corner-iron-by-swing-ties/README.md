# Recover a buried corner iron from two witness-nail tapes and stand the stake on it

A survey mock-up on the table in front of the robot. A tan mat is the "sand" over a buried corner iron; nothing on
the mat shows where the iron is. Two black witness nails stand far out at the two sides of the table, one in front of
each arm. Each nail holds the zero end of a white tie tape, and each tape carries one red band: the iron's distance
from that nail (its "tie"). Each tape lies in a loose U beside its own nail. A free orange marker stake stands on the
mat. The robot must find the iron and stand the stake on it, then let go.

The tested ability is recovering a point from two measured ties ("swing ties"). Drawn taut from its nail, a tape's
red band sweeps an arc of points at the right distance from that nail; the iron is where the two arcs cross. The
robot has to swing each tape out straight and lightly taut from its nail, see where the bands fall, and stand the
stake where the two bands meet. The two arcs cross twice: one crossing is on the mat (the sand, where the iron is
buried), the other is 36 cm further out, off the mat. A stake set by eye, at the middle of the mat, between the nails,
or at one band only ends far from the iron and scores 0.

Surveyors keep each monument tied to nearby reference marks so that it can be found and reset if it is buried or
disturbed, and a point fixed by distances from two known stations is the crossing of two arcs (a distance-distance
intersection), which has two solutions. O*NET 17-3031.00 (Surveying and Mapping Technicians) lists "Search for section
corners, property irons, or survey points." and "Set out and recover stakes, marks, or other monumentation."

Sources (each opened and quoted verbatim):
- O*NET 17-3031.00 Surveying and Mapping Technicians, https://www.onetonline.org/link/summary/17-3031.00: "Search for
  section corners, property irons, or survey points." and "Set out and recover stakes, marks, or other monumentation."
- Iowa DOT Construction Manual, 4.20 "Land Monuments and Road Centerline",
  https://ia.iowadot.gov/erl/archiveoct2012/CM/content/CM%204.20.pdf: "Placement of reference markers is critical since
  they are used to find the centerline control point at some future date and, more importantly, to restore a control
  point to its original position if disturbed." and "Iron pins should be driven so that top of pin is 125 to 150 mm (5
  to 6 inches) below ground surface."
- Wisconsin State Cartographer's Office, "Survey Monument Maintenance",
  https://www.sco.wisc.edu/surveying/survey-monument-maintenance/: "A map showing a monument with distances and
  directions to surrounding objects is called a "tie sheet"." and "The tie sheet helps others find the corner and
  documents the corner's location in case it is disturbed or destroyed."
- Kellie, Johnson and Bradley, "Trilateration as Basis for Radial Staking", Journal of Surveying Engineering 110(1),
  1984, https://iro.uiowa.edu/esploro/outputs/journalArticle/Trilateration-as-Basis-for-Radial-Staking/9984197233002771
  (abstract): "staking point positions determined by averaging distance-distance intersections to three perimeter
  control stations closely approximated a rigorous least squares solution for the same position."
- FAO training manual, "Measuring Horizontal Distances", section 2.4,
  https://www.fao.org/fishery/docs/CDrom/FAO_Training/FAO_Training/General/x6707e/x6707e02.htm: "you should pull bands
  and tapes tight, so that they do not sag" and "Keep the band or tape straight and tight".
- Esri, "Create a point or vertex at the intersection of two distances",
  https://doc.esri.com/en/arcgis-pro/latest/help/editing/create-a-point-or-vertex-at-the-intersection-of-two-distances.html:
  "The diagram shows intersections P1 and P2 measured from origin locations at A and B." (two distances give two
  candidate points; one must be chosen).

The instruction states only the goal ("The corner iron is buried under the sand. The two tapes are tied to its
witness nails and each red mark is the iron's distance from that nail. Find where the iron is, stand the orange
stake on it, then let go."). It does not say that the tapes must be drawn straight, that the answer is where the two
arcs cross, or which crossing to take.

Skills: measurement (two taped distances), placement/pinning (stake stood on the found point), sequencing (both
tapes drawn, then the stake). Industry: land surveying and construction layout (recovering and resetting survey
monuments). Two arms: reach-forced (below): each tape lies only within its own arm's reach.

## Scene (station table; cm from the midpoint between the arm bases, x forward, y left, heights above the table)

| object | type | size | material (real rebuild) | position |
|---|---|---|---|---|
| sand mat | fixed sheet | 24 x 26 cm, 2 mm thick | tan felt or cork sheet, taped down | x 22 to 46, y -12 to +14 |
| buried iron | none (a point in the score only) | | (a small pin under the mat, or a pencil cross under it, in a real rebuild) | x 27, y +3, under the mat |
| witness nails | fixed posts, 2 | 12 mm round, 40 mm tall | black-painted steel or PLA peg, screwed down | nail 1 at x 45.6, y +41.3 (left); nail 2 at x 45.5, y -41.3 (right) |
| tie springs | joint, one per tape | about 20 mm long, 150 N/m | small extension spring from the nail to the tape's zero ring | each tape's zero point sits 14 mm inboard of its nail: tape 1 at (45, +40), tape 2 at (45, -40); the zero end swivels freely and is drawn out against the spring when the tape is pulled |
| tie tape 1 | free cord, zero end on the left nail | 6 mm round, 46.1 cm, 42 g | white braided cord (a marked tag line) | laid from (45, +40) 15 cm away from the robot along x, a U bend (3.5 cm radius) outboard, then back towards the robot along y +47; its tab ends near (33, +47) |
| tie tape 2 | free cord, zero end on the right nail | 6 mm round, 51.5 cm, 48 g | as tape 1 | from (45, -40) 18 cm along x, U bend outboard, back along y -47; tab near (34, -47) |
| red bands | marks on the tapes | 2.2 cm long, all round | red tape wrapped round the cord | band centre 40.7 cm from the zero point on tape 1, 46.1 cm on tape 2: with the spring drawn out 5 mm they lie 41.2 / 46.6 cm from the zero point, the iron's distance from each |
| tabs | rigid blocks on the tape ends | 20 x 20 x 10 mm, 6 g | white PLA, glued on | at the free end of each tape |
| marker stake | free cylinder | 26 mm round, 60 mm tall, 50 g | orange-painted wood dowel | standing on the mat at (36, -2) |

The two arcs (tape 1's band from nail 1, tape 2's band from nail 2) cross at the iron (27, +3), on the mat, and again
at (63, +3), off the far edge of the mat. Distances from the iron: the stake's start 10.3 cm; middle of the mat 7.3
cm; midpoint between the nails 18.2 cm; the crossing with the two band distances swapped 6.0 cm.

## Why two arms

Reach-forced. Each tape lies wholly within reach of the arm on its side only, so drawing both tapes (which the score
requires) needs both arms. Offline position-only IK (any gripper orientation, 16 starts per point; `scratch/ik_final.py`,
output `scratch/ik_final.out`), at every point of each tape's start layout every 2 cm from its zero point to its tab
(plus the nail), with the TCP 1 and 3 cm above the table:
- tape 1 (left): the right arm misses every point by at least 72.0 mm (nearest: the zero point at (45, +40)); the
  left arm reaches the tape's end (0.0 mm) and the tab drawn out past the iron, (24.6, -1.9) cm, 2.5 cm up (0.0 mm);
- tape 2 (right): the left arm misses every point by at least 72.0 mm; the right arm reaches the tape's end (0.0 mm)
  and the drawn-out tab position (24.9, +7.9) cm (0.0 mm);
- the stake start, the iron's spot and the other crossing (63, +3) are within reach of both arms (0.0-0.1 mm).
A cross-midline map (`scratch/ik_edge.py`, `scratch/ik_edge_*.out`) shows how far an arm reaches across: at x 30 cm the
right arm reaches y +41 cm, at x 40 cm +36 cm, at x 50 cm +29 cm; the tapes lie at |y| >= 40 cm with x >= 35 cm, well
outside that edge. Once a tape has been drawn across to the iron it is within the other arm's reach, but each tape
must first be swung out from its own nail by its own arm.

## How this differs from similar tasks

- Set a pin at the one spot equally far from three survey marks (board TK-56LF8F, the nearest board task): the pin is
  placed by geometric reasoning from marks the robot can see, with no line in the task. Here the target is hidden and
  is found only by working two lines: each tape is meaningful only while it is drawn straight from its nail, and the
  point is where two drawn tapes' marks meet.
- Hold a tape measure pulled out along a board and mark at 300 mm (board TK-HXAS2M): one tape along one fixed line, one
  dimension, the mark placed at a printed reading. Here the direction of each tape is the unknown: each tape is swung
  through an arc, and two tapes from two nails fix a point in the plane (two dimensions), with a choice between two
  crossings.
- Lay weighted lines in batter-board notches so they cross over a corner pin (our earlier task, the nearest of ours):
  the point is given (a visible pin) and the lines are chosen to pass over it. This task is the inverse: the lines'
  lengths are given and the point is unknown. Mark a cable into thirds and pull a cable to a length mark (ours) work
  one line in one dimension. Our "bring a mark on a line to a point" tasks (slide a ring along a taut wire to a mark,
  centre a rope in an anchor ring) move the line until its mark meets a given point; here no point is given, the
  marks are read and the point is built from two of them. Peg a taut row line at marks lays one line straight and
  fixes it; here the lines are only measuring tools and are left loose.
- No board task or public set (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench, RoboDojo) recovers a hidden
  point from two taped distances.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (the cable simulation, the start-layout settle
  and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "The corner iron is buried under the sand. The two tapes are tied to its witness nails and each red mark is the iron's distance from that nail. Find where the iron is, stand the orange stake on it, then let go."
```

Run limit: 2,400 steps (2 minutes at 20 Hz) per trial; the scripted demo needs 1,470 steps (the run ends once success has held 1 s). The layout is fixed (`object_jitter=0`; the tapes are not jittered)
and is restored exactly at every reset (the tapes are laid into their U shapes and settled for 2 s).

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

The left arm takes tape 1's tab, swings it round its nail (turning the zero end away from the nail) onto the line
towards the mat, and draws it out in 1 cm steps, then 1.5 mm steps, until the tie spring stays out about 5 mm (the tab
is held low, just above the mat, so the taut tape pulls it level). It reads the red band's distance from the zero
point, then swings the tape on round, off the mat, and lays it down pointing back towards the robot (if the tape were
still near the iron's spot it would take the tab again and repeat). The right arm draws tape 2 the same way, but along
a line turned 15 deg off the nail-iron line (any line from the nail works: the band sweeps an arc), so the drawn tape
passes clear of the iron's spot, reads it and lets go of it where it lies. The demo then computes the crossing of the two measured arcs that lies on the mat (the
other crossing is 36 cm further out), and the right arm stands the stake there with one closed-loop correction from
the stake's measured position, lets go and backs off. Result: task score 1.0, success, 1470 steps, stake centre 4.4 mm from the iron (`demo/result.json`, `demo/setup.png`, three
camera videos). Measured band distances 41.45 / 46.80 cm (true 41.15 / 46.62 cm) put the computed crossing
at (26.4, +3.0) cm, 0.6 cm from the iron; the other crossing came out at (63.6, +3.0) cm. The straightness gate latched each tape as drawn at spring
extension 5.3 / 3.1 mm with the band 40.47 / 46.39 cm from the zero point (needs at least 39.65 / 45.12 cm). The wrist cameras show the red band clearly at 320 x 176 while a tape is drawn.

## Scoring

`score()` is read at the end; `success()` (score 1.0 with the stake and both tapes at rest) must hold for 1 s.
Latched for the whole trial (never reset): each tape's "drawn taut" flag (at some step its zero end is drawn out at
least 3 mm against the spring and its red band is at least as far from the zero point as the iron's distance minus
1.5 cm, i.e. the tape is pulled straight so the band lies on its arc; a 3 mm tug on a tape still lying in its U does not
count), the over-pull flag (a zero end drawn out more than 20 mm), the knocked-over flag
(stake tilt above 20 deg at any step).

Score 0 if any of these holds:
- a tape was never drawn taut and straight (as above);
- a tape was ever drawn out more than 20 mm against its spring (over-pull: a hard yank moves the zero end and spoils
  the measurement);
- the stake ever tilted more than 20 deg;
- at the end a gripper TCP is within 4 cm of any point of either tape, either tab or the stake (not let go);
- at the end the stake is not standing upright (tilt 10 deg or more) with its base on the mat or table.

Otherwise, by the horizontal distance from the stake's centre to the iron:

| score | end state |
|---|---|
| 1.0 | stake standing within 1.2 cm of the iron, both tapes drawn, everything let go |
| 0.6 | within 2.5 cm |
| 0.3 | within 4 cm |
| 0 | further, or any rule above broken |

In a real rebuild: mark the iron with a pin under the mat, read the stake's offset with a ruler after lifting the mat
corner, and watch each tie spring (a 3 mm and a 20 mm mark on a scale beside it) on video.

## Shortcut variants

Test only: `TIE_CHEAT=<name> python cabledemo.py task.py out`. Evidence: `scratch/sc_<name>.json` (result) and
`scratch/sc_<name>.log` (the demo's own log lines), all run after the last edit of `task.py`.

| variant | what it does | score | why |
|---|---|---|---|
| `none` | nothing moves | 0.0 | tapes never drawn; stake 10.3 cm off |
| `guess` | stake straight onto the iron's spot (known layout), tapes untouched | 0.0 | stake 0.5 cm from the iron but no tape drawn taut |
| `tug` | each tape gripped about 4 cm from its zero end and tugged until its spring is out 3-5 mm (3.3 / 3.9 mm), never straightened; then the stake on the iron's spot | 0.0 | bands still 7.0 cm from the zero points, so neither tape counts as drawn; stake 0.5 cm off |
| `onearm` | right arm only: draws tape 2, then stands the stake on the spot | 0.0 | stake 0.4 cm off, tape 1 never drawn |
| `over` | demo, but tape 1 hauled on until its spring is out 30 mm | 0.0 | over-pull latched (spring out over 20 mm), though the stake ends 0.65 cm from the iron |
| `hold` | demo, but the gripper keeps hold of the stake | 0.0 | stake 0.54 cm off, not let go |
| `mirror` | both tapes drawn, stake towards the other crossing (as far out as reachable) | 0.0 | 10.2 cm off |
| `centre` | both tapes drawn, stake at the middle of the mat | 0.0 | 7.6 cm off |
| `mid` | both tapes drawn, stake on the line midway between the nails | 0.0 | 10.9 cm off |
| `swap` | both tapes drawn, crossing computed with the two readings swapped | 0.0 | 5.8 cm off |
| `t06` | both tapes drawn, stake set 1.9 cm from the iron | 0.6 | 2.26 cm off |
| `t03` | both tapes drawn, stake set 3.2 cm from the iron | 0.3 | 3.62 cm off |
| `knock` | demo, but while letting go the open fingers shove the stake sideways | 0.0 | stake knocked over (tilt 90 deg, latched), lying 5.1 cm off |

Limit: the layout is fixed (one start state, as the rules require), so the iron's spot never moves. A policy that
had memorised it from training could still pull each tape straight, ignore the bands and stand the stake there; the
score cannot tell that apart from a real reading, but it does require each tape to be drawn straight from its nail
(`guess` shows the draw is required, and `tug` shows a 3 mm tug of a tape lying in its U does not count). Blind chance
is small: a stake stood at a random spot on the mat (24 x 26 cm) lands within 1.2 cm of the iron 0.7% of the time
and within 4 cm 8.1% of the time, and only scores if both tapes were also drawn taut and nothing was over-pulled.
