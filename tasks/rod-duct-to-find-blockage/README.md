# Rod a duct to find where it is blocked, and mark the spot

A closed grey duct lies on the table far out in front of the robot, running from the robot's left to its right. Its
mouth is open on the left; its far end is capped. Somewhere inside, out of sight, the bore is blocked, and where
changes from trial to trial. A yellow fibreglass duct rod with a brass bullet tip and black bands every 10 cm lies on
the table in one wide bend, with its tip just inside the mouth. An orange marker block stands on a small stand on the right. The robot must push the rod into the duct until
it stops, use how much rod went in to work out where the blockage is, set the marker on the duct lid right over it,
pull the rod back out and let go.

Nothing outside the duct shows where the blockage is. The only way to find it is to feed the rod in until it stops
and read how much went in, either from the bands at the mouth or from the length of rod left outside, and then carry
that length along the outside of the duct, using the white ticks on the lid (every 10 cm from the mouth). The
rod is the probe: the information comes out through it.

Building and utility maintenance staff and line crews do this before pulling cable or rope through a conduit. A rod is pushed through the duct; where it stops is the
blockage, and the length of rod paid in tells the crew how far along to dig. A traceable rodder's sonde "helps trace
the duct route and can locate duct blockages or collapses", and a push-rod camera's footage counter "gives a rough
distance to any problem". Here there is no sonde and no counter: the bands on the rod and the ticks on the lid play
the part of the footage marks on a real rod.

Skills: tool use (a flexible rod pushed in short regrasped strokes), measurement (length of rod inserted, transferred
to a position along the duct), placement. Setting: building and utility maintenance (any conduit or duct run that
has to be cleared before cable or rope is pulled through it), not one trade. Two arms:
reach-forced (below).

## Sources

- O*NET 49-9052.00 Telecommunications Line Installers and Repairers, core task statement (about pulling cable through ducts, the job rodding prepares; it does not name rodding or locating a blockage): "Pull cable through ducts by hand
  or with winches." https://www.onetonline.org/link/details/49-9052.00
- Cabling Installation & Maintenance, "Toneable system locates underground duct blockages" (March 8, 2012): "The
  Toneable Duct Rodder System from General Machine Products (GMP) allows technicians to remain above-ground, without
  having to dig or trench, when they need to find duct blockages or otherwise map an underground duct system." and
  "The sonde, a self-contained transmitter that connects to the end of the duct rod, helps trace the duct route and
  can locate duct blockages or collapses."
  https://www.cablinginstall.com/16477802
- Sewer camera inspection guide (push-rod camera, the same length-paid-in principle): "Footage counters on the reel
  display how many feet of rod have been deployed, which gives a rough distance to any problem." and "Look for the
  locator marks if the technician traced a sonde signal, because those tell a future excavator where to dig."
  https://inspectandtest.net/guides/sewer-camera-inspection/
- Marked duct rods are sold for this: a Jameson Duct Hunter listing, "Includes 800 feet of 5/16-inch diameter
  traceable fiberglass rod marked every 5 feet".
  https://www.emisupply.com/jmn-12a-516-800m-jameson-12a-516-800m-800-x-5-16-duct-hunter-marked-rod

- Minimum bend radius of a 6 mm duct rod: the Condux 6 mm Mini-Cobra rod (Colonial Teltek part 08072815) lists
  "Radius (mm): 127" and "Conduit Size (mm): 25 32 38 51 64"; the Ritelite Midi Cobra 6 data sheet ("ø6mm fibreglass rod") lists a minimum bend diameter of
  200 mm (100 mm radius). https://www.colonialteltek.com/printitem/rods/6-mm-mini-cobra-rods/08072815 and
  https://altitude-safety.cdn.cloud-infra.co.uk/media/2024/03/28195355/Ritelite-Midi-Cobra-6mm-Information-Sheet.pdf
  The starting bend here has a 130 mm centreline radius, at or above both.

## Scene (station table; cm from the midpoint between the arm bases, x forward, y left, heights above the table)

| object | type | size | material (real rebuild) | position |
|---|---|---|---|---|
| duct | fixed square tube: floor, two walls, cap | 2.6 x 2.6 cm outside, 2.0 x 2.0 cm bore, 82 cm long | grey PVC trunking | centre line x 49, open mouth at y +34, capped end at y -48 |
| lid | fixed plate on the duct | 5 cm wide, 3 mm thick, 82 cm long, top 2.6 cm above the table | opaque grey PVC, screwed down | over the duct |
| ticks | thin strips on the lid edges | 12 x 4 x 0.8 mm, two per tick | white tape | every 10 cm from the mouth: y +24, +14, +4, -6, -16, -26, -36, -46 |
| blockage | block filling the bore, does not move during a trial | 1.9 x 1.9 x 7 cm | dark wooden block wedged in | face (the end towards the mouth) D cm from the mouth, D drawn at every reset (see Reset and sampling); hidden under the lid |
| duct rod | free flexible rod | 6 mm diameter, 90 cm, 40 g | yellow fibreglass rod | bullet tip 3 cm inside the mouth; straight out along +y to y +45, then one 180-degree bend of 13 cm centreline radius away from the robot (to x 75, as far as y +58), then straight back along -y at x 75 to its end near y +9 |
| bands | on the rod | 2 cm long | black tape | centres 10.4, 20.4, ... 80.4 cm from the bullet tip |
| bullet tip | rigid, on the rod's leading end | 14 x 6 x 6 mm, 6 g | brass | - |
| marker | free block | 2.5 x 2.5 x 1 cm, 10 g | orange-painted wood | on top of its post |
| marker post | fixed | 1.8 x 1.8 x 4 cm | grey PLA | x 20, y -52 |

## Reset and sampling

At every reset the blockage face distance D from the mouth is drawn uniformly from 53 to 75 cm (to 1 mm) with
`numpy.random.default_rng([seed, 1101])`, where `seed` is the episode seed the runner passes to reset (inspect-robots
gives one per episode); without one (the scripted-demo runner) it is `RD_SEED` (default 0) plus the number of earlier
resets in the process. The block is moved there before the rod is laid; it is a kinematic body, so nothing in the
trial can move it. The scorer reads its position each step, so every tier is measured from that trial's face. The
range is set by reach: 53 cm keeps the spot at least 18 cm right of the robot's midline, where offline IK puts the
right arm's gripper-down pose over the lid within 2 mm (nearer the midline it cannot hold the gripper pointing down), and 75 cm leaves the 7 cm block room before the cap.

Real rebuild: before each trial, pull the rod out, draw D (for example with the same generator, or a printed list of
random draws), push the block in from the mouth with a marked stick until its face is D from the mouth (it is a snug
fit and stays put), close the lid, lay the rod as in the table above, and put the marker back on its post. The person
resetting must not be in view of the robot's cameras while the lid is open.

Nothing outside the duct shows where the block is, so a policy that does not rod cannot do better than chance: a
fixed guess (a tick, the middle of the range, the far end of the range) is within 2 cm of the face on at most 4/22 of draws
(18%), within 4 cm on at most 8/22 (36%), and only if the rodding to the stop was done first (see Scoring).

## Closest tasks

- A task that pushes a plug through a bent conduit (cable pushed-pulled): the plug is pushed to the far end and taken out there; the far end is the goal and the plug is seen coming out. Here the rod's end is never seen and never comes out: where it stops is the unknown, and the output is a mark placed outside the duct at a distance read off the rod. A correct push with the marker anywhere else scores 0, and the marker cannot score without the rodding (it must come onto the lid only after the rod has reached the blockage).
- A task that lowers a probe down a pipe to a marked depth: the target depth is given and the probe hangs in view. Here the depth is not given; it is measured by the stop and then transferred to a second object (the marker) in another place.
- TK-X25UCA (fish tape): the tape is already through the conduit and pulls a cable back; there is no hidden obstruction and nothing to locate.
- TK-MG9LMB (rod into a box without touching the back) is a depth placement of a rigid rod in a box the robot can see into; nothing is located by the stop and nothing is marked elsewhere. TK-9VE3QM (hidden pipe detector) and TK-TF83Z9 (hidden weight in a bar) hide a state too, but neither feeds a flexible line blind and carries the inserted length to a mark. TK-DA2CEU (peg home past a hidden obstruction) tests stopping at an unexpected stop on a rigid peg; nothing is located or marked.

Stripped of the story, the act here is: push a flexible line blind into a closed channel until it stops, read the inserted length, place a marker at that distance along the outside, withdraw the line.

## Why two arms

Reach-forced. Offline position-only IK (any gripper orientation, joint limits kept, every link frame above the table;
each number is how far the arm's best solution stays short of the point):
- the right arm stays 39.4 mm or more short of the duct mouth (x 49, y +34) and 101 mm or more short of every sampled
  point of the rod's starting lay outside the duct, so it can neither pinch the rod nor push it in;
- the left arm stays 88 mm short of the marker on its post (x 20, y -52), so it cannot fetch the marker.

The left arm rods and withdraws; the right arm places the marker. With the gripper pointing down, each arm reaches its
own work points (rod pinch 10 cm out from the mouth, push end at the mouth, marker on its post, the lid from 18 cm
right of the midline to the far end of the range). The duct lies far out at x 49 so that the right arm cannot reach
its mouth.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py`, `cabledemo.py` and `vision.py` are in this folder (the cable simulation, the shaped
  start of the rod, the scripted-demo runner and the top-camera helpers the demo uses).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 3000 \
    --instruction "This duct is blocked somewhere along its length. Use the yellow rod to find where, put the orange marker on top of the duct right over the blockage, and pull the rod back out. Then let go."
```

## Scripted demo

```bash
RD_SEED=0 python cabledemo.py task.py demo_out --max-steps 3000
```

Closed loop on what the robot can observe: the top camera image, the arms' own positions and the fixed layout (the
duct, its mouth, the marker post). The script never reads the blockage, rod or marker positions from the simulator to
choose a motion. Before each stroke the left arm moves out of the camera's line of sight and the script traces the
yellow rod in the top image from the duct mouth outwards (a hue mask and a bead chain along it). The left arm pinches
the rod 10 cm along it from the mouth, turns that length onto the duct axis, pushes it in until the fingers are 2.2 cm
from the mouth, lets go and looks again. The rod length seen outside the mouth shrinks by what went in: a stroke that
takes in under 3.5 cm means the bullet has met the block, and a second such stroke confirms it. The depth read is
3 cm (the start lay) plus how much shorter the rod seen outside the mouth is than at the start: the robot's reading of
the rod left outside. The right arm then finds the marker on its post in the
top image, takes it and sets it on the lid at that distance from the mouth, while the left arm pulls the rod back out
in strokes (look, pinch 10 cm out with the fingers opened wide, draw 12 cm out along the axis, swing the pulled
length aside) until the camera sees no rod at the mouth. Both arms open and move clear.

In the recorded run (seed 0, blockage face 58.1 cm from the mouth) the rod went in over 10 strokes: 8 that went in
(5.2-8.4 cm each as seen by the camera), then two that took in -0.8 and 0.4 cm, which is the stop. The depth read was
58.1 cm, and the bullet's simulated position was also 58.1 cm in. The marker came to rest 1 mm from the spot over the
blockage face. Four withdrawal strokes took the rod out, and the trial ended with score 1.0 at step 1,722 (86 s).
`demo/` holds the videos, `setup.png`, `run.log` (the simulator's own readings are printed beside each step for
checking only) and `result.json`.

## Scoring (`score()`, evaluated at the end; `success()` = score 1.0, must hold 1 s)

Score 0 if any of these holds:
- the marker came onto the lid (its centre over the lid and less than 2 cm above the lid top) at any moment before
  the rod's bullet came within 1.5 cm of the blockage face (latched for the whole trial: a marker guessed first and
  rodded afterwards does not count);
- the bullet never came within 1.5 cm of the blockage face (latched once it does: the rodding must reach the stop);
- at the end a gripper centre is within 3 cm of any point of the rod or within 4 cm of the marker centre (not let go);
- the marker is not resting on the lid at the end (centre over the 5 cm lid, bottom within 5 mm of the lid top, still).

Otherwise, by the distance along the duct between the marker centre and this trial's blockage face (read from the
block's position, which the reset draws; see Reset and sampling):

| score | end state |
|---|---|
| 1.0 | marker resting on the lid within 2 cm of the spot over the blockage face, no part of the rod left in the duct, both hands off |
| 0.6 | marker within 2 cm of the spot, but the rod (or part of it) is still in the duct |
| 0.3 | marker 2-4 cm from the spot (rod in or out) |
| 0 | marker more than 4 cm from the spot, or any zero condition above |

The lowest nonzero tier needs the whole rodding (53-75 cm of rod fed in strokes until it stops) and a marker placed
within 4 cm of a point that nothing outside the duct shows and that changes every trial; touching, pushing a little
or holding the rod earns nothing. In a real trial: void the trial if the duct moves; measure with a tape from the
mouth.

Limit: the score cannot tell how the robot chose the marker position once the rod has reached the blockage. Because
the face is drawn over a 22 cm range, a policy that rods to the stop and then puts the marker at a remembered or fixed
spot is within 4 cm (0.3) on at most 36% of trials and within 2 cm (1.0) on at most 18%; the far end of the range (75 cm from the mouth) is within 2 cm on 9% of draws and within 4 cm on 18%, and scored 0 on the
seed shown below. The rodding itself cannot be skipped (a marker set first, or without the rod reaching the blockage,
scores 0), and the marker is not allowed onto the lid, even briefly, with its centre under 2 cm above the lid top
before the rod has reached the stop; carrying it higher than that over the lid is fine.

`distance()`: marker centre to the spot on the lid over the blockage face (m). `progress()`: stage diagnostic (rod
depth reached, rodded, marker on the lid, within 2 cm, rod out, released, no early mark); not the score.

## Shortcuts (scripted variants, `RD_DEMO=<variant>`)

All runs below use seed 0 (blockage face 58.1 cm from the mouth). The variants that rod are recorded runs, because
they use the camera-driven feed. The others ran without recording.

| variant | what it does | score | why |
|---|---|---|---|
| nothing | waits | 0.0 | the rod never reaches the blockage |
| onearm_right | the right arm alone sets the marker on the middle of the lid; no rodding (the right arm cannot reach the rod) | 0.0 | marker on the lid before any rodding (latched), and the rod never reached the blockage |
| onearm_left | the left arm alone rods to the stop and withdraws; the marker stays on its post (the left arm cannot reach it) | 0.0 | marker not on the lid |
| early | test-only variant that reads the simulator's blockage position: the marker is set exactly over the blockage first, then the rod is fed to the stop and withdrawn | 0.0 | marker on the lid before the rod reached the blockage (latched): a right guess placed first still scores 0 (it ended 0 mm from the spot, rod out, hands off) |
| fixed75 | full rodding to the stop and withdrawal, but the marker goes 75 cm from the mouth (far end of the range), whatever was read | 0.0 | marker 16.8 cm from this trial's blockage face |
| tier06 | full rodding to the stop, marker set from the depth read; the rod is left in the duct (not withdrawn) | 0.6 | marker 1 mm from the spot, rod tip still inside the mouth |
| tier03 | full rodding and withdrawal, but the marker goes 3 cm short of the spot read | 0.3 | marker 2.8 cm from the spot, rod out, hands off |

Variants `partway`, `farend` and `hold` are in the script but have no recorded run on this layout, so they are not evidence here: their zero outcomes follow from the score() branches only. The tier06 and tier03 rows are recorded runs of the final demo build (seed-0 blockage face, 58.1 cm).

## Runtime and seeds

- Run limit 3,000 steps (2.5 minutes at 20 Hz), raised from 2,400 because the camera-driven demo looks at the rod
  before every stroke. The demo succeeds at step 1,722 (86 s). On a shared GPU the simulator ran at 0.1-0.4 step/s, so a
  full trial can take 2-8 hours of wall time.
- The blockage position is drawn per episode from the episode seed (see Reset and sampling); the rod is laid in the
  same shape at every reset and the marker is not jittered with `object_jitter=0`. `VARIATION` lists small marker
  offsets for a later variation runner.

## Simulation simplifications

- The 6 mm rod is 45 rigid 2 cm segments with bending springs (bend stiffness 0.02 N m/rad per joint; no twist,
  stretch or plasticity); the brass bullet is a rigid box. Its friction coefficient 0.5 is chosen, not measured
  (PhysX averages it with the fixed parts' friction).
- The bore is 20 mm. That is narrower than the smallest conduit the 6 mm rod is listed for (the Condux listing above
  gives conduit sizes from 25 mm). It is kept narrow so the rod cannot snake from wall to wall inside, and the length
  pushed in stays equal to the distance to the stop. A rebuild that follows the rod maker's sizes would use 25 mm
  conduit.
- Duct, lid and cap are rigid fixed boxes; the blockage is a rigid kinematic box that only the reset moves. Real
  rebuild: grey square PVC trunking (20 mm bore) with an opaque lid screwed down, a snug wooden block pushed in to the
  drawn distance, a fibreglass rod with black tape bands, a wooden marker on a small post.
- The bands are painted rod segments and the ticks thin strips; the robot sees them only in the cameras. The robot
  sees the stop only as the rod not going further in (no force sensing is modelled or needed).
- The start lay is written into the rod at reset and settles under gravity and friction for 2 s; the rod keeps no
  coil set.
