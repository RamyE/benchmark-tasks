# Tension a cable on its spring gauge and lock it in a toggle clamp

A cable-assembly fixture on the table. A black 6 mm cable, 46 cm long, is hooked at one end to the carriage of a spring
tension gauge on the left: a yellow carriage on a grey slide, pulled back by a 25 N/m spring (1.0 N at the gauge's zero),
with a green band on the rail marking the target pull. From the gauge the cable runs along the fixture line through a
hold-down toggle clamp: a red 10 cm bar hinged on a grey base, standing upright (open), with a black rubber pad under it.
The bar is sprung over centre: it stays upright on its own, and once pushed about 6 degrees past upright it snaps down and
presses the pad on the cable with about 6 N. Past the clamp the cable ends in an orange plug. At the start the cable lies
slack: a loop bulges away from the robot between the gauge and the clamp, and the tail curls towards the robot.

The goal is a cable locked in the clamp at the gauge's target tension, straight along the line, with both hands off.
Taking up the slack and drawing the carriage into the green band takes a hand on the plug, and that hand has to keep the
tension while the clamp is shut, so the other hand must close the clamp. Let go before the clamp shuts and the spring pulls
the carriage back to zero; shut the clamp first and pulling the plug only drags cable through the pad (scored as a fault)
without moving the gauge. The pad holds the cable by friction only, so the gauge reading after release shows whether the
clamp really holds the tension.

## Requirements

yam-isaac-lab-arena with Isaac Lab-Arena 0.3.1 installed (see that package's README). `cablekit.py` (cable simulation),
`cableshape.py` (lays the cable in its slack start shape at reset) and `cabledemo.py` (scripted-demo runner) are in this
folder.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Lock the black cable in the red toggle clamp with the tension gauge's yellow carriage held in the green band, the cable straight along the fixture line, then let go of everything."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

The demo chooses every motion from the top camera image, the arms' own poses and the fixed layout (it never reads object
poses from the simulator). The right arm finds the orange plug in the top image, pinches it across its long axis, lays the
tail onto the fixture line and draws it away from the gauge in small steps, re-reading the yellow carriage's position from
the camera after each step until the carriage sits in the middle of the green band (3.5 cm drawn, about 1.9 N). It holds
there while the left arm, fingers closed, pushes the upright bar over centre from the robot side; the bar snaps shut on the
cable. The camera confirms the bar lies flat, the right arm opens and both arms park beside their bases. The carriage
stays in the band.

The demo scores 1.0 (success after 1 s still) in 385 steps.

## Scoring

`score()` runs at the end of the episode. It returns 0 if any of these hold:
- any object state is not finite, or the gripper positions cannot be read;
- either gripper's TCP is within 3 cm of the cable, the plug, the clamp bar, the pad or the gauge carriage;
- the bar is more than 3 degrees above flat (clamp not shut);
- the cable is not under the pad with the pad on it (pad face within 2 mm of the cable surface);
- any cable point from the gauge to 8 cm past the clamp is more than 12 mm from the fixture line;
- the carriage is drawn less than 1 cm from zero (no tension held);
- at any time after the clamp shut, the cable was pulled more than 1 cm through it (latched for the whole run).

Otherwise the score is 1.0 when the carriage is drawn 2.5-4.5 cm (front face over the green band) and the cable between
the gauge and the clamp is within 6 mm of the line, and 0.5 when it is clamped and held but under- or over-tensioned
(drawn 1-2.5 cm or more than 4.5 cm) or not straight. `success()` needs 1.0 with the cable and carriage still for 1 s.

In a real trial: look from above with a ruler at the carriage face against the band, check the bar is flat and both hands
are clear; a tape flag on the cable at the pad shows any drag after closing.

Runtime per trial: 2,400 steps (2 minutes at 20 Hz). The layout is fixed with `object_jitter=0`; the cable's start shape is
written at reset and is the same every trial. `progress()` gives a stage diagnostic; `VARIATION` lists seeded ranges.

## Shortcut variants

Test-only variants, chosen with `TOGGLE_CHEAT=<name>` in the local demo script (not part of the task files); each was
run and scored as listed:

| variant | what it does | score | why |
|---|---|---|---|
| `none` | no action | 0.0 | bar open, carriage at zero |
| `clamp_slack` | one arm shuts the clamp, no tensioning | 0.0 | carriage at zero (no tension held) |
| `let_go` | tension to the band, let go, then shut the clamp | 0.0 | spring pulled the carriage back to zero before the clamp shut |
| `clamp_first` | shut the clamp, then pull the plug as far as the demo does | 0.0 | the cable slid out from under the pad; carriage back at zero once let go |
| `hold` | full demo but the plug stays pinched | 0.0 | gripper within 3 cm of the plug |
| `under` | tension only to about 1.4 cm, then clamp | 0.5 | clamped and held, below the band |
| `over` | tension to about 5.4 cm, then clamp | 0.5 | clamped and held, above the band |

## How this differs from similar tasks

Other tasks set a rigid lever or tensioner to a pointer band and lock it, or close a clamp on a part. Here the tension is
carried by a slack, deformable cable: the slack must be taken up through the open clamp, the cable kept straight on the
line, and the tension held by one hand while the other shuts an over-centre clamp on the cable itself. The result is read
after both hands let go, from the gauge, so a clamp that does not hold, or a cable tensioned and released too early, fails.

## Sources

- US 10,411,444 B2 (background): "fixing the ends of the wire so that the wires may be pulled taught as they're placed on
  the peg board". https://patents.google.com/patent/US10411444
- Bureau of Naval Weapons OP 2230, "Workmanship and Design Practices for Electronic Equipment" (via a cable-lacing
  reprint): "Another method of anchoring conductor ends on a harness jig is to attach springs on the board near the
  termination point of the wire." http://www.dairiki.org/hammond/cable-lacing-howto/
- Te-Co toggle clamp catalogue: "The holding capacity listed for each clamp refers to the maximum external force that the
  clamp can safely resist in its locked position"; vertical-handle clamps "Bar Opens: 100°", "Spindle Type: Bonded
  Neoprene". https://www.te-co.com/wp-content/uploads/2023/11/toggleclamps.pdf
- O*NET 51-2022.00 Electrical and Electronic Equipment Assemblers: "Position, align, or adjust workpieces or electrical
  parts to facilitate wiring or assembly." https://www.onetonline.org/link/summary/51-2022.00
