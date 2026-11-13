# Pay a multicore off a free-turning drum to its cradle without leaving the drum running unheld

A cable drum (grey 5 cm hub with a 12 cm top flange, 0.7 kg in all, and a white bar on top that shows it turning)
stands on a vertical axle at the left of the table. A red 6 mm multicore is wound about 2.5 turns round it, its inner
end fixed to the hub; the tail and a 35 mm connector lie out toward the right. The robot must pay the cable off along
a yellow-taped run and rest the connector in a white cradle at the far right, keeping the drum under one turn per
second and **never letting go of the cable while the drum turns**.

The rules protect the coil. The drum has inertia and no brake: if the pull stops dead or the cable is let go, the
drum runs on for about a second and throws its turns loose. Paced payout keeps the coil tight (about 1 cm standoff in
the demo); stopping dead from a brisk pull loosens it past the 2 cm limit. The drum is a small 12 cm cable reel with a
free-turning axle and no brake, so the pull itself is paced.
The occupation is electricians, whose O*NET task (47-2111.00, https://www.onetonline.org/link/summary/47-2111.00) is to
"pull insulated wires or cables through the conduit to complete circuits between boxes"; the cable comes off a reel or
drum that turns as it is pulled. O*NET does not mention reels; that link is the task's premise, not a quoted statement.

The tested ability is steady, rate-limited payout from a free drum through one hand-over: the hand that lets go must
first slow the pull and wait for the drum to stop.

Both arms are needed (reach-forced, offline IK over 13 grip yaws): the connector starts at x 32.5, y +14.5 cm, and the
right arm's straight-down grip ends near y +9 cm at that depth, so it cannot take the connector or the cable behind
it. The cradle is at x 41.4, y -23.2 cm, and the left arm has no solution there or at x 40, y -10 cm. So the left arm
pays the cable out to the middle of the table, slows, waits for the drum to stop and sets the connector down; the
right arm takes it from there to the cradle.

## Closest tasks

One board task scores a pull-to-length-mark from a feed channel (TK-EBGGYD); another scores pulling a reel line taut between two marks (TK-PK8X7K). Others wind a cord onto spools or posts, unwind turns from a fixed post, let a spring reel retract a cord
under hand control, or pull a cable to a length mark from a channel. Here the store is a free-turning drum that
the pull itself drives: the ability tested is pacing that pull (a drum-speed limit) and never leaving the drum
turning unheld at the hand-over. Nothing pulls the cable back, and the cable is not wound or unwound round a fixed
part.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py`, `cabledemo.py` and `vision.py` are in this folder (cable simulation, pre-wound
  start, scripted demo runner, top-camera estimates for the demo).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 900 \
    --instruction "Pay the red cable off the drum along the yellow-taped run and rest its connector in the white cradle, then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 900
```

The demo steers only from what the robot can observe: the top camera image, its own gripper poses and the fixed
layout. It finds the black connector in the top image (dark pixels near where it should be, back-projected onto the
connector's top face; their long axis gives the grip yaw) and judges the drum still when the image of the drum top
(white bar on the grey flange) stays unchanged over two 0.4 s intervals while the arms hold still. The left arm pinches the connector across its body (wrist
camera looking along the run toward the cradle) and pays the cable out along the run to the middle of the table over
3.5 s, lowers it, waits until the bar is still, lets go and leaves. The right arm finds the connector where the left
set it down, takes it, carries it 15 mm above the table to the cradle over 3.5 s, lowers it into the seat, waits for
the bar to stop, lets go and rises.

## Scoring

Holding is checked as a real pinch, not by nearness. Each control step, for each arm, the pad-to-pad gap is read
from the two finger joints (less the 3.3 mm pad offset measured on the 16 mm connector) and the gripper frame from its
pose. An arm holds the cable only if:
- connector: the gap is stopped at the connector's own width across the fingers (taken from the connector body's
  pose; 16 mm when pinched across), from 4 mm under to 2 mm over it, and the connector centre lies between the pads:
  within 10 mm of the tool point across the fingers (the two fingers can stop off-centre), overlapping the pads'
  height (15 mm above to 13 mm below the tool point) and their 20 mm width;
- or cable: the gap is 2 to 8 mm (6 mm cable, -4/+2 mm) and some point of the cable lies inside the pad box, within
  half the gap plus 1 mm of the tool point across the fingers.
A gap under 2 mm never counts. A gripper closed on nothing shuts to 0 mm, and a closed gripper hovering beside or
above the cable has nothing between its pads, so neither counts as holding.

`score()` latches three violations each control step for the whole trial:
1. the drum turns faster than 360 deg/s (one turn per second);
2. the drum turns faster than 20 deg/s while neither arm holds the cable or connector by the pinch test above;
3. a wound turn stands more than 2 cm off the wound radius, with cable further along still on the hub.

Then, at the scoring step:
- 0: any violation latched; or a gripper tool point within 3 cm of the cable or connector (not let go); or the
  connector centre not inside the cradle seat lying on the table.
- 0.5: connector seated and released, but some cable point more than 6 cm from the axle lies over 4 cm from the run
  centreline (outside the tape lines) or past the cradle's far wall, or the drum still turns faster than 3 deg/s.
- 1.0: connector seated and released, all that cable between the tape lines, drum under 3 deg/s.
`success()` is score 1.0 with the cable at rest (every point under 2 cm/s); the trial ends after 1 s of it.

Measuring these in a real trial: film the drum from directly above at 30 fps or more, with a ruler laid on the
flange. Drum speed is read from the white bar: 20 deg/s is one 18 deg step (a twentieth of a turn) between frames
0.9 s apart, 360 deg/s a full turn per second, 3 deg/s under 1.8 deg of bar movement in the last 0.6 s. A loose turn is
measured on the same video against the ruler: any turn whose outer edge stands more than 2 cm off the wound coil
while cable further along is still on the hub latches 0. The tape-line and cradle checks are read from a photo
of the table at the end; the 3 cm release check is a ruler held from each fingertip to the cable.

Variants (env `DRUM_CHEAT`), test only: `nothing` (waits), `hold` (seats the connector, keeps gripping), `yank`
(pulls over one turn per second), `overrun` (stops dead at peak pull speed, the drum runs on and throws a turn
loose), `letgo` (opens the fingers mid-pull while the drum still turns), `hover` (opens mid-pull, then closes the
fingers fully about 1.5 cm above the connector while the drum coasts: closed and near, but not pinching), `onearm`
(left arm alone pushes the connector as far as it reaches) all score 0; `tier05` (seat the connector, then drag the
middle of the run out over the tape) scores 0.5.

Runtime: about 45 s of sim time per trial (900 steps at 20 Hz). Seeds: the layout is fixed with
`object_jitter=0`; the cable is not jittered (its wound start is written at reset and settled, deterministic).
