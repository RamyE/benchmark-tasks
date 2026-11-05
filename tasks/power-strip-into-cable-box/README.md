# Hide a power strip in a cable box, one cord per end slot

A home cable-management task. An open cable box sits fixed to the table in front of the robot. It is dark grey,
23 x 11 cm inside, 6 cm tall, with 1 cm walls and a 4 mm floor, and its long side runs along the robot's left-right
axis. Each end wall has a cord slot: a 14 mm wide U notch cut from the rim down to 1.5 cm above the table. The box's
lid (a 13 x 25 cm, 6 mm thick grey plate with a small handle on top) lies flat on the table behind the box, on the
left. Nearest the robot lies an unplugged white power strip (180 x 50 x 35 mm, 150 g) along the left-right axis. Its
black 7 mm power cord (32 cm, mains plug on the end) leaves its right end face and lies straight to the right. A blue
5 mm USB cable (30 cm, USB-C plug on the end) leaves its left end face and lies straight to the left.

The robot has to:

1. put the strip into the box;
2. lead the power cord out through the right-hand slot and the USB cable out through the left-hand slot;
3. put the lid on flat;
4. let go.

Retail cable boxes such as the Bluelounge CableBox are sold for this: the strip sits inside and its cords leave
through openings at the two ends (retailer copy: "Two outlets run on both sides of CableBox to allow for necessary
cables to come out"). The task is about tidiness; it makes no claim that boxing a power strip is safer. A cord left
over a wall instead of in its slot ends up pinched under the lid, and the lid sits crooked on it. That outcome
scores 0. Both arms are needed: each end slot is within reach of only the arm on its side (checked with the arm's
IK: neither arm reaches the far slot).

Simulation stand-ins: the strip is one rigid block with no outlets or ports; both cords are welded into its end faces
(the USB cable is not plugged into a port); the cords are chains of 2 cm rigid segments with bending springs (no
twist, stretch or memory) and start out straight; the box is fixed and rigid; the lid is a rigid plate with a welded
handle.

Instruction: `Hide the unplugged power strip in the cable box: put the strip inside, lead its black power cord out through the slot in the box's right end and the blue USB cable out through the slot in its left end, then put the lid on flat and let go. No cord may lie over the box's rim.`

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the simulated cables) is in this folder next to `task.py`; `cabledemo.py` runs the scripted demo.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1600 \
    --instruction "Hide the unplugged power strip in the cable box: put the strip inside, lead its black power cord out through the slot in the box's right end and the blue USB cable out through the slot in its left end, then put the lid on flat and let go. No cord may lie over the box's rim."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1600
```

The demo is closed loop: every grasp uses the measured strip, cord and lid states.

1. The right arm grips the strip across its width at its middle, lifts it over the near wall, centres it over the
   box floor and sets it down.
2. Both arms then work at once. Each picks its cord up off the table a little way out from the box. It carries the
   cord to just outside its own end slot, with the wrist camera facing the slot, and sets it down there. The cord now
   runs strip, slot, table.
3. The left arm picks the lid up by its handle, carries it over the box, lowers it onto the rim and lets go. Both
   arms move clear.

Success (score 1.0 with the strip, lid and cords at rest) holds for 1 s and ends the episode at step 625 (about
31 s); task_score 1.0. `demo/` holds the videos, `setup.png` and `result.json`.

Unintended strategies, selected with `CABLEBOX_DEMO=<mode>` (each scores exactly 0 in simulation):

| `CABLEBOX_DEMO=` | what it does | result |
|---|---|---|
| `nothing` | waits | 0 |
| `drape` | strip in the box, each cord's outer part laid back over the near wall instead of leading away from its slot, lid put on | 0 (lid rests on the cords, one corner 7.3 mm above the rim; cords cross the rim) |
| `no_lid` | strip in, both cords through their slots, lid left on the table | 0 (lid not on the box) |
| `hold` | everything done, but the left hand keeps holding the lid's handle | 0 (gripper still on the lid) |

Three more outcomes were checked only offline, on synthetic cord shapes with the same `score()` (not run in
simulation). Both cords through the same slot scores 0. A cord that comes back into the box after its slot scores 0.
The cords swapped (power cord through the left slot, USB cable through the right) scores 0.5.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

Score 0 if any of these holds at the end:

- the simulated state is not finite;
- some part of the strip is outside the box's inside or above the rim;
- a lid underside corner is more than 3 mm above or below the rim height (e.g. the lid rests on a cord);
- the lid's centre is more than 1 cm from the box centre, or the lid is turned more than 5 degrees;
- a gripper is within 3.5 cm of the strip, a cord, a plug, the lid or its handle;
- either cord fails this path, followed from the strip: inside the box below the rim, then through one end slot
  (between its sides, below the rim), then outside the box's footprint for the rest of its length, plug included.
  A cord over a wall or the rim anywhere else, or one that comes back into the box, fails;
- both cords leave through the same slot.

Otherwise the score is 1.0 if the power cord leaves through the right slot and the USB cable through the left slot,
and 0.5 if they are swapped.

In a real trial the box is taped down; also score 0 if it moved more than 1 cm. All conditions can be checked with
a ruler, a 3 mm feeler and a top photo.

`progress()` reports the stages for analysis only: strip inside, each cord through its own slot, lid seated,
released.

Runtime per trial: 1,600 control steps (80 s at 20 Hz). Seeds: the layout is fixed with `object_jitter=0`; nothing,
cables included, is jittered.
