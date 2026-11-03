# Dress three cables into a comb

Three 6 mm round cables (red, blue, yellow; 38 cm) leave a grey cable-entry block fixed at the left of the table.
Red, which is nearest the robot at the block, lies crossed over blue and yellow, so at the free ends the colour
order is reversed. A white comb on the table in front of the robot has three 9 mm open-top slots, each marked by a
coloured tab (red nearest the robot, then blue, then yellow: the same order as the block holes). The robot must lift
the cables apart and press each one down into its own slot, so that no two cross between the block and the comb.
The natural bimanual strategy is for both hands to hold one cable on either side of the comb, stretch it across its
slot and lower it in together. A cable cannot be dragged across the others into a slot without leaving a crossing,
and slots are only 1.5 mm wider than the cable on each side, so the placement has to be accurate.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (see that package's README), activated with
  `source IsaacLab-Arena/yam.env`.
- `cablekit.py` (the flexible-cable simulation) and `cabledemo.py` (scripted-demo runner) are in this folder.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1800 \
    --instruction "Press the red, blue and yellow cables into the cable comb so each sits in its matching coloured slot, without crossing each other."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1800
```
Closed loop on the live cable state. For each cable in turn (red, the top one, first): the right hand grips it
past the comb's position and lifts it a little, the left hand grips it on the block side (red between its two
crossings, with the fingers only ~21 mm open so a neighbouring cable is not caught); together they carry it
stretched across its slot, measure where it actually hangs over the comb and shift until it is centred on the slot,
lower it in and let go (re-centring and retrying if it missed). If spare cable between the block and the comb
bulges towards the next cable's lane, the left hand lays the bulge back to the near side. Reaches task_score 1.0
(videos and result in `demo/`).

## Scoring (`score()`, end of episode; `success()` ends the episode early after holding 1 s)

- A cable is **seated in a slot** when every point of its centreline over the comb footprint lies within that
  slot (centre inside the 9 mm gap), its top is no more than 1 mm above the tooth tops, and it continues at least
  2 cm out of both sides of the comb.
- **Crossing gate**: if, in the top view, any two cables' paths from the block to the comb's near face cross,
  the score is 0 (untangling comes first).
- Otherwise 0.3 per cable seated in the slot of its own colour (not seated or wrong slot: 0 for that cable), +0.1
  when all three are. Possible scores: 0, 0.3, 0.6, 1.0. Success (ends the episode with 1.0 after 1 s) = all three
  seated correctly with no crossing.
- Real trial: score 0 if the comb or the block is moved or a cable is pulled out of the block.

`distance()` = 3 minus the number of cables counted (0 at success).

Unintended strategies score 0 (checked in sim with `COMB_CHEAT=nothing|skip_red|hover python cabledemo.py ...`):
doing nothing (red still crosses blue and yellow), dressing blue and yellow while leaving red crossed over them
(the crossing gate gives 0), and holding red stretched above its slot instead of pressing it in (not below the
tooth tops).

## Runtime and seeds

Run limit 1,800 steps (90 s at 20 Hz); the scripted demo succeeds after 865 steps. A 1,800-step trial takes ~5-10 min on one GPU
(including ~1-2 min of Isaac Sim start-up). The layout is fixed with `object_jitter=0`; the cables are never
jittered, so every trial starts from the same state.
