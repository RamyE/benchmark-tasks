# Route a cable through four clips

A 6 mm black cable, 52 cm long, leaves a grey anchor block fixed at the left rear of the station table and lies
straight across the table towards the front right, ending in a rigid plug body. Four open-top white U-clips
(numbered 1-4 from left to right) stand in a straight row 36 cm in front of the arms, and an open orange tray
sits at the right end of the row. The robot must press the cable down into clips 1, 2, 3 and 4 in that order,
starting from the anchor, and lay the plug in the tray. The row spans both arms' workspaces: clips 1-2 are
only reachable by the left arm, clips 3-4 and the tray only by the right arm, so the cable has to be handed
across the midline, and one arm should keep the routed part pinned while the other arm pulls
the rest of the cable through clips 3 and 4.

## Requirements

- yam-isaac-lab-arena with Isaac Lab-Arena 0.3.1 (see that package's README for the install).
- `cablekit.py` (the simulated cable) and `cabledemo.py` (the scripted-demo runner) are in this folder.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Starting at the grey block, press the cable down into the four white clips in order from left to right, then put the plug in the orange tray."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out
```

A closed-loop script (privileged cable state + IK): the left arm takes the cable 3.5 cm past clip 1 (pulled
taut from the anchor), lays it over the slot, nudges sideways until the cable crosses the slot centre and lowers
it to the table so it drops into the clip; same for clip 2, after which the left arm pins the cable on the table
just before clip 2. The right arm then does clip 3 the same way, takes the plug and, keeping the cable taut from clip 3,
lays it over clip 4 and lowers the plug into the tray, so the cable drops into clip 4; finally both arms press
the cable flat beside any clip it has ridden up in. Output of the
reference run is in `demo/` (videos from the three cameras, `setup.png`, `result.json`).

## Scoring

`score()` (`--scorer task_score`, evaluated at the end of the episode):

- A clip counts as seated when the cable centreline passes through its slot (inside the 10 mm gap, along at
  least 12 mm of the 20 mm slot) with the cable's top no higher than the clip walls (14 mm).
- Clips are credited in order only: 0.2 per clip for clips 1..n all seated and passed one after another along
  the cable from the anchor (a clip seated while an earlier one is not, or out of order, earns nothing).
- +0.2 for the plug body lying inside the tray (centre within the 8 x 9 cm interior, below the 12 mm wall
  top), only once all four clips are seated.
- `success()` = score 1.0 held for 1 s, which ends the episode early.

## Runtime and seeds

The scripted demo reaches full score after 859 steps (43 s of simulated time); a 2,400-step (2 min at 20 Hz) trial takes a few
minutes of wall time on one GPU. The layout is fixed (`object_jitter=0`); the cable is never jittered, so every
trial starts from the same state.
