# Recover a crawler by its tether around a wall

A hazardous-zone mock-up. On the far side of the table, out of reach of both arms, an orange inspection crawler
(80 x 60 x 40 mm, 120 g, low-friction skids, black camera head on its front) sits directly behind a fixed grey wall
(20 x 2 x 6 cm) with a round end post. Its blue 6 mm tether (65 cm) leaves the crawler's rear and lies straight past
the wall's end post towards the robot's left, ending in a black handle. In the wall's lee, nearer the robot, a 15 x 15
cm recovery zone is taped in green. The robot must bring the crawler into the zone by its tether alone, as inspection
crawlers are pulled out of pipes and radioactive tunnels by their tethers when they get stuck or disabled. A direct pull towards the zone
drags the crawler into the wall's end post (the tether wraps around it), so the crawler must first be hauled out along
the clear lane past the wall end and then steered sideways around the end into the zone, without the grippers ever
touching it, without it striking the wall or post, and without it tipping. Both arms are needed: only the left arm
reaches the tether's free part and the hauling lane; only the right arm reaches the right side the final sideways
pull must come from, so the left arm lays the tether across to the right arm (a handoff through the table).

Instruction: `Recover the orange crawler into the green taped zone using only its blue tether: never touch the crawler, don't let it hit the grey wall or its post or tip over. Steer it around the wall's end, then let go of the tether.`

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the simulated tether) is in this folder next to `task.py`; `cabledemo.py` runs the scripted demo and
  `vision.py` gives it the top-camera estimates it uses.
  `task.py` welds the tether and the camera head to the crawler, gives the crawler low-friction skids, and turns on
  PhysX contact reporting between the robot links and the crawler.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Recover the orange crawler into the green taped zone using only its blue tether: never touch the crawler, don't let it hit the grey wall or its post or tip over. Steer it around the wall's end, then let go of the tether."
```

Run one trial per process (or one per embodiment reset): the scorer keeps a short history of the trial (tipping,
touching, wall strikes), which `task.py` clears on every embodiment reset.

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

The demo uses only what the robot can observe: the top camera image and its own arm poses, plus the fixed layout
of the station (wall position, haul targets). It never reads object poses from the simulator. `vision.py` (next to
`task.py`) back-projects top-camera pixels onto horizontal planes with the station's camera calibration (pose and
pinhole intrinsics, resized to the 320 x 176 image). Hue and saturation masks find the orange crawler (only its
brightest face, its top, back-projected at its top-face height), the blue tether (traced pixel by pixel from the
crawler outwards into an ordered chain, so a point a given length along the tether can be looked up) and the
green tape (the zone's centre). Against the simulator these estimates are within about 5 mm (crawler) and
10 mm (tether chain) when nothing hides them.

The left arm grips the tether where the camera sees it 25 cm from the crawler and hauls it 25 cm straight in along
the clear lane. It regrips 10 cm from the crawler and hauls again, then moves clear of the camera's view and checks
that the crawler is past the wall's front face (one more haul if not). A third haul steers it to the right,
laying the tether across the middle of the table. The right arm grips the tether 9 cm from the crawler and pulls
the crawler sideways into the zone. This pull is closed loop on the crawler's position in the image, and the arm
re-grips if the crawler stops following. Both arms then let go and move clear. Arms park out of the camera's line of
sight before each look, and an estimate that jumps while an arm hides the tether is ignored. Both wrist cameras
look along the tether towards the crawler and the wall end while pulling. The demo reaches `task_score` 1.0 in
649 steps (about 32 s) of the 1,200-step limit; videos are in `demo/`. `CRAWLER_DEBUG=1` adds the simulator's
true poses to the demo's log for comparison. It only prints them; no motion uses them.

Shortcut variants (`CRAWLER_DEMO=...`), each scoring exactly 0: `nothing` (do nothing), `direct` (haul the tether
straight towards the zone: the crawler is dragged into the wall's end post), `straight` (haul straight in along the
tether's lane and stop: the crawler ends left of the wall, not around it), `push` (haul it in, then push the crawler
into the zone with the closed gripper). They use the same camera estimates.

## What changed in this revision

- The scripted demo is now closed loop on camera-based estimates only: the tether's grasp points and the crawler's
  position come from the top camera image (colour masks, back-projected with the camera calibration), never from
  simulator state. The scene, scoring, instruction and run limit are unchanged.
- The final pull grips the tether nearer the crawler (9 cm instead of 12 cm), where it leaves the crawler's rear
  straight, and re-grips if the crawler stops following.

## Scoring (`score()`, partial credit; `success()` = score 1.0 with the crawler at rest, held 1 s)

- **Disqualified (latched, score 0):** the crawler tilts more than 45 deg at any time; a gripper TCP comes within
  3 cm of the crawler body or any robot link touches it (PhysX contact, every physics substep); the crawler's
  footprint comes within 3 mm of the wall or its end post at any time.
- **Not released (score 0):** at the end a gripper TCP is within 3.5 cm of the tether or the handle.
- Otherwise **1.0** if all corners of the crawler's footprint are inside the taped square (inner tape edges);
  **0.5** if the crawler has come around the wall end (whole footprint nearer the robot than the wall's front face and
  its centre between the wall's two ends) but is not fully inside the square; else **0**.

Real-world measurement: a top photo for the zone, video for tipping, hand/gripper contact and wall strikes.

## Runtime and seeds

Run limit 1,200 steps (60 s at 20 Hz). A trial takes several minutes wall-clock on one GPU (first start of Isaac Sim
adds 1–2 min). The layout is fixed (`object_jitter=0`; the cable is never jittered), so every seed starts from the
same state.
