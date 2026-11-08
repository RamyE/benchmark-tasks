# Crank a hose reel with one hand while the other lays the turns side by side

A hand-crank garden hose reel stands on the right half of the table. Its drum turns on a horizontal axle running
left-right; a crank handle sits on the drum's right end, right of the robot's midline. A green 10 mm hose, its inner
end clamped to the drum core, lies on the table in a U and ends in a black nozzle. The robot must wind the hose on
neatly and let go; a one-way pawl on the axle then holds the drum.

Turning the crank winds the hose in under the drum. Where each turn lands across the drum depends on where the hose
is held as it goes on: left alone, it winds on as a loose open spiral (about 4 cm between turns) that runs to the far
flange; held in one place, the second turn climbs onto the first. A neat reel needs each turn laid against the one
before, which means one hand turns the crank while the other walks the hose across the drum, about one hose width
per turn, at the same time. Hose reel manuals ask for exactly this ("Distribute the hose evenly over the surface of
the drum assembly"), and a reel maker blames tangled, overlapping hose on a guide that is out of line.

The instruction states only the goal ("neatly"); what counts as neat is stated under Scoring.

## Objects

| object | type | size | material / colour | where (cm; x forward from the arm bases' midpoint, y left) |
|---|---|---|---|---|
| reel drum | cylinder on a revolute axle (y axis), 150 g, light drag and bearing friction (0.005 N m), one-way pawl | 6 cm dia x 14 cm | grey PVC pipe | axle x 34, 7.5 cm above the table; drum y 7-21 |
| flanges (2) | discs fixed to the drum | 12 cm dia x 6 mm | red PLA or plywood | y 6.35-6.95 and 21.05-21.65 |
| uprights (2) | fixed posts carrying the axle | 16 x 6 x 71 mm | dark grey | x 34, y 5.8 and 22.2 |
| crank arm | bar fixed to the drum's right end, through the axle | 14 x 6 x 118 mm | dark grey | y 4.7 |
| counterweight | block on the crank arm opposite the handle, 20 g | 22 x 6 x 22 mm | dark grey | 5 cm from the axle; balances the handle so the crank does not swing round when let go |
| crank handle | ball on a pin, on an 8.4 cm stem from the crank arm (handle + stem 20 g) | 24 mm dia ball | black | 5 cm from the axle, y -5; starts straight up |
| hose | cable, inner end clamped to the drum core at its bottom | 10 mm dia, 60 cm, 60 g | green PVC | clamp at y 7.8; 16 cm straight toward the robot, 10 cm radius U to the left, back out to the nozzle |
| nozzle | rigid end on the hose | 40 x 16 x 16 mm, 20 g | black | starts at x 31.9, y 27.8 |

## Why two arms (simultaneous)

The crank handle circles 5 cm right of the midline (y -5 cm), past the left arm's reach (left arm y > -2 cm); every
place the hose can be guided onto the drum (y 7.3-20.5 cm) is past the right arm's reach (right arm y < +2 cm).
Offline IK with the gripper straight down (`scratch/ik_final.py`, output in `scratch/ik_final.out`): the right arm
reaches all 24 points of the handle circle (x 29-39, 3.1-13.1 cm above the table, wrist turned 90 deg); the left arm
reaches the guide line x 24, y 7.8-12, fingertips just above the table. Guiding only works while
the drum turns: with the crank turned and the hose not held, it winds on at 4.3 cm per turn (score 0); held still it
piles up. So the two hands work at the same time, one turning, one walking the hose across.

## How this differs

- Our "Bring a spring cord reel's work light home into its dock": the reel's spring winds the cord by itself; the
  robot only carries the light home under spring pull and must never let go. Nothing about how the cord lies.
- Our "Pay a multicore off a free-turning drum to its cradle": the reverse act on a vertical drum. The arm pulls
  cable off and must stop the drum from overrunning; the drum is never driven.
- Our "Hold a reel's spring-loaded cord at a length mark while the other hand sets the reel lock": a spring reel
  held at a length while a lock is set; no winding.
- Board "Take up patch-cord slack on spools, then plug in" (our pending task): one turn of cord carried round two
  fixed posts, then a plug. The posts never turn; there is no lay pitch.
- Here the robot drives the drum itself through a crank (one arm moving the handle round a circle, two turns) while
  the other arm sets where each turn lands. The tested ability is two-hand coordination on one line: the lay pitch
  comes from matching the guide's motion to the drum's turning. The end state (a single layer of side-by-side turns)
  is something none of the above produces.

## Context and sources (quoted from the pages)

- O*NET 37-3011.00 Landscaping and Groundskeeping Workers, task: "Water lawns, trees, or plants, using portable
  sprinkler systems, hoses, or watering cans." (onetonline.org/link/summary/37-3011.00)
- Yardworks hose reel, model 059-0819-6, instruction sheet (Canadian Tire; media-www.canadiantire.ca/manual/product/
  0590819/0590819p-en-fr-7dc297df-8b22-4fa1-aae3-9e575d368e9a.pdf): "Distribute the hose evenly over the surface of
  the drum assembly." The same sheet lists a "Crank Handle".
- Hoselink manual hose reel support page (hoselink.com.au/pages/support-product-support-manual-hose-reel): "Use the
  crank handle to wind the hose into the box evenly." and, on a hose reel cart: "It sounds like the hose guide is out
  of alignment causing the hose to tangle and overlap."

## Paper fields

- Skill tags: winding, routing/seating, constraint-keeping.
- Industry: landscaping and groundskeeping (also fire, wash-down and workshop hose reels).
- Two-arm need: simultaneous (and reach-forced: handle right-only, hose guide left-only).

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (hose simulation, the U-shaped start layout,
  scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Wind the green hose onto the reel neatly, then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop on the live handle and hose. The right arm pinches the ball handle (wrist camera looking left at the
drum) and walks it round the axle in 10 deg steps from its live position. The left arm puts its fingers round the hose
on the table 10 cm in front of the drum, partly closed (an 18 mm slot round the 10 mm hose, so the hose slides through),
and moves left in step with the drum, 10.5 mm per turn. The crank keeps going until the nozzle has come round to the
guide fingers (nozzle centre within 3.5 cm of them): by then 50 cm of the 60 cm hose is on the drum, about 2.1 turns.
The left hand opens, lifts off the hose and goes back beside its base; then the right hand lets go of the handle and
goes back too. The pawl holds the drum. What is left off the reel is the nozzle on about 10 cm of hose, running from
the underside of the drum down to the table, as a reeled-up hose is left.

Demo result: score 1.0, success at step 685 of 1,200 (50 cm wound when the crank stopped, about 2.1 turns, one layer,
turns 10.7-12.0 mm apart, 0.02 rad of drum turn without the handle held). Recording: `demo/` (top camera and both
wrist cameras); the last top-camera frame shows the reel wound, the nozzle lead hanging to the table and both arms
back by their bases.

Why the wind stops at 50 cm: the guide fingers sit 10 cm in front of the drum, so once 50 cm is on, the nozzle (16 mm
square) reaches them and cannot pass through the 18 mm slot; cranking on jams it (a probe kept cranking: the nozzle
stopped against the fingers at 52 cm and the crank stalled). Moving the guide in toward the drum near the end to win
the last few cm stalled the crank earlier (the hose bends sharply round the fingers when the nozzle comes in from the
side). So the 1.0 mark is 48 cm: all but the last 12 cm of the hose (the lead from the drum down to the nozzle), one
2 cm hose segment below what the demo reaches.

## Scoring (`score()` at the end; `success()` must hold 1 s)

`success()` = score 1.0 and the hose resting (speed under 0.05 m/s); it only ends the episode early, the score itself has no speed condition.

Score 0 if, over the trial, the drum is turned more than 0.3 rad in all in the winding direction while no closed right
gripper is on the handle (turning the drum by its flange or the hose is not using the crank), or, at the end, a
gripper is within 3 cm of the hose, nozzle or handle, or the reel is not neat when judged. On the handle = the right
gripper closed (finger joint under 0.030 m, open is 0.047) with its tool point within 2.5 cm of the handle centre.
Only forward turning past the furthest point reached since the handle was let go counts, so back-and-forth jitter of
the drum cannot add up; an idle trial (do-nothing run, 60 s) reaches 0.06 rad.

Judged = 1 s after the last hand has let go (both grippers 3 cm or more from hose, nozzle and handle); a hand coming
back on the hose or handle starts that over. On the drum = segment centres within 1.5 cm of the first-layer radius,
between the flanges, counted from the clamped end. Wound = the most hose that was on the drum while a hand was on or
within 3 cm of the hose, nozzle or handle, as long as the judged reel still holds all but 6 cm of it; if more has sagged off, wound = what the judged reel holds plus 6 cm (the pawl stops
unwinding, so only the release sag can take hose off). Neat (on the judged reel) = one layer (no wound part more than
4 mm above the first layer) and every turn 8-17 mm across the drum from the turn before it, always the same way (the
hose is 10 mm wide, so touching turns are about 11 mm apart), and no hose bunched loose between the flanges. The last
2 cm, where the hose leaves the drum, is not checked for lift or spacing (the hanging lead pulls it a little sideways).

| end state | score |
|---|---|
| neat, 36-41 cm wound (1.5 turns or more), released | 0.3 |
| neat, 42-47 cm wound, released | 0.6 |
| neat, 48 cm or more wound (all but 12 cm of the 60 cm hose, about 2 turns), released | 1.0 |
| anything else | 0 |

Real trial: measure the turn spacing with a ruler across the drum, check by eye/photo that no turn sits on another,
mark the hose at 36, 42 and 48 cm from its clamped end and see which mark is on the drum, all from a photo taken 1 s
after the robot lets go.

Why judge 1 s after letting go: with no hose tension the loose wound end sags off the underside of the drum, and in
the sim it keeps slowly sliding off for the rest of the minute (a real hose's coil set and friction hold it). Stopping
at 38 cm left 34 cm on the drum a few seconds after letting go and 24 cm at the end of the trial; stopping at 44 cm
left 40 cm and then 28 cm. Judging the reel 1 s after let-go, with up to 6 cm of sag allowed, scores what the robot
did, not how long the trial ran on. The demo ends at success about 2 s after letting go.

Tiers in practice: neatness needs at least one turn beside the previous one, so a stop at about 30 cm (one turn)
scores 0; the 0.3 band starts at 1.5 turns. Every band needs the crank and the guide working together for more
than a turn and a half.

## Shortcuts tried (scripted, same layout, scored by `score()`; run to the 1,200-step limit unless ended earlier)

| shortcut | what happens | score |
|---|---|---|
| do nothing (idle the whole minute) | 4 cm on the drum; 0.06 rad of drum creep, under the 0.3 rad limit | 0 |
| crank only, hose not guided (one arm) | 56 cm wound as an open spiral, turns 40-61 mm apart | 0 |
| crank while holding the hose in one place | turns climb onto each other (13.5-13.7 mm lift), turns 0.9-2.9 mm apart | 0 |
| wind neatly to the nozzle but keep holding the handle and hose | 50 cm neat, grippers still on the handle and hose at the end | 0 |
| turn the drum by its flange rim with the left hand, fast (no crank, pushing) | 0.41 rad wound without the handle by step 41 (latched); hose not laid | 0 |
| turn the drum by its flange rim with the left hand, slowly (about 0.15 rad/s) | latched at 0.32 rad by step 104; hose not laid | 0 |
| neat, but stop at 42 cm (a long loop and the nozzle still out on the table) | 42 cm neat | 0.6 |
| tier check: neat, stop at 38 cm | 38 cm neat | 0.3 |
| tier check: neat, stop at 44 cm | 44 cm neat | 0.6 |
| edge check: neat, stop at 48 cm | 48 cm neat; success at step 613 | 1.0 |

Scores of the shortcut and tier runs are in `scratch/sc_*.json` (logs beside them).

## Runtime and seeds

1,200 steps (60 s at 20 Hz) per trial. One fixed layout (`object_jitter=0`); nothing is randomised.
