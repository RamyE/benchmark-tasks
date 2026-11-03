# Leave a drip loop before a cable gland

A grey weatherproof enclosure stands on a short post in front of the left arm. A black 6 mm cable comes out of a
gland on its right-hand side, 10 cm above the table, and runs straight down to the table. A support hook (an open U
cradle 14 cm above the table, 4 cm higher than the gland) stands 18 cm to the right. The robot must lay the cable
over the hook so that between the hook and the gland it hangs in a **drip loop**: it dips 3-7 cm below the centre of
the gland and then rises into it. Rain running along a cable then drips off the low point instead of running into
the enclosure. The robot must then let go.

This is a safety shape judged against the entry point, not a route and not a sag between two supports. One end is
sealed in the gland, and only the low point just before the entry is scored, relative to the gland's height. The
easy result looks almost the same and is unsafe: the cable laid over the hook and pulled straight (level with the
gland, or running down into it from the hook above). That scores 0. The task is bimanual: the right arm lifts the
cable over the hook, and the left arm pulls the span next to the gland down into a loop. This draws cable back over
the cradle from the tail.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "The black cable comes out of the gland on the side of the grey enclosure. Lay the cable over the support hook so that, between the hook and the gland, it hangs down in a drip loop whose lowest point is 3 to 7 cm below the centre of the gland, then rises into the gland. The rest of the cable hangs down on the far side of the hook. Then let go of the cable."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

The demo is closed-loop: it reads the live cable state.
1. The right arm takes the cable 24 cm from the gland. It carries that point over the cradle and past the hook,
   lowers it on the far side so the cable drops into the cradle, lets go, and moves clear. The cable now runs almost
   straight from the gland up to the hook, with a drip of about 0.4 cm (score 0).
2. The left arm takes the middle of the gland-to-hook span. It pulls the span down in steps until the measured drip
   is about 6 cm, drawing cable back over the cradle from the tail, then lets go and moves up and away.
3. The loop settles with a drip of about 5 cm, and success holds for 1 s.

`demo/` holds the videos, `setup.png` and `result.json`.

Cheat / failure variants (test only: `DRIP_DEMO=<name> python cabledemo.py task.py out`). Each scores exactly 0:
- `none`: do nothing. The cable stays on the table and is not on the hook.
- `level`: lay the cable over the hook, then pull the tail down on the far side so the span from the gland runs
  straight up to the hook (no drip loop), and let go.
- `hold`: form the loop, but the left arm keeps holding it down (not released).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** unless the cable is released: both grippers at least 3 cm from every part of the cable and plug. In a real
  trial, also 0 if the enclosure, gland or hook was moved, or the cable was pulled out of the gland.
- **0** unless the cable lies over the hook. Part of it must lie in the cradle slot (between the walls, at most 12 mm
  above the floor), and its plug end must lie on the far side, at least 3 cm past the hook centre and below the hook
  floor.
- Drip = height of the gland centre minus the lowest point of the cable's underside between the gland and the hook:
  - 3-7 cm: **1.0** (success).
  - 1.5-3 cm (too shallow) or more than 7 cm (loop down near or on the table): **0.4**.
  - Under 1.5 cm (level, taut, or running down into the gland from above: no drip loop): **0**.
- `success()` also requires the cable to be at rest. `distance()` is |drip - 5 cm| once the cable is on the hook,
  else 1 m.

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). The demo succeeds at step 411 (about 21 s). One trial takes about 5-10 min of
  wall time on one GPU, including the 1-2 min Isaac Sim start-up.
- The cable spawns straight out of the gland. After each reset, the task steps the scene for 2.5 s more with the arms
  at READY, so every trial starts with the cable at rest, hanging from the gland to the table.
- The layout is fixed with `object_jitter=0`. The cable is never jittered, so every trial starts identically.
