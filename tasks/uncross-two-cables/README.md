# Uncross two tangled cables

Two 6 mm cables, red and blue, run from two device boxes at the back of the table and lie straight, crossing each
other once in the middle, the blue one on top. Each cable's plug therefore lies on the wrong side, in front of the
other cable's port. At the front centre a two-port charging dock has a red (left) and a blue (right) port. The
robot must untangle the cables so that they no longer cross in top view, put each plug into the port of its own
colour and let go. Carrying the plugs straight to their ports drags one cable across the other and leaves them
crossed; plugging each into the nearer port leaves them crossed and in the wrong ports. The natural solution is
bimanual: the left arm lifts the top (blue) cable by its plug and holds it up while the right arm pulls the red
cable out from under it and drops it into the red port; then the left arm lowers the blue plug into the blue port.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "The red and blue cables are crossed. Untangle them so that they no longer cross each other, then put the red cable's plug into the red (left) port and the blue cable's plug into the blue (right) port of the charging dock, and let go of both plugs."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed-loop (reads the live plug positions): both arms grasp the plugs; the left arm lifts the blue (top) plug
12 cm up and holds it back-left, so the blue cable rises off the crossing; the right arm pulls the red plug 4 cm
above the table to the left, sliding the red cable out from under the blue one, and drops it into the red port;
the left arm then lowers the blue plug into the blue port and both arms move away. Success holds for 1 s and ends
the episode at step 371 (~19 s); task_score 1.0. `demo/` holds its videos, `setup.png` and `result.json`.

Shortcut checks (`UNCROSS_CHEAT=<name> python cabledemo.py task.py out`), all score 0:
`none` (do nothing: one crossing), `drag` (carry both plugs to their ports low over the table without lifting
either cable: the cables stay crossed), `swap` (drop each plug into the nearer, wrong-colour port: crossed and
wrong ports), `hold` (the full demo but the left gripper keeps holding the blue plug in its port: not released).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if either gripper is closer than 3 cm to either cable or plug (not released), if the two cables' centre
  lines cross anywhere in top view (plugs included; height ignored), or if a plug is seated in the other colour's
  port.
- Otherwise **0.2**, plus **0.4** for each plug seated in its own port (plug centre between that port's walls,
  lying on the table, sticking out of the open end by at most 1 cm). Both = **1.0** (success).
- Doing nothing scores 0 (one crossing). `distance()`: sum of the plugs' distances to their ports (m), +1 m while
  the cables cross.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). One trial takes ~3–5 min wall time on one GPU including the ~1–2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cables are never jittered (they always start straight), so every
  trial starts identically.
