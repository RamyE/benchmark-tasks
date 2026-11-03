# Clear FOD before laying a harness

A harness layout board lies in front of the robot with a blue route line and a plug outline drawn on it. A 40 cm
black 6 mm cord leaves a fixed connector block at the line's left end but lies 18 degrees off the line, towards the
robot. Two loose steel discs (30 mm washer stand-ins, foreign object debris) lie **on** the route line. The robot
must first put both discs into a red FOD bin, then lay the cord along the line with its plug inside the outline,
never laying the cord over or against a disc that is still on the board. Laying the cord straight away pushes it
onto the discs (score 0); pushing or dropping the discs anywhere but into the bin also scores 0. The natural
solution is bimanual: each arm bins the disc on its side (one bin between the arms, so they take turns), then the
left arm carries the middle of the cord and the right arm the plug onto the line together. The skill tested is
FOD discipline (clear the work area into the bin before layup), not clip routing: the route is a plain line.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "The two steel discs on the harness board are loose debris. First put both discs in the red bin, then lay the black cable along the blue line with its plug inside the plug outline at the end of the line. Never lay the cable over or against a disc that is still on the board."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed-loop (reads live object states): left arm picks disc A and drops it in the bin while the right arm picks
disc B and waits; the right arm then drops disc B; next the left arm grasps the cord 20 cm from the block and the
right arm grasps the plug, both lift, carry their part onto the line and lay it down, then release and retreat.
Success ends the episode at about step 380 (~19 s) with task_score 1.0. `demo/` holds the recorded run.

Shortcut variants (`FOD_DEMO=<mode> python cabledemo.py task.py out`), each scoring exactly 0:
`nothing` (idle), `route` (ignore the discs and lay the cord on the line: it ends up lying on them), `sweep` (push the
discs off the board onto the table, then lay the cord), `drop` (pick the discs but put them on the table, not in
the bin, then lay the cord). `bin_only` (discs binned, cord left alone) shows the intended partial credit 0.5.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if at any control step the cord or plug came within 2 mm (top view) of the edge of a disc resting on the
  board or table (not lifted, not in the bin) with its underside less than 1 cm above the disc top (carrying the
  cord higher over a disc is allowed) — tracked over the whole episode, reset at each episode start.
- **0** if at the end any disc is not in the bin (centre over the 8 x 8 cm inner floor, below the 3 cm wall top),
  or a gripper (pad centre) is within 3 cm of the cord, plug or a disc (still holding).
- Otherwise **0.5**, + **0.25** if the plug's whole footprint lies inside the outline (lying on the board), + **0.25**
  if every point of the cord is within 1.5 cm of the line and at most 1.5 cm above the board.
- `distance()`: plug to outline centre (m) once both discs are binned; 1 m before that or after a violation.

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). One trial takes ~3–4 min wall time on one GPU including Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight).
