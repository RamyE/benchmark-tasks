# Unplug a cable by its plug body

A black 6 mm cord, 35 cm long, runs from a fixed grey anchor block on the left of the table to a fixed white wall
socket block on the right. Its rigid plug body (55 x 20 x 14 mm) is pushed 25 mm into the socket's horizontal
cavity (floor 20 mm above the table) and sticks out 30 mm; the cord hangs from the plug down towards the table.
The robot must unplug the cable **by gripping the plug body, not the cord**, and lay the plug in a white tray.
Pulling or lifting the cord while the plug is still in the socket, letting the plug hang from its cord, or letting
it fall onto the table scores 0.

Instruction: `Unplug the cable from the wall socket by gripping the plug body, not the cable, and lay the plug in the tray.`

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the simulated cable) is in this folder next to `task.py`; `cabledemo.py` runs the scripted demo.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Unplug the cable from the wall socket by gripping the plug body, not the cable, and lay the plug in the tray."
```

Run one trial per process (or one per embodiment reset): the scorer keeps a short history of the trial (see
Scoring), which `task.py` clears on every embodiment reset.

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

The right arm comes down on the exposed half of the plug body (fingers across its 20 mm width), pulls it straight
out along the socket axis, lifts it, measures where the plug sits in its grip (closed loop) and lays it flat in the
tray. It finishes in about 10 s (201 steps) and ends with `task_score` 1.0. Videos are in `demo/`.
Cheat variants (all score exactly 0): `UNPLUG_DEMO=yank_low` (grab the cord 12 cm behind the plug, pull it out
by the cord, carry it to the tray), `UNPLUG_DEMO=yank_lift` (same, lifting the cord 4 cm while pulling),
`UNPLUG_DEMO=yank_near UNPLUG_YANK_AT=0.04` (grab the cord 4 cm behind the plug), `UNPLUG_DEMO=hold` (unplug
correctly but keep holding the plug above the tray), `UNPLUG_DEMO=nothing` (do nothing).

## Scoring (`score()`, partial credit; `success()` = score 1.0 with the plug at rest, held 1 s)

Three "mishandling" checks run every step and latch for the rest of the trial; any one makes the score 0:

- **Cord pull**: while any of the plug is still inside the socket (plug tip behind the socket face), a point of the
  cord 3–15 cm behind the plug moves more than 1.5 cm (beyond how far the plug itself has moved) from where it rested
  at the start. Gripping the plug body moves only the plug, and the cord just follows it; pulling or lifting the
  cord moves the cord before the plug comes out (the cord hangs from the elevated socket, so it must be lifted and
  straightened before it can pull).
- **Dangling**: once the plug is out of the socket and not in the tray, its body tilts more than 30° from
  horizontal, i.e. it hangs from its cord instead of being held by its body.
- **Drop**: once the plug is out of the socket and not in the tray, it comes down onto the table (plug centre below
  12 mm). Pulled out by its cord, the unsupported plug falls off the socket lip (the cavity floor is 20 mm up) and
  is dragged along the table; held by its body it never touches the table.

Otherwise (scored at the end of the trial): 1.0 when the plug centre lies inside the tray's inner footprint
(70 x 100 mm) on its floor (centre at most 19 mm above the table); 0.5 when the plug lies on the tray but not inside
its walls (centre within 1 cm of the tray's outer footprint and at most 32 mm above the table, e.g. across the rim);
0 otherwise — a plug still in the socket, or unplugged but still held in the air, scores nothing.

Real-world measurement: top-camera video; two thin tape flags on the cord 3 cm and 15 cm behind the plug, and a
scale on the board along the cord. A flag moving more than 1.5 cm (beyond the plug's own movement) while the plug is still partly in the socket
is a cord pull; a plug seen hanging nose-down from its cord is a dangle; a plug touching the board outside the tray is a drop.
Tray: plug released and fully inside the tray walls (1.0) or lying across the rim (0.5).

## Runtime and seeds

Run limit 1,200 steps (60 s at 20 Hz); the demo needs 201. A trial takes ~3–6 min wall-clock on one GPU
(first start of Isaac Sim adds 1–2 min). The layout is fixed (`object_jitter=0`; every object except the cord is
fixed and the cord is never jittered), so every seed starts from the same state.
