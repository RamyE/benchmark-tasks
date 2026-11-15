# Pull a cable from under stacked cases without dragging or tipping them

An everyday chore, scaled to a tabletop. A power or extension cord has ended up pinned under a stack of boxes or cases,
and you want it back: most people have tugged such a cord and felt the whole stack slide or start to topple with it. The
usual fix is to tip the bottom box up by its handle with one hand and slide the cord out with the other.

Here two hard cases stand stacked on an orange 6 mm power cord: a black base case (600 g) with a bar handle on standoffs
on its front face, and a grey top case (300 g). The cord runs left to right under the base case, 3.5 cm behind its front
face. The stack rests on the cord and on its own back edge, so most of its weight is on the cord. The robot must draw the
cord out from under the stack and drop its plug into a grey bin (where the cord is stowed), without sliding either case
more than 3 cm or tilting it more than 10°, and then let go of everything.

Pulled with one arm, the cord drags the whole stack with it (about 23 cm in sim), so a plain yank scores 0. In the
intended method, one arm lifts the base case's handle about 2 cm (its front edge rises to about 1.8 cm), so the stack pivots back on its
back edge to about 7.5° and the cord is unloaded. The other arm draws the cord out by its plug and drops the plug in
the bin. Then the case is set down. In this method both arms work at the same time: one holds the weight off the cord
while the other pulls.

## Requirements

- yam-isaac-lab-arena and Isaac Lab-Arena 0.3.1 (official install).
- `cablekit.py` (the cable simulation) and `cabledemo.py` (the scripted-demo runner) are in this folder.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Pull the orange cord out from under the stacked cases without sliding or tipping them, and put its plug in the grey bin. Let go of everything at the end."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

The demo is closed loop: it grasps the handle and the plug where they actually are. Both arms pinch with fingers closing front-to-back (yaw -90°), so the left wrist camera sits beside the stack and looks down at the cord exit. Test-only shortcut variants are
selected with `CASES_DEMO=nothing | yank | hold | tip`, and each one scores 0:

- `nothing`: no motion. The cord stays under the stack.
- `yank`: one arm pulls the plug to the bin and drops it in. The stack is dragged about 23 cm with the cord still under it.
- `hold`: the correct method, but the left gripper keeps holding the case handle at the end.
- `tip`: the handle is lifted 7 cm instead of 2 cm, so the stack tilts to about 24° (and its corners move about 5 cm).

## Scoring (`score()`)

- **0** if, at any step, a bottom corner of either case is more than 3 cm (top view) from its starting mark.
- **0** if, at any step, either case tilts more than 10°. Both of these checks are latched for the episode.
- **0** if, at the end, a gripper TCP is within 3 cm of the cord, the plug, a case or the handle.
- **1.0** if no cord or plug point lies inside the base case's footprint (top view) and the plug centre is inside the bin, below its rim.
- **0.5** if the cord is out from under the case but the plug is not in the bin.
- **0** otherwise.

Success means 1.0 held for 1 s. `progress()` reports these stages: cord drawn out, plug in bin, stack not moved,
stack not tipped, released.

## How this differs from similar tasks

Unlike tasks that take a rigid item out from under or within a stack, the item here is a deformable cord pinned under a load, and the cases must not slide more than 3 cm or tilt more than 10 degrees at any time; nothing is restacked. Pulled alone, the cord drags the stack, so one arm must take weight off while the other draws the cord out.

## Runtime and seeds

The scripted demo scores 1.0 and ends at step 391 (19.6 s at 20 Hz), after both grippers let go: the open left gripper rises toward a point about 18 cm above the table in front of the stack's right front corner, so its wrist camera shows the case corner with the freed cord tail beyond it, and the open right gripper stays over the bin, about 12 cm above the table, so its wrist camera looks down at the plug in the bin. Its own log reports a largest case-corner shift of 0.9 cm and a largest tilt of 7.5° over the whole run (limits 3 cm and 10°), and the plug ends inside the bin at x 23.9, y -42.7 cm, about 1.5 cm above the table (bin centre x 24, y -41; rim 4 cm). The run limit is 1,200 steps. The layout is fixed with
`object_jitter=0`, and the cable is never jittered.
