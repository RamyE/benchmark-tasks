# Stand two sonometer bridges under a weight-tensioned cord at the marks

A physics-lab sonometer: an orange cord is knotted to a peg at one end of a narrow white box, lies along the top, and
runs over a smooth steel rod at the other end down to a 40 g hanging weight. The weight keeps the cord taut and flat
on the bare top. The robot must stand two black notched bridges on the box, one at each blue mark, so the cord rests
in both notches, lifted about 3.5 cm off the box between them (cord underside at the notch floor), with the weight still hanging free.
A dark board stands just behind the box so the lifted cord shows against it from the top camera.

The hard part is the cord, not the bridges. The cord is always under tension from the hanging weight, so it falls
flat the moment it is let go. A bridge pushed against the lying cord only bows it sideways, because its faces are
vertical and there is nothing to ride up. The box top (6 cm) is narrower than a bridge is long (7 cm), so a bridge
can only stand on it across the cord line. One hand has to hold the cord about 9-10 cm above the box, about 15 cm from the
mark, while the other slides a bridge in underneath. Then the cord is lowered into the notch, where the weight presses it down and
seats it. The second bridge goes in the same way, with the hands swapping roles. From the top camera the finished
cord visibly steps up over the first bridge, runs level about 3.5 cm above the box, and steps down past the second.

## How this differs from similar tasks

Other "hold up and place" tasks lift a rope or hose so that a cart, a gate or a hose end can pass under it, or hand a
hose across. Here two supports must be stood at precise marks under a line whose tension comes from a free-hanging
weight. The cord shape (lifted over both bridges, level between them) is the outcome being scored, and the
restoring force of the weight both forces the second hand and seats the result.

## Source

"The Sonometer lab" (college physics lab manual), https://sites.gsu.edu/claphast/files/2017/09/Sonometer_Lab-2b6trmr.pdf:
"The length of the vibrating wire may be varied by the use of two metal bridges, inserted under the wire at any desired
point, to set up nodes at these bridge points."

## Requirements

yam-isaac-lab-arena with Isaac Lab-Arena 0.3.1 (official install). `cablekit.py` and `cableshape.py` are in this
folder.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1400 \
  --instruction "Set up the sonometer: stand the two black bridges on the box under the orange cord, one at each blue mark, so the cord rests in the notches of both bridges and the weight still hangs free. Then let go."
```

## Scripted demo

`python cabledemo.py task.py demo_out --max-steps 1400`. The right arm lifts the cord at y -7 cm while the left arm
slides bridge A in under it at the +8 cm mark (closed loop on the bridge's measured position). The right arm lowers the
cord into the notch and lets go, then the left lets go. Then the left arm lifts the cord at y +5.5 cm while the right arm
slides bridge B in at the -8 cm mark. Both arms let go and move away.

Shortcut variants (test only, `SONO_DEMO=<name>`), all score exactly 0:

| variant | what it does | why 0 |
|---|---|---|
| `nothing` | waits | no bridge at a mark |
| `onearm` | one arm alone pushes a bridge along the top into the lying cord at a mark | cord bows sideways, never rides up; not in the notch |
| `drop` | cord lifted and let go, then the bridge slid in | cord is flat again before the bridge arrives |
| `hold` | everything, but the right hand keeps hold of bridge B | gripper within 4 cm of a bridge |

`one` (only bridge A set) scores 0.5, the partial tier.

## Scoring

`score()` is read at the end of the run; the episode ends 2 s after the full set-up is reached with both hands 10 cm clear. It returns 0 if the state is non-finite, any gripper TCP or pad tip is within
4 cm of the cord or a bridge, or a bridge or the weight moves faster than 1 cm/s. A bridge counts when it stands on the
box top (tilt under 10 deg), with its centre within 1 cm of a mark and 6 mm of the cord line, and the cord centre inside
its notch. One counting bridge (one per mark) gives 0.5. Both give 1.0 if they are 16 +- 1.5 cm apart, the cord's
underside stays at least 3 cm above the box top all the way between them, and the weight hangs free (3 mm clear).

Runtime per trial: about 70 s simulated (1,400 steps at 20 Hz). Seeds: the layout is fixed with `object_jitter=0`; the
cord is not jittered.
