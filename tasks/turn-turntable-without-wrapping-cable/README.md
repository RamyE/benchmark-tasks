# Turn a camera turntable without wrapping its cable

A camera turntable (grey disc, 120 mm diameter) turns on a fixed vertical axle in front of the right arm. A red
arrow on its top points away from the robot; a green mark on the table sits at the disc's robot-side edge, so the
arrow must turn half a turn. The camera cable leaves a swivelling strain relief on the far rim tangentially to the
left, passes a fixed grey post and runs through a fixed cable clip; beyond the clip it lies loose. Turning the disc
clockwise winds the cable round the turntable; turning it anticlockwise unwinds it, but the cable then sweeps into
the post and hooks round its far side. The robot has to work out the winding: turn anticlockwise, and get the cable
over the post to the robot's side of it before the sweep reaches the post. The reference solution turns the disc
about 60 degrees first (the connector swings towards the clip and the cable goes slack), holds the disc there with
the right arm while the left arm lifts the slack cable over the post and lays it down on the robot's side, then
finishes the turn and lets go.

The reference solution splits the work by side (arm bases at y +/-30 cm): the knob's path stays on the right arm's
side (y -11 to -3 cm) and the post and clip on the left arm's side (y 7 and 18 cm); while the left arm moves the cable,
the right arm holds the turntable still. Holding matters: in the `reroute_only` test (disc not held) lifting the cable
over the post dragged the disc 69 deg round. Offline IK (gripper down, yaw 0): the right arm cannot reach the cable beyond the post (22 mm position error at x 37.5, y 10.5), so it cannot do the reroute; the left arm reaches the knob at most angles and the cable, so a single left-arm solution is not ruled out and was not tried.

## Setup and materials

Positions are in cm from the midpoint between the two arm bases on the table top (x away from the robot, y to the
robot's left). Everything except the disc, the knob and the cable is fixed to the table.

| Part | Specification (as simulated) | How to build it |
|---|---|---|
| Table | flat, level work surface | any smooth bench top; screw or clamp the fixed parts to it |
| Axle | vertical, at x 30, y -7; the disc may lift at most 3 mm on it | 6 mm steel pin (e.g. an M6 shoulder bolt) fixed upright in the table, 15 mm proud |
| Turntable disc | 120 mm dia, 20 mm thick, 100 g, rigid; rests flat on the table and turns on the axle | 3D-printed PLA disc (or light plywood) bored 6 mm on centre, a plain sliding fit on the pin (no ball bearing), trimmed or weighted to 100 g +/- 10 g, with a thin rubber sheet glued under it so it grips the bench; light grey |
| Bearing drag | the disc's own weight sliding on the table (rubber-like contact, friction coefficient about 0.75) plus 0.004 N m s/rad viscous damping: about 0.03 N m to keep it turning, so it stays wherever it is left | check by pulling the knob tangentially with a spring scale: it should start turning at about 0.7-0.8 N; add a felt ring under the disc if a smoother table gives less |
| Red arrow | flat red decal on the disc top, from the centre to 5.4 cm out, 1 cm wide, with a 14 x 22 mm head; points +x at the start | red vinyl tape |
| Knob | dark grey cylinder 20 mm dia, 25 mm tall, 10 g, on its own vertical pin 4 cm from the axle (on the right, -y, at the start), free to spin on that pin | printed or turned cylinder on an M3 screw through the disc, so a pinch on the knob does not lock the disc |
| Cable | black, round, 6 mm dia, 56 cm long, 30 g (54 g/m), no plugs; bending stiffness about 2e-4 N m^2 (a 10 cm length held level at one end droops about 3 cm at its tip); friction coefficient 0.8; does not twist | a soft, silicone- or soft-PVC-jacketed 6 mm round cable (e.g. a thin multi-core control cable); test the droop |
| Strain relief (swivel) | the cable's first 2 cm is pinned to the disc's far rim at x 36.35, y -7 by a vertical pin; it swivels from tangential to the left (+y, as laid) clockwise through radial to the other tangent, 220 deg of range (-200 to +20 deg), and turns with the disc | a short rigid sleeve (rubber cable boot) on a vertical M3 pin in a tab at the rim, with printed end stops |
| Post | dark grey cylinder 25 mm dia, 45 mm tall, rigid, fixed at x 33.3, y 7 | 25 mm aluminium round bar or hardwood dowel, screwed down from below |
| Cable clip | rigid closed bridge clip, dark grey: two side walls 6 mm thick, 16 mm high, 20 mm long, 18 mm apart, and a 4 mm top bar, so the jaw (the tunnel the cable runs through) is 18 mm wide, 16 mm high and 20 mm long along y; centred at x 36.35, y 18, fixed | printed PLA bridge screwed to the table; the cable must slide through it freely |
| Green mark | green tape, 14 x 24 mm, at x 22.8, y -7 (the disc's robot-side edge) | green tape |

Start state: the arrow points +x (away from the robot) and the knob is on the right. The cable leaves the rim
tangentially to the left, runs straight along +y at x 36.35, 3 cm beyond the post's far side, through the clip
(25 cm of cable from the rim to the clip centre), and lies loose to y 49.

Reset: turn the disc back to the start angle; lay the cable straight from the strain relief along +y through the
clip with the tape mark on the cable at the clip, the rest straight beyond it. Check the post and clip are on their
marks. Nothing is randomised.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cabledemo.py` and `vision.py` are in this folder (cable simulation, scripted-demo runner and the
  camera helpers the demo uses). `task.py` adds, on top of the kit: the turntable's axle (a joint free only in yaw
  and +/-3 mm of height, plus light damping), the knob's own vertical pin on the disc, and the swivel pin between the
  disc rim and the cable's first segment.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Turn the camera turntable half a turn so its red arrow points at the green mark, then let go. Its cable must not end up wound round the turntable or wrapped round the grey post."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

The demo chooses every motion from the top camera image, the arms' own gripper poses and the fixed layout above
(axle, knob start position, post, clip); it never reads object poses from the simulator (only its printed log
does). The left arm first parks low on the left, out of the camera's view. The right arm pinches the knob at its known
start position (wrist camera looking across the disc) and swings it anticlockwise round the axle in 12 deg moves,
each move rotating the gripper's own current position about the axle, to about 60 deg. Holding the disc there, the
demo finds the black cable 3.5 cm past the post in the top image (dark grey pixels back-projected onto the table,
a line fitted for position and heading), the left arm takes it there (wrist camera looking back along it at the post
and disc), lifts it over the post, lays it down well clear on the robot's side (pulling some spare cable through the
clip), lets go and parks; the demo looks again and repeats the carry if the cable is still beyond the post. The right
arm finishes the half turn, lets go and parks; the red arrow's angle is then measured in the top image and, if it is
more than 4 deg off the mark, the right arm re-takes the knob where that angle puts it and corrects the turn.
In the recorded run (`demo/`) the camera put the cable at x 38.2 cm where it lay at 38.0 cm; the left arm carried it
over the post on the first try; the half turn ended with the arrow 6.6 deg from the mark (inside the 10 deg limit,
so no correction was needed after letting go), and success held for 1 s, ending the episode at step 490 (~25 s):
task_score 1.0. `demo/` holds its videos, `setup.png`, `run.log` and `result.json`.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s with the disc and knob at rest)

All cable tests use the cable's 3D centre line (segment centres and both tips, resampled every 4 mm).

- **Through the clip (3D).** A centre-line point is *in the jaw* when it lies inside the clip's tunnel: within
  9 mm of the clip's centre line across it, within 10 mm of its centre along it, and between the table and the
  16 mm high underside of the top bar. The cable passes through the clip when a run of consecutive in-jaw points
  spans at least 11 mm of the jaw's length, the points just before and just after that run lie on opposite ends of
  the clip, and at least 2 cm of cable continues beyond each end. A cable lying over the top bar, beside a side
  wall or across the clip's mouth is not in the jaw. The cable's *run to the clip* is its centre line from the
  connector to the first in-jaw point of that run.
- **Winding angles.** For an axis, the winding angle of the run to the clip is the sum of the signed angles between
  consecutive centre-line points seen from that axis in the top view (+ = anticlockwise). It is under half a turn
  when the cable goes past the axis on one side, and half a turn or more when it goes round it. Measured: about
  +150 deg round the post at the start (passes beyond it), about -160 deg after the half turn with the cable on the
  robot's side of the post (whether close to it or well clear), about +200 deg when the cable is hooked round the
  post's far side, and 360 deg more for each full loop round it. With the arrow anywhere within 30 deg of the mark
  (the only angles that can score), a cable on the robot's side measures -157 to -170 deg and one round the far side
  +190 deg or more, so the half-turn threshold separates them.
- **0** if physics is NaN; if either gripper (TCP) is within 3 cm of the cable, the disc or the knob, or the gripper
  positions cannot be read (not let go); if the cable does not pass through the clip; if the run to the clip winds
  half a turn (180 deg) or more about the turntable axle (wound round the turntable: about +76 deg at the start,
  about +255 deg after a clockwise half turn); or if it winds half a turn or more about the post's axis (wrapped
  round the post).
- Otherwise **1.0** if the arrow is within 10 deg of the green mark, **0.5** if it is more than 10 and at most 30 deg off, else 0. End states: 1.0 = half turn done to within 10 deg, both grippers 3 cm or more clear, cable through the clip on the robot's side of the post and not wound; 0.5 = the same but the disc stopped 10-30 deg short of or past the mark (for example the turn ended 20 deg early and the hands were then lifted away).
- `distance()`: arrow error in half turns, +3 if the cable is out of the clip, else +1 if wound and +1 if wrapped.
  `progress()`: stage diagnostic (through clip, turned, not wound, post clear, released).

## Unintended strategies (test only, scored in sim; each logged run scores 0)

Robot variants, selected with `TT_DEMO=<mode>`; scene check selected with `TT_CLIP=mouth`:

| variant | what it does | result |
|---|---|---|
| `TT_DEMO=nothing` | waits | 0: arrow 180 deg off the mark |
| `TT_DEMO=ccw_no_reroute` | turns anticlockwise half a turn without moving the cable first | 0: arrow 5 deg from the mark and released, but the turn dragged the cable round the post's far side so hard that it was pulled out of the clip (no cable left in or near the jaw) |
| `TT_DEMO=cw` | turns clockwise half a turn (the other way) | 0: arrow 4 deg from the mark and released, but the cable winds 250 deg round the turntable and 199 deg round the post |
| `TT_DEMO=reroute_only` | lifts the cable over the post to the robot's side without holding or turning the disc | 0: lifting the cable over the post dragged the free disc 69 deg round: arrow 111 deg off the mark |
| `TT_DEMO=hold` | the reference solution, but the right arm keeps pinching the knob at the end | 0: arrow 7 deg from the mark, cable through the clip and on the robot's side of the post (-158 deg), but the gripper never lets go |
| `TT_CLIP=mouth` | scene check: the clip is turned 90 deg and set beside the cable with its mouth 1 mm from it, so the cable runs along the clip but not through its jaw; the reference demo then runs | 0: arrow 5 deg from the mark and released, but the cable is not through the clip |

The wrap cases the robot cannot stage on purpose were also checked offline on cable centre lines (synthetic paths
and saved sim states) against the two cable tests: a cable lying over the clip's top bar, across its mouth or along
a side wall is out of the clip; a cable on the robot's
side of the post, touching it or 10 cm clear, winds about -160 deg round it and passes; one hooked round the far
side, tight or in a wide loop, winds about +198 deg and one looped once fully round the post about +558 deg, and both
score 0; a saved sim state where the cable was dragged round the post's far side gave +197 deg. An attempt to make
the robot carry the cable once round the post in the air did not leave a loop on the post (the cable slid off it),
so no robot run of a full loop exists.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). The scripted demo ran at about 0.7 steps/s on a shared GPU (490 steps); a
  trial that uses all 2,400 steps would take about an hour at that rate, and a few minutes on an unshared GPU.
- Layout fixed with `object_jitter=0`; the cable is never jittered (it always starts straight). Always run with
  `object_jitter=0`.
