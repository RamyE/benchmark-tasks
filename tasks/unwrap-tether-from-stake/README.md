# Unwrap a grazing tether from a post by carrying its collar back round against the wrap

A tethered-grazing tangle on the table. A red 8 mm tether runs from a ground ring (its fixed end) to a wooden post,
goes round the post counter-clockwise (seen from above) and crosses over its own standing part, then runs out to a
blue collar block that lies far out on the right. A white painted circle (the grazing spot) is marked far out on the
left. The robot must take the tether off the post and leave the collar in the circle.

The post is 18 cm tall, so the turn cannot be lifted off over the top: the only way to undo it is to carry the
collar back round the post against the wrap, clockwise, which is the long way round past the robot's side of the
post. The short way to the circle, straight across the far side of the post, winds more of the tether on instead of
taking the turn off, and the tether runs out before the circle. Which way is right can only be read from the lay of
the tether round the post. Once the turn is off, the standing part still lies against the post; full score needs it
laid clear.

## Scene (station table; cm from the midpoint between the arm bases, x forward, y left, heights above the table)

| object | type | size | material (real rebuild) | position |
|---|---|---|---|---|
| post | fixed cylinder | 7 cm diameter, 18 cm tall | brown-painted wooden dowel, screwed down | axis x 36, y 0 |
| ground ring | fixed block | 3 x 3 x 1.4 cm | dark grey steel plate (tie-out ring), screwed down | centre x 38.0, y -27.0 |
| tether | rope on a sprung swivel at the ring | 8 mm, 80 cm, 75 g | red 8 mm braided cord (stand-in for a coated tie-out cable) | leaves the ring at x 38.2, y -24.9 heading 85 deg (along +y, towards the post), meets the far side of the post at x 40.2, y -0.4, goes about 210 deg round it counter-clockwise (centre line 4.25 cm from the axis), crosses over its standing part about 1 cm up near x 39, y -16, and runs straight out to the collar |
| shock spring and swivel snap | joint at the ring | about 20 N/m along the tether's lead | tie-out shock spring with a swivel snap | the tether's ring end turns freely about the vertical and can be drawn out along its lead against the spring |
| collar | rigid block on the tether's free end | 40 x 24 x 20 mm, 20 g | blue PLA (stand-in for a collar and buckle) | centre x 50.0, y -39.5 |
| white circle | painted ring, 1 mm thick | inner radius 4 cm, 8 mm wide | white paint or tape | centre x 50, y 40 |

The start layout is written at every reset (deterministic) and is the one in `demo/setup.png`.

## How this differs from similar tasks

- Nearest board task: TK-52TM2W "Take a round turn on a bollard" (ours) winds a line on a post from a straight start;
  the direction does not matter there, and the end state is a turn on the post. Here the start is a tangle the robot
  did not make, the end state is the opposite (no turn, line clear of the post), and the direction is the decision: it
  must be read from the lay, and the short, convenient way winds more on and cannot reach the circle (scores 0).
- TK-MZ7JMD "Turn a camera turntable without wrapping its cable" (ours) is about not creating a wrap while turning a
  device; TK-RLZC9K "Recover a crawler by its tether around a wall" (ours) hauls a tethered device round an obstacle by
  its line. Here no device is turned and the line is never hauled: a wrap that is already there is undone by leading
  its free end back round the post, and towing the collar by its tether scores 0.
- Our unwind-oxygen-tubing-to-rail (nearest, HIGH risk): a tube wound one turn round a capped post is walked back round it
  against the winding (direction read from the lay), with a hand-over. The act (undo a turn round a capped post by
  carrying the free end back round, with a reach-forced hand-over) is the same; only the line, the end placement (clip
  and cradle there, circle here) and the no-haul/no-tow latches differ.
- Our uncross-two-cables task undoes a crossing of two cables on a flat table; here one line is wound round a fixed
  post, and the topology (turns round the post) is what is scored.

## Why two arms

Reach, from offline position-only IK (any gripper orientation, `scratch/ik_final.py`, output `scratch/ik_final.out`)
and the scripted planner's straight-down command from the ready pose (`scratch/planreach.py`). Each arm's
position-only reach at table height ends 76-78 cm from its base, at every bearing (`scratch/reach_env.py`).

- The collar starts at x 50.0, y -39.5: the right arm reaches it (0 mm, also straight down). The left arm does not,
  with any gripper orientation: 91.9 mm short at its centre and 79.8 mm short at its near end (x 49.5, y -37.5);
  the planner stops 382 mm short.
- The circle is centred at x 50, y 40: the left arm reaches it (0 mm, straight down). The right arm does not, with
  any gripper orientation: 95.4 mm short at the centre and 57.2 mm short at the circle's nearest point to it
  (x 47.7, y 36.7), so it cannot reach any point where the collar centre would count as in the circle.
- Both arms reach the near side of the post (x 22, y 0: 0 mm each), where the demo hands the collar over.

So neither arm alone can both pick the collar up and set it in the circle, and the collar may not be towed by its
tether (score 0): one hand-over is needed. At the end the standing part still lies against the post; laying it clear while the
collar is held in the circle (so it is not towed out) is the demo's second two-handed step, though one arm could
also lay it after letting the collar go. With a strongly tilted gripper the IK reaches further across, so this is a
reach limit for top-down grasps, as on the real arm with a top-down grasp policy.

## Context and sources

- Australian Veterinary Association, "Guidelines for the tethering of animals"
  (www.ava.com.au/policy/guidelines-for-the-tethering-of-animals): the site should "be clear of obstructions that may
  cause the tether to become entangled or cause injury to the animal"; "an animal can be choked when the tether
  becomes entangled"; "The anchor point must allow the animal to cover the area without tangling."
- RSPCA (UK), "Tethering horses" (www.rspca.org.uk/adviceandwelfare/pets/horses/health/tethering): "If a horse is
  tethered to a tree, it can become entangled and harm itself." A tethered horse "needs high levels of monitoring."
- O*NET 45-2093.00 Farmworkers, Farm, Ranch, and Aquacultural Animals, task: "Shift animals between grazing areas to
  ensure that they have sufficient access to food."

The sources say that tethers get wound round posts and trees and that this is dangerous; the procedure (lead the
animal's end back round against the wrap, never haul the animal by its tether) is ordinary handling practice, not a
quoted rule.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (the rope simulation, the wound start layout
  and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Unwrap the red tether from the wooden post and leave its blue collar in the white circle."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

The scripted demo (`demo/`) works closed loop from the simulated state:

1. The right arm picks the collar up and carries it clockwise round the near side of the post, 8 cm above the table
   so the tail lifts clear of the standing part, and sets it down at x 22, y 0 (the near side of the post).
2. The left arm picks it up there and carries it on clockwise round the left of the post and out to the circle. The
   turn comes off as the collar passes the line from the post to the ground ring.
3. The left arm draws the collar out a little past the circle centre (watching the slack, so the ring spring is not
   drawn out) and lowers it into the circle, still holding it.
4. The standing part of the tether still lies against the far side of the post. While the left hand holds the
   collar, the right hand takes the tether just before and just past the post (y -7, y 7) and lays it up to 6 cm further
   out (stopping before the ring spring is drawn out), lets go and moves
   clear; then the left hand opens and moves clear.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

`turns()` follows the tether's centre line from the ring to the collar and counts its winding number round the post
axis (sweep of the line about the axis minus the sweep of the straight chord, divided by a full turn; an exact
integer). It starts at 1. The other checks are positions of the tether, collar and gripper tips.

- 0 at once and for the rest of the episode (latched) if at any step: the tether went round twice (|turns| >= 2);
  any tether point was within 1 cm of the post top and within 3 cm of its side (taken over the top); the ring end
  was drawn more than 2 cm out on its spring (the tether was hauled on); the collar moved more than 3 cm with no
  gripper tip within 5 cm of it (towed by its tether); or the state went non-finite.
- 0 at the end unless both gripper tips are at least 3.5 cm from every point of the tether and collar, turns is 0,
  and the collar centre is inside the circle (within 4 cm of its centre, resting on the table).
- Then 0.5 (unwound and placed, but the tether still touches or lies against the post), and 1.0 if every point of
  the tether and collar is at least 1 cm from the post surface. `success()` is score 1.0 with the collar at rest,
  held for 1 s.

## Shortcuts (scripted variants, `TETHER_CHEAT=<variant>`; evidence in `scratch/sc_<variant>.json/.log`)

| variant | what it does | end state | score |
|---|---|---|---|
| (demo) | the scripted demo above | turn off, collar in the circle, tether 2.7 cm clear of the post, both hands clear | 1.0 (692 steps) |
| `none` | nothing | still wound, collar at its start | 0 |
| `hold` | the full demo, but the left hand never lets go of the collar | everything right, collar still held | 0 |
| `wrong` | the short way: right arm carries the collar counter-clockwise across the far side of the post and sets it down there (x 46, y -5) for the left arm to carry on (reaching the circle that way would need about 91 cm of tether; it is 80 cm). In this run the left arm's grasp at that spot misses, so the collar stays there | wound further on (turns 1), collar 46 cm from the circle centre | 0 |
| `right` | right arm alone: clockwise round the near side and on round the left side as far as it reaches, then lets go | still wound (turns 1), collar outside the circle | 0 |
| `pull` | right arm grips the collar and hauls it towards the robot and out to the right (to x 10, y -55) | ring spring drawn out 102 mm (latched); the collar ends at x 10, y -49, still wound (turns 1) | 0 |
| `leftdrag` | left arm alone: pinches the tail where it can reach it and drags it to bring the collar into reach, then tries to finish | the pinch slips on the line and the collar never moves; still wound (turns 1), collar at its start | 0 |
| `lift` | right arm lifts the collar high and over the top of the post | tether over the post top (latched), still wound | 0 |
| `touch` | the demo without drawing the collar out and without laying the standing part clear | turn off, collar in the circle, hands clear, tether still against the post | 0.5 |

Limit: the start layout is fixed, so a policy that always carries the collar clockwise round the near side would
score without reading the lay. A uniformly random choice between the two directions is right 1 time in 2; the
wrong one (the shorter, more convenient one) always scores 0. We keep one fixed start layout (the review rules
forbid per-episode swaps or mirroring), so this is the fixed-layout limit, not something the scoring can block.

## Paper fields

- Skill tags: winding (unwinding), handoff, sequencing, placement/pinning.
- Industry: agriculture (livestock on tethers); also any line fouled round a post (boat painters, hoses round
  stakes).
- Two-arm need: reach-forced (collar start right-only, circle left-only, one hand-over); the demo also holds the
  collar with one hand while the other lays the tether clear.

## Runtime and seeds

The start layout is deterministic (no jitter; `VARIATION` lists zero offsets). The scripted demo finishes at step 692
of the 1,200-step limit (`demo/result.json`). Wrist-camera mean brightness (0-255, sampled every 2 s of the demo):
left 118-194, right 66-186; the top camera sees the post, the whole tether, the collar and the circle throughout.
