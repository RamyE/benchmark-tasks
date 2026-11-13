# Weave a wire past formboard pins along the drawn path, then plug it in

A wire-harness formboard lies in front of the robot: a white layout sheet with a row of five steel routing pins
across the board (9 cm apart), a larger steel turn post to the right of the row, and a blue route line printed
between them. An orange 7 mm wire, 95 cm long, leaves a fixed start fixture at the front-left of the board and lies
straight across the front of the board towards the right arm, with a connector plug on its free end. The blue line
runs from the start fixture into the row and **weaves through it as a slalom**: behind pin 1, in front of pin 2,
behind pin 3, in front of pin 4, behind pin 5. It then **turns sharply round the front and right side of the turn
post** (about 110 degrees) and runs straight into a black mating connector just past the post. The robot must lay
the wire so it passes every pin and the post on the same side as the blue line, push the plug into the connector,
and let go.

What makes the path hard:

- **Alternating sides.** Each pin is passed on the opposite side from its neighbour, so the free end has to be
  threaded forward and back through every gap. A straight pull from the start to the connector, or one sweep of the
  wire along either face of the row, leaves at least two pins on the wrong side.
- **A sharp turn round the post.** From behind pin 5 the line swings out in front of the post and round its right
  side into the connector. Going straight from pin 5 to the connector passes the post on its far side, which is
  the wrong side of it.
- **Both arms by reach.** Offline inverse kinematics at table height: the left arm reaches the start fixture and
  pins 1-4 but not pin 5, the post, the connector or the plug's start; the right arm reaches the plug's start, pins
  2-5, the post and the connector but not the fixture or pin 1. The plug has to be carried from the right-only area to
  pin 1 and back to the connector, so the free end is handed over in the middle of the board (pins 2-4 are within
  both arms' reach) and each arm threads its own half.
- **Little slack.** The 95 cm wire is about 19 cm longer than the shortest path that passes each pin and the post
  on its drawn side (about 76 cm of wire to the plug's seat, estimated from the layout), and the printed line itself
  is about 94 cm long, so wide loops cannot be afforded. Each pin has to be passed close in, and the free end must be
  fed through every gap without lifting the wire over a pin (40 mm tall, and any wire point more than 25 mm above the
  board scores 0).

The rule is topological and is judged pin by pin from the top view: slack and exact wire position do not matter,
only which side of each pin the wire is on.

## How this version raises the challenge

The path is a five-pin alternating weave (behind, in front, behind, in front, behind, 9 cm pitch) followed by a
turn of about 110 degrees round a post into the connector at (40, -32) cm, and the plug starts where only the right
arm can reach. A straight pull or one sweep along the row scores 0, and the free end has to be handed from one arm to
the other mid-board. Score tiers are 0.1 per pin up to 0.6, then 0.7, 0.8 and 1.0 for the plug, within a 2400-step limit.

## Closest tasks

- TK-CP66NZ scores a wire routed between two posts and seated in clips, passing a marked gap by height and crossing
  position; this task scores which side of each of five pins and a post the wire passes, read from the top view, and
  then plug insertion.
- TK-ARD8HC scores a rope's crossings of a line through two posts for a figure-eight; this task scores the wire's
  side of five alternating pins and a post along a printed route, with a connector seated at the end.
- TK-GQN8AP scores sections of a cord pressed into a channel; this task has no channel, and scores side-of-pin
  topology along a path.

Ability tested: laying a free-hanging wire's far end through alternating gaps and round a sharp turn by handing it
between two arms, then seating its plug.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cabledemo.py` and `vision.py` are in this folder (cable simulation, scripted-demo runner, top-camera
  helpers used by the demo).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Route the orange wire on the harness board along the blue printed line: the wire must pass every steel pin on the same side as the blue line. Then push the wire's plug into the black connector at the end of the line and let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

The demo chooses its motions from the top camera image (the black plug's position and direction), the arms' own
joint state and the fixed board layout; it never reads object poses from the simulator. The right arm picks the plug
up at the front-right, carries it back past the middle of the board between the wire and the row, and lays it
down there. The left arm picks it up, leads it round the left of pin 1 to behind it, then diagonally forward through
the first gap with the plug pointing along the diagonal, turns it in front of pin 2, diagonally back through the next
gap, turns it behind pin 3, forward through the next gap and lays it down in front of that gap. The right arm picks it
up again, leads it past the front of pin 4, diagonally back behind pin 5, forward in front of the post, round its
right side, and lays it down in front of the connector, looks at it again, re-grips it square and pushes it in; once
the tip is in, it re-grips further back (clear of the connector face) and pushes it home. Both arms let go and park.

Test-only variants (environment variable `PINS_DEMO`, read in `task.py`). These variant runs used a debug wrapper
that gave the demo the plug's pose from the simulator instead of the top camera; the scores come from the resulting
wire shapes, and the full demo above is the camera-driven one:

| variant | what it does | score |
|---|---|---|
| `nothing` | no motion | 0 |
| `shortest` | right arm only: plug straight to the connector in front of the row and the post, pushed in | 0 (pins 1, 3 and 5 on the wrong side) |
| `sweep` | the full two-arm handling, but the plug is swept once behind the whole row instead of weaving, then round the post and in | 0 (pins 2 and 4 on the wrong side) |
| `onearm` | right arm only: threads its own half (pin 4, pin 5, the post) and pushes the plug in | 0 (the wire passes in front of pins 1-3, the wrong side of pin 1) |
| `hold` | the full demo, but the plug is never let go | 0 |
| `half` | left half only (pins 1-3 woven), plug left in front of the gap between pins 3 and 4 | 0.3 |
| `noplug` | every pin and the post woven, plug laid with its tip 2 cm short of the mouth (centre 4.75 cm from it, outside the 4 cm limit) | 0.6 |

## Scoring

`score()` is evaluated at the end of the episode. Each route pin (pins 1-5, then the post) has two short **gates**:
straight lines from the pin centre 3.5 cm (post: 5 cm) towards the side the blue line passes, and towards the
other side. Following the wire in top view from the start fixture to the plug, a pin counts as **passed on the
drawn side** when the wire crosses its drawn-side gate exactly once, in the line's direction of travel, and never
crosses its other gate; it counts as **wrong side** when the wire crosses the other gate, or crosses the drawn-side
gate backwards or twice; otherwise it is **not yet passed**. When the plug is within 4 cm of the connector mouth or
inside it, the wire is also closed with the printed line (plug -> mouth -> line back to the start) and the winding
number of that loop around every pin and the post must be 0; a non-zero winding marks that pin wrong side too.

- **Score 0** if any pin or the post is on the wrong side; the wire lies over the top of a pin or the post; any
  part of the wire is more than 25 mm above the board; or either gripper is within 3 cm of any point of the wire or
  of the plug centre (not let go), or the gripper positions cannot be read.
- Otherwise, **0.1 per pin passed on the drawn side, counted in route order** from the start fixture up to the
  first pin not yet passed (pin 1, 2, 3, 4, 5, post: 0.1 to 0.6).
- With all six passed: plug centre within 4 cm of the mouth, not in it, **0.7**; plug tip inside the cavity mouth
  (centre within 6 mm of its axis, pointing within 30 deg of it) **0.8**; plug tip at least 12 mm into the cavity
  **1.0**.
- The episode ends early as a success only after the score is 1.0, the wire is at rest and both grippers are at
  least 10 cm from the plug.

`progress()` reports stage diagnostics (fraction of route pins passed in order, any pin on the wrong side, wire on
the board, plug near / seated, released); it does not change the score.

## Runtime and seeds

The layout is fixed with `object_jitter=0`; the wire is not jittered. `VARIATION` in `task.py` lists seeded ranges for
a later runner (wire spawn heading +-3 deg).
