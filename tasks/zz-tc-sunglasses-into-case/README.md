# Put sunglasses away: fold both temples holding the frame, lay them in the case, snap it shut

Putting sunglasses away after a drive. The glasses stand open on the table on the lower rim of the front and the
bent tips of their temples, lenses toward the robot. Nearer the robot lies their hard clamshell case, open on its spring
hinge. The robot folds both temples in, puts the folded glasses in the case and closes the lid.

Two hands make it go well: the frame is light, so a temple swung in at a normal pace drags or turns it unless the
other hand holds the front (a very slow one-handed push can fold a temple, letting the frame slide a little, which is
allowed); the folded glasses are lifted and carried with a hand at each end so they go into the case square and
level; and one hand braces the light case while the other pushes its sprung lid over.

Instruction: `Fold the sunglasses and put them away in their case, then close it.`

## What makes it hard

- **Folding a temple on a 48 g frame.** The hinges resist being swung, more the faster they turn, so pushing a
  temple in at a normal pace drags, turns or tips the whole frame unless the other hand holds the front. The hand that holds must stay clear of the hinge being
  folded, and the folding hand sweeps the temple round its hinge, not just pushes it.
- **The second temple goes over the first.** Folded, the first temple lies flat behind the lenses and the second one
  crosses over its bent earpiece, so the second stops about 70-90 deg in, not flat.
- **Into the case.** The folded glasses are 133 mm long in a case 150 mm long inside: they have to go in square and
  level, and loose temples swing open if the glasses are carried tilted.
- **The lid.** Its spring holds it open until it is pushed past about 40 deg; then it snaps shut. Pushing the open lid
  over slides the light (75 g) case unless the other hand braces it. A temple left open stops the lid from shutting.
- Knocking the glasses over on the table (letting them fall onto the lenses or onto their back) scores 0, so the
  front has to stay upright, held, while the temples are folded. Laying them down on purpose, slowly, is allowed. Putting them into the case with a temple still open, so
  that the case would fold it, also scores 0: both temples must be folded before any part of the glasses goes in.

## Real-world setup

Positions in cm from the midpoint between the two arm bases, x forward, y left, on the table top.

| item | real part | size / position |
|---|---|---|
| sunglasses | Ray-Ban RB2140 Original Wayfarer, size 50-22-150, acetate frame, glass lenses | size code 50-22-150: 50 mm lens width, 22 mm bridge, 150 mm temples; front about 133 mm wide and lenses about 41 mm tall (retailer listings); about 48 g with lenses. Open, standing on the lower rim and the temple tips, front centre at x 27, y 0, lenses toward the robot |
| case | XINPORT large hard clamshell sunglasses case | outside 154 x 62 x 56 mm (base 32 mm, lid 24 mm), inside about 150 x 58 x 50-52 mm; about 75 g. Long side along y, centred at x 15, y 0; lid hinge along the far (+x) edge; lid open against its 100 deg stop |

The case is listed with "Inside Dimensions: 15cm (5.9") x 5.8cm (2.2") x 5cm (2.0")". The sizes above come from the
makers' size codes and retailer listings: measure and confirm the glasses (front width, lens height, temple length,
weight) and the case (inside length, width, depth) before the first trial; the folded glasses must fit inside the
case lengthwise with a few mm to spare. Before trials, also check that its
hinge spring holds the lid open when it is lifted past about 40 deg and pulls it shut below that; a case whose lid
flops does not fit this task. Check the glasses' hinges too, at the pace the
temples are folded in trials: pushed at the tip (a push gauge, or a finger pressing through a kitchen scale) so that it
swings closed over about 4 s (about 22 deg/s), a temple should need about 0.5-1 N; snug or loosen the hinge screw to
bring it into that band (the sim needs about 0.75 N at that pace). If a pair can't be adjusted into the band, use another pair of similar size whose hinges can be, and record
which pair was used. The sim hinge is viscous (its resistance grows with speed) while a real hinge is mostly friction
(about the same at any speed), so the two agree only around this pace; trials fold the temples at about that pace
(about 4 s per temple, as in the demo). The case's shell material does not matter as long as
it is rigid and its spring hinge behaves as above.

Reset marks: tape outlines of the case base and of the front's lower rim and the two temple tips on the table.

## Scoring (real trial)

From the end photo and the trial video, plus a second photo after the referee reopens the case, showing both
temples folded and the whole pair inside.

- **Score 0** if at any time the glasses are knocked over on the table outside the case: they fall (tip over
  quickly, not lowered by a hand) and end up lying with the front more than 60 deg from upright, on the lenses or on
  its back. In the video a fall is a sudden topple; a careful lay-down by hand is not penalised, if the glasses go into the case with a temple not yet
  folded (folding them by pushing them against the case strains the hinges), if at the end the glasses or the case
  have fallen off the table, or if at the end a gripper is closer than 5 cm to the glasses or the case.
- A temple is **folded** when it is turned 60 deg or more in from its open stop.

| score | end state |
|---|---|
| 1.0 | both temples folded, the glasses wholly inside the case, lid shut (within 3 deg), and both arms parked: every gripper point at least 10 cm from the glasses and the case and still (under 2 cm/s) for the last 0.5 s, so the video ends with the hands back and still |
| 0.5 | both temples folded and the glasses wholly inside the case, but the lid not shut or the arms not parked |
| 0.25 | both temples folded, the glasses not (wholly) inside the case |
| 0 | otherwise |

The sim scorer checks the same: temple angles from the hinge joints' body poses; every corner of every part of the
glasses within the case's inner space (2 mm tolerance) in the case's own frame; the lid angle; knocked over = the front
comes to lie more than 60 deg from upright either way, below 1.5 cm and outside the case's footprint, having turned
faster than 150 deg/s at some point in the 0.5 s before (a fall; a hand laying it down turns it far slower); fallen
off = the front or the case base more than 1 cm below the table top at the end; went in with a temple open = some corner of the glasses within the base's inner footprint and
below its rim while a temple is turned in less than 60 deg; every gripper point 5 cm or more from the glasses' bounding
box and from the case's (base and lid) bounding box, each taken separately. Parked = over the last 10 control steps (0.5 s) every gripper point
10 cm or more from both boxes and moving under 2 cm/s.

## Sim model

- Glasses: front a 133 x 5 x 50 mm plate (lenses and brow bar), two endpieces 12 mm deep holding the hinges 10 mm
  behind the front, each temple a 105 mm straight bar plus a 45 mm earpiece bent 58 deg down (150 mm). Hinges: 0 deg
  open stop (2 deg play), 90 deg folded flat; the hinge's resistance is modelled as viscous damping (0.005 N m s/deg):
  about 0.11 N m, 0.75 N at the temple tip, when swung at 22 deg/s (a 4 s fold, the demo's pace), matching the
  hinge check above at that pace. A real hinge is mostly friction, so the two differ away from that pace: the sim resists less if a temple
  is pushed much more slowly (it has no static hold, so a very slow one-handed push folds a temple), and more if it is
  swung much faster. Masses: front 30 g,
  endpieces 2 g each, temples 5 g, earpieces 2 g.
- Case: base and lid are rigid open shells of 2 mm walls, free on the table (base 45 g, lid 30 g); the lid hinge has an
  over-centre spring: above 40 deg it pushes the lid to its 100 deg open stop, below 40 deg it pulls it shut
  (0.0006 N m/deg), which also keeps it shut.
- Run limit 1,400 steps (70 s at 20 Hz).

## Demo

A scripted demo (planned from the simulator's object poses): the right hand pinches the top of the front while the left
hand sweeps the left temple in; then the hands swap and the right hand folds the right temple over the first. Both
hands then pinch the front at its two ends, lift the folded glasses, carry them over the case and let them down into it.
The left hand braces the case's near side while the right hand pushes the lid over from behind until it snaps shut;
both hands move clear.

## Who does this / why it matters

Most adults who drive or commute own sunglasses or glasses; putting them away folded in a hard case so they are not
crushed or scratched is a daily habit. The skill is two-handed handling of a light articulated object: holding a 48 g
frame still while its hinges are swung, carrying it level with both hands, then bracing a light case while its spring
lid is pushed over.

## How this differs from close tasks

- Putting glasses **on** a person or manikin tests fitting earpieces over ears; here the glasses are folded and stowed.
- Closing a lid or snapping a case shut alone is a one-step act; here it is the last of three, after folding a small,
  light, two-hinge object without moving or tipping it, and the lid only shuts when both temples are fully in.
