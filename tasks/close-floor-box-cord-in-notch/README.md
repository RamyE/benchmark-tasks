# Close a floor box without pinching the cord

In offices, meeting rooms and libraries, the power for a table often comes from a floor box: a recessed well in
the floor with a receptacle mounted face up below the floor surface and a hinged cover on top. Whoever plugs in a
laptop charger or a lamp there plugs the cord in, leads it out through the cover's cord opening and closes the cover
flush. A cover closed over a cord that was not led out rests on the cord, crushes it, and is left standing proud of
the floor.

In this task the floor is a 4.5 cm raised floor across the table, and a floor box is set flush in it in front of
the robot: a white well, 12 x 14 cm inside and 4.5 cm deep, its rim level with the floor. A duplex NEMA 5-15R
receptacle with its mounting strap is fixed face up on the box's bottom. A grey lid, hinged along the box's left
edge, lies on the rim. Its far edge has a cord opening (20 mm wide, 25 mm deep) 5.5 cm right of the box's centre line, over a
matching 1 cm deep cord notch in the top of the far wall.
A black cord comes out of a fixed grey service box on the floor beyond the box, on the right, and runs across the
floor to a one-outlet power strip on the robot's right. Its right-angle NEMA 5-15P plug starts fully plugged into
that strip.

The robot has to unplug the cord from the strip, open the lid, push the plug straight down into the floor box
receptacle until its body sits flat on the receptacle face, lay the cord across the far wall where the lid's cord
opening will come down, close the lid flat and let go. From the receptacle, the cord's natural path towards the
service box runs straight over the far wall behind the receptacle, about 3 cm left of the opening, so a lid closed
straight away comes down on the cord and pinches it.

Instruction: `Plug the black cord into the floor box outlet, close the lid without pinching the cord, then let go.`

## Why it matters

- **Everyday act:** plugging something in at a meeting-room floor box and closing the lid over its cord is
  something most office workers and students have seen or done; a 10-second clip of it is recognisable at once.
- **The failure it guards against:** floor box makers design their covers around cords left plugged in. Thomas &
  Betts says its Steel City Residential Floor Box "allows lid to be closed, even when cords are plugged in", and its
  68R cover listing says "Lid can be rotated to expose a small opening for cords, enabling the lid to close flush with
  the floor while in use". Legrand's Evolution EFB45 covers have cable egress doors for the same purpose. The lid's
  opening here is modelled loosely on the cord opening in Steel City's 664-CST-SW recessed floor box
  cover (catalog drawing: "Lid Rotated 180º to expose Cord Opening."): a small opening in the edge of the lid that the
  cord leaves through with the lid shut flush. Its position and shape in the drawing are not copied. It is simplified to an open 20 mm slot, and the
  wall-top notch under it, which stands in for the cover frame's cord passage, has no door or gasket.
- **The ability tested:** in one run the robot must seat a three-prong plug along the receptacle axis by feel (the pin
  and blades have only 0.3-0.4 mm of clearance a side), then move the middle of the same cord, now fixed at both ends,
  sideways to where a narrow opening in the lid will come down, and only then close a hinged lid over it. Each step is checked by the end state, and an early
  error (a half-seated plug, a cord left on the wall top) cannot be repaired by the last step.

## Plug and receptacle

The plug follows the NEMA 5-15 geometry:

| Part | Model | Source |
|---|---|---|
| Flat blades | 6.3 mm wide, 1.5 mm thick, 15.9 mm long, 12.7 mm apart (centre to centre), faces towards each other | worldstandards.eu, North American plug page: "two flat 1.5 mm thick blades, spaced 12.7 mm apart, measuring 15.9 - 18.3 mm in length and 6.3 mm in width" |
| Ground pin | round, 4.8 mm diameter, 19.1 mm long (3.2 mm longer than the blades), 11.9 mm from the middle of the blade line | same page: "4.8 mm diameter round or U-shaped earth pin, which is 3.2 mm longer than the two flat blades"; "the earth pin is offset by 11.9 mm" |
| Blade spacing, receptacle | 12.7 mm, the same as the plug | Wikipedia, NEMA connector: "blade spacing is 0.5 inches (12.7 mm) for both" |
| Receptacle size | 107 mm long overall with its mounting strap, 34 mm wide | A distributor lists the Leviton 5320-WCP duplex 5-15R at "4.2 in H x 1.33 in W" (about 107 x 34 mm). The model has a 70 x 34 mm moulded face and the strap's two ears beyond it, 107 mm end to end; the 70 mm split between face and ears is a modelling choice, not taken from a source |
| Neutral blade | modelled at 6.3 mm like the hot blade | A real neutral blade is wider (7.9 mm); not modelled |
| Contact grip (retention) | 13.3 N to pull the plug out, and about the same plus a short peak to push it home | UL 498, as quoted by IAEI Magazine (Sept/Oct 2013, "A journey back to basics with receptacles"): a receptacle must "hold (retain) a polished steel 2 bladed plug gauge without holes (the grounding blade removed) against a 3 lb force for 1 minute" and "release a polished steel 3 bladed plug gauge with holes at no more than 15 lb of force". The model sits at the 3 lb (13.3 N) minimum; a new receptacle can grip up to the 15 lb (67 N) limit |

- **Plug body:** black moulded right-angle body, 36 x 30 x 20 mm, 40 g. The cord leaves its back end at mid-height.
  The blades and pin are rigidly fixed to the body's face (5 g each in the model). The ground pin sits on the cord
  side of the blades.
- **Receptacle:** ivory moulded face, 70 x 34 mm, 3 mm thick, 22 mm above the box floor, with its long side running
  towards the far wall. It has two outlets, 38 mm apart. Each outlet has two blade slots, 7.0 x 2.3 mm and 12.7 mm
  apart, and a 5.4 mm ground hole 11.9 mm from the blade line on the far-wall side. That leaves 0.35-0.4 mm a side
  around each blade and 0.3 mm a side around the pin, so the plug only goes in when it is square to the face and
  lined up with its openings, and it goes in straight down the receptacle's axis. A seated plug's cord leaves towards
  the far wall. The metal mounting strap's ears (18.5 mm each, 22 mm wide) stick out beyond both ends of the face, so
  the device is 107 mm long and fits the box's 120 mm inside length.
- **Blade contacts:** under each blade slot, 7 mm below the face, a spring contact stands on each side of the blade's
  path, as the brass contacts of a real receptacle do (Leviton lists "Heavy-duty double-wipe brass
  contacts" for its 15 A duplex receptacle 5325). Each is a rounded brass contact (5 mm across) on a short slide, pressed towards the blade by a
  preloaded spring with a near-constant 6.7 N. A blade going in pushes both contacts back about 0.15 mm and slides
  between them. With a contact friction of 0.5, the four contacts of an outlet hold the plug with 13.3 N, and pushing
  it home takes the same 13.3 N plus a short peak as the blade tips ride over the contacts' rounded faces. The power
  strip has the same contacts, so unplugging the cord from it also takes a 13.3 N pull.
- **Insertion is physical:** the body below the face is hollow, so the blades and pin slide through the openings and
  between the contacts until the plug body's face lands on the receptacle face (15.9 mm of blade inside). Nothing
  snaps or moves the plug: a plug that is off its openings stands on its pin or blade tips, one that is tilted jams,
  and one pushed only lightly stops with its blade tips on the contacts, well short of seated.
- **Power strip:** dark grey, face 44 x 40 mm, 22 mm tall, one outlet. Its openings are looser (10 x 4 mm slots,
  8 mm ground hole) and it has the same blade contacts. The plug starts fully in it: the plug settles into the strip
  in the first second with the strip's contacts held back, then they close on its blades (the strip's contacts are
  pushed back 0.11-0.12 mm by the seated blades from then on). It is only where the cord starts and is not scored.

## Scene

Table top = reference; x forward from the line through the arm bases, y left of their midpoint.

- **Floor:** a raised floor, 4.5 cm thick, fixed on the table from x 7 to 85 cm and y −55 to +55 cm, with a cut-out
  for the floor box. Its top is level with the box's rim. The arm bases stand on the table in front of it.
- **Floor box:** white walls, 1 cm thick and 4.5 cm tall, fixed, set in the floor's cut-out. Outer size x 13.5–27.5 cm,
  y −8 to +8 cm; inside 12 x 14 cm.
- **Receptacle:** face centre at x 20.5 cm, y 0, 2.2 cm above the box bottom (2.3 cm below the rim). The outlets'
  seated plug centres are at x 18.6 cm (near) and x 22.4 cm (far).
- **Lid:** grey plate, 13.6 x 16.5 x 0.6 cm, 50 g, lying on the rim. Its far edge stops 4 mm short of the far wall's
  outer face, as a lid sits inside its cover frame, and its left edge rests 5 mm onto the floor beyond the left wall. Its far edge has the cord opening, 20 mm wide and 25 mm deep, centred at y −5.5 cm, so with the lid shut
  the opening lies over the far wall. Under it, the far wall's top has a cord notch of the same 20 mm width, 1 cm deep. It is hinged along its left edge (y +8.5 cm, the hinge line running along x) and
  opens to a stop at 100 degrees. Past upright, an opened lid stays open on its stop, standing beside the box.
- **Lid handle:** a grey bail bar, 46 x 10 x 10 mm, running along x, centred x 18.5 cm, y +1.5 cm (7 cm from the
  hinge), its top 18 mm above the lid. It turns freely on a pin between two short posts, parallel to the hinge.
- **Service box:** grey, 6 x 7 x 5 cm, fixed on the floor. The cord leaves it at x 70 cm, y −10 cm, 3.7 cm above the
  floor.
- **Power strip:** on the floor, outlet centre at x 12.5 cm, y −25.5 cm, turned so the plug's cord leaves towards the
  service box.
- **Cord:** 6 mm black, 58 cm long. It starts as a straight line from the service box to the plug in the strip, and
  settles onto the floor.

## Materials (real-world setup)

- **Floor:** a 45 mm thick plywood or MDF platform (two layers of 22 mm board) screwed to the table, with a
  cut-out that holds the floor box, so the box is flush with the floor as it is in a building.
- **Floor box:** white painted sheet steel or rigid plastic, 1 cm walls and a bottom, 12 x 14 cm inside, set in the
  cut-out.
- **Lid:** grey rigid plastic plate, 6 mm thick at its edges, pocketed underneath (a 2 mm skin with ribs) to about
  50 g, the simulated lid's mass. A solid 6 mm ABS plate of this size would weigh about 140 g, which makes it harder
  to swing but changes nothing else in the task. It has the 20 x 25 mm cord opening cut in its far edge (and a 20 mm
  wide, 1 cm deep notch cut in the far wall's top under it), and sits on a piano hinge along the left wall, with a stop
  block that holds it at 100 degrees.
- **Lid handle:** a metal bail bar on a pin between two short posts screwed to the lid.
- **Receptacle:** an off-the-shelf duplex NEMA 5-15R, such as the Leviton 5320, screwed by its strap face up to a
  bracket 22 mm above the box bottom. It is **not wired or powered**.
- **Power strip:** a one-outlet 5-15R adapter screwed to the floor. It is **not powered** either.
- **Cord:** a 6 mm black PVC-jacketed cord with a moulded right-angle NEMA 5-15P plug. Its other end is clamped
  inside a grey plastic junction box screwed to the floor (the service box).

The simulated contacts hold at the UL 498 minimum (13.3 N). A new real receptacle can grip harder, up to the 67 N
release limit, so on the real setup plugging in and unplugging may need a firmer push or pull than in the simulation
(see Fidelity).

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the simulated cable) next to `task.py`, plus `cabledemo.py` for the scripted demo.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Plug the black cord into the floor box outlet, close the lid without pinching the cord, then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

The demo's motions use only the fixed layout (the box, the receptacle, the hinge and the power strip never move), the
arms' measured joint positions, and one fixed grip offset (how the plug hangs in the fingers after the pick, measured
once offline and written into the script). It never reads object positions from the simulator to move.

1. The left arm pinches the lid's handle bar, swings the lid up past upright onto its stop, lets go and moves aside.
2. Meanwhile the right arm pinches the plug body at its middle, across its width, pulls it straight up out of the
   power strip's contacts and waits clear of the swinging lid.
3. Once the lid is open, the right arm turns the plug so its cord points at the far wall, with its wrist camera on
   the far-wall side looking down at the plug and the receptacle face. It holds the plug 2 cm over the near outlet,
   checks where its hand really is from its own joint positions (the cord's pull holds it a millimetre or two
   short) and corrects for that, then lowers the plug until the ground pin's tip presses on the receptacle face.
4. It searches by feel on a 0.3 mm grid out to 1.8 mm, nearest points first: at each point it lifts the pin tip 1.5 mm,
   steps over, sets it down and presses lightly (a hard press pivots the plug in the grip), watching its own measured
   height; a drop of 0.8 mm or more means the pin has entered its hole. If the blade tips then still stand on the face,
   it turns the plug by up to 3.2 degrees and shifts it by 0.3 mm while pressing, until they drop into their slots.
   Then it pushes the plug straight down through the contacts to the seat. The arm's joints are compliant, so for a
   firm push it is commanded 40 mm below the seat (the plug stops on the receptacle face). It lets go and presses the
   plug's top firmly, over its middle and then over its cord end.
5. With closed fingers the right arm pushes the cord sideways from its left towards the opening's place, first just
   inside the far wall below the rim, then along the wall's top, stopping with the cord at y −5.5 cm,
   where the lid's opening comes down. Then it presses the cord down into the wall's notch with its closed fingers. It never lifts the cord, so the seated plug is not pulled up.
6. It presses the plug's top firmly once more, over its middle and its cord end, then parks.
7. The left arm takes the handle again, swings the lid back down until it lies flat on the rim and lets go.

The recorded demo (`demo/run.log`) ends with score 1.0 at step 1,984 of the 2,400 allowed (99 s): the plug body
0.2 mm above the receptacle face, the cord in the opening, the lid shut flat and both grippers clear.

How the contacts were checked: a diagnostic log (the `FLOORBOX_CALIB=1` option, offline only) prints how far each
contact is pushed back. With the plug seated, the four contacts of the near outlet stand 0.11-0.15 mm back (riding on
the blades), and those of the other outlet stay at rest. A plug pushed with a light commanded force (`underpush`)
stops with its blade tips on the contacts, 9.3 mm short of seated, and scores 0. With the contacts the earlier demo's
gentle push left the plug 9-12 mm short, so the demo now pushes harder.

Test variants (`FLOORBOX_DEMO=...`), each a scripted policy scored by the same scorer:

| Variant | What it does | Score |
|---|---|---|
| `tier04` | plug seated, cord only nudged along the far wall, well clear of the opening's place, lid left open | 0.4 |
| `tier06` | plug seated, cord swept to the opening's place on the far wall, lid left open | 0.6 |
| `lazy` | plug pushed in, lid closed without moving the cord: the lid comes down on the cord and is held up about 4 degrees | 0 |
| `pinch` | plug seated, cord pushed along the far wall towards the opening's place but stopped 3 cm short; lid closed on it | 0 |
| `perch` | plug set down 4 mm off the near outlet, so the pin misses its hole and the plug stands on its tips; cord led to the opening; lid left open | 0 |
| `underpush` | pin found and blades in their slots, but the plug is only pushed lightly (commanded 3 mm below the seat) and never pressed home, so it stops on the contacts 9 mm short of seated; cord led to the opening; lid closed | 0 |

## Scoring (`score()` at the end of the trial; `success()` = score 1.0 with the lid at rest and the cord still)

**Score 0** if any of these holds at the end:

- the simulation state is unreadable (NaN).
- a gripper is too close to the parts, or a gripper's state can't be read. Too close means within 3.5 cm of the
  cord, a blade, the ground pin, the lid (including the strips beside its opening), the handle bar or its posts, or within 3.5 cm plus half the body's length
  (5.3 cm) of the plug body's centre.
- the plug isn't fully seated in an outlet of the floor box receptacle. Fully seated means all of these hold for the
  near or the far outlet:
  - each blade's centre is inside one of that outlet's blade slots, and the pin's centre is inside the ground hole.
    This is checked horizontally, within the opening's clearance around the part plus 0.5 mm;
  - the blades and the pin are within 8 degrees of vertical;
  - the plug body's face (the blades' root) is at most 1.5 mm above the receptacle face, and no more than 2 mm below
    it. So at least 14.4 mm of each 15.9 mm blade is inside the receptacle.
- the lid is neither shut flat with the cord in the opening nor left open. Shut flat means within 1.3 degrees of
  closed (its free edge within 3.7 mm of the rim). Left open means 60 degrees or more. Anything in between is a lid
  resting on the cord, or ajar.

"Cord in the opening" means some part of the cord's centre line crosses the far wall where the shut lid's opening
lies: between the opening's sides (1 mm margin), within the far wall's thickness (3 mm margin each side), and no
higher than the shut lid's top (6 mm above the rim).

Otherwise the end state falls into exactly one tier:

| Score | End state |
|---|---|
| 0.4 | Plug fully seated, lid left open (60 degrees or more), cord not in the opening. |
| 0.6 | Plug fully seated, lid left open, cord in the opening. |
| 1.0 | Plug fully seated, cord in the opening, lid shut flat (1.3 degrees or less). |

A box left open is unfinished but harmless, so it earns partial credit. A lid closed on the cord crushes it, so it
scores 0. With the lid shut flat, the cord can only leave through the opening, so there is no "shut but not routed"
tier.

A plug sitting on the face without its blades in the slots, or only part of the way in, is not seated. A plug left
in the power strip is not in the floor box at all.

Real-world check:

- the plug body sits flat on the receptacle face: a 1.5 mm feeler gauge must not slide under the plug body anywhere;
- a 4 mm feeler under the lid's free edge must not fit (the sim's 1.3 degree limit puts that edge within 3.7 mm of the
  rim, so a 4 mm gauge is the nearest standard size);
- check by eye that the cord leaves through the lid's opening, that an open lid is up past 60 degrees, and that the
  grippers are open and away.

## How this differs from similar tasks

- **Plug-seating tasks** (pushing a plug or connector into a socket or port) score the seat alone. Here the seat is
  only the first stage: the seated plug's own cord, now fixed at both ends, must be moved sideways to where a
  narrow opening in the lid comes down, without pulling the plug back out, and the lid closed over it must not pinch it.
- **Cord-into-channel tasks** lay a free cord into a channel or tray. Here the cord is anchored at the service box and
  at the plug the same arm has just seated, and its middle has to lie in one 20 mm wide place on a wall top.
- **Grommet and pass-through tasks** feed a plug end through an opening and pull the cable to length. Here no end is
  fed through anything; the middle of the cord moves sideways before a lid with an opening comes down over it.
- **Tasks about not trapping a cord** under or behind an appliance involve no insertion. Here the end state also
  needs a NEMA 5-15 plug fully seated.

## What changed in this revision

Reviewer feedback: "Construct validity (not met): the simulated receptacle has no blade-contact resistance, so
seating the plug omits the force needed to insert it into a real outlet." and "What to change: the shorter
instruction and more realistic plug geometry address much of the earlier feedback. The insertion-fidelity point
remains open: the simulated outlets have no blade-contact grip, so the robot need not exert the force required by a
real receptacle. Add representative retention resistance for unplugging and seating, document how it compares with
the real hardware, and provide a full-score demo using it."

- **Blade contacts:** both outlets of the floor box receptacle and the power strip's outlet now have a spring contact
  on each side of each blade, which grip the blades with 13.3 N per outlet, the UL 498 minimum retention (see "Plug
  and receptacle" for the sources and the comparison with real receptacles). Unplugging from the strip takes a
  13.3 N pull, and seating takes a push of at least 13.3 N.
- **Scoring:** unchanged in form: the plug body must still sit within 1.5 mm of the receptacle face, which it only
  reaches when pushed past the contacts' grip. A lightly pushed plug stops on the contacts about 9 mm short and
  scores 0 (the new `underpush` variant).
- **Demo:** a new full-score demo, which aligns the blades by turning the plug a little while pressing, pushes the
  plug home through the contacts with a firm commanded push, and presses its top firmly after letting go.

## Earlier revision

Reviewer feedback: "Prompt should be simplified, and connector insertion should be more faithful to real-world
context"

- **Prompt:** the instruction is now one plain sentence (19 words) with the goal and the end state:
  `Plug the black cord into the floor box outlet, close the lid without pinching the cord, then let go.` The earlier version
  listed the steps (lift the cover off, plug in, lead the cord out through the notch, put the cover back flat, let
  go). The scorer still checks every part of the end state.
- **Real-world context:** the floor box is now set flush in a raised floor, as in a building, with the cord, the
  service box and the power strip on the floor. The box is 12 x 14 cm inside, so a real duplex receptacle with its
  107 mm strap fits, and the cord leaves through an opening in the lid's edge, as in commercial floor box covers,
  instead of only a notch in the wall (the wall top now also has a matching notch under the opening). The lid handle is lower (its top 18 mm above the lid).
- **Connector insertion:** the earlier version had a pinless rectangular plug dropped into a 36 x 26 mm opening and
  counted as seated 15 mm down. Now:
  - the plug has NEMA 5-15 blades and a ground pin, and the floor box holds a face-up duplex 5-15R receptacle with a
    blade slot and a ground hole for each prong, 0.3-0.4 mm larger a side than the prong;
  - the plug goes in by sliding its blades and pin down through those openings along the receptacle's axis, with no
    snap or teleport;
  - the scorer needs every prong in its own opening, all within 8 degrees of vertical, and the plug body within
    1.5 mm of the receptacle face (14.4 mm or more of each blade inside);
  - the plug starts plugged into a power strip, so the robot unplugs it before plugging it into the floor box, and the
    lid is now hinged, like a floor box cover.
- **Demo:** the demo finds the ground hole by feel (a search over the face, watching its own measured height) instead
  of reading the plug's position from the simulator. Its wrist camera looks down at the plug and the receptacle while
  it inserts.

## Fidelity

- **Cord:** a chain of short rigid segments with bending springs (no twist, stretch or plasticity), fixed at the
  service box.
- **Plug:** a rigid body with rigid blades and pin of NEMA 5-15 size. Both blades are 6.3 mm wide (a real neutral
  blade is 7.9 mm).
- **Receptacle:** real openings in the face, sized 0.3-0.4 mm a side larger than the prongs, and a spring contact on
  each side of each blade. The contacts are set at the UL 498 minimum grip (13.3 N per outlet); a new receptacle can
  grip up to the 67 N release limit, so a real plug may need a firmer push and pull. Each contact touches the blade at
  one point with a fixed friction of 0.5, where a real contact wipes along a strip of the blade; the ground pin has
  no contact. The openings are square-edged with no lead-in chamfer, the ground hole is square (the simulation builds
  the face from boxes), and the strap is modelled only by its two ears.
- **Lid and handle:** a rigid lid on a hinge with a stop (built as a plate and the two strips beside its opening,
  joined rigidly), and a handle bar on its own pin.
- **Floor:** rigid blocks at the rim's height; the box, the floor, the receptacle and the strip are fixed.

## Runtime and seeds

Run limit 2,400 steps (120 s at 20 Hz). A trial takes a few minutes wall-clock on one GPU (the first start of Isaac
Sim adds 1–2 min). The layout is fixed (`object_jitter=0`; nothing is jittered), so every seed starts from the same
state.
