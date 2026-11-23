# Seat a 6.6 kg Nexus 9504 line card: slide it in, then close both ejector levers together

Instruction: `Install the line card in the switch's empty slot, then let go.`

AI training clusters are wired through modular data-center switches (spine and leaf layers), and their line cards are
installed and replaced by hand during build-out and break-fix. Cisco's *Nexus 9504 NX-OS Mode Switch Hardware
Installation Guide* ("Installing a line card") gives the method: "Gently slide the module into the slot until it
stops. The line card stops when its front is about 0.25 inches (0.6 cm) outside the front of the chassis. Rotate the
ends of the two levers toward the front of the chassis until the levers click and they secure the line card to the
chassis. As you rotate the levers, the line card seats itself onto the fabric modules inside the chassis."

Why it needs both arms: the two ejector levers sit at the two ends of a 43.18 cm wide card. Each arm can reach only
its own lever (an arm cannot reach across in front of the other). The 6.62 kg card rides in side guides with 0.5 mm
clearance per side, so one end can get at most about 1 mm ahead of the other. Levering one end alone therefore stalls
about 5 mm short of seated, because the other, still-open lever's knob holds its end at the stop. The card has to be
pushed in square to the stop, then both levers rotated home together until both click.

## How it differs from nearby module-install tasks

- **Pressing a card down into an edge connector until a latch clicks** (e.g. a GPU into an x16 slot): there the card is
  light (about 1 kg), moves a few mm vertically, and pressing seats it. Here a 6.62 kg card makes a 12 cm horizontal
  slide in guides with 0.5 mm clearance, where pushing off-centre cocks and jams it (one arm alone jams 2 cm short,
  score 0.0), and pushing can never seat it: the last 6 mm against 50 N per end can only be driven by the levers.
- **Lowering a module onto guide pins and camming two levers down** (e.g. a server fan cage): there the part is lowered
  a short way onto pins, then cammed down. Here the levers impose an order: they must stay open through the whole slide
  and only catch at the stop (folding them early makes seating impossible, 0.3 at most), and they are 43 cm apart on
  the face toward the robot, so each arm must leave its push point and take its own lever while the other end holds.

## Requirements & Assets

- `yam-isaac-lab-arena` and `Isaac Lab-Arena 0.3.1` with `inspect-robots`.
- Third-party assets: none. All geometry is generated from USD primitives in `task.py`.

## Run a Trial

```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 600 \
  --instruction "Install the line card in the switch's empty slot, then let go."
```

Run limit: `600` steps (`30 s` at `20 Hz`). The layout is fixed with `object_jitter=0`.

## Scene (cm, origin on the table midway between the arm bases at y = +-30.5; x forward, y left, z up)

| Object | Real part | Size / material | Pose |
|---|---|---|---|
| Switch chassis | Cisco Nexus 9504 (N9K-C9504), unpowered, no cables | 44.5 W x 84.20 D (with handles) x 31.1 H cm (7 RU), steel; modelled body 78 cm deep | Front face at x 36.0, centred on y 0, standing on the table, fixed |
| Line-card slot 4 | Empty slot of the chassis | Inner width 43.28 cm (0.5 mm clearance per side), floor z 10.5, ceiling z 15.05; fabric wall 0.3 cm behind the seated card | Below slots 1-3 (blank covers) and above the supervisor section (dark panels) |
| Line card | Cisco N9K-X9736C-FX 36-port 100G QSFP28 | 43.18 W x 41.91 D (inside the chassis) x 4.4 H cm, 6.62 kg; steel faceplate block with 36 QSFP28 cages 4.35 cm deep | Resting in slot 4's guides, back 30 cm in, faceplate back face 12.0 cm proud of the chassis front |
| Ejector levers (2) | The card's two ejector levers | 10.0 cm long (pivot to tip) x 2.0 x 3.0 cm, light grey; pivots 1.2 cm in from each card end, in front of the faceplate (faceplate + lever = 6.35 cm proud when seated, as Cisco lists) | Both rotated 45 deg open, tips toward the robot |

### Trial fixture (about $250 in parts, no switch needed)

A used Nexus 9504 with an N9K-X9736C-FX card can be used directly, but the trial does not need one: the task depends only
on the slot, the card's size and mass, and the two-lever seating mechanism, which this fixture reproduces at the
published dimensions. Build files in this folder:

- `fixture_drawings.svg`: one A3 sheet, 1:1 when printed at 100 %. It has the dimensioned catch plate with every
  cut-out vertex, the hub arm, pivot ear and lever, an elevation of the parts stacked at the pivot, the plunger setting,
  the check values, and a 45 deg reset gauge.
- `fixture.scad`: parametric OpenSCAD source for every custom part and the assembly, with the same numbers. To export a
  part, for example the catch plate as DXF for laser or waterjet cutting:
  `openscad -o catch_plate.dxf -D 'part="catch_plate_2d"' fixture.scad`.

All dimensions are in mm. x = depth behind the steel strip's front face (the chassis-front reference plane; negative =
in front of it, toward the robot). z = height above the bench, with the slot floor at z 105. For per-end parts, y = the
distance in from that card end. The left end is drawn; every right-end part is its mirror image.

**How the seating mechanism works.** Each lever is clamped on a vertical 6 mm pivot pin. Below the faceplate block the
same pin carries a hub arm with a 6 mm dowel at 26.1 mm radius. The dowel runs in a notch in a steel catch plate fixed
under the slot.

- **Stop.** With the lever 45 deg open, the dowel enters the notch and its stop wall halts the card 6.0 mm proud. The
  crank is then 5 deg past dead centre (dead centre is 40 deg), so pushing the card harder turns the lever further open,
  never closed.
- **Seating.** Closing the lever swings the dowel outboard behind the notch's lip. The lip draws the card in to 0.4 mm
  proud against the two 50 N plungers.
- **Pre-closed lever.** A lever closed beforehand puts its dowel in front of the lip, and the card stops 9.5 mm proud.
- **Latch.** A pot magnet in the faceplate holds the closed lever and gives the click.

| Part | Build |
|---|---|
| Chassis frame | 2020 aluminium extrusion box, 485 W x 600 D x 311 H outside (445 clear between the uprights, the real chassis width), clamped to the bench, with cross members front and rear at the slot floor (z 105) and ceiling (z 150.5) heights, and a plunger cross member whose front face is at x 440, centred on z 119. A 2 mm sheet panel above and below the slot closes the front. |
| Slot guides and datum | Two 25 x 25 x 3 aluminium angles, 600 long, bolted to the cross members with their top faces at z 105 and their inner faces 432.8 apart (y = +-216.4). This gives 0.5 clearance per side for the 431.8 card and a 45.5 tall channel. A 3 x 50 steel strip (z 55-105) and a 3 x 25 strip (z 150.5-175.5), both 485 long, are bolted across the front uprights. Their front face is **datum A** (x = 0) for everything else. |
| Line card | 1.5 mm steel tray 431.8 x 419.1, with a 25 x 25 x 3 steel angle along its back edge (the plunger push bar; its rear face is the card's rearmost face). On its front is a 431.8 x 44 x 43.5 faceplate block (6061 aluminium, or 3D-printed with steel ballast). Steel bar ballast brings the total to 6.62 kg. Proud is measured from datum A to the block's back face. |
| Ejector levers (drawing D) | 6061 bar 20 x 30 x 110, full R10 pivot end, 6 H7 bore 10 from that end, 100 from pivot to tip, grip grooves at 88-98. Each lever sits at z 112-142 between two 3 mm steel pivot ears (drawing C). The ears are screwed with 2 x M4 to the block's top face (z 149-152) and underside (z 102-105), with the bore 12 in from the card end and 10 in front of the block face, and they hold 6 x 10 x 6 flanged sintered-bronze bushings and 5.5 spacers. The pin is a 6 h6 ground shaft, 66 long (z 91-157), with an E-clip on top. The lever is fixed to it with an M4 set screw on a flat plus a 2 mm spring pin. |
| Hub arm and dowel (drawing B) | 8 mm steel, 12 wide, 26.1 between bore centres, on the pin at z 91-99 and fixed like the lever. With the lever closed, the arm points outboard, 50 deg toward the chassis (130 deg from the lever axis). An ISO 8734 6 m6 x 20 dowel is pressed in flush at the top and reaches 12 below, through the catch plate's notch. With the lever 45 deg open, the dowel centre is 26.0 behind and 2.3 inboard of the pivot. |
| Catch plates (drawing A) | 5 mm mild steel, laser or waterjet cut, z 84-89 (top face 16 below the slot floor). The blank is x -40 to 0, y -20 to +35. Cut-out vertices (x, y), V1 to V6: (-40, 9.8), (-40, 20.5), (-30.5, 20.5), (-30.5, -8.75), (-36.9, -8.75), (-36.9, 9.8). The cut-out is a notch from the front edge (y 9.8 to 20.5) to the stop wall at x -30.5, plus a pocket behind the 3.1 mm lip (x -36.9 to -30.5, down to y -8.75). The stop wall and the lip's cam corner V6 are the working edges (+-0.05). Each plate is bolted with 2 x M5 flat-head screws onto a 25 x 25 x 3 angle that is bolted to the 50 mm strip. |
| Latch | A 16 mm neodymium pot magnet (M3 countersunk, about 60 N pull) sits flush in the block face, 85 inboard of each pivot at mid-height. A 16 x 2 steel disc sits flush in the lever's back face. The magnet holds a closed lever against the plunger's 0.96 N m back-drive (11 N at 85 mm, a 5x margin) and clicks shut. A 20-25 N outward pull at the knob opens the lever. |
| Connector resistance | Two M16 x 1.5 spring plungers with steel thrust pins: stroke at least 8, 50 N end force preloaded, at most 62 N at 8 (rate at most 1.5 N/mm). One sits behind each card end, 40 in from the card end at z 119, threaded through a 50 x 50 x 10 steel tab on the plunger cross member and locked with a nut. Each nose is set to x 413.1, so it meets the push bar when the card is 6.0 proud and is pressed 5.6 when the card is seated. The sim uses a flat 50 N. |

**Bill of materials** (generic catalogue parts; about $250 in total):

| Qty | Item |
|---|---|
| 6 m + 12 | 2020 aluminium extrusion, corner brackets, M5 T-nuts and screws |
| 1 + 1 | Steel flat bar 3 x 50 x 485 and 3 x 25 x 485 (front strips) |
| 2 + 3 | Aluminium angle 25 x 25 x 3 x 600 (guides); steel angle 25 x 25 x 3 (one 431.8 long push bar, two 55 long plate brackets) |
| 1 | Steel sheet 1.5 mm, 431.8 x 419.1 (tray), plus steel flat bar as ballast to 6.62 kg |
| 1 | 6061 bar 431.8 x 44 x 43.5, or a 3D-printed block (faceplate block) |
| 2 | 6061 flat bar 20 x 30 x 110 (levers) |
| 2 | Ground shaft 6 h6 x 66 (cut from 6 x 100 linear shaft) and DIN 6799 E-clips |
| 4 + 4 | Sintered bronze flanged bushings 6 x 10 x 6 (flange 12 x 1.5); 6.4 x 12 x 5.5 spacers |
| 4 | Pivot ears, 3 mm steel, laser cut (drawing C) |
| 2 | Hub arms, 8 mm steel flat 12 wide (drawing B) |
| 2 | Dowel pins ISO 8734 6 m6 x 20 |
| 4 + 4 | Spring pins ISO 8752 2 x 12; M4 set screws |
| 2 | Catch plates, 5 mm mild steel, laser or waterjet cut (drawing A); 4 x M5 flat-head screws |
| 2 + 2 | Neodymium pot magnets 16 mm (M3 countersunk, about 60 N); steel discs 16 x 2 |
| 2 + 2 | M16 x 1.5 spring plungers with thrust pin (see above) and lock nuts; 50 x 50 x 10 steel tabs |
| 1 set | Gauges: 6.00 mm gauge block (or slip-gauge stack), feeler gauges 0.5 and 1.0, calipers, 0-100 N push scale, the 45 deg gauge from the drawing sheet |

**Assembly and alignment** (datum A = the strip's front face; tolerances +-0.1 unless noted):

1. **Frame and guides.** Set the guide top faces at z 105, level to 0.2 over 600. Space the guides with the card itself:
   a 0.5 feeler must slide between the tray and each guide.
2. **Levers.** With each lever closed flat on the faceplate, set its hub arm 50 deg toward the chassis using the 1:1
   drawing B as a template. Tighten the set screws, then drill and fit the 2 mm spring pins. Check: with the lever 45
   deg open, the dowel centre is 16.0 behind the block's front face.
3. **Catch plates.** Bolt each plate's angle to the 50 mm strip with the plate top face at z 89. Press the plate's rear
   edge against datum A. Slide the plate in y until vertex V6 is 9.8 in from the card end line, which is 10.3 from the
   guide's inner face (set it with calipers or a 10.3 gauge block), then tighten.
4. **Plungers.** Put both levers at 45 deg and hold the card 6.00 proud at both ends, with a 6.00 gauge block between
   the block's back face and datum A at each end. Screw each plunger in until its nose just touches the push bar (a 0.05
   feeler drags), then lock it. Check that the push scale reads 50 N (+-5) at nose break-away.
5. **Verify** before any trial. All of these must hold at both card ends:
   - Levers open, push the card in: it stops at 6.0 +-0.3 proud (the 6.0 block fits, a 6.5 block does not).
   - Push hard: the levers turn further open, and the card stays at least 5.4 proud.
   - Close both levers together: both magnets click, and a 1.0 feeler does not enter between the block and datum A
     (0.4 nominal).
   - Close only one lever: it stalls, with its end at least 4 mm proud and no click.
   - Close both levers first, then push the card: it stops at 9.5 +-0.5 proud.

**Reset** (about 30 s, no tools besides the gauge):

1. Pull each lever knob outward to release its magnet. The plungers push the card back to about 6 mm proud.
2. Set both levers to 45 deg with the gauge.
3. Slide the card out until the block's back face lines up with a mark on each guide 120 from datum A.
4. Re-check both lever angles.

The fixture differs from the sim in six ways. Only the last changes a listed score: folding the levers before the push scores 0.3 in the sim and 0.0 on the fixture (stricter on the fixture). Every other listed policy scores the same on both.

- The fixture's stop is a sine profile, and the sim's is linear.
- The lever needs up to 10 N at the knob in the last degrees, against about 4 N in the sim.
- A hard push turns the levers open to about 52 deg, with the card 5.4-5.7 proud. In the sim the lever and the card
  stay at 45 deg and 6.0.
- A lever must be at least about 42 deg open (41-43 deg with the guide play) to catch, against 40 deg in the sim.
- A seated card ends 0.4 proud, against 0 in the sim.
- With the levers closed beforehand, the card stops 9.5 proud, against 6.0 in the sim.

## Scoring

`score()` is cumulative and judged at the end of the run (`SCORE_OVER = "end"`). **Proud** is the gap, measured at
each card end, from the chassis front face to the back face of the card's faceplate (the faceplate's rear plane, which
meets the chassis front when the card is seated; the 4.35 cm port block and the levers sit in front of it). In the sim
it is read at the two front corners of the card body, which is exactly that plane.

- **Score 0** if at any time the card is pulled back out (more than 30.0 cm proud at either end, leaving under 12 cm of the
  41.91 cm card in the guides; small back-and-forth adjustments are fine), or the chassis moves
  more than 1 cm (real trial; it is fixed in the sim).
- **0.3**: the card is pushed to the levers' stop: no more than 0.8 cm proud at both ends.
- **0.7**: seated: no more than 1 mm proud at both ends, and both levers closed and clicked (within 2 deg of closed).
- **1.0**: 0.7, plus both grippers open, both TCPs at least 10 cm from the card and the levers, and both arms still
  (each TCP moves at most 1.0 cm over the last 2.0 s).

End-state check, in plain words: both levers closed flat against the faceplate and clicked, the card seated (no gap at
the front panel at either end), and both grippers open, clear of the card and levers, and still for 2 s. Measured as: a
1 mm feeler gauge must not enter between the faceplate's back face and the chassis front at either
card end (seated), a photo of both levers closed against the faceplate, and the last 2 s of video for open, still
grippers 10 cm away.

## Simulation Fidelity

- Rigid bodies at Cisco's published dimensions and the card's published 6.62 kg mass. The card slides in physical
  side guides (0.5 mm clearance per side, steel friction 0.20 static / 0.15 dynamic).
- Each ejector lever is a separate hinged body (revolute joint, 0-50 deg). Its knob bearing on the chassis edge is a
  stiff contact model: once the card reaches the stop at that end (0.6 cm proud), the end cannot go deeper than the
  lever angle allows (6 mm over 45 deg, linear in angle, i.e. a 7.64 mm effective knob radius; the fixture's 26.1 mm crank and
  catch plate give the same 6 mm stop and seat with a sine profile), and closing the lever draws that end in, with the reaction
  torque on the lever. The knob catches only if its lever is at least 40 deg open when that end reaches the stop; a
  lever closed beforehand is a hard stop at 0.6 cm proud and cannot pull the card in. A knob transmits at most 150 N. Pushing the card does not turn an open lever.
- Fabric connector resistance: a 50 N push-back on each card end over the last 6 mm, independent of speed (it builds
  up over the first 0.5 mm past the stop), so the closing levers must hold 50 N per end; the value is assumed (Cisco
  publishes no mating force) and matches the fixture's spring plungers.
- Click latch: a lever within 2 deg of closed with its end seated (within 1.5 mm) clicks and then holds closed (40 N m/rad holding stiffness, so the 50 N push-back opens a latched lever by
  less than 1 deg).
- Omitted: the levers' release buttons (only needed to open them), interface cables, transceivers, and the chassis
  rear, power supplies and fans (unpowered chassis).
- No grasp welds: the arms push the faceplate and the lever faces with their closed fingers.

## Demo Video & Shortcut Checks

A scripted demo reads card and lever poses from simulator state. Both arms close their jaws, push the faceplate in
square at 8 cm either side of centre until the card stops 0.6 cm proud, then each arm moves to its own lever's tip and
both levers are rotated home together until both click. The jaws open, the arms back off and park, and hold still.

| Policy | Score |
|---|---|
| Scripted demo (full score at step 286 of 600) | 1.0 |
| Idle arms | 0.0 |
| Push the faceplate hard, levers untouched | 0.3 |
| One arm only: pushes from its side (the card cocks in its guides and jams 2 cm short), closes its own lever | 0.0 |
| Both arms push, only one lever closed | 0.3 |
| Fold both levers shut before pushing, then push and press them (they cannot catch; the sim stops the card 6 mm proud, the fixture's catch plate 9.5 mm, so 0.3 in the sim and 0.0 on the fixture) | 0.3 |
| Seat fully but keep the fingers on the levers | 0.7 |

## Sources

- Cisco, *Cisco Nexus 9504 NX-OS Mode Switch Hardware Installation Guide*: "Replacing or Installing Modules"
  (line card installation steps quoted above) and "System Specifications" (chassis 17.5 in / 44.5 cm wide, 33.15 in /
  84.20 cm deep with handles, 12.25 in / 31.1 cm, 7 RU; line cards 17.0 in / 43.18 cm wide, 16.5 in / 41.91 cm inside the
  chassis, ejector levers 2.5 in / 6.35 cm outside; N9K-X9736C-FX 14.6 lb / 6.62 kg).
