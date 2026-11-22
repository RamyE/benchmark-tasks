# Twist-fold a pop-up car window sunshade into its three-loop coil and slip it into its pouch

Instruction: `Fold the sunshade and put it in its pouch.`

Pop-up car window shades open by themselves from a spring-steel wire hoop sewn into their hem. So do pop-up laundry
hampers, photo reflectors and play tents. Folding one back up is a well-known knack that most people have fumbled. You
hold the hoop at two opposite points, twist your hands in opposite directions about the line between them, and bring
them together. The hoop buckles into three stacked loops about a third of its size. Let go too early—or let go with one
hand in mid-air while holding only a single loop—and it springs open again, so you press and re-pinch across the
stacked loops before sliding the coil into its flat pouch.

What makes it hard:
- **Two hands, one coordinated motion.** Each hand must grasp opposite sides of the hoop and turn about the line between
  the hands, in opposite senses, while closing inward against the wire's bending and torsional resistance. One hand
  alone, grasping adjacent points on the same arc, twisting only one wrist, or pushing together without twisting never
  buckles it.
- **Physical loop retention and hand-over.** Holding only one loop in mid-air lets the other loops spring open. To pass
  from the two-hand twist grip to a one-hand carry, one hand must press the coiled bundle flat onto the table while the
  other opens and re-pinches across the stacked loops together.
- **Insertion into a low sleeve.** The 14.5 cm three-loop bundle goes horizontally through the 20 cm mouth under the
  pouch's top layer (4 cm inside height), flat and low, and the hand lets go only once the coil is inside.

Compared with existing benchmarks: cloth and garment folding (SoftGym, GarmentLab, BEHAVIOR folding tasks, RoboDojo
fold_clothes, UMI, DaXBench) folds flat fabric along creases. Rope tasks coil a cord around a fixture. None folds a
closed elastic hoop by a two-hand twist into a buckled three-loop coil, transfers grip across the stacked loops so they
cannot spring open, and stows the bundle inside a flat sleeve.

## Scene

Positions in cm from the midpoint between the arm bases (x forward, y left), heights above the table.

| object | type | size | material (real part) | position |
|---|---|---|---|---|
| sunshade | free, 90 g | 44 cm across (wire centreline radius 22 cm), 1.15 mm wire in a 1 cm hem | round twist-fold pop-up side-window sunshade (e.g. Kinder Fluff / Enovoe 44 cm shade): 1.15 mm (19 AWG) ASTM A228 spring-steel music-wire hoop (11 g) sewn in a 1 cm polyester hem with 0.15 mm black mesh panel | open and flat, centre (38, 0) |
| pouch | free, 40 g | 22 x 20 cm outside, 4 cm inside height, 6 mm side/back walls, mouth along the near 20 cm edge, 7 x 7 cm thumb notch in the top layer at the middle of the mouth | flat open-mouth storage sleeve of 3 mm fluted polypropylene twinwall sheet (350 g/m^2 Coroplast) wrapped with nylon ripstop cloth tape (stands 4 cm open without a second hand holding the lip) | centre (45, 38), mouth facing the robot (-x) |

Sleeve build (reproducible bench fixture, 40 g): cut two `22 x 20 cm` base/top panels (cutting a centred `7 x 7 cm`
thumb notch in the top panel's mouth edge), one `20 x 4 cm` back strip, and two `21.4 x 4 cm` side strips (`6 mm`
doubled walls) from `3 mm` fluted polypropylene twinwall sheet (`350 g/m^2` Coroplast, `~1,080 cm^2` = `38 g`), and
join the edges with `2 g` of nylon ripstop cloth tape so the `20 x 4 cm` mouth stands open on its own (`40 g` total).
The bench top carries reference marks at `(38, 0)` and `(45, 38)`, a `20 cm` diameter reference circle centred at
`(34, 0)` where the buckled coil is pressed flat, and a calibrated `5 cm` table grid so coil diameter is measurable
anywhere in view. The hoop's grasp points at its sides (y = ±22) and the pouch mouth (x 34, y 38) are within both arms'
reach.

## Scoring (real trial)

From the trial video, an overhead end photo, and a mouth-level end photo looking into the 20 x 4 cm opening (together
with the 7 x 7 cm top thumb notch, confirming all three stacked wire loops—and therefore the mesh panel sewn inside the
closed hoop hem—sit wholly inside the sleeve under the blue top panel with no wire protruding past the mouth,
distinguishing a stowed coil from one resting on top of or outside the pouch; a transient fold in the air or on the
table is verified against the `20 cm` table circle, calibrated table grid, and `20 cm`-wide pouch mouth).

- **0**: either the shade or the pouch falls off the table at any time (`z < table - 3 cm`), or the shade is never
  folded (including doing nothing).
- **0.25**: the shade was folded at some point: every wire segment centre within 10 cm of the hoop's centre (inside a
  20 cm circle).
- **0.5**: at the end the pouch rests upright on the table and the shade is folded and wholly inside it: every point of
  the wire (including its 5 mm hem radius) inside the pouch's inner outline and below its top panel.
- **1.0**: as 0.5, and both grippers are released and parked at least 10 cm clear of the shade and the pouch, still
  (under 2 cm/s) for at least 0.5 s.

A coil that has sprung open (inside the pouch, half out of it, or on the table) is not folded.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Fold the sunshade and put it in its pouch."
```

## Demo video

`demo/` holds a scripted run (score 1.0, about 38 s at 20 Hz, well inside the 1,200-step / 60 s run limit). Both arms
open above the hoop's opposite sides, lower onto the 1 cm hem, and close until both opposing finger pads make PhysX
contact. They lift the hoop 14 cm, point their grippers inward (jaws vertical), and close from 24 cm to under 16 cm
apart while rolling in opposite senses about the hand-to-hand line (left 86 deg, right 30 deg) against the hoop's
calibrated bending and torsional springs (`~5 N` inward force, `~0.18 N m` wrist torque). Once opposite twists pass
20 deg each and 90 deg relative with the hands `<= 16 cm` apart, both hands lift to 21 cm above the table while the
ring's PhysX joint rest curvatures relax smoothly over 0.85 s into the topological three-loop coil in clear air,
keeping their original grips throughout. Both hands then turn downward and press the coil flat onto the table; while the
right hand holds it pressed against the table, the left hand opens, moves to the far rim, and pinches with two-pad
contact across all three stacked loops. The right hand then releases and parks clear. The left hand lifts the clamped
three-loop bundle clear of the pouch, turns it 90 deg so the coil leads, levels it horizontally at the pouch mouth,
slides it through the mouth under the top panel (fingers in the thumb notch), opens to release, and parks clear with
its wrist camera aimed into the pouch mouth while the overhead camera has an unobstructed view of the pouch. The
scripted run reads simulator state (hoop points, pouch pose) to plan its moves; a policy has only the cameras.

## Shortcut checks

Scripted variants of the same run, each scored by `task.py`:

| variant | what it does | score |
|---|---|---|
| nothing | does nothing | 0.0 (never folded) |
| hover pinch | closes jaws 2 cm above the wire without pad contact and runs the twist motion | 0.0 (no two-pad contact; never grasped) |
| one arm | left hand alone pinches the hoop, lifts and twists 80 deg, carries toward the pouch and releases | 0.0 (never folded) |
| no twist | both hands pinch opposite sides, lift and close to 15 cm apart without rolling | 0.0 (never folded) |
| one-hand twist | both hands pinch opposite sides and close to 15 cm while only the left hand rolls 96 deg | 0.0 (never folded) |
| same-side grip | both hands pinch segments 45 deg apart on the same arc, lift and twist in opposite senses | 0.0 (never folded) |
| single-loop lift | buckles the hoop in mid-air, then right hand releases before pressing onto the table so left holds only 1 loop | 0.25 (springs open) |
| drop outside | buckles and re-pinches all 3 loops, then releases the bundle in the air beside the pouch | 0.25 (springs open) |
| half insert | folds and pushes the coil only halfway into the pouch mouth so loops protrude, then releases | 0.25 (springs open) |
| hold | folds and slides the bundle into the pouch, but keeps holding it | 0.5 |
| full | the demo above | 1.0 |

## Fidelity

Rigid-body articulation in PhysX, 240 Hz physics, 20 Hz control.

- **Hoop (calibrated to 1.15 mm ASTM A228 spring-steel music wire).** A closed PhysX ring of 48 rigid wire-hem capsule
  segments (`1 cm` hem diameter, `2.88 cm` chord, `90 g` total, friction `0.9`) joined by 3-DOF spring-damper D6 joints
  matching `1.15 mm` (`19 AWG`) spring-steel wire (`E = 200 GPa`, `G = 79 GPa`, `EI = 0.0172 N m^2`,
  `GJ = 0.0136 N m^2` -> `EI / L = 0.60 N m/rad` bend, `GJ / L = 0.47 N m/rad` twist, `0.004 N m s/rad` damping; coil
  bending strain `0.53% < 1.1%` elastic limit) with a spherical loop-closure joint at the far `+x` edge. Segments do
  not self-collide.
- **Force-limited two-pad-contact grip (sim stand-in for elastomer pad friction; no rigid weld).** Parallel rigid jaws
  let the round 1 cm hem roll under twist, so a compliant 6-axis spring-damper pad grip (`20,000 N/m`, `50 N m/rad`)
  stands in for pad friction and **engages only on two-pad contact**: the jaw must first open (`>= 2.6 cm`) and then
  close (`commanded <= 1.4 cm`, `0.5 < measured <= 3.6 cm`, squeezing) while **both** opposing finger pads (`tip_left`
  and `tip_right`) simultaneously register `>= 0.02 N` PhysX normal contact force with wire segment(s) lying between
  the pads (anchoring at the current relative pose with zero snap, on one segment on the open ring or up to three
  segments from distinct loops `>= 10` segments apart across the coiled rim). Every axis is force-capped to YAM
  rubber-pad friction (`50 N` pull from `2 x mu 0.5 x 50 N` `linear_4310` squeeze effort, `0.50 N m` twist on `10 mm`
  pad-patch radius, `1.50 N m` tipping on `15 mm` half pad length; single-hem `rotX` allows damped axial wire roll
  inside the fabric hem). Opening past `2.2 cm` disables the joint immediately; proximity or one-pad contact never
  attaches.
- **Post-buckling $Lk=0$ relaxation (after the hands perform the pre-buckling work).** On a real 44 cm shade, squeezing
  opposite sides from `44 cm` down to `<= 16 cm` (`~14.5 cm` coil width) while twisting opposite wrists past `90 deg`
  relative (`>= 20 deg` each) against the `0.60 / 0.47 N m/rad` wire springs supplies the full pre-buckling peak load
  (`~5 N` inward force, `~0.18 N m` wrist torque—matching human hand effort and well inside YAM's `50 N / 1.50 N m`
  caps) up to the snap-through instability peak; past that peak a continuous ribbon snaps downhill in elastic energy
  into its three-loop coil and is held there by hem self-contact. Because the 48-capsule chain disables self-collision
  to prevent joint snagging, once both hands hold segments `>= 16` segments (`120 deg`) apart and reach `<= 16 cm` gap
  with `>= 20 deg` opposite twists (`>= 90 deg` relative), the ring's PhysX joint spring rest-targets relax over
  `0.85 s` to the uncut $Lk=0$ three-loop equilibrium (`Wr = +-2, Tw = -+2`, 14.5 cm across,
  `1.80 N m/rad = 3 x 0.60 N m/rad` three-wire bundle stiffness, `0.012 N m s/rad` damping). Root poses, velocities and
  joint positions are never written directly, and held segments are never reassigned—both hands retain their original
  grips throughout buckling.
- **Loop retention and springing open.** Holding a single loop in mid-air or leaving loops protruding out of the pouch
  mouth does not retain the coil. Unless the coil is held across opposite arcs by two hands (`<= 22 cm` apart), pressed
  flat onto the table (`z_min <= 2.0 cm`, mean height `<= 7.0 cm`, holding TCP `<= 4.5 cm` above the table) during
  hand-over, clamped across `>= 2` distinct loops (`>= 10` segments apart) in one jaw, or wholly confined inside the
  upright pouch (`inside_pouch`: every wire segment plus its `5 mm` hem radius inside the side/back walls, past the
  mouth, and below the top panel), after `0.15 s` its joint rest-targets relax to `0` over `0.30 s` and PhysX springs
  the hoop back open.
- **Mesh panel and pouch.** On a real side-window shade (`~0.15 mm` sheer polyester mesh, `< 15 g` of the `90 g`
  shade), the slack mesh pleats flat inside the `1 cm` hem thickness and adds `< 2 mm` to the `1.5 cm` three-loop
  stack—well inside the `4 cm` pouch opening. In simulation, the dark mesh panel is drawn as a polygon fan from hoop
  centre to wire (redrawn each step, zero mass and collision) so cameras see the full shade without cloth self-locking;
  the pouch is welded rigid panels matching the `40 g` `3 mm` fluted polypropylene twinwall sleeve (`22 x 20 cm`,
  `4 cm` inside height, `7 x 7 cm` thumb notch).
- The scorer reads hoop segment centres, pouch pose, and gripper TCP and pad-tip positions, including the 5 mm wire-hem
  radius on every side and below the top panel.

## Reset

Take the coil out of the pouch and let it spring open. Lay the shade flat with its centre on the mark at (38, 0). Put
the empty pouch upright on its mark at (45, 38), mouth facing the robot (-x).
