# Push a brake caliper's piston flush on the car with a C-clamp over the worn pad, hose attached

Instruction: `Push the brake caliper's piston flush into its bore with the C-clamp over the worn pad without cocking the piston or kinking or stretching the brake hose, then remove the clamp.`

## Who does this
Automotive service technicians and mechanics (O*NET 49-3023.00, 825,800 US jobs). O*NET core task: "Repair, reline,
replace, and adjust brakes." Front disc pad replacement on a single-piston floating caliper is one of the most frequent
shop brake jobs: as pads wear, the caliper piston walks out of its bore, and thicker new pads will not fit over the
rotor until the piston has been pressed back flush. Tire shops, quick-service bays and home mechanics use the same
C-clamp-over-worn-pad method.

## The brake-specific skill and why it needs two hands
With the front wheel removed on a lift or jack stands, the unbolted caliper rests slot-up on a wooden support block on
the lower control arm inside the wheel well, still tethered to the chassis hardline bracket by its flexible rubber brake
hose so the hydraulic line is never opened at the banjo bolt.

Unlike brake-pad insertion (sliding pads into clips and swinging a hinged caliper down), brake bleeding (timing a
bleeder wrench), or simple one-shot clamping on an open bench, this operation tests four tightly coupled constraints:
- **Confined wheel-well placement and extraction:** lifting a `0.51 kg`, `4 in` C-clamp off its rack, clearing the
  `16 cm` fender arch into the wheel well in front of the `262 mm` rotor and hub studs, and seating its swivel pad on
  the caliper's rear housing with its fixed jaw inside the caliper slot against the worn pad's backing plate/lining.
- **Continuous `<= 3 deg` bore alignment on a free `1.80 kg` caliper:** one hand must hold and re-square the C-clamp by
  contact alone while the other turns the T-handle against `150-220 N` piston-seal resistance; letting the screw drift
  more than `3 deg` off the bore axis cocks and binds the piston in its bore.
- **Multi-turn T-pin regrasping within wrist limits:** the sliding T-pin hits the wrist's joint limit after half a turn,
  requiring ~14 forward half-turns (release, unwind, re-pinch) to close the jaw clearance and press the `8.0 mm`
  protrusion flush, followed by 4 reverse half-turns to relieve preload before lifting the clamp clear.
- **Hose strain and kink protection:** dragging or twisting the loose caliper pulls the `18.5 cm` EPDM brake hose taut
  (`> 18.0 cm` span) or kinks it (`> 55 deg` bend), damaging the reinforced hose carcass.

## Scene (cm; `x` forward from the arm-base midpoint, `y` left; heights above the table)
Compact-car front corner with the wheel removed, facing the open wheel well (`-x` toward the robot):

| Item | Real product / size | Place |
|---|---|---|
| Wheel-well liner and fender arch | rigid static polypropylene liner + painted arch trim: back wall `2.0 x 36.0 x 33.0`, right wall at `y = -23.5` (`18 cm` front, `31 cm` rear), left wall at `y = +10.5` (`16 cm` front, `31 cm` rear), top hood `9.0 x 35.8 x 2.0` at `32.0 cm` up | back wall at `x = 62.5`, `y = -6.5` |
| Steering knuckle, strut, hub, rotor | static: knuckle `5.2 x 10.5 x 20.0` at `(58.5, -7.5, 14.0 up)` with `54 mm` strut tube and coil spring; `136 mm` hub hat with `4 x 100 mm` lug studs and spindle nut; `262 x 21 mm` vented rotor (Honda Civic front size) with caliper bracket and outer pad | rotor centre `(54.8, -7.5, 16.8 up)`, axis along `x` |
| Lower control arm & wood block | static dark steel arm `8.0 x 24.0 x 7.4` centred `(46.5, -4.5)` carrying static hardwood support block `9.0 x 7.0 x 4.0` | block centred `(42.5, -1.5)`, top `11.4 cm` up |
| Caliper | single-piston floating front caliper (reference: Cardone `19-B1833` for Honda Civic, `54 mm` steel piston); `1.80 kg` cast-iron body: `5.0 x 11.6 x 6.8 cm` housing, `6.0 cm` lower bridge, outboard fingers (`x = 46.6..48.2`), side ears, top bleeder, brass `-y` banjo block | free on block, bore axis along `x` at `y = -1.5`, `14.8 cm` up; back face `x = 36.5`, bore face `x = 41.5`, open top inspection slot |
| Piston + worn inner pad | `54 mm dia x 30 mm` steel piston with clipped worn inner pad (`6 mm` red shimmed steel backing plate `11.0 x 5.4 cm` + `3 mm` friction lining `10.2 x 4.8 cm`); `0.25 kg` combined | piston face `8.0 mm` out of bore (lining face at `x = 43.2`) |
| Flexible brake hose | `12 mm` OD SAE J1401 EPDM rubber hose, `18.5 cm` arc length (`72 g`), from caliper banjo `(39.0, -8.1, 15.6 up)` to fixed steel hardline bracket on right liner `(49.0, -21.2, 20.5 up)` | `17.2 cm` straight-line chord at spawn (`~16.7 cm` settled loop) |
| C-clamp | IRWIN Quick-Grip `225104` `4 in` ductile-iron C-clamp (`0.51 kg`: `0.40 kg` frame, `0.06 kg` T-handle, `0.05 kg` screw/pad): `7.6 cm` (`3 in`) throat, `18 mm` top-bar I-beam grip web over `26 x 4 mm` lower flange with keeper ribs, `9/16-12 UNC` screw, `22 x 6 mm` swivel pad, `10.2 cm x 9 mm` sliding steel T-pin | jaws opened to `73 mm` (`e = 4.5 cm`, fixed jaw at `11.8 cm`), hanging outside `+y` arch on rack |
| Tool rack | `1.0 cm dia x 9.5 cm` steel rod along `y` at `(43.7, 17.0, 21.9 up)` on a `1.6 x 2.0 x 23.9 cm` post at `(43.7, 22.8)`, plus a `1.8 x 4.0 x 13.6 cm` wood rest at `(33.6, 17.0)` (all static) | clamp boss at `(33.5, 17.0, 14.8 up)` |

Grasp heights: C-clamp top-bar saddle `22.4-24.6 cm` up, T-pin `10-19 cm` up (all inside YAM reach; inter-wrist clearance stays `>= 14 cm`).

## Scoring (evaluated on the final state; "at any time" rules latch over the whole trial)
- **Piston out** = axial distance the piston face stands past the caliper's bore face along the bore axis (`8.0 mm` at start).
- **Clamp seated** = clamp pointing forward along the bore (`dot(cx, kx) > 0`), swivel pad within `2.0 mm` axially of the caliper's back face and within `2.0 cm` radially of the bore axis, and fixed jaw (anvil) within `2.0 mm` axially of the worn pad's lining face and within `2.0 cm` radially of the bore axis.

**Score `0`** if at any time:
- the caliper, piston, C-clamp frame, T-handle or screw drops off the table (`low_z < table_top - 2.0 cm`);
- the caliper tips more than `15 degrees` from its initial upright axis or slides more than `3.0 cm` horizontally on its support block;
- the piston is pushed `1.0 mm` or more inward while the C-clamp is not seated (e.g., shoving or wedging a gripper or unseated tool into the slot);
- while seated, the clamp pushes the piston `1.0 mm` or more inward while its screw axis is more than `3.0 degrees` off the bore axis (cocked);
- the flexible brake hose is pulled taut (`banjo-to-bracket` chord `> 18.0 cm`, over `97%` of its `18.5 cm` length) or kinked (any bend angle between adjacent hose segments `> 55 degrees`).

**Otherwise:**
- **`0.25`**: piston more than `4.0 mm` out, and the C-clamp is currently seated (requires closing the `73 mm` jaw opening onto the `67 mm` caliper-and-pad span).
- **`0.5`**: piston at most `4.0 mm` out (retracted with the clamp seated).
- **`0.8`**: piston at most `1.0 mm` out (flush).
- **`1.0`**: piston at most `1.0 mm` out; the C-clamp (frame, handle and screw) is supported above `table_top - 1.0 cm`, at rest (linear speed `<= 2.0 cm/s`, angular speed `<= 0.5 rad/s`), and at least `2.0 cm` clear of the caliper, piston/pad and brake hose; and both grippers are released (`>= 20 mm` jaw opening) with their TCPs at least `10.0 cm` from the bounding boxes of the caliper, piston/pad, brake hose and C-clamp, each TCP moving at most `1.0 cm` over the last `1.0 s` (`20` steps).

## Real-trial setup, hydraulic repeatability and reset
- **Closed, spill-free hydraulic circuit:** Bolt a salvage compact-car front corner (knuckle, strut, `4x100` hub, `262 mm` rotor with bracket and outer pad, lower control arm, plastic wheel-well liner) to the station table over a shallow drip tray. Fill the Cardone `19-B1833` caliper and its `18.5 cm` SAE J1401 rubber hose with DOT 3 glycol fluid (or propylene-glycol/water test fluid), bolt and plug the hose's chassis fitting at the right liner bracket so no master cylinder is needed, and crack the caliper's top bleeder `1/4` turn with a snug clear vinyl tube submerged below fluid level in a Weighted catch bottle behind the liner. Retracting the piston expels fluid through the fixed bleeder restriction against the real square-cut EPDM bore seal, giving repeatable `~150-220 N` resistance with zero open fluid spills.
- **Safe reset between trials (cannot over-extend or eject piston):** Place an `8.0 mm` step block against the bore face and slowly pull the clipped inner pad outward with pliers until the backing plate meets the `8.0 mm` stop — drawing fluid cleanly back through the submerged bleeder tube without introducing air or risking piston pop-out (`30 mm` piston skirt stays `22 mm` inside the bore). Set the caliper slot-up inside tape marks on the hardwood block, back the IRWIN `225104` C-clamp open to `73 mm`, and hang it on the rack.
- **Real-trial measurement:** Paint high-contrast axial witness stripes along the caliper housing top and C-clamp screw boss, mark `8 / 4 / 1 mm` protrusion ticks on the caliper inspection-window rim beside the red pad backing plate, place a `55 deg` bend-limit template card behind the hose loop, and record from a fixed side-slot camera alongside the overhead and wrist cameras so `<= 3.0 cm` block slide (tape outline), `<= 15 deg` caliper tilt, `<= 3 deg` screw-to-bore alignment while turning, `<= 18.0 cm` hose span (`18 cm` witness string) and absence of hose kinks (`> 55 deg` fold) stay continuously visible on video. Measure final piston protrusion directly through the open top window with a vernier depth rod from the machined bore face to the flat pad backing plate (`8.0 mm` at start, `<= 1.0 mm` flush), and verify `>= 2.0 cm` clamp clearance, `>= 20 mm` jaw opening, `>= 10.0 cm` TCP clearance and `1.0 s` end stillness from ruler marks and video.

## Run a trial
Requirements: `yam-isaac-lab-arena` on Isaac Lab-Arena `0.3.1` (`20 Hz` control, `object_jitter=0` nominal layout).

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Push the brake caliper's piston flush into its bore with the C-clamp over the worn pad without cocking the piston or kinking or stretching the brake hose, then remove the clamp."
```

## Simulation fidelity
- **True-size rigid bodies (`240 Hz` PhysX, `20 Hz` control):** static wheel-well liner/fender arch, steering knuckle/strut, `136 mm` hub with lug studs, `262 x 21 mm` vented rotor with bracket and outer pad, lower control arm, hardwood block and rack; dynamic `1.80 kg` caliper, `0.25 kg` piston + worn pad, `0.40 kg` C-clamp frame, `0.06 kg` T-handle, `0.05 kg` screw/swivel pad, and `6`-capsule (`72 g`) brake hose.
- **Pure-contact grasp (no weld):** both grippers hold and manipulate the C-clamp and T-pin purely by physical contact — no fixed-joint weld or attachment constraint is used anywhere. On the real robot, the `0.51 kg` (`5.0 N`) clamp's `26 mm` lower I-beam flange (`4 mm` shelf per side) and keeper ribs mechanically support the frame on the fingers while `~45 N` parallel pinch with compliant elastomer pads (`~0.9-1.2` friction, `> 40 N` tangential hold) turns the `9 mm` T-pin against `~7.8 N` tangential load (`0.28 N m` at `3.6 cm` radius under `220 N` axial load); because rigid flat PhysX finger boxes lack soft elastomer wrap around ribs and round pins, the finger tips and C-clamp grip saddle/T-pin use `2.8` static / `2.4` dynamic friction as a stand-in for pad conformability.
- **Piston seal and bleeder drag:** prismatic bore joint (`-8.3 mm` to `0 mm`) with a `1.0e6 N/m` elastic seal-shear drive whose rest target slips plastically whenever axial load exceeds `150 N` (`0.15 mm` elastic shear), plus `6.0e4 N s/m` viscous fluid drag (`~190-210 N` while advancing). Bare-gripper pushes (`< 150 N`) deflect the seal `< 0.15 mm` and spring back without slipping.
- **Self-locking screw thread:** screw prismatic drive (`4.0e5 N/m`, `2.0e3 N s/m`) tracks cumulative T-handle revolute rotation at `2.117 mm/turn` (`9/16-12 UNC`). Axial load never back-drives the handle, and handle angular damping scales with `0.03 N m + load * lead / (2 pi * 0.30)` divided by `2.0 rad/s` (`~0.28 N m` torque under `220 N` load). The swivel pad is fixed to the screw tip; piston cocking is measured from the screw-to-bore angle while advancing.
- **Articulated brake hose:** `6` `12 mm` OD capsules connected from the caliper banjo to the fixed liner bracket by locked-translation, `2`-axis bend-spring joints (`0.20 N m/rad`, `0.03 N m s/rad`, `+-65 deg` stops); internal fluid mass transfer is omitted.

## Demo video and shortcut checks
`demo/` holds a scripted closed-loop two-arm run reaching `1.0` in `2,003` steps (`100.2 s`), within the `2,400`-step (`120 s`) limit (packaged side-by-side as `top_cam | left_cam | right_cam`). The left arm grasps the C-clamp's `18 mm` top-bar grip web by contact, lifts it off the rack and over the `16 cm` left fender arch into the wheel well, seats it across the caliper housing back and worn inner pad clear of the rotor and brake hose (`left_cam` at `t = 13 s` shows the bright chrome piston protruding `8.0 mm` between the grey bore face and red pad backing plate), and squares the screw `<= 0.6 deg` to the bore. The right arm turns the T-pin in `15` forward strokes (releasing, unwinding and re-gripping each stroke while the left arm floats the clamp with screw advance and keeps bore alignment `<= 0.4 deg` and hose span `<= 16.8 cm`) until `left_cam` at `t = 66 s` shows the chrome piston gap closed with the red backing plate pressed flush against the bore face, then reverses the T-pin to relieve preload. The left arm lifts the C-clamp straight out of the wheel well, hangs it back on the rack and releases, and both arms park `> 15 cm` clear (`t = 100.2 s`) with `left_cam` framing the stowed C-clamp on its rack and `right_cam` aimed into the open caliper slot showing the red pad backing plate flush against the grey bore face (no exposed piston skirt) and the unstretched, unkinked brake hose.

Scripted shortcut and partial policies evaluated by `score()`:

| Policy | Behaviour | Score |
|---|---|---|
| Idle | Both arms stay parked | `0.0` (clamp never seated, piston `8.0 mm` out) |
| One-arm bare shove | Right gripper shoves the worn pad directly without the C-clamp | `0.0` (`< 150 N` cannot slip piston seal; tips/slides caliper) |
| Two-arm bare squeeze | Left fist braces caliper back while right fist pushes worn pad | `0.0` (`< 150 N` gripper force cannot slip `150 N` bore seal) |
| Hose pull / kink | Right gripper pulls/folds the attached flexible brake hose | `0.0` (triggers hose kink `> 55 deg` / taut `> 18.0 cm` latch) |
| Cocked clamp (`> 3 deg`) | Seats clamp skewed `~6.5 deg` off bore and turns T-pin | `0.0` (latches cocked push `>= 1.0 mm` at `> 3.0 deg`) |
| Seat clamp only | Seats C-clamp square across caliper back and worn pad (`8.0 mm` out) | `0.25` (clamp seated, piston `> 4.0 mm` out) |
| Halfway push | Seats clamp square and retracts piston halfway (`~3.0 mm` out) | `0.5` (partial retraction `<= 4.0 mm` out) |
| Full push, clamp left on | Retracts piston flush (`<= 1.0 mm`), leaves clamp on caliper and parks | `0.8` (clamp not `>= 2.0 cm` clear of caliper/pad) |
| Full method | Retracts piston flush, backs off, stows clamp on rack and parks still | `1.0` |
