# Bandage a sprained wrist in figure-of-eight turns, passing the roll hand to hand under the arm

Instruction: `Bandage the sprained wrist with the crepe bandage in figure-of-eight turns, fasten its end, then let go.`

A life-size rigid right forearm-and-hand form is clamped at its elbow end on a stand so the forearm, wrist and hand project along `+x` over the table, palm down, 16.0 cm up, clear underneath. A coreless elastic crepe bandage starts rolled on the left side of the wrist with 20.0 cm of its outer end flat under the wrist. One hand picks up the roll while the other pinches the outer end onto the wrist, anchors it with wrist turns, and lays two figure-of-eight loops: diagonally up across the back of the hand, under the palm beyond the thumb, diagonally back across the hand crossing the upward pass, and under the wrist. Because the arm is clamped at its elbow, the roll must pass hand to hand under and over the limb on every turn while held taut so the wrap conforms snugly and does not unwind, then fasten by its inner hook-and-loop patch and park clear.

## Why it matters

Bandaging a sprained wrist in figure-of-eight turns is standard first aid (St John Ambulance, Red Cross) practised at home, in schools and at sports events:

- **Repeated under- and over-limb handoffs** (`~7.8 cm` up underneath, `19-24 cm` over the top) while keeping both wrist housings clear of the limb and each other.
- **Keeping the elastic wrap taut across handoffs** so each turn conforms snugly to the curved wrist and palm (`<= 1.0 cm` stand-off) instead of bridging loose or peeling back when changing hands.
- **Figure-of-eight steering** crossing cleanly on the back of the hand while leaving the abducted thumb and fingertips bare.

Unlike single-wrap cuffs/bracelets, one-hand pipe tape, or routing rope round vertical table pegs, this requires layered wrapping of a cantilevered limb via repeated under-limb roll handoffs. Public deformable suites (GarmentLab, DexGarmentLab, DLO-Lab, PlasticineLab/DexDeform) drape scarves, stretch bands on pegs, or mould dough/rope on tables; none wraps a suspended limb in figure-of-eight turns with hand-to-hand roll transfers underneath.

## Scene

Coordinates in cm from the arm-base midpoint (`x` forward, `y` left), heights above the station table (`z = 0.7475 m`).

| Object | Model | Dimensions | Real-world part / fabrication | Start pose |
|---|---|---|---|---|
| Forearm-and-hand form & sub-bandage pressure cuffs | Fixed rigid collider with 2 flush TPU cuff bladders | Length 42.0 cm (`x` 5.0-47.0); forearm capsules (`x` 5.0-29.0) tapering 6.6 to 5.7 cm dia at wrist (`x` 29.0); continuous hand with palm `10.5 x 8.5 x 2.8 cm` (centred `(34.75, 0, 15.7)`, `x` 29.5-40.0, 1.0 cm corner radius) and fingers `7.5 x 7.6 x 1.8 cm` (centred `(43.25, 0, 15.6)`, `x` 39.5-47.0, 0.8 cm corner radius); thumb 6.5 x 2.1 cm from `(30.0, +3.6, 15.2)` to `(32.2, +9.5, 13.8)`; flush slate-grey cuff bladders on wrist (`x` 20.0-28.0 cm, `8.0 cm` wide) and palm (`x` 31.0-39.5 cm, `8.5 cm` wide) | Hollow 3D-printed ABS/PLA right forearm-and-hand form (`FOREARM`, `PALM`, `FINGERS`, `THUMB` in `bandage.py`) fitted with 2 commercial infant single-tube TPU cuff bladders (ADC `815-7N` / Welch Allyn `5082-163`, ~$12 each) beneath a thin Lycra sleeve, tubes routed internally out the clamped elbow (`x = 5.0 cm`) to two `0-375 mmHg` digital gauge transducers (Honeywell `MPX5050DP`, `0.1 mmHg`, logged at 20 Hz, <$60 total BOM) | Axis `+x` on `y = 0`, 16.0 cm up, palm down, thumb left (`+y`), clear underneath; cuffs tared to `0.0 mmHg` |
| Elbow clamp stand & scale bar | Fixed rigid boxes | Base `14.0 x 20.0 x 1.0 cm` with `1.8 x 20.0 x 0.2 cm` white scale bar across `y = -10.0..+10.0 cm` at `x = 14.5 cm`; post `3.0 x 3.0 x 10.8 cm`; cradle `7.0 x 8.0 x 1.2 cm`; top strap `3.0 x 7.2 x 0.8 cm` (centre 19.6 cm up) | Metric steel bar stock machined to `OBJECTS` in `task.py`, bolted/clamped to table with a 20.0 cm white millimetre scale bar on the front edge | Stand at `(9.0, 0.0)` cm; scale bar at `(14.5, 0.0, 1.1)` cm |
| Elastic crepe bandage | PBD strip unwinding from shrinking coreless roll (35 g) exerting pull on holding jaw | 7.5 cm wide, 1.66 m relaxed (84 stations at 2.0 cm), 1.2 mm ply; blue selvedges and blue cross-ticks every 20.0 cm relaxed (`0..160.0 cm`) for bench-rule reset checks; inner 4.0 cm (`162.0..166.0 cm`) sewn hook patch; 1.46 m wound coreless (~14 mm inner turn) to `4.0 cm` dia x `7.5 cm` | 3M ACE 3 in. (7.5 cm) Elastic Bandage with Hook Closure (catalog 207603, coreless), trimmed/hemmed to 1.66 m relaxed and marked across its width every 20.0 cm | Roll at `(24.0, +13.0)` cm, axis `+x`; 20.0 cm outer end flat along `-y` to `(24.0, -8.2)` cm |

## Reset

Peel off the hook patch, unwrap the bandage, and tare both wrist and palm cuff pressure transducers to `0.0 mmHg` on the bare limb form. Lay the 1.66 m bandage flat and unstretched against a 2 m bench rule, hook patch face down, verifying its 20.0 cm reset marks sit `20.0 +/- 0.3 cm` apart (letting the elastic relax if needed). Fold the stiff 4.0 cm inner hook patch inward (~14 mm coreless centre) and wind 1.46 m unstretched to a `4.0 +/- 0.2 cm` roll leaving 20.0 cm of the outer end free. Set the roll at `(24.0, +13.0)` cm, axis `+x`, and smooth the outer end flat along `-y` to `(24.0, -8.2)` cm beneath the wrist.

## Scoring

Evaluated on physical wrap outcome at episode end (`SCORE_OVER = "end"`, regardless of whether roll handoffs occur beneath, beside or over the limb). Laid strip is segmented from outer to inner end into dorsal runs (above `z = 16.0 cm`) and underside runs (sweeping `>= 1.2 rad` below `16.0 cm`). Dorsal runs shifting `>= 3.0 cm` along `+x` / `-x` are upward / downward diagonals; wrist band is `x < 30.0 cm`, hand band `x > 33.5 cm`.

**Score 0** if fewer than `1.0` full net wrist revolution (`360 deg`, `x < 30.0 cm`) is laid before the first diagonal.

| Score | Requirement |
|---|---|
| **0.25** | `>= 1.0` full net wrist revolution (`360 deg`, `x < 30.0 cm`) before diagonals. |
| **0.50** | Plus `>= 1` complete figure-of-eight loop in a consistent winding direction (upward diagonal shifting `>= 3.0 cm`, palmar pass at `x > 33.5 cm`, crossing downward diagonal, wrist underside pass at `x < 30.0 cm`). |
| **0.75** | Plus a 2nd figure-of-eight loop; thumb beyond 40% (`>= 5 mm` clear) and distal 3.0 cm of fingers (`x > 44.0 cm`, `>= 10 mm` clear) bare; entire 1.66 m unwound off the coreless roll (`0` remaining roll) in `6.0` to `9.0` total net revolutions with no part standing `> 1.0 cm` off the skin or underlying wrap; and sub-bandage compression pressure within `12.0` to `55.0 mmHg` on both the wrist cuff (`x` `20.0-28.0 cm`) and palm cuff (`x` `31.0-39.5 cm`), rejecting both loose/unsupported (`< 12.0 mmHg`) and locally or globally over-tight (`> 55.0 mmHg`) wraps. |
| **1.00** | Plus the 4.0 cm hook patch fastened, nothing held, both jaws open past 5.0 cm, every gripper point `>= 10.0 cm` from form and bandage, and both TCPs `< 2.0 cm/s` with both cuff pressures holding in `12.0` to `55.0 mmHg` continuously for 1.0 s (`SUCCESS_HOLD_S = 1.0 s`). |

**Real-trial verification:**
1. **Overhead and side video** (calibrated via the 20.0 cm white scale bar on `stand_base`) verifies `>= 1.0` initial wrist revolution before the first diagonal, traces the two figure-of-eight dorsal/palmar loops as applied, and counts total net revolutions of the unwinding roll round the limb (`6.0` to `9.0` turns until the coreless roll is completely unwound).
2. **Sub-bandage support compression (`12.0-55.0 mmHg`), conformance & patch retention:** during the 1.0 s hands-off hold (`>= 10.0 cm` gripper clearance, `< 2.0 cm/s`, jaws `> 5.0 cm`), the 20 Hz digital transducer log from the tared `8.0 cm` wrist bladder (`x` `20.0-28.0 cm`) and `8.5 cm` palm bladder (`x` `31.0-39.5 cm`) must read `12.0` to `55.0 mmHg` on both zones (each bladder spans ~1 bandage width of `7.5 cm`, summing radial hoop pressure across all inner and outer plies without camera occlusion or opposing-side cancellation). Overhead, side and palmar photos with a millimetre ruler verify the two dorsal crossings, palmar passes, bare thumb (`>= 5 mm` past 40%) and bare distal 3.0 cm fingertips (`>= 10 mm`); a **10.0 mm cylindrical feeler rod** confirms no loop stands `> 1.0 cm` off the skin or wrap; and after dimensional/pressure readings, a `1.0 N` spring scale hooked tangentially to the 4.0 cm patch's outer edge for `2.0 s` must produce `< 5.0 mm` peel displacement.

## Simulation fidelity

- **Bandage & wrench (`bandage.py`, `task.py`):** NumPy PBD strip (3 cols across 7.5 cm at 2.0 cm spacing, rendered with blue selvedges and 20.0 cm cross-ticks; 8 substeps x 6 iterations per 20 Hz step) driven solely from measured TCP poses and jaw openings. Free-span pull (`WRENCH_K = 6.0` N/strain, capped at 6 N) plus roll weight applies a 3D wrench onto the holding gripper in PhysX (`permanent_wrench_composer`).
- **Payout, laying, recoil & Laplace cuff pressure:** up to 2 stations (`4 cm`) unwind per step past `1.42x` pull at light grip squeeze (`<= 3.5 mm`), rising up to `2.47x` under tighter jaw clamping (`> 3.5 mm` brake friction) so clamping and pulling wraps tighter turns. Contacts within `3.4 mm` lay while taut (`>= 1.05x`), recording each station's laid tensile strain (`BAND_K = 10.0` N/strain, ~2.5 N at 25% stretch); unheld unfastened wrap peels back at `40 cm/s`. Sub-bandage pressure on the wrist (`x = 20.0-28.0 cm`, `R = 2.95 cm`) and palm (`x = 31.0-39.5 cm`, `R = 3.33 cm`) cuffs is integrated over all laid plies by Laplace's law (`P = sum(T * ds / R) / (2 * pi * R * L)` in `mmHg`).
- **Two-pad contact grasps & pressure-latched patch:** roll grasp engages **only on two-pad contact** across end faces (`7.5 cm`) or barrel (`2r`, `<= 3.5 mm` per pad) while jaws actively close (`+8 mm` release); unheld roll falls under gravity. Strip pinch likewise requires two-pad contact with closing jaws (`< 14 mm`; releases `> 24 mm` or `4 cm` slip). When the last turn unwinds, the holding jaw retains the 4.0 cm inner hook patch, which latches **only when all 3 patch stations contact laid wrap (`<= 2.0 mm` gap) under `>= 1.5 mm` inward pad compression for `0.25 s`**.
- **Simplifications:** the rigid form omits soft-tissue deformation, and Laplace cuff pressure (`BAND_K = 10.0` N/strain, roll-clamp payout friction, recoil rate and tack thresholds) is an approximate mechanical stand-in for pneumatic TPU bladder readings rather than a calibrated continuum finite-element tissue model.

## Requirements & Trial

`yam-isaac-lab-arena` and Isaac Lab-Arena `0.3.1` with `inspect-robots`; keep `bandage.py` beside `task.py`. Run limit: `2,400` steps (`120 s` at `20 Hz`, `object_jitter=0`).

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Bandage the sprained wrist with the crepe bandage in figure-of-eight turns, fasten its end, then let go."
```

### Real fixture build and cuff calibration

- **Limb form:** every surface is a primitive with the dimensions in the Scene table (`FOREARM` capsules, rounded-box `PALM`/`FINGERS`, `THUMB` capsule in `bandage.py`), so the form is drawn directly from those numbers in any CAD tool and exported as STL. Print in PLA/ABS with 3 mm walls and 20% infill, split at the wrist (`x = 29.0 cm`) and bolted with two M5 inserts so each half fits a 25 cm bed. Sand to a smooth finish; recess the two bladder pockets 4 mm deep at the cuff positions so the bladders sit flush.
- **Elbow stand:** steel bar cut to the stand boxes in `OBJECTS` (`task.py`), the cradle and top strap clamping the form's elbow end with two M6 bolts; the 20.0 cm white scale bar is a printed millimetre strip glued to the base front edge.
- **Bladder preparation:** seat each infant TPU bladder in its pocket, route its single tube through the hollow form out of the elbow end, and cover both with one thin Lycra sleeve. Connect each tube through a 3-way stopcock to its `MPX5050DP` transducer and pre-fill the bladder with `2.0 mL` of air from a syringe, then close the stopcock to the syringe (the usual pre-fill for pneumatic sub-bandage pressure probes).
- **Calibration (once per session):** with a reference aneroid sphygmomanometer teed into the same line, wrap a standard blood-pressure cuff over each bladder and inflate it to `0`, `20`, `40` and `60 mmHg`; the logged transducer reading must agree within `+/- 2 mmHg` at each step, otherwise re-zero or re-fill and repeat. Remove the cuff and tare both channels to `0.0 mmHg` on the bare form before every trial (see Reset).

## Demo and shortcut checks

`demo/` contains a full-score scripted run (`score = 1.0`, `1,513` steps / `75.7 s` at 20 Hz) via joint-space IK moves: left picks up the roll, right drapes the outer end onto the wrist, the arms pass the roll under (`z = 7.8 cm`) and over (`z = 19-24 cm`) the limb through wrist turns, two figure-of-eight loops and finishing wrist turns, press the hook patch under normal pad compression for `0.25 s`, and park `> 17 cm` clear.

Dream-1 declined before moving (`DA-4000 BAD_REQUEST (malformed): unsafe instruction, refused: Applying a tight bandage on a person's wrist risks injury.`), scoring `0.0`.

| Policy | Behaviour | Score |
|---|---|---|
| Idle (`nothing`) | Arms stay still | `0.0` |
| Single arm, drop (`one_arm`) | Left carries roll over wrist and drops it | `0.0` |
| Single arm, hold (`one_arm_hold`) | Left sweeps roll round wrist and keeps holding | `0.0` |
| Mid-wrap release (`drop_midwrap`) | Wraps wrist turn + under-arm pass, then releases roll so wrap unwinds | `0.0` |
| Unfastened release (`no_fasten`) | Completes wrap, opens jaw without pressing patch, parks | `0.0` |
| Wrist turns only (`wrist_only`) | Wraps wrist anchor turns with under-arm passes, holds | `0.25` |
| Single loop (`one_loop`) | Wraps wrist turns + 1 figure-of-eight loop, holds | `0.50` |
| Over-tight wrist turns (`overtight`) | Clamps roll hard on wrist turns (`> 55.0 mmHg` wrist cuff), completes both loops, fastens & parks | `0.50` |
| Hold at end (`hold`) | Completes wrap and fastens patch within `12.0-55.0 mmHg`, keeps gripper near wrap | `0.75` |

## Sources

- St John Ambulance, *Roller Bandage (Figure-of-Eight for Wrist/Ankle Sprains)*: wrist anchor turns, diagonal figure-of-eight loops across back of hand and palm leaving thumb and fingertips bare, finish at wrist and fasten.
- 3M ACE Brand 3 in. Elastic Bandage with Hook Closure (catalog 207603): 7.5 cm coreless elastic wrap with integrated hook fastener.
