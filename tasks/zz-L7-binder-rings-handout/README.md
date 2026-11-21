# Take a punched handout from the 3-hole punch, thread all three binder rings and snap them shut

A 10-sheet US letter handout has just been punched and sits in the throat of a desktop 3-hole punch on the left of the
desk. Beside it on the right, a 1 in three-ring view binder lies open flat with its three round rings open. The robot
draws the floppy handout out of the punch throat without snagging, carries it across, threads all three punched holes
onto the three open binder rings, lays the pages flat on one cover, and snaps the rings shut.

Instruction: `Put the punched handout into the binder and close the rings.`

## Why it matters and what makes it hard

- Filing a punched handout in a three-ring binder is an everyday school, home and office task anyone recognises at once.
- Ten loose sheets of 20 lb bond paper are floppy: pinched at one corner or one side, the unsupported 27.9 cm width
  sags and twists so the three holes (10.8 cm apart, 21.6 cm outer span) fail to meet the three open ring tips together.
  Both hands grasp the left and right side margins to pull the packet straight out of the 4 mm punch throat and hold the
  punched edge straight while lowering all three 7.14 mm holes onto the 3.5 mm ring wires simultaneously.
- Closing the rings while any ring sits against unthreaded paper pinches or tears the handout (disqualified with 0).

## Scene (station desk; cm from the arm-base midpoint, x forward, y left, z above desk top)

Arm bases sit at `(0, +-30.5)`, 0.25 cm above the desk top (`z = 74.75 cm` world). Both the punch base and the open
binder's outer cover edges are held on their desk marks with masking tape for repeatable resets.

| Object | Commercial product & material | Dimensions & pose |
|---|---|---|
| 3-hole punch | Swingline 74063 12-sheet desktop 3-hole punch (steel deck/jaw, dark chip base and handle) | `29.7 (y) x 10.2 (x) x 5.7 (z)` cm centred at `(43.7, +15.5)` (`x = 38.6..48.8`, `y = +0.65..+30.35`); 4 mm throat slot facing `-x` at `x = 38.6`, `z = 2.8..3.2` cm, internal paper stop at `x = 40.5` cm |
| Punched handout | 10 sheets of US letter 20 lb (75 g/m2) white bond paper, printed header and text lines up | `21.6 (x) x 27.9 (y)` cm, `46 g`, `1.1 mm` thick; three `9/32 in (7.14 mm)` holes on `4.25 in (10.8 cm)` centres, `9.5 mm (3/8 in)` from the `+x` edge; starts seated in the throat at `x = 40.4`, `y = +1.5..+29.5`, overhanging `-x` onto the desk |
| 3-ring binder | Avery 05714 Heavy-Duty View 1 in round-ring binder (navy polypropylene covers, nickel steel mechanism) | Open flat `54.0 (x) x 29.5 (y) x 0.3` cm centred at `(29.5, -15.5)` (`x = 2.5..56.5`, `y = -30.25..-0.75`); `27.0 x 2.2` cm spine plate along `y` at `x = 29.5` (`y = -29.0..-2.0`, top `z = 0.9`) with `1.8` cm end booster levers; three `25.4 mm` ID round rings (`3.5 mm` wire) at `y = -26.3, -15.5, -4.7`, open `26 deg` per half (tips `~2.4` cm apart at `z = 3.15` cm) |

## Run a trial

Requires `yam-isaac-lab-arena` on Isaac Lab-Arena 0.3.1 (`paper.py` is in this folder). Step limit: **900 steps**
(45 s at 20 Hz; layout fixed with `object_jitter=0`).

```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 900 \
  --instruction "Put the punched handout into the binder and close the rings."
```

## Scoring (from trial video and end photo; exact match to `score()`)

- **Score 0** if the handout ends torn or crimped, checked on the end state by turning the pages one by one: a hole torn
  out toward the sheet edge, or a page crimped under a shut ring. (In sim, a threaded hole pulled `> 6 mm` past its ring
  wire for `> 0.15 s` records a permanent tear, and snapping the rings shut while unthreaded paper lies across a ring
  records a crimp. Both persist to the end state.) Also 0 if at the end any part
  of the handout remains inside the punch footprint (`x` in `38.6..48.8` cm, `y` in `+0.65..+30.35` cm).
- **1.0** when the handout is out of the punch, all three holes are threaded onto the three distinct binder rings (in
  either cover orientation), all three rings are snapped shut, the handout lies flat on one cover (sheet centre `>= 6 cm`
  to one side of `x = 29.5` cm, `y` centre within `-25.25..-5.75` cm, and `>= 95%` of the sheet points lying beyond `+-2.5 cm`
  of `x = 29.5` cm within `3.5 cm` of the desk; the ring strip itself is excluded), nothing is held (both jaws open `> 2.4 cm`), and both arms are parked
  (every gripper part `>= 10 cm` from all binder parts and the handout, TCP speed `< 2 cm/s`, held `>= 0.5 s`).
- **0.6** when the handout is out of the punch and all three holes are threaded onto the three distinct binder rings
  (rings open or shut), without meeting every `1.0` condition.
- **0.3** when the handout is out of the punch and `1` or `2` holes are threaded onto distinct rings, with no tear/pinch.
- **0.0** otherwise.

## Simulation fidelity and demo

- The taped 3-hole punch and open binder covers/spine plate are fixed rigid boxes.
- The 10-sheet handout is modelled as ONE reduced position-based (XPBD) `22 x 15` sheet (pages do not slip against each other) (`46 g`, `1.1 mm` thick, inextensible stretch
  and tethers, bending stiffness `3.3e-3 N m` for ten loose 20 lb bond sheets, uncalibrated) with Coulomb friction on
  the desk, punch, binder, ring wires and gripper boxes, rendered every step with 8-sided hole cut-outs and printed text
  bars. Closing a jaw onto the sheet (`<= 0.4 cm`, both pads on the `1.1 mm` packet) with border nodes between the pads clamps that margin until the jaw opens (`> 2.4 cm`)
  or slips (`> 4.5 cm`).
- All six half-rings and both end booster tabs share one opening angle (`26 deg` open) with a bistable over-centre snap
  past `12 deg`, driven only by opposing finger pads squeezing a ring's two halves or pushing an end booster tab inward.
  Open ring tips enter the sheet only within `3.2 mm` of a hole centre.
- A scripted policy planning from simulator poses grasps both side margins outside the punch throat, slides the handout
  out, aligns and lowers all three holes onto the three near ring halves, releases, squeezes the middle ring shut and
  parks clear in ~400 steps (`1.0`); idle (`0.0`), unthreaded ring-close (`0.0`), one-arm (`0.3`), one-ring (`0.3`),
  open-rings (`0.6`) and unparked (`0.6`) policies verify the partial-credit and disqualifier tiers.
