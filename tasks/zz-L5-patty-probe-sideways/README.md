# Check a burger patty's doneness: hold it with tongs and probe sideways into its centre

Instruction: `Take the burger patty's centre temperature with the thermometer, probe in through its side, then put the thermometer and tongs down.`

## Why it matters

Restaurant cooks (O*NET 35-2014.00, 1,417,500 US workers) regularly *"Observe and test foods to determine if they have been cooked sufficiently, using methods such as tasting, smelling, or piercing them with utensils"*, and food preparation workers (O*NET 35-2021.00, 908,500 US workers) *"Take and record temperature of food and food storage areas."* USDA FSIS safe-cooking guidance for ground beef patties specifies inserting an instant-read thermometer **sideways through the edge of a thin patty** until the sensing tip reaches the geometric centre, because probing from the top passes through a 1.6 cm patty or reads near the surface.

On an oiled flat-top griddle, pushing a 3.0 mm stainless probe into cooked beef requires roughly 2.5 N of axial force, while a 100 g patty slides at ~0.20 N. One arm must pick up the spring tongs and hold the patty steady while the other arm drives the unfolded instant-read probe horizontally (`≤ 20°`) into the side rim to the centre (`5.25 cm` deep, `±8 mm` in plan, `±4 mm` of mid-thickness), holds for `2.0 s` until the thermometer displays a stabilised core reading, withdraws the probe without dragging the patty, places both tools flat on the counter off the griddle, and parks both hands clear.

## Real-world setup and repeatable reset

Origin is the tabletop directly below the midpoint between the two YAM arm bases (`x` forward away from the arms, `y` left, `z` up above the table, in cm):

| Item | Commercial product / material | Dimensions & mass | Starting pose & orientation |
|---|---|---|---|
| Counter mat | Food-safe silicone mat | `60.0 × 44.0 × 0.15 cm` | Taped flat on the table centred at `(31.0, 0.0)` under the griddle to protect the shared rig |
| Griddle | **Lodge Double Play Reversible Grill/Griddle (`LDP3`)**, black seasoned cast iron, unheated | `42.5 × 24.1 × 1.6 cm` (`4.3 kg`), smooth flat griddle side up | Fixed/on mat, centred at `(34.0, 0.0)`, `42.5 cm` axis along `y` (`x: 21.95..46.05`, `y: -21.25..+21.25`, top face `z = 1.6 cm`); eight grey start ticks ring `(30.0, 0.0)` |
| Burger patty | Standard foodservice fully cooked flame-broiled quarter-pound beef patty (e.g., AdvancePierre / Hormel 4 oz cooked beef patty, or `113 g` 80/20 beef pressed in an `11.4 cm` ring, cooked to `71 °C` and chilled in a covered pan to **`4 ± 2 °C`**) | `10.5 ± 0.4 cm` dia (`R = 5.25 cm`), `1.6 ± 0.2 cm` thick, `100 ± 8 g` | Flat at `(30.0, 0.0)` on the oiled griddle (`mid-thickness z = 2.4 cm`); `4 mm` white food-safe starch/icing dot at its top centre |
| Thermometer | **ThermoWorks Thermapen ONE** folding-probe instant-read digital thermometer (yellow ABS housing, auto-rotating backlit LCD, motion-sensing sleep/wake, `1 s` response) | Body `15.6 × 4.8 × 1.9 cm` (`115 g`); 304 stainless probe unfolded straight `180°` along `+x`: `10.9 cm` exposed length, `3.0 mm` shaft tapering to `1.5 mm` sensing tip (thermocouple in distal `3 mm`); `3.0 mm` red food-safe vinyl tape band centred `5.25 cm` from tip | Flat on the right counter off the griddle, body centred at `(22.0, -28.0)`, probe pointing `+x` (`tip at (40.7, -28.0)`), asleep (`LCD "--"`, wakes to `~22 °C` ambient upon pickup) |
| Spring tongs | **12-inch (`30.5 cm`) brushed stainless-steel spring tongs** (e.g., OXO Good Grips 12-in / commercial 18-8 stainless scalloped utility tongs, **unlocked**) | `30.5 cm` overall length, `165 g`, non-slip cushioned grip pads `10.5 cm` from rear (`6.8 cm` outer width at grip pads), `5.0 × 2.8 cm` scalloped concave jaws sprung open `12.5 cm` between paddles | Standing stably on edge on the left counter off the griddle, rear hinge centred at `(13.0, +30.5)`, open jaws pointing `+x` |

**Between-trial reset and rig protection:**
1. Fit disposable `0.08 mm` nitrile covers/sleeves over both robot finger-pad pairs (taped clear of jaw travel) so neither gripper pad can ever be contaminated by food or oil; wipe both tool handles with a 70% isopropyl wipe between trials.
2. Wipe the cold cast-iron griddle dry with a paper towel, dispense `1.25 mL` (`1/4 tsp`) refined vegetable oil onto the central `15 × 15 cm` area, and spread evenly with a lint-free wipe. Before a session, verify with a spring scale that a `100 g` chilled patty slides horizontally at `0.15–0.30 N` (`μ ≈ 0.20`) while side-rim penetration with the `3.0 mm` probe requires `2.0–3.5 N`.
3. Take a fresh unpierced precooked patty directly from a `4 ± 2 °C` (`39 ± 4 °F`) refrigerator (marked with the `4 mm` white top-centre dot) and place it inside the start ticks at `(30.0, 0.0)`. Place the unfolded Thermapen ONE still at `(22.0, -28.0)` (`+x`, allowed `30 s` untouched to sleep if needed; motion-wake activates automatically when lifted) and unlocked tongs at `(13.0, +30.5)` (`+x`). Return both arms to `READY`.

## Scoring

`score(objects)` in `task.py` and the real-trial protocol evaluate the exact same conditions:

- **Score `0.0` (latched over the whole trial) if at any time:**
  1. Either robot gripper/finger pad directly touches the patty (`< 6 mm` clearance).
  2. The probe tip is inside the patty without the tongs held by a gripper and clamping both sides of the patty (`< 6 mm` jaw-to-patty clearance on each paddle).
  3. The patty's centre slides `> 2.0 cm` horizontally from its starting position at `(30.0, 0.0)`, tilts `> 30°` from flat, or leaves the griddle.
  4. The probe enters through the patty's top or bottom face (or its shaft cuts through either face) rather than its cylindrical side rim, or the tip ever exits through the top, bottom, or far side (anywhere other than back out through its rim entry hole).
- **Otherwise, highest tier achieved:**
  - **`0.2` (Stabilized rim entry):** With held tongs clamping both sides of the patty, the probe enters sideways through the rim (`≤ 20°` from horizontal) to at least `1.0 cm` radial depth from the rim, without reaching the centre zone.
  - **`0.5` (Centre reached, hold incomplete):** Probe entered sideways through the rim (`≤ 20°` from horizontal) and the tip reached the patty's centre zone — within `8 mm` horizontally of the patty's centre axis and within `4 mm` vertically of mid-thickness (`4–12 mm` above the griddle face) — with tongs stabilizing the patty, but was held there `< 2.0 s` (no stabilised reading).
  - **`0.7` (Valid centre reading acquired):** Tip held continuously in the centre zone (`≤ 8 mm` plan, `≤ 4 mm` mid-thickness, probe `≤ 20°` from horizontal, held tongs clamping patty throughout) for `≥ 2.0 s` (`40` steps at `20 Hz`) with a valid stabilised chilled core reading (`≤ 8.0 °C` / `46 °F`, rate `≤ 1.5 °C/s` on the Thermapen ONE LCD), **without** completing withdrawal, tool set-down, and arm parking.
  - **`1.0` (Full completion):** Achieved the `0.7` valid centre reading **and** at the end of the trial:
    1. Probe shaft and tip are completely withdrawn (`≥ 2.0 cm` clear of the patty).
    2. Thermometer and tongs both lie at rest (`< 2.0 cm/s`, `< 0.5 rad/s`) on the table **off the griddle** (zero footprint overlap with the griddle, part centres `0–3.0 cm` above the table surface, thermometer flat within `25°` with LCD facing up).
    3. Both grippers are open (`≥ 3.0 cm` jaw aperture, tools released) and both arms are parked (`≥ 10.0 cm` clear of the griddle, patty, thermometer, and tongs, moving `< 2.0 cm/s`), held continuously for `≥ 1.0 s` (`20` steps at `20 Hz`).
  - **`0.0`:** All other outcomes (e.g., idle or picking tools without rim insertion).

**Real-trial measurement (calibrated overhead camera, griddle-level side camera, LCD readout, and ruler):**
1. **Overhead camera** (calibrated with a `2 cm` scale strip beside the griddle) + **griddle-level side camera** (perpendicular to probe entry, with a vertical mm scale card): during every frame of the `2.0 s` hold, track the **probe-shaft base** where the `109.0 mm` rigid stainless shaft exits the front nose of the yellow Thermapen ONE housing (`L = 109.0 mm` from the sensing tip — always `≥ 48 mm` outside the `105 mm` patty and never obscured, unlike the `52.5 mm` visual guide band which sits near the rim). Measure the patty's `4 mm` white top-centre dot `(x_c, y_c)`, shaft-base plan coordinates `(x_s, y_s)`, and horizontal shaft unit vector `(u_x, u_y)` from the overhead view, and measure that same shaft base's height `z_s` above the griddle face and signed shaft pitch `θ` (`|θ| ≤ 20°`, positive tip-up) from the side view. Reconstruct the hidden sensing tip's exact 3D position directly from `(x_s, y_s, z_s)`:
   - Horizontal plan coordinates: `(x_tip, y_tip) = (x_s, y_s) + (109.0 mm · cos θ) · (u_x, u_y)`, enforcing the **combined Euclidean horizontal offset** `r_xy = √((x_tip − x_c)² + (y_tip − y_c)²) ≤ 8.0 mm` directly (not separate per-axis bounds).
   - Vertical height above griddle: `z_tip = z_s + 109.0 mm · sin(θ)`, enforcing `|z_tip − 8.0 mm| ≤ 4.0 mm` (`z_tip ∈ [4.0, 12.0] mm`) directly.
2. **LCD & contact check (video):** during the same continuous `2.0 s` window, confirm the Thermapen ONE LCD displays a stabilised chilled core reading `≤ 8.0 °C` (`46 °F`, changing `≤ 1.5 °C/s`), neither bare gripper comes within `6 mm` of the patty, and the held tongs clamp both sides of the patty (`< 6 mm` per jaw) throughout probing.
3. **Post-trial ruler & tape check:** lift the patty to verify a single entry hole on the side rim (`4–12 mm` above the bottom face) and **zero puncture marks** on the top face, underside, or far rim; measure patty centre drift from `(30.0, 0.0)` (`≤ 2.0 cm`), probe clearance (`≥ 2.0 cm`), both tools resting flat on the table off the griddle (`LCD` up, `0–3.0 cm` part-centre height), jaw openings (`≥ 3.0 cm`), and `≥ 10.0 cm` stationary (`< 2.0 cm/s`) gripper clearance held over the final `≥ 1.0 s`.

## Requirements and running a trial

Requires `yam-isaac-lab-arena` on Isaac Lab-Arena `0.3.1` (`inspect-robots` with `yam_arena`). No third-party mesh assets are used; all geometry is procedural USD primitives.

```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 900 \
  --instruction "Take the burger patty's centre temperature with the thermometer, probe in through its side, then put the thermometer and tongs down."
```

Runtime is ~30–60 s per 900-step trial (`45 s` simulated time at `20 Hz`); start layout is deterministic with `object_jitter=0`.

## Demo video and simulation fidelity

The demonstration video (`590` steps / `29.5 s` at `20 Hz`, within the `900`-step / `45 s` run limit; final score `1.0`, ending with `1.5 s` stationary parked hold `> 20 cm` clear of all items and both tools resting off the griddle) is produced by a scripted policy planning from simulator object poses and robot joint states (using only joint commands, with no object teleports or simulation edits). Shortcut policies verified against `score()`: idle (`0.0`), one-arm probing without tongs (`0.0`, patty slides on the oiled iron and lacks tong clamping), bare-gripper patty pinning (`0.0`), top-face probing (`0.0`), shallow `2.5 cm` rim insertion (`0.2`), brief `< 2.0 s` centre touch (`0.5`), and holding tools/probe at the end (`0.7`).

**Simulation fidelity notes:**
- The cast-iron griddle is fixed and unheated. The patty is a rigid `100 g` cylinder (`10.5 × 1.6 cm`) with friction `0.20` against the oiled griddle and `0.95` against the scalloped stainless tong jaws.
- Direct collision between the thermometer and patty is disabled so the rigid shaft can enter the rigid disc: while the sensing tip is inside the patty, a D6 joint at the entry point applies axial damping capped at `2.5 N` (sliding penetration resistance, equal and opposite on patty and thermometer) plus `1,500 N/m` lateral springs and `0.6 N·m/rad` angular springs representing the cooked meat supporting the shaft.
- A first-order tip thermistor (`τ = 0.40 s`) wakes on thermometer pickup (`22 °C` room air) and cools toward the `4 °C` chilled patty interior while the tip is inside, updating the 7-segment LCD digits on the thermometer body.
- As a simulation stand-in for nitrile-pad friction against narrow smooth tool handles in rigid-body PhysX, a grasp-assist fixed constraint engages only on two-pad contact (both `tip_left` and `tip_right` registering positive PhysX contact force `> 0.02 N` against the thermometer body or tongs while the jaws are commanded closed) and releases immediately when the jaws open; while the tongs are held, their two symmetric spring-hinged arms track the commanded jaw squeeze so squeezing the gripper physically clamps both scalloped paddles onto the patty's rim.
