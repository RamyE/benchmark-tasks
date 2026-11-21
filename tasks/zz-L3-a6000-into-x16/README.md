# Install a 1.2 kg RTX A6000 in a workstation: carry by both ends, seat in the x16 slot until latched

Instruction: `Install the graphics card in the top x16 slot of the workstation, then let go.`

Building and servicing deep-learning workstations and GPU servers requires installing full-length dual-slot accelerators
into motherboard PCIe x16 slots inside a chassis. Because an NVIDIA RTX A6000 is 266.7 mm long and weighs 1.18 kg (above
a single YAM arm's ~0.9 kg payload at reach), both arms must grasp opposite ends of its upper edge, lift it clear of the
CPU cooler and populated DDR4 DIMMs inside the open chassis, simultaneously register the stainless I/O bracket's bottom
tab into the rear-wall pocket and the keyed 89.0 mm gold-finger edge into the top PCIe x16 slot, and press both ends
down evenly against connector contact resistance until the retention latch snaps closed into the card's rear hook.
Pressing only one end cants the card so the far end stays unseated and unlatched.

## Requirements & Assets

- `yam-isaac-lab-arena` and `Isaac Lab-Arena 0.3.1` with `inspect-robots`.
- Third-party assets: none. All geometry is generated procedurally from USD primitives in `task.py`.

## Run a Trial

```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 600 \
  --instruction "Install the graphics card in the top x16 slot of the workstation, then let go."
```

Run limit: `600` steps (`30 s` at `20 Hz`). Seeds: layout is deterministic with `object_jitter=0` (`VARIATION` defines
optional `+-0.8 cm x`, `+-1.2 cm y`, `+-0.04 rad` perturbation ranges for downstream evaluation).

## Reproducible Scene Geometry

Origin `(0, 0)` on the station table (`z = 0`, `74.75 cm` above floor) at the midpoint between the arm bases
`(0, +-30.5 cm)`; `+x` forward, `+y` left, `+z` up above the table. In physical trials the workstation is completely
**unplugged and unpowered** (no mains cord attached; standard ESD wrist strap / mat practice applies).

| Object | Product / Low-Cost Real-Trial Specification | Dimensions & Material | Exact Pose (cm, table frame) |
|---|---|---|---|
| Workstation case | Fractal Design Define 7 Compact mid-tower, unplugged, lying horizontally on its right side panel; left side panel, snap-off top panel/fan bracket (Builder's Guide open-access mode), and rear PCIe covers 1 & 2 removed | `427 (y) x 210 (z) x 474 (x) mm` matte charcoal steel (`454 mm` body + `20 mm` feet at `+x`, `4 mm` wall thickness in sim) | Body `x 16.5..61.9` (feet to `63.9`), `y -19.5..+23.2`; rear/front/bottom walls `z 21.0`, near top frame rail `z 4.0`; PSU shroud `x 51.7..61.3`, `z 0.4..16.4` |
| Rear slot opening, ledge & tab pocket | Dual-slot PCIe cutout for Slots 1-2 in rear wall (`y -19.5..-19.1`) | Opening `x 32.8..37.26`, `z 2.6..14.0`; steel bracket ledge (`44.6 (x) x 12.8 (y) x 6.0 (z) mm` lip); tab pocket `15.0 (x) x 4.4 (y) mm` window | Ledge top at `z 14.00` (`114.0 mm` above board); tab pocket centred at `(34.00, -17.71)`, mouth at `z 3.20` |
| Motherboard | ASUS Prime B550-PLUS (or any standard ATX board with x16 in Slot 1), unpowered, horizontal on `6.5 mm` standoffs over `z 1.79` tray | `305 (x) x 244 (y) x 1.6 mm` FR-4 fibreglass PCB with aluminium VRM/PCH heatsinks and rear I/O shield | `x 17.9..48.4`, `y -17.7..+6.7`, top surface `z 2.60` |
| Prohibited board components | CPU pump/cooler block, 4 populated DDR4 DIMMs, and lower PCIe x16 Slot 3 | CPU block `75 x 75 x 48 mm` aluminium/ABS; each DIMM `133.4 x 4.5 x 38 mm`; Slot 3 CEM x16 LCP housing | CPU centred `(24.5, -10.5)`, top `z 7.40`; DIMMs `x 18.4..31.74` at `y -2.4, -1.4, -0.4, +0.6`, top `z 6.40`; Slot 3 at `x 38.064` |
| Top PCIe x16 Slot 1 & latch | PCI-SIG CEM 164-pin x16 connector + winged white retention latch | Housing `7.6 mm` wide (`3.4 mm` chamfered cavity), `11.25 mm` tall; cavity `y -13.75..-4.45`, key ridge (`0.9 mm`) at `y -12.29`; `22 mm` latch wing | Centred on `x 34.00`; mouth lip `z 3.725`, seating floor `z 2.925` (`8.0 mm` depth); latch pivot `(34.00, -3.99, 3.00)`, initially open `38 deg` toward `+y` |
| ESD foam cradle | Slotted closed-cell black ESD polyethylene foam block | `76 (x) x 252 (y) x 58 (z) mm` overall, `39.4 mm` wide channel, `z 2.80 cm` support shoulders with finger/tab relief cutouts | Centred at `(12.00, -1.50)`, base on table (`z 0..5.80`) |
| Graphics card | NVIDIA RTX A6000 (`1.18 kg`), **or low-cost real-trial equivalent** (below) | `266.7 (y) x 111.15 (z) x 38.0 (x) mm` (`102.65 mm` shroud + `8.5 mm` gold fingers; `89.0 mm` keyed x16 edge + `8 mm` rear hook; `40.64 mm` dual-slot steel bracket with `12 mm` top flange & `13 mm` bottom tab), `1.18 kg` | Upright connector-down in cradle at `(12.00, -1.50)`, shroud resting on `z 2.80` shoulders (top edge `z 13.09`, lowest tab tip `z 1.52`), bracket toward `-y`, PCB `1.40 cm` toward `-x` of body centre |

### Affordable Repeatable Real-Trial Hardware (~$220 Total)

An operational RTX A6000 is unnecessary for physical evaluation. Any **unpowered / nonfunctional / for-parts full-height,
full-length (`10.5 in x 4.4 in`, `266.7 x 111.15 x 38 mm`) dual-slot NVIDIA blower card** sharing the identical PCI-SIG
CEM x16 edge connector, rear retention hook, and dual-slot bracket—such as a for-parts **NVIDIA Quadro M6000 / P5000** or
**GTX 1080 Ti / Titan X Founders Edition** (~$45-$80 on secondary markets), trimmed to `1.18 kg` (`+-0.03 kg`) with
adhesive steel wheel weights along the shroud spine—preserves the exact mass, centre of mass, grasp geometry, bracket
registration, connector insertion force, and latch mechanics. Paired with an unpowered ATX motherboard (~$85), open ATX
mid-tower (~$90), and cut PE foam block (~$5), the entire rig is inexpensive and safe for repeated trials.

## Scoring & Observable Real-Trial Verification

`score()` evaluates deterministic geometry at every step (latching whole-run disqualifiers) and scores cumulative
progress at episode end (`SCORE_OVER = "end"`):

- **Score `0.0` (whole-run disqualifiers, latched if triggered at any step)**:
  1. **Single-gripper lift**: card lifted clear of the cradle (`lowest point > 2.20 cm` above table while neither finger
     end is inside Slot 1) with only 1 gripper holding it for `> 0.5 s` (`> 10` steps) — verified from video.
  2. **Drop or off-slot impact**: card touches the table or case floor (`lowest point < 1.00 cm` above table), enters
     lower PCIe Slot 3, or contacts the CPU cooler block, DDR4 DIMMs, or bare motherboard outside Slot 1 — verified from
     overhead/wrist video and component inspection.
  3. **Chassis displacement (real trial)**: moving the workstation case `> 2.0 cm` from its marked outline (fixed in sim).
- **Cumulative partial-credit tiers at the end of the trial**:
  - **`0.3` (Bracket & keyed connector registered)**: Bracket bottom tab inside the rear-wall tab pocket (`tab tip <= 2.90 cm`
    above table within the pocket) and both ends of the `89.0 mm` gold-finger edge `>= 1.0 mm` below Slot 1's `3.725 cm`
    mouth lip (`finger bottom <= 3.625 cm`, within cavity width and `<= 5 deg` yaw). *Real-trial check*: end/overhead
    photos confirm the bracket tab is inside the rear slit and both ends of the gold-finger strip sit inside Slot 1's
    mouth with the key aligned.
  - **`0.6` (Fully seated & level)**: Cumulative with `0.3`, plus both ends of the gold-finger strip seated `>= 7.0 mm`
    into the `8.0 mm` slot (`<= 1.0 mm` from the `2.925 cm` slot floor, leaving **`<= 1.5 mm` of the `8.5 mm` gold
    fingers visible above the slot lip**; bracket top flange within `1.5 mm` of the `14.00 cm` rear ledge), `|dy| <= 1.5 mm`
    along the slot, and card tilt & yaw `<= 1.5 deg` relative to the motherboard normal. *Real-trial check*: macro side
    photo or `1.5 mm` feeler gauge verifies `<= 1.5 mm` exposed gold finger height at both ends and flange-to-ledge gap;
    a digital inclinometer zeroed on the motherboard PCB reads `<= 1.5 deg` on the card's upper edge.
  - **`0.85` (Retention latch closed)**: Cumulative with `0.6`, plus Slot 1's white retention latch wing swung upright
    (`<= 5 deg` from closed) into the card's rear hook. *Real-trial check*: close-up photo shows the white wing upright
    against the hook notch and retaining the card hands-free.
  - **`1.0` (Released & parked clear)**: Cumulative with `0.85`, plus both grips released with jaws open (`>= 20 mm`
    aperture), both TCPs `>= 10.0 cm` from the card's bounding box, and both arms stationary (`<= 1.0 cm` TCP motion
    over the final `2.0 s` / `40` steps). *Real-trial check*: final `2.0 s` of video + bench ruler/grid verify open,
    stationary jaws `>= 10 cm` clear of the card.

## Simulation Fidelity & Force Calibration

- **Connector insertion force**: Standard 164-contact (`82` contact-pair) PCIe x16 connectors specify `1.15 N` max
  insertion force per contact pair (`94.3 N` spec ceiling; `~30-45 N` typical lubricated gold-finger insertion force in
  PCI-SIG CEM / Amphenol / Molex datasheets). Simulation applies `36 N` total vertical dry sliding resistance (`18 N` at
  each end of the `89.0 mm` finger strip) while inside Slot 1—matching real lubricated x16 hardware and comfortably
  achievable when both YAM arms press down together (~20-25 N downward capacity per arm at `x = 35 cm`), while resisting
  single-end or light seating.
- **Disclosed simulation stand-ins**:
  1. *Force-limited pad grip*: the grip engages only on two-pad contact: jaws commanded closed onto the card's upper-edge
     grip band (`top 36 mm`) and stalled on its `38 mm` thickness (finger gap `41-45.5 mm`). It is then a compliant
     6-DOF spring-damper joint at the pad midpoint (`20 kN/m`, `50 N m/rad`), anchored at the current relative pose (no
     snap), with every axis capped at what friction pads can hold: force `50 N` = 2 pads x mu `0.5` x `50 N` squeeze
     (the YAM gripper actuator's `50 N` effort limit); twist about the pad normal `0.5 N m` (`10 mm` mean radius of a
     `~20 x 30 mm` pad patch); tipping in the pad plane `1.5 N m` (`50 N` x `15 mm` half pad length, 2 pads). When the
     load exceeds a cap the card slips to a new place in the jaws (the anchor moves) instead of being held. The card
     (`11.6 N` weight, `18 N` per end of connector resistance) stays within these limits; the demo shows a few short slips
     only while breaking the card out of its foam cradle. Opening the jaws releases the grip at once. The pads' own
     contact friction also acts.
  2. *Retention latch*: Slot 1's latch wing progressively rotates as the card's rear hook descends and snaps to `0 deg`
     with a fixed-joint lock once both ends reach `>= 7.0 mm` seating depth within `1.5 deg` level.
  3. *Omitted details*: spinning fan blades, PCIe auxiliary power cables, and the post-insertion rear bracket thumbscrew
     are not modelled. The steel bracket plate, top flange and bottom tab are all collision bodies, so the bracket must
     pass the rear-wall opening and ledge physically.

## Demo Video & Shortcut Checks

A scripted bimanual policy (reading card pose from simulator state, executing physical joint-space motions) grasps both
ends of the card's upper edge (`17.5 cm` apart), lifts the `1.18 kg` card out of the foam cradle and clear of the `7.4 cm`
CPU cooler and `6.4 cm` DDR4 DIMMs, aligns and registers the bracket tab and keyed fingers at Slot 1, presses both ends
home evenly until the white latch snaps shut, then opens both grippers and parks clear to score `1.0`. Verified shortcut policies—idle arms
(`0.0`), single-arm lift and insertion (`0.0`), inserting into lower Slot 3 (`0.0`), pressing only one end so the card
tilts (`0.0`-`0.3`: the unseated card can tip over once released), letting go halfway into Slot 1 so the card tips onto the board (`0.0`), and seating without releasing (`0.85`)—all score strictly below `1.0`.

## Sources

- PCI-SIG, *PCI Express Card Electromechanical Specification* (x16 89 mm edge connector, key datum, 111.15 x 266.7 mm
  full-height card, 20.32 mm slot pitch, 1.15 N/pair max mating force).
- NVIDIA, *RTX A6000 Datasheet* (`4.4 in x 10.5 in` / `111.15 x 266.7 mm`, dual-slot, `1.18 kg`).
- Fractal Design, *Define 7 Compact Manual* (`427 x 210 x 474 mm` ATX chassis).
- Dell Precision & HP Z Workstation Service Manuals ("Installing a PCIe expansion / graphics card").
