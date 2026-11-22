# Swap a failed QSFP28 optic: unplug its fibre first, open the bail and pull it, seat the spare

A top-of-rack switch reports that the optical transceiver in port 5 has failed. Replacing it is a routine field swap
performed across AI training clusters every week: draw the duplex LC uniboot connector's push-pull release tab back to
unplug the fibre patch cord first, hold the unplugged connector clear so its exposed ferrule end face touches nothing
(including either gripper), swing the failed module's wire bail open past 80 deg to release the cage latch, slide the
failed module out into the antistatic ESD tray, seat the spare module straight into port 5 until its cage latch clicks,
and mate the LC connector back into the spare until it latches.

Instruction: `The optic in port 5 has failed. Replace it with the spare from the tray, put the failed one in the tray, and reconnect its fibre.`

Neither the extraction order nor the clean-end-face constraint is spelled out in the instruction; both are standard
fibre-optic maintenance practice that the robot must apply on its own.

## Why it matters (Impact on AI timelines)

- **Pluggable optics are a leading hardware fault in AI clusters.** Every high-rate GPU-to-switch and switch-to-switch
  link in modern AI clusters terminates in pluggable transceivers (QSFP28 / QSFP56 / OSFP). In Meta's *The Llama 3 Herd
  of Models* (2024, Table 5), "Network Switch/Cable" accounted for 35 of 419 unexpected training interruptions (8.4%)
  across 54 days on 16,384 H100 GPUs. Automating hot-aisle/cold-aisle transceiver swaps directly cuts cluster recovery
  time.
- **Who performs it today:** data-centre hardware and network technicians (O*NET 49-2011.00 Computer, Automated Teller,
  and Office Machine Repairers; 15-1231.00 Computer Network Support Specialists).
- **Standard vendor procedure** (Cisco *40-Gigabit and 100-Gigabit Ethernet QSFP Transceiver Modules Installation Note*;
  TIA-604-10 / IEC 61300-3-35):
  1. Disconnect the LC fibre patch cord **before** unlatching or pulling the transceiver, by drawing the connector's
     push-pull release tab straight back. Pulling a module with the fibre attached stresses the cord, connector, and
     cage; moreover, a plugged-in LC connector physically blocks the wire bail from swinging fully open.
  2. Keep the uncapped ferrule end face clear of every surface (bench, switch, tray, module housing, bail, cord, and
     fingers/grippers). Any contact contaminates the 1.25 mm zirconia ferrules and requires inspection and cleaning
     before mating.
  3. Swing the wire bail clasp open to release the cage latch and pull the failed module straight out (gripping either
     the protruding module shell or the opened bail once unlatched; opening the bail past 80 deg is mechanically
     required to free the latch, whereas pulling by the bail vs. the module nose after unlatching is unrestricted).
  4. Slide the replacement QSFP28 module (bail closed upright) straight into port 5 until it seats against the cage back
     stop and latches, then push the LC connector straight home into its receptacle until the latch clicks.

## Why two hands are required

- **Unstated ordering + mechanical interlock:** Pulling port 5's module more than 2 mm out while the LC is still in its
  receptacle scores 0, and while the LC remains plugged in, the bail crossbar hits the LC housing near -67 deg (short of
  the -80 deg cage-release threshold).
- **Continuous clean-end-face hold:** Because the LC uniboot body is 12.4 x 6.8 mm and its front ferrule shroud is
  11.0 x 6.0 mm, laying the unplugged connector down in any orientation brings its front face within <1 mm of the
  resting surface and triggers a contamination failure. One hand must hold the unplugged fibre clear in the air while
  the other hand opens the bail, extracts the failed module to the ESD tray, and seats the spare.
- **True-size crowded port pitch:** Cages sit on a 21.5 mm horizontal pitch (SFF-8663), with live corded modules two
  ports away on both sides (ports 1 and 9 at y = +-4.30 cm; moving either by more than 2 mm scores 0).

## Scene and reproducible real-trial setup

Coordinates in cm from the arm-base midpoint (`x` forward away from the arms, `y` left; heights `z` above the table top
at 74.75 cm).

- **Rack section:** the switch is installed as a top-of-rack switch in a 2U two-post 19-inch EIA-310 open-frame rack
  standing on the table. Steel posts (`3.5 x 7.6 cm` section, `450 mm` rail opening) at `y = +-22.5` to `+-26.0 cm`,
  `x` 37.0 to 44.6 cm, `9.9 cm` tall, tied by a top bar across their rear half (`x` 41.0 to 44.6 cm). **U1** (`z` 0 to
  4.36 cm) holds a 1U LC fibre patch panel with eight live blue LC uniboot plugs at `y = +-8.5, +-10.5, +-12.5, +-14.5
  cm`, `z = 2.2 cm`; **U2** (`z` 4.45 to 8.81 cm) holds the switch, the top unit, open above as in a top-of-rack
  position. Rack ears sit flush at `x = 36.6-37.0 cm`. A vertical finger cable manager (`7.6 cm` channel, fingers `10
  cm` deep reaching to `x = 30.0 cm`, at `z = 0.8` and `6.7 cm`, `9.9 cm` tall) stands beside each post (`y = +-26.0`
  to `+-33.6 cm`).
- **Live patch cords (fixed, the arms must work around them):** all ten are the same yellow `4.0 mm` OS2 single-mode
  duplex uniboot cord as port 5. Port 1's and port 9's cords leave their LC boots at `x = 27.8 cm`, drop `2.5 cm`, angle
  back to `x = 33 cm` and run straight outward at `z = 5.0 cm`, enter the left/right manager at `z = 5 cm` and run up its
  channel to the manager top. The eight patch-panel cords turn sideways right at their boots and run outward at
  `x = 31.2 cm`, `z = 2.5-3.0 cm`, into the same managers and up. Each cord is held by one hook-and-loop strap at its manager entry.
- **Switch:** An unpowered 1U 32-port QSFP28 switch (`43.8 x 26.0 x 4.36 cm`, NVIDIA/Mellanox Spectrum `MSN2700`
  class, steel chassis with `2.2 cm` rack ears) mounted in U2 (`z` 4.45 to 8.81 cm). Faceplate at `x = 37.0 cm` facing
  `-x`; upper port row centred at `z = 7.53 cm`, lower row at `z = 6.38 cm`; 16 columns at `21.5 mm` pitch. Cages (`20.4 x 10.5 mm` opening, `45.0 mm` insertion depth
  to back stop, SFF-8663) are open at upper-row ports 1 (`y = +4.30`), 5 (`y = 0.00`), and 9 (`y = -4.30`). Ports 3
  (`y = +2.15`), 7 (`y = -2.15`), the remaining 11 upper ports, and all 16 lower ports hold black rubber QSFP dust
  plugs (`18.35 x 8.5 mm`, protruding 3 mm).
- **Four QSFP28 modules (`18.35 x 8.5 x 89.4 mm`, 35 g, Finisar/Coherent `FTLC1154RDPL` 100GBASE-LR4, SFF-8661):**
  1. **Port 5 (failed):** wire bail painted red, closed upright (`0 deg`), latched home (front at `x = 41.45 cm`, rear
     at `x = 32.51 cm`, protruding `4.49 cm` outside the faceplate).
  2. **ESD tray (spare):** unpainted silver wire bail, closed upright (`0 deg`), resting label-up on a white outline at
     `(23.5, -21.3) cm` pointing `+x` inside a red rigid antistatic polypropylene ESD tray (`14.0 x 10.0 cm` inner
     floor, `0.8 cm` walls, `0.4 cm` wall thickness, centred at `(23.5, -19.0) cm`). The inner half of the tray
     (`y` around `-16.6 cm`) is empty to receive the failed module.
  3. **Ports 1 and 9 (live neighbours):** two modules (`y = +4.30` and `-4.30 cm`), bails upright, each with a blue LC
     uniboot plug latched in and its live cord dressed into the side manager (above), plus a white witness-tape stripe bridging module shell and
     faceplate bezel.
- **Patch cord on port 5:** A yellow `4.0 mm` duplex single-mode cord with a blue LC uniboot push-pull connector
  (`20.0 x 12.4 x 6.8 mm` inner body, `12.0 x 11.0 x 6.0 mm` duplex shroud with two `1.25 mm` ferrules at `6.25 mm`
  pitch per TIA-604-10, and a `26.0 x 12.4 x 7.4 mm` blue sliding push-pull release tab behind it). From the seated plug
  (`x = 29.1 cm` at the tab), `31 cm` of cord curves smoothly down to the table in front of the rack and is held at `(10.0, +6.0) cm` under
  a `2.4 x 4.0 cm` black gaffer-tape strip.

### Affordable real-trial bill of materials (~$260-$380 total)

- **Four** used/pulled 100GBASE-LR4 QSFP28 modules (e.g. Finisar `FTLC1154RDPL` or Cisco `QSFP-100G-LR4-S`, ~$15-$20
  each on surplus = ~$70 for four; one bail painted red). Switch is unpowered so no laser is energised.
- **Switch / cage fixture (two interchangeable options):** (a) a surplus/for-parts unpowered 1U 32-port QSFP28 switch
  (e.g. Mellanox `MSN2700` parts chassis, ~$120), **or** (b) a low-cost benchtop equivalent (~$40): a 1U sheet-aluminium
  panel (`43.8 x 4.36 cm` on a `26 cm` deep box) populated with standard Molex 100G QSFP28 ganged EMI cages (Molex
  `111114` series, `21.5 mm` pitch, `45 mm` depth) at ports 1, 5, and 9 and rubber dust plugs in the remaining cutouts,
  with `2.2 cm` rack ears.
- **Rack section:** a 2U two-post 19-inch open-frame desktop rack (~$40), a 1U 24-port LC fibre patch panel (~$30), two 4-inch vertical finger cable managers
  (~$30 each), ten more `1 m` OS2 duplex LC uniboot cords (~$8 each) and hook-and-loop straps.
- Two `1 m` OS2 duplex LC uniboot push-pull-tab patch cords (FS `#41655`, ~$8 each; excess behind the `(10, +6) cm`
  gaffer-tape anchor dressed off the near table edge), one `14 x 10 cm` antistatic ESD component tray (~$8), black QSFP
  dust plugs, and white witness tape. No third-party mesh or CAD assets are included in this archive; all geometry is
  procedural USD primitives.

## Scoring

**Score 0** if, at any time during the trial:
- port 5's failed module moves **more than 2 mm** out of its cage while its LC connector is still in its receptacle;
- once the LC connector leaves port 5, its front face (ferrule end) comes within **1 mm** of any surface outside a
  module's open LC receptacle — table, rack posts, rack units, patch panel, cable managers, the ten live patch cords and their plugs, switch, dust plugs, ESD tray, cord, a module's exterior shell, a bail, or
  **either robot gripper / finger pad**;
- the live module in port 1 or port 9 moves **more than 2 mm** from its starting pose;
- the failed module, once extracted (>50 mm out), re-enters any port cage.

**Otherwise** (evaluated at the end, with placed/mated parts released by the grippers):

| Score | Condition |
|---|---|
| `0.0` | Failed module still in port 5, or pulled out less than 50 mm |
| `0.25` | Failed module extracted (>50 mm out; front face clear of the cage) |
| `0.50` | Also spare module seated home and latched in port 5 (front within 1.5 mm of back stop, within 1.5 mm of port axis, bail closed within 10 deg of upright, released) |
| `0.75` | Also LC connector seated home and latched in the spare module's receptacle (within 1.5 mm of bottom, released) |
| `1.0` | Also failed module resting flat inside the ESD tray (released), and both grippers open (>=20 mm), at least 10 cm clear of every module, bail, and LC part, moving under 2 cm/s for 0.5 s |

**Real-trial measurement aids:** An overhead camera plus a close side camera aimed across the switch face and ESD tray
record the full swap alongside the two wrist cameras. Witness-tape stripes across ports 1 and 9 show >2 mm neighbour
displacement directly; video confirms the LC leaves port 5 before port 5's module moves. The end-face rule is a contact
rule in both settings: the simulation's 1 mm band only absorbs collider rounding, so "within 1 mm" there means
"touching" here. In real trials the ferrule end faces are cleaned and inspected (IEC 61300-3-35, handheld fibre
inspection probe) before the trial and inspected again at the end; any new debris or smudge, or any touch of the front
shroud seen on video, scores 0. At trial end, light manual tug checks on the spare's bail and on the LC cord verify
positive mechanical latching.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1800 \
    --instruction "The optic in port 5 has failed. Replace it with the spare from the tray, put the failed one in the tray, and reconnect its fibre."
```

Requirements: `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1. Layout is fixed (`object_jitter=0`), 1,800 steps
(90 s at 20 Hz; the scripted demonstration completes in 1,115 steps, ~56 s).

Dream-1 run on this exact task (instruction above, 1,800 steps): score `0.0` (ran to the step limit).

## Demo video

A scripted policy plans from simulator object poses and latch states without teleports or state writes. The left arm
pinches the blue LC push-pull release tab, draws it straight back along `-x` (sliding the tab >=2.5 mm to release the
LC latch before pulling the connector straight out of port 5), and holds the connector elevated and clear at `(22.0,
+18.0, 15.0) cm` throughout the module exchange. The right arm closes its fingertips to flick the red wire bail open
past `-80 deg`, pinches the protruding module shell, extracts the failed module straight out and releases it flat in the
open half of the ESD tray, then picks up the silver-bail spare, aligns it on port 5, slides it home until the cage latch
engages, releases, and parks clear. Finally, the left arm brings the LC connector in front of the spare's receptacle,
centres it on the receptacle axis from the measured poses, pushes it in short steps that also remove the measured
sideways offset (backing off 1.5 mm and re-centring if it stops on the receptacle rim), drives it home via the push-pull
tab until the LC latch clicks, opens the jaws, and parks clear for over 1 s (1,115 steps, score `1.0`).

## Shortcut checks

| Policy | Behaviour | Score |
|---|---|---|
| Full bimanual swap | Unplugs LC via push-pull tab, holds ferrule clear, opens bail, trays failed optic, seats spare, reconnects LC, parks | `1.0` |
| Idle | Arms stay still | `0.0` |
| Pull module before unplugging LC | Flicks bail (stopped near -66 deg by the plugged LC, so the cage stays latched) and pulls on the module with the LC still mated | `0.0` |
| Grab the LC body instead of the tab | Pinches the LC front body; the fingers drag the tab and unplug it, but the exposed end face is then touching the gripper | `0.0` |
| One-arm swap (set LC down) | Unplugs LC with one arm and lays it down near the rack to free that hand; its end face touches a live patch cord | `0.0` |
| Careless swing | Unplugs LC correctly, then swings the held connector toward the faceplate; its exposed end face brushes the neighbouring live patch cord | `0.0` |
| Never release the spare | Seats the spare but keeps the right gripper closed on it; while the LC is re-mated beside that gripper, its exposed end face brushes the gripper (a held part would not count as seated either) | `0.0` |
| Partial swap (stop before LC reconnect) | Extracts failed optic to tray and seats/releases spare while holding LC clear, stopping before re-mating | `0.5` |

## Sim fidelity

- **Force-limited pad grip (no rigid weld):** a 6-axis spring-damper joint between gripper and a module or LC part engages only when both finger pads touch that part (PhysX contact on each pad), the jaws are commanded closed, and the pads sit on its grip band (module nose, LC pull tab or LC body). It is anchored at the current relative pose, so nothing snaps. Each axis is force-limited to what two rubber pads can transmit: 50 N pull (2 pads x mu 0.5 x 50 N squeeze), 0.50 N m twist about the pad normal, 1.50 N m tipping (twist cap = pull cap x 10 mm mean pad-patch radius; tipping cap = 2 x 50 N x 15 mm half pad length). Above a cap the part slips in the jaws. It releases when the jaws are commanded open or both-pad contact is lost for 5 control steps. Sources: YAM gripper effort limit 50 N per finger (i2rt linear_4310 actuator); rubber pad friction mu 0.5; pad patch ~20 x 30 mm.
- **Prismatic LC push-pull release tab:** `lc_pull_tab` rides a spring-returned prismatic joint (`-4.5` to `0.0 mm`,
  `120 N/m`, `3 N s/m`) on `lc_connector`. The LC latch weld (`lc_old` / `lc_new`) releases only when `lc_pull_tab` is
  drawn back `>= 2.5 mm` relative to `lc_connector`, and engages when `lc_connector` reaches within `1.5 mm` of the
  receptacle bottom with the tab at rest. Because the fibre cord attaches to `lc_connector` (inner strain relief),
  pulling the cord or gripping `lc_connector` directly cannot unlatch it.
- **Bail hinge and cage latch:** Each wire bail turns on a revolute joint (`0` to `-90 deg`) with an over-centre spring
  (`0.0006 N m/deg`, snapping closed above `-45 deg` and open below). With the LC mated, the bail crossbar strikes the
  LC latch clip near `-67 deg`, preventing it from reaching the `-80 deg` cage-unlatch threshold. Pushing an unlatched
  module within `1.5 mm` of `SEAT_X` and `1.5 mm` of the port axis with its bail within `10 deg` of closed engages the
  cage weld.
- **Live neighbours and cord:** Ports 1 and 9 are rigid compound bodies held by welds that break above `15 N` or
  `0.6 N m`. The rack, its units, the managers and the ten live patch cords are fixed colliders (the live cords are
  static capsule chains on their stated routes, so pushing on them does not deflect them). The `30 cm` cord is 15 ball-jointed `2 cm` rigid capsules (`4 mm` dia, `45 g/m`, bend stiffness
  `0.0025 N m/deg`) anchored under the fixed tape strip at `(10, 6) cm`. Switch, rack, and ESD tray are fixed;
  optical transmission is not modelled.
- **End-face contamination check:** Evaluated every control step across 9 points on `lc_connector`'s front face against
  all static parts (including the rack, managers and live patch cords), modules (outside open receptacle mouths), bails, cord capsules, and all URDF collision boxes of both
  robot grippers (`1 mm` threshold).

## Sources

- Meta AI, *The Llama 3 Herd of Models* (2024), Table 5 (35/419 = 8.4% switch/cable interruptions on 16,384 GPUs):
  https://arxiv.org/abs/2407.21783
- Cisco, *40-Gigabit and 100-Gigabit Ethernet QSFP Transceiver Modules Installation Note* (disconnect fibre cable before
  releasing bail clasp; protect optical end faces; seat until latched).
- SNIA SFF-8661 (QSFP+ 28 Gb/s 4X Pluggable Transceiver Solution, 18.35 x 8.5 mm module mechanical spec) and SFF-8663
  (QSFP+ 28 Gb/s Cage / Connector system, 21.5 mm ganged pitch, ~43-45 mm insertion depth).
- Coherent / Finisar `FTLC1154RDPL` 100GBASE-LR4 QSFP28 Optical Transceiver Datasheet (89.4 mm overall length, wire
  bail, duplex LC receptacle).
- TIA-604-10 (FOCIS-10) LC Fiber Optic Connector Intermateability Standard & IEC 61300-3-35 end-face cleanliness.
- O*NET 49-2011.00 and 15-1231.00.
