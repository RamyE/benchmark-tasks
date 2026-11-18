# Flex-test cable assemblies on a continuity tester and reject the one with an intermittent break

Three identical cable assemblies (leads a, b and c) are plugged into a bench continuity tester. Each lead's connector
plug sits in a seat in front of the tester, its cable runs across the table, and the ring terminal at its far end
lies on a brass return bar. Each lead has a green continuity lamp on the tester. One lead has an intermittent open at
the strain relief next to its plug. Its lamp stays lit while the lead lies still and goes dark only while that part
of the cable is bent. Which lead is faulty is drawn at random for every trial, and nothing in the scene shows it. The
robot has to make the fault appear: flex leads near their plugs and watch the lamps until it knows which lead is
faulty, take that lead out of the tester by its plug, and put it in the red reject tray. The good leads must stay
connected throughout.

The tested ability is interactive perception. The faulty lead can only be found by deforming a cable in the right
place and reading the lamp, and then the robot has to act on what it saw. A lamp that goes dark while its lead is
flexed identifies the fault at once. Two leads that stay lit when flexed also identify it (the third lead), so
the robot does not have to test all three before rejecting. Rejecting a lead before the lamps have shown which one is
faulty scores 0; a completed diagnosis that is not acted on earns partial credit.

Instruction: `One of these three cable assemblies on the continuity tester has an intermittent break. Find it and put it in the red reject tray.`

## What changed in this revision

Review feedback, verbatim: "Importance (not met): Inspectors and electrical assemblers perform broad testing work,
but the submission does not establish that 500,000 workers regularly flex-test cable assemblies for intermittent
faults. Scoring (not met): A correct diagnosis demonstrated by testing earns zero unless the robot also unplugs the
lead, reopening the earlier feedback on diagnostic progress." What to change: "award partial credit for verified
diagnostic progress, including a completed diagnosis before the faulty lead is unplugged" and "Please substantiate
that this specific operation meets the Impact on jobs threshold, or use Other and explain evidence of comparable
impact."

- **Scoring.** A completed diagnosis alone now earns credit, without unplugging anything: 0.25 once the lamps have
  shown which lead is faulty (its lamp went dark while it was flexed, or both good leads were flexed with their
  lamps lit), with no lead unplugged. Acting on the diagnosis earns more: 0.5 for unplugging the faulty lead, 0.75 for taking it
  out of the tester, 1.0 for putting it in the reject tray. New variants `diagstop` (tests until the lamps identify the
  faulty lead, then stops) and `diagnose` (tests all three, then stops): 0.25. The variant modes formerly named `tier025` and `tier05` are now `unplug` and `outside`.
- **Category: Other** (next section), with sourced evidence of what intermittent wiring faults that escape testing
  cost; the occupation head counts are no longer the case.
- Re-records the demo and reruns the shortcut variants on this build (table below).

## Why this matters (category: Other)

The latest review found that the occupation counts do not show 500,000 workers who flex-test cable assemblies, and
offered Other instead: "Please substantiate that this specific operation meets the Impact on jobs threshold, or use
Other and explain evidence of comparable impact." We use Other. The impact is not workers' time but what an
intermittent open costs when it escapes a test: it passes a still continuity check, ships, and comes back as a
field failure that the next technician cannot reproduce, a "no fault found" removal.

- **Military aircraft.** A US Navy study (Muja, NAVAIR, and Lamper, SAE Int. J. Aerosp. 5(2):579-589, 2012, paper
  2012-01-2241): "Wiring/conductive path faults inside Weapon Replaceable Assemblies (WRAs) that result in No Fault
  Found (NFF) or intermittent diagnosis are estimated to cost the DoD over $2B annually." On the F/A-18 radar receiver
  chassis it tested, "93% of all RR chassis tested had latent wiring faults"; of the faults found, "74% were
  diagnosed using static tests and an additional 26% were diagnosed during dynamic testing", that is, only when the
  unit was moved (vibrated).
- **Airlines and other industries.** A review of no-fault-found events (Khan, Phillips, Jennions and Hockley,
  Reliability Engineering & System Safety 123, 2014) reports: "Figures published by the Air Transport Association
  (ATA) in 1997 estimated annual NFF costs for an airline operating 200 aircraft at $20M, or $100,000 per aircraft
  per year", and that NAVAIR "spent $94M on NFF equipment removals due to wiring issues with unconfirmed faults". On
  the cause: "In electronic systems, loose connections probably cover most of the fault cases that have been
  published as NFF", and it lists "Intermittent system connection (e.g. connector integrity, harness/loom
  integrity)" among the root causes.

The fault is the standard one in cable testing: the cable-tester maker Cirris defines it in its glossary as
"Intermittent open: The intended electrical path contains an unintended gap that appears and disappears when the
assembly is flexed." The remedy these sources point to is the one this task scores: move the assembly while its
circuit is watched, so the gap shows before the part leaves the bench. Testing and rejecting finished assemblies is
listed as Core O*NET work for inspectors and testers (51-9061.00) and for electrical and electronic equipment
assemblers (51-2022.00); we no longer rest the case on their head count.

## How this differs from similar tasks

- Tasks that plug a cord into a cable tester score the connection. Here the leads are already connected and the
  plugging is not scored. The core act is the test itself: flex each lead near its plug while watching its lamp on
  the tester, because the intermittent fault exists only while a cable is bent, then set the lead that failed aside
  in the reject tray.
- Tasks that map or check the wiring of a harness read a static pass/fail. Here every lead passes while it lies
  still; only the flex reveals the bad one.
- Tasks that sort parts by a measurement or a visible label judge parts by a reading that is there from the start.
  Here all three leads look the same, and the faulty one changes from trial to trial, until a lead is flexed.

## Closest tasks

- A cable-tester task scores both plugs of a patch cord pushed into a cable tester; this task scores finding, by flexing
  connected leads at their strain reliefs, which one has a hidden open, and removing only that one.
- A gauging task scores bars sorted into GO and NO-GO boxes from caliper readings; this task scores a fault that no static
  reading shows, revealed only by bending the cable while watching its lamp.
- A meter task scores a rigid test plug inserted at a meter test switch; this task scores which of three leads fails
  only while bent.

## Why two arms (reach)

Offline IK, with the gripper pointing down:

- Only the left arm reaches the connector seats at y +18 cm. The right arm misses them by 23 to 68 mm (lead a 23 mm,
  b 42 mm, c 68 mm).
- Only the right arm reaches the reject tray. The left arm misses even the tray's nearest inner corner by about
  4 cm.
- So the left arm takes the faulty plug out of its seat and the right arm puts the lead in the tray.
- Pulling a lead out of the tester by its cable is a scored violation (score 0): the plug must leave its seat
  carried by a gripper (see Scoring). Cable assemblies are unplugged by the connector body, because yanking the
  cable is what breaks them at the strain relief. The right arm's TCP gets no closer than about 42 mm to lead a's
  plug centre (IK: it can reach a grasp on the cable no further than y +13.8 cm, while the plug centre is at y +18
  cm). So a right-arm-only pull always trips this rule (the `cordpull` variant below). The left arm's pinch holds
  its TCP about 10 mm above the plug centre.

Flexing a lead does not need its plug held. In this rig the seat rails hold an unheld plug within 1 to 2 mm while the
cable is flexed, so the right arm flexes a lead on its own. During each test the left arm hovers over the lead's plug,
and its wrist camera shows that lead's lamp, 26 mm across, filling much of the frame. The lamps are best read in that
wrist view, where a dark lamp is plain at 320x176. From the top camera the tester is small and partly hidden by the
left arm.

## Real-world build

The simulated rig is a bench continuity tester for cable assemblies; this is how to build it for real trials.

- **Tester body:** a 24 x 6 x 3 cm block (printed PLA or machined acetal), fixed to the table, with three seats in
  front of it 6 cm apart. Each seat is a pair of rails 30 x 4 x 3 mm, 23 mm apart inside, so the plug (30 x 20 x 15
  mm) slides in from the front and stops against the tester face.
- **Seat contact:** a gold-plated 1.5 mm pin sticks 4 mm out of the tester face at the centre of each seat, at the
  height of the plug's pin socket. Pushing the plug against the face slides the pin into its socket. Pulled back by
  more than about 4 mm, or tipped up out of the rails, the plug loses contact.
- **Lamp circuit (one per lead):** 3 V (two AA cells, shared) -> 100 ohm resistor -> 5 mm green LED under a 26 mm
  diffuser cap on the tester top -> seat pin -> lead plug -> lead conductor -> ring terminal -> brass return bar ->
  battery minus. The return bar is the common return: a brass plate 30 x 17 cm, 2 mm thick, with three phosphor-bronze
  leaf springs that press each ring terminal flat against the bar; a terminal slides out from under its spring when
  the lead is lifted away. A lamp is lit exactly while its lead conducts from seat to bar.
- **Leads (cable assemblies):** three identical leads, grey 5 mm jacketed single-conductor cable, 28 cm long, each
  with a 30 x 20 x 15 mm plastic plug (one socket contact) at one end and a blue insulated crimp ring terminal at the
  other. All three carry an identical grey 30 mm heat-shrink sleeve over the cable 4.5 to 7.5 cm from the plug, so the
  faulty one cannot be told apart by eye.
- **Reproducible intermittent break (one lead):** a spring-contact joint spliced into the conductor 6 cm from the
  plug, under the sleeve. Parts, all ordinary stock:
  - a spring contact probe (pogo pin) with a 1 mm barrel, 10 to 17 mm long, at least 1 mm plunger travel, solder tail;
  - a 1.5 mm brass rod, 5 mm long (the contact disc), one end filed flat;
  - an insulating collar: PTFE tube (or printed PLA) 3 mm outside, 1.5 mm inside, 5 mm long, pressed over the rod so
    the rod's flat face is flush with the collar's end;
  - soft silicone tube, 4 mm inside, 6 mm outside, cut to (probe length + 7 mm);
  - heat-shrink: 30 mm of grey 8 mm heat-shrink as the sleeve.

  Assembly (side view, cable axis left to right, not to scale):

  ```
   plug side                     silicone tube (4 mm ID)                       bar side
   ====cable====[solder]|=== pogo pin barrel ===>|tip   [collar|brass rod]|[solder]====cable====
                        <------- probe, 10-17 mm ------>  <- 5 mm ->
                                                 ^ tip presses on the rod's flat face, plunger
                                                   about half compressed with the cable straight
  ```

  Cut the conductor 6 cm from the plug, solder one end to the probe's tail and the other to the brass rod's free end,
  slide both into the silicone tube from either side until the probe tip rests on the centre of the rod face with the
  plunger about half compressed, and cover the joint with the 30 mm sleeve (centred 6 cm from the plug). With the
  cable straight the circuit is closed. Bending it at the joint shifts the probe tip sideways across the rod face; past
  about 30 degrees, in any direction, the tip leaves the 1.5 mm face for the insulating collar and the circuit opens,
  and straightening it closes it again. Tune the threshold before the first trial by sliding the probe and the rod
  along the tube (where the joint sits relative to the bend changes the angle at which the tip leaves the face) until
  the bend-jig check in step 4 below passes.
- **Good leads:** the same plug, cable, ring terminal and the same 30 mm grey sleeve at the same place, with an unbroken
  conductor underneath, so all three leads look alike.
- **Reject tray:** red, 22 x 28 cm inside, 15 mm walls.

### Reset and verification (every trial)

1. Draw the faulty seat from the trial seed: lead `["a", "b", "c"][numpy.random.default_rng(seed).integers(3)]`, the
   same draw the simulation uses (a, b, c are the seats from nearest to farthest from the robot). The faulty lead
   carries a small dot inside its ring terminal's insulating sleeve, which the cameras cannot see; only the operator
   uses it.
2. Put the faulty lead in the drawn seat and the two good leads in the others. Push each plug into its seat against
   the tester face, square to it, lay each cable straight towards the robot and slide its ring terminal under its leaf
   spring on the bar.
3. Check all three lamps are lit with the cables lying still.
4. Verify the break with a bend jig (a protractor card at the sleeve): the faulty lead's lamp stays lit at 15 degrees
   and goes dark at 35 degrees, bent up, down, left and right. A good lead's lamp stays lit at 45 degrees. If not,
   adjust or replace the faulty lead before the trial.
5. After the trial, take the rejected lead out of the tray and repeat from step 1.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the simulated cables) is in this folder next to `task.py`, and `cabledemo.py` runs the scripted
  demo.

## Assets

No third-party assets: every object (tester, seats, lamps, return bar, leads, tray) is built from primitive shapes in
`task.py`, and the cables come from `cablekit.py` in this folder.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "One of these three cable assemblies on the continuity tester has an intermittent break. Find it and put it in the red reject tray."
```

The faulty lead is drawn on every embodiment reset, uniformly over the three leads, from the seed the runner passes to
`reset()` (each trial and epoch gets its own seed, so repeats vary and a given seed reproduces). Nothing visible
changes with it. Without a seed (the local demo runner), `WIGGLE_SEED` seeds the draw, or fresh entropy is used. The
scorer keeps a short history of the trial (lamps, tests, how plugs left their seats), and `task.py` clears it on
every reset. The layout is fixed (object_jitter=0, and cables are never jittered).

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

The demo does not know the faulty lead. It chooses its motions from the fixed layout, the arms' own poses and the
left wrist camera image; the simulator's object state is used only for the debug log.

1. Test a lead: the left arm hovers over its plug, with the wrist camera on the lead's lamp. The right arm takes the
   cable 20 cm from the plug and pushes it 5 cm towards the tester while lifting it 5 cm. The cable humps and bends
   about 40 degrees where it leaves the plug. The script measures the share of bright green pixels in the wrist image
   before and during the flex; below 30% of the starting share counts as dark. The cable is laid back.
2. Leads are tested in the order a, b, c and testing stops as soon as a lamp went dark (that lead is faulty) or two
   leads stayed lit (the third is faulty, untested).
3. Reject: the left arm lifts the faulty plug 2 cm up out of its seat, draws it 3 cm back and holds it. The right arm
   takes the cable near the middle of the lead, lifts it, the left arm lets go, and the right arm carries the lead
   hanging from that point over the middle of the tray, waits for it to stop swinging, lowers it slowly to the tray
   floor and lets go. Both arms park.

The recorded demo has the fault on lead a: lamp a goes dark on the first test, and it reaches `task_score` 1.0 in
656 steps (33 s at 20 Hz). With the fault on b or c the same script tests a then b, and rejects c untested when both
stay lit (run with the fault on c: 1.0, table below).

## Scoring (`score()`; `success()` = score 1.0 with every lead at rest, held 1 s)

"Evidence" means the lamps have shown which lead is faulty, in one of two ways, each while the lead is connected
(its plug seated and its ring terminal on the return bar, so its lamp can show the result):

- the faulty lead's lamp went dark while it was flexed (its break opens past about 30 degrees of bend); or
- both good leads had their cable swung sideways past 30 degrees (cable 5 to 7 cm from the plug against the plug's
  axis, seen from above) and their lamps stayed lit, so only the third lead can be faulty.

Judging this in a real trial needs nothing beyond what a camera plainly shows. A camera straight above the tester
shows the lamps, which plug is in its seat, which ring terminal is on the bar, and which lead a gripper holds. The
deck carries two lines printed at 30 degrees either side of each cable exit, 5 to 7 cm long, so in the same top view
a swung cable either crosses a line or it does not; the camera looks straight down on the deck plane in which these
lines lie. The judge notes the time of each lamp going dark, of each crossing, and
of the first plug leaving its seat, so "evidence before removal" is read straight from the video, and checks in the
same frames that the plug left its seat between a gripper's fingers (a plug that slides out while no gripper holds
it was pulled by its cable).

Score 0 if any of the following holds:

- **A good lead's lamp went dark at any moment** (latched). A lamp goes dark when its plug moves more than 4 mm from
  its seat, rises more than 5 mm, or turns more than 12 degrees out of line, or when its ring terminal leaves the
  return bar (the circuit runs from the seat through the lead to the bar). Taking a good lead out of the tester
  (rejecting the wrong lead) always does this.
- **A plug left its seat by more than 1 cm without being carried by a gripper** (latched). Carried means: at that
  moment a gripper TCP is within 2.5 cm of the plug centre and has moved with the plug since the plug was last seated
  (the TCP's displacement and the plug's agree within 6 mm). Otherwise the plug was pulled out by its cable; a gripper
  that only hovers beside the plug while the cable is pulled does not move with it, so it does not count.
- **Not released.** At the end, a gripper TCP is within 3.5 cm of any part of any lead.

Otherwise:

- **1.0:** the evidence was complete when the faulty plug first left its seat (more than 1 cm), and the faulty lead
  now lies entirely in the tray: every plug, cable and ring-terminal point within the tray's inner footprint (up to
  3 mm outside it allowed, for the wall's inner edge) and below 4 cm above the table.
- **0.75:** the evidence was complete when the faulty plug left its seat, and the lead is out of the tester (its
  plug at least 3 cm from its seat and its ring terminal off the bar) but not entirely in the tray.
- **0.5:** the evidence was complete when the faulty plug left its seat, but the lead is not out of the tester
  (unplugged, not rejected: for example the plug set down beside its seat with the ring terminal still on the bar).
- **0.25:** a completed diagnosis, not acted on: the evidence above is complete (the faulty lead's lamp went dark
  when flexed, or both good leads were flexed with their lamps lit), and no lead has left its seat.
- **0:** anything else. In particular, a faulty lead taken out before the evidence was complete scores 0 whatever
  happens next, so a guess cannot score even when it is right.

Chance: a policy that never flexes a lead scores 0 on every trial, whichever lead it takes. One that flexes leads until the
lamps have shown the faulty one, or flexes all three, and stops earns 0.25 on every trial: it has done the diagnosis,
though not acted on it. One that flexes
all three and then ignores the lamps (always takes lead a, say) scores 1.0 on the one trial in three where a is
faulty and 0 on the other two (it takes a good lead out), 0.33 on average. Reading the lamps and rejecting the lead
they point to scores 1.0.

## Shortcut and tier variants (`WIGGLE_DEMO=<name>`, test only; `WIGGLE_FAULT` forces the fault for these runs)

Each variant was run with the fault forced to the lead shown, so that the run tests the loophole it names. The
last code change (the 0.25 rule now takes either sufficient lamp result instead of all three tests) came after the
`none`, `diagnose`, `cordpull`, `unplug`, `outside` and `correct` runs; it changes nothing for them (`none` tested no
lead, `diagnose` tested all three, the others took a lead out). The demo, `diagstop` runs and final check ran on
the final code.

| variant | fault | what it does | score |
|---|---|---|---|
| `none` | a | does nothing | 0 (no test) |
| `alwaysa` | a | no test; rejects lead a, which happens to be the faulty one | 0 (taken out before any evidence) |
| `flexalla` | b | tests all three, sees lamp b go dark, takes lead a out of its seat anyway | 0 (good lamp went dark) |
| `diagstop` | a | tests a, sees its lamp go dark, then stops without unplugging | 0.25 (diagnosis complete, not acted on) |
| `diagstop` | c | tests a and b, both stay lit (so c is faulty), then stops without unplugging | 0.25 (diagnosis complete, not acted on) |
| `diagnose` | c | tests all three (a and b stay lit, c goes dark), then stops without unplugging | 0.25 (diagnosis complete, not acted on) |
| `cordpull` | a | finds the fault, then the right arm alone pulls the lead out by its cable and carries it to the tray | 0 (plug not carried by a gripper) |
| `unplug` | a | finds the fault, unplugs the lead (plug lifted out and set down 3 cm back), leaves it in the tester | 0.5 |
| `correct` | c | the full demo with the fault on c: tests a and b, both stay lit, rejects c untested | 1 |
| `outside` | a | finds the fault, takes the lead out and lays it on the table beside the tray | 0.75 |
| (default) | a | the full demo (recorded) | 1.0 |

A policy that tests all three leads and then always rejects lead a scores 1.0 on the trials where lead a happens to be
faulty (one in three) and 0 on the others; it cannot do better than chance without reading the lamps.
