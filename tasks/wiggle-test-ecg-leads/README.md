# Find the ECG lead with an intermittent break by flex-testing the leads

Three identical ECG lead wires are connected to a training lead tester. Each lead's connector sits in a seat in front
of the tester, its cord runs across the table, and its snap clip lies on a brass shorting bar. Each lead has a green
status lamp on the tester. One lead has an intermittent break at the strain relief next to its connector. Its lamp
stays lit while the lead lies still and goes dark only while that part of the cord is bent. Which lead is faulty is
drawn at random for every trial, and nothing in the scene shows it. The robot has to make the fault appear: flex
leads near their connectors and watch the lamps until it knows which lead is faulty, take that lead out of the tester
by its connector, and put it in the red reject tray. The good leads must stay connected throughout.

The tested ability is interactive perception. The faulty lead can only be found by deforming a cable in the right
place and reading the lamp, and then the robot has to act on what it saw. A lamp that goes dark while its lead is
flexed identifies the fault at once. Two leads that stay lit when flexed also identify it (the third lead), so
the robot does not have to test all three. Rejecting a lead before the lamps have shown which one is faulty scores 0.

Instruction: `Three ECG lead wires are connected to this training lead tester, and one of them has an intermittent break. Find the faulty lead and put it in the red reject tray.`

## How this differs from similar tasks

- Tasks that sort parts with a tester judge rigid parts by placing them in a fixture and reading a lamp. In this task
  the fault exists only while the cable is bent, so the robot must deform each cable near its connector to reveal it.
- Tasks that find a loose plug or match leads by label or colour use a cue that is visible from the start. Here all
  three leads look the same, and the faulty one changes from trial to trial, until a lead is flexed.

## Closest tasks

- TK-LXP38D scores ECG leads snapped onto the correct electrodes; this task scores finding, by flexing leads at their strain reliefs, which lead has a hidden break, and removing only that one.
- TK-SSBT8F scores a rigid test plug inserted at a meter test switch; this task scores which of three leads fails only while bent.
- TK-VBCLHX scores a cable kept unwrapped while a stage turns; this task scores a cord flexed to reveal a hidden fault.

## Why two arms (reach)

Offline IK, with the gripper pointing down:

- Only the left arm reaches the connector seats at y +18 cm. The right arm misses them by 23 to 68 mm (lead a 23 mm,
  b 42 mm, c 68 mm).
- Only the right arm reaches the reject tray. The left arm misses even the tray's nearest inner corner by about
  4 cm.
- So the left arm takes the faulty connector out of its seat and the right arm puts the lead in the tray.
- Pulling a lead out of the tester by its cord is a scored violation (score 0) when no gripper is within 2.5 cm of
  the connector. Real lead wires are unplugged by the connector body, because yanking the cord is what breaks them at
  the strain relief. The right arm's TCP gets no closer than about 42 mm to lead a's connector centre (IK: it can
  reach a grasp on the cord no further than y +13.8 cm, while the connector centre is at y +18 cm). So a right-arm-only
  pull always trips this rule; the `cordpull` variant scores 0. The left arm's pinch holds its TCP about 10 mm above
  the connector centre.

Flexing a lead does not need its connector held. In this rig the seat rails hold an unheld connector within 1 to
2 mm while the cord is flexed, so the right arm flexes a lead on its own. During each test the left arm hovers
over the lead's connector, and its wrist camera shows that lead's lamp, 26 mm across, filling much of the frame. The lamps are best read in that wrist view, where a dark lamp is plain at 320x176. From the top camera the tester is small and partly hidden by the left arm.


## Real-world build

The simulated rig is a bench lead tester; this is how to build it for real trials.

- **Tester body:** a 24 x 6 x 3 cm block (printed PLA or machined acetal), fixed to the table, with three seats in
  front of it 6 cm apart. Each seat is a pair of rails 30 x 4 x 3 mm, 23 mm apart inside, so the connector (30 x 20 x
  15 mm) slides in from the front and stops against the tester face.
- **Seat contact:** a gold-plated 1.5 mm pin sticks 4 mm out of the tester face at the centre of each seat, at the
  height of the connector's pin socket. Pushing the connector against the face slides the pin into its socket. Pulled
  back by more than about 4 mm, or tipped up out of the rails, the connector loses contact.
- **Lamp circuit (one per lead):** 3 V (two AA cells, shared) -> 100 ohm resistor -> 5 mm green LED under a 26 mm
  diffuser cap on the tester top -> seat pin -> lead connector -> lead conductor -> snap clip -> brass shorting bar
  -> battery minus. The shorting bar is the common return: a brass plate 20 x 11 cm, 2 mm thick, with three 4 mm snap
  studs on which the leads' snap clips are clipped. A lamp is lit exactly while its lead conducts from seat to bar.
- **Leads:** three identical ECG lead wires, grey 5 mm cord, 28 cm long. All three carry an identical grey 15 mm
  heat-shrink sleeve over the cord 5 to 7 cm from the connector, so the faulty one cannot be told apart by eye.
- **Reproducible intermittent break (one lead):** under its sleeve, the conductor is cut 6 cm from the connector. One
  cut end is soldered to a small spring contact (a 1 mm pogo pin, 1 mm travel); the other end is soldered to a 1.5 mm
  brass contact disc set in a 3 mm insulating collar. The two ends are held coaxially in a 12 mm length of soft silicone
  tube so the pin presses on the disc with the cord straight. Bending the cord there by more than about 30 degrees,
  in any direction, slides the pin tip off the disc onto the collar and the circuit opens; straightening it closes the
  circuit again. Set the threshold by the collar's length before the first trial.
- **Reject tray:** red, 22 x 28 cm inside, 15 mm walls.

### Reset and verification (every trial)

1. Draw the faulty seat from the trial seed: lead `["a", "b", "c"][numpy.random.default_rng(seed).integers(3)]`, the
   same draw the simulation uses (a, b, c are the seats from nearest to farthest from the robot). The faulty lead
   carries a small dot inside its snap clip, which the cameras cannot see; only the operator uses it.
2. Put the faulty lead in the drawn seat and the two good leads in the others. Push each connector into its seat
   against the tester face, square to it, lay each cord straight towards the robot and clip its snap onto the bar stud.
3. Check all three lamps are lit with the cords lying still.
4. Verify the break with a bend jig (a protractor card at the sleeve): the faulty lead's lamp stays lit at 15 degrees
   and goes dark at 35 degrees, bent up, down, left and right. A good lead's lamp stays lit at 45 degrees. If not,
   adjust or replace the faulty lead before the trial.
5. After the trial, take the rejected lead out of the tray and repeat from step 1.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the simulated cables) is in this folder next to `task.py`, and `cabledemo.py` runs the scripted
  demo.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Three ECG lead wires are connected to this training lead tester, and one of them has an intermittent break. Find the faulty lead and put it in the red reject tray."
```

The faulty lead is drawn on every embodiment reset, uniformly over the three leads, from the seed the runner passes to
`reset()` (each trial and epoch gets its own seed, so repeats vary and a given seed reproduces). Nothing visible
changes with it. Without a seed (the local demo runner), `WIGGLE_SEED` seeds the draw, or fresh entropy is used. The
scorer keeps a short history of the trial (lamps, tests, how connectors left their seats), and `task.py` clears it on
every reset. The layout is fixed (object_jitter=0, and cables are never jittered).

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

The demo does not know the faulty lead. It chooses its motions from the fixed layout, the arms' own poses and the
left wrist camera image; the simulator's object state is used only for the debug log.

1. Test a lead: the left arm hovers over its connector, with the wrist camera on the lead's lamp. The right arm takes
   the cord 20 cm from the connector and pushes it 5 cm towards the tester while lifting it 5 cm. The cord humps and
   bends about 40 degrees where it leaves the connector. The script measures the share of bright green pixels in the
   wrist image before and during the flex; below 30% of the starting share counts as dark. The cord is laid back.
2. Leads are tested in the order a, b, c and testing stops as soon as a lamp went dark (that lead is faulty) or two
   leads stayed lit (the third is faulty, untested).
3. Reject: the left arm lifts the faulty connector 2 cm up out of its seat, draws it 3 cm back and holds it. The right
   arm takes the cord near the middle of the lead, lifts it, the left arm lets go, and the right arm carries the lead
   hanging from that point over the middle of the tray, waits for it to stop swinging, lowers it slowly to the tray
   floor and lets go. Both arms park.

The demo reaches `task_score` 1.0 with the fault on lead a (flicker seen on the first test), lead b (a stays lit, b
flickers) and lead c (a and b stay lit, so c is rejected untested), in 661 to 843 steps (33 to 42 s at 20 Hz).

## Scoring (`score()`; `success()` = score 1.0 with every lead at rest, held 1 s)

"Evidence" means the lamps have shown which lead is faulty: the faulty lead was flexed past 30 degrees (cord 5 to 7
cm from the connector against the connector's axis) while its connector was seated, so its lamp went dark; or both
good leads were flexed past 30 degrees while seated, and stayed lit.

Judging this in a real trial: a camera looks straight down on the tester, which has a printed 30-degree wedge on
the deck beside each connector (two lines from the connector's axis, one on each side). A flex counts when, in any
video frame, the cord 5 to 7 cm out from the connector crosses the wedge line while that lamp is in view; a lit
lamp in the same frame means the lead passed, a dark lamp means it is the faulty one. The judge notes the frame time
of each counted flex and of the moment the first connector leaves its seat, so "evidence before removal" is read
straight from the video.

Score 0 if any of the following holds:

- **A good lead's lamp went dark at any moment** (latched). A lamp goes dark when its connector moves more than
  4 mm from its seat, rises more than 5 mm, or turns more than 12 degrees out of line. Taking a good lead out of the
  tester (rejecting the wrong lead) always does this.
- **A connector left its seat by more than 1 cm while no gripper was within 2.5 cm of it** (latched). This means it
  was pulled out by its cord.
- **Not released.** At the end, a gripper TCP is within 3.5 cm of any part of any lead.

Otherwise:

- **1.0:** the evidence was complete when the faulty connector first left its seat (more than 1 cm), and the faulty
  lead now lies entirely in the tray: every connector, cord and snap point inside the tray's inner footprint (3 mm
  margin) and below 4 cm above the table.
- **0.5:** the evidence was complete when the faulty connector left its seat, and the lead is out of the tester (its
  connector at least 3 cm from its seat and its snap off the bar) but not entirely in the tray.
- **0.25:** the evidence is complete, and the faulty connector never left its seat (fault found, lead not rejected).
- **0:** anything else. In particular, a faulty lead taken out before the evidence was complete scores 0 whatever
  happens next, so a guess cannot score even when it is right.

Chance: a policy that never flexes a lead scores 0 on every trial, whichever lead it takes. A policy that flexes all
three leads and then ignores the lamps (always lead a, say) is right on one trial in three, because the fault is drawn
uniformly per trial, and scores 0 on the other two (it takes a good lead out).

## Shortcut and tier variants (`WIGGLE_DEMO=<name>`, test only; `WIGGLE_FAULT` forces the fault for these runs)

Each variant was run with the fault forced to the lead shown, so that the run tests the loophole it names.

| variant | fault | what it does | score |
|---|---|---|---|
| `none` | a | does nothing | 0 (no evidence) |
| `alwaysa` | a | no test; rejects lead a, which happens to be the faulty one | 0 (taken out before any evidence) |
| `alwaysa` | b | no test; rejects lead a | 0 (good lamp went dark) |
| `randompick` | c | no test; rejects a random lead (it drew c, the faulty one) | 0 (taken out before any evidence) |
| `flexalla` | b | tests all three, sees lamp b go dark, rejects lead a anyway | 0 (good lamp went dark) |
| `flexonly` | c | tests all three, rejects nothing | 0.25 (fault found, not rejected) |
| `cordpull` | a | finds the fault, then the right arm alone pulls the lead out by its cord and carries it to the tray | 0 (connector pulled out by the cord) |
| `hold` | a | finds the fault, lowers the lead into the tray but keeps gripping it | 0 (not released) |
| `tier05` | a | finds the fault, takes the lead out and lays it on the table beside the tray | 0.5 |
| (default) | a, b, c | the full demo | 1.0 |

A policy that tests all three leads and then always rejects lead a scores 1.0 on the trials where lead a happens to be
faulty (one in three) and 0 on the others; it cannot do better than chance without reading the lamps.
