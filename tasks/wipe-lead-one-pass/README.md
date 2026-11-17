# Wipe a monitor lead, cord and both end pieces, outward from its blue band, never wiping back

A free grey patient-monitor lead (6 mm PVC cord, 60 cm) lies straight across the table: its grey plug in front of
the left arm, its red patient clip in front of the right arm, and a blue band on the cord 14-16 cm from the plug.
Both grippers' finger pads are covered with disinfectant wipe. The robot must wipe the whole lead: the cord from the
plug face to the clip face, and every outer face of the plug and clip housings, undersides included (both long sides,
the top, the free tip face, the face the cord comes out of, all round the cord, and the underside), outward from the
blue band to each end, every part once and never back over a part already wiped. The faces that show while the lead
lies on the table are wiped there, the other hand holding the far end; the underside of each end piece, and the strip
of its cord face under the cord, face the table, so one hand holds the piece up and rolls it a quarter turn about
the lead, turning the underside to face sideways, while the other wipes them. Then the whole lead goes onto a clean white drape and the robot lets go.

Who does it: medical assistants (O*NET 31-9092.00, 833,900 US workers) "Operate x-ray, electrocardiogram (EKG), or
other equipment to administer routine diagnostic tests" and "Clean and sterilize instruments and dispose of
contaminated supplies" (rated importance 94 of 100 for the occupation, i.e. core work), and O*NET lists "Clean
medical equipment" among their work activities. The person who runs the ECG cleans its lead set after each patient,
as the manufacturer's guide below requires, so this is routine, repeated work for the occupation, not a rare chore. Licensed
practical nurses (O*NET 29-2061.00, 666,900 US workers), who monitor patients on these leads, also list "Sterilize
equipment and supplies, using germicides, sterilizer, or autoclave" (importance 84) and the work activity "Clean
medical equipment or facilities". Together that is about 1.5 million US workers whose listed work includes cleaning
patient equipment such as this lead between patients.

Why: the Philips lead-set guide says "Clean and disinfect the ECG cable and accessories after each use" and, for the
MX40 lead sets, "Wipe the entire outer surfaces of the cable to remove soil". Only the connector pins get a separate step
(alcohol, connector pointed down), so the pins are not part of this task, but the plug and clip housings are. A
damp-wiping guide adds the one-direction rule: "Avoid using a back and forth motion to prevent the cloth picking up
dirt going one way and then dropping it off on the way back" and "wiping each part of the surface ONCE is enough".

Start point, stated plainly: none of these sources says where on a lead to start or which way to wipe it, so the blue
band is a task choice, not a rule from a manual. What the task takes from real practice is the one-direction rule: wipe
each part once and never back over cleaned surface. Starting in the middle and wiping outward to both ends is one way
to satisfy that rule with a free lead and two hands (see below); wiping from one end to the other in a single pass
would satisfy it too, but a closed hand moving toward the hand that holds the lead cannot go all the way, so the
scoring splits the lead at the band.

The tested ability is continuous sliding contact along a free deformable object in one direction, with the anchoring
role changing between the two hands. The lead is free, so a closed hand moving along it just drags it unless the
other hand holds the far end: to wipe the clip half and the clip outward, one hand holds the plug while the other
slides toward and over the clip; to wipe the plug half and the plug, the hand on the clip holds it while the other
slides from the band toward and over the plug. The wiping hand has to stay closed on the thin cord while following
it right up to the end piece, open there so the pads' edges sweep the face round the cord, pass over each end piece's
two long sides with the pads opened just to its width, then turn the closed fingers along the lead and run their tip
pads down the face above the cord, over the top and down the tip face, all without dragging the free end piece; it
must not open and hop forward (gaps stay unwiped) and must not scrub back and forth. Last, each end piece is held up
rolled a quarter turn about the lead (the holding wrist turns), and the other hand's shut fingers sweep out across
the strip of cord face under the cord and then along the underside, from the cord face to the tip.

Why the blue band: a hand closed on the cord needs about 15 cm of clearance from the other hand, so a stroke can
start no nearer than that to the held end. The band marks that point; from it each half is wiped outward, away from
the hand that anchors it, so the two strokes meet at the band and nothing is wiped twice beyond a short overlap (the
damp-wiping guide asks passes to overlap by 2-5 cm).

## What changed from the earlier version

The latest review said: "The task permits a supposedly clean lead to retain contamination on both end-piece
undersides and cord-face strips, although the pieces can be repositioned", "Full credit does not require tracer
removal from the end-piece undersides or the strips beneath their cord connections", and "Lift or reorient the end
pieces to wipe these surfaces, add tracer setup and full-score checks for them, and update the simulated scoring and
full-score demo to demonstrate the complete procedure." (The round before asked for the whole cord and every outer
face of the plug and clip; that version added the 60 end-to-end cord marks and the side, top, tip and cord-face
passes, which stay as they were.)

What was done:
- Tracer: the real-trial setup now dusts each end piece's underside and the strip of its cord face below the cord
  (between the cord and the table) as well, so every outer surface of the lead carries tracer; the old "What is not
  wiped" note is gone.
- Scoring: each end piece has two more scored parts, eight in all (16 for the lead): the strip below the cord and the
  underside. Both must be wiped (sim) and free of tracer (real trial) for a full score; missing either caps the score
  at 0.6. The underside pass falls under the same one-way rules as the other end-piece passes: going over 1 cm back
  toward the cord, or wiping either part a second time, scores 0.
- Demo: after its other faces are done, each end piece is lifted 4 cm by one hand, which rolls its wrist a quarter
  turn about the lead so the underside faces sideways. The other hand's shut fingers come in beside the cord, creep
  along until they feel the cord face, sweep out across the strip below the cord until they are past the underside's
  edge, then slide along the whole underside to past the tip, and move off outward. The plug goes first and then
  waits on the drape; the clip, set down meanwhile, is found again by feel. The run limit went from 2,000 to 3,000
  steps for the extra passes.
- Three shortcut variants test exactly these gaps: `nounder` (both pieces turned up but neither surface wiped)
  scores 0.6, `nostrip` (underside wiped, strip skipped) 0.6, `noslide` (strip wiped, underside
  skipped) 0.6. All variants below were rerun on the final code.

## Closest tasks

- TK-7P97ME (finish painting a panel without repainting a coworker's work) scores grid-cell coverage of a rigid flat
  panel by a roller with a no-repaint rule; here the "surface" is a free cord that must be anchored by the other hand
  while a closed pinch slides along it, and the anchor role switches between hands.
- TK-GM789T (check engine oil with a dipstick) scores a rag touching a rigid blade pulled from its tube; here the pads
  close round a free deformable cord and two end bodies and the score depends on stroke direction and anchoring.
- TK-LMMBEC (swipe a card through a reader in one pass) scores a single one-way pass of a rigid card through a fixed
  slot; here the object being passed over is itself free and deformable and has to be held taut by the other hand.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Setup and materials (real trial)

Positions in cm from the midpoint between the arm bases on the table (x forward, y to the robot's left).

- Lead: grey 6 mm round PVC cord, 60 cm, lying straight along y at x 40 from y 6.25 to y -53.75; grey plug
  35 x 16 x 12 mm (12 g) on its left end (centre y 8), red patient clip 25 x 15 x 15 mm (10 g) on its right end
  (centre y -55). Blue tape band 2 cm wide on the cord 14-16 cm from the plug end of the cord (y -7.75 to -9.75).
- Drape: white cloth on a 3 mm board, 18 x 76 cm, x 12-30, y -56 to 20, taped down.
- Wipes: PDI Sani-Cloth Plus pre-moistened disinfectant wipes, large-canister sheet 6 x 6.75 in (about 15 x 17 cm).
  Before each trial, cut one fresh wipe in half for each gripper; fold each half into an 8-layer pad about 4 x 4 cm
  (about 3 mm thick), back it with a 4 x 4 cm square of 3 mm EVA craft foam, so the pad is about 6 mm thick and
  gives when pressed, and lay one over each finger, foam side in, so it covers the finger's gripping face, wraps round its tip and about
  2.5 cm up its outer face, and, centred on the finger (about 15 mm wide near its tip), stands out about 1 cm past each
  side edge; hold it with a rubber band round the finger above the pad. First cover each finger with a nitrile finger
  cot (or a fingertip cut from a disposable nitrile glove) pulled over its tip and up past the pad, so neither the moist
  wipe nor the tracer powder reaches the gripper; discard cots and pads after each trial. The wipes are fitted before the trial starts, so whatever a gripper
  touches, it touches through wipe; the robot never picks a wipe up. Keep the wipe box off the table.
- Pad gap g0: with the pads fitted, close each gripper fully on nothing and note its finger gap g0 (the two pads'
  pressed thickness, about 10 mm). The simulation has bare fingers; real-trial gap thresholds are given next to the
  simulation's below.
- Tracer: before the trial, with the lead in its start place, dust with UV fluorescent tracer powder the whole cord,
  from the plug face to the clip face, and every face of the plug and of the clip: both long sides (plug 35 x 12 mm,
  clip 25 x 15 mm), the top and the underside (plug 35 x 16 mm, clip 25 x 15 mm; lift each piece by its cord to dust
  the underside and set it back), the free tip face and the face the cord comes out of, all round the cord. Mark the cord
  every 1 cm with a fine pen (60 marks, 0.5 cm to 59.5 cm from the plug face); draw a line round the plug and the
  clip 8 mm from the cord face.
- Start: both arms in their ready pose (grippers about 26 cm above the table at x 32, in front of their bases).
  The right gripper hovers over the middle of the lead in the top-view image and hides part of it.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 3000 \
    --instruction "Wipe the whole grey monitor lead with the wipe pads, the cord from end to end and every face of both end pieces, undersides included, wiping outward from the blue band and never back over any part already wiped, then lay the lead on the white drape and let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 3000
```

The demo chooses its motions only from the fixed, known starting layout and the robot's own measured joint angles
(TCP positions computed from them); object poses from the simulator are used only for scoring and the run log. While
one hand holds an end piece and the other slides along the taut cord, the slide position along the cord is the
measured distance between the two hands minus a constant 1.8 cm. When a stroke has run up to an end piece, that end piece lies
in line with the stroking hand's pads. Along the lead, the clip is located from the hand holding the plug (the lead is
then taut and straight, centres 62.8 cm apart, a measured constant; the right hand's own reading is a centimetre off
out there), and the plug from the stroking left hand itself (pad centre stopped 11.5 mm from its face). Sequence:
1. The left hand takes the plug and lifts it (TCP 2.5 cm above the table). The right hand closes lightly on the cord at the blue band (pads
   5 mm apart on the 6 mm cord) and steps toward the clip in 1.5 cm moves, slowing to 5 mm moves near the end. For the
   last 2 cm it eases the pads to 6.6 mm apart (just round the cord; at full squeeze the cord's stiff last 2 cm drags
   the clip and pulls the lead out of the left hand), lifts about 1 cm (10.5 mm) so the fingers' wider part clears the clip's top
   edge, and steps on in 3 mm moves until the hand stops advancing with the fingers against the clip (pad centre
   about 11.5 mm from its face).
2. The left hand is re-sent to where it now is (so it does not spring back when the right lets go). The right hand
   opens on the spot, sweeping the clip's cord face, and lifts. The left hand draws the lead 5 cm toward itself, so
   the clip lies where the right hand places accurately.
3. The right hand comes down wide open over the clip's middle, closes to 2 mm wider than the clip and opens again,
   which squares the clip under it (when the pads let go, the cord can spring an end piece a few mm sideways).
4. Top-and-tip pass on the clip: the right hand shuts its fingers, turns them a quarter turn so they close along the
   lead, comes down just behind the clip's cord face to 2 mm above the cord, rises to clear the clip's top by a few
   mm, runs along the top to just past the tip, comes down the tip face to 2 mm above the table, moves off outward
   and up, and opens.
5. Side pass on the clip: the right hand sets down wide open 4 mm short of the clip's cord face, its pads straddling it, closes to
   21 mm (bare fingers, about 3 mm clear of each side, so the real 6 mm pads press on it) and slides in one pass over the
   clip and 2.2 cm off past its tip, then opens and lifts. In steps 4 and 5 the hand settles a few mm off target, so
   the offset measured at a hover point is fed back. The left hand holds the plug throughout steps 1-5.
6. The right hand takes the clip by its centre. Both hands carry the lead 8 cm to the left; the left hand sets the
   plug down and lets go. The right hand lifts the clip (TCP 2.5 cm above the table), so the cord near the band is clear of the table.
7. The left hand closes lightly on the cord at the blue band and steps toward the plug the same way, easing the pads
   for the last 2 cm, until it stops with the fingers against the plug. The right hand is re-sent to where it is, and
   the left opens on the spot, sweeping the plug's cord face, and lifts.
8. The plug is squared, then gets the same top-and-tip pass and a side pass (set down wide open 4 mm short of the plug's cord face,
   closed to 22 mm, slid over the plug and off past its tip), the right hand holding the clip throughout.
9. The left hand takes the plug where it lies; the right hand sets the clip down where it is and lets go. The left
   lifts the plug, brings it to the middle 4 cm above the table and rolls its wrist a quarter turn about the lead
   (fingers turned to point away from the robot), so the plug's underside faces away from the robot.
10. Underside pass on the plug: the right hand shuts its fingers (closing across the lead) and comes straight in from
   4.5 cm out to beside the cord, short of the plug's cord face, its outer face about 3 mm off the cord and its
   fingertip 1 mm above the underside's lower edge (so the fingers' wider part, 14 mm up, about clears the piece). It
   then creeps toward the face 1 mm at a time until it feels it (its measured position lags the command by 4 mm),
   moves straight out across the strip of cord face under the cord until its outer face is 9 mm past the
   underside's plane, comes back to 5 mm past it, slides along the underside and off past the tip, and moves off and up. The plug is located
   from the left hand's measured pose (centre 5.5 mm beyond its TCP along the fingers); the hover-point offset is
   fed back as in steps 4-5.
11. The left hand rolls back and puts the plug on the drape. The right hand finds the clip again by feel (the bent
   lead can spring a let-go piece 1-3 cm along itself): it closes across the lead at points stepping in 1 cm at a
   time from past the clip's tip, reading the finger gap, until the fingers stop on the clip, and takes it 6 mm
   further in. It brings the clip to the middle and rolls it the same way, and the left hand gives it the same
   underside pass (the clip, taken near its tip, is estimated 1.2 cm toward the cord from the right's TCP; the creep
   finds its face).
12. The right hand rolls back; the left finds the plug on the drape by feel the same way and takes it; both lift the
   ends, draw the lead straight 62.5 cm apart along the drape, set it down, open and move clear.

Recorded run (`demo/`): score 1.0, `success()` true, 2,583 steps (129 s); all 60 marks and all eight parts of both
end pieces wiped, no 0-rule fired. The run ends on `success()`: at that step both grippers are open and over 3 cm
from every point of the lead (the 0-rule check), and the whole lead, plug and clip lies on the drape.

## Scoring (`score()`, evaluated at the end; `success()` must hold at the end)

Terms:
- **Closed on the cord**: finger gap at most 7 mm (cord diameter + 1 mm; real trial: 7 mm + g0) with part of the
  cord between the pads.
- **Holds** an end piece: finger gap at most its larger cross size + 3 mm and its centre within 2.5 cm of the TCP.
- **Stroke**: one continuous closure of a hand on the cord. Its direction is set once it has slid 1 cm along the
  cord. Its **anchor** is the end piece behind it (the plug for a stroke toward the clip, the clip for a stroke toward
  the plug), and the stretch it passes over (the pads' 2 cm footprint, swept) counts only while the other hand holds
  that anchor.
- **Wiped mark**: the cord has 60 marks, one per cm from end to end (0.5 to 59.5 cm from the plug face). A stroke
  wipes the marks within 0.5 cm of the stretch it passed over once it has slid at least 2 cm. A stroke toward the clip
  counts only marks from 13.5 cm on, a stroke toward the plug only marks up to 16.5 cm: each side of the blue band
  must be wiped outward. Opening and re-closing further on starts a new stroke, so hops leave gaps.
- Each end piece has eight **parts**, all needed for a full score; u is a position along the piece from its middle,
  positive toward its free tip (the piece runs from u = -L/2 at its cord face to +L/2 at its tip; L = 35 mm plug,
  25 mm clip):
  1. **Cord up to the face**: an anchored stroke toward the piece brings the pad centre to within 13 mm of its cord
     face, measured along the table (the bare finger's leading edge, 7.7 mm from the centre, within 5.3 mm; the wipe
     pad, standing out about 1 cm past that edge, is pressed against the face). So the last centimetre of cord at each
     end is wiped by the pads, not just credited from a mark near it. 13 mm is about as close as the fingers get:
     14 mm above their tips they widen, and that wider part meets the top edge of the 15 mm clip.
  2. **Cord face beside the cord**: when that stroke ends, the hand opens where it is (TCP moves at most 5 mm) to at
     least the piece's larger cross size + 6 mm, so the pads' leading edges, pressed to the face, sweep across it from
     the cord out past its sides over the pads' full height.
  3. **Both long sides** (side pass): one continuous closure with the finger gap fitting the piece (sim, bare fingers:
     plug 10-23 mm, clip 13-22 mm, i.e. about 3 mm clear of each side, so the 6 mm pads press on it; real trial: at most the
     larger cross size + g0 + 1 mm, the pads touching), the piece's centre line between the pads (within 5 mm of
     their mid-plane) and within their height, that closes with the pads straddling the cord face (pad centre at most
     8 mm onto the piece) and slides outward until the pad centre reaches the tip, the other hand holding the other
     end piece throughout.
  4-6. **Top-and-tip pass**: one continuous closure with the fingers fully closed (gap at most 3 mm) and turned so
     they close along the piece's axis (within 30 deg), so the closed fingers' 15 mm width, with the pad standing out
     about 1 cm past each edge (about 35 mm in all), lies across the piece; the TCP within 7 mm of the piece's centre
     line, so that footprint covers the 15-16 mm wide top; the other hand holding the other end piece throughout.
     Each closed finger is 4 mm thick at its tip. In it:
     4. **Cord face above the cord**: the leading finger's outer face 0-6 mm behind the cord face and the fingertip
        down to within 4 mm of the cord's top (sim; the 6 mm pad presses on it).
     5. **Top**: the fingertip 0-6 mm above the top face (the 6 mm pad touches or presses on it anywhere in that range) at TCP positions covering
        u <= -L/2 + 4 mm to u >= +L/2 - 4 mm, so the 8 mm footprint has passed over the whole top.
     6. **Tip face**: after the top, the trailing finger's outer face 0-6 mm past the tip face and the fingertip down to
        within 4 mm of the piece's bottom edge (the table).
  7-8. **Underside pass**: the end piece is turned a quarter turn so its underside faces sideways, the other hand
     holding this piece or the other one throughout (as above: gap at most its larger cross size + 3 mm, centre within
     2.5 cm), and the wiping hand's fingers are fully closed (gap at most 3 mm) and close along the underside's normal
     (within 30 deg), so the outer face of the finger nearer the piece, with the pad wrapped 2.5 cm up it, is what
     wipes. The piece's turn is read from its own orientation in the simulation. Two ways are accepted, by how the
     turned piece lies under the downward fingers:
     - **rolled about the lead** (the fingers point across the underside within 30 deg, as in the demo; the fingertip
       from 4 mm above the underside's lower edge to 6 mm below it):
       7. **Cord face under the cord**: with the fingers' leading edge 0-8 mm behind the cord face (the fingers widen
          14 mm above their tips and that part can stop against the face first; the pad, standing out about 1 cm past
          the edge, still touches the face), the outer face goes from within 4 mm of the cord (sim; the 6 mm pad presses on
          it) out to 3 mm past the underside's plane (the 6 mm pad then spans the face's edge), sweeping the strip of
          the face between the cord and the underside's edge.
       8. **Underside**: with the outer face 0-6 mm off the underside (the 6 mm pad touches or presses on it), the TCP covers
          u <= -L/2 + 7.5 mm to u >= +L/2 - 7.5 mm, so the 15 mm finger width passes over the whole underside.
     - **tipped end-down** (the fingers point along the piece; their 15 mm width centred on it within 7 mm, the pad's
       ~35 mm covering its 15-16 mm width): 7 as above with the fingertip 0-6 mm above the cord face instead of the
       leading edge; 8 with the fingertip running from within 4 mm of the cord face to the tip.

Score:
- **0** if at any time (latched for the whole run): a stroke goes more than 3 cm back against its direction along
  the cord, or its hand comes more than 3 cm closer to the anchor end piece than it has been during the stroke (a
  closed stroke back, which pushes the cord instead of sliding on it); or a stroke passes over more than 4 marks that
  were already wiped before it began (wiping back over clean cord; the strokes may overlap by up to 4 cm, as passes
  should overlap); or a side pass that closed straddling the cord face, or a top pass on the top face, or an underside pass on the
  underside, goes more than 1 cm back toward the cord; or a new closure on a piece's sides, top or underside already
  wiped slides 1 cm or more along them (wiping them again).
- **0** if the plug is not on the drape at the end, or a gripper is within 3 cm of the cord, plug or clip at the end
  (not released), or the gripper state cannot be read.
- Otherwise, with c = fraction of the 60 marks wiped:
  - **0.0**: c under 0.5.
  - **0.3**: c from 0.5 to under 0.9 (for example only the stretch from the blue band to 55 cm).
  - **0.6**: c of 0.9 or more, but not all of: every mark wiped, all eight parts of both end pieces wiped, the whole
    lead on the drape (for example a stroke stopped 1.5 cm short of an end piece, or one piece's sides, top or
    underside skipped).
  - **1.0**: all 60 marks wiped, all eight parts of the plug and of the clip wiped, and the whole lead (cord, plug and
    clip) lies on the drape.
- `success()` additionally requires the lead to be at rest. `distance()`: 1 - (marks wiped + end-piece parts wiped) /
  76 (1.0 once a 0-rule fired).
- How it is seen: wiping is a path, not a pose in the final photo. In the video, a stroke is a closed gripper sliding
  along the cord while the other gripper holds the end behind it, from the blue band until its fingers are against
  the end piece; it then opens on the spot. A side pass is the same hand set down astride the piece's cord face,
  closed to just over its width and sliding outward over it and off past its tip. A top-and-tip pass is the hand
  turned a quarter turn with its fingers shut, coming down just behind the piece onto the cord, rising onto the top,
  running along it and coming down just past the tip. The other gripper holds the far end throughout. An underside
  pass is one gripper holding an end piece up with its wrist rolled a quarter turn, and the other coming down with
  its fingers shut beside the cord just behind the piece, moving out across the face and then along the underside
  and off past the tip.
- Real trial: a mark, or a face of an end piece, counts as wiped when the tracer is gone from it under a UV torch
  (each cm of cord all round, each face over its whole area) and the video shows the motion that wipes it as above,
  with its anchor held. Gap thresholds are the real-trial ones above (pads fitted). Backward strokes and passes and
  the final layout are read from the video and a photo of the drape: a back-slide is read against the 1 cm pen marks
  under the pad edge; for the side pass, the middle of the 2 cm pad must be at or behind the 8 mm line when the pads
  close and reach the tip before they open.

## Shortcut variants (test only, `WIPE_CHEAT=<name> python cabledemo.py task.py out`)

| Variant | What it does | Score | Why |
|---|---|---|---|
| `none` | does nothing | 0.0 | nothing wiped, plug not on the drape |
| `onearm` | right hand slides from the band toward the clip with the plug not held, then lays the lead | 0.0 | no anchor held, so no mark counts (c = 0); plug not on the drape |
| `hover` | full demo motions with the pads open on the cord and on the end pieces' sides | 0.0 | no closure on the cord, c = 0 |
| `hops` | clip-side half done by opening, moving 10 cm and closing again | 0.0 | each closure slides under 2 cm, so the clip half earns nothing and the clip's stroke never reaches it; c = 0.28 |
| `hops25` | the same with 2.5 cm hops | 0.0 | had just taken the clip for its underside pass when the 3,000-step limit ran out (c = 0.27), lead never laid or released |
| `back` | opens, goes back 8 cm, closes and wipes on | 0.0 | 0-rule: re-wiped 5 cm of already-wiped cord |
| `hold` | whole correct wipe and lay, but the hands never let go | 0.0 | not released (all 60 marks and all 16 parts wiped) |
| `midonly` | only the middle stretch: blue band to 55 cm, then lay and let go | 0.3 | c = 0.78, no end-piece part wiped |
| `nounder` | whole run, both pieces lifted and turned, but neither underside pass is made | 0.6 | c = 1.0, all other parts wiped; both pieces' strip below the cord (7) and underside (8) not wiped |
| `nostrip` | whole run, but each underside pass comes in level with the underside, skipping the strip below the cord | 0.6 | c = 1.0; plug: underside credited, strip not; clip: neither part credited |
| `noslide` | whole run, the strip below the cord wiped but the slide along the underside skipped | 0.6 | c = 1.0; plug: strip credited, underside not; clip: neither part credited |

## Runtime and seeds

- Run limit 3,000 steps (150 s at 20 Hz). One trial takes ~5-15 min wall time on one GPU including the ~1-2 min Isaac
  Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight), so every trial
  starts from the same state.
