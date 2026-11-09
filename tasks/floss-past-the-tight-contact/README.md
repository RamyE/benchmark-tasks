# Floss two model teeth past their tight contact without snapping into the gum, wiping both sides

A dental training model stands on the table in front of the robot: two white tooth blocks, one behind the other, on a
pink gum base. Their crowns meet in a tight contact 1 cm below the tops (the back tooth is held against the front one
by a spring, so the contact opens only when something is pressed into it). Below the contact the two teeth stand 1 cm
apart and a dark-pink papilla, the gum between the teeth, fills the bottom of that gap. A white floss cord lies on the
table in front of the model, each end joined to a blue pull tab through a short elastic link.

The robot holds a tab in each hand (the floss taut between them, as a person holds floss between two fingers), lays the
floss in the notch above the contact and eases it past with a back-and-forth along the floss, keeping the hands well
above the gum so the floss does not snap down into it. It then curves the floss against the side of the front tooth in
the gap and rubs it up and down, does the same on the back tooth, brings the floss back up past the contact, puts it
down and lets go.

The tested ability is controlled two-handed tension on a thin deformable line in a contact-rich, hidden space:
- the floss only opens the contact when it is taut and pressed down from both sides (about 3 N of floss tension with the
  hands 2 cm below the contact, plus a back-and-forth to beat friction); a floss held at one end slides off;
- how it gets through is scored, not only that it gets through: forced through with the hands low, the floss pops past
  and hits the gum faster than 0.12 m/s, which scores 0 for the whole trial (ADA: "Never snap the floss into the gums");
- cleaning is a second, separate act on each tooth: the floss must be pulled against one gap face (a C-shape round
  that tooth) and moved up and down at least 8 mm, twice, then the same on the other tooth;
- then the floss must come back out through the contact and be let go.

## How this differs from similar tasks

- No task on the board or in LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench or RoboDojo flosses or works a line
  through a sprung contact into a closed gap. The board's only dental task (TK-3FNB5Y) is a dental-lab casting ring.
- Shoe-shine sanding a pipe with a two-handed strip (a related idea; it never reached a demo): a strip drawn back and
  forth round an exposed cylinder. Here the line is worked through a closed, sprung contact into a hidden gap, then
  wrapped on one face at a time with up-down strokes, with a speed rule on how it goes through.
- Seat a cable in a channel / lay a tube into a pump head (other cable tasks): a line pressed into an open slot that
  stays open. Here the slot is shut by a spring, opens only under two-sided tension, and must be passed twice (in and
  out) without a snap.
- Wipe a lead in one pass: a single stroke along an exposed line. Here the line is the tool and the strokes are
  on two hidden faces, counted per face.

## Scene (station table; cm from the midpoint between the arm bases, x forward, y left, heights above the table)

| object | type | size | material (real rebuild) | position |
|---|---|---|---|---|
| gum base | fixed block | 13 x 4 x 4 cm | pink PLA | x 30-43, y -2..+2, on the table |
| front tooth | fixed block | 5 x 4 x 5.5 cm (x, y, z) | white PLA | x 30.5-35.5 on the base (tops at height 9.5) |
| back tooth | block on a slide along x | 5 x 4 x 5.5 cm, 50 g | white PLA on a linear rail (MGN7) with a compression spring, 0.7 N preload, 120 N/m, 8 mm travel | x 36.5-41.5 at rest |
| crown ridges | one on each tooth's gap face | 12 mm round rod, 4 cm long along y, centre 1 cm below the top | white PLA, printed with the tooth | the two ridges meet at x 36, height 8.5 (0.4 mm apart at rest: shut to a 3 mm floss) |
| gap | between the teeth below the ridges | 1 cm wide, 4 cm deep (y) | | faces at x 35.5 and 36.5 |
| papilla | fixed block in the gap | 0.95 x 4 x 0.8 cm | dark-pink PLA (silicone in a rebuild) | x 35.5-36.45, height 4-4.8 (3.7 cm below the contact) |
| floss | free cord | 3 mm round, 20 cm, 6 g | white braided cord (stand-in for floss at model scale) | straight along y at x 24, y -10..+10, on the table |
| pull tabs | 2 free fins, one on each floss end | 2.5 x 1.2 x 2.5 cm (along the floss, thick, tall), 3 g | blue PLA | standing on the table at x 24, y +-11.6 |
| elastic links | tab to floss end | 100 N/m, rest length 0 | rubber loop with a 4.5 N breakaway snap | at each tab's inner face, level with the floss (2 mm above the table) |

## Sources (each page opened; quotes verbatim)

- American Dental Association, MouthHealthy, "Flossing", https://www.mouthhealthy.org/all-topics-a-z/flossing ("5 steps
  for flawless flossing"): "Hold the floss tightly between your thumbs and forefingers." "Guide the floss between your
  teeth using a gentle, rubbing motion. Never snap the floss into the gums." "When the floss reaches the gum line, curve
  it into a C shape against one tooth." "Hold the floss tightly against the tooth, slightly away from the gums. Gently
  rub the side of the tooth with an up-and-down motion, following the shape of the tooth." Also: "If you floss too hard,
  you might damage the tissue between your teeth."
- NHS, "How to keep your teeth clean", https://www.nhs.uk/live-well/healthy-body/how-to-keep-your-teeth-clean/ (section
  "How to use dental floss"): "Take 30cm to 45cm of floss or dental tape and grasp it so you have 3cm or 4cm of floss taut
  between your hands." "Curve the floss into a C-shape and then gently slide it up and down each tooth to clean it and
  remove any food and plaque."
- O*NET 31-9091.00 Dental Assistants, https://www.onetonline.org/link/summary/31-9091.00: "Instruct patients in oral
  hygiene and plaque control programs."
- O*NET 29-1292.00 Dental Hygienists, https://www.onetonline.org/link/summary/29-1292.00: "Provide clinical services or
  health education to improve and maintain the oral health of patients or the general public."

What the sim takes from them: the floss is held taut between two hands; it is guided past the contact with a rubbing
(back-and-forth) motion and must not be snapped into the gum; each tooth's side is cleaned separately by curving the
floss against it and moving it up and down. The model is a training model on a table; no patient is involved.

## Why two arms

Simultaneous hold, not reach. Offline IK (`scratch/ik2.py`, output `scratch/ik2.out`): every working point (tab pick-up,
notch, stroke heights, curve positions) is reachable by either arm (0.0 mm error, straight-down grip for its own side,
any orientation for the other side). Two arms are needed by physics: the floss presses on the contact only when it is
pulled down on both sides of it at once (the downward force is twice the floss tension times the slope of the floss on
each side), and curving it against one face needs both ends pulled the same way. With one tab held and the other end
free, the floss just slides over the ridges (`onearm` below: the contact never opens, score 0). There is nothing in the
scene to anchor the second end to: a tab laid on the table slides (3 g), and a tab touching the model scores 0.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (floss simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "A white plastic two-tooth model stands on a red block on the table. Pass the floss down between the two plastic teeth, wipe the side of each tooth in the gap, then take the floss out and let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop on the live floss and tooth. Each arm pinches a tab across its thickness (wrist cameras turned in towards the
model), both lift the floss and carry it over the model, and lay it in the notch above the contact. The hands go down to
2 cm below the contact on either side and move apart until the elastic links are stretched 3 cm (about 3 N of tension),
then move together along the floss, 12 mm each way, until the floss is past the contact (the back tooth opens 1.8 mm
and lets it through; the floss drops at most to 2 cm under the contact, 1.7 cm above the gum, and settles 0.5 cm under
the contact). The hands shift 2 cm towards the front tooth (floss curved against its gap face) and move up and down twice between 5.4 and 7.6
cm, then the same towards the back tooth, then come back to the middle and rise with the same back-and-forth until the
floss is back above the contact. They lift it 6 cm above the tooth tops, bring the hands 1.5 cm in so the floss hangs
slack (a long straight, taut floss is the sim's weak point, see Limits), carry it to the front, put the tabs down, open,
lift 4 cm and sweep back to the sides, out of the top camera's view of the model.
Result (`demo/result.json`, recorded with all three cameras): score 1.0, success at step 712 (35.6 s). Floss past
the contact at 14 s; front face wiped with the floss moving between 6.2 and 7.8 cm high (4 strokes of about 16 mm),
back face between 6.0 and 7.7 cm (the back tooth gives 0.8-1.8 mm while the floss is pulled against it); floss back
above the contact at 30 s; tabs down and let go at 34 s. Closest the floss came to the gum: 1.0 cm
(floss surface, during the back-face strokes).

## Scoring (`score()`, evaluated at the end; `success()` = score 1.0 held 1 s with the floss at rest)

Score 0 if any of these holds (all latched for the trial):
- snapped: any floss segment within 3 mm above the papilla (inside the gap) that moved down more than 6 mm in one
  control step (1/20 s), i.e. reached the gum faster than 0.12 m/s, while the floss was under tension (either elastic
  link stretched 1 cm or more, about 1 N; slack floss settling onto the gum under its own weight is harmless and not
  counted) (real trial: 20 fps video of the gap plus the link stretch);
- a gripper link or a tab touched a tooth, the papilla or the gum base (contact read every physics step);
- an elastic link was stretched to 4.5 cm (4.5 N; the tabs pulled apart; the real snap releases);
- the floss never went down past the contact: the back tooth must have been pushed 1.5 mm or more open, and then a
  floss point must lie between the teeth (inside their depth by 4 mm) 4 mm or more below the contact;
- at the end a gripper TCP is within 3 cm of the floss or a tab (not let go);
- any non-finite state.

Otherwise:

| score | end state |
|---|---|
| 1.0 | both gap faces wiped and the floss out of the space between the teeth, both tabs let go |
| 0.6 | both gap faces wiped but the floss is still between the teeth at the end (let go there) |
| 0.3 | the floss was passed below the contact but at least one face was not wiped (floss left in the gap or taken out) |

A face counts as wiped when the floss lay against it (at least 1 cm of floss within 0.5 mm of the face, between the gum
and the contact) while moving 8 mm or more up or down four times (two up-and-down strokes), with 2 mm hysteresis
between direction changes; lifting off the face ends a stroke. The lowest tier needs the contact opened and the floss
through it; touching, laying or tensioning the floss on top earns nothing.

## Shortcuts and partial tiers (each run as a `demo_script` variant after the last edit of `task.py`; evidence `scratch/sc_<name>.json`)

| variant (`FLOSS_DEMO=`) | what it does | outcome | score |
|---|---|---|---|
| `none` | touches nothing | contact never opened, floss never passed | 0 (step 120) |
| `hold` | does the whole task, puts the tabs down but never lets go | both faces wiped, floss out, grippers still on the tabs | 0 (step 780) |
| `onearm` | right arm only: drapes the floss over the notch by one tab, pulls it down and works it back and forth for 6 s, puts it down | the contact never opens (0.0 mm); the trailing free tab is dragged into the model (touch latch) | 0 (step 480) |
| `snap` | tensions the floss, then shoves both hands 3 cm down in 0.15 s; then does the rest correctly | the floss pops past and drops 27 mm onto the gum in one step | 0 (step 831) |
| `yank` | pulls the hands apart (y +-20 cm) with the floss over the contact; then does the rest | links stretched to 4.5 cm | 0 (step 820) |
| `fingers` | the right hand lets go of its tab and pushes the floss into the contact with its fingertips | fingertip touches a tooth | 0 (step 448) |
| `pass` (tier) | passes the contact, slowly brings the hands in until the floss is slack in the gap, lets go | floss left in the gap, no face wiped | 0.3 (step 490) |
| `front` (tier) | passes, wipes only the front tooth's face, takes the floss out, puts it down, lets go | back face not wiped, floss out | 0.3 (step 672) |
| `leave` (tier) | passes and wipes both faces, slackens the floss in the gap, lets go | both wiped, floss left between the teeth | 0.6 (step 742) |
| correct demo | | both wiped, floss out, let go | 1.0 (step 712) |

Every run above ended with a finite state (no non-finite flag in any log).

## Feasibility checks

- Passing: with 0.7 N preload the contact opens at about 3 N floss tension with the hands 2 cm below it and a 12 mm
  back-and-forth (demo); the floss settles 1 cm under the contact, above the gum. No non-finite state in any run after
  the fixes below.
- Snap: the same tension forced through by moving both hands 3 cm down in 0.15 s drops the floss onto the gum in one
  step (`snap`).
- Wiping: with the floss curved against a face, the floss in the gap follows the hands, about 16 mm per stroke.
- Lift-out: rising with the back-and-forth opens the contact from below; the floss comes out above it.

Flaws found and fixed during the build (sim): a breakable tab link tore on small jerks (now an elastic link, pull-apart
judged at 4.5 cm stretch); a twist-free link let the whole floss spin and go non-finite (twist locked); a straight,
taut floss under the contact vibrated and went non-finite (bend damping raised); a 3 mm slot beside a narrow papilla
trapped the floss (papilla now fills the gap); the stroke counter dropped a finished stroke when the floss left the face
and counted a face touched by the floss lying mid-gap (strokes credited on lift-off; contact now within 0.5 mm). The
snap rule first counted slack floss falling onto the gum under its own weight when let go in the gap (a harmless act);
it now also needs the floss under tension (1 N). Lowering the floss taut to the gum in the sim can slip it several mm in
one step, which is counted (a taut slip into the gum is the real hazard). A legitimate slow taut lowering to within 3 mm of the papilla can therefore be scored 0 by this artefact; nothing in the task needs the floss that low (wiping happens higher up).

## Limits

- Blind replay: the layout is fixed (nothing is randomised), so a policy that replays the demo's arm motions open loop
  would also score 1.0. What it tests is the tension and manner of a two-handed line act in a hidden gap.
- Sim weak point: a floss held long, straight and taut between the hands in free air can vibrate until the state goes
  non-finite (one build run did so while carrying the floss taut over the tooth tops). The demo carries it slack. A
  non-finite state scores 0; the real model has no such failure.
- Wrist cameras: they look down at the pinched tab and the floss leading to the model; the gap itself is seen only by the
  top camera (mean brightness of every wrist-camera second 107-190).
- Pulling the floss out sideways (letting go of one tab and drawing the other) drags the free tab, which is wider than the
  gap, into the teeth and scores 0 under the touch rule; the floss must come back up through the contact.

## Runtime

One trial: 1,200 control steps at 20 Hz (60 s of sim time); the scripted demo succeeds at step 712 (35.6 s). Seedless:
no randomisation, `object_jitter=0`.

## Paper fields

- Skills: insertion (line through a sprung contact), constraint-keeping (no snap, no touching the model), tool-use (the
  line is the cleaning tool), extraction (floss back out).
- Industry: dental care / oral-hygiene instruction (training model).
- Two arms: simultaneous hold (floss taut between both hands; reach alone does not force it).
