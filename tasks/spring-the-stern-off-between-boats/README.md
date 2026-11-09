# Spring a boat's stern off a tight berth on the bow spring, then bring all four lines aboard

A model marina berth on the table. A grey dock runs left-right in front of the robot. An orange boat lies alongside
it on two black fenders, bow to the robot's left, berthed tight between two grey boats ahead (12 cm off its bow) and
astern (8 cm off its stern). Four dark-blue 6 mm lines run from cleats on the boat to four black posts on the dock, each line ending in a
red square eye (with a grip bail) dropped over its post: the bow line leads forward to the far-left post, the bow
spring leads aft from the bow to a post just left of the middle, the stern spring leads forward from the stern to a
post just right of the middle, and the stern line leads aft to the far-right post. All four are slack. A small lamp
box on the dock shows the engine state: amber 5 s before the engine starts, green while it runs ahead.

At t = 30 s the boat's engine runs ahead for 10 s (a scripted thrust at the stern, with the helm put towards the dock)
and then goes to neutral. The robot cannot steer it or stop it; it can only choose which lines hold the boat when the
engine bites, and act before then. The right set-up is the one seamen use to leave a berth boxed in between other
boats: cast off everything except the bow spring, so the spring stops the boat going ahead, the bow pivots on its
fender and the engine swings the stern out into clear water. Then, with the engine in neutral, the spring is freed
and brought aboard, so the boat can back out with no line in the water.

The tested ability is predicting what a powered motion will do to a hull on a given set of lines, and acting in time:
- which line to keep follows from how each line would act on the hull under thrust ahead, not from how the lines look
  (all four are slack and alike at rest);
- most other choices fail by physics, not only by the score: with only the stern spring kept, or everything cast
  off, the boat drives ahead into the boat in front; with the stern line kept as well as the bow spring, or all four
  kept, the boat is held in and turns under 7 deg. Two single-line probes were run only on an earlier layout (boats
  4 cm ahead and astern, eyes only, `scratch/probe_runs/pb_*`): the bow line alone let the boat drive ahead into the
  boat in front, the stern line alone held it in (1.9 deg). One wrong set-up is rejected by the score, not by
  physics: the bow line left on as well as the bow spring (`ropelift` below) still lets the stern swing out, 36.6 deg;
  it scores 0 because the boat would then leave with a line still made fast ashore;
- the three cast-offs must be done before the engine starts (30 s), the spring must stay on through the run, and it
  may only be freed after neutral, by hand, and laid aboard.

## How this differs from similar tasks

- Strike the free lines from a pin rail, leaving the taut lines that hold a load (our earlier task, TK-MFYZK9, the
  nearest): it also lifts rope eyes off pins and leaves some on. There the answer is visible in the start state (taut
  vs slack lines) and nothing moves; the act ends when the eyes are in the crates. Here all four lines are slack and
  alike; which one to keep is a prediction of how each line will load and steer a free-floating hull under a scripted
  engine run that has not happened yet, the set-up has a deadline (the engine starts at 30 s), the outcome is a
  scored motion (stern 8 cm and 15 deg off, no contact with either neighbour), and the kept line must then be released
  after the event and brought aboard. A rule such as "keep the taut one" or "keep the inner one" does not give it.
- Tend a boat's bow and stern lines while the lock fills (our TK-BUGRAB): two lines are hauled continuously during an
  outside motion to hold the boat still. Here the robot does nothing during the motion (hands must be clear); the
  skill is choosing the line set-up beforehand so the motion itself moves the boat where it must go.
- Haul a heavy dock line across with its messenger and drop the eye over the bollard (TK-56NBWT), take a round turn on
  a bollard (TK-52TM2W), make the anchor line's bitter end fast (TK-VAW63C): making a line fast. Here lines are cast
  off and one is used as a pivot for an engine.
- No board task or public set (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench, RoboDojo) has a vessel leaving
  a berth on its lines.

## Scene (station table; cm from the midpoint between the arm bases, x forward, y left, heights above the table)

| object | type | size | material (real rebuild) | position |
|---|---|---|---|---|
| dock | fixed block | 8 x 106 x 4 cm | grey painted wood or PLA | x 18-26, y -53 to +53 |
| dock posts | 4 fixed round posts | 2 cm diameter, 3.5 cm tall | black PLA (bollards) | on the dock top at x 22: bow line +48, bow spring +5, stern spring -5, stern line -48 |
| lamp box | fixed block with two lamps | 2.6 x 2.4 x 2 cm | black PLA with an amber and a green LED block | on the dock top at x 19.5, y 0; a lit lamp is a coloured block standing 1.2 cm out of the box top |
| water | fixed plate (visual) | 42 x 106 x 0.8 cm | blue sheet (a water tray in a real rebuild) | x 27-69 under the boats |
| boat | free hull, 200 g, floating (free in x, y, yaw) | 10 x 46 x 2.5 cm, deck flush with the dock top | orange PLA hull | dock-side edge at x 27 (on its fenders), y -23 to +23, bow at +y |
| toe rail | part of the boat | 0.4 x 44 x 0.6 cm | white PLA strip | along the deck's outboard edge |
| cleats | part of the boat (drawn) | 1 x 2.6 x 0.6 cm | black | bow line (28.5, +21.5), bow spring (29, +19), stern spring (29, -19), stern line (28.5, -21.5) |
| fenders | 2 fixed blocks | 1 x 4 x 2.5 cm | black rubber | on the dock face (x 26-27) at y +17 and -17 |
| neighbouring boats | 2 fixed hulls | 10 x 18 x 2.5 cm | grey PLA | x 28-38; ahead y +35 to +53, astern y -31 to -49 (12 cm from the bow, 8 cm from the stern) |
| lines | 4 ropes tied to the cleats | 6 mm; bow line and stern line 28.1 cm (14 g), springs 14.5 cm (7 g) | dark-blue braided cord | straight from each cleat to its post, slack |
| eyes | one rigid eye on each line end, 8 g | 44 x 44 mm square ring of 6 mm bars (5 mm thick, 32 x 32 mm hole) with a 45 mm grip bail (8 mm cross bar) over the hole | red PLA, glued to the line | flat on the dock top, hole round its post |

## Sources (each opened; quotes verbatim)

- Practical Boat Owner, "Springing off and on a pontoon",
  https://www.pbo.co.uk/seamanship/springing-off-and-on-a-pontoon-90572: "By using a bow spring the stern can be swung
  out, allowing you to reverse out into clear water." "As the stern swings out, make sure the fender is still
  protecting the stem." "Make sure you give time for the crew to pull the spring aboard." "A pause in neutral will
  reduce the risk of trapped fingers or a line round the propeller."
- NauticEd sailing blog, "Using Spring Lines for Docking" (section "Spring the Aft Out from a Tee Head"),
  https://sailing-blog.nauticed.org/using-spring-lines-for-docking/: "To do this maneuver, place a spring line on a
  forward dockside cleat and run it aft to the dock." "Turn the wheel into the dock and apply forward thrust." "The
  aft of the boat will turn out." "Use a fender to protect the bow against the dock." "Once the angle is clear of other
  boats, release the spring line and go."
- Boating Magazine, "How to Dock Easier With Spring Lines", https://www.boatingmag.com/how-to-dock-easier-with-spring-lines/:
  "put the boat in forward and turn the wheel toward the dock", "the aft end will swing out, giving you clearance to
  back the boat away." and "A fender can be deployed as a pivot cushion."
- Yachting Monthly, "Springing on and off", https://www.yachtingmonthly.com/sailing-skills/springing-on-and-off-29899:
  "I also steered 'into' the dock so that the prop-wash would help throw the stern out." and "Always check that there
  are no lines near the propeller".
- O*NET 53-5011.00 Sailors and Marine Oilers, https://www.onetonline.org/link/summary/53-5011.00: "Handle lines to
  moor vessels to wharfs, to tie up vessels to other vessels, or to rig towing lines."
- O*NET 53-5021.00 Captains, Mates, and Pilots of Water Vessels, https://www.onetonline.org/link/summary/53-5021.00:
  "Dock or undock vessels, sometimes maneuvering through narrow spaces, such as locks."

What the sim takes from them: only the bow spring is left on, the engine goes ahead with the helm towards the dock
(the thrust is turned 20 deg outboard at the stern), the bow pivots on its fender and the stern swings out; the spring
is freed after the engine is back in neutral and brought aboard. The sources describe a crew slipping the spring from
on board (a slip line); here the robot stands on the dock side and lifts the eye off the post, then lays it aboard,
which is the same end state (no line left ashore or in the water).

## Why two arms

Reach-forced. The bow line's post (x 22, y +48) is reachable only by the left arm and the stern line's post
(x 22, y -48) only by the right arm, even with any gripper orientation. Offline IK (`scratch/ik.py`, output
`scratch/ik.out`; `scratch/ik_scan.py`, output `scratch/ik_scan.out`):

| point (x, y, height above table) | own arm, gripper down | other arm, any orientation (position only) |
|---|---|---|
| bow line post (22, +48, 10 cm) | 0.0 mm | misses by 38.0 mm |
| bow line post (22, +48, 16 cm) | 0.0 mm | misses by 39.4 mm |
| 3 cm inside it (22, +45, 10 cm) | 0.0 mm | misses by 6.7 mm |
| stern line post (22, -48, 10 / 16 cm) | 0.0 mm | misses by 38.0 / 39.4 mm |
| spring posts (22, +-5, 10 cm), deck drops (31, +-15, 10 cm) | 0.0 mm | 0.0 mm (pointing down: only for the near spring post) |

Scan of the other arm's position-only reach at 7 cm height: it gets to |y| 0.46 at x 10-15 cm, 0.44 at x 20, 0.42 at
x 25 and 0.40 at x 30, so the far posts at |y| 0.48 sit 4 cm beyond it. So one arm cannot cast off both the bow line
and the stern line: a one-arm policy always leaves one of them on its post at engine start.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (rope simulation, the reset that lays the lines
  out, scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "This boat is berthed tight between two others; its engine runs ahead for 10 s, starting in 30 s. Before then, lay three dock lines on the deck and keep one on its post, so the engine swings the stern out clear of the other boats. Then bring the last line aboard and let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop on the live eyes and the live hull. Each eye is pinched by its bail bar (fingers closing along the ring),
lifted straight up off its post, carried over its place on the deck (computed on the live hull pose, correcting on the
live hole centre), lowered and let go. Each eye is laid well away from its own cleat (bow line near midships, springs
about 11 cm from their cleats) so the line lies out on the deck instead of jamming against the cleat. The left arm casts
off the bow line while the right arm casts off the stern spring and then the stern line; both arms sweep back to the
sides, out of the top camera's view of the boat (set-up done at 18 s). Handling the lines drags the floating hull about: it ends the set-up 3 cm off its berth with
4 deg of yaw (limit 8) and comes within 1.1 cm of the boat astern. The engine runs 30-40 s; after neutral the left arm
lifts the bow spring's eye off its post (46 s) and lays it on the deck by the bow (50 s), then opens, lifts
4 cm straight up and sweeps back to its side (about 0.7 s), so the last frame shows the swung boat with all four
lines aboard.
Result (`demo/result.json`, recorded with all three cameras): score 1.0, success at step 1,019 (51 s). At neutral the
stern's dock-side corner is 27.2 cm off the dock face and the boat has turned 36.3 deg; closest approach to a
neighbouring boat 1.1 cm (astern, during the set-up), no contact.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s with the boat at rest)

Score 0 if any of these holds (all latched for the trial):
- the boat's hull came within 3 mm of either neighbouring boat at any time (contact);
- a gripper TCP was within 2 cm of the hull for more than 0.3 s in total (holding or pushing the boat), or the boat
  was turned more than 8 deg before the engine ran;
- an eye left its post with no gripper TCP within 2.5 cm of its bail bar (pulled off by its line, not lifted by hand);
- a gripper TCP came within 4 cm of the hull, any line or any eye while the engine ran ahead (30-40.5 s);
- at engine start (30 s) any eye other than the bow spring's was not lying on the deck (its hole over the hull
  outline and within 2.5 cm above the deck), or the bow spring's eye was not on its post;
- at neutral (40 s) the stern's dock-side corner was less than 8 cm from the dock face or the boat had turned less
  than 15 deg;
- at the end a gripper TCP is within 4 cm of the hull, a line or an eye (not let go).

Otherwise:

| score | end state |
|---|---|
| 1.0 | right set-up at 30 s, stern swung clear without contact, then the bow spring's eye also lifted off and laid on the deck: all four eyes aboard, grippers clear |
| 0.5 | the same up to the swing, but the bow spring's eye is still on its post (or was dropped off the boat) at the end; grippers clear |
| 0 | anything else |

The lowest tier needs three eyes lifted off their posts and laid aboard before 30 s and the right one left on;
touching or lifting a single eye earns nothing.

## Shortcuts (each run as a `demo_script` variant after the last edit of `task.py`; evidence `scratch/sc_<mode>.json`)

| variant (`SPRING_DEMO=`) | what it does | outcome | score |
|---|---|---|---|
| `none` | touches nothing (all four kept) | held in: stern 5.4 cm off, 6.3 deg at neutral | 0 |
| `keep_stern_spring` | keeps the stern spring instead | drives ahead into the boat in front at 31.6 s | 0 |
| `keep_stern_line` | keeps the stern line as well as the bow spring (not the stern line alone; that case is rejected by the same set-up gate) | held in: 5.7 cm, 6.7 deg | 0 |
| `keep_bow_line` | meant to keep the bow line alone, but both arms miss the middle posts, so three lines stay on (does not test the bow line alone; that case is rejected by the set-up gate: the bow spring is not on its post at 30 s, whatever the physics) | held in: 7.1 cm, 8.7 deg | 0 |
| `cast_all` | casts off all four before the engine | drives ahead into the boat in front at 37.0 s | 0 |
| `early` | right set-up, but lifts the bow spring at 34 s, engine still ahead | hands within 4 cm during the run (36.2 s) | 0 |
| `hold` | correct, but never lets go of the last eye | eye still in the gripper at the end | 0 |
| `push` | right set-up, then pushes the stern out by hand at 22 s | hull handled 1.8 s, turned 11.8 deg before the engine | 0 |
| `onearm` | left arm only, everything it reaches | stern line still on its post: held in, 5.8 cm, 6.8 deg | 0 |
| `ropelift` | right arm only; pulls the bow line by its rope to flick the eye off | cannot reach the rope (TCP stops at y +21.5 cm for a grab at +39.8 cm); bow line still on its post at 30 s, though the swing itself happens (36.6 deg, the bow line is slack ahead) | 0 |
| `tier05` | correct, but leaves the bow spring on its post | swing done, spring not aboard | 0.5 |
| correct demo | | all four aboard, grippers clear | 1.0 |

## Feasibility checks

- Swing: with only the bow spring on, the scripted 0.3 N push (20 deg outboard) turns the boat 36.3 deg and puts the
  stern 27.2 cm off the dock by neutral, no contact, no non-finite state (demo; first shown in
  `scratch/probe_runs/pb_spring_r04`, 32.1 deg).
- Wrong set-ups: table above (`scratch/sc_*.json`).
- Handling: every eye is lifted off its post by its bail and laid on the deck with the two-finger pinch (demo, all four
  'on deck' at the end).
- Two arms: offline IK above; the one-arm variants (`onearm`, `ropelift`) leave the far line on its post.

## Limit: blind replay and chance

The layout is fixed (nothing is randomised, and the engine always starts at 30 s), so a policy that replays the demo's
arm motions open loop, without looking, would also score 1.0; the task does not test perception of a varying scene.
What it tests is the choice and its timing, and a guess does poorly: of the four ways to keep exactly one line, only
one is right (1 in 4); the stern spring alone drives the boat into the one ahead (shown at this layout) and the bow
line or stern line alone fail by physics in the earlier-layout probes; keeping none or all four also scores 0. Of the
16 keep/cast-off subsets only one scores; at least one other (bow spring plus bow line) still swings the stern out
and is rejected by the rule that every line except the spring must be aboard at 30 s, not by physics. A policy that does the right handling but at the wrong time (a cast-off after 30 s, the spring
lifted before neutral) scores 0.

## Runtime

One trial: 1,200 control steps at 20 Hz (60 s of sim time); the scripted demo is done at about 51 s. Seedless:
no randomisation, `object_jitter=0`.

## Paper fields

- Skills: sequencing, placement (eyes on and off posts), constraint-keeping (hands clear, no contact), timing against a
  scripted event, prediction of a powered motion.
- Industry: maritime (boat handling, mooring).
- Two arms: reach-forced (far posts each reachable by one arm only).

