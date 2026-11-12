# Keep a condensate drain hose sloping downhill

An air conditioner's white indoor-unit box stands on the robot's left. Its off-white PVC condensate hose (8 mm
outside diameter, 5 mm bore, 56 cm long) leaves an outlet 14 cm above the table, hangs down to the table and lies
straight towards the robot's right, ending in a small grey nozzle. On the right, a drain tundish (open box, rim
3.5 cm above the table) stands on a plinth. Between them, further from the robot, two grey steps (supports) of decreasing
height (12 and 9.5 cm) form a bent path from the unit to the drain. The robot must lay the hose over both supports
and put the nozzle inside the tundish so that the hose **falls continuously** from the outlet to the drain, then
let go. Carried straight across, the hose sags onto the table and has to climb back up into the raised tundish: a
water trap. Condensate piping must keep "a minimum horizontal slope in the direction of discharge of not less than
one-eighth unit vertical in 12 units horizontal (1% slope)" (International Mechanical Code sec. 307.2.1, as adopted
in the Baltimore City 2024 building code). Laid over the supports, the hose falls all the way. The supports are off
the straight line between the unit and the drain, so a free-hanging hose does not find them by itself.

Two arms are natural (simultaneous hold): one lays the hose's middle on the low support (the part from the outlet
drops onto the high support) and holds it there while the other carries the nozzle to the drain, keeping the last
span short of slack, and seats it; one arm alone can't hold the hose up on the supports while seating the nozzle,
and the left arm can't reach the drain well while the right arm can't reach the high support.

## Materials (real-world setup)

- Indoor unit: a white box 12 x 10 x 20 cm with the hose clamped in a hole on its right face,
  hose centre 14 cm above the table.
- Hose: flexible PVC tube, 8 mm outside / 5 mm inside diameter, 56 cm from the outlet to a 30 x 14 x 14 mm grey
  plastic nozzle (12 g) pushed onto its end.
- Supports: two grey wooden blocks. High: 4 cm along the path x 6 cm across, 12 cm tall. Low (at the bend):
  6 x 6 cm, 9.5 cm tall.
- Drain: 11 x 11 x 1.5 cm grey plinth with a dark open box on it (inner 6.5 x 6.5 cm, 8 mm walls, 2 cm tall; rim
  3.5 cm above the table). Everything except the hose is taped to the table (positions in task.json "setup").

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cabledemo.py` and `vision.py` are in this folder (hose simulation, scripted-demo runner, top
  camera geometry for the demo).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Route the air conditioner's white condensate hose into the drain tundish so that it slopes downhill all the way from the unit to the drain, with no dip that would trap water: lay it over the two grey steps and put the nozzle inside the tundish. Then let go of the hose."
```

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

Gates (any one gives **0**):
- either gripper's TCP (centre between the finger pads) is within 3.5 cm of the hose or nozzle, or a gripper
  position can't be read;
- the nozzle centre is not inside the tundish (within its 6.5 x 6.5 cm inner opening, below the rim);
- the hose does not rest on both supports: for each support, some hose segment centre lies inside its footprint and
  0-15 mm above its top (the 8 mm hose's underside touching it or at most 1.1 cm above it).

Then the hose centre line's height profile is read at 30 points from the outlet to the hose end (outlet tip, every
2 cm segment centre, the end where it enters the nozzle). Horizontal run is measured along the hose's path seen from
above. Three numbers:
- **pocket** = the largest height by which any point stands above any point before it (towards the outlet). A dip
  followed by a climb of h holds back water h deep, so this is the depth of the deepest water pocket.
- **window slope** = for every stretch from one point to the first point at least 10 cm of run further on,
  fall / run; the smallest of these.
- **mean slope** = (outlet height - hose end height) / total run.

Ladder:
- **1.0**: pocket <= 1.5 mm **and** window slope >= 1/8 in per ft (1.04 %) **and** mean slope >= 1.04 %. No 10 cm
  stretch is level or climbing, and no pocket deeper than 1.5 mm. Why 1.5 mm: water held by such a rise is at most
  1.5 mm deep, under a third of the 5 mm bore, so the air path over it stays open and it cannot seal the hose like a
  trap; 1.5 mm is also about the reading error of a ruler or a side photo and above the simulator's settling noise
  (under 1 mm on a resting hose).
- **0.5**: not 1.0, but pocket <= 3 mm (a shallow sag or a level run that leaves standing water but still under the
  bore, so it does not seal).
- **0**: a pocket deeper than 3 mm (approaching the 5 mm bore: a trap).
- Real trials: 0 if the unit, a support or the drain is moved.
- `distance()`: nozzle-to-tundish-centre distance plus pocket (m). `progress()`: stage diagnostic (nozzle grasped,
  on each support, nozzle in drain, pocket <= 3 mm, pocket <= 1.5 mm, falls 1 % over every 10 cm, released).

Measuring a real trial: photograph the hose from the side against a ruler (or measure the hose centre height every
2 cm with a ruler), note heights to the millimetre, and compute the three numbers above.

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Camera-driven: the demo chooses its motions only from the top camera image, the arms' own poses and the fixed layout
(unit, supports, drain, hose length). It finds the grey nozzle and the off-white hose line on the table in the top
image, then: the left arm pinches the hose at the arclength that belongs on the low support (the straight-line
distance from the outlet to the support plus 6 mm, counted back from the seen nozzle along the seen hose line), the
right arm pinches the nozzle. Both lift; the left lowers its point onto the low support (the hose from the outlet
settles onto the high support) while the right carries the nozzle, wrist camera ahead, to 1 cm beyond the tundish
centre (so the last span hangs nearly straight) and 1 cm above the rim. The right lets go (nozzle drops in) and moves
back beside its base; then the left lets go, lifts straight up and swings out past the far side of the unit. If the
camera can't see the hose (a low-resolution debug render), it falls back to the reset positions on the setup sheet.
task_score 1.0 (success held 1 s, episode ends at step 282, ~14 s). Once the nozzle is in, the hose profile has no
pocket (0.0 mm), its smallest 10 cm window slope is about 8 % and its mean slope about 20 %. `demo/` holds its videos, `setup.png` and `result.json`.

Shortcut variants (`DRAIN_CHEAT=...`; run without video, where the 64 x 36 debug image is too small for the camera
step, so they start from the reset positions on the setup sheet instead of the camera estimate):
- `nothing`: no motion: **0** (nozzle not in the drain).
- `direct`: the right arm alone carries the nozzle into the tundish; the hose sags to the table and climbs 35 mm
  into the tundish: **0** (not on the supports; pocket 35 mm).
- `onestep`: the hose is picked where it lies on the table and laid on the high support only, then the nozzle is
  carried to the drain: **0** (not on the low support; the short hose also pulls the nozzle up onto the tundish rim).
- `onestep_taut`: tries to pick the hose where it hangs near the outlet to lay it taut over the high support; the
  hanging hose slips out of the fingers, so it ends as `direct`: **0** (pocket 35 mm).
- `hold`: the full demo, but the right gripper keeps holding the nozzle down in the tundish: **0** (not released).
- `sag`: the full demo, but the left arm takes 6 mm less hose and the nozzle goes 5 mm short of the tundish centre,
  so the last span sags to 38.6 mm and climbs to 40.8 mm just before the tundish wall: pocket 2.2 mm. It scores **0.5** (pocket 2.2 mm > 1.5 mm, <= 3 mm).
  With 9 mm more slack in the last span (`DRAIN_SAG_A=0.015 DRAIN_SAG_OFF=-0.02`) the pocket is 9.1 mm: **0**.

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). One trial takes a few minutes wall time on one GPU including the
  ~1-2 min Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the hose always starts straight (it is never jittered), so every trial
  starts identically. `VARIATION` lists offsets a later seeded runner may apply.

## Reset

Lay the hose straight from the outlet towards the robot's right along x 14 cm (it hangs from the outlet to the
table), nozzle flat on the table at y -19 cm. Check the unit, supports and drain are on their tape marks and the
tundish is empty.
