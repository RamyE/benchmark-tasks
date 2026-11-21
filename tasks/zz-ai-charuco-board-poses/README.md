# Calibrate a robot cell's camera: show it a ChArUco board held still at every tilt, edge and distance

Instruction: `Calibrate the camera with the board, then put the board back in its stand.`

A robot cell has a fixed Intel RealSense D455 looking back at the robot. After a workcell camera is fitted, moved or
refocused, it is calibrated by holding a rigid calibration board in front of it in a set of still poses. Here the robot
does that calibration work itself: lift the 40 x 32 cm ChArUco board out of its slotted stand with both hands, hold it
still for the camera across the poses a calibration needs (straight on and tilted four ways, shifted to every edge of
the image, and both near and far, keeping the whole pattern inside the frame each time), and stand the 3 mm plate back
upright in its 4 mm slot.

## Why it matters (sources)

- OpenCV, *Calibration with ArUco and ChArUco*: the board is shown to the camera in varied poses and `calibrateCamera`
  is run on the detected ChArUco corners; boards are generated with `cv::aruco::CharucoBoard`, as this one is
  (`DICT_5X5_100`, 9 x 7 squares of 40 mm, 30 mm markers, 20 mm white margin).
  https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html
- ROS `camera_calibration`, *How to Calibrate a Monocular Camera*: the calibrator tracks four coverage bars (`X`, `Y`,
  `Size`, `Skew`) and asks for the target "on the camera's left, right, top and bottom of field of view", "toward/away
  from the camera", and "tilted to the left, right, top and bottom (Skew)", held still at each capture until the
  `CALIBRATE` button lights.
  https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
- Zhang, *A flexible new technique for camera calibration* (IEEE TPAMI 2000): pure translations of a planar target add
  no intrinsic constraint, so distinct board orientations are mandatory; radial distortion is strongest near the image
  border, so edge coverage pins down distortion coefficients; motion blur shifts corner detections, so each view must be
  held still.
- Intel RealSense D455 datasheet: RGB global-shutter sensor 1280 x 800 (~90 x 65 deg FOV), housing 124 x 26 x 29 mm.

Calibrating workcell cameras is routine hands-on work in robot building and lab bring-up, normally requiring a person
to stand at the cell walking a board through poses.

## How it differs from existing tasks

Existing calibration tasks on the board ("Calibrate a tool tip by touching one point from four angles" and "Capture
three calibration impressions on an optical robot fingertip") press a single held tool or fingertip against a fixed
point or pad, and external tasks such as RoboFactory `CameraAlignment-rf` or BEHAVIOR-1K `attach_a_camera_to_a_tripod`
aim or mount a camera body. Here the camera is fixed and nothing is touched during capture: both arms carry one 40 x 32
cm rigid plate by opposite side edges as a closed kinematic chain, move and tilt it together through free air across
orientations, image borders and depths while keeping all four pattern corners inside the camera's FOV and holding still
to 2 mm and 0.3 deg for 1 s per capture, then align and lower the 3 mm plate back into a 4 mm slot.

## What makes it physically demanding

- **Closed-chain carry:** one parallel gripper on one side edge cannot hold a 40 cm, 0.45 kg board flat; it twists and
  drops onto the table. Both hands must hold opposite side edges and move on coordinated arcs around the board's centre
  for every yaw, pitch and translation.
- **Uncovered pattern:** the grippers must stay in the outer 6 cm side bands (the 2 cm white margin or outer 4 cm
  column of squares) without drifting inward over the pattern.
- **Framing vs. edge and tilt coverage:** strong tilts (18-50 deg, cross-axis under 12 deg) and pushing the pattern into
  the outer 10% of the frame at all four borders pull right up against keeping all four pattern corners at least 10 px
  inside the 1280 x 800 image.
- **1 s stillness and narrow stowing:** each view requires under 2 mm translation and 0.3 deg rotation across a full
  second off all supports, followed by seating a 3 mm plate upright into a 4 mm-wide slot (with 6 mm 45 deg chamfers).

## Scene and materials (cm from arm-base midpoint; x forward, y left, z above the table)

| Item | Real-world part & material | Dimensions / mass | Pose |
|---|---|---|---|
| Camera | Intel RealSense D455 screwed via its bottom `1/4"-20` tripod thread onto a 2 cm cylindrical steel post clamped/bolted at `(64, 0)` | Body 124 x 26 x 29 mm; post 2 cm dia, 23.55 cm tall | Optical centre at `(62, 0)`, 25 cm up, level, looking along `-x` (`1280 x 800`, `fx = fy = 640 px`, `cx = 640`, `cy = 400`) |
| Board | OpenCV `CharucoBoard` (`DICT_5X5_100`, 9 x 7 of 40 mm squares, 30 mm markers, 36 x 28 cm pattern, 2 cm margin) matte UV-printed on 3 mm white aluminium composite sheet (Dibond/ACM, ~3.5 kg/m^2) | 40 x 32 x 0.3 cm, 0.45 kg | Upright on its 40 cm edge in the stand at `(30, 0)`, bottom at `z = 1.0 cm`, pattern facing `+x` (toward camera) |
| Stand | Hardwood block clamped/bolted at `(30, 0)` with a 4 mm table-saw kerf along `y` and 45 deg router chamfers | 20 x 8 x 4 cm (`y` x `x` x `z`); slot 4 mm wide, 3 cm deep (floor 1 cm up), 6 mm 45 deg mouth chamfers | Centred at `(30, 0)` on the table |

## Run a trial

Requirements: `yam-isaac-lab-arena` on Isaac Lab-Arena 0.3.1 (`opencv-contrib-python` / `cv2.aruco` renders the board texture).

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1800 \
    --instruction "Calibrate the camera with the board, then put the board back in its stand."
```

## Scoring

**Score 0** (latched over the whole trial) if at any step:
- any corner of the board comes within 4 mm of the table outside the stowed slot state (dropped on the table); or
- the board comes within 3 cm of the 124 x 26 x 29 mm D455 camera housing.

A **held view** is a continuous 1.0 s window (21 consecutive steps, i.e. 20 control intervals at 20 Hz) throughout which:
- all four corners of the 36 x 28 cm pattern project inside the 1280 x 800 image with at least a 10 px margin (`depth > 10 cm`) and the pattern face points toward the camera;
- both grippers hold opposite side edges of the board: each arm's TCP and pad-tip midline lie within 1.2 cm of the 3 mm plate and within the outer 6 cm on its side (`|y_board| >= 14 cm`, one arm on `+y`, one on `-y`);
- the board's lowest corner is at least 2 cm above the 4 cm stand top (`>= 6 cm` above the table); and
- every step in that 1.0 s window stays within 2 mm and 0.3 deg of the window's final board pose (an orientation item requires that orientation class on all 21 steps of the window; an image-coverage or distance item requires that bound on all 21 steps of the window).

**Eleven calibration items** (`0.08` each, up to `0.88`), each latched once shown by any held view:
- **Orientation (`ROS Skew`):** from the board's pattern normal `n` in the OpenCV camera frame (`x` right, `y` down, `z` forward; `yaw = atan2(n_x, -n_z)`, `pitch = atan2(-n_y, -n_z)`):
  1. `straight`: `|yaw| < 8 deg` and `|pitch| < 8 deg`;
  2. `left`: `-50 <= yaw <= -18 deg` and `|pitch| < 12 deg`;
  3. `right`: `18 <= yaw <= 50 deg` and `|pitch| < 12 deg`;
  4. `up` (top edge leaned back): `18 <= pitch <= 50 deg` and `|yaw| < 12 deg`;
  5. `down` (top edge leaned toward camera): `-50 <= pitch <= -18 deg` and `|yaw| < 12 deg`.
- **Image coverage (`ROS X` and `Y`):** projected pattern corners reach the outer 10% of the 1280 x 800 image on the (6) `image left` (`u <= 128 px`), (7) `image right` (`u >= 1152 px`), (8) `image top` (`v <= 80 px`), and (9) `image bottom` (`v >= 720 px`).
- **Distance (`ROS Size`):** projected horizontal pattern width `(u_max - u_min) / 1280` is (10) `>= 60%` (`near`) and (11) `<= 50%` (`far`).

**Full score (`1.0`):** all eleven items latched, no disqualifier, and at the end the board rests upright in the slot (horizontal centre within 1 cm of `(30, 0)`, lowest corner within 4 mm of the slot floor 1 cm above the table, upright tilt `<= 8 deg`, pattern toward the camera with `r_xx > 0.9`, speed `<= 1 cm/s`, spin `<= 0.2 rad/s`) while both grippers (TCP and pad-tip midline) are at least 10 cm clear of the board and still (each TCP staying within 1 cm of its pose 0.5 s earlier, i.e. `<= 2 cm/s`). Holding success for 1 s ends the run early.

**Real-trial measurement:** record the D455 1280 x 800 colour stream at 20 fps plus overhead and side video. Run OpenCV `CharucoDetector` on the D455 frames: a 20-frame (1 s) window counts when at least 40 of the 48 inner corners are detected, the four outer pattern corners projected from `solvePnP` stay `>= 10 px` inside the image, `solvePnP` poses across the 20 frames agree within 2 mm and 0.3 deg, and overhead/side video shows both grippers on opposite 6 cm side bands with the board `>= 2 cm` clear of the stand. Orientation, edge bands and width fraction come directly from those `solvePnP` poses; drops, 3 cm camera clearance, final slot seating and 10 cm parked gripper clearance are checked from video and a ruler.

## Demo video and shortcut checks

`demo/` shows a scripted bimanual run reaching `1.0` in about 1,150 steps (~57 s at 20 Hz): both hands grip opposite side edges, lift 8 cm clear of the slot, carry the board to `x = 26 cm` (`36 cm` from the D455) and hold straight (far), yaw `+23` and `-23 deg`, pitch back `26 deg` and forward `21 deg`; then step in to `x = 34 cm` (`28 cm` from the D455, near) and hold shifted `8 cm` right (`image left`), raised `2.5 cm` (`image top`), and shifted `8 cm` left (`image right`), pausing `1.8 s` still at each pose; then align over the slot, lower the plate onto the slot floor, release, withdraw and park clear and still.

Scripted shortcut and partial policies scored on the final task code:

| Variant | Behaviour | Score |
|---|---|---|
| idle | Leaves the board in its stand and does nothing | 0.0 (not lifted >= 2 cm above the stand or gripped on both edges) |
| one-arm | Grips one side edge with the left hand only and lifts | 0.0 (single-hand edge grip drops the 40 cm plate onto the table) |
| shaky | Carries the board through all eight poses pausing only 0.2 s at each, then stows it | 0.0 (never holds still within 2 mm and 0.3 deg for a full 1 s) |
| partial views | Captures only the first three poses (straight far, yaw+, yaw-) and stows the board | 0.40 (5 of 11 items latched) |
| never stowed | Captures all eleven items across the eight held poses, then keeps holding the board in mid-air | 0.88 (all 11 items latched, board not returned to its slot) |

## Sim fidelity

The board is one rigid plate of true size (`40 x 32 x 0.3 cm`) and mass (`0.45 kg`), static/dynamic friction `0.9 / 0.8`, held purely by the parallel gripper pads' contact friction with no weld or attach constraint. The stand (with fixed 45 deg bevel colliders at the slot mouth), post and D455 body are fixed rigid colliders. The OpenCV ChArUco texture is rendered onto a visual mesh on the board's `+x` face for the cameras; held views are evaluated geometrically from the board pose through the D455 colour intrinsics rather than running `CharucoDetector` on rendered frames in the loop.
