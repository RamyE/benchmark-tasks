# Play Twinkle Twinkle's first verse on a classical guitar: fret with one hand, pluck with the other

Instruction: `Play the first verse of Twinkle Twinkle Little Star on the guitar (C C G G A A G, F F E E D D C) in a steady beat, then move both hands away from the guitar.`

Category: Societal relatability. The guitar is the most played instrument after the piano, and picking out a nursery
tune on it is how most people first play one. The verse needs both hands doing different things in time: the fretting
hand presses the right string onto the right fret, moves along and across the neck between notes and lifts off for the
open E; the plucking hand plucks the right string over the soundhole, crossing between two strings, in a steady beat.
A note pressed too lightly, on the wrong fret or string, or plucked off the beat is heard at once. One hand alone cannot
play it: the fret positions (frets 1-5, 0.02-0.15 m from the nut) and the plucking point (0.48 m from the nut) are
0.33-0.46 m apart along the strings.

## The tune on the guitar (standard tuning E A D G B E, C major)

| Note | C4 | D4 | E4 | F4 | G4 | A4 |
|---|---|---|---|---|---|---|
| String, fret | B, 1 | B, 3 | high E, open | high E, 1 | high E, 3 | high E, 5 |

Verse (quarter notes, the 7th and 14th notes are half notes): `C C G G A A G | F F E E D D C` = B1 B1 E3 E3 E5 E5 E3 |
E1 E1 E0 E0 B3 B3 B1. Beats: 0 1 2 3 4 5 6 | 8 9 10 11 12 13 14.

## Requirements & assets

- `yam-isaac-lab-arena` and Isaac Lab-Arena 0.3.1 with `inspect-robots`.
- No third-party assets: all geometry is built from USD primitives in `task.py`.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 900 \
  --instruction "Play the first verse of Twinkle Twinkle Little Star on the guitar (C C G G A A G, F F E E D D C) in a steady beat, then move both hands away from the guitar."
```

Run limit: 900 steps (45 s at 20 Hz). The layout is fixed (`object_jitter=0`); nothing is randomised. A trial takes
about 3-5 min of wall time on one GPU.

## Scene (metres; origin on the table top midway between the arm bases, x forward, y to the robot's left, z up)

| Object | Real product / build | Size, material | Pose |
|---|---|---|---|
| Guitar | Yamaha C40 classical guitar (Yamaha spec: 650 mm scale, 52 mm nut width, spruce top, meranti back and sides), nylon strings in standard tuning | Body 485 mm long, lower bout 370 mm, upper bout 280 mm, 100 mm deep; neck joins at the 12th fret | On its side as held to play: soundboard vertical, its face at x = 0.400, facing the robot; neck along y, nut at y = -0.220 (headstock to the robot's right), saddle at y = +0.430; strings' centre line z = 0.197; high E string lowest |
| Strings | Nylon (high E 0.71 mm, B 0.82 mm; wound bass strings) | Spacing 8.6 mm at the nut to 11.6 mm at the saddle; string bottom 0.5 mm over the fret crowns at the nut, 3.0 mm at the 12th fret; frets 1.2 mm high | Fingerboard face at x = 0.394 |
| Cradle | Two black closed-cell EVA foam blocks, taped to the table and to the guitar | Under the lower bout 140 x 80 x 12 mm; under the neck at the 10th-11th frets 60 x 50 x 171 mm | Centred at (0.450, 0.405) and (0.409, 0.080) |

Fret positions from the nut follow the equal-tempered rule `d_n = 650 mm x (1 - 2^(-n/12))`: fret 1 at 36.5 mm,
fret 3 at 103.4 mm, fret 5 at 163.0 mm. The plucking point is 480 mm from the nut (over the soundhole's bridge side).
Arms start at the ready pose with grippers open. Nothing else is on the table.

## Scoring (`score()`, deterministic; the same rules on a real trial from audio + video)

- **Pluck**: a string over the soundhole pushed at least 2.0 mm toward the soundboard by a fingertip and then let
  go by slipping off that fingertip: it springs back below 0.5 mm within 0.1 s (2 control steps) while the fingertip
  either slid at least 2 mm across the strings or was pulled back out by less than 1 mm (still pressed in) since the
  push. This is the classical rest/free stroke. Pressing the string and pulling straight back (the string follows the
  finger and does not sound), or easing it back slowly, is not a pluck.
  *Real trial*: an audible note in the recording (a clip-on tuner or phone tuner app reads its pitch; the 14-note
  sequence is checked against the audio).
- **Pitch**: the plucked string's open note raised by the highest fret space in which it is pressed onto the fret
  (within 0.15 mm of the fret crown) at the moment it is let go. If a fret space beyond that one is pressed more than
  0.2 mm but not onto its fret, the note is **dead** (buzz). *Real trial*: tuner reading; buzz or muted note = dead.
- **Counting**: plucks are compared with the 14 notes in order; the count stops at the first wrong-pitch, wrong-string
  or dead note.
- **Timing**: a tempo is fitted (least squares) to the counted notes' times against their beats. If it is outside
  60-100 bpm the count is 0; otherwise the count stops at the first note more than 150 ms from its fitted beat.
- **Score** = counted notes / 14 x 0.9 (e.g. the first line, 7 notes = 0.45).
- **1.0** when all 14 notes count, no further pluck follows, and at the end both grippers' fingertips are at least
  5 cm in front of the strings (toward the robot) and still (moved at most 1 cm over the last 1.0 s).
- A real trial in which the guitar leaves its foam blocks or tape marks scores 0 (fixed in simulation).

Possible scores: 0, multiples of 0.9/14 (0.064) up to 0.9, and 1.0.

## Simulation fidelity

- Guitar body, neck, frets and fingerboard are fixed rigid shapes at true size (body outline as two overlapping discs
  for the upper and lower bouts).
- Strings are 1 mm rigid bars (real 0.71-1.1 mm). On the G, B and high E strings, each fret space from the nut to
  fret 7 is a separate segment on a sprung slider toward the fingerboard; its stiffness is the transverse stiffness of a
  string under tension `T x (1/a + 1/b)` (a, b = distances to nut and saddle; T = 51 N for G, 54 N for B and 72 N
  for high E, normal-tension nylon), and its travel is the string's height over the fret crowns at that point (0.64 mm at fret 1,
  1.64 mm at fret 5). Every string has a 30 mm segment over the soundhole on a sprung slider (travel 4.5 mm). Fretting
  and plucking are therefore real contacts of the fingertips with these segments.
- Pitch and buzz are computed from the segment positions as above; no string vibration or sound is simulated. Strings
  are parallel to the neck at their mean spacing (real strings fan out by 3 mm over the scale).
- The grippers touch the strings with their fingertips; nothing is grasped.

## Scripted demo and shortcut checks

A scripted demo (not part of the submitted files) plans its moves from the known guitar geometry and the live
fingertip poses. The right hand frets with its closed fingertips; the left hand plucks with its closed fingertips,
pressing each string and sweeping down across it so it slips off the fingertip. It plays at 66.7 bpm (0.9 s per beat), then both
hands move back and park. It scores 1.0 (437 of the 900 control steps).

| Variant (same scene) | What it does | Score |
|---|---|---|
| none | arms stay at the start pose | 0.0 |
| one arm (pluck only) | the left hand plucks every note, the fretting hand never presses | 0.0 (first note is an open B, not C) |
| fret only | the right hand frets every note, nothing is plucked | 0.0 |
| wrong fret | the 5th note fretted at fret 4 (G#) instead of 5 (A) | 0.257 (4 notes) |
| too slow | correct notes at 41.7 bpm | 0.0 (tempo outside 60-100 bpm) |
| press and pull back | correct notes, but each string is pressed and pulled straight back, with no sweep across it | 0.0 (no string slips off a fingertip: no note) |
| hands stay | correct notes, but both hands stay at the strings | 0.9 (no 1.0 without hands clear) |
