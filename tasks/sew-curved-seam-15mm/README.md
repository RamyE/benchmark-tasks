# Sew a curved seam on a sewing machine, steering the fabric so it stays 15 mm in from the edge

Instruction: `Sew the seam on the machine from the start mark to the end mark, 15 mm in from the curved edge, then stop the machine and let go of the fabric.`

A home sewing machine is sunk flush into a sewing table. Two basted layers of quilting cotton (24 x 20 cm, one corner
rounded) lie under the lowered presser foot with the needle down in the chalked start mark, the straight right edge on
the throat plate's 15 mm guide line. The robot presses Start/Stop, guides the fabric while the feed dogs draw it away,
turns it steadily round the curve so the cut edge stays on the 15 mm line, presses Start/Stop again when the end mark
reaches the needle, and lets go.

Sewing a seam at the standard 5/8 in (about 15 mm) seam allowance is the basic act of home sewing: dressmaking, mending,
curtains, cushions, bags and quilting all start with it, and every curved piece (a pocket corner, a cushion corner, a
bag bottom, a collar) has to be steered round on the machine like this.

What makes it hard:

- **The machine sets the pace.** The feed dogs draw the fabric through at a fixed rate (1 cm/s, 2.5 mm stitches) and
  the needle pins it: under the presser foot the fabric can only turn about the needle. The hands cannot move the
  seam point; they can only turn the fabric about it, continuously, at the right rate, as the curve passes the needle.
  Turn too little and the seam runs towards the edge, too much and it wanders into the fabric.
- **Guide, don't drag.** Pulling the fabric through or holding it back makes the stitches too long or bunches them (on a
  real machine it bends the needle). The hands have to move with the feed while they turn the fabric.
- **Both hands, on a moving, floppy sheet.** The fabric drags on the table and swings round as it is fed, so a hand
  that only follows it lets it drift. The hands take turns: the right hand presses Start/Stop (on the head, to the right
  of the needle) and steadies the straight right edge, the left hand turns the fabric round the curve by its far edge,
  and each lets go and takes a new hold before its wrist reaches the machine head. Neither may come within 3 cm of
  the running needle.
- **Stop on the mark.** The seam must end within 1 cm of the end mark, so Stop has to be pressed as the mark arrives,
  allowing for the button's travel.

Compared with public benchmarks: cloth tasks there (SoftGym, DeformableRavens, GarmentLab, ManipBench "Move Fabric
Hard" and similar) flatten, fold, move or hang a cloth that stays wherever the robot puts it. None has a machine that
feeds the cloth at its own rate and pins it at one point, where the robot must steer it continuously along a line on
the cloth; the nearest are fabric-flattening and fabric-moving tasks. Needle-threading tasks (MimicGen Threading,
DexMimicGen TwoArmThreading) push a rigid needle through a ring.

## Scene

Positions in cm from the table point below the midpoint between the arm bases (x forward, y left), heights above the
table.

| object | type | size | real part | position |
|---|---|---|---|---|
| sewing machine | fixed | modelled 41 wide x 17 deep x 31 tall: bed flush with the table, head 15 deep x 11 wide x 22 tall from 9 to 31 cm up, arm 20 wide x 15 tall, pillar at the right end | computerized home machine, Brother CS7000X class (about 41 x 17 x 31 cm), sunk in a sewing-table cutout (41.5 x 17.5 cm opening, shelf 9 cm down) so its bed is level with the top | needle at x 33, y -5; bed x 28 to 45, y -37 to 4; head front face at x 30, from y -16 to -5 |
| throat plate and guide lines | fixed (flush) | plate 7 x 10; lines at 10, 15 and 20 mm right of the needle, 5 cm long; the 15 mm line 3 mm wide, yellow | steel throat plate with engraved seam guide lines, the 15 mm line marked with yellow tape | around the needle |
| presser foot | fixed | 2.6 x 1.8 x 0.35 on a shank | standard zigzag foot, lowered | at the needle |
| finger guard | fixed | 4 mm wire loop, 2.6 wide, standing 2.2 cm in front of the needle, 1.2 cm above the bed | generic low-shank finger/needle guard (wire loop clamped under the presser-foot screw), fitted for every real trial | in front of the needle |
| Start/Stop button | sprung slide | 1.8 x 1.8 face, stands 5 mm proud of the head, 4 mm travel, clicks at 2.5 mm (about 1.4 N) | the machine's Start/Stop push button | head front, x 30, y -13.5, 12 cm up |
| fabric | cloth sheet | 24 x 20, near right corner rounded to 6 cm radius | two layers of quilting cotton, cut from a template and basted together | far end at x 36, right edge at y -6.5 (on the 15 mm line), needle down at the start mark |
| chalk marks | marks on the fabric | 3 cm ticks across the seam line, and the 15 mm sewing line itself (1.6 mm wide) | white tailor's chalk, drawn with a 15 mm seam gauge | start: on the right edge's 15 mm line, 3 cm from the far end; end: on the near edge's 15 mm line, 11 cm from the right edge (5 cm past the curve); the sewing line joins them, 15 mm in from the edge all the way round the curve |

The machine runs at a low setting on its front speed slider (about 240 stitches a minute, roughly a quarter up the
Brother CS7000X's 70–750 spm slider): 2.5 mm stitches, one every 0.25 s (1 cm of fabric a second). The seam is 27.1 cm
long: 15 cm straight along the right edge, a quarter circle of 4.5 cm radius round the corner (7.1 cm), then 5 cm
straight along the near edge. As it is sewn the fabric turns a quarter turn about the needle, so its far part sweeps
from in front of the machine to the left of it.

Reach: the fabric lies 12 to 36 cm in front of the arm bases; all grips are on its edges at table height, straight
down.

## Reset

Cut two 24 x 20 cm navy quilting-cotton pieces from the template (near-right corner rounded to 6 cm radius) or unpick
the seam, baste the two layers together, and chalk the 15 mm sewing line and both start and end marks. Thread the upper
thread and bobbin with contrasting topstitching thread (size 80/12 needle), lay the fabric with its right edge on the
15 mm line, and lower the needle into the start mark, then the presser foot. Set the speed slider to low (about 240
stitches a minute, roughly a quarter up from minimum) and the stitch length to 2.5 mm (about 1 cm of fabric a second).
Confirm the wire finger guard is clamped firmly around the needle and keep an accessible power/emergency stop within
the operator's reach throughout real trials.

## Scoring (real trial)

From the trial video and a ruler on the sewn seam afterwards. The 7 x 10 cm steel throat plate and 2.2 cm wire finger
guard frame the needle in the camera views so the 3 cm finger-clearance zone is directly visible on video. The thread
is a contrasting colour against the navy cotton, so a seam that leaves the chalked 15 mm line shows at a glance; a
ruler then measures each stitch length, where the seam runs and ends along the 27.1 cm chalked line (0 cm at the start
mark, 18.5 cm halfway round the curve, 27.1 cm at the end mark), and each stitch's perpendicular seam allowance from
the curved cut edge (right edge, 6 cm rounded corner, near edge).

Score 0 if:

- the fabric tears;
- any part of a gripper finger comes within 3 cm of the needle while the machine runs (the finger guard is fitted in
  real trials as well; the rule still applies);
- the machine is still running at the end;
- a gripper still holds the fabric, or for the last 0.5 s the grippers are not both open (2 cm or more), at least
  10 cm from the fabric and each moving under 2 cm/s;
- fewer than 2 stitches are made, the needle ever runs off the fabric, or any stitch after the first is under 1 mm or
  over 4 mm long (the fabric was pulled through or held back);
- the seam ends before 19 cm along the chalked line (before clearing halfway round the curve) or runs more than 1 cm
  past the start or end mark along the chalked line.

Otherwise the seam allowance from the curved edge is measured at every stitch:

- 1.0: every stitch is 12-18 mm from the curved edge (15 ± 3 mm) and the seam ends within 1 cm of the end mark along
  the chalked line;
- 0.5: every stitch is 9-21 mm from the curved edge (15 ± 6 mm) and the seam ends within 1 cm of the end mark along
  the chalked line;
- 0.25: every stitch is 8-22 mm from the curved edge (15 ± 7 mm), either ending within 1 cm of the end mark along the
  chalked line or stopping short along the chalked line after at least 19 cm (past halfway round the curve);
- 0 otherwise.

Every outcome is covered: doing nothing scores 0 (the machine never runs); starting the machine and letting the hands
ride along without turning the fabric runs off the curve before 19 cm and scores 0; cutting straight or diagonally
across the fabric to the end mark (or pivoting a sharp L-corner without steering round the curve) leaves the 8-22 mm
curved-edge allowance band and scores 0; one hand alone either never starts the machine or lets the fabric drift
outside the allowance band (0); never pressing Stop scores 0; still holding the fabric at the end scores 0; steering
cleanly past halfway round the curve within 8-22 mm before stopping short scores 0.25; completing the seam to the end
mark within 9-21 mm scores 0.5 and within 12-18 mm scores 1.0.

## Simulation notes

The fabric is a first-party reduced position-based cloth sheet (`fabric_sheet.py`, written in NVIDIA Warp with no
third-party assets; 8 mm cells) stepped with the rigid simulation, not a full cloth solver; one sheet stands in for the
two basted layers. It slides on the bed with friction. A jaw grips the fabric (a stand-in for pad friction) only on
two-pad contact when the jaws close across a fabric edge inside their 22 x 20 mm pad footprint, and lets go when it
opens past 23 mm or the fabric is pulled 3.5 cm away from it. The cloth pushes nothing back on the fingers: the two
pieces weigh about 15 g.

The feed, presser foot and needle are a rule model (`seam_machine.py`), not contact: while the machine runs, the
fabric within 15 mm of the needle is carried 2.5 mm per stitch away from the operator and turns only about the needle,
at a rate set by how hard the fabric outside the foot is pulled round (above a small static threshold); a hard pull
or hold-back along the feed makes it slip, so stitches come out longer or shorter. Stitches are recorded where the
needle enters the fabric; the thread is drawn along them but not simulated, and the needle's stroke is drawn only.
The machine and the finger guard are rigid and the arms collide with them and with each other; the cloth collides
only with the table and the fingers (it slides under the guard, 1.2 cm up, as on the real machine). The Start/Stop button is a sprung slide that
toggles the machine each time it is pushed past its click point and let back out. The model is uncalibrated.

## Requirements

yam-isaac-lab-arena with Isaac Lab-Arena 0.3.1 (the official YAM launchable). `fabric_sheet.py` (the cloth, needs
`warp`), `seam_machine.py` (the feed and stitch model) and `fabric_look.py` (its colours, marks, thread and needle) are
in this folder.
