# Put a raincoat on a small-dog form and fasten its neck and belly straps under the body

Instruction: `Put the raincoat on the dog and fasten both straps, then let go.`

A rigid small-dog form stands side-on (`+y` head). Behind it on tape marks stands a lined ripstop raincoat in its saddle arc (yellow shell up, navy neck `+y`). The robot lifts the coat by its spine tab, seats it centred on the back (neck forward, grey lining in), and fastens both straps underneath: the neck strap under the chest ahead of the front legs, and the belly band under the belly between the front and rear legs.

## What it tests

1. **Oriented draping:** seating a curved garment centred on a cylindrical torso, neck forward and lining in, clear of tail and head.
2. **Under-body strap retrieval:** reaching into the `17 cm` clearance under the chest and belly between the legs to pinch each far-side strap tip (`x ≈ 44 cm`, `~59 cm` from the opposite arm base, requiring left arm for neck and right arm for belly).
3. **Hold against pull and press:** drawing each `16 cm` strap under the barrel and pressing its hook patch onto the near-side loop patch while the other hand holds the `132 g` coat centred on the spine.

## How this differs from existing benchmarks

Public suites fold/hang flat garments (`DexGarmentLab` *Hang Coat*, `BEHAVIOR-1K` *clean_a_raincoat*), drape a scarf (*Wear Scarf*), close a shoe strap (`ALOHA` *Put On Shoe*), or thread a Velcro tie (*Thread Velcro*). None drapes a coat on a standing quadruped and reaches underneath between its legs to fetch far-side straps and press them onto the near flank while holding the coat centred.

## Scene and reproducible hardware

Coordinates in cm from table top below arm-base midpoint (`x` fwd, `y` left, `z` up; bases `(0, ±30.5)` in `READY`).

### 1. Small-dog display form (fixed, rigid, 38 cm chest girth, 28 cm torso)

White 3D-printed PLA union of 11 capsules (20% infill, ~480 g; or shimmed commercial dog mannequin), bolted through its four feet on tape marks:

| part | `a` `(x, y, z)` cm | `b` `(x, y, z)` cm | `r` (`dia`) cm | notes |
|---|---|---|---|---|
| `body` | `(38.0, -10.0, 23.0)` | `(38.0, +6.0, 23.0)` | `6.0` (`12.0`) | girth `37.7`; caps `-16..+12`; belly `17`, back `29` |
| `neck` | `(38.0, +8.0, 25.0)` | `(38.0, +14.0, 31.0)` | `3.5` (`7.0`) | neck base `y = 8.0` |
| `skull` | `(38.0, +15.5, 34.0)` | `(38.0, +17.5, 34.2)` | `4.0` (`8.0`) | |
| `muzzle` | `(38.0, +19.0, 33.0)` | `(38.0, +23.5, 32.0)` | `2.0` (`4.0`) | |
| `ear_l` | `(35.2, +15.0, 37.0)` | `(34.5, +15.5, 39.5)` | `1.0` (`2.0`) | top `z = 40.5` |
| `ear_r` | `(40.8, +15.0, 37.0)` | `(41.5, +15.5, 39.5)` | `1.0` (`2.0`) | top `z = 40.5` |
| `tail` | `(38.0, -15.0, 24.5)` | `(38.0, -20.0, 27.0)` | `1.0` (`2.0`) | |
| `leg_fn` | `(35.0, -1.0, 1.5)` | `(35.0, -1.0, 23.0)` | `1.5` (`3.0`) | `y = -2.5..+0.5` |
| `leg_ff` | `(41.0, -1.0, 1.5)` | `(41.0, -1.0, 23.0)` | `1.5` (`3.0`) | |
| `leg_rn` | `(35.0, -16.0, 1.5)` | `(35.0, -16.0, 23.0)` | `1.5` (`3.0`) | `y = -17.5..-14.5` |
| `leg_rf` | `(41.0, -16.0, 1.5)` | `(41.0, -16.0, 23.0)` | `1.5` (`3.0`) | |

Two `7 mm` black sphere eyes at `(35.5, +18.8, 35.0)` and `(40.5, +18.8, 35.0)` cm.

### 2. Raincoat, straps and hook-and-loop patches (free, 132 g: 108 g body + two 12 g straps)

Cut and sew (1:1 rectangular pattern, ~3 mm thick):
- **Body (`30.0 cm` spine `v = 0..30 cm` tail-to-neck x `32.0 cm` across `u = 0..32 cm` `-x..+x`):** yellow 70D ripstop shell, grey lining, `2 mm` EVA heat-formed to an `8.0 cm`-radius arc (`11.6 cm` apex on table, edges `14.5 cm` apart), `1.6 cm` navy neck binding (`v = 28.4..30.0 cm`), spine seam (`u = 16.0 cm`), and three `2.4 x 1.2 x 2.2 cm` folded `2.5 cm` navy webbing spine tabs at `v = 3, 15, 27 cm`.
- **Straps (`16.0 cm` black webbing, `12 g` each: `2.5 cm` neck at `v = 28.0 cm`, `3.8 cm` belly at `v = 13.5 cm`):** sewn to the `+x` edge (`u = 32.0 cm`), with `5.5 cm` (`1 mm`) black hook tape on the lining side (`10.5..16.0 cm`) and an outer white stitch line `5.5 cm` from the tip.
- **Loop patches (`5.0 x 4.0 cm`, `1 mm` grey loop tape):** sewn outside the `-x` edge over `u = 0.2..5.2 cm` at `v = 28.0, 13.5 cm`.
- **Start pose on marks:** centred on `x = 38.0 cm`, edges at `x = 30.7, 45.3 cm` over `y = -55.0..-25.0 cm` (navy neck `+y`, yellow shell up, `spine z = 11.6 cm`), straps folded flat up the `+x` outside (hook out).

## Scoring

Evaluated at trial end (`SCORE_OVER = "end"`). Under the belly (`~11.5 cm` span between lower coat edges at `z ≈ 18 cm`), each `16 cm` strap reaches `~3.5–4 cm` onto its loop patch; over the `32 cm` back (`~30 cm` path) it cannot.

**Score `0`** if any holds (states below `0.25` also score `0`):
- form moved `> 2.0 cm` from its marks (fixed in sim), or either gripper holds the coat or a strap at trial end;
- coat is backwards/twisted (`R[1,1] < 0.5`, `> 60°` from `+y` on any of the 6 middle spine panels) or inside-out/rolled (`R[2,2] < 0.5`, `> 60°` from `+z`);
- coat is off the back (or held above it): any of the 3 spine-seam midpoints (`u = 16.0 cm`, `v = 5, 15, 25 cm`) is `> 5.0 cm` off `x = 38.0 cm`, outside `y = -19.0..+15.0 cm`, or outside `z = 27.0..32.0 cm` (`z = 29.0 cm` back).

| score | condition |
|---|---|
| `0.25` | Coat on back in right orientation (`on_back` and `orientation_ok`). |
| `0.50` | Also **centred**: all 3 spine midpoints within `3.0 cm` of `x = 38.0 cm`, neck-end spine within `3.0 cm` of `y = 8.0 cm`. |
| `0.75` | Also **at least one strap fastened under the body**: passes under the belly (`z < 17.5 cm`, `<= 5.0 cm` off `x = 38.0 cm`) in its leg gap (neck `y = +0.5..+14.0 cm`; belly `y = -14.5..-2.5 cm`) with `>= 3.0 cm` hook pressed engaged on its loop patch. |
| `1.00` | **Both** straps fastened under the body, coat centred, and **both grippers open and parked**: both jaws `>= 4.0 cm` open (`joint7, joint8 <= -2.0 cm`), nothing held, every gripper point (pads through wrist housing) `>= 10.0 cm` from form and coat, and speed `< 2.0 cm/s` for the final `0.5 s`. |

### Real-trial measurement protocol

1. **Overhead photo + ruler:** verify yellow up, navy binding `+y`, feet `<= 2.0 cm` from marks, spine within `3.0 cm` (`0.50+`) or `5.0 cm` (`0.25`) of `x = 38.0 cm` over `y = -19..+15 cm`, and neck-end seam within `3.0 cm` of `y = 8.0 cm`.
2. **Side belly photo (`z = 17 cm`, looking `+x`) + ruler:** verify each strap spans under the body in its leg gap, measure `>= 3.0 cm` of hook strip (tip to `5.5 cm` white stitch line) flat on the `5.0 x 4.0 cm` loop patch, and lightly tug after trial end to confirm pressed engagement.
3. **Video (final `0.5 s`, 10 frames at 20 Hz) + `10 cm` block:** confirm `>= 4.0 cm` open jaw gap, `< 1.0 cm` motion over final `0.5 s`, and `>= 10.0 cm` clearance from form and coat.

## Simulation notes

- **Form & coat:** fixed 1:1 capsule form (`0.6 / 0.5` friction). Coat = 18 rigid `3 mm` panels (`3 x 10 cm` by `6` strips, `1 mm` contact offset) on sprung hinges (`0.05 / 0.03 N m/rad`, damping `3e-4`) + three fixed `2.4 x 1.2 x 2.2 cm` spine webbing tabs (`108 g` body). Straps = four `4.0 cm` hinged black webbing links (`3 g`).
- **Two-pad opposing pinch (pad-friction stand-in):** engages only on two-pad contact when **both** `tip_left` and `tip_right` register PhysX normal contact force (`>= 0.05 N` each) and their 3D hulls contact opposite sides of a tab, panel or strap link (`dL, dR <= 1.5 mm`) under active inward PD clamp (`qt - q >= 0.4 mm` each, `>= 1.6 mm` total); commanding jaws open (`qt < q - 2.0 mm` or `qt <= -2.0 cm`) releases at once.
- **Pressed hook-and-loop:** welds when `>= 3.0 cm` of `1 mm` hook contacts `1 mm` loop (`<= 1.0 mm` gap, `<= 30°`) while a finger hull contacts the strap over the patch (`<= 3.0 mm`) with `>= 2.0 mm` inward normal PD deflection for `0.25 s`.

## Reset

Peel both straps, stand the coat on marks (`x = 30.7, 45.3`, `y = -55..-25 cm`, yellow up, navy `+y`), fold straps up `+x` outside, arms `READY`.

## Run a trial

Requires `yam-isaac-lab-arena` (`0.3.1`) and `inspect-robots`:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 \
  -E cam_height=176 --scorer task_score --max-steps 1500 \
  --instruction "Put the raincoat on the dog and fasten both straps, then let go."
```

Run limit: `1,500` steps (`75 s` at `20 Hz`, `object_jitter=0`).

## Demo video

The scripted policy plans from live poses using arm joint commands (~963 steps): right drapes the coat centred on the back; right holds `tab_0` while left reaches under the chest, pinches the neck strap, wraps it under, and presses it onto its loop patch; left holds `tab_2` while right reaches under the belly, fetches the belly band, and presses it onto its loop patch; both grippers open and park `> 13 cm` clear aimed at the fastened straps for `> 1 s`.

Ablations (`score()`): idle (`0.0`), off-centre (`0.0`), left alone (`0.0`), right alone (`0.0`), held at end (`0.0`), drape only (`0.50`), no press (`0.50`), neck only (`0.75`).
