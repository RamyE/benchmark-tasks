# Restock two milk lanes in a dairy case, oldest code date in front

When a stocker brings new milk to the dairy case, the new cartons go behind the ones already on the shelf. Shoppers
take from the front, so older stock left behind new stock goes past its date and is thrown away. Each lane in this
case holds two cartons. One older carton is already standing at the front of each lane, so the usual way to get the
older carton in front of the new one is to take it out, set the new carton in the back, and put the older carton
back at the front.

Instruction: `Put the two new cartons of milk on the shelf.`

The instruction doesn't mention dates or order. A stocker knows to rotate; the robot has to read the code dates
printed on the gables (NOV 18 on the cartons in the case, NOV 27 on the new ones) and work out the order itself.

## Who does this and why it matters

- **Occupation:** stockers and order fillers (O*NET 53-7065.00). Their task list includes: "Stock shelves, racks,
  cases, bins, and tables with new or transferred merchandise." and "Determine proper storage methods,
  identification, and stock location, based on turnover, environmental factors..." (O*NET OnLine, Summary Report
  for 53-7065.00). Milk is restocked often because it is perishable and
  dated, so rotation errors turn straight into thrown-away stock.
- **The practice:** the Illinois State Board of Education's food-safety handout *Stock Rotation Plan* (FCS 700108)
  says: "First in, first out or FIFO is the stock rotation method of dating and arranging new stock behind (or
  below) older stock." "At delivery, items already on the shelf are physically moved forward, with the newest items
  placed behind." and, under *Front of the Case*: "Grocery store refrigerated units place milk with the oldest
  expiration date in the front of the display."
- **Why it is hard for a robot:** the shelf constrains the order of the moves. A lane holds exactly two cartons, and
  its back slot is walled in on three sides by the rear lip and the 12.5 cm cartons in the lanes on both sides. The
  practical way in, and the way stockers do it, is a fixed sequence:
  1. draw the old carton forward out of the lane and up over the 2.5 cm lip without tipping it, then park it upright
     on the cart;
  2. carry the new carton in over the lip, low over the empty 18 cm lane, and set it down upright in the back slot.
     The lane is only 7 mm wider than the carton on each side, so a twist of more than about 12 degrees jams it on a
     divider;
  3. set the old carton back in the front slot, against the lip, without pushing the new one over.

  The alternative, lifting the new carton above the old one and lowering it straight down into the back slot, is
  allowed, but it needs the carton held more than 12.5 cm up and lowered into a pocket only 1.45 cm wider than the
  carton, with the old carton just in front, walled in by other cartons. Each step also has to keep clear of the full lanes on both sides of each working lane. That is a constrained
  order plus precise position control in a narrow, walled lane, not just placing items upright. Reading the code date
  only says which carton is which; the hard part is the physical sequence that the shelf imposes.

## Scene (sim and real)

Table top = reference; x forward from the line through the arm bases, y left of their midpoint (cm).

- **Shelf deck:** a refrigerated-case shelf deck (0.5 cm thick) at table level, x 20.5-53, y -29.3 to +29.3. A front
  price-channel lip (x 20.5-21.5, 2.5 cm tall), a rear lip (x 39.5-40.5, 5 cm tall), the case's back wall behind the
  air-return gap (x 52-53, 30 cm tall), all fixed. The deck is one section from the middle of a lineup of dairy
  cases, so it has no end panels: the next sections of the lineup would continue past y +/-29.3.
- **Lane dividers:** wire dividers, 3 mm thick and 4 cm tall, running front to back at y -28.45, -20.15, -11.5, -4, +4,
  +11.5, +20.15 and +28.45 (the outer two mark the section's edges). That makes seven lanes 18 cm deep (x 21.5-39.5): the 2% lanes 8.35 cm wide inside, the
  skim lanes 7.2 cm, the chocolate lane 7.7 cm and the whole-milk lanes 8 cm.
- **2% lanes (y +15.8, y -15.8):** 2% milk. One older carton (code date NOV 18) stands at the front
  of each, 2 mm behind the lip (x 21.7-28.6). The back slot of each lane is empty.
- **Other lanes:** the middle lane (y 0) holds two chocolate milk pints and the lanes beside it (y +7.75, y -7.75)
  two skim pints each, front and back. The outer lanes (y +24.3, y -24.3) have sold down to one whole-milk pint each,
  at the back. They aren't part of the job, but they must not be knocked over or moved. The back cartons on both
  sides wall in each working lane's back slot, so the slot can't be reached from the side over a 4 cm divider. The
  two working lanes are 31.6 cm apart, so each arm works its own lane.
- **Stocking cart:** a grey cart deck, 2 cm tall, x 6-20, y -30 to +30, in front of the case. Two new 2% pints (code
  date NOV 27) stand on it at x 15, y +8 and y -8. The rest of the deck is free space for setting a carton down.
- **Cartons:** pint gable-top cartons, 6.9 x 6.9 cm base, 12.5 cm to the top of the gable fin, 500 g. The code date is
  printed in 2 cm black characters on the sloped gable panel facing the aisle.

The back slot of each 2% lane is boxed in: rear lip behind, the next lanes' back cartons on both sides, and the old
carton in front. A new carton reaches it either through an emptied lane or by being lowered straight down from above
the old carton into a pocket about 11 cm deep and 8.35 cm wide.

## Materials (real-world setup)

- **Shelf:** a standard reach-in cooler or multideck dairy shelf, 16 in deep, with its price-channel lip (for
  example a True or Hussmann replacement shelf). Clamp it to the table and screw a white back wall onto
  it. No end panels: the shelf stands for one section in the middle of a case lineup.
- **Dividers:** standard 4 cm wire shelf dividers that clip onto the shelf front and back, as sold for cooler
  shelves.
- **Cartons:** twelve empty pint gable-top milk cartons from any dairy, rinsed and dried. Each has a 450 g steel disc
  glued to the inside of its base and its gable sealed shut with clear tape. They contain no liquid. Stick printed
  code-date labels on the gable (NOV 18 on two 2% cartons, NOV 27 on two 2% cartons; two chocolate, four skim and two whole-milk cartons as the neighbouring stock).
  Dummy display cartons sold for store fixtures also work, weighted the same way.
- **Cart deck:** a grey plastic board, 2 cm thick, standing in for the stocking cart's deck.

## Scoring (end of trial)

A lane is **correct** if, at the end:

- it holds one old (NOV 18) and one new (NOV 27) 2% carton, each standing on its base on the shelf deck between
  that lane's dividers, tilted at most 10 degrees;
- the old carton's centre is nearer the lip than the new carton's;
- the old carton's front face is within 3 cm of the lip.

**Score 0** if, at the end:

- any carton is lying down or tipped against a divider or another carton, or stands anywhere other than the shelf
  deck or the cart (a dropped carton);
- a chocolate, skim or whole-milk carton has moved more than 1 cm or is tipped;
- a gripper is within 3 cm of any carton (not let go).

**Otherwise:** 0.5 for each correct lane.

| Score | End state |
|---|---|
| 0 | Neither lane correct (for example both lanes have new in front of old, or the new cartons are left on the cart) |
| 0.5 | One lane correct; the other is not (left as it started with its new carton still on the cart, wrong order, or its old carton more than 3 cm from the lip) |
| 1.0 | Both lanes correct |

**Real-trial check:**

- take one photo from above the case, and read off each lane's two code dates front to back;
- measure the old carton's front face to the lip with a ruler (3 cm or less);
- check upright and on the deck by eye from the photo;
- put two tape marks on the deck beside each chocolate, skim and whole-milk carton's starting position, and check from the photo that
  none has moved past its marks;
- check from the video that the grippers are open and away at the end.

## Demo video

The full-score demo works one lane at a time, left then right, each with one arm, following the fixed layout. The arm
draws the NOV 18 carton forward out of the lane over the lip and stands it on the cart, picks up the
NOV 27 carton, carries it in low over the empty lane and sets it down in the back slot, then picks the NOV 18 carton
up again from the cart and sets it in the front slot, between the lip and the new carton. It lets go and lifts clear.
None of the neighbouring cartons is touched. It ends at 1.0 in about 2,100 steps.

Shortcuts checked in simulation, each scored by the same rules:

- doing nothing: 0 (the new cartons are still on the cart);
- putting the old carton at the back and the new one in front, in both lanes (all four cartons on the deck,
  upright, wrong order): 0;
- the right order in one lane and the wrong order in the other: 0.5;
- doing one lane correctly and leaving the other lane and its new carton as they started: 0.5.

## How this differs from close tasks

- **Benchmark restocking tasks** (for example RoboCasa's RestockCannedFood and RestockPantry, BEHAVIOR-1K's
  stock_grocery_shelves) move new items from a counter or carton onto a shelf or into a cabinet, and at most take old
  items out; any free spot counts. Here the shelf itself forces the order: each lane holds exactly two cartons and its
  back slot is walled in by stock, so the robot has to pull the old carton out, set the new one into the narrow lane's
  back slot, and put the old one back in front. The same four cartons in the same four slots score 0, 0.5 or 1.0
  depending on that order.
- **Spring-pusher restocking tasks** hold a pusher back and slot new stock behind old stock that is never touched. Here
  there is no pusher and the old carton blocks the lane, so it has to come out and go back in, and the robot works out
  the order from the printed code dates.
- **Facing tasks** pull products forward to the shelf edge. Nothing is pulled forward here; the work is taking stock
  out and putting it back in the right sequence.

## Fidelity

- Cartons are rigid 6.9 x 6.9 x 12.5 cm boxes (the gable top is modelled flat) with mass and friction; the milk isn't
  modelled. The four 2% cartons have rotational damping: a simulated finger pad touches a carton at a few points, so
  without it a pinched carton pivots between the pads, which a real carton held by rubber pads doesn't. The real trial uses sealed dummy cartons weighted to 500 g, so nothing can leak onto the grippers.
- The code date is shown in the simulation as its day (18 or 27), in black block digits on the carton top; a real
  carton prints the full date on the gable.
- The shelf deck, lips, dividers and back wall are rigid and fixed. Refrigeration, condensation and the air curtain
  are not modelled.
- The deck stands for the top shelf of an open dairy case, which has only the canopy above it, usually 40 cm or more
  up. The canopy is left out so the top camera can see in. Leaving it out doesn't open a new way in: the back slot
  is reached the same two ways as in a real case, through the emptied lane from the front or straight down past the
  old carton, and the rear lip, the dividers and the neighbouring stock that box the slot in are all there.

## Runtime

Run limit 2,400 steps (120 s at 20 Hz). The layout is fixed; every seed starts from the same state.
