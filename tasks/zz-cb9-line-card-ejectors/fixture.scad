// Line-card seating fixture: parametric source for every custom part (OpenSCAD 2021+).
// Units mm. Fixture frame (as in README and fixture_drawings.svg):
//   x = depth behind the steel strip's front face (chassis-front reference plane), negative = in front (robot side)
//   y = across the slot; for per-end parts y = distance in from that card end (left end; right end = mirror)
//   z = height above the bench; slot floor at z = 105
// Render one part for export with e.g.:  openscad -o catch_plate.dxf -D 'part="catch_plate_2d"' fixture.scad
//   part = "assembly" | "catch_plate" | "catch_plate_2d" | "hub_arm" | "ear" | "lever" | "plunger_tab" |
//          "faceplate_block" | "guide" | "strip" | "plate_angle"
part = "assembly";
proud = 6.0;          // assembly view: card position (6.0 = at the open levers' stop, 0.39 = seated)
theta = 45;           // assembly view: lever open angle, deg (0 = closed flat on the faceplate)
$fn = 48;

// ---- line card / slot (published N9K-X9736C-FX sizes) ----
CARD_W = 431.8;  CARD_D = 419.1;  CARD_H = 44.0;  CLEAR = 0.5;   // guide clearance per side
SLOT_W = CARD_W + 2 * CLEAR;     // 432.8
Z_FLOOR = 105;   BLOCK_D = 43.5; // faceplate + port block in front of the faceplate back face
TRAY_T = 1.5;
// ---- lever, pivot, crank ----
PIV_IN = 12.0;     // pivot 12 in from each card end
PIV_FWD = 10.0;    // pivot 10 in front of the block's front face
LEV_L = 100;  LEV_T = 20;  LEV_H = 30;  LEV_Z0 = 112;            // lever z 112..142
CRANK_R = 26.1;    // hub arm centres (pivot -> dowel)
CRANK_PHI = 50;    // hub arm 50 deg past straight-outboard toward the chassis when the lever is closed
DOWEL_D = 6;  DOWEL_L = 20;
HUB_T = 8;  HUB_W = 12;  HUB_Z0 = 91;                            // hub arm z 91..99
PIN_D = 6;  PIN_L = 66;  PIN_Z0 = 91;
EAR_T = 3;  EAR_W = 20;  EAR_FWD = 20;  EAR_BACK = 22;           // ear: 20 in front of block face .. 22 behind
// ---- catch plate (left end), plate frame x, y ----
PL_T = 5;  PL_Z0 = 84;                                           // plate z 84..89
PL_X0 = -40.0;  PL_Y0 = -20.0;  PL_Y1 = 35.0;                   // blank x -40..0, y -20..35
FRONT = -40.0;  REAR = -30.5;  LIPB = -36.9;                     // front edge, stop wall, lip back face
LIP_Y = 9.8;  NOTCH_IN = 20.5;  POCK_Y = -8.75;                  // lip cam corner, notch inboard side, pocket floor
CUT = [[FRONT, LIP_Y], [FRONT, NOTCH_IN], [REAR, NOTCH_IN], [REAR, POCK_Y], [LIPB, POCK_Y], [LIPB, LIP_Y]];  // V1..V6
PL_HOLES = [[-12, -12], [-12, 27]];
// ---- connector-resistance plungers ----
PLG_IN = 40;  PLG_Z = 119;  PLG_NOSE_X = CARD_D - 6.0;         // nose meets the tray back when 6.0 proud (413.1)
TAB = [50, 50, 10];  TAB_X = 430;                                // tab front face at x 430 on the cross member at 440

// ======================= parts (each in its own local frame) =======================
// catch plate, 2D (for laser / waterjet DXF): drawing frame X = y (in from card end), Y = -x (toward the robot)
module catch_plate_2d() {
    difference() {
        polygon([[PL_Y0, 0], [PL_Y1, 0], [PL_Y1, -PL_X0], [PL_Y0, -PL_X0]]);
        polygon([for (v = CUT) [v[1], -v[0]]]);
        for (h = PL_HOLES) translate([h[1], -h[0]]) circle(d = 5.5);
    }
}
// catch plate in the fixture frame (x, y, z), built directly from CUT (x, y)
module catch_plate_xy() {
    translate([0, 0, PL_Z0]) linear_extrude(PL_T) difference() {
        translate([PL_X0, PL_Y0]) square([-PL_X0, PL_Y1 - PL_Y0]);
        polygon(CUT);
        for (h = PL_HOLES) translate(h) circle(d = 5.5);
    }
}
// hub arm: local frame origin at the pivot, arm along +X to the dowel, z up from its underside
module hub_arm() {
    difference() {
        hull() { cylinder(r = HUB_W / 2, h = HUB_T); translate([CRANK_R, 0, 0]) cylinder(r = HUB_W / 2, h = HUB_T); }
        translate([0, 0, -1]) cylinder(d = PIN_D, h = HUB_T + 2);
        translate([CRANK_R, 0, -1]) cylinder(d = DOWEL_D, h = HUB_T + 2);
        translate([0, 0, HUB_T / 2]) rotate([0, -90, 0]) cylinder(d = 3.3, h = 10);   // M4 set screw tapping drill
    }
}
module dowel() { translate([CRANK_R, 0, HUB_T - DOWEL_L]) cylinder(d = DOWEL_D, h = DOWEL_L); }
// pivot ear: local origin at the pivot bore, -X = toward the robot, +X = toward the chassis (block face at X = +10)
module ear() {
    difference() {
        hull() { cylinder(r = EAR_W / 2, h = EAR_T); translate([PIV_FWD + EAR_BACK - 1, -EAR_W / 2, 0]) cube([1, EAR_W, EAR_T]); }
        translate([0, 0, -1]) cylinder(d = 10, h = EAR_T + 2);                         // bushing OD 10 H7
        for (dx = [PIV_FWD + 8, PIV_FWD + 18]) translate([dx, 0, -1]) cylinder(d = 4.5, h = EAR_T + 2);
    }
}
// lever: local origin at the pivot axis (bottom face), lever along +Y to the tip, back face (faceplate side) at X = +10
module lever() {
    difference() {
        hull() { cylinder(r = LEV_T / 2, h = LEV_H); translate([-LEV_T / 2, LEV_L - 1, 0]) cube([LEV_T, 1, LEV_H]); }
        translate([0, 0, -1]) cylinder(d = PIN_D, h = LEV_H + 2);
        translate([0, 0, LEV_H / 2]) rotate([90, 0, 0]) cylinder(d = 3.3, h = 20);     // M4 axial set screw
        translate([LEV_T / 2 - 2, 85, LEV_H / 2]) rotate([0, 90, 0]) cylinder(d = 16.2, h = 3);  // strike disc pocket
        for (k = [0:4]) translate([-LEV_T / 2 - 0.5, 88 + 2.5 * k, -1]) cube([1.5, 1, LEV_H + 2]);  // grip grooves
    }
}
module plunger_tab() {
    difference() {
        cube(TAB);                                                                      // x 50 (y), y 50 (z), z 10 (thick)
        translate([TAB[0] / 2, TAB[1] / 2, -1]) cylinder(d = 14.5, h = TAB[2] + 2);   // M16x1.5 tapping drill
        for (sx = [8, TAB[0] - 8]) hull() for (sy = [12, TAB[1] - 12]) translate([sx, sy, -1]) cylinder(d = 6.5, h = TAB[2] + 2);
    }
}
// faceplate block: front face at x = -BLOCK_D (when the card is seated), y across the card from its left end (y = 0)
module faceplate_block() {
    difference() {
        translate([-BLOCK_D, -CARD_W, Z_FLOOR]) cube([BLOCK_D, CARD_W, CARD_H]);
        for (s = [0, 1]) let(yp = s == 0 ? -PIV_IN : -CARD_W + PIV_IN) {
            for (dx = [8, 18], z = [Z_FLOOR - 1, Z_FLOOR + CARD_H - 9])                 // M4 tapped holes for the ears
                translate([-BLOCK_D + dx, yp, z]) cylinder(d = 3.3, h = 10);
            translate([-BLOCK_D - 1, s == 0 ? yp - 85 : yp + 85, Z_FLOOR + CARD_H / 2]) rotate([0, 90, 0])
                cylinder(d = 16.2, h = 5.6);                                            // pot magnet pocket (16 x 4.5 + 1)
        }
    }
}
module guide() { cube([600, 25, 3]); translate([0, 0, 0]) cube([600, 3, 25]); }        // 25 x 25 x 3 angle, 600 long
module strip() { translate([0, -242.5, 55]) cube([3, 485, 50]); }                     // 3 x 50 x 485 steel strip
module plate_angle() { cube([25, 55, 3]); translate([22, 0, 0]) cube([3, 55, 30]); }

// ======================= assembly (left end detail + card) =======================
module left_end(p, th) {
    yl = SLOT_W / 2 - CLEAR;                       // world y of the left card end (card centred)
    piv = [-(p + BLOCK_D + PIV_FWD), yl - PIV_IN];
    // catch plate: plate frame y (in from the card end) -> world y = yl - y
    translate([0, yl, 0]) mirror([0, 1, 0]) catch_plate_xy();
    // lever, hub arm and dowel rotate together about the pivot pin
    translate([piv[0], piv[1], 0]) rotate([0, 0, -th]) {
        translate([0, 0, LEV_Z0]) mirror([0, 1, 0]) lever();                         // closed: tip toward the card centre (-y world)
        translate([0, 0, HUB_Z0]) rotate([0, 0, 90 - CRANK_PHI]) { hub_arm(); dowel(); }  // closed: 50 deg from outboard (+y) toward the chassis (+x)
        translate([0, 0, PIN_Z0]) cylinder(d = PIN_D, h = PIN_L);
    }
    for (z = [Z_FLOOR - EAR_T, Z_FLOOR + CARD_H]) translate([piv[0], piv[1], z]) ear();
}
module assembly() {
    color("silver") strip();
    color("lightsteelblue") translate([-proud, SLOT_W / 2 - CLEAR, 0]) faceplate_block();
    color("gainsboro") translate([-proud, -CARD_W / 2, Z_FLOOR]) cube([CARD_D, CARD_W, TRAY_T]);
    color("gray") translate([-proud + CARD_D - 25, -CARD_W / 2, Z_FLOOR + TRAY_T]) cube([25, CARD_W, 3]);   // back push angle
    color("steelblue") left_end(proud, theta);
    color("steelblue") mirror([0, 1, 0]) left_end(proud, theta);
    for (s = [1, -1]) color("dimgray") translate([PLG_NOSE_X, s * (CARD_W / 2 - PLG_IN), PLG_Z]) rotate([0, 90, 0]) cylinder(d = 16, h = TAB_X - PLG_NOSE_X);
}

if (part == "assembly") assembly();
else if (part == "catch_plate") catch_plate_xy();
else if (part == "catch_plate_2d") catch_plate_2d();
else if (part == "hub_arm") { hub_arm(); dowel(); }
else if (part == "ear") ear();
else if (part == "lever") lever();
else if (part == "plunger_tab") plunger_tab();
else if (part == "faceplate_block") faceplate_block();
else if (part == "guide") guide();
else if (part == "strip") strip();
else if (part == "plate_angle") plate_angle();
