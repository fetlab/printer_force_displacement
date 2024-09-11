// top blech 2

$fn = 16;

cellRadius = 1.0;

cellSize = 5.0;

layerHeight = 2.0;

blechWidth  = cellSize * 7;
blechLength = cellSize * 7;

cellCenter = cellSize / 2;

holeOffset = 0.15;

pinXOffset = 1.75;
pinYOffset = 1.5;


use <basics.scad>

module topBlech() {
    translate([0, 0, 0]) {
        difference() {
            color("gray") hull() {
                translate([0                    , 0,                      0]) cell();
                translate([blechWidth - cellSize, 0,                      0]) cell();
                translate([0                    , blechLength - cellSize, 0]) cell();
                translate([blechWidth - cellSize, blechLength - cellSize, 0]) cell();
            }
            hull() {
                translate([cellSize                 , cellSize,                   -0.1]) cell(layerHeight + 0.2);
                translate([blechWidth - cellSize * 2, cellSize,                   -0.1]) cell(layerHeight + 0.2);
                translate([cellSize                 , blechLength - cellSize * 2, -0.1]) cell(layerHeight + 0.2);
                translate([blechWidth - cellSize * 2, blechLength - cellSize * 2, -0.1]) cell(layerHeight + 0.2);
            }
            translate([cellSize / 2, cellSize * 1.5, 0]) pin();
            translate([blechWidth - cellSize / 2, cellSize * 1.5, 0]) pin();
            translate([cellSize / 2, blechLength - cellSize * 1.5, 0]) pin();
            translate([blechWidth - cellSize / 2, blechLength - cellSize * 1.5, 0]) pin();
        }
    }
}

topBlech();