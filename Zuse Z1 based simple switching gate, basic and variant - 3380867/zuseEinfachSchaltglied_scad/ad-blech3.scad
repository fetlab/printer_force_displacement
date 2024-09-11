// a - d blech 3

$fn = 16;

cellRadius = 1.0;

cellSize = 5.0;

layerHeight = 2.0;

blechLength = cellSize * 12;
blechWidth  = cellSize * 3;

cellCenter = cellSize / 2;

holeOffset = 0.15;

use <basics.scad>


module dBlech() {
    difference() {
        translate([0, 0, 0]) color("yellow") cube([blechLength, blechWidth, layerHeight]);
        translate([cellSize * 4.5, cellSize / 2, -0.1]) color("red") dualYCell();
    }
    translate([0 , 0 , layerHeight]) cell();
    translate([0, cellSize * 2, layerHeight]) cell();
    translate([cellSize * 9 , 0 , layerHeight]) cell();
    translate([cellSize * 9, cellSize * 2, layerHeight]) cell();
}

module dualYCell() {
    hull() {
        translate([0, 0       , 0]) cell(layerHeight + 0.2);
        translate([0, cellSize, 0]) cell(layerHeight + 0.2);
    }
}

dBlech();



// hole center
//translate([0,            , 0, layerHeight]) color("red") cube([cellSize * 4.5, cellSize, layerHeight]);
//translate([cellSize * 4.5, 0, layerHeight]) color("blue") cube([cellSize * 4.5, cellSize, layerHeight]);