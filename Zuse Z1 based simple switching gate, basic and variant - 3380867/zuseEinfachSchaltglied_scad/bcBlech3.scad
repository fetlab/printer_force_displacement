// b-c blech 3

$fn = 16;

cellRadius = 1.0;

cellSize = 5.0;

layerHeight = 2.0;

pinOffset = 0.15;
pinXOffset = 0.75;

blechWidth  = cellSize * 7;
blechLength = cellSize * 7;


use <basics.scad>


module pinSlitY() {
    color("gray") hull() {
        translate([0, 0       , 0]) pin();
        translate([0, cellSize, 0]) pin();
    }
}

module bcBlech() {
    difference() {
        color("blue") cube([cellSize * 2.4, blechWidth + cellSize * 5, layerHeight]);
        translate([cellSize * 1.5, blechWidth / 2 + cellSize, -0.1]) 
            color("red") cell(layerHeight + 0.2);
        translate([cellSize * 2, blechWidth / 2 + cellSize, -0.1]) 
            color("red") cube([cellSize, cellSize, layerHeight + 0.2]);
        translate([cellSize * pinXOffset, cellSize * 2.5, -0.1]) pinSlitY();
        translate([cellSize * pinXOffset, cellSize * 6.5, -0.1]) pinSlitY();
    }

    translate([0, 0, layerHeight]) color("blue") cell();
    translate([cellSize * 1.4, 0, layerHeight]) color("blue") cell();
    translate([0, blechWidth + cellSize * 2, layerHeight]) color("blue") cell();
    translate([cellSize * 1.4, blechWidth + cellSize * 2, layerHeight]) color("blue") cell();
}

bcBlech();
translate([cellSize * 5, 0, 0]) 
    mirror([180, 0, 0])
        bcBlech();

//translate([-cellSize, cellSize, layerHeight]) color("gray") cube([blechWidth, blechLength, layerHeight]);