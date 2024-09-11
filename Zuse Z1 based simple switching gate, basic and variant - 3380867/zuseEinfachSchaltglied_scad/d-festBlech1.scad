// einfachen Schaltglied Festblech 1

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

module basePlate() {
    hull() {
        translate([0          , 0                    , 0]) cell();
        translate([0          , blechWidth - cellSize, 0]) cell();
        translate([blechLength - cellSize, 0                    , 0]) cell();
        translate([blechLength - cellSize, blechWidth - cellSize, 0]) cell();
    }
}

module base() {
  difference() {
    // base
    translate([0,0,0]) basePlate();
    // center hole
    translate([cellSize * 2.5, cellSize * 2.5, -0.1]) elCell();
    
    // holes for base pins
    translate([cellSize / 2             , cellSize / 2              , -0.1]) pin();
    translate([blechWidth - cellSize / 2, cellSize / 2              , -0.1]) pin();
    translate([cellSize / 2             , blechLength - cellSize / 2, -0.1]) pin();
    translate([blechWidth - cellSize / 2, blechLength - cellSize / 2, -0.1]) pin(); 
  }
}

module elCell() {
    hull() {
        translate([0       , 0       , 0]) cell(layerHeight + 0.2);
        translate([cellSize, 0       , 0]) cell(layerHeight + 0.2);
    }
    hull() {
        translate([0       , 0, 0]) cell(layerHeight + 0.2);
        translate([0, cellSize, 0]) cell(layerHeight + 0.2);
    }
    difference() {
        translate([cellSize, cellSize, 0]) 
                cube([cellRadius, cellRadius, layerHeight + 0.2]);
            translate([cellSize + cellRadius, cellSize + cellRadius, - 0.1]) 
                cylinder(r = cellRadius, h = layerHeight + 0.4);
           
        
    }
}

// guides
module guidePillar() {
    translate([0, 0, 0])               cell();
    translate([0, 0, layerHeight])     cell();
    translate([0, 0, layerHeight * 2]) cell();
    translate([cellSize / 2, cellSize / 2, layerHeight * 3]) pin(holeOffset);    
}

module pin3() {
    translate([0,0,0])               pin(holeOffset);
    translate([0,0,layerHeight])     pin(holeOffset);
    translate([0,0,layerHeight * 2]) pin(holeOffset);    
}


translate([0,0,0]) base();

// pins / supports
translate([cellSize * pinXOffset             , cellSize * pinYOffset             , layerHeight]) {
    pin3();
}
translate([blechWidth - cellSize * pinXOffset, cellSize * pinYOffset             , layerHeight]) {
    pin3();
}
translate([cellSize * pinXOffset             , blechWidth - cellSize * pinYOffset, layerHeight]) {
    pin3();
}
translate([blechWidth - cellSize * pinXOffset, blechWidth - cellSize * pinYOffset, layerHeight]) {
    pin3();
}

translate([0, 0, layerHeight]) {
    translate([0, cellSize, 0]) {
        guidePillar();
    }
    translate([blechWidth - cellSize, cellSize, 0]) {    
        guidePillar();
    } 
    translate([0, blechLength - cellSize * 2, 0]) { 
        guidePillar();
    }
    translate([blechWidth - cellSize, blechLength - cellSize * 2, 0]) {
        guidePillar();
    }
}

// d-blech support
translate([0, cellSize * 2, layerHeight]) cell();
translate([0, cellSize * 4, layerHeight]) cell();
translate([blechLength - cellSize, cellSize * 2, layerHeight]) cell();
translate([blechLength - cellSize, cellSize * 4, layerHeight]) cell();




