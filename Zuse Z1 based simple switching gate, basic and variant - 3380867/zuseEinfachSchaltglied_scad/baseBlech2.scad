// einfachen Schaltglied Base blech 2

$fn = 16;

cellRadius = 1.0;

cellSize = 5.0;

layerHeight = 2.0;

blechWidth  = cellSize * 7;
blechLength = cellSize * 7;

cellCenter = cellSize / 2;

holeOffset = 0.15;

use <basics.scad>

// base
translate([0,0,0]) cube([blechWidth, blechLength, layerHeight]);
    
// support for base pins
translate([0                    , 0                     , layerHeight]) cell();
translate([blechWidth - cellSize, 0                     , layerHeight]) cell();
translate([0                    , blechLength - cellSize, layerHeight]) cell();
translate([blechWidth - cellSize, blechLength - cellSize, layerHeight]) cell(); 

// holes for base pins
translate([cellSize / 2             , cellSize / 2              , layerHeight * 2]) pin(holeOffset);
translate([blechWidth - cellSize / 2, cellSize / 2              , layerHeight * 2]) pin(holeOffset);
translate([cellSize / 2             , blechLength - cellSize / 2, layerHeight * 2]) pin(holeOffset);
translate([blechWidth - cellSize / 2, blechLength - cellSize / 2, layerHeight * 2]) pin(holeOffset); 


module quadCell() {
    hull() {
        translate([0       , 0       , 0]) cell(layerHeight + 0.2);
        translate([cellSize, 0       , 0]) cell(layerHeight + 0.2);
        translate([0       , cellSize, 0]) cell(layerHeight + 0.2);
        translate([cellSize, cellSize, 0]) cell(layerHeight + 0.2);
    }
}