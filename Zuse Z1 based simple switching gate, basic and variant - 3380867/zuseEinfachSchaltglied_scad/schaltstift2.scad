// schaltstift 2

$fn = 64;

cellRadius = 1.0;

cellSize = 5.0;

layerHeight = 2.0;

pinDiameter = 4.5;

blechSize  = cellSize * 3;


translate([0, 0, 0]) cylinder(h = layerHeight, d = blechSize);
translate([0, 0, layerHeight]) cylinder(h = layerHeight * 5, d = pinDiameter);