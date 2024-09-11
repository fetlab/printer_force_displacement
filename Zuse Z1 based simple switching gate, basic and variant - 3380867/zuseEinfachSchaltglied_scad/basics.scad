// basics 1

$fn = 16;

cellRadius = 1.0;

cellSize = 5.0;

layerHeight = 2.0;

pinOffset = 0.15;

module cellCorner(height = layerHeight) {
    cylinder(h = height, r = cellRadius);
}

module cell(height = layerHeight) {
    translate([0, 0, 0]) {
        hull() {
            translate([cellRadius,            cellRadius, 0])            cellCorner(height);
            translate([cellSize - cellRadius, cellRadius, 0])            cellCorner(height);
            translate([cellRadius,            cellSize - cellRadius, 0]) cellCorner(height);
            translate([cellSize - cellRadius, cellSize - cellRadius, 0]) cellCorner(height);
        }
    }
}

module pin(pinOffset = 0) {
    cylinder(h = layerHeight + 0.2, r = cellRadius * 1.5 - pinOffset);
}

