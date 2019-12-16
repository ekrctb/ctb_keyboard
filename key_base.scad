
// MX Series Datasheet: <https://cdn.sparkfun.com/datasheets/Components/Switches/MX%20Series.pdf>

INCHES = 25.4;

pin_length = 3.30;
center_hole_diam = 0.157 * INCHES;
pin_hole_diam = 0.059 * INCHES;
led_hole_diam = 0.039 * INCHES;
pin_plus_pos = [3 * 0.05 * INCHES, -2 * 0.05 * INCHES];
pin_minus_pos = [-2 * 0.05 * INCHES, -4 * 0.05 * INCHES];
led_plus_pos = [1 * 0.05 * INCHES, 4 * 0.05 * INCHES];
led_minus_pos = [-1 * 0.05 * INCHES, 4 * 0.05 * INCHES];

// 

hole_clearance = 0.1;
square_dim = 20;
floor_thickness = 1.2;
square_thickness = pin_length + floor_thickness;
wire_hole_diam = 2.8;
wire_hole_z = square_thickness / 2;
bottom_hole_diam = 4;

EPS = 0.01;
$fa = 1;
$fs = 0.2;

difference() {
    translate([-square_dim/2, -square_dim/2, 0]) cube([square_dim, square_dim, square_thickness]);
    translate([0, 0, floor_thickness]) linear_extrude(height = pin_length + EPS, convexity = 5) {
        circle(d = center_hole_diam);
    };
    for (pin = [pin_plus_pos, pin_minus_pos]) {
        translate([pin[0], pin[1], -EPS]) cylinder(d = pin_hole_diam, h = square_thickness + EPS * 2);
        translate([pin[0], pin[1], -EPS]) cylinder(d1 = bottom_hole_diam, d2 = wire_hole_diam, h = wire_hole_z);

        d = wire_hole_diam;
        translate([pin[0], pin[1] + square_dim / 2 - d / 2, wire_hole_z]) rotate([0, 45, 0]) cube([d, square_dim, d], center = true);
    };
}
