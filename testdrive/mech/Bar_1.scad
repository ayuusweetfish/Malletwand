module bar(w, h, alpha) {
    difference() {
        square([w, h], center = true);
        translate([0, h * (1/2 - alpha), 0])
            circle(r = 5.5/2, $fn = 96);
    }
}

alpha = 0.216;
w = 32;
space = w * 1.25;
rate = 2^(1/12);
translate([space*-1, 0, 0]) bar(w, 200/rate^0, alpha);
translate([space* 0, 0, 0]) bar(w, 200/rate^1, alpha);
translate([space* 1, 0, 0]) bar(w, 200/rate^2, alpha);