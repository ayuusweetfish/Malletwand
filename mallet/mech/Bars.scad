// Testrun:
// 641.7 Hz, 200 mm
// 719.6 Hz, 200 * 2^(-2/24) mm
// 807.8 Hz, 200 * 2^(-4/24) mm

// Solving k / L^2 = f for k gives
// A = [200^-2; (200*(2^(-1/12)))^-2; (200*(2^(-2/12)))^-2]
// y = [641.7; 719.6; 807.8]
// A \ y = 2.56509629165051e7
// = 641.2740729126275 * 200^2

// At A4 = 442 Hz, Db5 = 442*2^(4/12) = 556.885104053534 Hz
// L = sqrt(k / f) = 200 * sqrt(641.2740729126275 / 556.885104053534)
//   = 214.6194293496366 mm

L0 = 214.6194293496366;

module bar(w, h, drill_pos) {
  echo(h * (0.25 - drill_pos)); // Clearance from the fixation to the node
  difference() {
    square([w, h], center = true);
    translate([0, h * (1/2 - drill_pos), 0])
      circle(r = 4.5/2, $fn = 96);
  }
}

drill_pos = 0.216;
w = 32;
space = w * 1.25;

scale_notes = [
// I  II III  IV #IV   V  #V  VI VII
   0,  2,  4,  5,  6,  7,  8,  9, 11,
  12, 14, 16, 17, 18, 19, 20, 21, 23,
  24, 26, 27
];
for (i = [0 : len(scale_notes) - 1]) {
  translate([space*i, 0, 0]) bar(w, L0 * 2^(-scale_notes[i]/24), drill_pos);
}
