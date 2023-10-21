// Configuration.h
// This file contains a bunch of constants and things you need to tune.

// Step 1: Tune the 90-degree position using setAngles(90, 90); (degrees)
#define L_OFFSET 2
#define R_OFFSET 10

// Step 2: Invert the necessary sides. When you run
// setAngles(0,0), both servos should move inwards.
// setAngles(180, 180), both servos should move outwards.
// If they don't, flip the corresponding variable here.
#define INVERT_LEFT true
#define INVERT_RIGHT false

// Step 3: Tune the scale factor.
#define L_SCALE 1.15
#define R_SCALE 1.15

// ---------- DO NOT EDIT BELOW THIS LINE ----------

// robot geometry constants (millimeters)
#define MOTOR_TO_ORIGIN 23.25
#define MOTOR_ARM_LEN 60
#define PEN_ARM_LEN 80

// lifting servo positions
#define PEN_UP 45
#define PEN_DOWN 0