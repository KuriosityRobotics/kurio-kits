// Configuration.ino
// This file contains a bunch of constants and things you need to tune.

// Step 1: Tune the 90 degree position.
#define L_OFFSET 0
#define R_OFFSET 0

// Step 2: Invert the neccessary sides. when you run 
// set_angles(0,0), both servos should move inwards. 
// If they don't, flip the corresponding variable here.
#define INVERT_LEFT true
#define INVERT_RIGHT false

// Step 3: Tune the scale factor.
#define L_BOZO 1
#define R_BOZO 1

//----------Everything beyond this line does not need to be edited----------

//calculate scale factors


//offset+real*bozo = th
//real*bozo = th-offset
//real = (th-offset)/bozo

//test values
// #define L_OFFSET 57
// #define R_OFFSET -13

// constants about the lengths of the arms
#define MOTOR_TO_ORIGIN 45.5/2.
#define MOTOR_ARM_LEN 60
#define FLOATING_ARM_LEN 80
