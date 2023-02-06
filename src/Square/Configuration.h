// Configuration.ino
// This file contains a bunch of constants and things you need to tune.

// Step 1: Tune the 90 degree position.
#define L_OFFSET 0
#define R_OFFSET 0

// Step 2: Invert the neccessary sides. See instructions pg. 33
#define INVERT_LEFT true  // set this to true if the left servo goes out on set_angles(0,0);
#define INVERT_RIGHT false // set this to true if the right servo goes out on set_angles(0,0);

// Step 3: Tune the scale factor. See instructions pg. 
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
