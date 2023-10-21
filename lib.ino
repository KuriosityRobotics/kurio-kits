#include <Arduino.h>
#include <Servo.h>

#include "lib.h"
#include "Configuration.h"

struct coord {
  double _1;
  double _2;
};

struct coord Coord(double _1, double _2) {
  struct coord c = { _1, _2 };
  return c;
}

double toRadians(double theta) {
  return theta * PI / 180.;
}

double toDegrees(double theta) {
  return theta * 180. / PI;
}

Servo left;
Servo right;
Servo lift;

bool penLifted = false;

coord currentPosition = Coord(0, 80);

// solves angle opposite to Side c
double solveTriangle(double a, double b, double c) {
    // LoC
    return acos((c*c-a*a-b*b)/(-2*a*b));
}

coord calcAngle(coord c) {
    double l1 = sqrt(pow(c._1+MOTOR_TO_ORIGIN), 2) + pow(c._2));
    double l2 = sqrt(pow(c._1-MOTOR_TO_ORIGIN), 2) + pow(c._2));
    double theta1 = toDegrees(solveTriangle(l1, 2*MOTOR_TO_ORIGIN, l2));
    double theta2 = toDegrees(solveTriangle(l2, 2*MOTOR_TO_ORIGIN, l1));
    double omega1 = toDegrees(solveTriangle(MOTOR_ARM_LEN, l1, PEN_ARM_LEN));
    double omega2 = toDegrees(solveTriangle(MOTOR_ARM_LEN, l2, PEN_ARM_LEN));
    return Coord(theta1 + omega1, theta2 + omega2);
}

coord calc_angles(coord c) {
  // calculate inner angles with inverse tangent
  struct coord beta = {
    atan2(c._2, (MOTOR_TO_ORIGIN + c._1)),
    atan2(c._2, (MOTOR_TO_ORIGIN - c._1))
  };

  // calculate rhs of law of cosine: cos(alpha) = (a^2 + b^2 - c^2)/(2ab)
  struct coord alpha_calc = {
    (MOTOR_ARM_LEN * MOTOR_ARM_LEN + ((MOTOR_TO_ORIGIN + c._1) * (MOTOR_TO_ORIGIN + c._1) + c._2 * c._2) - PEN_ARM_LEN * PEN_ARM_LEN) /
      (2 * MOTOR_ARM_LEN * sqrt((MOTOR_TO_ORIGIN + c._1) * (MOTOR_TO_ORIGIN + c._1) + c._2 * c._2)),
    (MOTOR_ARM_LEN * MOTOR_ARM_LEN + ((MOTOR_TO_ORIGIN - c._1) * (MOTOR_TO_ORIGIN - c._1) + c._2 * c._2) - PEN_ARM_LEN * PEN_ARM_LEN) /
      (2 * MOTOR_ARM_LEN * sqrt((MOTOR_TO_ORIGIN - c._1) * (MOTOR_TO_ORIGIN - c._1) + c._2 * c._2))
  };

  // double check that cos(alpha) < 1
  // cosine is bounded to [-1, 1] so if it's 
  // over 1 then something is very wrong
  if (alpha_calc._1 > 1 || alpha_calc._2 > 1){ 
    // exit(1);
  }

  // use inverse cosine to calculate the angle
  struct coord alpha = Coord(
    acos(alpha_calc._1),
    acos(alpha_calc._2)
  );

  // add the inner and outer angles to get the 
  // full motor angle, then convert to degrees.
  struct coord motors = Coord(
    toDegrees(beta._1 + alpha._1),
    toDegrees(beta._2 + alpha._2)
  );

  return motors;
}

coord calc_position(coord theta) {
    // convert to radians
    struct coord theta_rad = Coord(
        toRadians(theta._1),
        toRadians(theta._2)
    );

    // calculate the coordinates of the joints
    struct coord left_joint = Coord(
        - MOTOR_TO_ORIGIN + cos(theta_rad._1) * MOTOR_ARM_LEN,
        sin(theta_rad._1) * MOTOR_ARM_LEN
    );
    struct coord right_joint = Coord(
        MOTOR_TO_ORIGIN - cos(theta_rad._2) * MOTOR_ARM_LEN,
        sin(theta_rad._2) * MOTOR_ARM_LEN
    );

    // we treat left_joint to right-joint as a line segment, find the midpoint, and step left by a certain distance
    double joint_distance = sqrt(pow(left_joint._1 - right_joint._1, 2) + pow(left_joint._2 - right_joint._2, 2));

    // joint_distance can not be more than 2 times the pen arm length
    if (joint_distance > 2 * PEN_ARM_LEN) {
        // exit(1);
    }

    double pen_deviation = sqrt(pow(PEN_ARM_LEN, 2) - pow(joint_distance / 2, 2));

    // from the midpoint, we step left by pen_deviation
    double pen_x = (left_joint._1 + right_joint._1) / 2 - pen_deviation * (right_joint._2 - left_joint._2) / joint_distance;
    double pen_y = (left_joint._2 + right_joint._2) / 2 + pen_deviation * (right_joint._1 - left_joint._1) / joint_distance;

    return Coord(pen_x, pen_y);
}

void setAngles(coord theta) { // in degrees
  // invert if necessary
  double l = INVERT_LEFT ? 180 - theta._1 : theta._1;
  double r = INVERT_RIGHT ? 180 - theta._2 : theta._2;

  // scale and offset
  l = (L_SCALE * (l - 90) + 90 + L_OFFSET);
  r = (R_SCALE * (r - 90) + 90 + R_OFFSET);

  currentPosition = calc_position(theta);

  Serial.println("setting angles: " + String(l) + ", " + String(r));
  left.write(l); 
  right.write(r);
}

void setAngles(double l, double r) {
  setAngles(Coord(l, r));
}

void penUp() {
  lift.write(PEN_UP);
  penLifted = true;
}

void penDown() {
  lift.write(PEN_DOWN);
  penLifted = false;
}

bool getPenState(){
  // true if lifted, false if lowered
  return penLifted;
}


void goTo(double x, double y){
  setAngles(calc_angles(Coord(x, y)));
//  currentPosition._1 = x;
//  currentPosition._2 = y;
}

void glideTo(double x, double y, double seconds) {
  glideTo(x, y, seconds, 1000);
}

void glideTo(double x, double y, double seconds, int interpSegments) {
  double startTime = millis();
  coord startPos = currentPosition;
  double endTime = startTime + seconds * double(interpSegments);
  while (millis() < endTime){
    double timePassed = millis() - startTime;
    double fractionMoved = timePassed/(seconds * double(interpSegments));
    goTo(startPos._1 + fractionMoved * (x - startPos._1), startPos._2 + fractionMoved * (y - startPos._2));
  }
  goTo(x, y);
}

double getX() {
  return currentPosition._1;
}

double getY() {
  return currentPosition._2;
}

coord getPos() {
  return currentPosition;
}

void setup() {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  left.attach(9);
  right.attach(10);
  lift.attach(11);

  // setAngles(90, 90);
  lift.write(0);

  Serial.begin(9600);
  Serial.println("Configuration:");
  Serial.println("L_OFFSET: " + String(L_OFFSET) + "\nR_OFFSET: " + String(R_OFFSET));

  // initialization sequence
  penDown(); // lift pen 
  // goTo(0, 0); // go to initial home position
}