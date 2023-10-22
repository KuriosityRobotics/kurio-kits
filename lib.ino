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
    // Law of Cosines
    return acos((c*c-a*a-b*b)/(-2*a*b));
}

coord calcAngles(coord c) {
    double leftDistance = sqrt(pow(c._1 + MOTOR_TO_ORIGIN, 2) + pow(c._2, 2));
    double rightDistance = sqrt(pow(c._1 - MOTOR_TO_ORIGIN, 2) + pow(c._2, 2));

    double thetaLeft = solveTriangle(leftDistance, 2 * MOTOR_TO_ORIGIN, rightDistance);
    double omegaLeft = solveTriangle(MOTOR_ARM_LEN, leftDistance, PEN_ARM_LEN);

    double thetaRight = solveTriangle(rightDistance, 2 * MOTOR_TO_ORIGIN, leftDistance);
    double omegaRight = solveTriangle(MOTOR_ARM_LEN, rightDistance, PEN_ARM_LEN);

    return Coord(toDegrees(theta1 + omega1), toDegrees(theta2 + omega2));
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
  Serial.println("setting angles: " + String(theta._1) + ", " + String(theta._2));

  // invert if necessary
  double l = INVERT_LEFT ? 180 - theta._1 : theta._1;
  double r = INVERT_RIGHT ? 180 - theta._2 : theta._2;

  // scale and offset
  l = (L_SCALE * (l - 90) + 90 + L_OFFSET);
  r = (R_SCALE * (r - 90) + 90 + R_OFFSET);

  currentPosition = calc_position(theta);

  // range between 1000 and 2000
  l = (l / 180) * 1000 + 1000;
  r = (r / 180) * 1000 + 1000;

  left.writeMicroseconds(l); 
  right.writeMicroseconds(r);
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

double clamp(double a, double min, double max){
  return max(min, min(a, max));
}

void goTo(double x, double y) {
  // adjust offset to fit in whiteboard
  double X_MIN = -60;
  double X_MAX = 60;
  double Y_MIN = 55;
  double Y_MAX = 100;

  x = clamp(x, X_MIN, X_MAX);
  y = clamp(y, Y_MIN, Y_MAX);

  setAngles(calcAngles(Coord(x, y)));
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