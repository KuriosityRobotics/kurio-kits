#include <Arduino.h>
#include <Servo.h>

#include "lib.h"
#include "Configuration.h"

struct pair {
  double first;
  double second;
};

struct pair Pair(double first, double second) {
  struct pair c = { first, second };
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

pair currentPosition = Pair(0, 80);

// solves angle opposite to Side c
double solveTriangle(double a, double b, double c) {
    // Law of Cosines
    return acos((c*c-a*a-b*b)/(-2*a*b));
}

pair calcAngles(pair c) {
    double leftDistance = sqrt(pow(c.first + MOTOR_TO_ORIGIN, 2) + pow(c.second, 2));
    double rightDistance = sqrt(pow(c.first - MOTOR_TO_ORIGIN, 2) + pow(c.second, 2));

    double thetaLeft = solveTriangle(leftDistance, 2 * MOTOR_TO_ORIGIN, rightDistance);
    double omegaLeft = solveTriangle(MOTOR_ARM_LEN, leftDistance, PEN_ARM_LEN);

    double thetaRight = solveTriangle(rightDistance, 2 * MOTOR_TO_ORIGIN, leftDistance);
    double omegaRight = solveTriangle(MOTOR_ARM_LEN, rightDistance, PEN_ARM_LEN);

    return Pair(toDegrees(theta1 + omega1), toDegrees(theta2 + omega2));
}

pair calc_position(pair theta) {
    // convert to radians
    struct pair theta_rad = Pair(
        toRadians(theta.first),
        toRadians(theta.second)
    );

    // calculate the pairinates of the joints
    struct pair left_joint = Pair(
        - MOTOR_TO_ORIGIN + cos(theta_rad.first) * MOTOR_ARM_LEN,
        sin(theta_rad.first) * MOTOR_ARM_LEN
    );
    struct pair right_joint = Pair(
        MOTOR_TO_ORIGIN - cos(theta_rad.second) * MOTOR_ARM_LEN,
        sin(theta_rad.second) * MOTOR_ARM_LEN
    );

    // we treat left_joint to right-joint as a line segment, find the midpoint, and step left by a certain distance
    double joint_distance = sqrt(pow(left_joint.first - right_joint.first, 2) + pow(left_joint.second - right_joint.second, 2));

    // joint_distance can not be more than 2 times the pen arm length
    if (joint_distance > 2 * PEN_ARM_LEN) {
        // exit(1);
    }

    double pen_deviation = sqrt(pow(PEN_ARM_LEN, 2) - pow(joint_distance / 2, 2));

    // from the midpoint, we step left by pen_deviation
    double pen_x = (left_joint.first + right_joint.first) / 2 - pen_deviation * (right_joint.second - left_joint.second) / joint_distance;
    double pen_y = (left_joint.second + right_joint.second) / 2 + pen_deviation * (right_joint.first - left_joint.first) / joint_distance;

    return Pair(pen_x, pen_y);
}

void setAngles(pair theta) { // in degrees
  Serial.println("setting angles: " + String(theta.first) + ", " + String(theta.second));

  // invert if necessary
  double l = INVERT_LEFT ? 180 - theta.first : theta.first;
  double r = INVERT_RIGHT ? 180 - theta.second : theta.second;

  // scale and offset
  l = (L_SCALE * (l - 90) + 90 + L_OFFSET);
  r = (R_SCALE * (r - 90) + 90 + R_OFFSET);

  currentPosition = calc_position(theta);

  // for simple 180 degree servos, we can use write(theta)
  left.write(l);
  right.write(r);
}

void setAngles(double l, double r) {
  setAngles(Pair(l, r));
}

void penUp() {
  lift.write(PEN_UP);
  penLifted = true;
}

void penDown() {
  lift.write(PEN_DOWN);
  penLifted = false;
}

bool getPenState() {
  // true if lifted, false if lowered
  return penLifted;
}

double clamp(double a, double min, double max) {
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

  setAngles(calcAngles(Pair(x, y)));
  //  currentPosition.first = x;
  //  currentPosition.second = y;
}

void glideTo(double x, double y, double seconds) {
  glideTo(x, y, seconds, 1000);
}

void glideTo(double x, double y, double seconds, int interpSegments) {
  double startTime = millis();
  pair startPos = currentPosition;
  double endTime = startTime + seconds * double(interpSegments);
  while (millis() < endTime){
    double timePassed = millis() - startTime;
    double fractionMoved = timePassed/(seconds * double(interpSegments));
    goTo(startPos.first + fractionMoved * (x - startPos.first), startPos.second + fractionMoved * (y - startPos.second));
  }
  goTo(x, y);
}

double getX() {
  return currentPosition.first;
}

double getY() {
  return currentPosition.second;
}

pair getPos() {
  return currentPosition;
}

void setup() {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  left.attach(9);
  right.attach(10);
  lift.attach(11);

  lift.write(0);

  Serial.begin(9600);
  Serial.println("Configuration:");
  Serial.println("L_OFFSET: " + String(L_OFFSET) + "\nR_OFFSET: " + String(R_OFFSET));

  // initialization sequence
  penDown();
  // goTo(0, 80); // go to initial home position
}