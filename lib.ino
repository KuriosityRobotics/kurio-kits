#include <Arduino.h>
#include <Servo.h>

#include "lib.h"
#include "Configuration.h"

struct coord {
    double _1;
    double _2;
};

struct coord Coord(double _1, double _2) {
  struct coord c = {_1, _2};
  return c;
}

Servo left;
Servo right;
Servo lift;

bool penLifted = false;


coord calc_angles(coord c) {
  //calculate inner angles with inverse tangent
  struct coord beta = {atan2(c._2, (MOTOR_TO_ORIGIN + c._1)),
                       atan2(c._2, (MOTOR_TO_ORIGIN - c._1))};

  // calculate rhs of law of cosine: cos(alpha) = (a^2 + b^2 - c^2)/(2ab)
  struct coord alpha_calc = {
    (MOTOR_ARM_LEN * MOTOR_ARM_LEN + ((MOTOR_TO_ORIGIN + c._1) * (MOTOR_TO_ORIGIN + c._1) + c._2 * c._2) - FLOATING_ARM_LEN * FLOATING_ARM_LEN) / 
      (2 * MOTOR_ARM_LEN * sqrt((MOTOR_TO_ORIGIN + c._1) * (MOTOR_TO_ORIGIN + c._1) + c._2 * c._2)),
    (MOTOR_ARM_LEN * MOTOR_ARM_LEN + ((MOTOR_TO_ORIGIN - c._1) * (MOTOR_TO_ORIGIN - c._1) + c._2 * c._2) - FLOATING_ARM_LEN * FLOATING_ARM_LEN) / 
      (2 * MOTOR_ARM_LEN * sqrt((MOTOR_TO_ORIGIN - c._1) * (MOTOR_TO_ORIGIN - c._1) + c._2 * c._2))
  };

  // double check that cos(alpha)<1
  // cosine is bounded to [-1, 1] so if it's 
  // over 1 then something is very wrong
  if (alpha_calc._1 > 1 || alpha_calc._2 > 1){ 
    exit(1);
  }

  // use inverse cosine to calculate the angle
  struct coord alpha = {acos(alpha_calc._1),
                        acos(alpha_calc._2)};

  // add the inner and outer angles to get the 
  // full motor angle, then convert to degrees.
  struct coord motors = {(beta._1+alpha._1)*180./PI,
                         (beta._2+alpha._2)*180./PI};

  return motors;
}

//abs(5); ((5)>0?(5):-(5))

void set_angles(coord theta){
  // Serial.println("angles: " + String(180./PI * theta._1) + " " + String(180./PI * theta._2));
  double l = INVERT_LEFT ? 180 - theta._1 : theta._1;
  double r = INVERT_RIGHT ? 180 - theta._2 : theta._2;
  l = (L_BOZO*(l-90) + 90 + L_OFFSET); //for the reasoning behind this see Configuration.ino
  r = (R_BOZO*(r-90) + 90 + R_OFFSET); //for the reasoning behind this see Configuration.ino
  Serial.println("final angles: " + String(l) + String(r));
  left.write(l); 
  right.write(r);
}

void set_angles(double l, double r){
  set_angles(Coord(l, r));
}

void penUp() {
  lift.write(90);
  penLifted = true;
}

void penDown() {
  lift.write(0);
  penLifted = false;
}

bool getPenState(){
  // true if lifted, false if lowered
  return penLifted;
}


coord position = Coord(0, 80);
void goTo(double x, double y){
  set_angles(calc_angles(Coord(x, y)));
  position._1 = x;
  position._2 = y;
}

void glideTo(double x, double y, double seconds){
  glideTo(x, y, seconds, 1000);
}

void glideTo(double x, double y, double seconds, int interpSegments){
  double startTime = millis();
  coord startPos = position;
  double endTime = startTime + seconds * double(interpSegments);
  while (millis() < endTime){
    double timePassed = millis() - startTime;
    double fractionMoved = timePassed/(seconds * double(interpSegments));
    goTo(startPos._1 + fractionMoved * (x - startPos._1), startPos._2 + fractionMoved * (y - startPos._2));
  }
  goTo(x, y);
}

double getX(){
  return position._1;
}
double getY(){
  return position._2;
}

coord getPos(){
  return position;
}

void setup() {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  left.attach(9);
  right.attach(10);
  lift.attach(11);

  // set_angles(PI/2, PI/2);
  lift.write(0);

  Serial.begin(9600);
  Serial.println("Configuration:");
  Serial.println("L_BOZO: " + String(L_BOZO) + "\nR_BOZO: " + String(R_BOZO)+"\nL_OFFSET: "+String(L_OFFSET)+"\nR_OFFSET: "+String(R_OFFSET));

  // initialization sequence
  penUp(); // lift pen 
  goTo(0, 80); //go to initial home position
}