#include <Arduino.h>
#include <Servo.h>
#include <Configuration.h>
// #include <Kinematics.cpp>

Servo left;
Servo right;
Servo lift;

struct coord{
    float _1;
    float _2;
};

struct coord Coord(float _1, float _2){
  struct coord c = {_1, _2};
  return c;
}

struct coord get_angles(coord x){
  //pythag to get len from motor pivot to end effector
  struct coord d = {sqrt(sq(x._2) + sq(((MOTOR_DISTANCE/2) + x._1))),
                    sqrt(sq(x._2) + sq((MOTOR_DISTANCE/2) - x._1))};

  //law of cosines to get angle from above to motor arm
  struct coord a = {acos((sq(FLOATING_ARM_LEN) - sq(MOTOR_ARM_LEN) - sq(d._1))/(2*MOTOR_ARM_LEN*d._1)),
                    acos((sq(FLOATING_ARM_LEN) - sq(MOTOR_ARM_LEN) - sq(d._2))/(2*MOTOR_ARM_LEN*d._2))};

  //arctan to get angle from horiz to line referenced above
  struct coord b = {acos(d._1/((MOTOR_DISTANCE/2) + x._1)),
                    acos(d._2/((MOTOR_DISTANCE/2) - x._1))};

  //add angles together now and return them
  return Coord(a._1+b._1, a._2+b._2);
}

void set(coord theta){
  left.write(theta._1*180/PI + L_OFFSET);
  right.write(theta._2*180/PI + R_OFFSET);
}

void set(float l, float r){
  left.write(l*180/PI + L_OFFSET);
  right.write(r*180/PI + R_OFFSET);
}

void draw(boolean up){
  if(up){
    lift.write(45);
  } else {
    lift.write(0);
  }
}



void setup() {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  left.attach(9);
  right.attach(10);
  lift.attach(11);

  set(PI/2, PI/2);
  lift.write(0);

  Serial.begin(9600);
  struct coord pt = {0, 100};
  Serial.println(String(get_angles(pt)._1) + String(get_angles(pt)._2));
}

void loop() {
  set(get_angles(Coord(-10, 30)));
  delay(1000);
  set(get_angles(Coord(0, 30)));
  delay(1000);
  set(get_angles(Coord(10, 30)));
  delay(1000);
}
