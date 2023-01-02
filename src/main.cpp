#include <Arduino.h>
#include <Servo.h>

#define L_OFFSET 10
#define R_OFFSET 25
#define MOTOR_DISTANCE 50
#define MOTOR_ARM_LEN 60
#define FLOATING_ARM_LEN 80

Servo left;
Servo right;
Servo lift;

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
void go_xy(float x, float y){
  //pythag to get len from motor pivot to end effector
  float d1 = sqrt(y*y + (x+(MOTOR_DISTANCE/2))*(x+(MOTOR_DISTANCE/2)));
  float d2 = sqrt(y*y + (x-(MOTOR_DISTANCE/2))*(x-(MOTOR_DISTANCE/2)));

  //law of cosines to get angle from above to motor arm
  float a1 = acos((FLOATING_ARM_LEN*FLOATING_ARM_LEN - MOTOR_ARM_LEN*MOTOR_ARM_LEN - d1*d1)/(2*MOTOR_ARM_LEN*d1));
  float a2 = acos((FLOATING_ARM_LEN*FLOATING_ARM_LEN - MOTOR_ARM_LEN*MOTOR_ARM_LEN - d2*d2)/(2*MOTOR_ARM_LEN*d2));

  //arctan to get angle from horiz to line referenced above
  float b1 = atan(y/(x+(MOTOR_DISTANCE/2)));
  float b2 = atan(y/(x-(MOTOR_DISTANCE/2)));

  float theta1 = a1 + b1;
  float theta2 = a2 + b2;
  Serial.println(String(x) + " " + String(y) + " " + String(theta1) + " " + String(theta2));
  set(theta1, theta2);

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
}

void loop() {
  go_xy(-10, 30);
  delay(1000);
  go_xy(0, 30);
  delay(1000);
  go_xy(10, 30);
  delay(1000);
}
