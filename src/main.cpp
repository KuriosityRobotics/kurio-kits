#include <Arduino.h>
#include <Servo.h>
#include <Configuration.h>

Servo left;
Servo right;
Servo lift;

struct coord{
    double _1;
    double _2;
};

struct coord Coord(double _1, double _2){
  struct coord c = {_1, _2};
  return c;
}

struct coord get_angles(coord x){
  //pythag to get len from motor pivot to end effector
  struct coord d = {sqrt(sq(x._2) + sq(MOTOR_TO_ORIGIN + x._1)),
                    sqrt(sq(x._2) + sq(MOTOR_TO_ORIGIN - x._1))};

  //law of cosines to get angle from above to motor arm
  struct coord a = {acos((sq(FLOATING_ARM_LEN) - sq(MOTOR_ARM_LEN) + sq(d._1))/(2*MOTOR_ARM_LEN*d._1)),
                    acos((sq(FLOATING_ARM_LEN) - sq(MOTOR_ARM_LEN) + sq(d._2))/(2*MOTOR_ARM_LEN*d._2))};

  //arctan to get angle from horiz to line referenced above
//  struct coord b = {acos(((MOTOR_DISTANCE/2) + x._1)/d._1),
  //                  acos(((MOTOR_DISTANCE/2) - x._1)/d._2)};
    struct coord b = {atan2(x._2, (MOTOR_TO_ORIGIN + x._1)),
                      atan2(x._2, (MOTOR_TO_ORIGIN - x._1))};

  Serial.println("a");
  Serial.println(String(a._1) + " " + String(a._2));
  Serial.println("b");
  Serial.println(String(b._1) + " " + String(b._2));
  Serial.println("d");
  Serial.println(String(d._1) + " " + String(d._2));
  // Serial.println(String(d._2/((MOTOR_DISTANCE/2) + x._1)));
  //add angles together now and return them
  return Coord(a._1+b._1, a._2+b._2);
}

coord calc_angles(coord c) {
  double beta1 = atan2(c._2, (MOTOR_TO_ORIGIN + c._1));
  double beta2 = atan2(c._2, (MOTOR_TO_ORIGIN - c._1));

  double alpha1_calc = (MOTOR_ARM_LEN * MOTOR_ARM_LEN + ((MOTOR_TO_ORIGIN + c._1) * (MOTOR_TO_ORIGIN + c._1) + c._2 * c._2) - FLOATING_ARM_LEN * FLOATING_ARM_LEN) / 
                      (2 * MOTOR_ARM_LEN * sqrt((MOTOR_TO_ORIGIN + c._1) * (MOTOR_TO_ORIGIN + c._1) + c._2 * c._2));
  double alpha2_calc = (MOTOR_ARM_LEN * MOTOR_ARM_LEN + ((MOTOR_TO_ORIGIN - c._1) * (MOTOR_TO_ORIGIN - c._1) + c._2 * c._2) - FLOATING_ARM_LEN * FLOATING_ARM_LEN) / 
                      (2 * MOTOR_ARM_LEN * sqrt((MOTOR_TO_ORIGIN - c._1) * (MOTOR_TO_ORIGIN - c._1) + c._2 * c._2));

  if (alpha1_calc > 1 || alpha2_calc > 1){ 
    exit(1);
  }

  double alpha1 = acos(alpha1_calc);
  double alpha2 = acos(alpha2_calc);

  double shoulder1 = beta1 + alpha1;
  double shoulder2 = beta2 + alpha2;

  return Coord(shoulder1, shoulder2);
}

//abs(5); ((5)>0?(5):-(5))

void set(coord theta){
  Serial.println("angles: " + String(180./PI * theta._1) + " " + String(180./PI * theta._2));
  double l = INVERT_LEFT ? theta._1 : PI - theta._1;
  double r = INVERT_RIGHT ? theta._2 : PI - theta._2;
  left.write(l*180/PI + L_OFFSET);
  right.write(r*180/PI + R_OFFSET);
}

void set(double l, double r){
  set(Coord(l, r));
}

void draw(boolean up){
  if(up){
    lift.write(45);
  } else {
    lift.write(0);
  }
}

coord position = Coord(0, 0);

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
  struct coord pt = {0, 70};
  Serial.println("angles: " + String(calc_angles(pt)._1) + String(calc_angles(pt)._2));
}

void goTo(double x, double y){
  set(calc_angles(Coord(x, y)));
  position = Coord(x, y);
}

void glideTo(double x, double y, double seconds){
  double startTime = millis();
  coord startPos = position;
  double endTime = startTime + seconds * 1000.;
  while (millis() < endTime){
    double timePassed = millis() - startTime;
    double fractionMoved = timePassed/(seconds * 1000.);
    goTo(startPos._1 + fractionMoved * (x - startPos._1), startPos._2 + fractionMoved * (y - startPos._2));
  }
  goTo(x, y);
}

void loop() {
  // glideTo(-40, 50, 2);
  // delay(1000);
  // glideTo(40, 50, 2);
  // delay(1000);
  set(M_PI/2, M_PI);
  delay(1000);
  set(M_PI, M_PI/2);
  delay(1000);
}