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


// resuelve para el ángulo opuesto al lado c
double solveTriangle(double a, double b, double c) {
    // Ley de cosenos
    return acos((-c*c+a*a+b*b)/(2*a*b));
}

pair calcAngles(pair c) {
    double leftDistance = sqrt(pow(c.first + MOTOR_TO_ORIGIN, 2) + pow(c.second, 2));
    double rightDistance = sqrt(pow(c.first - MOTOR_TO_ORIGIN, 2) + pow(c.second, 2));

    double thetaLeft = solveTriangle(leftDistance, 2 * MOTOR_TO_ORIGIN, rightDistance);
    double omegaLeft = solveTriangle(MOTOR_ARM_LEN, leftDistance, PEN_ARM_LEN);

    double thetaRight = solveTriangle(rightDistance, 2 * MOTOR_TO_ORIGIN, leftDistance);
    double omegaRight = solveTriangle(MOTOR_ARM_LEN, rightDistance, PEN_ARM_LEN);


    return Pair(180 - toDegrees(thetaLeft + omegaLeft), 180 - toDegrees(thetaRight + omegaRight));
}

pair calc_position(pair theta) {
    // convierte a radianes
    struct pair theta_rad = Pair(
        toRadians(theta.first),
        toRadians(theta.second)
    );


    // calcula los coordenas de las uniones
    struct pair left_joint = Pair(
        - MOTOR_TO_ORIGIN + cos(theta_rad.first) * MOTOR_ARM_LEN,
        sin(theta_rad.first) * MOTOR_ARM_LEN
    );
    struct pair right_joint = Pair(
        MOTOR_TO_ORIGIN - cos(theta_rad.second) * MOTOR_ARM_LEN,
        sin(theta_rad.second) * MOTOR_ARM_LEN
    );


    // Tratamos left_joint a right-joint como un segmento de linéa, encontramos el punto del medio, y pisamos a la izquierda una distancia
    double joint_distance = sqrt(pow(left_joint.first - right_joint.first, 2) + pow(left_joint.second - right_joint.second, 2));

    // joint_distance no puede ser más que dos veces más largo que el brazo
    if (joint_distance > 2 * PEN_ARM_LEN) {
        // exit(1);
    }

    double pen_deviation = sqrt(pow(PEN_ARM_LEN, 2) - pow(joint_distance / 2, 2));


    // desde el punto medio, damos un paso a la izquierda por desviación de pluma
    double pen_x = (left_joint.first + right_joint.first) / 2 - pen_deviation * (right_joint.second - left_joint.second) / joint_distance;
    double pen_y = (left_joint.second + right_joint.second) / 2 + pen_deviation * (right_joint.first - left_joint.first) / joint_distance;
    Serial.println("current position thinks is:" + String(pen_x) + " ,.." + String(pen_y));
    return Pair(pen_x, pen_y);
}

void setAngles(pair theta) { // en grados
  Serial.println("setting angles: " + String(theta.first) + ", " + String(theta.second));

  // invierta si es necesario
  double l = INVERT_LEFT ? 180 - theta.first : theta.first;
  double r = INVERT_RIGHT ? 180 - theta.second : theta.second;

  // escala y compensación
  l = (L_SCALE * (l - 90) + 90 + L_OFFSET);
  r = (R_SCALE * (r - 90) + 90 + R_OFFSET);

  // currentPosition = calc_position(theta); 
  Serial.println(l);
  Serial.println(r);
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
  // true si está levantado, false si no
  return penLifted;
}

double clamp(double a, double min, double max) {
  return max(min, min(a, max));
}

void goTo(double x, double y) {

  // ajustando para encajar a la pizarra
  
  double X_MIN = -60;
  double X_MAX = 60;
  double Y_MIN = 45;
  double Y_MAX = 100;

  x = clamp(x, X_MIN, X_MAX);
  y = clamp(y, Y_MIN, Y_MAX);

  setAngles(calcAngles(Pair(x, y)));

    currentPosition.first = x;
    currentPosition.second = y; 
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
  
  goTo(x,y);
  
  
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


  // sequencia de iniciación
  penDown();

  goTo(0, 80); // va a la posición inicial
}