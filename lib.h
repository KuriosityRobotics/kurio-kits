#ifndef LIB_H
#define LIB_H

double toRadians(double theta);

double toDegrees(double theta);

void setAngles(double l, double r);

struct pair;

void penUp();
void penDown();

double getX();
double getY();

bool getPenState();
pair getPos();

void goTo(double x, double y);
void glideTo(double x, double y, double seconds);

#endif