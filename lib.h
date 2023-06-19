#ifndef LIB_H
#define LIB_H

void set_angles(double l, double r);

struct coord;

void penUp();
void penDown();

double getX();
double getY();

bool getPenState();
coord getPos();

void create_drawing();

void goTo(double x, double y);
void glideTo(double x, double y, double seconds);

#endif