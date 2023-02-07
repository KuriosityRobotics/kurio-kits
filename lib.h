#ifndef LIB_H
#define LIB_H

void set_angles(double l, double r);

void penUp();
void penDown();

void create_drawing();

void goTo(double x, double y);
void glideTo(double x, double y, double seconds);

#endif