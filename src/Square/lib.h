struct coord {
    double _1;
    double _2;
} ;

struct coord Coord(double _1, double _2) {
  struct coord c = {_1, _2};
  return c;
}

void set_angles(coord theta);
void set_angles(double l, double r);

void penUp();
void penDown();

void create_drawing();

void goTo(double x, double y);
void glideTo(double x, double y, double seconds);
