#include "lib.h"

void loop() {
  penUp();
  goTo(-10, 60);
  penDown();
  glideTo(10, 60, 2);
  glideTo(10, 80, 2);
  glideTo(-10, 80, 2);
  glideTo(-10, 60, 2);
}
