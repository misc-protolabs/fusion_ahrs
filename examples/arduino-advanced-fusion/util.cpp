#include "util.h"

float filt_1ord( float x, float y_z1, float tau, float dt)
{
  float a0, y;
  if( tau <= 0 || dt <= 0)
  {
    y = x;
  }
  else
  {
    a0 = dt / tau;
    y = (a0 * (x - y_z1)) + y_z1;
  }
  return y;
}
