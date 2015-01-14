#include "_encoder.h"
#include "encoder.h"

int rotate_dir(int before_a, int before_b, int now_a, int now_b)
{
  static int rotate = 0;

  rotate = before_b + now_a;
  rotate = rotate % 2;

  return rotate;
}

int rotate_dir_2(int before_a, int before_b, int now_a, int now_b)
{
  if(now_b == 1)
    return CW;
  else
    return CCW;
}
