// interpolate.h

#pragma once

#include <Arduino.h>

// Does linear interpolation on a curve
// The curve is defined by the constructor call
// curve is an array of input and outpout pairs
// input must always increase
// the last pair must be 0, 0,
// If the input is below the first input value,
// if lower_limit != 0 returns lower_limit otherwise the first output value
// If the input is above the last input value,
// if upper_limit != 0 returns upper_limit otherwise the last output value
// ie for 1,1 2,2 ,3,3, 0,0 with lower and upper_limit = 0
// inputs below 1 will output 1
// inputs above 3 will output 3
// inputs between 1 and 3 be be the same as the input
// 

class Interpolate {
 public:
  Interpolate(float (*curve)[2], float upper_limit = 0, float lower_limit = 0)
    { curve_ = curve; upper_limit_ = upper_limit; lower_limit_ = lower_limit; }
  float interpolate(float input);
  void debug();
 private:
  float (*curve_)[2];
  float lower_limit_;
  float upper_limit_;
};
