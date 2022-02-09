// interpolate.cpp

#include "interpolate.h"
#include "pf.h"

#define IN 0
#define OUT 1

float Interpolate::interpolate(float input) {
  size_t i;
  float output = 0;
  float slope;
  
  if (input <= curve_[0][IN])
    return(lower_limit_ == 0? curve_[0][OUT] : lower_limit_);
  for(i = 1; curve_[i][IN] != 0; i++) {
    if ((input < curve_[i][IN]) && (input >= curve_[i-1][IN])) {
       output = curve_[i-1][OUT];
       slope = (curve_[i][OUT]-curve_[i-1][OUT])/(curve_[i][IN]-curve_[i-1][IN]);
       output += (input - curve_[i-1][0])*slope;
       //pfln("i=%i in %f out %f[i-1][IN] %f [i][IN] %f [i-1][OUT] %f [i][OUT] %f slope %f",
         //i, input, output, curve_[i-1][IN], curve_[i][IN], curve_[i-1][OUT], curve_[i][OUT], slope); 
       return(output);
    }
  }
  return(upper_limit_ == 0 ? curve_[i-1][OUT] : upper_limit_);
}

void Interpolate::debug() {
  size_t i;

  for (i = 0; (curve_[i][IN] != 0) || (i == 0);i++) {
    pfln("curve_ %p i %i IN %f OUT %f",curve_, i, curve_[i][IN], curve_[i][OUT]);
  }
}
