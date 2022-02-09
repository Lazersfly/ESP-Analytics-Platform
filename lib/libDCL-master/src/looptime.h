/*
Class Looptime
Time execution time in uS of a code segment called repeatably
Time measured is 0-8uS greater.
If interrputs are disabled (ie within an interupt routine)
it only works up to 1024uS. This is due to the way micros() is implemented.
The average is calculated by storing the total and dividing by the no of loops,
this will overflow and the average will be invalid.
For 1000uS it will overflow after about 4,000,000 loops.
*/
#pragma once

#include <Arduino.h>

class LoopTime {
public:
  LoopTime();
  void start();     // start timing
  void end();       // end timing
  void clear();     // clear the results
  void print(const char *name = NULL);     // print the results
private:
  unsigned long StartTime;
  unsigned long NoofLoops;
  unsigned long Total;
  unsigned long Max;
  unsigned long Min;
};
