// rollingav.cpp

#include "rollingav.h"
#ifdef STANDALONE
#include <stdio.h>
extern void loop();
#endif
#include <Arduino.h>
#include "pf.h"

//#define DEBUGROLLING

#define MAXROLLING (NOOFPERIODS*period_)

RollingAv::RollingAv(unsigned long t) { 
  period_ = t/NOOFPERIODS;
  reset();
}

void RollingAv::reset() {
  average = 0;
  average_start_ = 0;
  rolling_start_ = current_start_ = last_call_ = 0;
  clear_array(); 
}

void RollingAv::clear_array() {
  int i;
  for (i = 0; i < NOOFPERIODS; i++)
    array_[i] = 0;
}

//rolling - call repeatably with the value you want to average over and the time of the call
// Times in order largest (latest) first
// time = Current time
// last_call_ = Last time rolling was called
// current_start_ - Start time of latest rolling period_
void RollingAv::rolling(float value,unsigned long time) {
  size_t i;

#ifdef DEBUGROLLING
  pf("rolling(%f, %lu)  current_start_ %lu last_call_ %lu average %f\n",
    value, time, current_start_, last_call_, average);
#endif
  if (current_start_ == 0) {
    // First call after reset
#ifdef ROLLIUNGDEBUG
    pfln("First rolling %lu", time);
#endif
    rolling_start_ = current_start_ = last_call_ = time;
    array_[0] = value;
    return;
  }
  if (time == current_start_) time++;      // Fix divide by 0
  if ((time - current_start_) < period_) {
    // In the current period_, modify array_[0]
    array_[0] = (array_[0]*(last_call_-current_start_) + value*(time-last_call_))/(time-current_start_);
  }
  else {
    // If no calls for a period_ or more reset
    if ((time - current_start_) >= 2*period_) {
#ifdef ROLLINGDEBUG
      pfln("RollingAv: no calls for > period_ time %lu start time %lu", time, current_start_);
#endif
      clear_array();
      current_start_ = rolling_start_ = last_call_ = time;
      // make sure has valid value
      array_[0] = value;
      return;
    }
    // New period_ has started, move all periods back one and start new current period_
    for (i = NOOFPERIODS; i > 0;i--)
      array_[i] = array_[i - 1];
    current_start_ += period_;
    array_[0] = value;
  }
  if (average_start_ == 0) {
    average_start_ = time;
    average = value;
  }
  else
    average = (average*(last_call_-average_start_) + value*(time-last_call_))/(time - average_start_); 
  last_call_ = time;
#ifdef DEBUGROLLING
  pf("array_ ");
  for (i = 0;i < NOOFPERIODS; i++)
    pf("%f ", array_[i]);
  pf();
  pfln("average %f", average);
#endif
}

// get the rolling average for the AvT time units. If AvT is 0 time is MAXROLLING.
float RollingAv::get_rolling(unsigned long AvT) const {
  int i;
  float Av = 0;
  unsigned long diff_t = last_call_ - current_start_;;

  if (last_call_ == 0)
    // no rolling calls!
    return(0.0);
  if (rolling_start_ == last_call_)    // called after only one call to rolling
    return(array_[0]);
  if (diff_t == 0) diff_t = 1;      // Fix divide by 0
  if (AvT == 0) AvT = MAXROLLING;
  if (AvT > MAXROLLING) AvT = MAXROLLING;
  // to fix startup
  if (current_start_ == 0)    // never made a call to rolling
    return(0);
  if (AvT > (last_call_ - rolling_start_)) AvT = last_call_ - rolling_start_; 
  // Contribution from current period_ 
  Av = array_[0]*(diff_t);
#ifdef DEBUGROLLING
  pfln("AvT %lu diff_t %lu", AvT, diff_t);
  pf("C %f ", Av);
#endif
  for (i=1; i <  (NOOFPERIODS+1); i++) {
    if (((diff_t) + i*period_) > AvT) {
      // Contribution from oldest period_
      Av += array_[i]*(AvT-(i-1)*period_-(diff_t));
#ifdef DEBUGROLLING
      pf("O%d %f ",i, array_[i]*(AvT-(i-1)*period_-(diff_t)));
#endif
      break;
    }
    // Contribution from middle periods
    Av += array_[i]*period_;
#ifdef DEBUGROLLING
    pf("M%d %f ", i, array_[i]*period_);
#endif    
  }
#ifdef DEBUGROLLING
  pf();
  if (Av/AvT == 0) {
    pfln("Av %f AvT %lu",Av, AvT);
    debug();
  }
#endif   
  return (Av/AvT);
}

float RollingAv::get_average() const { return(average); }

void RollingAv::debug() const {
  int i;
  pf("period_ %i current_start_ %lu last_call_ %lu Av start time %lu average %f\n",
    period_, current_start_, last_call_, average_start_, average);
  for (i = 0; i < NOOFPERIODS; i++)
    pf("%f ", array_[i]);
  pf(); 
}

//#define MAIN
#ifdef MAIN

#ifdef STANDALONE
int main(){
  setup();
  for(;;)
    loop();
  return 0;
}
#endif //STANDALONE

void setup() {
  // put your setup code here, to run once:
#ifndef STANDALONE
  Serial.begin(115200);
#endif
  int i;
  float res;
  char str[10];
  pf("\nStarting\n");
  // put your main code here, to run repeatedly:
  RollingAv RA = RollingAv(60);
  RA.rolling(1.0,60);
  RA.rolling(2.0,120);
  if ((res = RA.getRolling(120)) != 0.5)
    pf("failed 1 %s\n",dtostrf(res, 7, 2, str));
  if ((res = RA.getAverage()) != 1.5)
    pf("failed 2 %s\n",dtostrf(res, 7, 2, str));
  RA.rolling(3.0,240);
  if ((res = RA.getRolling(240)) != 1.25)
    pf("failed 3 %s\n",dtostrf(res, 7, 2, str));
  if ((res = RA.getRolling(150)) != 1.8)
    pf("failed 4 %s\n",dtostrf(res, 7, 2, str));
  if ((res = RA.getRolling(120)) != 2.0)
    pf("failed 5 %s\n", dtostrf(res, 7, 2, str));
  RollingAvFixed RAF = RollingAvFixed(12);
  for (i = 0;i < 12;i++) {
    if (i == 10)
      RAF.rolling(13);
    else
      RAF.rolling(1);
  }      
  if ((res = RAF.getRolling()) != 2.0)
    pf("failed 6 %s\n", dtostrf(res, 7, 2, str));
  pf("Test succeeded");
/*
#define FAST    // Run rolling calls fast
#ifdef FAST
  for (i = 0; i < 50;i++)
    RA.rolling(1, i*6000+1);
  for (i = 50; i < 100;i++)
    RA.rolling(2, i*6000+1);
#else    
  RA.rolling(1, 1*60000);
  RA.rolling(1, 2*60000);
  RA.rolling(1, 3*60000);
  RA.rolling(1, 4*60000);
  RA.rolling(1, 5*60000);
  RA.rolling(2, 6*60000);
  RA.rolling(2, 7*60000);
  RA.rolling(2, 8*60000);
  RA.rolling(2, 9*60000);
  RA.rolling(2, 10*60000);
#endif
  pf("Rolling(0) %f (1.5) Rolling(5*60000) %f (2.0) Rolling(6*60000) %f (1.83..) average %f (1.5)\n",
  RA.getRolling(11*60000-1), RA.getRolling(5*60000), RA.getRolling(6*60000), RA.getAverage());
*/
}

void loop() {
  // put your main code here, to run repeatedly:   
}

#endif //MAIN
