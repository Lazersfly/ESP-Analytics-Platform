/*
Generates a rolling average by dividing into a fixed no of fixed time buckets   
                                                                                
It can return the rolling average over a time up to no of buckets * length of   
bucket or the average since startup.                                            
                                                                                
Note time is an arbiatary unit, it could be seconds, microseconds or any time   
unit. Time in a call to rolling should be greater than the previous call but
this is not checked which allows the use of Time that wraps round.                                                               
                                                                                
It is intended that rolling should be called many times per period but it       
works with infrequent calls and even with periods with no calls. It assumes     
after a call to rolling the Value stays constant till the next call.            
On a call to get_rolling the average is calculated stating from the last        
rolling call. This has the effect that the Value in the last call to rolling    
is ignored in the calculation.
                                                                                
Classes:                                                                        
    RollingAv                                                             
                                                                                
Methods:
    RollingAv                                                                       
    RollingAv(Time)      No of Time units for rolling average            
    rolling(float, Time) call repeatably with a Value(float) and Time           
    getRolling(Time)     Returns rolling for the last Time units if 0 all periods                             
    getAverage()         Returns average since startup                             
*/
#pragma once

#include <Arduino.h>

#define NOOFPERIODS 10

class RollingAv {
public:
  RollingAv(unsigned long);
  void rolling(float, unsigned long);
  float get_rolling(unsigned long = 0) const;
  float get_average() const;
  void reset();
  void debug() const;
private:
  void clear_array();
  unsigned long period_;             // Length of period
  float array_[NOOFPERIODS+1];// First entry is latest period
  unsigned long current_start_;          // StartTime of latest rolling average period
  unsigned long last_call_ = 0;           // Last rolling call time
  unsigned long rolling_start_;      // Start time of rolling 
  float average;                    // Average since reset
  unsigned long average_start_;  // Start time of average
};
