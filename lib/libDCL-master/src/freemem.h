// Returns amount of free memory BSS to stack
#pragma once

class FreeMem {
public:
  void check();		// Call from deepest nesting or interrupt routine
  unsigned long get() { check(); return(min_free_mem_); };
private:
  volatile unsigned long min_free_mem_ = 0xffffffff;
};
