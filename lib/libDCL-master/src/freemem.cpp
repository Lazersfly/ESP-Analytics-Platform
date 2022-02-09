// Class to return the number of bytes currently free in RAM      
#include <Arduino.h>
#include "freemem.h"

void FreeMem::check() { 
  uint free_memory = ESP.getFreeHeap();
  if (free_memory < min_free_mem_)
    min_free_mem_ = free_memory;
}
