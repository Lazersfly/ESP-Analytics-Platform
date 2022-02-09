// Class Looptime

#include "looptime.h"
#include "pf.h"

LoopTime::LoopTime() {
  NoofLoops = 0;
  Total = 0;
  Max = 0;
  Min = 0xffffffff;
}

void LoopTime::start() {
  StartTime = micros();
}

void LoopTime::end() {
  unsigned long time;

  time = micros() - StartTime;
  NoofLoops++;
  if (time > Max)
    Max = time;
  if (time < Min)
    Min = time;
  Total += time;
}

void LoopTime::print(const char *name) {
  pfln("%s cnt %lu avg %lu min %lu max %lu", (name == NULL)?"Looptime":name, NoofLoops, Total/NoofLoops, Min, Max);
}

void LoopTime::clear() {
  NoofLoops = 0;
  Total = 0;
  Max = 0;
  Min = 0xffffffff;
}

//#define MAIN
#ifdef MAIN

LoopTime Test;
LoopTime Testnoirq;

void setup() {
  Serial.begin(115200); 
  pf("Starting\n");
}

void loop() {
  static int loops = 0;
  static unsigned long d = 100;

  Test.start();
  // delayMicroseconds only works to 16383uS
  if (d > 16000)
    delay(d/1000);
  else
    delayMicroseconds(d);
  Test.end();
  noInterrupts();
  Testnoirq.start();
  // No need to worry about delayMicro not working
  // As with irq of times over 1024 dont work anyway
  delayMicroseconds(d);
  Testnoirq.end();
  interrupts();
  loops++;
  if (loops >= 1000) {
    pfln("Delay %iuS",d);
    pf("With irqs    ");
    Test.print();
    Test.clear();
    pf("Without irqs ");
    Testnoirq.print();
    Testnoirq.clear();
    loops = 0;
    d += 100;
    delay(500);
  }

}
#endif
