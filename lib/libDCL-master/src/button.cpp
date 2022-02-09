#include "button.h"

#define DEBOUNCEDELAY 30      // the debounce time in milliseconds
#define LONGPUSHDELAY 1500    // long push time in milliseconds
#define REPEATPUSHDELAY 1000  // time to start repeating
#define REPEATPUSHRATE 200    // how quickly to repeat

// Keeps track of difference between millis and mymillis so can pass a mymillis time
// on button pushs.
// Button needs to use millis as if using mymillis changes to Clock Cal and offset causes
// button to stop working
long TimeOffset;

#define DL 0x0004
/*
buttonCheck - Check which buttons have been presses or switches have changed

Call from your main Loop
*/

void Button::check()
{
  byte Reading;
  unsigned long Time;

  Time = millis();
  TimeOffset = millis() - Time;
  // read the state of the switch into a local variable:
  Reading = digitalRead(Pin);
  if (PushHigh) 
    Reading = !Reading;
  // check to see if the state has changed, if so start debounce timing
  if (Reading != LastState) {
    ChangeTime = Time;
    LastState = Reading;
    RepeatState = 0;
  }
  else if ((Time - ChangeTime) >= DEBOUNCEDELAY) {
    // debouncing finished
    switch (Type) {
      case SWITCH:
      if (CurrentState != Reading) {
        // State change
        if (pushCall)     //Check for null function
          pushCall(0, Pin, ChangeTime + TimeOffset);
      }
      break;
      case PUSH:
      if ((CurrentState)  && (!Reading))  // button pushed
        pushCall(0, Pin, ChangeTime + TimeOffset);  
      break;
      case PUSHREPEAT:
      switch (RepeatState) {
        case 0:     // waiting for push
        if ((CurrentState) && (!Reading)) { 
          pushCall(0, Pin, ChangeTime + TimeOffset);
          RepeatState = 1;
        }
        break;
        case 1:     // had push waiting to see if repeats
        if ((!CurrentState) && (!Reading)) {
          if ((Time - ChangeTime) >= REPEATPUSHDELAY) {
            pushCall(0, Pin, Time + TimeOffset);
            ChangeTime = Time;
            RepeatState = 2;
          }
        }
        break;
        case 2:     // Repeating
        if ((!CurrentState) && (!Reading)) {       
          if ((Time - ChangeTime) >= REPEATPUSHRATE) {
            pushCall(0, Pin, Time + TimeOffset);
            ChangeTime = Time;                
          }
        } 
      }
      break;
      case LONGPUSH:
      if (!CurrentState) {
        if (Reading) {
          if (!LongPushCalled)
            pushCall(0, Pin, ChangeTime + TimeOffset);
          LongPushCalled = 0;
        }
        else if (!LongPushCalled && ((Time - ChangeTime) >= LONGPUSHDELAY)) {
          pushCall(1, Pin, ChangeTime + TimeOffset);
          LongPushCalled = 1;
        }
      }
      break;
    }
    CurrentState = Reading;
  }
}

// Button Constructor
Button::Button(byte Pin, byte Type, void (*push)(bool, byte, unsigned long), bool PushHigh)
{ 
  pinMode(Pin, INPUT_PULLUP);
  this->Pin = Pin;
  pushCall = push; 
  this->Type = Type;
  LastState = CurrentState = 1;
  this->PushHigh = PushHigh;
}

//#define MAIN
#ifdef MAIN
#include "pf.h"

void push(bool Long, byte Pin, unsigned long Time)
{
  static unsigned long PushTime = 0;
  unsigned long Elasped;
  
  pf("%s push pin %hhu time %lu\n",Long?"Long":"Short",Pin,Time);
  if (PushTime == 0) {
    PushTime = Time;
    pf("Start\n");
  }
  else {
    Elasped = (Time - PushTime);
    pf("%lu ", Elasped);
    Elasped /= 1000;
    pf("%lu %02lu:%02lu:%02lu\n",Elasped, Elasped/3600, (Elasped%3600)/60, Elasped%60);
  }
}

// buuton in 
Button C = Button(8, LONGPUSH, &push);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pf("\nSetup\n");  
}

void loop()
{
  static unsigned long PrintTime = 0;
  unsigned long Time;
  // put your main code here, to run repeatedly:
  C.check();
     
  Time = millis();
  if (Time > (PrintTime + 1000)) {
    pf("%hhu ",C.getState()); // print state once a second
    PrintTime = Time;
  }

  delay(10);
}


#endif
