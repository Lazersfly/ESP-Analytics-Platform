/*
Button class for Arduino

The button class debounces push buttons and switches. It can detect short or 
long button pushes. Push button and switches should be connected between the input poin and 0V,
no pullup is required.

The constructor Button is called with
Pin Arduino pin number
Type Type of button or switch
pushCall Function to call (optional for Type SWITCH)
PushHigh Set if push makes button go high (default false)

pushCall is called with Long, Pin and the Time (in milliseconds) the change was
first detected. Time may be useful because the pushCall will be
called a time after the actual push (particuarly in the case of LONGPUSH).

pushCall for Type SWITCH is called when a debounced change is detected
either high to low or vice versa. getState must be called to find out which.

For Type PUSH pushCall is called when the button is pressed. Long is always 0.

For Type PUSHREPEAT if the button is held down multiple pushCalls are made

For Type LONGPUSH pushCall is called when the long push time has been reached on a long push (Long = 1)
or when the button is released a short push (Long = 0)
*/
#pragma once

#include <Arduino.h>

// class Button
class Button 
{
public:
  Button(byte Pin, byte Type, void (*push)(bool, byte, unsigned long) = NULL, bool PushHigh = 0);
  bool getState() const { return (CurrentState); }
  void check();
private: 
  byte Type;
  byte Pin;
  bool PushHigh;
  bool CurrentState;    // debounced state
  bool LastState;       // last reading
  byte RepeatState = 0;
  bool LongPushCalled = 0;      // long push called while still pushed
  unsigned long ChangeTime;
  void (*pushCall)(bool Long, byte Pin, unsigned long Time);
 
};


// Types
#define SWITCH 0    // Switch
#define PUSH 1      // Push button doesn't look for long pushes
#define PUSHREPEAT 2 // Push button repeats
#define LONGPUSH 3  // Push button detects long or short pushes
