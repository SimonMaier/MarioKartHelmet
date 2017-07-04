#include "Joystick.h"

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_MULTI_AXIS, 4, 0, true, true, true,  // 4 buttons (0,1,2,3), 3 axes (X,Y,Z)
  false, false, false, false, false, false, false, false); // Nothing else

void setup() {
  Joystick.setXAxisRange(-127, 127);
  Joystick.setYAxisRange(-127, 127);
  Joystick.setZAxisRange(-127, 127);
  Joystick.begin(false);  // if false, Joystick.sendState() must be called to send updated values

  pinMode(A0, INPUT_PULLUP);
  pinMode(13, OUTPUT);  // Onboard indicator LED

  // calibrate
}



void loop() {
//  // Button on A0 -Y reset (do with interrupt instead)
//  if (digitalRead(A0) != 0) {
//    digitalWrite(13, 0);
//    return;
//  }
  digitalWrite(13, 1);

  Joystick.setXAxis(1);
  Joystick.setYAxis(2);
  Joystick.pressButton(1); //  button is int 0-4
//  Joystick.releaseButton(button);
  Joystick.sendState();
}

