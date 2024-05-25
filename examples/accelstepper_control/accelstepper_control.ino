
#include "AccelStepper.h"
#define motorPin1  8      // IN1 on the ULN2003 driver
#define motorPin2  9      // IN2 on the ULN2003 driver
#define motorPin3  10     // IN3 on the ULN2003 driver
#define motorPin4  11     // IN4 on the ULN2003 driver

#define MotorInterfaceType 8
int crane_point = 1;
AccelStepper craneStepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

void crane_change_height(int point) {
  if (point) {
    craneStepper.setSpeed(-1000);
  } else {
    craneStepper.setSpeed(1000);
  }
  craneStepper.runSpeed();
}

void crane_loop() {
  crane_point = digitalRead(4);
  crane_change_height(crane_point);
}

void setup() {
  pinMode(4, INPUT_PULLUP);
  crane_setup();
}

void loop() {
  crane_loop();
}

void crane_setup() {
  craneStepper.setMaxSpeed(1000);
  craneStepper.setCurrentPosition(0);
}