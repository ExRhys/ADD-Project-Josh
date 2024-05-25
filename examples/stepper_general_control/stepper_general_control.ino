#include <Stepper.h>

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution of motor

Stepper myStepper(stepsPerRevolution, 7,6,5,4); //Initialise your stepper motor on pins 8-11

void setup() {
myStepper.setSpeed(60); //sets the speed of the motor in rpm
pinMode(9, OUTPUT);
}

void loop() {
  digitalWrite(9, HIGH);
  myStepper.step(stepsPerRevolution); //used to move motor a certain number of steps, negative values will step the motor backwards
}