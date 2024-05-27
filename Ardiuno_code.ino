#include <Servo.h>
#include "AccelStepper.h"


#define motorPin1  2      // IN1 on the ULN2003 driver
#define motorPin2  4      // IN2 on the ULN2003 driver
#define motorPin3  5     // IN3 on the ULN2003 driver
#define motorPin4  6     // IN4 on the ULN2003 driver

#define MotorInterfaceType 8

Servo theta;  // create servo object to control a servo
AccelStepper craneStepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

int crane_point = 0;
int crane_control[2][3] = { {10, 90, 10000}, {40, 180, 0} }; //row is for option, then the 3 item array is rotation, in and out then height
int stepCount = 0;
int pos = 45;


unsigned long lastRun = 0;
void crane_theta(int point) {
  //if (lastRun - millis() > 10000) {
  if (pos < crane_control[point][0]) {
    theta.write(pos+1);
    pos = pos + 1;
  } else if (pos > crane_control[point][0]) {
    theta.write(pos-1);
    pos = pos - 1;
  }
  //lastRun = millis();
  Serial.println(pos);
  //}
}


void crane_change_height(int point) {
  if (crane_control[point][2] < craneStepper.currentPosition()) {
    craneStepper.setSpeed(-1000);
    Serial.println("a");
  } else if (crane_control[point][2] > craneStepper.currentPosition()) {
    craneStepper.setSpeed(1000);
    Serial.println("b");
  } else {
    Serial.println("c");
    craneStepper.setSpeed(0);
  }
  craneStepper.runSpeed();
}

void crane_loop() {
  if (digitalRead(10)) {
  crane_point = digitalRead(8);
  //crane_theta(crane_point);
  crane_change_height(crane_point);
  digitalWrite(9, !crane_point);
  }
}

void setup() {
  pinMode(8, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  crane_setup();
  Serial.begin(9600);
}

void loop() {
  crane_loop();
}

void crane_setup() {
  theta.attach(3);  // attaches the servo on pin 9 to the servo object  
  craneStepper.setMaxSpeed(1000);
  craneStepper.setCurrentPosition(0);
  pinMode(9, OUTPUT);
}
