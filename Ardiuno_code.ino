#include <Servo.h>
//#include <PID_v1.h>
#include "AccelStepper.h"
#include <Wire.h>
#include <SparkFun_TB6612.h>
//#include <VL53L1X.h>

#define AIN1 2
#define AIN2 4
#define PWMA 5
#define STBY 9

#define motorPin1  8      // IN1 on the ULN2003 driver
#define motorPin2  9      // IN2 on the ULN2003 driver
#define motorPin3  10     // IN3 on the ULN2003 driver
#define motorPin4  11     // IN4 on the ULN2003 driver

#define MotorInterfaceType 8

Servo theta;  // create servo object to control a servo
AccelStepper craneStepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

const int offsetA = 1;
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
//const uint8_t sensorCount = 2;
//const uint8_t xshutPins[sensorCount] = {4, 5};
//VL53L1X sensors[sensorCount];

int crane_point = 0;
int crane_control[2][3] = { {90, 20, 15}, {180, 15, 10} }; //row is for option, then the 3 item array is rotation, in and out then height
int crane_height = 0;
int stepCount = 0;
int sensor_constant = 10;

double Setpoint, Input, Output;
double Kp=2, Ki=5, Kd=1;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void crane_theta(int point) {
  theta.write(crane_control[point][0]);
}

int* distance_sensor() {
  int readings[2] = {0,0};// {sensors[0].read(),sensors[1].read()};
  return readings;
}

void crane_depth(int point) {
  /*int* depths = distance_sensor();
  Input = (depths[0] + sensor_constant-depths[1])/2;
  Setpoint = crane_control[point][1];
  //myPID.Compute();
  analogWrite(1, Output); //change PIN_OUTPUT to the output pin*/

  if (point) {
    motor1.drive(50);
  } else {
    motor1.drive(-50);
  }
}

void crane_change_height(int point) {
  if (point) {//crane_control[point][2] > stepCount) {
    craneStepper.setSpeed(1000);
  } else {
    craneStepper.setSpeed(-1000);
  }
  craneStepper.runSpeed();
}

void crane_loop() {
  crane_point = digitalRead(4);
  Serial.println(crane_point);
  crane_theta(crane_point);
  crane_change_height(crane_point);
  //crane_depth(crane_point);
}

void setup() {
  Serial.begin(9600);
  pinMode(4, INPUT_PULLUP);
  crane_setup();
}

void loop() {
  crane_loop();
}

void crane_setup() {
  theta.attach(3);  // attaches the servo on pin 9 to the servo object  
  craneStepper.setMaxSpeed(1000);
  craneStepper.setCurrentPosition(0);
  /*

  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(0x2A + i);

    sensors[i].startContinuous(50);  
  }

  */
}
