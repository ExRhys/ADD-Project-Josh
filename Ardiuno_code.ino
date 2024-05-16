#include <Servo.h>
#include <PID_v1.h>
#include <Stepper.h>
#include <Wire.h>
#include <VL53L1X.h>

Servo theta;  // create servo object to control a servo
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11) ;
const uint8_t sensorCount = 2;
const uint8_t xshutPins[sensorCount] = {4, 5};
VL53L1X sensors[sensorCount];

int crane_point = 0;
int crane_contro[2][3] = { {90, 20, 15}, {180, 15, 10} }; //row is for option, then the 3 item array is rotation, in and out then height
int crane_height= 0
int stepCount = 0;

double Setpoint, Input, Output;
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void crane_setup() {
  theta.attach(9);  // attaches the servo on pin 9 to the servo object  
  myStepper.setSpeed(400)

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
}

void crane_theta(point) {
  theta.write(crane_control[point][0])
}

int* distance_sensor() {
  return {sensors[0].read(),sensors[1].read()}
}

void crane_depth(point) {
  int* depths = distance_sensor();
  input = (depths[0] + sensor_constant-depths[1])/2
  Setpoint = crane_control[point][1];
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output); //change PIN_OUTPUT to the output pin
}

void crane_height(point) {
  if (crane_control[point][2] > stepCount) {
    myStepper.step(20);
    stepCount++;
  } else {
    myStepper.step(-20);
    stepCount--;
  }
}

void crane_loop() {
  crane_theta(crane_point);
  crane_height(crane_point);
  crane_depth(crane_point);
}

void setup() {
  crane_setup();
}

void loop() {
  crane_loop();
}
