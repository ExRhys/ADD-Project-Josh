#include <Servo.h>
#include <PID_v1.h>
#include <Stepper.h>

Servo theta;  // create servo object to control a servo
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11) ;
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
}

void crane_theta(point) {
  theta.write(crane_control[point][0])
}

int* distance_sensor() {
  //read sensor values here
  return {Sensor1,Sensor2}
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
