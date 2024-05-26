/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(3);  // attaches the servo on pin 9 to the servo object
  pinMode(0, INPUT_PULLUP);
}

void loop() {
  if (digitalRead(0)) {
  for (pos = 0; pos <= 45; pos += 1) { // goes from 0 degrees to 180 degrees
    if (digitalRead(0)) {
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(100);           
    }            // waits 15 ms for the servo to reach the position
  }
  for (pos = 45; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    if (digitalRead(0)) {
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(100);           
    }            // waits 15 ms for the servo to reach the position
  }
  }
}
