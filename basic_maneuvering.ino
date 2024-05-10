/*
Code for basic maneuvering of car (forward, backward, left, right)

Requirements: HC-06/05 module paired with phone
              A phone app that can send char/int commands
              TA6586 motor driver
              Servo
*/
#include <SoftwareSerial.h>
#include <Servo.h>

SoftwareSerial BTSerial(9, 10);

Servo steering;
const int right_limit = 170;
const int left_limit = 10;
int turn_angle = 0;

uint8_t yaw_angle = 0;

const int speed = 250;
const int D0 = 5;
const int D1 = 6;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  BTSerial.begin(9600);

  steering.attach(3);
  steering.write(90);
}

void loop() {


  if (BTSerial.available() > 0) {
    char incoming_value = BTSerial.read();
    Serial.println(incoming_value);

    switch (incoming_value) {
      case 'F':
        // move forward
        analogWrite(D0, speed);
        analogWrite(D1, LOW);
        break;
      case 'B':
        // move backward
        analogWrite(D0, LOW);
        analogWrite(D1, speed);

        break;
      case 'R':
        // turn right
        steering.write(right_limit);
        break;
      case 'L':
        // turn left
        steering.write(left_limit);
        break;
      case 'S':
        // stop and center
        analogWrite(D0, LOW);
        analogWrite(D1, LOW);
        steering.write(90);
        break;
    }
  }
}
