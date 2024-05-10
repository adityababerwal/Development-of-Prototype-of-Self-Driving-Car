/*
 * This simple arduino code uses commands from phone app received over bluetooth to drive the car,
 * while using PID to keep the car going straight even though there is play in joints.
 * 
 * Requirements: MPU6050 IMU sensor
 *               TA6586 motor driver
 *               Servo
 *               Any phone app that can send char/int commands
 *               
 *               Library used: MPU6050_light
 */

//------------------Includes----------------------
#include <SoftwareSerial.h>
#include <Servo.h>
#include <MPU6050_light.h>
#include "Wire.h"

MPU6050 mpu(Wire);
double yaw_angle = 0;
//--------------Bluetooth-----------------------
SoftwareSerial BTSerial(9, 10);

char incoming_value = 'S';

//---------------Servo--------------------------
Servo steering;
const int right_limit = 170;
const int left_limit = 10;
int turn_angle = 90;

//-----------------Motor-----------------------------
struct {
  const int speed = 150;
  const int D0 = 5;
  const int D1 = 6;
} Motor;

//----------------time------------------
uint64_t prevTime = 0;
//-------------------PID---------------------------
const int8_t Kp = 11;
const int8_t Ki = 0.55;
const int8_t Kd = 0;
int prev_error = 0;
double lastError = 0, cumError = 0, rateError = 0;
int Setpoint = 0;

int computePID(double inp) {
  int currentTime = millis();
  double elapsedTime = (double)(currentTime - prevTime);
  int Error = inp - Setpoint;
  cumError += Error * elapsedTime;

  cumError = (cumError > 100) ? int(cumError) % 100 : cumError;
  rateError = (Error - lastError) / elapsedTime;
  lastError = Error;

  int out = Kp * Error + Ki * cumError + Kd * rateError;

  prevTime = millis();

  return abs(out) < 5 ? 0 : out;
}

//----------------------Bluetooth--------------
void bluetooth() {
  if (BTSerial.available() > 0) {
    incoming_value = BTSerial.read();

    switch (incoming_value) {
      case 'F':
        // move forward
        analogWrite(Motor.D0, Motor.speed);
        analogWrite(Motor.D1, LOW);
        break;
      case 'B':
        // move backward
        analogWrite(Motor.D0, LOW);
        analogWrite(Motor.D1, Motor.speed);
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
        analogWrite(Motor.D0, LOW);
        analogWrite(Motor.D1, LOW);
        break;
      case 'C':
        Setpoint = yaw_angle;
        break;

    }
  }
}

// --------------------------SETUP---------------------
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  BTSerial.begin(9600);
  Wire.begin();


  steering.attach(3);
  steering.write(90);

  byte status = mpu.begin();
  while(status != 0){
    status = mpu.begin();
    Serial.println("quering mpu");
  }
  Serial.println("mpu running");


  prevTime = millis();
}

// ------------------------------LOOP--------------------
int waypoint_index = 0;
void loop() {
  // put your main code here, to run repeatedly:
  mpu.update();
  yaw_angle = mpu.getAngleZ();
  bluetooth();

  // incoming_value is a global variable, it stays 'C' even if no other input from BT is received
  // Hence to keep the car going straight, use PID, because there is some play in joints
  if(incoming_value == 'C'){
    turn_angle = computePID(yaw_angle);

    if(turn_angle > 90) turn_angle = 90;
    if(turn_angle < -90) turn_angle = -90;

    steering.write(90+turn_angle);
    Serial.println(90+turn_angle);
   
  }

}
