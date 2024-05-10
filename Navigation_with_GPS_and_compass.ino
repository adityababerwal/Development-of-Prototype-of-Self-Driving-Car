/*
 * This code uses compass heading and GPS course values to estimate steering turn angle.
 * 
 * Requirements: Neo-6M GPS module
 *               HMC5883L compass
 *               
 * Compass gives where the car is currently facing (wrt North)
 * GPS gives the direction in which the waypoint is wrt current location (wrt North)
 * If the difference of those two angles is zero, means the car is facing the waypoint. Then we just have to drive the car straight.
 * PID is used to minimise that difference.
 */

//------------------Includes----------------------
#include <SoftwareSerial.h>
#include <Servo.h>
#include "HMC5883L.h"
#include <TinyGPSPlus.h>
#include "Wire.h"

//--------------Bluetooth-----------------------
SoftwareSerial BTSerial(9, 10); //(RX,TX)
SoftwareSerial GPSSerial(11, 12); //(RX,TX)

char incoming_value = 'S';

//---------------Servo--------------------------
Servo steering;
const int right_limit = 170;
const int left_limit = 10;
int turn_angle = 90;

//-----------------Motor-----------------------------
struct {
  const int speed = 250;
  const int D0 = 5;
  const int D1 = 6;
} Motor;

//-----------------Goal coordinates------------------
struct {
  double lat = 0;
  double lng = 0;
} Goal;

//-----------------Current coordinates--------------
// To check if the gps is working
struct {
  double lat = 0;
  double lng = 0;
} locationNow;

//----------------time------------------
int prevTime = 0;
//-------------------PID---------------------------
const int8_t Kp = 7;
const int8_t Ki = 0.55;
const int8_t Kd = 0;
int prev_error = 0;
double lastError = 0, cumError = 0, rateError = 0;

int computePID(double inp) {
  int Setpoint = 0;
  int currentTime = millis();
  double elapsedTime = (double)(currentTime - prevTime);
  int Error = inp - Setpoint;
  cumError += Error * elapsedTime;

  cumError = (cumError > 100) ? int(cumError) % 100 : cumError;
  rateError = (Error - lastError) / elapsedTime;
  lastError = Error;

  int out = Kp * Error + Ki * cumError + Kd * rateError;

  return abs(out) < 5 ? 0 : out;
}

//--------------Compass------------------------
HMC5883L compass;
int compass_heading = 0;

void getCompass()  // get latest compass value
{
  Vector norm = compass.readNormalize();
  float heading = atan2(norm.YAxis, norm.XAxis);
  if (heading < 0)
    heading += 2 * M_PI;
  compass_heading = (int)(heading * 180 / M_PI);  // assign compass calculation to variable (compass_heading) and convert to integer to remove decimal places
}

//----------------GPS-------------------------
int GPS_Course;
TinyGPSPlus gps;
double Distance_to_Goal = 0;

void getGPS()  // Get Latest GPS coordinates
{
  if (GPSSerial.available() > 0)
    gps.encode(GPSSerial.read());

  locationNow.lat = gps.location.lat();
  locationNow.lng = gps.location.lng();
}

//----------------Go To Goal---------------
void goWaypoint() {
  analogWrite(Motor.D0, Motor.speed);
  analogWrite(Motor.D1, LOW);

  Distance_to_Goal = INT_MAX;

  while (Distance_to_Goal > 2) {

    getCompass();  // Update Compass heading
    getGPS();     // Tiny GPS function that retrieves GPS data - update GPS location
    
    Distance_to_Goal = TinyGPSPlus::distanceBetween(locationNow.lat, locationNow.lng, Goal.lat, Goal.lng);  // update distance
    GPS_Course = TinyGPSPlus::courseTo(locationNow.lat, locationNow.lng, Goal.lat, Goal.lng);  //Query Tiny GPS for Course to Destination

    if (abs(GPS_Course - compass_heading) >= 10)  // If GPS Course and the Compass Heading are within x degrees of each other then go Forward                                                                                                                            // otherwise find the shortest turn radius and turn left or right
    {
      int res = computePID(GPS_Course - compass_heading);
      if (res > 90)
        res = 90;
      else if (res < -90)
        res = -90;
      turn_angle = res + 90;
      steering.write(turn_angle);
    }
  }  // End of While Loop

  analogWrite(Motor.D0, LOW);
  analogWrite(Motor.D1, LOW);
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
        steering.write(90);
        break;

      case 'W': // Set current location as goal
        Goal.lat = gps.location.lat();
        Goal.lng = gps.location.lng();
        break;

      case 'G': // go to set goal location
        goWaypoint();
        break;
    }
  }
}

// --------------------------SETUP---------------------
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  BTSerial.begin(9600);

  steering.attach(3);
  steering.write(90);

  Wire.begin();

  compass.begin();
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);

  prevTime = millis();
}

// ------------------------------LOOP--------------------
void loop() {
  // put your main code here, to run repeatedly:
  bluetooth();
  getGPS();
  getCompass();
  prevTime = millis();
}
