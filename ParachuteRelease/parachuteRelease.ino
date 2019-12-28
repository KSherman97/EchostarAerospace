/**
* Kyle Sherman
* Echostar Aerospace
* 
* note: 
* All flight data is in metric units
*/

// these libraries are all necessary
#include <SFE_BMP180.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <Servo.h>

// This stops the servo motor from setting to
// a default value upon the start of the software
#define DEFAULT_PULSE_WIDTH 0 

// create an instance of the servo, pressure, and the kalman filter
Servo servo;
SFE_BMP180 pressure;
SimpleKalmanFilter pressureFilter(1, 1, 0.01);

// create a baseline variable
double baseline;

// constants for the servo
const int serv_i = 180; // the initial position
const int serv_r = 0; // the position to release
const int serv_pin = 9; // set the pin we connect the servo to

const int spkr_pin = 8; // the pin the speaker is connected to

// constants to define the conditions to release the parachute
const int rAlt = 100; // release altitude
const double rVel = -10.0; // release velocity
const double rAccel = 10; // release acceleration
const int rCounter = 0; // counts how many consecutive ticks the conditions have been met
const int rCount = 3; // the number of consectutive ticks the conditions must be met

// the buffer counter variables will probably be removed. I wanted the data to continue to be processed
// while the data is processed at the defined tick rate, we only want to print the data every 5 seconds
int bufferCounter = -1; // initialize the buffer counter to -1
int bufferCount = 5; // number of seconds we will use before trying to print text. will prevent serial buffer from filling up.

// variables used for the velocity and acceleration calculations
float lastAlt = 0; // used to calculate velocity (delta Alt / delta time)
float lastVel = 0; // used to calculate acceleration (delta vel / delta time)

// constants used to define the beep codes
// I will revisit these later
const int bp_success = 1; // if all devices are found
const int bp_fail = 2;    // if sensor is not found

// constants for the tick value
// these are self explanitory
const float tickSec = 1;          // value in seconds
const float tickMS = tickSec*1000; // convert to ms

// boolean to check if the program is running
boolean running = true;

void setup() {

  //tone(spkr_pin, 1000, 2000);
  
  Serial.begin(9600); // establish the serial stream
  Wire.begin();       // initialize the sensor
  
  delay(2000);
  servo.attach(serv_pin); // sets the servo to pin d9
  servo.write(serv_i); // initial servo position

  // this makes sure the pressure sensor is detected. If not, then dont continue
  if(!pressure.begin()) {
    Serial.println("Error finding sensor");
    for(int i = 0; i < bp_fail; i++){
      //tone(spkr_pin, 1000, 250);
      delay(1000);
    }
    while(1);
  } else {
    //Serial.println("sensor connected");
    //tone(spkr_pin, 1000, 1000);
  }

  // set the baseline pressure measurement
  baseline = readPressure();
  //Serial.print("baseline pressure: ");
  //Serial.print(baseline);
  //Serial.println(" hpa");
  delay(1000);
}

void loop() {

  // STOP if the program running is false
  if (!running) return;

  // set some values
  float P = readPressure(); // read the pressure at current time
  float a = pressure.altitude(P, baseline);  // calculate the altitude in relation to the baseline (starting pressure)
  float a_filter = pressureFilter.updateEstimate(a); // run the altitude through the kalman filter
  float vel = getVelocity(lastAlt, a_filter); // find the velocity 
  float accel = getAcceleration(lastVel, vel); // find the acceleration

  //Serial.print(a,2);
  //Serial.print(",");
  Serial.print(" Altitude ");
  Serial.print(a_filter,2);
  Serial.print(" m ");
  Serial.print(" Velocity ");
  Serial.print(vel, 2);
  Serial.print(" m/s ");
  Serial.print(accel, 2);
  Serial.print(" m/s^2 ");
  Serial.print(" Count: ");
  Serial.print(rCounter); 
  Serial.println();

  // this statement checks to see if the conditions are met, if they are then increment the counter
  if(a_filter <= rAlt && lastAlt <= rAlt && vel <= rVel && accel >= rAccel){
    rCounter = rCounter+1;

    // if the counter value is equal to or greater than the count value then we will release the parachute
    if(rCounter >= rCount){
      servo.write(serv_r);
      delay(2000);
      servo.write(serv_i);
      delay(1000);
      running = false;
    }
  }
  else {
    // reset the counter because we are looking for concurrent counts
    rCounter = 0;
  }

  // lets set the current altitude and velocity to the previous values
  lastAlt = a_filter;
  lastVel = vel;
  delay(tickMS);
}

// velocity function
double getVelocity(float last, float next){
  float v, lastAlt = last, nextAlt = next;
  v = ((nextAlt-lastAlt)/(tickSec));
  return v;
}

// acceleration function
double getAcceleration(float last, float next) {
  float a, lastVel = last, nextVel = next;
  a = ((nextVel - lastVel)/(tickSec));
  return a;
}

// calculate the pressure
double readPressure(){
  char status;
  double T,P;
  status = pressure.startTemperature();
  if (status != 0) {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0) {
      status = pressure.startPressure(3);
      if (status != 0) {
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0) {
          return(P);
        }
      } 
    }  
  } 
}
