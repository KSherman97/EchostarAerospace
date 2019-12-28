/**
* Kyle Sherman
* Echostar Aerospace
*/

#include <SFE_BMP180.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <Servo.h>

#define DEFAULT_PULSE_WIDTH 0

Servo servo;
SFE_BMP180 pressure;
SimpleKalmanFilter pressureFilter(1, 1, 0.01);

double baseline;

const int serv_i = 180;
const int serv_r = 0;
const int serv_pin = 9;

const int spkr_pin = 8;

int rAlt = 100;
double rVel = -10.0;
double rAccel = 10;
int rCounter = 0;
int rCount = 3;

int bufferCounter = -1;
int bufferCount = 5; // number of seconds we will use before trying to print text. will prevent serial buffer from filling up.

float lastAlt = 0;
float lastVel = 0;

int bp_success = 1; // if all devices are found
int bp_fail = 2;    // if sensor is not found

const int tickSec = 0.5;
const int tickMS = tickSec*1000;

boolean running = true;

void setup() {

  //tone(spkr_pin, 1000, 2000);
  
  Serial.begin(115200); // establish the serial stream
  // Wire.begin();       // initialize the sensor
  
  delay(2000);
  servo.attach(serv_pin); // sets the servo to pin d9
  servo.write(serv_i); // initial servo position
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
  baseline = readPressure();
  //Serial.print("baseline pressure: ");
  //Serial.print(baseline);
  //Serial.println(" hpa");
  delay(1000);
}

void loop() {
  if (!running) return;
  
  float P = readPressure();
  float a = pressure.altitude(P, baseline); 
  float a_filter = pressureFilter.updateEstimate(a);
  float vel = getVelocity(lastAlt, a_filter);
  float accel = getAcceleration(lastVel, vel);

  //Serial.print(a,2);
  //Serial.print(",");
  Serial.print("Altitude ");
  Serial.print(a_filter,2);
  Serial.print(" m");
  Serial.print(",");
  Serial.print("Velocity ");
  Serial.print(vel, 2);
  Serial.print(" m/s");
  Serial.print(accel, 2);
  Serial.print(" m/s^2 ");
  Serial.print("Count: ");
  Serial.print(rCounter); 
  Serial.println();

  if(a_filter <= rAlt && lastAlt <= rAlt && vel <= rVel && accel >= rAccel){
    rCounter = rCounter+1;
    if(rCounter >= rCount){
      servo.write(serv_r);
      delay(2000);
      servo.write(serv_i);
      delay(1000);
      running = false;
    }
  }
  else {
    rCounter = 0;
  }

  lastAlt = a_filter;
  lastVel = vel;
  delay(tickMS);
}

double getVelocity(float last, float next){
  float v, lastAlt = last, nextAlt = next;
  v = ((nextAlt-lastAlt)/(tickSec));
  return v;
}

double getAcceleration(float last, float next) {
  float a, lastVel = last, nextVel = next;
  a = ((nextVel - lastVel)/(tickSec));
  return a;
}

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
