// Kyle Sherman
// Created 12/24/2019

// This is the initial software for the parachute release
// uses a kalman filter to help eliminate some of the noise in the sensor

// These are the necessary libraries to work with the servo and MBP-180 pressure sensor
#include <SFE_BMP180.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <Servo.h>

#define DEFAULT_PULSE_WIDTH 0 // stops the servo from being set based on pulse

// instantiate a servo object and a sensor object
Servo servo;
SFE_BMP180 pressure;
SimpleKalmanFilter pressureFilter(1, 1, 0.01);


/** 
Create some variables that we will need for the components
**/
int seconds = 0;
int refresh = 1000; // 1 second refresh rate

double baseline; // this is for the baseline pressure
boolean running;  
boolean descending = false; // this will be set to true if the altitude is descending

const int serv_i = 180; // servo intial position 
const int serv_r = 0; // servo release position 
const int serv_pin = 9; // servo pin

const int spkr_pin = 8; // speaker pin

int rAlt = 100; // altitude to release
double rVel = -5.0; // velocity to release

double lastAlt = 0; // initialize the last alt to zero

// Beep codes
// 1 beep (2 seconds)
int bp_success = 1; // if all devices are found
int bp_fail = 2;    // if sensor is not found

void setup() {

  //tone(spkr_pin, 1000, 2000);
  
  delay(3000); // 3 second pause to give everything a chance to start up
  
  Serial.begin(9600); // establish the serial stream
  Wire.begin();       // initialize the sensor
  
  delay(2000); // 2 second pause

  servo.attach(serv_pin); // sets the servo to pin d9
  servo.write(serv_i); // initial servo position
  
  // check if we can see the sensor
  if(!pressure.begin()) {
    Serial.println("Error finding sensor");
    for(int i = 0; i < bp_fail; i++){
      //tone(spkr_pin, 1000, 250);
      delay(1000);
    }
    while(1); // infinite loop. Connect sensor and reboot
  } else {
    Serial.println("sensor connected");
    //tone(spkr_pin, 1000, 1000);
  }

  baseline = readPressure();


  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" hpa");
  delay(5000);
}

void loop() {
  double a = lastAlt, P, vel, a_filter;
 
  P = readPressure();
 
  a = pressure.altitude(P, baseline);
  a_filter = pressureFilter.updateEstimate(a);
  vel = getVelocity(lastAlt, a_filter);

//  Serial.print("MET: ");
//  Serial.print(seconds);
//  Serial.print(" seconds ");
//  Serial.print(" ");
//  // if (a >= 0.00) Serial.print(" ");
//  // Serial.print(a, 2);
//  //Serial.print(" meters | ");
//  Serial.print(" filtered alt:");
//  Serial.print(a_filter, 2);
//  //Serial.print(" meters | velocity ");
//  Serial.print(" ");
//  Serial.print(vel);
//  Serial.println(" m/s");
//  //tone(spkr_pin, 1000, 250);

  if(a_filter >= rAlt && vel <= rVel){
    servo.write(0);
  }

  lastAlt = a_filter;
  seconds = seconds+1;
  delay(refresh);
}

double getVelocity(double last, double next){
  double v, lastAlt = last, nextAlt = next;

  v = ((nextAlt-lastAlt)/1);
  
  
  return v;
}

double readPressure()
{
  char status;
  double T, P, p0, a;
  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
        // else Serial.println("error retrieving pressure measurement\n");
      }
      // else Serial.println("error starting pressure measurement\n");
    }
    // else Serial.println("error retrieving temperature measurement\n");
  }
  // else Serial.println("error starting temperature measurement\n");
}
