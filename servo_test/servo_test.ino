// Lets test the servo
#include <Servo.h>

#define DEFAULT_PULSE_WIDTH 0

Servo servo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo.attach(9);
  servo.write(0);
  delay(5000);
  servo.write(180);
  
  // put your main code here, to run repeatedly:
  for(int i = 0; i<180; i++){
    servo.write(i);
  }
}

void loop() {


}
