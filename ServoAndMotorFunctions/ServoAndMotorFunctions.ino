/*************************************************** 
Servo and Motor Functions
****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "CytronMotorDriver.h"


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  490 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define MIDDLE_POINT  325  // Straight 
#define LEFT_POINT  365  // Left Max
#define RIGHT_POINT  285  // Right Max


CytronMD motor(PWM_DIR, 3, 8);

uint8_t servonum = 0;


void setup() {

  /*
  Setting up Servo
  */
  Serial.begin(9600);
  // Serial.println("Servo test!");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); 
  delay(10);


  /*
  Test Calls
  */

  pwm.setPWM(servonum, 0, 325); // middle point for driving (straight)
  delay(500);

  // motor.setSpeed(100);  // Run forward at 100 speed.
  // delay(3000);

  // pwm.setPWM(servonum, 0, 285); // left turn max
  // delay(1300);

  // pwm.setPWM(servonum, 0, 325); // middle point for driving (straight)
  // delay(3000);

  // motor.setSpeed(0);  // Stop.

  motor.setSpeed(-100);  // Run bakcward at 100 speed.
  delay(10000);

  motor.setSpeed(0);  // Stop.

}


void loop() {


}


/*
Set speed to slow down gradually to a stop by taking steady increments
*/
void coastSpeed (int currSpeed) {

  int speedDec = currSpeed;
  int speedPrct = currSpeed / 20;
  
  for (int i = 0; i <= 20; i++) {
    speedDec -= speedPrct;
    motor.setSpeed(speedDec);
    delay(100);
  }
  
  motor.setSpeed(0);

} 

