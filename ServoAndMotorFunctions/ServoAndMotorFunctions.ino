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

/*
Moves the car forward by a set speed
*/
void forward (int speed) {
  motor.setSpeed(speed);
}


/*
Moves the car backwards by a set speed
*/
void backward (int speed) {
  motor.setSpeed(-speed);
}


/*
Turn the car left by a given value. Value should fall within the range MIDDLE_POINT to LEFT_POINT
or 285 to 325. 
*/
void turnLeft (int left) {

  // If left is greater than the max left or less than/equal to the middle, set to value to midpoint between MIDDLE_POINT and LEFT_POINT
  if (left > LEFT_POINT || left <= MIDDLE_POINT) {
    
    // 325 + 20 = 345 
    left = MIDDLE_POINT + ((LEFT_POINT - MIDDLE_POINT) / 2);
  }
  pwm.setPWM(servonum, 0, left);

}


/*
Turn the car left by a given value. Value should fall within the range RIGHT_POINT to MIDDLE_POINT
or 325 to 365. 
*/
void turnRight (int right) {

  // If right is less than the min right or greater than/equal to the middle, set to value to midpoint of RIGHT_POINT AND MIDDLE_POINT 
  if (right < RIGHT_POINT || right >= MIDDLE_POINT) {
    
    // 285 + 20 = 305
    right = RIGHT_POINT + ((MIDDLE_POINT - RIGHT_POINT) / 2);
  }
  pwm.setPWM(servonum, 0, right);

}



