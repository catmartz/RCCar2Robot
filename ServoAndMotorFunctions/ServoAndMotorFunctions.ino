/*************************************************** 
Servo and Motor Functions
****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "CytronMotorDriver.h"
#include "NewPing.h"

#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  490 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define MIDDLE_POINT  325  // Straight 
#define RIGHT_MAX  365  // Right Max
#define LEFT_MAX  285  // Left Max

// Hook up HC-SR04 with Trig to Arduino Pin 9, Echo to Arduino pin 10
#define TRIGGER_PIN 9
#define ECHO_PIN 10

// Maximum distance we want to ping for (in centimeters).
#define MAX_DISTANCE 400  

// Adafruit Servo Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// NewPing setup of pins and maximum distance.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Cytron setup of motor driver
CytronMD motor(PWM_DIR, 3, 8);

// Servo Count
uint8_t servonum = 0;


/*
Function prototypes to set default values for optional parameters
*/
void backward (int speed, int time = NULL);
void forward (int speed, int time = NULL);


/*
Setting up Servo
*/
void setup() {

  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); 
  delay(10);

  // Reset servo to middle point for driving (straight)
  pwm.setPWM(servonum, 0, 325); 
  
  // Make sure motor is stopped before running other code
  motor.setSpeed(0);

  Serial.print("Setup Complete");


  // SAMPLE CALLS

  // forward(100, 2000);
  // turnLeft(285);
  // forward(150, 1700);

  // turnRight(320);
  // forward(100, 1000);
  // resetSteering();
  // forward(50, 1000);

}


void loop() {
  printUltrasonic(500);
}


/*
Set speed to slow down gradually to a stop by taking steady increments
*/
void coastSpeed (int currSpeed) {

  int speedDec = currSpeed;
  int speedPrct = currSpeed / 20;
  
  for (int i = 0; i <= 5; i++) {
    speedDec -= speedPrct;
    motor.setSpeed(speedDec);
    delay(100);
  }
  
  motor.setSpeed(0);

} 

/*
Stops the motor of the car.
*/
void stop() {
  motor.setSpeed(0);
}

/*
Moves the car forward by a set speed. Optional parameter to specify delay time.
If no delay time is passed in, the car will move forward indefinitely. 
*/
void forward (int speed, int time) {

  motor.setSpeed(speed);
  if (time != NULL) {
    delay(time);
    stop();
  }
}


/*
Moves the car backward by a set speed. Optional parameter to specify delay time.
If no delay time is passed in, the car will move backward indefinitely. 
*/
void backward (int speed, int time) {

  motor.setSpeed(-speed);
  if (time != NULL) {
    delay(time);
    stop();
  }
}




/*
Turn the car right by a given value. Value should fall within the range MIDDLE_POINT to RIGHT_MAX
or 325 to 365. Maximum right turn is 365. Medium right turn is 345.
*/
void turnRight (int right) {

  // If right is greater than the max right or less than/equal to the middle, set to value to midpoint between MIDDLE_POINT and RIGHT_MAX
  if (right > RIGHT_MAX || right <= MIDDLE_POINT) {
    
    // 325 + 20 = 345 
    right = MIDDLE_POINT + ((RIGHT_MAX - MIDDLE_POINT) / 2);
  }
  pwm.setPWM(servonum, 0, right);

}


/*
Turn the car left by a given value. Value should fall within the range LEFT_MAX to MIDDLE_POINT
or 285 to 325. Maximum left turn is 285. Middle left turn is 305.
*/
void turnLeft (int left) {

  // If left is less than the min left or greater than/equal to the middle, set to value to midpoint of RIGHT_POINT AND MIDDLE_POINT 
  if (left < LEFT_MAX || left >= MIDDLE_POINT) {
    
    // 285 + 20 = 305
    left = LEFT_MAX + ((MIDDLE_POINT - LEFT_MAX) / 2);
  }
  pwm.setPWM(servonum, 0, left);

}

/*
Reset the steering / servo to the middle point.
*/
void resetSteering() {
  pwm.setPWM(servonum, 0, MIDDLE_POINT);
}

/*
Print out distances from the ultrasomic sensor using a given delayInterval in ms. 
To see print output, go to Tools > Serial Monitor
*/
void printUltrasonic(int delayInterval) {
  
  Serial.print("Distance = ");
  Serial.print(sonar.ping_cm());
  Serial.println(" cm");
  delay(delayInterval);
}


void navigate(){
  int distance = sonar.ping_cm();
  Serial.print(distance);
  if (distance > 20 || distance == 0) {
    forward(50, 0);
    distance = sonar.ping_cm();
  } 
  else{
    backward(50, 2000);
    turnLeft(300);
    forward(50,2000);
    resetSteering();
  }
}
