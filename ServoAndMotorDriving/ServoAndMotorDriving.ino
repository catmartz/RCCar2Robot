/*************************************************** 
Servo and Motor Driving Together
****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "CytronMotorDriver.h"


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  490 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

CytronMD motor(PWM_DIR, 3, 8);

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

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

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() {

  // pwm.setPWM(servonum, 0, 325); // middle point for driving (straight)
  // delay(500);

  // motor.setSpeed(50);  // Run forward at 50 speed.
  // delay(2000);

  // pwm.setPWM(servonum, 0, 285); // left turn max
  // delay(50);

  // motor.setSpeed(50);  // Run forward at 50 speed.
  // delay(2000);

  // motor.setSpeed(128);  // Run forward at 50% speed.
  // delay(1000);
  
  // motor.setSpeed(255);  // Run forward at full speed.
  // delay(1000);

  // motor.setSpeed(0);    // Stop.
  // delay(1000);

  // motor.setSpeed(-128);  // Run backward at 50% speed.
  // delay(1000);
  
  // motor.setSpeed(-255);  // Run backward at full speed.
  // delay(1000);

  // motor.setSpeed(0);    // Stop.
  // delay(1000);


  // Drive each servo one at a time using setPWM()
  // for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
  //   pwm.setPWM(servonum, 0, pulselen);
  // }

  // delay(1000);
  // for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
  //   pwm.setPWM(servonum, 0, pulselen);
  // }


  // delay(500);

  // pwm.setPWM(servonum, 0, SERVOMIN);

  // delay(2000);

  // pwm.setPWM(servonum, 0, SERVOMAX);

  // delay(2000);

  // pwm.setPWM(servonum, 0, 325); // middle point for driving (straight)

  // delay(2000);

  // pwm.setPWM(servonum, 0, 365); // right turn max

  // delay(2000);

  // pwm.setPWM(servonum, 0, 285); // left turn max

  // delay(2000);

  // delay(2000);


}