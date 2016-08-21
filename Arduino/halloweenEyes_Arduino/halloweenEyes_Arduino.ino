#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  160 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  pwm1.begin();
  pwm1.setPWMFreq(60);

  //Set to middle to start
  pwm1.setPWM(0,0, round((SERVOMAX - SERVOMIN)/2)+SERVOMIN);
  pwm1.setPWM(1,0, round((SERVOMAX - SERVOMIN)/2)+SERVOMIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned char inByte = 0;

  if(Serial.available() > 0) {
    inByte = Serial.read();

    int servoVal = (int)inByte;
    servoVal = map(servoVal,0,255,SERVOMIN,SERVOMAX);
    //Serial.print("Shifted to Range: ");
    //Serial.println(servoVal);
    
    pwm1.setPWM(0,0,servoVal);
    pwm1.setPWM(1,0,servoVal);
  }
}
