#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  700 // this is the 'maximum' pulse length count (out of 4096)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  pwm1.begin();
  pwm1.setPWMFreq(60);

  //Set to middle to start
  pwm1.setPWM(0,0, round((SERVOMAX - SERVOMIN)/2));
  pwm1.setPWM(1,0, round((SERVOMAX - SERVOMIN)/2));
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned char inByte = 0;

  if(Serial.available() > 0) {
    inByte = Serial.read();

    int servoVal = (int)inByte;
//    Serial.print("Raw In: ");
//    Serial.println(servoVal, DEC);
    float scaleFactor = (SERVOMAX - SERVOMIN) / 255;
    servoVal = round((float)servoVal * (float)scaleFactor);
//    Serial.print("Scaled: ");
//    Serial.println(servoVal);
    servoVal += SERVOMIN;
//    Serial.print("Shifted to Range: ");
//    Serial.println(servoVal);
    
    if(servoVal > SERVOMAX) servoVal = SERVOMAX;
    if(servoVal < SERVOMIN) servoVal = SERVOMIN;
//    Serial.print("After Capping: ");
//    Serial.println(servoVal);
//    Serial.println("==============");
    
    pwm1.setPWM(0,0,servoVal);
  }
}
