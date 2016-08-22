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


bool fillingCommand = false;
int numCommandBytes = 0;
const int HORIZ_MIDDLE = round(((SERVOMAX - SERVOMIN)/2)+SERVOMIN);
unsigned char commandBytes[4] = {HORIZ_MIDDLE,HORIZ_MIDDLE,HORIZ_MIDDLE,HORIZ_MIDDLE};
unsigned char overrunBytes[4];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  pwm1.begin();
  pwm1.setPWMFreq(60);

  //Set to middle to start
  pwm1.setPWM(0,0, round((SERVOMAX - SERVOMIN)/2)+SERVOMIN);
  pwm1.setPWM(1,0, round((SERVOMAX - SERVOMIN)/2)+SERVOMIN);
  pwm1.setPWM(2,0, round((SERVOMAX - SERVOMIN)/2)+SERVOMIN);
  pwm1.setPWM(3,0, round((SERVOMAX - SERVOMIN)/2)+SERVOMIN);
}

void loop() {
  //TODO: Capture all bytes and update as needed
  //for now drop partial commands, wait for init byte
  unsigned char inByte = 0;
  int numBytes = Serial.available();
  for(int i = 0; i < numBytes; i++){
      inByte = Serial.read();
      //Serial.print("Byte: ");
      //Serial.println(inByte);
      //Serial.print("===== NUM BYTES: ");
      //Serial.println(numBytes);
      if((int)inByte == 255){
        //We are now filling the command
        fillingCommand = true;
        numCommandBytes = 0;
        //Serial.println("******************** Filling Command...");
      } else if(fillingCommand && numCommandBytes < 4){
        commandBytes[numCommandBytes] = inByte;
        numCommandBytes++;
      } else if(fillingCommand && numCommandBytes >= 4){
        fillingCommand = false;
        //Serial.println("$$$$$$$$$$$$$$$$$$$ Full Command...");
      }
  }

  if(fillingCommand && numCommandBytes < 4){
    //Serial.println("=== DIDN'T GET ALL BYTES IN THIS READ");
  }

  //Move to the last command sent
  int servoVal;
  
  //left eye horizontal
  servoVal = (int)commandBytes[0];
  servoVal = map(servoVal,0,254,SERVOMIN,SERVOMAX);
  pwm1.setPWM(0,0,servoVal);
  
  //left eye vertical
  servoVal = (int)commandBytes[1];
  servoVal = map(servoVal,0,254,SERVOMIN,SERVOMAX);
  pwm1.setPWM(1,0,servoVal);
  
  //right eye horizontal 
  servoVal = (int)commandBytes[2];
  servoVal = map(servoVal,0,254,SERVOMIN,SERVOMAX);
  pwm1.setPWM(2,0,servoVal);
  
  //right eye vertical
  servoVal = (int)commandBytes[3];
  servoVal = map(servoVal,0,254,SERVOMIN,SERVOMAX);
  pwm1.setPWM(3,0,servoVal);
  
/* 
  if(Serial.available() > 0) {
    inByte = Serial.read();

    int servoVal = (int)inByte;
    servoVal = map(servoVal,0,254,SERVOMIN,SERVOMAX);
    //Serial.print("Shifted to Range: ");
    //Serial.println(servoVal);
    
    pwm1.setPWM(0,0,servoVal);
    pwm1.setPWM(1,0,servoVal);
*/
}
