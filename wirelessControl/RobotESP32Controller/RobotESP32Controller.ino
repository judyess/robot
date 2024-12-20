// Serial output is correct

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

Servo claw;
Servo base;

#define MIN_PULSE_WIDTH       500 
#define MAX_PULSE_WIDTH       2500 
#define FREQUENCY             60

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int xout = A0;
int yout= A1;

// tick positions
float posA = 400;
float posB = 400;

float precision = 0.5;
float delayTime = 200;

struct joint{
  int pin;              
  float len;         
  float jointPosition;  
};

joint link1 = {1, 5, posA};
joint link2 = {2, 3.5, posB};

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pwm.setPWM(1,0,posA);
  pwm.setPWM(2,0,posB);
}

void moveMotor(joint *joint, float change){
  float currentPosition = joint->jointPosition;
  float newPosition = currentPosition + change;
  joint->jointPosition = newPosition;
  Serial.println(joint->jointPosition);
  delay(delayTime);
}


void loop() {
  while(analogRead(xout) > 600){
    moveMotor(&link2, 1);
  }
  while(analogRead(xout) < 400){
    moveMotor(&link2, -1);
  }
    while(analogRead(yout) > 600){
    moveMotor(&link1, 1);
  }
  while(analogRead(yout) < 400){
    moveMotor(&link1, -1);
  }
}