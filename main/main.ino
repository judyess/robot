// functions to test servos
// Base link, link 1, link 2, link 3, link 4, end effector link
// link 1 == link2 when link 2 is at 90 degrees

/* FORMULAS
    theta = arctan(y/x);      Get angle from (x,y) coordinates

    

  */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <math.h>

#define MIN_PULSE_WIDTH       500 
#define MAX_PULSE_WIDTH       2500 
#define FREQUENCY             60
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)
#define PI 3.1415926535897932384626433832795

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int delayTime = 5;

// tick positions ; 368 is 90 degrees
float tickPosA = 368;
float tickPosB = 368;
float tickPosC = 368;
float tickPosD = 368; 
float tickPosE = 368;

struct motor{
  int pin;
  float x;
  float y;
  float z;
  float length;
  float motorAngle;
};

motor base = {0, 0, 0, 0, 0, tickPosA};
motor link1 = {1, 0, 4.75, 0, 4.75, tickPosB};
motor link2 = {2, 0, 3.5, 0, 3.5, tickPosC};
motor link3 = {3, 0, 0, 0, 4, tickPosD};
motor link4 = {4, 0, 0, 0, 0, tickPosE};

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pwm.setPWM(base.pin,0,base.motorAngle);
  pwm.setPWM(1,0,tickPosB);
  pwm.setPWM(2,0,tickPosC);
  pwm.setPWM(3,0,tickPosD);
}

// Conversion Functions
float degreeToPWM(float angle){ // --WORKS--
  float pulse = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); // converts degrees to a pulse value
  int ticks = int(float(pulse) / 1000000 * FREQUENCY * 4096); // converts the pulse values to "tick" values
  return ticks;
}

void getPosition(float theta, int i){
  motor *link;
  if (i == 0){
    link = &base;
  }
  if (i == 1){
    link = &link1;
  }
  if (i == 2){
    link = &link2;
  }
  if(i == 3){
    link = &link3;
  }
  if (i == 4){
    link = &link4;
  }
  float betaB = 90 - theta; // this gets the angle complementary to theta
  float distance = link->length * sin(degreesToRadians(betaB)); // WORKS
  link -> x = abs(distance);
  Serial.print(" x: ");
  Serial.println(link -> x);

  float height = link->length * (sin(degreesToRadians(theta))); //WORKS
  link -> y = abs(height);
  Serial.print(" y: ");
  Serial.println(link -> y);
}

void checkPulse(int pin, float angle) { // a test function
  int pulse_ticks, degreePulse;
  Serial.println(angle);
  motor *link; // using if statements bc too lazy to pass objects rn
  if (pin == 0){
    link = &base;
  }
  if (pin == 1){
    link = &link1;
  }
  if (pin == 2){
    link = &link2;
  }
  if(pin == 3){
    link = &link3;
  }
  if (pin == 4){
    link = &link4;
  }
  pulse_ticks = degreeToPWM(angle);
  float current = link -> motorAngle; // initializes a reference variable to the current position
  if(current < pulse_ticks){
    for(float pos = current; pos <= pulse_ticks; pos+=1){
      pwm.setPWM(pin, 0, pos);
      link -> motorAngle = pos;
      delay(delayTime);
    }
  }
  if(current > pulse_ticks){
    for(float pos = current; pos >= pulse_ticks; pos-=1){
      pwm.setPWM(pin, 0, pos);
      link -> motorAngle = pos;
      delay(delayTime);
    }
  }
  Serial.print("-----");
  Serial.print("Motor: ");
  Serial.println(pin);
  getPosition(angle, pin);
}
// LINKS ARE HARD CODED
void l2Parallel(float angle){ //Keep L2 parallel to floor. --WORKS--
  float theta1, theta2;
  theta1 = angle;
  if (theta1 < 90){
    theta2 = 180 - (90 - theta1);
  }
  if (theta1 > 90){
    theta2 = theta1 - 90;
  }
  if(theta1 == 90){
    theta2 = 90;
  }
  Serial.print("theta1: ");
  Serial.println(theta1);
  Serial.print("theta2: ");
  Serial.println(theta2);
  motor *linkp, *linkq;
  linkp = &base;
  linkq = &link1;
  // Link1
  int pwm1 = degreeToPWM(theta1);
  float current = linkp -> motorAngle; 
  if(current < pwm1){
    for(float pos = current; pos <= pwm1; pos+=1){
      pwm.setPWM(0, 0, pos);
      linkp -> motorAngle = pos;
      delay(delayTime);
    }
  }
  if(current > pwm1){
    for(float pos = current; pos >= pwm1; pos-=1){
      pwm.setPWM(0, 0, pos);
      linkp -> motorAngle = pos;
      delay(delayTime);
    }
  }
  // Link2
  int pwm2 = degreeToPWM(theta2);
  float current2 = linkq -> motorAngle; 
  if(current2 < pwm2){
    for(float pos2 = current2; pos2 <= pwm2; pos2 += 1){
      pwm.setPWM(1, 0, pos2);
      linkq -> motorAngle = pos2;
      delay(delayTime);
    }
  }
  if(current2 > pwm2){
    for(float pos2 = current2; pos2 >= pwm2; pos2 -= 1){
      pwm.setPWM(1, 0, pos2);
      linkq -> motorAngle = pos2;
      delay(delayTime);
    }
  }
  Serial.println("---------------");
}
// L2 is always linear with L1 when L2 is at 90 degrees
void l2equalL1(float angle){ //linearly equivalent. L2 constantly at 90 degrees
  float theta2;
  theta2 = 90;
}
// L2 is perpendicular to to the ground when theta1 and theta 2 are the same
void l2Perpendicular(float angle){ // keep L2 at 90 degrees to floor. (or perpendicular)
  float theta1, theta2;
  theta1 = degreeToPWM(angle);
  theta2 = theta1;
  pwm.setPWM(0, 0, theta1);
  pwm.setPWM(0, 0, theta2);
}


void loop() {
  float pin, angle;
  int i;
  angle = 0;
  while(Serial.available() > 0)
  {
    pin = Serial.parseInt();
    angle = Serial.parseInt();
    //i = Serial.parseInt();
    char r = Serial.read();
    if(r == '\n'){}
    checkPulse(pin,angle);
    //l2Parallel(angle);
  }
}
