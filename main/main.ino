/* Written for a 6R-DOF manipulator. 

 FORMULAS
    theta = arctan(y/x);      Get angle from (x,y) coordinates
    Link 1 == Link 2, when Joint 2 is 90 degrees
    Link 2 is parallel to the base when |Joint1 - Joint2| = 90
    Link 2 is perpendicular to the base when Joint2 == Joint1

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

struct joint{
  int pin;              // which pins on the PCA 9685 the motor is connected to.
  float coords[3];      // changed x, y, z values into an array
  float length;
  float jointPosition;     // check if this ever switches between holding tick values or degree values !!!
};

// joint objects
joint base = {0, {0, 0, 0}, 0, tickPosA};
joint link1 = {1, {0, 4.75, 0}, 4.75, tickPosB};
joint link2 = {2, {0, 3.5, 0}, 3.5, tickPosC};
joint link3 = {3, {0, 0, 0}, 4, tickPosD};
joint link4 = {4, {0, 0, 0}, 0, tickPosE};

void setup() {
  Serial.begin(9600); 
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pwm.setPWM(base.pin,0,base.jointPosition);
  pwm.setPWM(1,0,tickPosB);
  pwm.setPWM(2,0,tickPosC);
  pwm.setPWM(3,0,tickPosD);
}

/*
Parameters: an angle
  converts an angle(degrees) to a pulse(seconds), then to a pulse(ticks)
Returns: angle in ticks
*/
float convertToTicks(float angle){ // --WORKS--
  float pulse = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); 
  int ticks = int(float(pulse) / 1000000 * FREQUENCY * 4096); 
  return ticks;
}

/*
Parameters: a joint object, the desired angle in degrees
converts the angle to tick-pulses and incrementally sets a new position
*/
void setPosition(joint *joint, float newAngle){
  float newPosition = convertToTicks(newAngle);
  float previousPosition = joint -> jointPosition; 
  if(previousPosition < newPosition){
    for(float updatedPosition = previousPosition; updatedPosition < newPosition; updatedPosition++){
      pwm.setPWM(joint -> pin, 0, updatedPosition);
      joint -> jointPosition = updatedPosition;
      delay(delayTime);
    }
  }
  if(previousPosition > newPosition){
    for(float updatedPosition = previousPosition; updatedPosition > newPosition; updatedPosition--){
      pwm.setPWM(joint -> pin, 0, updatedPosition);
      joint -> jointPosition = updatedPosition;
      delay(delayTime);
    }
  } 
}

void getPosition(float theta, int i){
  joint *link;
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
  link -> coords[0] = abs(distance); // should this be absolute?
  Serial.print(" x: ");
  Serial.println(link -> coords[0]);

  float height = link->length * (sin(degreesToRadians(theta))); //WORKS
  link -> coords[1] = abs(height); // should this be absolute?
  Serial.print(" y: ");
  Serial.println(link -> coords[1]);
}

void checkPulse(int pin, float angle) { // a test function
  int pulse_ticks, degreePulse;
  Serial.println(angle);
  joint *link; // using if statements bc too lazy to pass objects rn
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
  pulse_ticks = convertToTicks(angle);
  float current = link -> jointPosition; // initializes a reference variable to the current position
  if(current < pulse_ticks){
    for(float pos = current; pos <= pulse_ticks; pos+=1){
      pwm.setPWM(pin, 0, pos);
      link -> jointPosition = pos;
      delay(delayTime);
    }
  }
  if(current > pulse_ticks){
    for(float pos = current; pos >= pulse_ticks; pos-=1){
      pwm.setPWM(pin, 0, pos);
      link -> jointPosition = pos;
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

  /*
    When |L1 - L2| = 90, then L2 will be parallel to the base.
  */
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
  joint *linkp, *linkq;
  linkp = &base;
  linkq = &link1;
  // Link1
  int pwm1 = convertToTicks(theta1);
  float currentPos = linkp -> jointPosition; 
  if(currentPos < pwm1){
    for(float pos = currentPos; pos <= pwm1; pos+=1){
      pwm.setPWM(0, 0, pos);
      linkp -> jointPosition = pos;
      delay(delayTime);
    }
  }
  if(currentPos > pwm1){
    for(float pos = currentPos; pos >= pwm1; pos-=1){
      pwm.setPWM(0, 0, pos);
      linkp -> jointPosition = pos;
      delay(delayTime);
    }
  }
  // Link2
  int pwm2 = convertToTicks(theta2);
  float currentPos2 = linkq -> jointPosition; 
  if(currentPos2 < pwm2){
    for(float pos2 = currentPos2; pos2 <= pwm2; pos2 += 1){
      pwm.setPWM(1, 0, pos2);
      linkq -> jointPosition = pos2;
      delay(delayTime);
    }
  }
  if(currentPos2 > pwm2){
    for(float pos2 = currentPos2; pos2 >= pwm2; pos2 -= 1){
      pwm.setPWM(1, 0, pos2);
      linkq -> jointPosition = pos2;
      delay(delayTime);
    }
  }
  Serial.println("---------------");
}
// L2 is always linear with L1 when L2 is at 90 degrees, no matter what position L1 is in.
void l2equalL1(float angle){ //linearly equivalent. 
  float theta2;
  theta2 = 90;
}
// L2 is perpendicular to to the ground when theta1 and theta 2 are the same
void l2Perpendicular(float angle){ 
  float theta1, theta2;
  theta1 = convertToTicks(angle);
  theta2 = theta1;
  pwm.setPWM(0, 0, theta1);
  pwm.setPWM(0, 0, theta2);
}


void loop() {
  float pin, angle;
  int i;
  joint *joint;
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
