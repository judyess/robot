/*
  I need to decide how I want the arm to reach a target. 

  if the distance between the target and the last link is less-than the length of the link + it's starting point (it's origin)
     then that link can reach the target, at which case, I just need to calculate the angle it needs to rotate to get there.
    
     Step A: Calculate distance and compare to the reach of a link
     Step B: Calculate necessary angle required to reach target

    else
    Do step A. for the last link, link[i] + the next-to-last link, link[i-1]. Where the origin is the next-to-last link's origin
    then calculate how much link[i-1] needs to rotate to put link[i] at a position where it's in reach of the target.
      this step is step B. but the position that link[i-1] needs to be in is just where link[i] can reach the target
    then do step B. for how much link[i] needs to rotate to reach the target.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define MIN_PULSE_WIDTH       500 
#define MAX_PULSE_WIDTH       2500 
#define FREQUENCY             60
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)
#define PI 3.1415926535897932384626433832795

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int motorSpeed = 2; 
int minTicks = 122;
int maxTicks = 614;

// starting tick positions
float tickPos0 = 368;
float tickPos1 = 368;
float tickPos2 = 368;
float tickPos3 = 368; 
float tickPos4 = 368;

struct joint{
  int pin;              
  float coords[3];      
  float length;         
  float jointPosition;  
};

const float ybase=0;
const float xbase=3.5;

joint base = {0, {xbase, ybase, 0}, 3.5, tickPos0}; // only the base's Z value will be changing
joint link1 = {1, {0, 5, 0}, 5, tickPos1};
joint link2 = {2, {0, 3.5, 0}, 3.5, tickPos2};
joint link3 = {3, {0, 0.5, 0}, 0.5, tickPos3};
joint link4 = {4, {0, 2, 0}, 2, tickPos4};
joint efx = {5, {0, 4.5, 0}, 4.5, tickPos4};

joint *jointsList[6]={&base, &link1, &link2, &link3, &link4, &efx};

void setup() {
  Serial.begin(9600); 
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pwm.setPWM(base.pin,0,base.jointPosition);
  pwm.setPWM(1,0,tickPos1);
  pwm.setPWM(2,0,tickPos2);
  pwm.setPWM(3,0,tickPos3); 
  pwm.setPWM(4,0,tickPos4); 
  Serial.println(" ");
  Serial.println("start");
}

int convertToTicks(float degrees){
  float pulse = map(degrees, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); 
  int ticks = int(float(pulse) / 1000000 * FREQUENCY * 4096); 
  return ticks;
}

float convertToDegrees(float ticks){ 
  float degrees = map(ticks, minTicks, maxTicks, 0, 180);
  return degrees;
}

/* 
calculate the distance from a target position (x,y) from some position                                                                                                                                                                                                                                                                                
*/
void distanceToTarget(float pointA, float pointB){
  float targetX = pointB[0]
  float targetY = pointB[1]
  float myPosX = pointA[0]
  float myPosY = pointA[1]
  float distanceBetweenPointAtoPointB
    distanceBetweenPointAtoPointB = sqrt(pow((targetX - myPosX),2) + pow((targetY - myPosY),2))
}

/* 
Takes an angle, outputs a position
*/
void rotateLinks(){
  int pin
  float newPosition
  float offset// the point of rotation's offset is calculated from the previous links position
  int numberOfLinks = 6
  for(int i = 0; i <= 6; i++){
    /* newPosition = (newX, newY)
    newX = ((x)*math.cos(theta)) - ((y)*math.sin(theta)) + offset.x
    newY = ((x)*math.sin(theta)) + ((y)*math.cos(theta)) + offset.y
    */
  }
}



void loop(){

}