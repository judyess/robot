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
float tickPos0 = 0;
float tickPos1 = 368;
float tickPos2 = 368;
float tickPos3 = 368; 
float tickPos4 = 368;
float tickPos5 = 368;

struct joint{
  int pin;              
  float coords[3];      
  float length;         
  float jointPosition;  
};

const float xbase=0;
const float ybase=3.5;

joint base = {0, {xbase, ybase, 0}, 3.5, 0}; // only the base's Z value will be changing
joint link1 = {1, {0, link1.length, 0}, 5, tickPos1};
joint link2 = {2, {0, link2.length, 0}, 3.5, tickPos2};
joint link3 = {3, {0, link3.length, 0}, 0.5, tickPos3};
joint link4 = {4, {0, link4.length, 0}, 2, tickPos4};
joint efx = {5, {0, efx.length, 0}, 4.5, tickPos5};

joint *jointsList[6]={&base, &link1, &link2, &link3, &link4, &efx};

float target[3] = {7,6, 0};

void setup() {
  Serial.begin(19200); 
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pwm.setPWM(base.pin,0,base.jointPosition);
  pwm.setPWM(1,0,tickPos1);
  pwm.setPWM(2,0,tickPos2);
  pwm.setPWM(3,0,tickPos3); 
  pwm.setPWM(4,0,tickPos4); 
  pwm.setPWM(5,0,tickPos5); 
  initialize();
  Serial.println(" ");
  Serial.println("start");
  print();
}

void initialize(){
  for(int i=0; i<6;i++){
    joint *previousLink;
    float prevX = 0;
    float prevY = 0;
    if(i!=0){
      prevX = jointsList[i-1]->coords[0];
      prevY = jointsList[i-1]->coords[1];
    }
    else{
      prevX = xbase;
      prevY = ybase;
      i++;
    }
    joint *link = jointsList[i];
    float angle = convertToDegrees(link->jointPosition);
    float x = (link->length * (cos(degreesToRadians(angle)))) + prevX; 
    link -> coords[0] = (x); 
    float y = (link->length * (sin(degreesToRadians(angle)))) + prevY; 
    link -> coords[1] = (y);
    //previousLink = jointsList[i];
    Serial.println(angle);
  }
}

void print(){
  //joint *previousLink = &base;
  for(int i=0; i<6;i++){
    joint *link = jointsList[i];
    Serial.print("pin: ");
    Serial.print(i);
    Serial.print(", ");
    Serial.print(link->pin);
    Serial.print(" : ");
    Serial.print(link->coords[0]);
    Serial.print(", ");
    Serial.println(link->coords[1]);
    //previousLink = jointsList[i];
  }
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
calculate the distance between two positions.                                                                                                                                                                                                                                                                                
*/
float  distanceToTarget(float pointA[3], float pointB[3]){
  float targetX = pointB[0];
  float targetY = pointB[1];
  float myPosX = pointA[0];
  float myPosY = pointA[1];       
  float distance = sqrt(pow((targetX - myPosX),2) + pow((targetY - myPosY),2));   //distance Between PointA to PointB
  Serial.println(distance);
  return distance;
}

/*
for linki to reach the target, point T, 
  then link[i-1] needs to move into a position where the distance between it's end position and point T is equal to the length of link[i]
  this requires getting the distance from distanceToTarget(link[i-1], point T)
*/
float moveToReach(joint *link){
  float d = distanceToTarget(link->coords, target);
  float x = d - jointsList[link->pin + 1]->length;
  float theta = radiansToDegrees(asin(x/d));
  Serial.println(theta);
  position(link, theta);
  return theta;
}

/*
I have to set an initial angle, and everytime a joint rotates, I have to update the new position. 
So this should assume that it is always receiving a current and valid value.
DistanceToTarget() and moveToReach() gets the necessary angle that a link should turn. 
So, this should take an angle. Then calculate the new position from the angle of rotation and the current position.
But this also needs to take into consideration that all links will be moving with it.
Refer to the code in linksModel.py for the function that handles this.
*/
void position(joint *link, float theta){
    int n = 6;
  for(int i=link->pin; i < n; i++){
    joint *previousLink = jointsList[0];
    float prevX;
    float prevY;
    if(i!=0){
      prevX = jointsList[i-1]->coords[0];
      prevY = jointsList[i-1]->coords[1];
    }
    else{
      prevX = jointsList[0]->coords[0];
      prevY = jointsList[0]->coords[1];
      i++;
    }
    
    joint *link = jointsList[i];
    float newX = (((link->coords[0])*cos(degreesToRadians(theta))) - ((link->coords[1] * sin(degreesToRadians(theta))))) + prevX;
    float newY = (((link->coords[0])*sin(degreesToRadians(theta))) + ((link->coords[1] * cos(degreesToRadians(theta))))) + prevY;
    previousLink = jointsList[i];
    Serial.print("pin: ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(newX);
    Serial.print(", ");
    Serial.println(newY);
  }
}



void loop(){
  float pin, angle;
  joint *link;
  angle = 0;
  while(Serial.available() > 0)
  {
    pin = Serial.parseInt();
    angle = Serial.parseFloat();
    char r = Serial.read();
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
      if(r == '\n'){}
    moveToReach(link);
  }
}