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
  initializeY();
  delay(100);
  print();
}
/*
  The total height of the arm at start. 
  I have each of my links starting at 90 degrees (links extend straight up) so the y-coords are just an accumulation of link lengths
  idk what that last print line is (with the 3 digit pin and some random +'s)
*/
void initializeY(){ 
  float jLength;
  float iLength;
  float total;
  for(int i=0; i <= 6; i+=1){
    iLength = jointsList[i]->length;
    Serial.print(jointsList[i]->pin);
    Serial.print(": ");
    Serial.print(iLength);
      for(int j=i-1; j>=0; j-=1){
        jLength = jointsList[j]->length;
        total = jointsList[j]->length + jointsList[i]->coords[1];
        jointsList[i]->coords[1] = total;
        Serial.print(" + ");
        Serial.print(jLength);
      }
      jointsList[i]->coords[1] = total;
      Serial.print(" = ");
      Serial.println(total);
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
float requiredAngleToReachTarget(joint *link){
  float d = distanceToTarget(link->coords, target);
  float x = d - jointsList[link->pin + 1]->length;
  float theta = radiansToDegrees(asin(x/d));
  Serial.println(theta);
  return theta;
}

//******************************ROTATION MATRICES********************************
/* ROTATION MATRICES BECAUSE TRYING TO DO THE MATH OTHER WAYS KEEPS BEING WEIRD
//*******************************************************************************
              [x]   |x'|
  (RMatrix) * [y] = |y'|
              [z]   |z'|

  for each ROW of Rrc's, where r=row label, c=column label. (Each row[i] refers to the R elements in it's own row.)
   row[X] =   x'  =  Rxx*[x] + Rxy*[y] + Rxz*[z]
   row[Y] =   y'  =  Rxx*[x] + Rxy*[y] + Rxz*[z]
   row[Z] =   z'  =  Rxx*[x] + Rxy*[y] + Rxz*[z]
*/
void rotationMatrixOnX(){
  /* rotate on X
      X   Y   Z
  X   1   0   0
  Y   0 cosθ  -sinθ
  Z   0 sinθ  cosθ
  */
  float *rotMatrix[3];
  float theta = 90;
  float rowX[3] = {1, 0, 0};
  float rowY[3] = {0, cos(degreesToRadians(theta)), -sin(degreesToRadians(theta))};
  float rowZ[3] = {0, sin(degreesToRadians(theta)), cos(degreesToRadians(theta))};
  rotMatrix[0] = rowX;
  rotMatrix[1] = rowY;
  rotMatrix[2] = rowZ;
}
void rotationMatrixOnY(){
  /* rotate on Y

      X    Y     Z
  X  cosθ  0   sinθ
  Y  0     1   0
  Z  -sinθ 0   cosθ

  */
  float *rotMatrix[3];
  float theta = 90;
  float rowX[3] = {cos(degreesToRadians(theta)), 0, sin(degreesToRadians(theta))};
  float rowY[3] = {0, 1, 0};
  float rowZ[3] = {0, -sin(degreesToRadians(theta)), cos(degreesToRadians(theta))};
  rotMatrix[0] = rowX;
  rotMatrix[1] = rowY;
  rotMatrix[2] = rowZ;
}
void rotationMatrixOnZ(){
  /* rotate on Z
      X     Y     Z
  X  cosθ  -sinθ  sinθ    
  Y  sinθ  cosθ   0     
  Z  0     0      1     
  */
  float *rotMatrix[3];
  float theta = 90;
  float rowX[3] = {cos(degreesToRadians(theta)), -sin(degreesToRadians(theta)), sin(degreesToRadians(theta))};
  float rowY[3] = {sin(degreesToRadians(theta)), cos(degreesToRadians(theta)), 0};
  float rowZ[3] = {0, 0, 1};
  rotMatrix[0] = rowX;
  rotMatrix[1] = rowY;
  rotMatrix[2] = rowZ;
  
}

//*******************************************************************************


void loop(){
  float pin, angle;
  joint *link;
  angle = 0;
  float theta = 0;
  while(Serial.available() > 0)
  {
    pin = Serial.parseInt();
    angle = Serial.parseInt();
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
    theta = requiredAngleToReachTarget(link);
    print();
  }
}