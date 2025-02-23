#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define MIN_PULSE_WIDTH       500 
#define MAX_PULSE_WIDTH       2500 
#define FREQUENCY             60
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)
#define PI 3.1415926535897932384626433832795

// #define getArraySize(array) (sizeof(array)/sizeof(int)) ----- Always returns 1. Do not use. Will have to hard code the array sizes in
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

float target[3] = {7,6,0};

struct joint{
  int pin;              
  float coords[3];      
  float length;         
  float jointPosition;  
};
struct rotationMatrix{
  float theta;
  float x[3];
  float y[3];
  float z[3];
};

joint base = {0, {0, 0, 3.5}, 3.5, 0}; // only the base's Z value will be changing
joint link1 = {1, {0, 0, link1.length}, 5, tickPos1};
joint link2 = {2, {0, 0, link2.length}, 3.5, tickPos2};
joint link3 = {3, {0, 0, link3.length}, 0.5, tickPos3};
joint link4 = {4, {0, 0, link4.length}, 2, tickPos4};
joint efx = {5, {0, 0, efx.length}, 4.5, tickPos5};
joint *jointsList[6]={&base, &link1, &link2, &link3, &link4, &efx};


float *inputAngle; 
void setAngle(float angle){ // WORKS. CORRECTLY SETS THE ANGLE FOR THE ROTATION MATRICES TO USE
  *inputAngle = angle;
}
 rotationMatrix xaxis = {
  *inputAngle,
  {1, 0, 0},
  {0, cos(*inputAngle * M_PI / 180.0), -sin(*inputAngle * M_PI / 180.0)},
  {0, sin(*inputAngle * M_PI / 180.0), cos(*inputAngle * M_PI / 180.0)}
  };
 rotationMatrix yaxis = {
  *inputAngle,
  {cos(*inputAngle * M_PI / 180.0), 0, sin(*inputAngle * M_PI / 180.0)},
  {0, 1, 0},
  {0, -sin(*inputAngle * M_PI / 180.0), cos(*inputAngle * M_PI / 180.0)}
  };
 rotationMatrix zaxis = {
  *inputAngle,
  {cos(*inputAngle * M_PI / 180.0), -sin(*inputAngle * M_PI / 180.0), sin(*inputAngle * M_PI / 180.0)},
  {sin(*inputAngle * M_PI / 180.0), cos(*inputAngle * M_PI / 180.0), 0},
  {0, 0, 1}
  };

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
  initialize();
  printLinkList();
  setAngle(90); // WORKS... but only here. If I call this in the loop I get overflow stuff
}

void printArray(int array[], int arraySize){
  for (int i=0; i<arraySize; i++){
    Serial.print(array[i]);
    Serial.print(",");
  } Serial.println();
}

void initialize(){ 
  float jLength;
  float iLength;
  float total;
  for(int i=1; i <= 5; i+=1){
    iLength = jointsList[i]->length;
      for(int j=i-1; j>=0; j-=1){
        jLength = jointsList[j]->length;
        total = jointsList[j]->length + jointsList[i]->coords[2];
        jointsList[i]->coords[2] = total;
      }
    jointsList[i]->coords[2] = total;
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
void printLinkList(){
  for(int i=0; i<6;i++){
    joint *link = jointsList[i];
    Serial.print("pin: ");
    Serial.print(i);
    Serial.print(", ");
    Serial.print(link->pin);
    Serial.print(" : ");
    Serial.print(link->coords[0]);
    Serial.print(", ");
    Serial.print(link->coords[1]);
    Serial.print(", ");
    Serial.println(link->coords[2]);
  }
}

float  distanceToTarget(float pointA[3], float pointB[3]){ // this is just on a 2D plane. 
  float targetX = pointB[0];
  float targetY = pointB[1];
  float myPosX = pointA[0];
  float myPosY = pointA[1];       
  float distance = sqrt(pow((targetX - myPosX),2) + pow((targetY - myPosY),2));
  Serial.print("Distance: "); 
  Serial.println(distance); // the hypotenuse
  return distance;
}
void getCoords(joint *link, float theta){
  float distance = link->length * cos(degreesToRadians(theta)); // WORKS
  link -> coords[0] = (distance); // should this be absolute?
  Serial.print("Link's new coords: (");
  Serial.print(link -> coords[0]);
  Serial.print(", ");
  float height = link->length * (sin(degreesToRadians(theta))); //WORKS
  link -> coords[1] = (height); // should this be absolute?
  Serial.print(" y: ");
  Serial.print(link -> coords[1]);
  Serial.println(")");
}
/*
Calls distanceToTarget(), getCoords()
*/
float requiredAngleToReachTarget(joint *link){  // for a 2D plane
  float d = distanceToTarget(link->coords, target);
  float x = d - jointsList[link->pin + 1]->length;
  float theta = radiansToDegrees(asin(x/d));
  Serial.println(theta);
  getCoords(link, theta);
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
  /* rotate on X
      X   Y   Z
  X   1   0   0
  Y   0 cosθ  -sinθ
  Z   0 sinθ  cosθ

  /* rotate on Y
      X    Y     Z
  X  cosθ  0   sinθ
  Y  0     1   0
  Z  -sinθ 0   cosθ

  /* rotate on Z
      X     Y     Z
  X  cosθ  -sinθ  sinθ    
  Y  sinθ  cosθ   0     
  Z  0     0      1     
  */

//*******************************************************************************
//*******************************************************************************

void multiplyMatrices(joint *link, rotationMatrix *axis){ // WORKSSSSS, for single links at least. 2nd rotations after updating coords are wrong though. 
  float newVector[3] = {0,0,0};
  Serial.println(axis->theta);
    for(int i=0; i<=2; i++){
      Serial.print(i);
      Serial.print(": ");
      Serial.print(newVector[0]);
      Serial.print(", ");
      Serial.print(newVector[1]);
      Serial.print(", ");
      Serial.print(newVector[2]);
      newVector[0] += newVector[i] + (axis->x[i] * (link->coords[i]));
      newVector[1] += newVector[i] + (axis->y[i] * (link->coords[i]));
      newVector[2] += newVector[i] + (axis->z[i] * (link->coords[i]));
      Serial.print("  =>  ");
      Serial.print(newVector[0]);
      Serial.print(", ");
      Serial.print(newVector[1]);
      Serial.print(", ");
      Serial.println(newVector[2]);
    }
    /* // BLOCKED OUT BECAUSE UPDATING THE LINK OBJECTS ARE CAUSING ISSUES IF I TRY ROTATING AGAIN
    for (int i = 0; i<3 ; i++){
      link->coords[i] = newVector[i];
      Serial.print(link->coords[i]);
      Serial.print(", ");
    }
    */
    Serial.println();
}



void loop(){
  float pin;
  joint *link;
  float angle = 0;
  int axis;
  rotationMatrix *rAxis;
  while(Serial.available() > 0)
  {
    pin = Serial.parseInt();
    angle = Serial.parseFloat();
    axis = Serial.parseInt();
    
    char r = Serial.read();
    if (pin == 0){
      link = &base;}
    if (pin == 1){
      link = &link1;}
    if (pin == 2){
      link = &link2;}
    if (pin == 3){
      link = &link3;}
    if (pin == 4){
      link = &link4;}
    if(r == '\n'){}

    if (axis == 0){
      rAxis = &xaxis;
      Serial.println("x-axis");}
    if (axis == 1){
      rAxis = &yaxis;
      Serial.println("y-axis");}
    if(axis == 2){
      rAxis = &zaxis;
      Serial.println("z-axis");
    }
    //setAngle(angle); // will update axis->theta but causes overflow and matrix multiplication is weird
    delay(100);
    //requiredAngleToReachTarget(link);
    //Serial.println(pin);
    multiplyMatrices(link, rAxis);

  }
}