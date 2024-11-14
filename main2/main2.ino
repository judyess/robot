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

struct joint{
  int pin;              
  float coords[3];      
  float length;         
  float jointPosition;  
};

const float xbase=0;
const float ybase=0;

joint base = {0, {xbase, ybase, 3.5}, 3.5, 0}; // only the base's Z value will be changing
joint link1 = {1, {0, 0, link1.length}, 5, tickPos1};
joint link2 = {2, {0, 0, link2.length}, 3.5, tickPos2};
joint link3 = {3, {0, 0, link3.length}, 0.5, tickPos3};
joint link4 = {4, {0, 0, link4.length}, 2, tickPos4};
joint efx = {5, {0, 0, efx.length}, 4.5, tickPos5};

joint *jointsList[6]={&base, &link1, &link2, &link3, &link4, &efx};
float target[3] = {7,6, 0};

double *rotMatrix[3]; // = {0, 0, 0}; //~~$$$$$$$$~~ 
int *testMatrix = malloc(sizeof(int)*3);


void setup() {
  
  Serial.begin(19200); 
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
  testMatrix[0] = 88;
  testMatrix[1] = 99;
  testMatrix[2] = 100;
  printArray(testMatrix, 3); 
  updateArray(11, 22, 33);
  printArray(testMatrix, 3);

}


void printArray(int array[], int arraySize){
  arraySize = 3; // arraySize isn't right
  for (int i=0; i<arraySize; i++){
    Serial.print(array[i]);
  }
}

void updateArray(int x, int y, int z){
  testMatrix[0] = x;
  testMatrix[1] = y;
  testMatrix[2] = z;
}

void initializeY(){ 
  float jLength;
  float iLength;
  float total;
  for(int i=0; i <= 5; i+=1){
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
void print(){
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

float  distanceToTarget(float pointA[3], float pointB[3]){
  float targetX = pointB[0];
  float targetY = pointB[1];
  float myPosX = pointA[0];
  float myPosY = pointA[1];       
  float distance = sqrt(pow((targetX - myPosX),2) + pow((targetY - myPosY),2));
  Serial.print("Distance: "); 
  Serial.println(distance); // the hypotenuse
  return distance;
}
float requiredAngleToReachTarget(joint *link){  
  float d = distanceToTarget(link->coords, target);
  float x = d - jointsList[link->pin + 1]->length;
  float theta = radiansToDegrees(asin(x/d));
  Serial.println(theta);
  getCoords(link, theta);
  return theta;
}

void getCoords(joint *link, float theta){
  float distance = link->length * cos(degreesToRadians(theta)); // WORKS
  link -> coords[0] = (distance); // should this be absolute?
  Serial.print("Coords: (");
  Serial.print(link -> coords[0]);
  Serial.print(", ");
  float height = link->length * (sin(degreesToRadians(theta))); //WORKS
  link -> coords[1] = (height); // should this be absolute?
  Serial.print(" y: ");
  Serial.print(link -> coords[1]);
  Serial.println(")");
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

(?) Treat the arm as if it only has two links to make it easier to figure out how to handle the math
    then handle the individual links so they are equivalent to the two-link representation */
void rotationMatrixOnX(float theta){
  /* rotate on X
      X   Y   Z
  X   1   0   0
  Y   0 cosθ  -sinθ
  Z   0 sinθ  cosθ
  */
 *rotMatrix[0] = 1, 0, 0;
 *rotMatrix[1] = 0, cos(degreesToRadians(theta)), -sin(degreesToRadians(theta));
 *rotMatrix[2] = 0, sin(degreesToRadians(theta)), cos(degreesToRadians(theta));

// Issues
  printMatrix();
  for (int i=0; i<2; i++){
    rotMatrix[i];
    //Serial.println(i);
    for (int j = 0; j<2;j++){
      delay(100);
      Serial.print(rotMatrix[i][j]);   //~~$$$$$$$$~~ 
      Serial.print(",");
    }
    Serial.println(" ");
  }
}

void rotationMatrixOnY(float theta){
  /* rotate on Y
      X    Y     Z
  X  cosθ  0   sinθ
  Y  0     1   0
  Z  -sinθ 0   cosθ
  */
  *rotMatrix[0] = cos(degreesToRadians(theta)), 0, sin(degreesToRadians(theta));     //~~$$$$$$$$~~ 
  *rotMatrix[1] = 0, 1, 0;     //~~$$$$$$$$~~ 
  *rotMatrix[2] = 0, -sin(degreesToRadians(theta)), cos(degreesToRadians(theta));     //~~$$$$$$$$~~ 
}

void rotationMatrixOnZ(float theta){
  /* rotate on Z
      X     Y     Z
  X  cosθ  -sinθ  sinθ    
  Y  sinθ  cosθ   0     
  Z  0     0      1     
  */
  *rotMatrix[0] = cos(degreesToRadians(theta)), -sin(degreesToRadians(theta)), sin(degreesToRadians(theta));
  *rotMatrix[1] = sin(degreesToRadians(theta)), cos(degreesToRadians(theta)), 0;
  *rotMatrix[2] = 0, 0, 1;
}

// Issues
void multiplyMatrices(int axisOfRotation, float vector[3], float theta){ // the "vector" is the link coords
    int option = axisOfRotation;
    if (option == 1){
      Serial.println("rotate on X");
      rotationMatrixOnX(theta);  //~~$$$$$$$$~~ 
    }
    if (option == 2){
      Serial.println("rotate on Y");
      rotationMatrixOnY(theta);
    }
    if (option == 3){
      Serial.println("rotate on Z");
      rotationMatrixOnZ(theta);
    }
    float coord;
    float product;
    float total=0;
    float newVector[3];
    for(int row=0; row<=2; row++){
      Serial.println("in loop");
      coord = vector[row];
      for(int j=0; j<=2; j++){
        product = rotMatrix[row][j] * coord;     //~~$$$$$$$$~~ 
        total = total + product;
      }
      newVector[row] = total;
    }
}

void printMatrix(){        //~~$$$$$$$$~~ 
  Serial.println("printing");
  int i = 0;
  int j = 0;
  for(i; i<3;i++){
    delay(100);
    Serial.println(i);
    for(j; j<3; j++){
    Serial.print("pin: ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(rotMatrix[i][j]);
    Serial.print(", ");
    Serial.print(rotMatrix[i][j]);
    Serial.print(", ");
    Serial.println(rotMatrix[i][j]);
    }
  }

}
//*******************************************************************************
//*******************************************************************************

void loop(){
  float pin, angle;
  joint *link;
  angle = 0;
  float theta = 0;
  int axis;
  while(Serial.available() > 0)
  {
    
    pin = Serial.parseInt();
    
    axis = Serial.parseInt();
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
    if (pin == 3){
      link = &link3;
    }
    if (pin == 4){
      link = &link4;
    }
    if(r == '\n'){}
    //theta = requiredAngleToReachTarget(link);
    multiplyMatrices(axis, link->coords, angle); // 11/11 Try passing the link object instead, then taking the coords in the function
    printMatrix();
  }
}