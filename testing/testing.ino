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

/***
The inRangePosition should be equal to the distance to the target minus the length of the efxr. (distance is measured relative to the x-axis)

The length of the arm ... 

***/


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
  float a[3];
  float len;         
  float jointPosition;  
  float b[3];
  
};

joint base = {0, {0, 0, 3.5}, 3.5, tickPos0}; // only the base's Z value will be changing
joint link1 = {1, {0, 0, link1.len}, 5, tickPos1};
joint link2 = {2, {0, 0, link2.len}, 3.5, tickPos2};
joint link3 = {3, {0, 0, link3.len}, 0.5, tickPos3};
joint link4 = {4, {0, 0, link4.len}, 2, tickPos4};
joint efx = {5, {0, 0, efx.len}, 4.5, tickPos5};
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
  printLinkList();
}

void printArray(int array[], int arraySize){
  for (int i=0; i<arraySize; i++){
    Serial.print(array[i]);
    Serial.print(",");
  } Serial.println();
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
    Serial.print(link->a[0]);
    Serial.print(", ");
    Serial.print(link->a[1]);
    Serial.print(", ");
    Serial.print(link->a[2]);
    Serial.print(" ==> ");
    Serial.print(link->b[0]);
    Serial.print(", ");
    Serial.print(link->b[1]);
    Serial.print(", ");
    Serial.println(link->b[2]);
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
  float distance = link->len * cos(degreesToRadians(theta)); // WORKS
  link -> b[0] = (distance); // should this be absolute?
  Serial.print("Link's new coords: (");
  Serial.print(link -> b[0]);
  Serial.print(", ");
  float height = link->len * (sin(degreesToRadians(theta))); //WORKS
  link -> b[1] = (height); // should this be absolute?
  Serial.print(" y: ");
  Serial.print(link -> b[1]);
  Serial.println(")");
}
/*
Calls distanceToTarget(), getCoords()
*/
float requiredAngleToReachTarget(joint *link){  // for a 2D plane
  float d = distanceToTarget(link->a, target);
  float x = d - jointsList[link->pin + 1]->len;
  float theta = radiansToDegrees(asin(x/d));
  Serial.println(theta);
  getCoords(link, theta);
  return theta;
}

//calculate the angle that a link has to turn to face the target. 
  // and then I can just work with a 2D coordinate plane where (x,y) is the targets (x, z) coords
float targetDir(float xT, float yT, float zT){ 
  float targetAngle = atan(yT/xT) * 180.0 / M_PI; 
  float target = (xT, zT); // 
  return targetAngle;
}
/*
calculates the B coords by getting the angle of the link times the offset of its len 
then adds those coords to A, to effectively treat A like it's root. (Local coords)
*/
void calculateB() { // cartesian to polar
  for(int i=0; i<6;i++){
    joint *link = jointsList[i];
    float currentAngle = convertToDegrees(link->jointPosition);
    float dX = link->len * cos(degreesToRadians(currentAngle)); // B's coords are offset by the len of the segment
    float dY = link->len * sin(degreesToRadians(currentAngle));
    link->b[0] = link->a[0]+dX;
    link->b[1] = link->a[1]+dY;
  } 
}

/* this gets the distance to the target. Use to calculate whether or not a link can reach the target.
 calculate where the efxr needs to be to reach the point T. 
1.  if the distance from the efxr's current position is exactly equal to its length, then find the angle to go to it.
2.  if the distance is greater-than it's length, then the arm needs to extend further to a point that is a distance equal to efxrs length 
3.  if the distance is less-than the arms length, then the arm needs to pull back to place the efxr in range.

  1a.find the position that the arm needs to be by adding the length of the efxr to the target, then flipping it across the target-point towards the direction of the link
  1b. 

  a midpoint formula finds the center between two known points that represent a circles endpoints:
    this formula is also equivalent to just finding the average between the coords. or divind the diameter in half
    (a, b) = ((x1 + x2)/2,  (y1 + y2)/2 )
    

    
    
  (just in case if useful):
  the standard equation for a circle is:  (x, y): (x - h)^2 +(y - k)^2 = r^2
      (h, k) = center point
      r = link length
      (x, y) = arbitrary point on the boundary of the circle

      ex. if the center is (0,0) then the formula for a circle at that root is x^2 + y^2 = r, 
        where you solve for (x, y) and r is the length of the link.
      make sure that the target point is on the 0-180 degree semi-circle side of the circle

      general form: x2 + y2 + 2gx + 2fy + c = 0
        I don't get this formula
      
      parametric form: x2 + y2 + 2hx + 2ky + C = 0 
        where x = (-h + rcosθ) and y = (-k + rsinθ)

      polar form: ((dcosθ) + h)^2 + ((dsinθ) + k)^2 = r^2
                  = r^2((cos^2)θ + (sin^2)θ) = p^2
        where:
          (dcosθ, dsinθ) = a point on the boundary of the circle,
            d = distance from center to a point on the boundary of the circle

      if a point on the boundary lies on the x-axis, then the center = (x, r)
        the point lies at (x, 0) then  y of the center is equal to the radius
      if a point on the boundary lies on the y-axis, then the center = (r, y)
      if the boundaries of a circle touch both the x and y axes, then the circle is (r, r)
 */
void follow(joint *link, float tarX, float tarY) {
  float pX = tarX - link -> a[0];
  float pY = tarY - link -> a[1];
}

void loop(){
  float pin;
  joint *link;
  float angle = 0;
  while(Serial.available() > 0)
  {
    pin = Serial.parseInt();
    angle = Serial.parseFloat();
    
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
    //requiredAngleToReachTarget(link);
    //Serial.println(pin);
    calculateB();
    printLinkList();
  }
}





