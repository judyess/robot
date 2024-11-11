/* Written for a 6-DOF manipulator. Currently only focused x,y coords. 

  Note: Servos are connected to a PCA 9685 in case I want to expand the number of servos.
        And because I have my servos connected to PCA 9685, I have to refer to its position in terms of PWM instead of Degrees.
        Which is why I'm using the Adafruit library instead of Arduino's Servo library.

  PHYSICAL INFO: (motors physically actually have a 200 degree range)
    6-8.4v
    Motor 0 (base) 25Kg DSServo, model ?
      180 degrees
      4"
    Motor 1 (link) HiWonder LDX-218
      180 degrees
      5"
    Motor 2 (link) HiWonder LDX-218
      180 degrees
      3.5"
    Motor 3 (wrist-y) HiWonder LFD-06
      180 degrees
      0.5"
    Motor 4 (wrist-x) HiWonder LFD-06
      180 degrees
      1.5"
    Motor 5 (end effector) Generic/Unknown
      90 degrees
      4.5"


  REMINDERS/TODO:
    SERVO L1 and SERVO L2 are physically positioned opposite of eachother. 
      (This will affect the if-statements in both setPosition() functions if I change them.)
      L1 0    leans towards the "butt"   L1 180  leans away from the "butt"
      L2 180  leans towards the "butt"   L2 0    leans away from the "butt"
    
    SERVO L3 needs to be calibrated. ~10 degrees 0ff

 INFORMATION/FORMULAS:
    "Terminal Point" P(x,y) - The point you end up at when you go counter-clockwise some amount, starting from point (1, 0)
    theta = arctan(y/x);      Get angle from (x,y) coordinates

    Link 1 == Link 2, when Joint 2 = 90 degrees
    Link 2 is parallel to the base when |Joint1 - Joint2| = 90
    Link 2 is perpendicular to the base when Joint2 == Joint1  (this might change if I physically flip the motors, idk)
    Since, 
      Link 1 is perpendicular to the Z-axis whenever Joint1 = 0 or Joint1 = 180
    then for Link 2 with a z-axis equal to Link 1,  
      Link 2 is always perpendicular to Link 1 whenever Joint2 = 0 or Joint2 = 180

    0   degrees  =  122 ticks (min)
    90  degrees  =  368 ticks
    180 degrees  =  614 ticks (max)

    Quadrant 1 = 0,90 
      Sin, Cos, Tan = positive
    Quadrant 2 = 90, 180
      Sin = positive
      Cos, Tan = negative

    Frame Rules:
      Revolute Joints always revolve around the Z-axis
      X-axis must be perpendicular to the previous Z-axis
      Y-axis, use RIGHT hand rule. 
        Thumb -> x+, Index -> y+, Middle -> z+
        Make an L with thumb and index, point middle straight out, 90 degrees from index. Palms up.
        All 3 fingers point in the positive directions along their axis.
  */

  // x coords are between [-5, 5]

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

int motorSpeed = 2; // used in position-setting functions to slow the motor down
int minTicks = 122;
int maxTicks = 614;

// starting tick positions
float tickPos0 = 368;
float tickPos1 = 368;
float tickPos2 = 368;
float tickPos3 = 368; 
float tickPos4 = 368; // 45 degrees

struct joint{
  int pin;              // physical pin on the PCA 9685.
  float coords[3];      // position on its own localized frame. coords[3] = {x, y, z}
  float length;         // physical length of a link. 
  float jointPosition;  // position of motor angle in ticks
};

const float ybase=0;
const float xbase=3.5;
// joint objects
joint base = {0, {xbase, ybase, 0}, 3.5, tickPos0}; // coords are fixed. Need to put this in the code.
joint link1 = {1, {0, link1.length, 0}, 5, tickPos1};
joint link2 = {2, {0, link2.length, 0}, 3.5, tickPos2};
joint link3 = {3, {0, link3.length, 0}, 0.5, tickPos3};
joint link4 = {4, {0, link4.length, 0}, 2, tickPos4};
joint efx = {5, {0, efx.length, 0}, 4.5, tickPos4};

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
/* WORKS
Parameters: an angle in degrees
  converts an angle(degrees) to a pulse(seconds), then to a pulse(ticks)
Returns: int ticks
*/
int convertToTicks(float degrees){
  float pulse = map(degrees, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); 
  int ticks = int(float(pulse) / 1000000 * FREQUENCY * 4096); 
  return ticks;
}
/* WORKS
Parameters: angle in ticks
  converts ticks to degrees
Returns: float degrees
*/
float convertToDegrees(float ticks){ 
  float degrees = map(ticks, minTicks, maxTicks, 0, 180);
  return degrees;
}
/* WORKS
Parameters: a joint object, and a DEGREE value
  converts degrees to ticks, sets position, and updates joint -> jointPosition 
Calls: convertToTicks()
*/
void setDegreePosition(joint *joint, float newAngle){
  float newPosition = convertToTicks(newAngle);
  float previousPosition = joint -> jointPosition; 
  Serial.print("Setting position: ");
  Serial.println(joint -> pin);
  if(previousPosition < newPosition){
    for(float updatedPosition = previousPosition; updatedPosition < newPosition; updatedPosition++){
      pwm.setPWM(joint -> pin, 0, updatedPosition);
      joint -> jointPosition = updatedPosition;
      delay(motorSpeed);
    }
  }
  if(previousPosition > newPosition){
    for(float updatedPosition = previousPosition; updatedPosition > newPosition; updatedPosition--){
      pwm.setPWM(joint -> pin, 0, updatedPosition);
      joint -> jointPosition = updatedPosition;
      delay(motorSpeed);
    }
  }
}
/*
Parameters: joint object, ticks
  Sets position and updates joint -> jointPosition 
*/
void setTickPosition(joint *joint, float ticks){
  float previousPosition = joint -> jointPosition; 
  if(previousPosition < ticks){
    for(float updatedPosition = previousPosition; updatedPosition < ticks; updatedPosition++){
      pwm.setPWM(joint -> pin, 0, updatedPosition);
      joint -> jointPosition = updatedPosition;
      delay(motorSpeed);
    }
  }
  if(previousPosition > ticks){
    for(float updatedPosition = previousPosition; updatedPosition > ticks; updatedPosition--){
      pwm.setPWM(joint -> pin, 0, updatedPosition);
      joint -> jointPosition = updatedPosition;
      delay(motorSpeed);
    }
  }
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


void checkPulse(joint *link, float angle) { // a test function
  int pulse_ticks, degreePulse;
  pulse_ticks = convertToTicks(angle);
  float current = link -> jointPosition; 
  if(current < pulse_ticks){
    for(float pos = current; pos <= pulse_ticks; pos+=1){
      setTickPosition(link, pos);
    }
  }
  if(current > pulse_ticks){
    for(float pos = current; pos >= pulse_ticks; pos-=1){
      setTickPosition(link, pos);
    }
  }
  Serial.print("Motor ");
  Serial.print(link -> pin);
  Serial.print(": ");
  Serial.print(angle);
  Serial.print(", ");
  Serial.println(pulse_ticks);
  getCoords(link, angle);
  Serial.println("-----");
}

// called by L2parallel() 
float calculateBeta(float theta){
  float beta;
  theta = convertToDegrees(theta);
  if (theta < 90){
    beta = 180 - (90 - theta);
  }
  if (theta > 90){
    beta = theta - 90;
  }
  if(theta == 90){
    beta = 90;
  }
  int pwm2 = convertToTicks(beta);
  return pwm2;
}
/* WORKS. is hardcoded to pins 0 and 1.
Parameters: an angle in degrees
  Takes user input to set Link 1's position. 
Calls: calculateBeta() to keep Link 2 parallel to the floor.
Note: Link 2 is parallel to the floor when joint 2 is 90 degrees from the Z-axis -or- when |joint 1 - joint 2| = 90
      (!) Use this same concept when I need to move the end effector along a line
*/
void l2Parallel(float angle){
  float theta1, theta2;
  theta1 = angle;
  Serial.println("---------------");
  joint *linkp, *linkq;
  linkp = &link1;
  linkq = &link2;
  int pwm1 = convertToTicks(theta1);
  float currentPos = linkp -> jointPosition; 
  if(currentPos < pwm1){
    for(float pos = currentPos; pos <= pwm1; pos+=1){
      float betaTicks = calculateBeta(pos);
      setTickPosition(linkp, pos);
      setTickPosition(linkq, betaTicks);
    }
  }
  if(currentPos > pwm1){
    for(float pos = currentPos; pos >= pwm1; pos-=1){
      float betaTicks = calculateBeta(pos);
      setTickPosition(linkp, pos);
      setTickPosition(linkq, betaTicks);
    }
  }
}

void loop() {
  float pin, angle;
  //int i;
  joint *link;
  angle = 0;
  while(Serial.available() > 0)
  {
    pin = Serial.parseInt();
    angle = Serial.parseFloat();
    //i = Serial.parseInt();
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
    
    checkPulse(link,angle);
    //l2Parallel(angle);
  }
}


/* UNUSED

// Link 2 = Link 1; linearly equivalent. 
void l2equalL1(float angle){ 
  float theta2;
  theta2 = 90;
}
// Joint 2 = Joint 1, always perpendicular to floor
void l2Perpendicular(float angle){ 
  float theta1, theta2;
  theta1 = convertToTicks(angle);
  theta2 = theta1;
  pwm.setPWM(0, 0, theta1);
  pwm.setPWM(0, 0, theta2);
}

*/
