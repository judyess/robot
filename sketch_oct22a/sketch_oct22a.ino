/* Testing references and math functions for the automatedMovements project

 "local" refers to a links own frame i.e. local coords of a links position assumes the origin starts at (0,0) rather than the position of the previous link. 

  "global" refers to values on the "universal frame" that the entire arm is defined on.

 forward calculations version. FIGURE OUT HOW TO INVERT.
 */

#include <math.h>

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)
#define PI 3.1415926535897932384626433832795

float totalPos[2];

struct linkObj{
  int pin;
  float length;
  float localCoords[2];
  float globalCoords[2];
  float tickPos;
};

linkObj link0 = {0, 5, {0, 5}, {0,5}, 368};
linkObj link1 = {0, 3.5, {0, 3.50}, {0,8.5}, 368};
linkObj link2 = {0, 6.5, {0, 6.5}, {0,15}, 368};

linkObj *linksList[3] = {&link0, &link1, &link2};

// Printing functions just for testing values
void printCoords(float x, float y){ 
    Serial.print("(");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.println(")");
}


// These functions are only useful when called inside another function that can pass it information
void updateLocal(linkObj *link, float x, float y){
  link -> localCoords[0] = x;
  link -> localCoords[1]= y;
}
void updateGlobal(int i, float x, float y){ // Updates the individual global positions of a link
  linkObj thisPos[2] = {*linksList[i] -> globalCoords};
  linksList[i] -> globalCoords[0] = x;
  linksList[i] -> globalCoords[1]= y;
}

// These get LOCAL coords. Replaced getX() and getY(). CANNOT RETURN AN ARRAY
float getCoords(linkObj *link, float angle){
  float x = link->length * (cos(degreesToRadians(angle))); 
  link -> localCoords[0] = (x); 
 float y = link->length * (sin(degreesToRadians(angle))); 
  link -> localCoords[1] = (y);
}

/*
This function just adds up all of the positions currently in memory and updates a links global position during each iteration
*/
void accumulate(){ //renamed from accumulateAll()
  int i;
  float x=0;
  float y=0;
  int size = sizeof(linksList)/sizeof(int);
  for(i=0; i<size; i++){
     x += linksList[i] -> localCoords[0];
     y += linksList[i] -> localCoords[1];
     updateGlobal(i, x, y);
  }
  totalPos[0]=x; // end effector x coord
  totalPos[1]=y;
}


// ------------------------------------------------------------------------------
void setup() {
  Serial.begin(19200);
}

void loop() {
  linkObj *var;
  float link, angle, x, y;
   while(Serial.available() > 0)
  {
    link = Serial.parseFloat();
    angle = Serial.parseInt();
    //x = Serial.parseInt();
    //y = Serial.parseInt();
    char r = Serial.read();
    if (link == 0){
      var = &link0;
    }
    if (link == 1){
      var = &link1; 
    }
    if (link == 2){
      var = &link2; 
    }
    if(r == '\n'){}
      Serial.println("BEGIN.");

      
    }
}

// --------------------REWRITING AREA--------------------------

// Trying to rewrite this function to take any link but if I pass an object, I have to find a way to reference the connecting link
void l2Parallel(joint *joint, float angle){ //Keep L2 parallel to floor. --WORKS--
  float theta1, theta2;
  joint *linkp, *linkq;
  theta1 = angle;

  /*
    When |L1 - L2| = 90, L2 will be parallel to the base. The if-statements are written out so I know if the links are to the left or right of the 90 degree position.
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
  // Link1
  int pwm1 = degreeToPWM(theta1);
  float currentPos = linkp -> motorAngle; 
  if(currentPos < pwm1){
    for(float pos = currentPos; pos <= pwm1; pos+=1){
      pwm.setPWM(0, 0, pos);
      linkp -> motorAngle = pos;
      delay(delayTime);
    }
  }
  if(currentPos > pwm1){
    for(float pos = currentPos; pos >= pwm1; pos-=1){
      pwm.setPWM(0, 0, pos);
      linkp -> motorAngle = pos;
      delay(delayTime);
    }
  }
  // Link2
  int pwm2 = degreeToPWM(theta2);
  float currentPos2 = linkq -> motorAngle; 
  if(currentPos2 < pwm2){
    for(float pos2 = currentPos2; pos2 <= pwm2; pos2 += 1){
      pwm.setPWM(1, 0, pos2);
      linkq -> motorAngle = pos2;
      delay(delayTime);
    }
  }
  if(currentPos2 > pwm2){
    for(float pos2 = currentPos2; pos2 >= pwm2; pos2 -= 1){
      pwm.setPWM(1, 0, pos2);
      linkq -> motorAngle = pos2;
      delay(delayTime);
    }
  }
}


