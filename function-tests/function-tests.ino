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
  float globalCoords[2]; // RENAMED
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
void printAllLocal(){
  int size = sizeof(linksList)/sizeof(int);
  int i;
  float x = 0;
  float y = 0;
  for(i=0; i<size; i++){
    x = linksList[i] -> localCoords[0];
    y = linksList[i] -> localCoords[1];
     Serial.print("Link: ");
     Serial.print(i);
     Serial.print(" LOCAL: ");
     printCoords(x, y);
  }
}
void printAllGlobal(){
  int size = sizeof(linksList)/sizeof(int);
  int i;
  float x = 0;
  float y = 0;
  for(i=0; i<size; i++){
    x = linksList[i] -> globalCoords[0];
    y = linksList[i] -> globalCoords[1];
     Serial.print("link: ");
     Serial.print(i);
     Serial.print(" GLOBAL: ");
     printCoords(x, y);
  }
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

// These get LOCAL coords. Global Coords are accumulated from local coords.
// WHENEVER EITHER OF THESE FUNCTIONS ARE CALLED, THEY ARE AUTOMATICALLY UPDATED. THEY SHOULD ALWAYS BE CALLED TOGETHER
float getX(linkObj *link, float angle){
  float x = link->length * (cos(degreesToRadians(angle))); 
  link -> localCoords[0] = (x); 
  return x;
}
float getY(linkObj *link, float angle){
  float y = link->length * (sin(degreesToRadians(angle))); 
  link -> localCoords[1] = (y);
  return y;
}

/*
 Gets the position of the end effector by accumulating the local coords. 
 Updates the individual link global coords as well.
 TO DO: Check if values are correct when this first runs. or figure out how to run this during setup()
*/
void accumulate(){ 
  int i;
  float x=0;
  float y=0;
  int size = sizeof(linksList)/sizeof(int);
  for(i=0; i<size; i++){
     x += linksList[i] -> localCoords[0];
     y += linksList[i] -> localCoords[1];
     updateGlobal(i, x, y);
  }
  totalPos[0]=x;
  totalPos[1]=y;
  Serial.print("End Effector: ");
  printCoords(totalPos[0], totalPos[1]);
}

/* Calculates the local coords of a link from a given angle. 
  THIS IS REDUNDANT */
void degreesToCoords(linkObj *link, float angle){ 
  Serial.println("DegreesToCoords");
  Serial.print("length: ");
  Serial.print(link -> length);
  Serial.print("  angle: ");
  Serial.println(angle);
  float x = getX(link, angle);
  float y = getY(link, angle);
  updateLocal(link, x, y);
}

void calculateLine(){
  
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
    x = Serial.parseInt();
    y = Serial.parseInt();
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
      Serial.println("---------");
      Serial.print("Option: ");
      Serial.println(link);
      //UNCOMMENT FOR FORWARD CALCULATIONS
      degreesToCoords(var, angle);
      printAllLocal();
      accumulate();
      printAllGlobal();
      //coordsToDegEFX();
      //test(link, angle);
    }
}


// FUNCTIONS THAT HAVE GONE FISHING, BBL
/*
// ------------------------------------------INVERSE--------------------------------- in progress

void coordsToDeg(float y, float x){                   
                                                      // Correct. maybe use this to point the link to the target coords? Then calculate if the target coords are within reach?
  float theta = atan2(x, y) * 180/PI;
  Serial.print(theta);
  Serial.print(" : Input ");
  printCoords(x,y);
}

// -----------------------------------------

void coordsToDegEFX(){                                // correct
  float x = totalPos[0];
  float y = totalPos[1];
  float theta = atan2(y, x) * 180/PI;
  Serial.print("Slope: ");
  Serial.print(theta);                                // THIS RETURNS THE SLOPE OF THE LINE BASE->TARGET where base target is TotalPos's (x,y)
  Serial.print(" : Target Position  ");
  printCoords(x,y);
}

// -----------------------------------------

void degToCoords(float angle){                        // Correct. have to multiply it by the length of a link to get the position. 
  float length = 1;
  linkObj *l2Pos;
  l2Pos = &link2;
  float x = cos(degreesToRadians(angle)) * l2Pos -> length; 
  float y = sin(degreesToRadians(angle)) * l2Pos -> length; 
  Serial.print(angle);
  Serial.print(" : Output ");
  printCoords(x,y);
}

// -----------------------------------------

void test(float length, float angle){
  Serial.print(angle);
  float x = cos(degreesToRadians(angle)) * length;      // correct
  float y = sin(degreesToRadians(angle)) * length; 
  Serial.print(" : Output ");
  printCoords(x,y);
}



// -----------------------------------------


*/
