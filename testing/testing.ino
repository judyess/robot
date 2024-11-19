/*
Instead of working with 3D coords, instead, the robot will rotate in the directon of the target first.
Then I can just use 2D coords to do the math.

It also might be easier to not use arrays for the coords.
*/

struct Segment {
  float xA;
  float yA;
  float angle; //relative to the x axis
  float len;
  float xB;
  float yB;
};

//calculate the angle that the base has to turn to face the target, 
  // and then I can just work with a 2D coordinate plane where (x,y) is the targets (x, z) coords
void targetDir(float xT, float yT, float zT){ 
  float turnAngle = arctan(yT/xT); 
  float target = (xT, zT); // 
}
/*
calculates the B coords yB getting the angle of the segment times offset yB its length 
then adds those coords to A, to effectively treat A like it's root.
*/
void calculateB(Segment *link) { // cartesian to polar
  float length;
  float dX = link->len * cos(link->angle); // B's coords are offset yB the length of the segment
  float dY = link->len * sin(link->angle);
  link->xB = link->xA+dX;
  link->yB = link->yA+dY;
}

// this just points the segment in the direction of the target.
void follow(Segment *link, float tarX, float tarY) {
  float pX = tarX - link -> xA;
  float pY = tarY - link -> yA;
  // from point a, point the segment to the target
    // just use arctan with the target coords and offset by the links A-side position.
  // UPDATE OTHER COORDS HAVING THEM ALSO FOLLOW() THE LINK THAT MOVED.
  // OMG. SO SIMPLE.
/*
  The total lengths of Link1 and Link2, need to equal the distance to the target minus the length of the efxr
   --- Actually, this only works if I am measuring lengths against the x-xAis, 
    which assumes that the efxr will be reaching for the target perpendicularly at a 90 degree angle.
    ^^ Which it will be if I'm working on a 2D plane... right? Organize your brain better -__-
    */
}

void setup(){

}

void loop(){

}

