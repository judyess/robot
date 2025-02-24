/* Receiver that controls a 6DOF robot arm with all revolute joints. Connected to the transmitter with the receivers Mac address.
Position is in terms of "ticks". 

I am using 6 ~25kg motors.
 Supplying 11-12 V to the PCA 9685 at the terminal block with a variable DC power supply.
 If robot doesn't move to its starting position on power up and you know they are working, 
  it is possible that the motors are not getting enough power.
*/

#include <esp_now.h>
#include <WiFi.h> // this wireless controller uses WiFi
#include <Wire.h> //for I2C communication
#include <Adafruit_PWMServoDriver.h> // for the PCA9685
#define MIN_PULSE_WIDTH       500 
#define MAX_PULSE_WIDTH       2500 
#define FREQUENCY             60
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// same as on the transmitter sketch for the controller
int j1x = 601;
int j1y = 602;
int j1btn = 603;
int j2x = 701;
int j2y = 702;
int j2btn = 703;
int btn1 = 310;
int btn2 = 320;

// *define the same data structure sent by the transmitter*
typedef struct data_struct {
  int controlID;
  int direction;
  float output;
}; data_struct myData;

// defines a joint object. (a joint is a link and its associated motor)
struct joint{
  int onPCA9685pin;              
  float len;         
  float jointPosition;  
  float min;
  float max;
  float stepsize;
  float direction;
};

joint base = {0, 4, 400, 100, 700, 5, 1};    // rotate
joint link1 = {1, 5, 400, 150, 700, 5, 1};   // bend reverse of link2
joint link2 = {2, 3.5, 400, 150, 700, 5, 1}; // bend reverse of link1
joint link3 = {3, 1, 400, 150, 700, 5, 1};   // bend
joint link4 = {4, 2, 400, 150, 700, 5, 1};   // rotate
joint endEffector = {5, 2, 400, 350, 665, 5, 1}; //close-open

void moveMotor(joint *joint){
  Serial.println(joint->direction);
  float currentPosition = joint->jointPosition;
  float newPosition = currentPosition + (joint->stepsize * joint->direction);
  Serial.print("New Position: ");
  delay(10);
  if(newPosition > joint->max){
    newPosition = joint->max;
  }
  if(newPosition < joint->min){
    newPosition = joint->min;
  }
  pwm.setPWM(joint->onPCA9685pin, 0, newPosition);
  joint->jointPosition = newPosition;
  Serial.println(joint->jointPosition);
}

void onDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Received: ");
  Serial.print(myData.controlID);
  Serial.print(" | ");
  Serial.println(myData.direction);
  if(myData.controlID == j2btn || myData.controlID == j1btn){
    base.direction = myData.direction;
    moveMotor(&base);
  }
  if(myData.controlID == j1x){
    link1.direction = myData.direction;
    moveMotor(&link1);
  }
  if(myData.controlID == j1y){
    link2.direction = myData.direction;
    moveMotor(&link2);
  }
  if(myData.controlID == j2x){
    link3.direction = myData.direction;
    moveMotor(&link3);
  }
  if(myData.controlID == j2y){
    link4.direction = myData.direction;
    moveMotor(&link4);
  }
  if(myData.controlID == btn1){
    endEffector.direction = myData.direction;
    moveMotor(&endEffector);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pwm.setPWM(0,0,400);
  pwm.setPWM(1,0,400);
  pwm.setPWM(2,0,400);
  pwm.setPWM(3,0,400);
  pwm.setPWM(4,0,400);
  pwm.setPWM(5,0,400);
  Serial.println("Receiver Ready");
}
 
void loop() {
}