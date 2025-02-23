/* Receiver that controls a 6DOF robot arm with all revolute joints. Connected to the transmitter through WiFi and the receivers Mac address.
This code moves a motor by calculating where a motor should be, and sets its position over and over again.
Position in degrees is referred to in terms of "ticks". 

I am using 6 ~25kg motors.
 Supplying 7.8 V to the PCA 9685 at the terminal block with a variable DC power supply.
 If robot doesn't move to its starting position on power up and you know they are working, 
  it is possible that the motors are not getting enough power.


PCA 9685 and the Receiving ESP32's pin connections:
GND . GND
DE not used.
SCL . IO 22 or 21 idr, my esp32 pin labels are under the board which is now soldered in place. But these are the IO pins for SCL and SDA. 
SDA . IO 21 or 22 idr
VCC . 3.3V 
V+ not used. Don't use.
*/

#include <esp_now.h>
#include <WiFi.h> // this wireless controller uses WiFi
#include <Wire.h> //for I2C communication
#include <Adafruit_PWMServoDriver.h> // for the PCA9685

// The frequency is usually given in the data sheet if one is available.
#define MIN_PULSE_WIDTH       500 
#define MAX_PULSE_WIDTH       2500 
#define FREQUENCY             60
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
//static QueueHandle_t queue;
//static TaskHandle_t taskB;
//int queueLength = 2; 

// same as the transmitter
int j1x = 601;
int j1y = 602;
int j1btn = 603;
int j2x = 701;
int j2y = 702;
int j2btn = 703;
int btn1 = 310;
int btn2 = 320;
int btn3 = 330;
int btn4 = 340;
int btn5 = 350;
int btn6 = 360;

int count =2;
float posA = 400;
float posB = 400;
float posC = 400;
float posD = 400;
float posE = 400;
float posF = 400;
float len;         
float jointPosition;  

// *define the same data structure sent by the transmitter*
typedef struct data_struct {
  int controlID;
  int direction;
  float output;
}; data_struct myData;

// defines a joint object. (a joint is a link and its associated motor)
struct joint{
  int pca9685pin;              
  float len;         
  float jointPosition;  
  float min;
  float max;
  float stepsize;
  float direction;
};

joint base = {0, 4, posA, 100, 700, 5, 1};    // rotate
joint link1 = {1, 5, posB, 150, 700, 5, 1};   // bend reverse of link2
joint link2 = {2, 3.5, posC, 150, 700, 5, 1}; // bend reverse of link1
joint link3 = {3, 1, posD, 150, 700, 5, 1};   // bend
joint link4 = {4, 2, posE, 150, 700, 5, 1};   // rotate
joint endEffector = {5, 2, posF, 350, 665, 5, 1};

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
  pwm.setPWM(joint->pca9685pin, 0, newPosition);
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
    delay(10);
    moveMotor(&base);
  }
  if(myData.controlID == j1x){
    link1.direction = myData.direction;
    delay(10);
    moveMotor(&link1);
  }
  if(myData.controlID == j1y){
    link2.direction = myData.direction;
    delay(10);
    moveMotor(&link2);
  }
  if(myData.controlID == j2x){
    link3.direction = myData.direction;
    delay(10);
    moveMotor(&link3);
  }
  if(myData.controlID == j2y){
    link4.direction = myData.direction;
    delay(10);
    moveMotor(&link4);
  }
  if(myData.controlID == btn1){
    endEffector.direction = myData.direction;
    delay(10);
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
  pwm.setPWM(0,0,posA);
  pwm.setPWM(1,0,posB);
  pwm.setPWM(2,0,posC);
  pwm.setPWM(3,0,posD);
  pwm.setPWM(4,0,posE);
  pwm.setPWM(5,0,posF);

/*
  xTaskCreatePinnedToCore(
    queue, 
    "Task", 
    5000, 
    NULL, 
    0, // if priority is too high (at max 10) then the main loop2 won't run
    &TaskB, 
    0
  ); 
  */

  Serial.println("Receiver Ready");
}
 
void loop() {
}