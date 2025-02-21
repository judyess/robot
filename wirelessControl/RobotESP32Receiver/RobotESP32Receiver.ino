/* Receiver that controls a 6DOF robot arm with all revolute joints. Connected to the transmitter through WiFi and the receivers Mac address.
This code moves a motor by calculating where a motor should be, and sets its position over and over again.
Position in degrees is referred to in terms of "ticks". 

I am using 6 ~25kg motors.
 Supplying 7.8 V to the PCA 9685 at the terminal block with a variable DC power supply.
 If robot doesn't move to its starting position on power up and you know they are working, 
  it is possible that the motors are not getting enough power.


PCA 9685 and the Receiving ESP32's pin connections:
GND -> GND
DE not used.
SCL -> IO 22 or 21 idr, my esp32 pin labels are under the board which is now soldered in place. But these are the IO pins for SCL and SDA. 
SDA -> IO 21 or 22 idr
VCC -> 3.3V 
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
  int pca9685pin;
  int change;
  float output;
};

// defines a joint object. (a joint is a link and its associated motor)
struct joint{
  int pca9685pin;              
  float len;         
  float jointPosition;  
  float min;4
  float max;
};

data_struct myData;
joint base = {0, 4, posA, 100, 700};
joint link1 = {1, 5, posB, 150, 700};
joint link2 = {2, 3.5, posC, 150, 700};
joint link3 = {3, 1, posD, 150, 700};
joint link4 = {4, 2, posE, 150, 700};
joint endEffector = {5, 2, posF, 350, 665};

void moveMotor(joint *joint, float change){
  float currentPosition = joint->jointPosition;
  float newPosition = currentPosition + change;
  if(newPosition > joint -> max){
    newPosition = joint -> max;
  }
  if(newPosition < joint -> min){
    newPosition = joint -> min;
  }
  pwm.setPWM(joint->pca9685pin, 0, newPosition);
  joint->jointPosition = newPosition;
  Serial.println(joint->jointPosition);
  delay(20);
}

void onDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Link : ");
  Serial.println(myData.pca9685pin); 
  if(myData.pca9685pin == 0){
    moveMotor(&base, myData.change);
  }
  if(myData.pca9685pin == 1){
    moveMotor(&link1, myData.change);
  }
  if(myData.pca9685pin == 2){
    moveMotor(&link2, myData.change);
  }
  if(myData.pca9685pin == 3){
    moveMotor(&link3, myData.change);
  }
  if(myData.pca9685pin == 4){
    moveMotor(&link4, myData.change);
  }
  if(myData.pca9685pin == 5){
    moveMotor(&endEffector, myData.change);
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
  Serial.println("Receiver Ready");
}
 
void loop() {
}