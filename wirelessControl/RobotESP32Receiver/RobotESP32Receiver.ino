// Robot Side (Receiver) Works
// ESP32 | PCA9685
//  IO21   SDA
//  IO22   SCL
//  3v3    VCC (NOT VIN)
//  GND    GND

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h> //I2C
#include <Adafruit_PWMServoDriver.h> // PCA9685

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

#define motor1 1
#define motor2 2

float precision = 0.5;
float delayTime = 200;

// *define the same data structure sent by the transmitter*
typedef struct data_struct {
  int pin;
  int change;
  float output;
} data_struct;
data_struct myData;

struct joint{
  int pin;              
  float len;         
  float jointPosition;  
  float min;
  float max;
};

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
  pwm.setPWM(joint->pin, 0, newPosition);
  joint->jointPosition = newPosition;
  Serial.println(joint->jointPosition);
  //delay(delayTime);
}

void onDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Link : ");
  Serial.println(myData.pin); 
  if(myData.pin == 0){
    moveMotor(&base, myData.change);
  }
  if(myData.pin == 1){
    moveMotor(&link1, myData.change);
  }
  if(myData.pin == 2){
    moveMotor(&link2, myData.change);
  }
  if(myData.pin == 3){
    moveMotor(&link3, myData.change);
  }
  if(myData.pin == 4){
    moveMotor(&link4, myData.change);
  }
  if(myData.pin == 5){
    moveMotor(&endEffector, myData.change);
  }
}

void setup() {
  Serial.begin(921600);
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