// Robot Side (Receiver) Works
// ESP32 | PCA9685
//  IO21   SDA
//  IO22   SCL
//  3v3    VCC (NOT VIN)
//  GND    GND

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH       500 
#define MAX_PULSE_WIDTH       2500 
#define FREQUENCY             60
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

float posA = 400;
float posB = 400;

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
};

joint link1 = {1, 5, posA};
joint link2 = {2, 3.5, posB};

void moveMotor(joint *joint, float change){
  float currentPosition = joint->jointPosition;
  float newPosition = currentPosition + change;
  joint->jointPosition = newPosition;
  Serial.println(joint->jointPosition);
  delay(delayTime);
}

void onDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Link : ");
  Serial.println(myData.pin); 
  if(myData.pin == 1){
    moveMotor(&link1, myData.change);
  }
  if(myData.pin == 2){
    moveMotor(&link2, myData.change);
  }
}


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pwm.setPWM(1,0,posA);
  pwm.setPWM(2,0,posB);
  Serial.println("Receiver Ready");
}
 
void loop() {
}