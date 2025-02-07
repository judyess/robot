// Joystick Controller Side (Transmitter) Works
//  ESP32   | Joystick
// pin IO32   x out
// pin IO35   y out
// pin IO34   switch (joystick push)
// ESP32 analog output ranges from 0 to 4095
/* Rcvr pinouts:
  Base = 0

*/
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

int delayTime = 100;
int precision = 5;
int precision2 = 10;

const int xOut = 32;
const int yOut = 35;
const int sel = 34;

//top row
const int lBlackPin = 26;
const int upPin = 27;
const int rBlackPin = 14;
// top row
const int leftPin = 4;
const int downPin = 2;
const int rightPin = 15;

// MAC Address of the ESP32 that will receive this data.
uint8_t broadcastAddress[] = {0xD8, 0x13, 0x2A, 0x7E, 0xF5, 0x28}; //MAC addresses with 0x in front of each part
// {0xD8, 0x13, 0x2A, 0x7E, 0xF5, 0x28}; HILETGO ESP32 (slim profile one)
// {0xA0, 0xDD, 0x6C, 0x0E, 0xFB, 0x54}; AITRIP ESP32 "1" (fried)
// {0xA0, 0xDD, 0x6C, 0x0F, 0x87, 0xF4}; AITRIP ESP32 "2"
// a0:dd:6c:0c:85:98 ESP32 

// Define a data structure
typedef struct data_struct {
  int pin;
  int change;
} data_struct;

data_struct myData;

esp_now_peer_info_t peerInfo;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(921600);
  WiFi.mode(WIFI_STA);
  
  esp_now_init();
  esp_now_register_send_cb(onDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  pinMode(leftPin, INPUT_PULLUP);
  pinMode(downPin, INPUT_PULLUP);
  pinMode(rightPin, INPUT_PULLUP);
  pinMode(lBlackPin, INPUT_PULLUP);
  pinMode(upPin, INPUT_PULLUP);
  pinMode(rBlackPin, INPUT_PULLUP);

  pinMode(sel, INPUT_PULLUP);
  Serial.println("Transmitter Ready");
}

void loop(){
  int leftState = digitalRead(leftPin);
  int downState = digitalRead(downPin);
  int rightState = digitalRead(rightPin);
  int lBlackState = digitalRead(lBlackPin);
  int upState = digitalRead(upPin);
  int rBlackState = digitalRead(rBlackPin);


// JOYSTICK CONTROLS
  while(analogRead(xOut) > 2200){
    myData.pin = 1;
    myData.change = precision;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(delayTime);
  }
  while(analogRead(xOut) < 1600){
    myData.pin = 1;
    myData.change = -precision;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(delayTime);
  }
    while(analogRead(yOut) > 2200){
    myData.pin = 3;
    myData.change = precision;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(delayTime);
  }
  while(analogRead(yOut) < 1600){
    myData.pin = 3;
    myData.change = -precision;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(delayTime);
  }
  //BUTTON CONTROLS
  if(digitalRead(leftPin) != leftState){
    leftState = digitalRead(leftPin);
    while(leftState == digitalRead(leftPin)){
      leftState = digitalRead(leftPin);
    myData.pin = 0;
    myData.change = -precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    Serial.println("left");
    delay(20);
    }
  }
  if(digitalRead(downPin) != downState){
    downState = digitalRead(downPin);
    while(downState == digitalRead(downPin)){
      downState = digitalRead(downPin);
      myData.pin = 2;
      myData.change = -precision2;
      esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
      Serial.println("down");
      delay(delayTime);
    }
  }
  if(digitalRead(rightPin) != rightState){
    rightState = digitalRead(rightPin);
    while(rightState == digitalRead(rightPin)){
      rightState = digitalRead(rightPin);
    myData.pin = 0;
    myData.change = precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    Serial.println("right");
    delay(delayTime);
    }
  }
  if(digitalRead(lBlackPin) != lBlackState){
    lBlackState = digitalRead(lBlackPin);
    while(lBlackState == digitalRead(lBlackPin)){
      lBlackState = digitalRead(lBlackPin);
    myData.pin = 5;
    myData.change = -precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    Serial.println("left Black");
    delay(100);
    }
  }
    if(digitalRead(upPin) != upState){
    upState = digitalRead(upPin);
    while(upState == digitalRead(upPin)){
      upState = digitalRead(upPin);
    myData.pin = 2;
    myData.change = precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    Serial.println("up");
    delay(delayTime);
    }
  }
    if(digitalRead(rBlackPin) != rBlackState){
    rBlackState = digitalRead(rBlackPin);
    while(rBlackState == digitalRead(rBlackPin)){
      rBlackState = digitalRead(rBlackPin);
    myData.pin = 5;
    myData.change = precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    Serial.println("right Black");
    delay(delayTime);
    }
  }
}