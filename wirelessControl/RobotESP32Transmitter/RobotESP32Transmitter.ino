// Joystick Controller Side (Transmitter) Works
//  ESP32   | Joystick
// pin IO32   x out
// pin IO35   y out
// pin IO34   switch (joystick push)
// ESP32 analog output ranges from 0 to 4095
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

const int xOut = 32;
const int yOut = 35;
const int sel = 34;

// MAC Address of the ESP32 that will receive this data.
uint8_t broadcastAddress[] = {0xA0, 0xDD, 0x6C, 0x0E, 0xFB, 0x54}; //MAC addresses with 0x in front of each part
// {0xD8, 0x13, 0x2A, 0x7E, 0xF5, 0x28}; HILETGO ESP32 (slim profile one)
// {0xA0, 0xDD, 0x6C, 0x0E, 0xFB, 0x54}; AITRIP ESP32 "1"
// {0xA0, 0xDD, 0x6C, 0x0F, 0x87, 0xF4}; AITRIP ESP32 "2"

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
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  esp_now_init();
  esp_now_register_send_cb(onDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop(){
/*
  if(Serial.available() > 0){ // try a while loop instead to fix the double send thing
    int input = Serial.parseInt();
    myData.a = input;
    // Why is this sending data twice? First is the int that I enter, then "0" gets sent after. ??
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
  }
  */
  while(analogRead(xOut) > 2200){
    myData.pin = 1;
    myData.change = 1;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(200);
  }
  while(analogRead(xOut) < 1600){
    myData.pin = 1;
    myData.change = -1;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(200);
  }
    while(analogRead(yOut) > 2200){
    myData.pin = 2;
    myData.change = 1;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(200);
  }
  while(analogRead(yOut) < 1600){
    myData.pin = 2;
    myData.change = -1;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(200);
  }
}