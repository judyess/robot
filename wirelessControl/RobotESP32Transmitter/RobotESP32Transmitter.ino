
/* Using a homemade controller connected to an ESP32 to send commands to another ESP32 connected to the robot arm. 
This program defines how a controller controls the robot arm.
each button is either pressed or not.
The joystick's position has either moved far enough outside of a self-defined neutral range to either be active or not.

There are 4 motors mapped to the joystick with each corresponding to a direction on the joystick. (up, down, left, right)
the value and direction of the joystick defines which motor will move and by how much.

This program works by watcching when a button state has changed, which tells the receiver to toggle the motor on or off.
So what the value is doesn't matter, it only matters if a value is equal to its previous value or not

Details on how each device is connected and to watch
Joystick Controller Side (Transmitter) connections between ESP32 and the joystick.
 ESP32    | Joystick
 pin IO32   x out
 pin IO35   y out
 pin IO34   switch (joystick push)
 ESP32 analog output ranges from 0 to 4095 ( Arduino's s 0-1028)

PCA 9685 -> Receiving ESP32 pin connections:
GND -> GND
DE not used.
SCL -> IO 22 or 21 idr, my esp32 pin labels are under the board which is now soldered in place. But these are the IO pins that also accept SCL and SDA connections. 
SDA -> IO 21 or 22 idr
VCC -> 3.3V 
V+ not used. Don't use.

Controller -> Transmitting ESP32 pin connections:
Joystick:
X out = IO32
Y out = IO35
Press Joystick = IO34
GND -> GND
Vin -> 3.3V
*/
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

// the joystick fluctuates even when youre not moving it. So this defines a range of values to ignore. Values outside of the range (1600, 2200) will send commands to the receiver to
int neutral_upper_bound = 2200;
int neutral_lower_bound = 1600

int delayTime = 100;
// this tells the receiver how much to move a motor by.
int precision = 5;
int precision2 = 10;

// pin connections on the ESP32
  // joystick
const int xOut = 32;
const int yOut = 35;
const int sel = 34;
  // 6 buttons
const int btn_LB = 26;
const int btn1 = 27;
const int btn_RB = 14;
const int bhtn2 = 4;
const int btn3 = 2;
const int btn4 = 15;

// MAC Address of the ESP32 that will receive this data.
uint8_t broadcastAddress[] = {0xD8, 0x13, 0x2A, 0x7E, 0xF5, 0x28}; //MAC addresses with 0x in front of each part

// Define a data structure that accomodates the data being sent to the receiver. 
//   Receiver should have the exact same data structure defined so that it has the tools to receive the data and manipulate it as needed.
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

  pinMode(bhtn2, INPUT_PULLUP);
  pinMode(btn3, INPUT_PULLUP);
  pinMode(btn4, INPUT_PULLUP);
  pinMode(btn_LB, INPUT_PULLUP);
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn_RB, INPUT_PULLUP);

    //defines the initial state of the buttons when not pressed. These are used as a point of reference to be able to tell when a button or joystick is pressed or moved.
    // moved from the loop to setujp 2/8/25
  int bhtn2_state = digitalRead(bhtn2);
  int btn3_state = digitalRead(btn3);
  int btn4_state = digitalRead(btn4);
  int lBlackState = digitalRead(btn_LB);
  int btn1_state = digitalRead(btn1);
  int rBlackState = digitalRead(btn_RB);

  pinMode(sel, INPUT_PULLUP);
  Serial.println("Transmitter Ready");
}

void loop(){

// JOYSTICK CONTROLS. Joysticks are analog.
  while(analogRead(xOut) > neutral_upper_bound){
    myData.pin = 1;
    myData.change = precision;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(delayTime);
  }
  while(analogRead(xOut) < neutral_lower_bound){
    myData.pin = 1;
    myData.change = -precision;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(delayTime);
  }
    while(analogRead(yOut) > neutral_upper_bound){
    myData.pin = 3;
    myData.change = precision;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(delayTime);
  }
  while(analogRead(yOut) < neutral_lower_bound){
    myData.pin = 3;
    myData.change = -precision;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(delayTime);
  }
  //BUTTON CONTROLS. Buttons are digital.
  if(digitalRead(bhtn2) != bhtn2_state){
    bhtn2_state = digitalRead(bhtn2);                 // bhtn2_state now equals the active state. 
    while(bhtn2_state == digitalRead(bhtn2)){         // keep reading the button's current state until it no longer is equal to its active state
      bhtn2_state = digitalRead(bhtn2);
    myData.pin = 0;                                   // defines which of the robot arm's motor this controls by the pin its connected to on the PCA9685. 
    myData.change = -precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    Serial.println("left");
    delay(delayTime);
    }
  }

  if(digitalRead(btn4) != btn4_state){
    btn4_state = digitalRead(btn4);
    while(btn4_state == digitalRead(btn4)){ 
      btn4_state = digitalRead(btn4);
    myData.pin = 0;
    myData.change = precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    Serial.println("right");
    delay(delayTime);
    }
  }
  if(digitalRead(btn_LB) != lBlackState){
    lBlackState = digitalRead(btn_LB);
    while(lBlackState == digitalRead(btn_LB)){
      lBlackState = digitalRead(btn_LB);
    myData.pin = 5;
    myData.change = -precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    Serial.println("left Black");
    delay(delayTime);
    }
  }
  if(digitalRead(btn_RB) != rBlackState){
    rBlackState = digitalRead(btn_RB);
    while(rBlackState == digitalRead(btn_RB)){
      rBlackState = digitalRead(btn_RB);
    myData.pin = 5;
    myData.change = precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    Serial.println("right Black");
    delay(delayTime);
    }
  }
    if(digitalRead(btn1) != btn1_state){
    btn1_state = digitalRead(btn1);
    while(btn1_state == digitalRead(btn1)){
      btn1_state = digitalRead(btn1);
    myData.pin = 2;
    myData.change = precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    Serial.println("up");
    delay(delayTime);
    }
  }
    if(digitalRead(btn3) != btn3_state){
    btn3_state = digitalRead(btn3);
    while(btn3_state == digitalRead(btn3)){
      btn3_state = digitalRead(btn3);
      myData.pin = 2;
      myData.change = -precision2;
      esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
      Serial.println("down");
      delay(delayTime);
    }
  }
    
}