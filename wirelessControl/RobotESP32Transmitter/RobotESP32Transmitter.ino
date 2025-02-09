
/* Using a homemade controller connected to an ESP32 to send commands to another ESP32 connected to the robot arm. 
This program defines how a controller controls the robot arm.
each button is either pressed or not.
The joystick's position has either moved far enough outside of a defined neutral range. Within range = inactive. Outside of range = active.
  This program continually watches for any state-changes, on a change of state, it toggles the corresponding motor on/off.
  Connected to the receiver through WiFi and the receiver is located through its Mac address.

There are 4 motors mapped to the joystick with each corresponding to a direction on the joystick. (up, down, left, right)
the value and direction of the joystick defines which motor will move and by how much.
the 6 buttons are mapped to 2 motors. The base and the claw.

Details on how each device is connected and to what
Joystick Controller (Transmitter) connections between ESP32 and the joystick.
 ESP32    | Joystick
 pin IO32   x out
 pin IO35   y out
 pin IO34   switch (joystick push)
 GND -> GND
 Vin -> 3.3V
*/
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

// the joystick fluctuates even when youre not moving it. So this defines a range of values to ignore. Joystick values within the range (1600, 2200) will not trigger any movement.
int neutral_upper_bound = 2200;
int neutral_lower_bound = 1600;

// using a delay to control the speed of the motors
int delayTime = 100;
// this tells the receiver how to increment a motors position by. Lower number = more precision
int precision = 5;
int precision2 = 10;

// pin connections on the ESP32
  // joystick
const int xOut = 32;
const int yOut = 35;
const int sel = 34;
  // 6 buttons
const int btn_LB = 26;
const int btn_RB = 14;
const int btn1 = 27;
const int btn2 = 4;
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
  Serial.begin(921600); // lower baud rates may not work. just fyi.
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_send_cb(onDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(btn3, INPUT_PULLUP);
  pinMode(btn4, INPUT_PULLUP);
  pinMode(btn_LB, INPUT_PULLUP);
  pinMode(btn_RB, INPUT_PULLUP);

  pinMode(sel, INPUT_PULLUP);
  Serial.println("Transmitter Ready");
}

void loop(){
    //defines the initial states of the controller. These are used as a point of reference to be able to tell when a button or joystick is pressed or moved.
  int btn1_state = digitalRead(btn1);
  int btn2_state = digitalRead(btn2);
  int btn3_state = digitalRead(btn3);
  int btn4_state = digitalRead(btn4);
  int btn_LB_state = digitalRead(btn_LB);
  int btn_RB_state = digitalRead(btn_RB);
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
  if(digitalRead(btn1) != btn1_state){
    Serial.println("wrist up");
    btn1_state = digitalRead(btn1);
    while(btn1_state == digitalRead(btn1)){
      btn1_state = digitalRead(btn1);
    myData.pin = 2;
    myData.change = precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(delayTime);
    }
  }  

  if(digitalRead(btn2) != btn2_state){
    Serial.println("rotate the base to the left");
    btn2_state = digitalRead(btn2);                 // btn2_state now equals the active state. 
    while(btn2_state == digitalRead(btn2)){         // keep reading the button's state until it changes
      btn2_state = digitalRead(btn2);
    myData.pin = 0;                                 // defines which of the robot arm's motor this controls by the pin its connected to on the PCA9685. 
    myData.change = -precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    delay(delayTime);
    }
  }
  if(digitalRead(btn3) != btn3_state){
    Serial.println("wrist down");
    btn3_state = digitalRead(btn3);
    while(btn3_state == digitalRead(btn3)){
      btn3_state = digitalRead(btn3);
    myData.pin = 2;
    myData.change = -precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(delayTime);
    }
  }
  if(digitalRead(btn4) != btn4_state){
    Serial.println("rotate the base to the right");
    btn4_state = digitalRead(btn4);
    while(btn4_state == digitalRead(btn4)){ 
      btn4_state = digitalRead(btn4);
    myData.pin = 0;
    myData.change = precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    delay(delayTime);
    }
  }
  if(digitalRead(btn_LB) != btn_LB_state){
    Serial.println("open claw ");
    btn_LB_state = digitalRead(btn_LB);
    while(btn_LB_state == digitalRead(btn_LB)){
      btn_LB_state = digitalRead(btn_LB);
    myData.pin = 5;
    myData.change = -precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    delay(delayTime);
    }
  }
  if(digitalRead(btn_RB) != btn_RB_state){
    btn_RB_state = digitalRead(btn_RB);
    while(btn_RB_state == digitalRead(btn_RB)){
      btn_RB_state = digitalRead(btn_RB);
    myData.pin = 5;
    myData.change = precision2;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
    Serial.println("close claw");
    delay(delayTime);
    }
  }    
}