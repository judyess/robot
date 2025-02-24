
/* Using a homemade controller connected to an ESP32 to send commands to another ESP32 connected to the robot arm. 
This program defines how a controller controls the robot arm.
each button is either pressed or not.
The joystick's position has either moved far enough outside of a defined neutral range. Within range = inactive. Outside of range = active.
  This program continually watches for any state-directions, on a direction of state, it toggles the corresponding motor on/off.
  Connected to the receiver through WiFi and the receiver is located through its Mac address.

 TO-DO (2/20/25):
 Move all code specific to the robot arm to the receiver code, so this controller can be reused for other projects.
 Add "tasks" (to access the 2nd core?)

 Board: "ESP32 Dev Module"

*/
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

// the joystick fluctuates even when youre not moving it. So this defines a range of values to ignore. Joystick values within the range (1600, 2200) will not trigger any movement.
int neutral_upper_bound = 2400;
int neutral_lower_bound = 1400;
int neutral_upper_bound2 = 3200;
int neutral_lower_bound2 = 1500;
int delayTime = 50;

// controller IDs for buttons. (This info needs to be available to the receiver so it knows what I'm sending)
int j1x = 601;
int j1y = 602;
int jbtn1 = 603;
int j2x = 701;
int j2y = 702;
int j2btn = 703;
int btn1 = 310;
int btn2 = 320;

// pin connections on the ESP32
const int xOut = 34;
const int yOut = 35;
const int sel = 13; 
const int x2Out = 33; 
const int y2Out = 32;
const int sel2 = 25; 

// MAC Address of the ESP32 that will receive this data.
uint8_t broadcastAddress[] = {0xD8, 0x13, 0x2A, 0x7E, 0xF5, 0x28}; //MAC addresses with 0x in front of each part

// Define a data structure that accomodates the data being sent to the receiver. Receiver should have the exact same data structure defined so it can receive it.
typedef struct data_struct {
  int controlID; // this info tells the receiver to move the motor connected to this pin
  int direction;
}; data_struct controllerData;

esp_now_peer_info_t peerInfo;
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200); // lower baud rates may not work. just fyi.
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_send_cb(onDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
/*
  pinMode(btn0, INPUT_PULLUP);
  pinMode(btn1, INPUT_PULLUP);
*/
  pinMode(sel, INPUT_PULLUP);
  pinMode(sel2, INPUT_PULLUP);
/*  xTaskCreatePinnedToCore(
    taskB, 
    "Task", 
    5000, 
    NULL, 
    0, // if priority is too high (at max 10) then the main loop2 won't run
    &Task1, 
    0
  ); //core 1 is the defaault core used by the regular loop()
  */
  Serial.println("Transmitter Ready");
}

void sendData(){
    esp_now_send(broadcastAddress, (uint8_t *) &controllerData, sizeof(controllerData)); 
    //count++;
    //Serial.print("!!!!! COUNT: ");
    //Serial.println(count);
    //delay(delayTime);
}

/*
void taskB(void * parameter) {
  //"setup()" area
   //neutral_upper_bound2 = analogRead(x2Out) + 500; // joystick starts at around 2400-2500 and joystick 1 starts around 1800-1900
   //neutral_lower_bound2 = analogRead(x2Out) - 500;
  
  while(count>0){
    for(;;){
      delay(20);
      while(analogRead(x2Out) > neutral_upper_bound2){
        Serial.print(x2Out);
        Serial.print("  : ");
        Serial.println(analogRead(x2Out));
        controllerData.controlID = 2;
        controllerData.direction = 1
        count--;
        sendData();
        delay(delayTime);
      }
    while(analogRead(x2Out) < neutral_lower_bound2){
      Serial.print(x2Out);
      Serial.print("  : ");
      Serial.println(analogRead(x2Out));
      controllerData.controlID = 2;
      controllerData.direction = -1
      count--;
      sendData();
      delay(delayTime);
      }
    while(analogRead(y2Out) > neutral_upper_bound2){
      Serial.print(y2Out);
      Serial.print("  : ");
      Serial.println(analogRead(y2Out));
      controllerData.controlID = 4;
      controllerData.direction = 1
      count--;
      sendData();
      delay(delayTime);
      }
    while(analogRead(y2Out) < neutral_lower_bound2){
      Serial.print(y2Out);
      Serial.print("  : ");
      Serial.println(analogRead(y2Out));
      controllerData.controlID = 4;
      controllerData.direction = -1
      count--;
      sendData();
      delay(delayTime);
      }
    while(digitalRead(sel2) == LOW){
      Serial.print("Joystick 2 pin: ");
      Serial.println((sel2));
      controllerData.controlID = 5;
      controllerData.direction = -1;
      count--;
      sendData();
      delay(delayTime);
      }      
    }
  }
}*/

void loop(){
  // joystick1
  while(analogRead(xOut) > neutral_upper_bound){
    Serial.print(xOut);
    Serial.print("  : ");
    Serial.println(analogRead(xOut));
    controllerData.controlID = j1x;
    controllerData.direction = 1;
    sendData();
    delay(delayTime);
    }
  while(analogRead(xOut) < neutral_lower_bound){
    Serial.print(xOut);
    Serial.print("  : ");
    Serial.println(analogRead(xOut));
    controllerData.controlID = j1x;
    controllerData.direction = -1;
    sendData();
    delay(delayTime);
    }
  while(analogRead(yOut) > neutral_upper_bound){
    Serial.print(yOut);
    Serial.print("  : ");
    Serial.println(analogRead(yOut));
    controllerData.controlID = j1y;
    controllerData.direction = 1;
    sendData();
    delay(delayTime);
    }
  while(analogRead(yOut) < neutral_lower_bound){
    Serial.print(yOut);
    Serial.print("  : ");
    Serial.println(analogRead(yOut));
    controllerData.controlID = j1y;
    controllerData.direction = -1;
    sendData();
    delay(delayTime);
    }
  while(digitalRead(sel) == LOW){
    Serial.print("Joystick 1 pin: ");
    Serial.println((sel));
    controllerData.controlID = jbtn1;
    controllerData.direction = -1;
    sendData();
    delay(delayTime);
    }
  while(analogRead(x2Out) > neutral_upper_bound2){
    Serial.print(x2Out);
    Serial.print("  : ");
    Serial.println(analogRead(x2Out));
    controllerData.controlID = j2x;
    controllerData.direction = 1;
    sendData();
    delay(delayTime);
  }
  while(analogRead(x2Out) < neutral_lower_bound2){
    Serial.print(x2Out);
    Serial.print("  : ");
    Serial.println(analogRead(x2Out));
    controllerData.controlID = j2x;
    controllerData.direction = -1;
    sendData();
    delay(delayTime);
  }
  while(analogRead(y2Out) > neutral_upper_bound2){
    Serial.print(y2Out);
    Serial.print("  : ");
    Serial.println(analogRead(y2Out));
    controllerData.controlID = j2y;
    controllerData.direction = 1;
    sendData();
    delay(delayTime);
  }
  while(analogRead(y2Out) < neutral_lower_bound2){
    Serial.print(y2Out);
    Serial.print("  : ");
    Serial.println(analogRead(y2Out));
    controllerData.controlID = j2y;
    controllerData.direction = -1;
    sendData();
    delay(delayTime);
  }
  while(digitalRead(sel2) == LOW){
    Serial.print("Joystick 2 pin: ");
    Serial.println((sel2));
    controllerData.controlID = j2btn;
    controllerData.direction = 1;
    sendData();
    delay(delayTime);
  } 
/*
  if(digitalRead(btn0) != btn0_state){
    Serial.println("elbow up");
    btn0_state = digitalRead(btn0);
    while(btn0_state == digitalRead(btn0)){
      btn0_state = digitalRead(btn0);
    controllerData.controlID = 2;
    controllerData.direction = 1;
    esp_now_send(broadcastAddress, (uint8_t *) &controllerData, sizeof(controllerData)); 
    delay(delayTime);
    }
  }  
  */
}