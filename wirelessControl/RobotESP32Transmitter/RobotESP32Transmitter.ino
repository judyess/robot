
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

TaskHandle_t Task1;
SemaphoreHandle_t semaphore;
QueueHandle_t queue;

int semaphoreWaitTime = 60;
int task0queue = 2;
int task1queue = 2;
int count = 2;

// the joystick fluctuates even when youre not moving it. So this defines a range of values to ignore. Joystick values within the range (1600, 2200) will not trigger any movement.
int neutral_upper_bound = 2200;
int neutral_lower_bound = 1600;
int neutral_upper_bound2 = 3000;
int neutral_lower_bound2 = 1600;

// using a delay to control the speed of the motors
int delayTime = 50;
// this tells the receiver how to increment a motors position by. Lower number = more 1int 1= 5;

int j1x = 601;
int j1y = 602;
int jbtn1 = 603;
int j2x = 701;
int j2y = 702;
int j2btn = 703;


int btn1 = 310;
int btn2 = 320;
int btn3 = 330;
int btn4 = 340;
int btn5 = 350;
int btn6 = 360;

// pin connections on the ESP32
  // WORKS
const int xOut = 34;
const int yOut = 35;
const int sel = 13; 
  // WORKS
const int x2Out = 33; 
const int y2Out = 32;
const int sel2 = 25; 

  /* (6) buttons digital pins
const int btn5_LB = 12; //originally was 26. Unsure if this pin will work
const int btn5_RB = 14;
const int btn0 = 2;
const int btn1 = 4;
const int btn2 = 15;
const int btn3 = 9;     //originally was 27. Unsure if this pin will work
*/
// MAC Address of the ESP32 that will receive this data.
uint8_t broadcastAddress[] = {0xD8, 0x13, 0x2A, 0x7E, 0xF5, 0x28}; //MAC addresses with 0x in front of each part

// Define a data structure that accomodates the data being sent to the receiver. 
//   Receiver should have the exact same data structure defined so that it has the tools to receive the data and manipulate it as needed.
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
  pinMode(btn2, INPUT_PULLUP);
  pinMode(btn3, INPUT_PULLUP);
  pinMode(btn5_LB, INPUT_PULLUP);
  pinMode(btn5_RB, INPUT_PULLUP);
*/
  pinMode(sel, INPUT_PULLUP);
  pinMode(sel2, INPUT_PULLUP);

  neutral_upper_bound = analogRead(xOut) + 500;     // The new joystick2 has different values from joystick1, so the old neutral range does not work for joystick2. Instead I'll read the joystick positions on startup and +_ 500 to set the neutral zone.
  neutral_upper_bound2 = analogRead(x2Out) + 500; // joystick starts at around 2400-2500 and joystick 1 starts around 1800-1900
  neutral_lower_bound = analogRead(xOut) - 500;
  neutral_lower_bound2 = analogRead(x2Out) - 500;

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
    /*defines the initial states of the controller. These are used as a point of reference to be able to tell when a button or joystick is pressed.
  int btn0_state = digitalRead(btn0);
  int btn1_state = digitalRead(btn1);
  int btn2_state = digitalRead(btn2);
  int btn3_state = digitalRead(btn3);
  int btn5_LB_state = digitalRead(btn5_LB);
  int btn5_RB_state = digitalRead(btn5_RB);
  */

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
    break;
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
    //works
  while(digitalRead(sel2) == LOW){ // directiond from !=Sel2_state. "== HIGH", "!=0" and "==1" do not work and triggers the send function constantly
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
  if(digitalRead(btn1) != btn1_state){
    Serial.println("rotate the base to the left");
    btn1_state = digitalRead(btn1);                 // btn1_state now equals the active state. 
    while(btn1_state == digitalRead(btn1)){         // keep reading the button's state until it directions
      btn1_state = digitalRead(btn1);
    controllerData.controlID = 0;                                 // defines which of the robot arm's motor this controls by the pin its connected to on the PCA9685. 
    controllerData.direction = -1;
    esp_now_send(broadcastAddress, (uint8_t *) &controllerData, sizeof(controllerData));
    delay(delayTime);
    }
  }
  if(digitalRead(btn2) != btn2_state){
    Serial.println("elbow down");
    btn2_state = digitalRead(btn2);
    while(btn2_state == digitalRead(btn2)){
      btn2_state = digitalRead(btn2);
    controllerData.controlID = 2;
    controllerData.direction = -1;
    esp_now_send(broadcastAddress, (uint8_t *) &controllerData, sizeof(controllerData)); 
    delay(delayTime);
    }
  }
  if(digitalRead(btn3) != btn3_state){
    Serial.println("rotate the base to the right");
    btn3_state = digitalRead(btn3);
    while(btn3_state == digitalRead(btn3)){ 
      btn3_state = digitalRead(btn3);
    controllerData.controlID = 0;
    controllerData.direction = 1;
    esp_now_send(broadcastAddress, (uint8_t *) &controllerData, sizeof(controllerData)); 
    delay(delayTime);
    }
  }
  if(digitalRead(btn5_LB) != btn5_LB_state){
    Serial.println("open claw ");
    btn5_LB_state = digitalRead(btn5_LB);
    while(btn5_LB_state == digitalRead(btn5_LB)){
      btn5_LB_state = digitalRead(btn5_LB);
    controllerData.controlID = 5;
    controllerData.direction = -1;
    esp_now_send(broadcastAddress, (uint8_t *) &controllerData, sizeof(controllerData));
    delay(delayTime);
    }
  }
  if(digitalRead(btn5_RB) != btn5_RB_state){
    Serial.println("close claw");
    btn5_RB_state = digitalRead(btn5_RB);
    while(btn5_RB_state == digitalRead(btn5_RB)){
      btn5_RB_state = digitalRead(btn5_RB);
    controllerData.controlID = 5;
    controllerData.direction = 1;
    esp_now_send(broadcastAddress, (uint8_t *) &controllerData, sizeof(controllerData)); 
    delay(delayTime);
    }
  }
  */
  
}