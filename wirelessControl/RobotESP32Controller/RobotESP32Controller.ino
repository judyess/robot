// Serial output is correct

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


#define MIN_PULSE_WIDTH       500 
#define MAX_PULSE_WIDTH       2500 
#define FREQUENCY             60

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int xout = 32;
int yout= 35;
int sel = 34;
int btnPin1 = 14;
int btnPin2 = 27;
int btnPin3 = 26;
int btnPin4 = 15;
int btnPin5 = 2;
int btnPin6 = 4;


// tick positions
float posA = 400;
float posB = 400;

float precision = 0.5;
float delayTime = 200;

struct joint{
  int pin;              
  float len;         
  float jointPosition;  
};

joint link1 = {1, 5, posA};
joint link2 = {2, 3.5, posB};

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pinMode(14, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
}

void moveMotor(joint *joint, float change){
  float currentPosition = joint->jointPosition;
  float newPosition = currentPosition + change;
  joint->jointPosition = newPosition;
  Serial.println(joint->jointPosition);
  delay(delayTime);
}


void loop() {
  /*
  while(analogRead(xout) > 600){
    moveMotor(&link2, 1);
  }
  while(analogRead(xout) < 400){
    moveMotor(&link2, -1);
  }
    while(analogRead(yout) > 600){
    moveMotor(&link1, 1);
  }
  while(analogRead(yout) < 400){
    moveMotor(&link1, -1);
  }
  */

  int buttonState1 = digitalRead(14);
  int buttonState2 = digitalRead(27);
  int buttonState3 = digitalRead(26);
  int buttonState4 = digitalRead(15);
  int buttonState5 = digitalRead(2);
  int buttonState6 = digitalRead(4);
  
  if(digitalRead(14) != buttonState1){
    buttonState1 = digitalRead(buttonState1);
    Serial.print("14: ");
    Serial.println(buttonState1);
    delay(500);
  }
  if(digitalRead(27) != buttonState2){
    buttonState2 = digitalRead(buttonState2);
    Serial.print("27: ");
    Serial.println(buttonState2);
    delay(500);
  }
  if(digitalRead(26) != buttonState3){
    buttonState3 = digitalRead(buttonState3);
    Serial.print("26: ");
    Serial.println(buttonState3);
    delay(500);
  }
  if(digitalRead(15) != buttonState4){
    buttonState4 = digitalRead(buttonState4);=230
    Serial.print("Yellow: ");
    Serial.println(buttonState4);
    delay(500);
  }
    if(digitalRead(2) != buttonState5){
    buttonState5 = digitalRead(buttonState5);
    Serial.print("White1: ");
    Serial.println(buttonState5);
    delay(500);
  }
    if(digitalRead(4) != buttonState6){
    buttonState6 = digitalRead(buttonState6);
    Serial.print("White2: ");
    Serial.println(buttonState6);
    delay(500);
  }
}