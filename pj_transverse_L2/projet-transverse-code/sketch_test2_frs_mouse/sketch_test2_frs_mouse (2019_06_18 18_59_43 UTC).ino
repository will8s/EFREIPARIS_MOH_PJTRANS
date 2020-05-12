#include "Mouse.h"

#define mouse_left A0
#define mouse_right A2

void setup() {
  // put your setup code here, to run once:
  pinMode(mouse_left,INPUT);
  pinMode(mouse_right,INPUT);

  pinMode(11,OUTPUT);
  pinMode(10,OUTPUT);

  Mouse.begin();
  Serial.begin(9600);
}


void loop() {
  // put your main code here, to run repeatedly:
  int click_left = analogRead(mouse_left);
  int click_right = analogRead(mouse_right);
  
  Serial.print("bouton gauche : ");
  Serial.println(click_left);
  delay(50);
  Serial.print("bouton droit : ");
  Serial.println(click_right);
  
  if(click_left>0){
    Mouse.click(MOUSE_LEFT);
    digitalWrite(10,LOW);
    allume_led(11);
  }
  else if(click_right>0){
    Mouse.click(MOUSE_RIGHT);
    digitalWrite(11,LOW);
    allume_led(10);
  }
  else{
    digitalWrite(11,LOW);
    digitalWrite(10,LOW);
  }
}

void allume_led(int led){
    digitalWrite(led,HIGH);
    delay(250);
    digitalWrite(led,LOW);
}
