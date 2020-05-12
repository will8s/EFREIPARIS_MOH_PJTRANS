/*
GANT OFFICIEL VERSION SANS BLUETOOTH
*/

#include <CommunicationUtils.h>
#include <FreeIMU.h>
#include <FSR.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Mouse.h>
#include <Keyboard.h>

#define mouse_left A0
#define mouse_right A1
#define mouse_end A2

FSR fsr_left(mouse_left);
FSR fsr_right(mouse_right);
FSR fsr_end(mouse_end);

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
float vx, vy;

int connection = 1;

void setup() {
  pinMode(mouse_left, INPUT);
  pinMode(mouse_right, INPUT);
  pinMode(mouse_end, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);
  Serial.begin(9600);
  Wire.begin();
  Mouse.begin();
  Keyboard.begin();
  mpu.initialize();
}


void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  vx = (((float)(gx)+300.0)/150.0)-4.0;  // "+300" parce que l'axe x du gyro ne gere pas à plus de -350
  vy = -((float)(gz)-100.0)/150.0; // pareil pour "-100" ici
  
  float fsrForce_left = fsr_left.getForce();
  float fsrForce_right = fsr_right.getForce();
  float fsrForce_end = fsr_end.getForce();
  
  if(fsrForce_end>10){
    connection = 0;
    digitalWrite(13,LOW);
    delay(1000);
    
  }
  
  while(connection==0){
    fsrForce_end = fsr_end.getForce();
    if(fsrForce_end>10){
      digitalWrite(13,HIGH);
      connection = 1;
      delay(500);
    }
  }
  if(fsrForce_left>10 && connection == 1){
    Mouse.click(MOUSE_LEFT);
    delay(500);
  }
  else if(fsrForce_right>10 && connection == 1){
    Mouse.click(MOUSE_RIGHT);
    delay(500);
  }

  Serial.print("gx = ");
  Serial.print(gx);
  Serial.print(" | gz = ");
  Serial.print(gz);
  
  Serial.print(" | X = ");
  Serial.print(vx);
  Serial.print(" | Y = ");
  Serial.println(vy);
  
  Mouse.move(vy, -vx);
  
  delay(20);
}
