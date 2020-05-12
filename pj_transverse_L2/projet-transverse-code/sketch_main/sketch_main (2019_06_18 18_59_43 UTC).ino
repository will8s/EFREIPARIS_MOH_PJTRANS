#include <FSR.h>
#include <SoftwareSerial.h>
#include <CommunicationUtils.h>
#include <FreeIMU.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#define mouse_left A0
#define mouse_right A1

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
float vx, vy;

FSR fsr_left(mouse_left);
FSR fsr_right(mouse_right);
SoftwareSerial BT(10, 11);
// connection BT module TX a D10
// connection BT module RX a D11
// connection BT Vcc a 5V, GND a GND
// il faut ouvrir le port COM8 pour la connection blu

void setup()
{
  pinMode(13, OUTPUT);
  pinMode(mouse_left, INPUT);
  pinMode(mouse_right, INPUT);

  Wire.begin();
  Serial.begin(9600);
  BT.begin(9600);

  mpu.initialize();
}

void loop()
{
  // Force Sensitive Resistor 0.5'' - Test Code
  // Read FSR resistance value. try also fsr.getResistance()
  // For more information see Sparkfun website - www.sparkfun.com/products/9375
  // Note, the default Vcc and external resistor values for FSR calculations are 5V ang 3300Okm, if you are not
  // using these default valuse in your circuit go to FSR.cpp and change default values in FSR constructor
  if (BT.available()) {
    float fsrForce_left = fsr_left.getForce();
    float fsrForce_right = fsr_right.getForce();

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    vx = (((float)(gx) + 300.0) / 150.0) - 4.0; // "+300" parce que l'axe x du gyro ne gere pas à plus de -350
    vy = -((float)(gz) - 100.0) / 150.0; // pareil pour "-100" ici

    analogWrite(13, fsrForce_left);

    BT.print("/");
    BT.print(fsrForce_left, 5);
    BT.print("/");
    BT.print(fsrForce_right, 5);
    BT.print("/");
    BT.print(vx, 5);
    BT.print("/");
    BT.print(vy, 5);
    delay(15);
  }
}
