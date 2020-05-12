#include <FSR.h>
#include <SoftwareSerial.h>

#define FSR_PIN_1  A0
FSR fsr(FSR_PIN_1);

SoftwareSerial BT(10, 11); 
// connection BT module TX a D10
// connection BT module RX a D11
// connection BT Vcc a 5V, GND a GND
// il faut ouvrir le port COM8 pour la connection blu

void setup()  
{
  // pin LDE
  pinMode(13, OUTPUT);
  BT.begin(9600);
  
  // dit bonjour aux autres appreils connecté par erreure a la carte
  BT.println("Hello from Arduino");
}

void loop() 
{
  // Force Sensitive Resistor 0.5'' - Test Code
  // Read FSR resistance value. try also fsr.getResistance()
  // For more information see Sparkfun website - www.sparkfun.com/products/9375
  // Note, the default Vcc and external resistor values for FSR calculations are 5V ang 3300Okm, if you are not 
  //       using these default valuse in your circuit go to FSR.cpp and change default values in FSR constructor
  

  if (BT.available())
  {
    float fsrForce = fsr.getForce();
    BT.print(F("Force: ")); BT.print(fsrForce); BT.println(F(" [g]"));
    analogWrite(13,fsrForce*2);
    /*
    if(digitalRead(12)==HIGH){
      BT.println("111"); 
      digitalWrite(13,HIGH);
    }
    else if (digitalRead(12)==LOW){
      BT.println("000"); 
      digitalWrite(13,LOW);
    }*/
  }
}
