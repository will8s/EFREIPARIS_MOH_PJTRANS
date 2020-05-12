#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11); 
// connection BT module TX a D10
// connection BT module RX a D11
// connection BT Vcc a 5V, GND a GND
void setup()  
{
  // pin LDE
  pinMode(13, OUTPUT);

  BT.begin(9600);
  // dit bonjour aux autres appreils connecté par erreure a la carte
  BT.println("Hello from Arduino");
}
char a; // stockage des caracteres
void loop() 
{
  if (BT.available())
  {
    a=(BT.read());
    if (a=='1')
    {
      digitalWrite(13, HIGH);
      BT.println("LED on");
    }
    if (a=='2')
    {
      digitalWrite(13, LOW);
      BT.println("LED off");
    }
    if (a=='?')
    {
      BT.println("Send '1' to turn LED on");
      BT.println("Send '2' to turn LED on");
    }   
  }
}
