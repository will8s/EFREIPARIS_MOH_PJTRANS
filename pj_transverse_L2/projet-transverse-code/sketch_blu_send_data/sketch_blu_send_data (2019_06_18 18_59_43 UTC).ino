 #include <SoftwareSerial.h>
SoftwareSerial BT(10, 11); 
// connection BT module TX a D10
// connection BT module RX a D11
// connection BT Vcc a 5V, GND a GND
// il faut ouvrir le port COM8 pour la connection blu
void setup()  
{
  // pin LDE
  pinMode(13, OUTPUT);
  pinMode(12,INPUT_PULLUP);

  BT.begin(9600);
  // dit bonjour aux autres appreils connecté par erreure a la carte
  BT.println("Hello from Arduino");
}
char a; // stockage des caracteres
void loop() 
{
  if (BT.available())
  {
    if(digitalRead(12)==HIGH){
      BT.println("111"); 
      digitalWrite(13,HIGH);
    }
    else if (digitalRead(12)==LOW){
      BT.println("000"); 
      digitalWrite(13,LOW);
    }
  }
}
