
//Pin number where the sensor is connected. (Analog 0)
#define EMG_PIN 0

/*-------------------------------- void setup ------------------------------------------------*/

void setup(){
  
  //BAUDRATE set to 115200, remember it to set monitor serial properly. 
  //Used this Baud Rate and "NL&CR" option to visualize the values correctly.
  Serial.begin(115200);
 
}

/*--------------------------------  void loop  ------------------------------------------------*/

void loop(){

  //The "Value" variable reads the value from the analog pin to which the sensor is connected.
  int value = analogRead(EMG_PIN);


  //You can use serial monitor to set THRESHOLD properly, comparing the values shown when you open and close your hand.
  Serial.println(abs(value-400));
  delay(20);
}
