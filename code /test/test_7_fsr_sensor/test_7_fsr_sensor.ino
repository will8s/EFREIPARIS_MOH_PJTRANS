
int fsrAnalogPin = A0; // FSR is connected to analog 0
int LEDpin = 11;      // connect Red LED to pin 11 (PWM pin)
int fsrReading;      // the analog reading from the FSR resistor divider

unsigned long current_time, temps, tempszero = 0 ;

void setup(void) {
  Serial.begin(9600);   // We'll send debugging information via the Serial monitor
  pinMode(LEDpin, OUTPUT);
}
 
void loop(void) {
  fsrReading = analogRead(fsrAnalogPin);
  Serial.print("Analog reading = ");
  Serial.println(fsrReading);
  if(fsrReading==0){
    current_time = millis();
  }
  if(fsrReading>0 && millis()-current_time>1000){// si on appuie plus 1,5 secondes sur le boutton
    Serial.println("long tap : ");
  }
  else if(fsrReading>0 && millis()-current_time>50){
    Serial.println("1 tap : ");
    
    temps = millis();
    while(fsrReading>0 && millis()-temps<150){///
      fsrReading = analogRead(fsrAnalogPin);
      Serial.print("Analog reading ========== ");
      Serial.println(fsrReading);
      
      if(fsrReading==0){
        tempszero = millis();
        while(fsrReading==0 && millis()-tempszero<200){
          fsrReading = analogRead(fsrAnalogPin);
          Serial.print("Analog reading ================== ");
          Serial.println(fsrReading);
          if(fsrReading>0 && millis()-temps<350 && millis()-tempszero<200){
            Serial.println("Analog reading ================== tap 222222");
            digitalWrite(LEDpin, LOW);    // turn the LED off by making the voltage LOW
            //il faut retourner à 0 apres : delay(500) ?
          }
        }
      }
    }
  }

  
  delay(20);
}
