
void fsr_led(int, int);

void setup(void) {
  pinMode(A0,INPUT);
  pinMode(11,OUTPUT);
  pinMode(A1,INPUT);
  pinMode(10,OUTPUT);
  pinMode(A2,INPUT);
  pinMode(9,OUTPUT);
  Serial.begin(9600);
}
 
void loop(void) {
  fsr_led(11,0);
  fsr_led(10,1);
  fsr_led(9,2);
}

void fsr_led(int led,int fsr){//donnez en parametre le pin de la led et fsr 
  int fsr_valeur = analogRead(fsr);
  Serial.println(fsr);
  Serial.print(": fsr ,force: ");
  Serial.println(4.65*fsr_valeur);

  int luminosite_led = map(4.65*fsr_valeur, 0, 1023, 0, 255);

  analogWrite(led,luminosite_led);

}
