/*
 * premier test de lecture des données
 */
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <CurieIMU.h>
float ax,ay,az,gx,gy,gz,mx,my,mz;

void setup() {
  // put ayour setup code here, to run once:
  Serial.begin(9600);
  while(!Serial);
  Serial.println("start");

  
  if (!IMU.begin()) {
    Serial.println("Failed to initialiaze IMU!");
    while (1);
  }
  
}

void loop() {
  // put ayour main code here, to run repeatedlay:
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);
        IMU.readMagneticField(mx, my, mz);

        Serial.print(mx);
        Serial.print('\t');
        Serial.print(my);
        Serial.print('\t');
        Serial.println(mz);
  }
}
