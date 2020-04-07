#include <MadgwickAHRS.h>

#include <compiler_abstraction.h>
#include <mbed_memory_status.h>

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

#include <USBMouse.h>
#include <USBHID_Types.h>
#include <USBMouseKeyboard.h>
#include <PluggableUSBHID.h>
#include <USBKeyboard.h>
#include "mbed.h"
#include "USBKeyboard.h"

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
float sensitivity = 10.0;

USBMouse mouse;

void setup() {
  // start serial
  Serial.begin(9600);
  while(!Serial);
  Serial.println("start");
  // start the IMU and filter
  if (!IMU.begin()) {
    Serial.println("Failed to initialiaze IMU!");
    while (1);
  }
  filter.begin(104.0);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 100;
  microsPrevious = micros();
}

void loop() {
  //float aix, aiy, aiz, gix, giy, giz,mix,miy,miz;
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  float roll, pitch, heading;
  float vertValue, horzValue, vertZero, horzZero;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      IMU.readMagneticField(mx, my, mz);
    }

    // convert from raw data to gravity and degrees/second units
    /*ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);
    mx = convertRawMagn(mix);
    my = convertRawMagn(miy);
    mz = convertRawMagn(miz);*/
    /*Serial.print(ax);
    Serial.print(" ");
    Serial.print(ay);
    Serial.print(" ");
    Serial.println(az);
    Serial.print(" ");
    Serial.print(gx);
    Serial.print(" ");
    Serial.print(gy);
    Serial.print(" ");
    Serial.print(gz);
    Serial.print(" ");
    Serial.print(mx);
    Serial.print(" ");
    Serial.print(my);
    Serial.print(" ");
    Serial.println(mz);*/
    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    /*heading = heading/3.14*180.0;
    pitch = pitch/3.14*180.0;
    roll = pitch/3.14*180.0;*/
    
    Serial.print("Orientation: ");
    Serial.print(" heading : ");
    Serial.print(heading);
    Serial.print(" pitch : ");
    Serial.print(pitch);
    Serial.print(" roll : ");
    Serial.println(roll);

    float vx = (((float)(gx)+300.0)/150.0);  // "+300" parce que l'axe x du gyro ne gere pas à plus de -350
    float vy = -((float)(gz)-100.0)/150.0; // pareil pour "-100" ici

  
    mouse.move(vy,-vx);                                     // move mouse on y axis
    
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;

   
  }
}

/*code propre
 * #include <MPU6050.h>
#include <Mouse.h>
#include <Keyboard.h>

#define mouse_left A0
#define mouse_right A1
#define mouse_end A2


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
}*/
 */
