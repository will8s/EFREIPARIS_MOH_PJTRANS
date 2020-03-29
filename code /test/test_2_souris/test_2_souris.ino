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
  filter.begin(100);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 100;
  microsPrevious = micros();
}

void loop() {
  //float aix, aiy, aiz, gix, giy, giz,mix,miy,miz;
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  float roll, pitch, heading;
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
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);


    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}

/*float convertRawAcceleration(float aRaw) {
  // since we are using 4G range
  // -4g maps to a raw value of -32768
  // +4g maps to a raw value of 32767
  
  float a = (aRaw * 4.0) / 32768.0;
  //float a = map(aRaw, -4, 4, -32768, 32768);
  return a;
}

float convertRawGyro(float gRaw) {
  // since we are using 2000 degrees/seconds range
  // -2000 maps to a raw value of -32768
  // +2000 maps to a raw value of 32767
  
  //float g = (gRaw * 2000.0) / 32768.0;
  float g = map(gRaw, -2000, 2000, -32768, 32768);
  return g;
}

float convertRawMagn(float mRaw){
  // -400 to 400
  //float m = (mRaw * 400.0) / 32768.0;
  float m = map(mRaw, -400, 400, -32768, 32768);
  return m;
}*/
/*
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h"
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, pitch, roll;

int left_button_pin = 9; // Left button
int right_button_pin = 10; // right button
int leftClickFlag = 0;
const int sensitivity = 30;
float vertZero, horzZero;
float vertValue, horzValue;  // Stores current analog output of each axis

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

USBMouse mouse;
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() 
{
  Wire.begin();                                // join I2C bus (I2Cdev library doesn't do this automatically)
  Serial.begin(115200);                       // initialize serial communication
  while (!Serial);                            // wait for Leonardo enumeration, others continue immediately
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
   if (devStatus == 0) 
   {
      mpu.setDMPEnabled(true);                // turn on the DMP, now that it's ready
      attachInterrupt(0, dmpDataReady, RISING);     // enable Arduino interrupt detection
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;                        // set our DMP Ready flag so the main loop() function knows it's okay to use it
      packetSize = mpu.dmpGetFIFOPacketSize();      // get expected DMP packet size for later comparison
  } 
  else 
  {                                          // ERROR!        1 = initial memory load failed         2 = DMP configuration updates failed        (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
  pinMode(LED_PIN, OUTPUT);                 // configure LED for output
  pinMode(left_button_pin, INPUT);
  pinMode(right_button_pin, INPUT);
  yaw = 0.0;
  pitch = 0.0;
  roll = 0.0;
  vertZero = 0;
  horzZero = 0;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    if (!dmpReady) return;                                                    // if programming failed, don't try to do anything
    mpuInterrupt = true;
    fifoCount = mpu.getFIFOCount();                                           // get current FIFO count
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)                           // check for overflow (this should never happen unless our code is too inefficient)
    {
        mpu.resetFIFO();                                                      // reset so we can continue cleanly
        Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & 0x01)                                             // otherwise, check for DMP data ready interrupt (this should happen frequently)
    {    
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();        // wait for correct available data length, should be a VERY short wait
        mpu.getFIFOBytes(fifoBuffer, packetSize);                             // read a packet from FIFO
        fifoCount -= packetSize;                                              // track FIFO count here in case there is > 1 packet available
        #ifdef OUTPUT_READABLE_YAWPITCHROLL                                               // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = ypr[1] /PI * 180;
            pitch = ypr[2] /PI * 180;
            roll = ypr[0] /PI * 180;
            Serial.print("ypr\t");
            Serial.print(yaw);
            Serial.print("\t");
            Serial.print(pitch);
            Serial.print("\t");
            Serial.println(roll);
        #endif
        blinkState = !blinkState;                                             // blink LED to indicate activity
        vertValue = yaw - vertZero;
        horzValue = roll - horzZero;
        vertZero = yaw;
        horzZero = roll;   
        if (vertValue != 0)
          mouse.move(0, vertValue * sensitivity);                                      // move mouse on y axis
        if (horzValue != 0)
          mouse.move(horzValue * sensitivity, 0);                                      // move mouse on x axis
    }
}*/
