// Include Libraries
#include "Arduino.h"
#include "MPU6050.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "Mouse.h"

// Pin Definitions



// Global variables and defines
int16_t mpu6050Ax, mpu6050Ay, mpu6050Az;
int16_t mpu6050Gx, mpu6050Gy, mpu6050Gz;
// object initialization
MPU6050 mpu6050;


// define vars for testing menu
const int timeout = 10000;       //define timeout of 10 sec
char menuOption = 0;
long time0;

// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup() 
{
    // Setup Serial which is useful for debugging
    // Use the Serial Monitor to view printed messages
    Serial.begin(9600);
    while (!Serial) ; // wait for serial port to connect. Needed for native USB
    Serial.println("start");
    Mouse.begin();
    Wire.begin();
    mpu6050.initialize();
    menuOption = menu();
    
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop() 
{
    if(menuOption == '1') {
    // SparkFun MPU-6050 - Accelerometer and Gyro - Test Code
    mpu6050.getMotion6(&mpu6050Ax, &mpu6050Ay, &mpu6050Az, &mpu6050Gx, &mpu6050Gy, &mpu6050Gz);   //read accelerometer and gyroscope raw data in three axes
    double mpu6050Temp = ((double)mpu6050.getTemperature() + 12412.0) / 340.0;
    Serial.print("a/g-\t");
    Serial.print(mpu6050Ax); Serial.print("\t");
    Serial.print(mpu6050Ay); Serial.print("\t");
    Serial.print(mpu6050Az); Serial.print("\t");
    Serial.print(mpu6050Gx); Serial.print("\t");
    Serial.print(mpu6050Gy); Serial.print("\t");
    Serial.print(mpu6050Gz); Serial.print("\t");
    Serial.print(F("Temp- "));   
    Serial.println(mpu6050Temp);



    double pitch, roll;
    double G[3];
    
    //Roll & Pitch Equations
    roll  = (atan2(-mpu6050Ay, mpu6050Az))*180/PI;
    pitch = (atan2(mpu6050Ax, sqrt(mpu6050Ay*mpu6050Ay + mpu6050Az*mpu6050Az)))*180/PI;
    Serial.print("\n");
    Serial.println(pitch);
    Serial.print(":");
    Serial.println(roll);
    
    Serial.print("\n");
    G[0]=sin(pitch)*9.8;
    G[1]=-cos(pitch)*sin(roll)*9.8;
    G[2]=-cos(pitch)*cos(roll)*9.8;

    Serial.println(G[0]);
    Serial.println(G[1]);
    Serial.println(G[2]);

    ///float x = 0.5*
    
    ///Serial.print(vx); Serial.print("\t");
    ///Serial.print(vy); Serial.print("\t"); 
    Mouse.move(vx,vy);
    delay(25);

    }
    
    if (millis() - time0 > timeout)
    {
        menuOption = menu();
    }
    
}



// Menu function for selecting the components to be tested
// Follow serial monitor for instrcutions
char menu()
{

    Serial.println(F("\nWhich component would you like to test?"));
    Serial.println(F("(1) SparkFun MPU-6050 - Accelerometer and Gyro"));
    Serial.println(F("(menu) send anything else or press on board reset button\n"));
    while (!Serial.available());

    // Read data from serial monitor if received
    while (Serial.available()) 
    {
        char c = Serial.read();
        if (isAlphaNumeric(c)) 
        {   
            
            if(c == '1') 
          Serial.println(F("Now Testing SparkFun MPU-6050 - Accelerometer and Gyro"));
            else
            {
                Serial.println(F("illegal input!"));
                return 0;
            }
            time0 = millis();
            return c;
        }
    }
}
