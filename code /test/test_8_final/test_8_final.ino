//#include <MPU6050_9Axis_MotionApps41.h>
//#include <MPU6050.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
//#include <MPU6050_6Axis_MotionApps20.h>

// il est peut etre necessaire d'appuyer sur le bouton reset pour activiter la souris avant d'ouvrir le moniteur serial

#include <Mouse.h>
#include "Wire.h"
#include "I2Cdev.h"

#include <ButtonEvents.h> // we have to include the library in order to use it

MPU6050 mpu;
const byte buttonPin = 16; // our button will be connected to pin 2
ButtonEvents myButton; // create an instance of the ButtonEvents class to attach to our button

#define OUTPUT_READABLE_YAWPITCHROLL

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
float yaw, pitch, roll = 0.00;
VectorInt16 accele;

const int sensitivity = 30;
float yo, xo, y, x, y_prec, x_prec = 0.00;  // Stores current analog output of each axis
int16_t ax, ay, az, gx, gy, gz;

int count = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() 
{
  Wire.begin();                                // join I2C bus (I2Cdev library doesn't do this automatically)
  Serial.begin(115200);                       // initialize serial communication
  while (!Serial);                            // wait for Leonardo enumeration, others continue immediately
  
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

    // configure the button pin as a digital input with internal pull-up resistor enabled
  pinMode(buttonPin, INPUT_PULLUP);  

  // attach our ButtonEvents instance to the button pin
  myButton.attach(buttonPin);

  // If your button is connected such that pressing it generates a high signal on the pin, you need to
  // specify that it is "active high"
  myButton.activeHigh();

  // By default, the raw signal on the input pin has a 35ms debounce applied to it.  You can change the
  // debounce time if necessary.
  myButton.debounceTime(40); 
  
  myButton.doubleTapTime(200); // set double-tap detection window to 250ms
  
  // The hold duration can be increased to require longer holds before an event is triggered, or reduced to
  // have hold events trigger more quickly.
  myButton.holdTime(1500); // require button to be held for 2000ms before triggering a hold event
  Mouse.begin();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    if (!dmpReady) return;                                                    // if programming failed, don't try to do anything
    //Serial.println(F("dmp ready!"));
    mpuInterrupt = true;
    fifoCount = mpu.getFIFOCount();                                           // get current FIFO count
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)                           // check for overflow (this should never happen unless our code is too inefficient)
    {
        mpu.resetFIFO();                                                      // reset so we can continue cleanly
        Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & 0x01)                                             // otherwise, check for DMP data ready interrupt (this should happen frequently)
    {    
        Serial.println("FIFO no overflow!\t");
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

            mpu.dmpGetAccel(&accele, fifoBuffer);                             // acces aux valeurs de l'accele et du gyro
            Serial.print(accele.x);
            Serial.print("\t");
            Serial.print(accele.y);
            Serial.print("\t");
            Serial.println(accele.z);
          
        #endif
        y = yaw - yo;
        x = roll - xo;
        yo = yaw;
        xo = roll;   

        on_off_inclinaison(yaw,x,y);// active la souris selon une certaine inclinaison de 45 à -45 deg
         
        //mouse_move(x,y);
        
        //fonction permettant à l'utilisateur d'activer ou de desactiver 
        //la fonctionnalité de clique droit au bout de plus de 3sec grace à un bouton
        //auto_click(x,y);

        bouton();
    }     
}

void mouse_move(float x, float y){
  if (y != 0){Mouse.move(0, y * sensitivity, 0);}                               // move mouse on y axis
  if (x != 0){Mouse.move(x * sensitivity, 0, 0);}                               // move mouse on x axis
}

void auto_click(float x, float y){
  if ( (x_prec-1)<=x && x<=x_prec+1 && (y_prec-1)<=y && y<=y_prec+1) { // checking the pointer doesn't move too much from its actual position: (in this case a 2 pixel square)
    count++;                                                     
    if(count == 500){ //environ 4sec d'attente pour activer le clic
      if (!Mouse.isPressed(MOUSE_LEFT)) {
        Mouse.press(MOUSE_LEFT);
        count = 0;
      }
    }
    else if (Mouse.isPressed(MOUSE_LEFT)) {
        Mouse.release(MOUSE_LEFT);
    }
  }
  else {
    x_prec = x; // updating values to check the position of the pointer and allow the click
    y_prec = y;
    count = 0;
  }
}

void on_off_inclinaison(float yaw,float x,float y){
  if (yaw<45.0 && yaw>-45.0){
    mouse_move(x,y);
    //auto_click(x,y);
  }else{
    
  }
}

void bouton(){
  if (myButton.update() == true) {
    Mouse.release(MOUSE_LEFT);

    switch(myButton.event()) {
      // things to do if the button was tapped (single tap)
      case (tap) : 
        Serial.println("TAP event detected");  
        Mouse.release(MOUSE_LEFT);
        Mouse.click(MOUSE_LEFT);
        break;
        
      // things to do if the button was double-tapped
      case (doubleTap) : 
        Serial.println("DOUBLE-TAP event detected");
        Mouse.release(MOUSE_LEFT);
        Mouse.click(MOUSE_RIGHT);
        break;
      
      // things to do if the button was held
      case (hold) : 
        Serial.println("HOLD event detected");
        Serial.println(buttonPin);
        Mouse.press(MOUSE_LEFT);
        break;
    }
  }
}
