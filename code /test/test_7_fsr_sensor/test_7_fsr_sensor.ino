

/*
  ButtonEvents - An Arduino library for catching tap, double-tap and press-and-hold events for buttons.
  
      Written by Edward Wright (fasteddy@thewrightspace.net)
        Available at https://github.com/fasteddy516/ButtonEvents

      Utilizes the Bounce2 library by Thomas O. Fredericks
        Available at https://github.com/thomasfredericks/Bounce2 

  Example Sketch - Advanced Usage:
    This sketch demonstrates the use of some of the additional methods provided in this library.  As in
    the 'Basic' example, it will monitor a button connected to pin 7 and send strings to the serial monitor
    indicating when events are triggered.  
 */
#include <Mouse.h>
#include <ButtonEvents.h> // we have to include the library in order to use it

const byte buttonPin = 2; // our button will be connected to pin 2

ButtonEvents myButton; // create an instance of the ButtonEvents class to attach to our button


// this is where we run one-time setup code
void setup() {
  
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
   
  // initialize the arduino serial port and send a welcome message
  Serial.begin(9600);
  Serial.println("ButtonEvents 'Advanced' example started");
  Mouse.begin();
}


// this is the main loop, which will repeat forever
void loop() {

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
