#include <AccelStepper.h>

#define MOTOR_STEP_PIN 38
#define MOTOR_DIR_PIN 37
#define MOTOR_ENABLE_PIN 25

#define SET_MAX_SPEED 1500
#define SET_ACCELERATION_SPEED 5000
#define MOVE_TO 1000
#define MOTOR_RATIO 1.5 

// on an ARDUINO you MUST use pins #2 and #3
//#define WHITE_WIRE_PIN 15
//#define GREEN_WIRE_PIN 2

AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN); // Defaults to AccelStepper::DRIVER

void setup() {
  // Setup serial monitor
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("-------------------------");
  Serial.println("-Starting Keypad Sketch!-");
  Serial.println("-------------------------");
  Serial.println(" ");
 
    // the encoder has open collector connections, so we need a pullup resistor.
    // caution : NEVER connect the WHITE or GREEN WIRE to a GPIO which is configured as OUTPUT
    //           because if the OUTPUT IS HIGH it might burn the transistor inside the encoder
    //pinMode(WHITE_WIRE_PIN, INPUT_PULLUP); // internal pullup input pin 2 
    //pinMode(GREEN_WIRE_PIN, INPUT_PULLUP); // internalเป็น pullup input pin 3
 
    // Use two interrupt function to catch rising pulses
    //attachInterrupt(digitalPinToInterrupt(WHITE_WIRE_PIN), onRotaryChange, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(GREEN_WIRE_PIN), onRotaryChange, CHANGE);

      // Change these to suit your stepper if you want
  stepper.setMaxSpeed(SET_MAX_SPEED);
  stepper.setAcceleration(SET_ACCELERATION_SPEED);
  stepper.moveTo(MOVE_TO);

}

 
// the position of the axis; changes whenever an interrupt happens due to rotation of the encoder
volatile int pos=0, posM1=0;
 
unsigned long showTime = millis(); // returns the number of milliseconds passed since the Arduino board began running the current program
 
//volatile int lastState= (digitalRead(GREEN_WIRE_PIN)==HIGH ? 1 : 0) + (digitalRead(WHITE_WIRE_PIN)==HIGH ? 2:0); // condition ? result_if_true : result_if_false


void loop() {
  // Get key value if pressed
  char customKey = customKeypad.getKey();

  if (customKey) {
    // Print key value to serial monitor
    Serial.println(customKey);
    Serial.println(" ");
  }

    if (millis() >= showTime) {
    if(posM1 != pos) {
      Serial.println(String(showTime)+"\t"+String(pos));
      stepper.moveTo(0 - pos / MOTOR_RATIO);
      posM1=pos;
    }
    showTime+=50;
  }
  
  stepper.run();
}

void onRotaryChange() {
  // state = 0 .. 3 depending on input signal level
  int state = (digitalRead(GREEN_WIRE_PIN)==HIGH ? 1 : 0) + (digitalRead(WHITE_WIRE_PIN)==HIGH ? 2 : 0);
  if      (lastState==0) pos += state==1 ? 1 : -1;
  else if (lastState==1) pos += state==3 ? 1 : -1;
  else if (lastState==2) pos += state==0 ? 1 : -1;
  else if (lastState==3) pos += state==2 ? 1 : -1;
  lastState=state;
}
