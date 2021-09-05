#include <AccelStepper.h>

// on an ARDUINO you MUST use pins #2 and #3
#define WHITE_WIRE_PIN 15
#define GREEN_WIRE_PIN 2

void log()
{
  Serial.print(micros() + "    ");
}

void setup()
{
  Serial.begin(115200);

  // the encoder has open collector connections, so we need a pullup resistor.
  // caution : NEVER connect the WHITE or GREEN WIRE to a GPIO which is configured as OUTPUT
  //           because if the OUTPUT IS HIGH it might burn the transistor inside the encoder
  pinMode(WHITE_WIRE_PIN, INPUT_PULLUP); // internal pullup input pin 2 
  pinMode(GREEN_WIRE_PIN, INPUT_PULLUP); // internalเป็น pullup input pin 3

  // Use two interrupt function to catch rising pulses
  attachInterrupt(digitalPinToInterrupt(WHITE_WIRE_PIN), onRotaryChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(GREEN_WIRE_PIN), onRotaryChange, CHANGE);
  Serial.println("-startup-");

}

// the position of the axis; changes whenever an interrupt happens due to rotation of the encoder
volatile int pos=0, posM1=0;
 
unsigned long showTime = millis(); // returns the number of milliseconds passed since the Arduino began running the program
unsigned long showMicros = micros(); // returns the number of microseconds passed since the Arduino began running the program
unsigned long distance = 0;

volatile int lastState= (digitalRead(GREEN_WIRE_PIN)==HIGH ? 1 : 0) + (digitalRead(WHITE_WIRE_PIN)==HIGH ? 2:0); // condition ? result_if_true : result_if_false

void loop()
{
 // if(pos <

    if (millis() >= showTime) {
    if(posM1 != pos) {
      Serial.println(String(pos)+"\t"+String(showTime/10)+"\t"+String(micros()-showMicros)+"\t"/* steps/micros */ );
      posM1=pos;
      showMicros=micros();
      
    }
    showTime+=50;
  }

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
