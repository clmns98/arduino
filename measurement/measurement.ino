#include <AccelStepper.h>

// on an ARDUINO you MUST use pins #2 and #3
#define WHITE_WIRE_PIN 15
#define GREEN_WIRE_PIN 2

void setup()
{
  Serial.begin(115200);

  pinMode(WHITE_WIRE_PIN, INPUT_PULLUP); // internal pullup input pin 2
  pinMode(GREEN_WIRE_PIN, INPUT_PULLUP); // internalเป็น pullup input pin 3

  // Use two interrupt function to catch rising pulses
  attachInterrupt(digitalPinToInterrupt(WHITE_WIRE_PIN), onRotaryChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(GREEN_WIRE_PIN), onRotaryChange, CHANGE);
  Serial.println("\n");
  Serial.println("---startup---");
  Serial.println(String("position") + "\t" + String("time") + "\t" + String("deltaT") + "\t" + String("speed-omega"));
  Serial.println("-------------------------------------------");
}

// the position of the axis; changes whenever an interrupt happens due to rotation of the encoder
volatile int pos = 0, posM1 = 0;

unsigned long showTime = micros(); // returns the number of milliseconds passed since the Arduino began running the program
unsigned long showMicros = micros(); // returns the number of microseconds passed since the Arduino began running the program

  // these are the important values wich are going to be printed
  unsigned long distance = 0;
  double deltaT = 0;
  double omega = 0;
  short steps = 0;

volatile int lastState = (digitalRead(GREEN_WIRE_PIN) == HIGH ? 1 : 0) + (digitalRead(WHITE_WIRE_PIN) == HIGH ? 2 : 0); // condition ? result_if_true : result_if_false

short outputCounter = 0;
unsigned int output[100][4];

void loop()
{
  if (micros() >= showTime) {
    if (posM1 != pos) {
      if (pos >= 2400)
      {
        pos -= 2400;
      }
      if (pos < 0)
      {
        pos += 2400;
      }
      //Serial.println(String(pos) + "\t" + "\t" + String(showTime) + "\t" + String(int(deltaT)) + "\t" + String(int(omega)));
      output[outputCounter][0] = pos;
      output[outputCounter][1] = showTime;
      output[outputCounter][2] = deltaT;
      output[outputCounter][3] = omega;
      if(outputCounter == 100)
      {
        outputCounter = 0;
        for(int i=0;i<101;i++)
        {
          Serial.print(String(output[i][0]) + "\t");
          Serial.print(String(output[i][1]) + "\t");
          Serial.print(String(output[i][2]) + "\t");
          Serial.print(String(output[i][3]));
          Serial.print("\n");
        }             
      }
      outputCounter++;
      posM1 = pos;

    }
    showTime += 50;
  }
}

void onRotaryChange() {
  deltaT = micros() - showMicros;
  steps = pos - posM1;
  omega = steps / deltaT * 1000000;
  // state = 0 .. 3 depending on input signal level
  int state = (digitalRead(GREEN_WIRE_PIN) == HIGH ? 1 : 0) + (digitalRead(WHITE_WIRE_PIN) == HIGH ? 2 : 0);
  if      (lastState == 0) pos += state == 1 ? 1 : -1;
  else if (lastState == 1) pos += state == 3 ? 1 : -1;
  else if (lastState == 2) pos += state == 0 ? 1 : -1;
  else if (lastState == 3) pos += state == 2 ? 1 : -1;
  lastState = state;
  showMicros = micros();
}
