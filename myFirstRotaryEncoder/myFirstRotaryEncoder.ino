// A sample program to read pulses from an optical encoder
 
// on an ARDUINO you MUST use pins #2 and #3
#define WHITE_WIRE_PIN 15
#define GREEN_WIRE_PIN 2
 
void setup() {
    // Arduino UNO should use 9600 baud
  Serial.begin (115200);
  Serial.println("angle tracker 1.0");
 
    // the encoder has open collector connections, so we need a pullup resistor.
    // caution : NEVER connect the WHITE or GREEN WIRE to a GPIO which is configured as OUTPUT
    //           because if the OUTPUT IS HIGH it might burn the transistor inside the encoder
    pinMode(WHITE_WIRE_PIN, INPUT_PULLUP); // internal pullup input pin 2 
    pinMode(GREEN_WIRE_PIN, INPUT_PULLUP); // internalเป็น pullup input pin 3

}
 
volatile int lastStateGrn;

volatile int lastStateWht;

void loop() {

  lastStateGrn = (digitalRead(GREEN_WIRE_PIN)==HIGH ? 1 : 0);
  lastStateWht = (digitalRead(WHITE_WIRE_PIN)==HIGH ? 1 : 0);

  Serial.println(lastStateGrn);
  Serial.println(lastStateWht);
  
  delay(1000);
  
}
 