// a prongram to test the random() function

// syntax -> random(min, max) min is included, max is excluded
// data type long

void setup() {
    Serial.begin(115200);
    Serial.println("Random Test ESP32");

}
int n = 0;

void loop() {
  n = random(3);
  Serial.println(n);
  delay(200);

}
