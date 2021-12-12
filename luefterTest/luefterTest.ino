#define FAN 27

void setup() {
  // put your setup code here, to run once:  
  Serial.begin(115200);
  delay(200);
  Serial.println("START");
  pinMode(FAN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(FAN , HIGH);
  delay(4000);
  digitalWrite(FAN, LOW);
  delay(4000);
}
