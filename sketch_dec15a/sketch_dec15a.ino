#define PIN 27
long long sum = 0;
short iterator = 60;

void setup() {
  // Setup serial monitor
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("hello");
  Serial.println(" ");

}

void loop() {
  for(int i=0;i<iterator;i++)
  {
    sum += 20;
    Serial.println(i);
    Serial.println(sum);
    Serial.println("---------------------------");
    Serial.println(" ");
  }
  sum /= iterator;
  Serial.println(sum);
  delay(50000);

}
