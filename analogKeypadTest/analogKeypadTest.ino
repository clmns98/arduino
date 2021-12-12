#define PIN 27
long long sum;
short iterator = 10;

void setup() {
  // Setup serial monitor
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("---------------------------");
  Serial.println("--Starting Keypad Sketch!--");
  Serial.println("---------------------------");
  Serial.println(" ");
  Serial.println("      (pin side) ");
  Serial.println(" /------/------/------/");
  Serial.println(" / 4097 / 4097 / 4097 /");
  Serial.println(" /------/------/------/");
  Serial.println(" / 4097 / 4097 / 4097 /");
  Serial.println(" /------/------/------/");
  Serial.println(" / 3677 / 3319 / 3064 /");
  Serial.println(" /------/------/------/");
  Serial.println(" / 2878 / 2721 / 2582 /");
  Serial.println(" /------/------/------/");
  Serial.println("      (bottom side) ");
  Serial.println(" ");
  
  pinMode(PIN, INPUT);
}

void loop() {
  for(int i=0;i<(iterator+1);i++)
  {
    sum += analogRead(PIN);
  }
  Serial.println(sum /= iterator);
  delay(200);

}
