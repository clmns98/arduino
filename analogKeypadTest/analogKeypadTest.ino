#define PIN 27
long long sum = 0;
short iterator = 300;

void keypad(int n) {
  switch (n) {
    case 4045 ... 4145: // 4095
      Serial.println("first key");
      break;
    case 3775 ... 3875: // 3825
      Serial.println("second key");
      break;
    case 3238 ... 3338: // 3288
      Serial.println("third key");
      break;

    default:
      Serial.println("no button pressed");
  }
}

void keypadSetup()
{  
  // analogReadResolution(9);
  pinMode(PIN, INPUT);
  Serial.println(" ");
  Serial.println("---------------------------");
  Serial.println("--Starting Keypad Sketch!--");
  Serial.println("---------------------------");
  Serial.println(" ");
  Serial.println("      (pin side) ");
  Serial.println(" /------/------/------/");
  Serial.println(" / 4095 / 3825 / 3288 /");
  Serial.println(" /------/------/------/");
  Serial.println(" / 2962 / 2731 / 2532 /");
  Serial.println(" /------/------/------/");
  Serial.println(" / 2396 / 2224 / 2096 /");
  Serial.println(" /------/------/------/");
  Serial.println(" / 1978 / 1872 / 1778 /");
  Serial.println(" /------/------/------/");
  Serial.println("      (bottom side) ");
  Serial.println(" ");
}

void setup() {
  // Setup serial monitor
  Serial.begin(115200);
  keypadSetup();
}

void loop() {
  for(int i=0;i<iterator;i++)
  {
    sum += analogRead(PIN);
  }
  sum /= iterator;
  Serial.println(sum);
  keypad(sum);
  sum = 0;
  delay(500);

}
