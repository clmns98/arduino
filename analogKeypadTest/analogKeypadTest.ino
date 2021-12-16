#define PIN 27
long long sum = 0;
short iterator = 300;

void keypad(int n) {
  switch (n) { // +- 30
    case 4065 ... 4125: // 4095
      Serial.println("key one");
      break;
    case 3795 ... 3855: // 3825
      Serial.println("key two");
      break;
    case 3258 ... 3318: // 3288
      Serial.println("key three");
      break;
    case 2932 ... 2992: // 2962
      Serial.println("key four");
      break;
    case 2701 ... 2761: // 2731
      Serial.println("key five");
      break;
    case 2502 ... 2562: // 2532
      Serial.println("key six");
      break;
    case 2366 ... 2426: // 2396
      Serial.println("key seven");
      break;
    case 2194 ... 2254: // 2224
      Serial.println("key eight");
      break;
    case 2066 ... 2126: // 2096
      Serial.println("key nine");
      break;
    case 1948 ... 2008: // 1978
      Serial.println("key ten");
      break;
    case 1842 ... 1902: // 1872
      Serial.println("key eleven");
      break;
    case 1748 ... 1808: // 1778
      Serial.println("key twelve");
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
