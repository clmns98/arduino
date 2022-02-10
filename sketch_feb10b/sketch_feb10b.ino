#define KEYPAD_PIN 25
int keypadValue = 0;
int n = 0;
int n1 = 0;
int n2 = 0;
bool alreadyDone = false;

void keypadSetup()
{
  pinMode(KEYPAD_PIN, INPUT);
  Serial.println(" ");
  Serial.println("---------------------------");
  Serial.println("--Starting Keypad Sketch!--");
  Serial.println("---------------------------");
  Serial.println(" ");
  Serial.println("              (pin side) ");
  Serial.println(" /----------------------------------/");
  Serial.println(" / ______   / LOW         / HIGH    /");
  Serial.println(" /-----------------------------------");
  Serial.println(" / forward  / backwards   / restart /");
  Serial.println(" /----------------------------------/");
  Serial.println(" / ______   / ______      / ______   ");
  Serial.println(" /----------------------------------/");
  Serial.println(" / ______   / ______      / ______  /");
  Serial.println(" /----------------------------------/");
  Serial.println("              (bottom side) ");
  Serial.println(" ");
}

void readKeypad() {
  int temp = 0;
  for(int i=0;i<8;i++)
  {
    temp += analogRead(KEYPAD_PIN);
    if(n2 == 2)
    {
      n = micros();
    }
    if(n2 == 3)
    {
      n1 = micros() - n;  
    }
    n2 += 1;
  }
  Serial.println(n1);
  n, n1, n2 = 0;
  temp /= 8;
  keypadValue = temp;
}

void setup() {
  Serial.begin(115200);
  keypadSetup();

}

void loop() {
  readKeypad();
  delay(1000);

}
