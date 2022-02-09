#include <AccelStepper.h>

#define MOTOR_STEP_PIN 33
#define MOTOR_DIR_PIN 32
#define MOTOR_ENABLE_PIN 26

#define KEYPAD_PIN 25

#define SET_MAX_SPEED 1500
#define SET_ACCELERATION_SPEED 8000
#define MOVE_TO 100

short iterator = 8;
int sum = 0;
int position = 0;

AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN); // Defaults to AccelStepper::DRIVER

void keypadSetup()
{
  pinMode(KEYPAD_PIN, INPUT);
  Serial.println(" ");
  Serial.println("---------------------------");
  Serial.println("--Starting Keypad Sketch!--");
  Serial.println("---------------------------");
}

void keypad(int n) {
  switch (n) { // +- 40
    case 4055 ... 4135: // 4095
      Serial.println("key one");
      break;
    case 3785 ... 3865: // 3825
      Serial.println("motor on: ENABLE PIN LOW");
      digitalWrite(MOTOR_ENABLE_PIN, LOW);
      break;
    case 3248 ... 3328: // 3288
      Serial.println("motor off: ENABLE PIN HIGH");
      digitalWrite(MOTOR_ENABLE_PIN, HIGH);
      break;
    case 2922 ... 3002: // 2962
      position += 10;
      stepper.moveTo(position);
      break;
    case 2691 ... 2771: // 2731
      position -= 10;
      stepper.moveTo(position);
      break;
    case 2492 ... 2572: // 2532
      ESP.restart();
      break;
  }
}

void readKeypad() {
  int temp = 0;
  for(int i=0;i<iterator;i++)
  {
    temp += analogRead(KEYPAD_PIN);
  }
  temp /= iterator;
  sum = temp;
  
}

void motorSetup() {
    /*
     * power supply information:
     * 800 mAmp
     * 20 Volt
     */
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);  // enable (LOW)
    pinMode(MOTOR_DIR_PIN, OUTPUT);  // dir
    pinMode(MOTOR_STEP_PIN, OUTPUT);  // step
    
    stepper.setMaxSpeed(SET_MAX_SPEED);
    stepper.setAcceleration(SET_ACCELERATION_SPEED);
    stepper.setCurrentPosition(position); // position = 0
    stepper.moveTo(MOVE_TO);
}

void setup() {
  // Setup serial monitor
  Serial.begin(115200);
  keypadSetup();
  motorSetup();

}

void loop() {
  readKeypad();
  keypad(sum);
  stepper.run();
  delay(1);
}
