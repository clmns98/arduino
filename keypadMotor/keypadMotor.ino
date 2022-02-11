#include <AccelStepper.h>

#define MOTOR_STEP_PIN 33
#define MOTOR_DIR_PIN 32
#define MOTOR_ENABLE_PIN 26

#define KEYPAD_PIN 25

short iterator = 8;
int keypadValue = 0;
int position = 0;
bool isPressed = false;
boolean hasButtonReleaseBeenOutput = true;
const int standardAccelerationSpeed = 8000;
int accelerationSpeed = standardAccelerationSpeed;

AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN); // Defaults to AccelStepper::DRIVER

void keypadSetup()
{
  pinMode(KEYPAD_PIN, INPUT);
  Serial.println(" ");
  Serial.println("---------------------------");
  Serial.println("--Starting Keypad Sketch!--");
  Serial.println("---------------------------");
  Serial.println(" ");
  Serial.println("              (pin side) ");
  Serial.println(" /------------------------------------------/");
  Serial.println(" / ______     / LOW               / HIGH    /");
  Serial.println(" /-------------------------------------------");
  Serial.println(" / clockwise  / antiClockwise     / restart /");
  Serial.println(" /------------------------------------------/");
  Serial.println(" / ______     / ______            / ______   ");
  Serial.println(" /------------------------------------------/");
  Serial.println(" / ______     / ______            / ______  /");
  Serial.println(" /------------------------------------------/");
  Serial.println("              (bottom side) ");
  Serial.println(" ");
}

void keypad() {
  switch (keypadValue) { // +- 40
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
    case 2922 ... 3002: // 2962 - move clockwise
      position += 1;
      stepper.moveTo(position);
      pressButton();
      break;
    case 2691 ... 2771: // 2731 - move antiClockwise
      position -= 1;
      stepper.moveTo(position);
      pressButton();
      break;
    case 2492 ... 2572: // 2532
      ESP.restart();
      break;
    /*default:
      Serial.println("no button pressed");*/
  }
}

void readKeypad() {
  int temp = 0;
  for(int i=0;i<iterator;i++)
  {
    temp += analogRead(KEYPAD_PIN);
    delayMicroseconds(1950);
  }
  temp /= iterator;
  keypadValue = temp;
}

void pressButton() {
    hasButtonReleaseBeenOutput = false;
    isPressed = true;
    Serial.println("Button pressed");
}

void adaptAcceleration() {
    if(isPressed) {
        accelerationSpeed += 1;
        stepper.setAcceleration(accelerationSpeed);
    } else {
        stepper.setAcceleration(standardAccelerationSpeed);
    }
}

void showPosition() {
    Serial.println(stepper.currentPosition());
}

void motorSetup() {
    /*
     * power supply information:
     * 800 mAmp
     * 20 Volt
     */

    #define SET_MAX_SPEED 1500
    #define MOVE_TO 100

    pinMode(MOTOR_ENABLE_PIN, OUTPUT);  // enable (LOW)
    pinMode(MOTOR_DIR_PIN, OUTPUT);  // dir
    pinMode(MOTOR_STEP_PIN, OUTPUT);  // step

    stepper.setMaxSpeed(SET_MAX_SPEED);
    stepper.setAcceleration(standardAccelerationSpeed);
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
  keypad();
  // showPosition();
  adaptAcceleration();
  Serial.println(keypadValue);
  stepper.run();
  if(!isPressed && !hasButtonReleaseBeenOutput) {
      Serial.println("---Button released---");
      hasButtonReleaseBeenOutput = true;
  }
  isPressed = false;
  delay(random(1,10));
}
