
// NOTE: we use a resolution of 9600 to have a common denominator for encoder and motor

// =============================================================== MOTOR

#include "AccelStepper.h"    // step motor control

#define MOTOR_STEP_PIN 33
#define MOTOR_DIR_PIN 32
#define MOTOR_ENABLE_PIN 26

#define LOG_MAX 900

// motor resolution in 1/8 step mode (jumper setting) is 3200

int   motorSpeed  = 3000;   // tolerable maximum speed
int   motorAccel  = 8000;   // tolerable acceleration
bool  motorIsOn   = false;  // true if motor has electric power

AccelStepper motor = AccelStepper(AccelStepper::DRIVER,MOTOR_STEP_PIN,MOTOR_DIR_PIN); // use pins #25 (dir) and #26 (step)

long motorPos() {
  // return a number between 0 (bottom) and 9599, TOP = 4800
  return (motor.currentPosition() % 3200) *3;
}

void motorTo(long target) {
  motor.moveTo(target/3);
}
void motorOn()  {
  digitalWrite(27,LOW);
  motorIsOn=true;
}

void motorOff() {
  digitalWrite(27,HIGH);
  motorIsOn=false;
}

void motorSetup() {
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);  // enable (LOW)
    pinMode(MOTOR_DIR_PIN, OUTPUT);  // dir
    pinMode(MOTOR_STEP_PIN, OUTPUT);  // step

    //pinMode( 2, OUTPUT);  // step rate divider (1/8 or 1/16)
    //digitalWrite(2,HIGH); // 1/8

    motor.setAcceleration(motorAccel);
    motor.setMaxSpeed(motorSpeed);
    motorOn();  // assuming that the motor arm is hanging down ==> zero position
    motor.setCurrentPosition(0);
}

// ========================================================================= ENCODER

#define ENCODER_PIN_A 22
#define ENCODER_PIN_B 17

long      encoderPos    = 0;
long      encoderPosM1  = 0;
unsigned long   encoderTime   = 0;
unsigned long   encoderTimeM1 = 0;
volatile int  encoderStateM1  = (digitalRead(ENCODER_PIN_A)==HIGH ? 1:0)
                + (digitalRead(ENCODER_PIN_B)==HIGH ? 2:0);

void onRotaryChange() {
  encoderTimeM1 = encoderTime;  // store last encoder time
  encoderTime   = micros();   // store the time as soon as possible
  encoderPosM1  = encoderPos; // note last position
  // calculate current state (0 .. 3 depending on input signal level)
  int state = (digitalRead(ENCODER_PIN_A)==HIGH ? 1 : 0) + (digitalRead(ENCODER_PIN_B)==HIGH ? 2 : 0);

  // encoder resolution is 2400;
  // compare and change position (we add subtract 4 for each pulse change to achieve 9600 per full circle)
  if      (encoderStateM1 == 0) encoderPos += state==1 ? 4 : -4;
  else if (encoderStateM1 == 1) encoderPos += state==3 ? 4 : -4;
  else if (encoderStateM1 == 2) encoderPos += state==0 ? 4 : -4;
  else if (encoderStateM1 == 3) encoderPos += state==2 ? 4 : -4;
  encoderStateM1=state;
}

void encoderSetup() {

  // the encoder has open collector connections, so we need a pullup resistor.
  // caution : NEVER connect the WHITE or GREEN WIRE to a GPIO which is configured as OUTPUT
  //           because if the OUTPUT IS HIGH it might burn the transistor inside the encoder
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  // Use an interrupt function to catch pulse changes on both pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), onRotaryChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), onRotaryChange, CHANGE);

  encoderPos = encoderPosM1 = 0;
}

// ========================================================================= KEYPAD
#define KEYPAD_PIN 32

// this function averages the value

int getKeypadValue() {
  int sum = 0;
  for(int i=0;i<11;i++) {
    sum += analogRead(KEYPAD_PIN);
  }
  sum /= 10;
  return sum;
}

// interpret the value

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

void keypadSetup() {
  pinMode(KEYPAD_PIN, INPUT);

}

// ======================================================================================== SETUP

#define NR_SWING_STEPS 8
// pairs of acceleration and target position
int swingSteps[NR_SWING_STEPS][2] = {
  { 1100,  -900 },
  { 1700,  1200 },
  { 2300, -1350 },
  { 2300,  1350 },
  { 2300, -1500 },
  { 2300,  1500 },
  { 2300, -1500 },
  { 2190,  4800 }
};

unsigned long startTime;

void setup() {
  Serial.begin(115200);

  motorSetup();
  encoderSetup();
  keypadSetup();

  Serial.println("\n");

  // perform swingup
  int swingStepNr=0;
  for (int s=0; s<NR_SWING_STEPS; s++) {
    motor.setMaxSpeed(swingSteps[s][0]);
    motor.setAcceleration(swingSteps[s][0]*3);
    motor.moveTo(swingSteps[s][1]/3);
    motor.runToPosition();  // blocks until target will be reached
  }

  Serial.println("swing up complete, motPos="+String(motorPos())+"\tpenPos="/*+String(encoderPos())*/);

  startTime=micros();
}

typedef struct valueStruct {
    int n;

    long mot[LOG_MAX];
    long pos[LOG_MAX];
    long posM1[LOG_MAX];
    long dTime[LOG_MAX];
    long lastPos;
} valueStruct;

valueStruct measuredValues;

measuredValues.n=0;
measuredValues.lastPos=0;

int state = 0;

void loop() {
  if (state==2) return;

  if (encoderPos != measuredValues.lastPos && n< LOG_MAX) {
    if (state==0 && (motorPos()+encoderPos) > 4360) {
      motor.setMaxSpeed(5000);
      motor.setAcceleration(15000);
      motor.moveTo(5200/3);
      // hier evtl eine Geschwindigkeit berechnen aus den Variablen
      state=1;
    }
    measuredValues.mot[measuredValues.n]=motorPos();
    measuredValues.pos[measuredValues.n]=encoderPos;
    measuredValues.posM1[measuredValues.n]=encoderPosM1;
    measuredValues.dTime[measuredValues.n]=encoderTime-encoderTimeM1;
    measuredValues.n++;
    measuredValues.lastPos=encoderPos;
  }
  else if (n>=LOG_MAX) {
    motorOff();
    // show values
    for (int i=0;i<LOG_MAX;i++) {
      Serial.println(
        String(i)+"\t"+
        String(measuredValues.mot[i])+"\t"+
        String((measuredValues.pos[i]-measuredValues.posM1[i])/4)+"\t"+
        String(measuredValues.mot[i]+measuredValues.pos[i])+"\t"+
        String(measuredValues.dTime[i])
      );
    }
    state=2;
  }

  motor.run();
}
