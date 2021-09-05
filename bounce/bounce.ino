#include <AccelStepper.h>

#define MOTOR_STEP_PIN 33
#define MOTOR_DIR_PIN 32
#define MOTOR_ENABLE_PIN 26
#define MOTOR_SHUTDOWN_PIN 13

//#define ENCODER_WHITE
//#define ENCODER_GREEN

// Define a stepper and the pins it will use
AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN); // Defaults to AccelStepper::DRIVER

bool isOn = true;

void shutdown()
{
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  Serial.println("-power off-");
  isOn = false;
  delay(100);
}

void setup()
{
  Serial.begin(115200);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_SHUTDOWN_PIN, INPUT);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  
  stepper.setMaxSpeed(4000);
  Serial.println("-startup-");
  delay(2000);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);

}

  int swingSteps[11][2] = { // setAcceleration   moveTo
    {  1300,  -300},        // 1100             -300
    {  1900,   400},        // 1700              400
    {  2200,  -450},        // 2300             -450
    {  2500,   450},        // 2300              450
    {  2500,  -500},        // 2300             -500
    {  2500,   500},        // 2300              500
    {  2500,  -450},        // 2300             -500
    {  2500,   700},        // 2300              1600
    {  2500,  -900},
    {  2500,   1200},
    {  2950,  -1700},
  };

  int d = 0;
  int f = 0;

void loop()
{
  if (stepper.distanceToGo() == 0)
  {
    stepper.setAcceleration(swingSteps[d][0]*3);
    stepper.moveTo(swingSteps[d][1]*4/5);
    if(d<11)
    {
      d++;
    } else {
      if(isOn)
      {
        shutdown();
      }
    }
  }
  stepper.run();

  if (isOn && micros() == 20000000)
  {
    shutdown();
  }

  if (isOn && digitalRead(MOTOR_SHUTDOWN_PIN) == HIGH)
  {
    shutdown();
  }
}
