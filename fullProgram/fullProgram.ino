/*
             A program to control an inverted pendulum
             attached to the end of a rotating arm controlled by a step
motor.

                             @   pendulum tip  has a tendency to move
right with increasing speed
                            /                  will stop its movement
due to balancing forces
                  pendulum /
                          /
                         /
                        o  pendulum axis, must be moved right quite fast
                         \
              rotating arm \
                           O  motor axis (fixed), must turn clock wise

         In the scheme above the motor must turn clockwise pretty fast
to get the pendulum axis
         below its tip (and even somewhat more to the right).
         Without such a movement the pendulum would fall to the right
with increasing speed.
         During the movement of the motor arm (and the pendulum axis)
the pendulum will continue falling
         to the right but (hopefully) with decreasing speed.
         The motor must go faster than the pendulum tip. It must even go
further right
         than the pendulum tip to "catch the pendulum" and bring the
whole system back to a state
         where the pendulum tip, the pendulum axis and the motor axis
are all three aligned on
         a vertical line ("ideal balancing point"). Note that the
horizontal movement of the
         pendulum axis is proportional to the cosine() of the motor
rotation.
         Once the motor arm is nearly horizontal it will have almost no
balancing effect
         because it is essentially going up and down instead of left and
right.
         This is an additional complication of this control task
compared to a linear motor
         which would move the pendulum axis only horizontally.

         The position of the pendulum AXISs is defined by the position
of the stepper motor.
         As we control the motor by our software we always know its
position - provided the motor
         does not lose any steps.

         The signal about the pendulum´s current POSITION comes from a
rotary encoder which works
         purely incrementally. If we define a starting position and
track the
         encoder signals continously we are able to calculate the
position of the pendulum RELATIVE
         TO THE MOTOR ARM ANGLE because the encoder ifs fixed on the
motor arm.
         Subtracting the motor arm position from that encoder angle will
finally give us the position
         of the pendulum RELATIVE to the motor mounting, i.e. to the
outside world.
         This will be the pendulum angle we need to control its position.

         The encoder gives us ANGULAR POSITIONS RELATIVE to its housing.
         The encoder disk hast 600 lines and the encoder has two
detector channels
         with an offset of 25% between the lines (90° shift). A full
circle of the
         pendulum axis produces 1200 voltage transitions per channel. So
we have a
         total resolution of 2400 impulses per rotation ( ~ 7 pulses per
degree).
         Values are between -inf .. +inf as the pulses are being counted
continuously
         when the pedulum goes over the top.
         positive direction == CCW. i.e. pendulum swinging to the right
form the perspective
         of an observer looking from the frontside ("into" the encoder).

         The motor gives us ANGULAR POSITIONS relative to its (fixed)
mounting
         a full circle is 3200 because we use a driver which
interpolates the 200 physical
         steps of the stepper motor by a factor of 16.
         Values are between -inf .. +inf and the positive direction== CCW

         When the motor turns, the housing of the encoder gets turned, too.
         so what we need, is a NORMALIZED PENDULUM POSITION
         a full circle is 2400, values may be translated (for display)
to -180°..+180°
         with 0 being the vertical hang-down position.

         It is useful to have an integer representaion of angles.
         If we multiply motor steps by 9, pendulum steps by 12 and
degrees by 8 we
         get the same value (28800).

         positive values are on the right hand side.
         the penPos values get calculated whenever the encoder gives us
a changed signal

  */

// =============================================================== RESET
BUTTON

void resetFunc() {
     Serial.println("RESET .............");
     ESP.restart();
}

void resetButtonSetup() {
     show("setup RESET BUTTON ..");

     // the right button on the ESP32 board is attached to GPIO 35
     pinMode(35, INPUT_PULLUP);        // analog in
     attachInterrupt(digitalPinToInterrupt(35), resetFunc, FALLING);
}

// =============================================================== LEDs

#include <FastLED.h>
CRGB             leds[2];                    // two LEDS (left and rightside)

void ledsSetup() {
     show("setup LEDs ..");
     FastLED.addLeds<WS2801,13,15,RGB>(leds,2);            // data pin = #13, clock pin = #15
}
void ledsShow(unsigned long col) {
     leds[0]=leds[1]=col;
     FastLED.show();
}

// the code for changing the LEDs is included in the encoder checking

// ===============================================================
POSITION SENSOR

int positionSensorValue;

int setupPositionSensor() {
     show("setup POSITION SENSOR ..");
        pinMode(32, INPUT);        // analog in
}
int checkPositionSensor() {
     int val    = analogRead(32);
     // show values during approach
     // if (val<200) show(String("POS_SENSOR=")+val);
     return val;
}


// =============================================================== MOTOR

#include "AccelStepper.h"        // step motor control

int                motorPPR        =    3200;    // 200 * 16 = 3200
steps = full circle
int                motorMultiplier =      3;    // a factor to make
motor steps commensurable with encoder steps
int                motorDist        =     600;    // steps for the next
motor movement
int             motorSpeed        =   1000;    // tolerable maximum speed
int                motorAccel        =    3000;    // tolerable acceleration
bool            motorIsOn        =    false;    // true if motor has
electric power

AccelStepper motor = AccelStepper(AccelStepper::DRIVER,25,26);

float motorDEG() {
     // convert motor position to degrees ( -360 .. + 360 )
     return (motor.currentPosition() % motorPPR) * 360. / motorPPR;
}
void  motorOn()  {
     digitalWrite(27,LOW);
     motorIsOn=true;
     // show("motor is ON");
}
void  motorOff() {
     digitalWrite(27,HIGH);
     motorIsOn=false;
     // show("motor is OFF");
}
void  motorSetTarget(int pos) {
     motor.moveTo(pos*3);
}
int   motorPosition() {
     return motor.currentPosition()*3;
}
void  motorSetup() {
     show("setup MOTOR ..");

        pinMode(27, OUTPUT);    // enable (LOW)
        pinMode(26, OUTPUT);    // dir
        pinMode(25, OUTPUT);    // step

        pinMode( 2, OUTPUT);    // NOT USED, CONNECTED TO step rate
divider (1/8 or 1/16)
        digitalWrite(2,HIGH);

     motor.setAcceleration(motorAccel);
     motor.setMaxSpeed(motorSpeed);
        motorOn();
        motor.setCurrentPosition(0);
}
void  motorCalibrate() {

     int sensorThreshold = 150;    // values below this mean that the
motor arm is close to the sensor
     int sensorOffset = 190;    // the angle difference in steps between
the sensor and the vertical position

     // use decent speed and acceleration
     motor.setAcceleration(1500);
     motor.setMaxSpeed(700);
        motorOn();
        // wherever we are: assume it to be ZERO
        motor.setCurrentPosition(0);


        motor.moveTo(-300);    // should be enough to get the motor arm
away from the sensor
        motor.runToPosition();

     // in case the motor arm is still close to the sensor: move away
     if (checkPositionSensor() < sensorThreshold) {
            motor.moveTo(-600);    // move farther away
            motor.runToPosition();
     }

     // move counter clockwise until we get a sensor signal
        motor.moveTo(3200);    // not more than a full circle
        int zeroPosition=0;
        int posValue;
     for (;;) {
         if (!motor.isRunning()) break;    // full circle completed
without detection
         if (motor.run()) {
             // if we made a step check the sensor
             posValue=checkPositionSensor();
             if (posValue<200) show(String("POSITION_SENSOR = ")+posValue);
             if (posValue < sensorThreshold) {
                 show("POSITION_SENSOR is below threshold");
                 zeroPosition=motor.currentPosition();
                 break;
             }
         }
     }

     Serial.println(String("sensor signal at ")+zeroPosition);
     motor.setAcceleration(1000);
     motor.setMaxSpeed(500);
     motor.moveTo(zeroPosition+sensorOffset);
     motor.runToPosition();

     // set the vertical position of the motor arm as new ZERO
        motor.setCurrentPosition(0);
     Serial.println(String("motor is at position ")+motorPosition());
}

// =============================================================== RECORDER

// rec holds several series of data
// a series is characterized by the final speed at the bottom position
// rec holds per series and pendulum position the current speed which
leads to the final speed of the series
// zero = 9600 ticks = bottom, negative values = 9600 - pos

#define            REC_NO_SERIES             10        // the number of
serieses we hold
#define            REC_SIZE                100        // the number of
positions we hold : 100 * 96 = 9600
long            rec[REC_NO_SERIES][2][REC_SIZE];// time gap for time /
direction / pendulum position
int                recSpeedResolution    =      24;    // time interval
for speed detection
int                recResolution        =      96;    // time interval
for recording; MUST BE multiple of speedRes
int                recSeriesNr            =       0;    // the index of
the next series to be recorded

// for performance reasons the code to fill the recorder array is
included inline where needed

// ===============================================================
ENCODER (CONTROL WHEEL)

#include "RotaryEncoder.h"        // an optical encoder with two output
signals

int                wheelMultiplier    =        1;        // numerical
steps per physical pulse
int                wheelPPR        =    9600;        // physical pulses
per round * multiplier; 600 lines => 2400 * 4 pulses
long            wheelPos        =        0;        // position of
control weheel
long            wheelPosM1        =        0;        // previous
position of control wheel
boolean            wheelMode        =     false;
RotaryEncoder    wheel;

void onPulseWheel() { wheel.handlePulse(); }    // pulse detection

void wheelReset() {
     wheel.setPositionDeg(motorDEG());
}

void wheelSetup() {
     show("setup CONTROL WHEEL ..");
     wheel = RotaryEncoder(36,37,wheelPPR,wheelMultiplier,onPulseWheel);
     wheelReset();    // hanging down
}

void wheelCheck() {
     // check if value changed; the encoder itself works based on interrupts
     // and should NEVER loose any steps; if we are fast enough we should
     // only see incremental +1/-1 changes here. If we are too slow
     // the observed changes may be larger

     if (wheel.position / 20 != wheelPosM1) {
         // shift values
         wheelPosM1 = wheelPos;
         wheelPos = wheel.position / 10;    // reduce resolution
         motorSetTarget(wheelPos*4);    // translate gear ratio to 1:1
between wheel and motor
         // Serial.println(String("wheel ")+String(wheelPos));
     }
}

// ===============================================================
ENCODER (PENDULUM)

unsigned long     encAt            =          0;    // time (usec) of
last pendulum (encoder) change
int                encMultiplier    =        4;    // numerical steps
per physical pulse
int                encPPR            =    9600;    // physical pulses
per round * multiplier; 600 lines => 2400 * 4 pulses
long            penPos            =        0;    // position of pendulum
(from outside world perspective
long            penPosM1        =        0;
long            penPosM2        =        0;
long            penPosMax        =        0;
long            penPosMin        =        0;
int                penState         =     ' ';    // current state like
left side, left peek, right side, right peek
int                penStateM1        =        ' ';
long            penGap            =        0;    // the number of
microseconds between two physical pulses
int             encPositionM1     =         0;    // previous encoder
position

unsigned long     now;                        // current time in micro
seconds
int                progMode        =         0;    // the current
function mode of the program like swinging or balancing

RotaryEncoder encoder;

void  onPulsePenEnc() {
     // pulse detection
     encoder.handlePulse();
}
float encDEG(int pos) {
     // convert pendulum position to degrees ( -360 .. + 360 )
     return (pos % encPPR) * 360. / encPPR;
}
void  encReset() {
     motorOn();
     encoder.setPositionDeg(-motorDEG());
     penPos=penPosM1=penPosM2=penGap=penPosMin=penPosMax=0;
     ledsShow(0x070700);
     progMode=0;
}
void  encSetup() {
     show("setup PENDULUM ENCODER ..");
     encoder = RotaryEncoder(22,17,encPPR,encMultiplier,onPulsePenEnc);
     encReset();    // hanging down
     penPos=penPosM1=penPosM2=0;
}
void  encCheck() {
     // check if value changed; the encoder itself works based on interrupts
     // and should NEVER loose any steps; if we are fast enough we should
     // only see incremental +1/-1 changes here. If we are too slow
     // the observed changes may be larger
     // this function records the positions on demand

#ifdef BALANCE

     if (encoder.position != encPositionM1) {
         // shift values
         penPosM2 = penPosM1;
         penPosM1 = penPos;
         // add commensurable positions of encoder and motor to get
outside world view of pendulum axis
         penPos   = encoder.position + motorPosition();
         encAt     = encoder.timeAbs;    // time of last pulse detection

         // decide about hemisphere, detect turning points
         int penPosNorm = penPos % encPPR;
         if ((penPosNorm < 0 && penPosNorm < -encPPR / 2) ||
(penPosNorm > 0 && penPosNorm < encPPR / 2 )) {
             penState='r';
             if (penPosM2<penPosM1 && penPos<penPosM1) {
                 penPosMax=penPosM1;
                 penState='R';
                 leds[1]=CRGB::White;
                 // show("penMax=\t"+String(penPosMax));
             }
             else {
                 leds[1]=0x000033;
             }
             leds[0]=CRGB::Black;
         }
         else {
             penState='l';
             if (penPosM2>penPosM1 && penPos>penPosM1) {
                 penPosMin=penPosM1;
                 penState='L';
                 // show("penMin=\t"+String(penPosMin));
                 leds[0]=CRGB::White;
             }
             else {
                 leds[0]=0x000033;
             }
             leds[1]=CRGB::Black;
         }
         FastLED.show();

         // create profile if requested
         if (recState==1) {
             // Serial.println("time    penAbs    motAbs penPos
penHeight    penSpeed    penKin    penEnergy motSpeed");
             Serial.println("time    penEnergy    penHeight penKin
penSpeed    motHeight    motSpeed    penPos    penAbs motAbs");
             rec[0][0]=encAt;
             rec[0][1]=encoder.position;
             rec[0][2]=motorPosition();
             recInx=1;
             recState=2;
         }
         else if (recState==2) {
             // collect snapshots every recResolution steps and at
turning points and bottom passage
             if (abs(penPos-rec[recInx-1][1])>=recResolution ||
(penStateM1=='l' && penState=='r') || (penStateM1=='r' &&
penState=='l')  || penState=='L' || penState=='R') {
                 rec[recInx][0]=encAt;
                 rec[recInx][1]=encoder.position;
                 rec[recInx][2]=motorPosition();
                 if (++recInx>=RECORDING_SIZE) recState=3;
             }
         }
         else if (recState==3) {
             // log recording to screen (doing this at the end as it
would slow down the measurement if done in sync)
             int height;
             float motSpeed, penSpeed, penKin, penKin0, penKinM1=0;
             for (int r=1;r<recInx;r++) {
                 penSpeed = 100000. * float(rec[r][1]-rec[r-1][1]) /
(rec[r][0]-rec[r-1][0]);
                 motSpeed = 100000. * float(rec[r][2]-rec[r-1][2]) /
(rec[r][0]-rec[r-1][0]);
                 penKin0 =  penSpeed * penSpeed * 0.0045;
                 penKin = 0.5 * (penKinM1 + penKin0);
                 penKinM1 = penKin0;
                 long penHeight =         4800. * (1.
-cos((rec[r][1]+rec[r][2])* PI * 2 / encPPR));
                 long motHeight = 0.2 *     4800. * (1. -cos(rec[r][2]*
PI * 2 / encPPR));
                 Serial.println(
                      String(rec[r][0]-rec[0][0])+"\t"    // time
relative to first row
                     +String(penHeight+penKin)+"\t"        // pendulum
total energy
                     +String(penHeight)+"\t"                // pendulum
height relative to horizont
                     +String(penKin)+"\t"                // pendulum
kinetic energy
                     +String(penSpeed)+"\t"                // pendulum speed
                     +String(motHeight)+"\t"                // motor height
                     +String(motSpeed)+"\t"                // motor speed
                     +String(rec[r][1]+rec[r][2])+"\t"    // pendulum
position relative to horizont
                     +String(rec[r][1])+"\t"                // pendulum
position absolute
                     +String(rec[r][2])+"\t"                // motor
position absolute
                 );
             }
             recState=0;
         }

         encPositionM1 = encoder.position;
         penStateM1 = penState;

     }
     else if (encAt != 0 && now-encAt > 1000000) {
         // encoder unchanged for one second, assume we have zero position
         encAt=0;
         if (!motorIsOn) motor.setCurrentPosition(0);
         encoder.setPositionDeg(-motorDEG());
         penPos=penPosM1=penPosM2=penPosMin=penPosMax=0;
         leds[0]=0x070700; leds[1]=0x070700;    FastLED.show();
     }
#endif

}
long  penPosition() {
     return encoder.position+motorPosition();
}
void encCalibrate() {
     // move motor slowly until pendulum starts to accelerate

     long lastPos, leftMot,leftPen,rightMot,rightPen;

     // let pendulum hang to the left side
     motor.runToNewPosition(-400);
     motor.runToNewPosition(0);

     // wait a moment then set encoder to zero (accepting a bias for the
moment)
     delay(2000);
     encoder.setPosition(0);

     lastPos=encoder.position;
     // look for the moment when the pendulum begins to fall to the
other side
     for (int n=0;n<400;n++) {
         motor.runToNewPosition(n);
         delay(20);
Serial.println(String(motorPosition())+"\t"+String(encoder.position));
         if (abs(lastPos-encoder.position) > 20) {
             rightMot=motorPosition();
             rightPen=encoder.position;
             break;
         }
         lastPos=encoder.position;
     }

     // let pendulum hang to the right side
     motor.runToNewPosition(200);        // motor arm on left side
     motor.runToNewPosition(0);            // motor arm vertical

     // wait a moment then start moving clock wise and wait until
pendulum falls
     delay(1000);
     lastPos=encoder.position;
     for (int n=0;n<400;n++) {
         motor.runToNewPosition(-n);    // run clockwise
         delay(20);
Serial.println(String(motorPosition())+"\t"+String(encoder.position));
         if (abs(lastPos-encoder.position) >= 20) {
             leftMot=motorPosition();
             leftPen=encoder.position;
             break;
         }
         lastPos=encoder.position;
     }

     motor.runToNewPosition(0);            // motor arm vertical

  Serial.println(String(leftMot)+"\t"+String(leftPen)+"\t"+String(rightMot)+"\t"+String(rightPen));

}

// =============================================================== balance

void catcher() {
     // move motor slowly until pendulum starts to accelerate

     motor.setAcceleration(3000);
     motor.setMaxSpeed(1500);

     long lastPos, leftMot,leftPen,rightMot,rightPen;

     // let pendulum hang to the left side
     motor.runToNewPosition(0);

     delay(1000);

     lastPos=encoder.position;
     // look for the moment when the pendulum begins to fall to the
other side
     for (int n=0;n<400;n++) {
         motor.runToNewPosition(n);
         delay(20);
         if (penPosition()<-30) break;
         lastPos=encoder.position;
     }

     motor.setMaxSpeed(2000);
     motor.setAcceleration(6000);

     int penPos,motPos;
     double factor=1.1;
     for (int n=0;n<20;n++) {
         penPos=penPosition();
         motPos=motorPosition();
         Serial.println(String(motPos)+"\t"+String(penPos));
         // make sure motor does not move beyond 90 degrees
         if (motPos>-800 && motPos<800) {

             if         (penPos < -10) motor.move((int)(penPos*factor));
             else if (penPos >  10) motor.move((int)(penPos*factor));
             else {
                 delay(20);    // pendulum is almost upright
                 continue;
             }
             motor.runToPosition();
         }
     }

     delay(2000);
     motor.runToNewPosition(-130);

     delay(400);
     motor.setMaxSpeed(1000);
     motor.setAcceleration(1000);
     motor.runToNewPosition(0);
}

void balanceM() {
     // this function is called after the motor made a step
     // so we have some time for calculations
}

void balance() {
     // this function is called if the motor did not make a step
     // so we should not use too much time here, because this might
postpone
     // the next due motor step , producing jitter and vibrations
}

// =============================================================== swing up

void swing(long stopPos, long speed, long acc, long dec) {

     // start moving with acc, begin to stop at stopPos, stopping with dec
     Serial.println("swing to "+String(stopPos));

     boolean decApplied = false;
     motorOn();
     long stopPos3= stopPos / 3;
     long acc3=acc / 3;
     long dec3 = dec / 3;
     long speed3 = speed / 3;
     motor.setMaxSpeed(speed3);
     int dir = motor.currentPosition() > stopPos3 ? -1 : 1;
     motor.setAcceleration(acc3);
     // move to a location beyond stopPos
     motor.moveTo(stopPos3 + dir * 1000);
     float breakSpeed;
     for(;;) {
         motor.run();
         if (!decApplied) {
             // when close to max speed change to deceleration
             if (abs(abs(motor.speed())-speed3)<10) {
                 Serial.println("applying decel at
"+String(motorPosition()));
                 motor.setAcceleration(dec3);
                 motor.moveTo(stopPos3);
                 decApplied=true;
             }
             else if (false) {
                 Serial.println("slow down early at
"+String(motor.speed()));
             }
         }
         if (!motor.isRunning()) break;
     }
     Serial.println("motor speed
"+String(breakSpeed)+"\t"+String(breakSpeed*breakSpeed));
     Serial.println("motor is at "+String(motorPosition()));
     Serial.println("break distance "+String(motorPosition()-stopPos));
}

//
=====================================================================================
void swingUpAuto() {

     if (!motorIsOn) {
         motorOn();
         delay(1000); // make sure pendulum is calm;
     }

        motor.setCurrentPosition(0);
        encReset();    // hanging down

     long motPos = -3600;
     swing(motPos, 6000, 24000, 60000);

     // loop several times

     long penPos, lastPenPos=penPosition();
     long minPos = 9600, maxPos = -9600;
     int dir = -1;
     boolean start=false;

     for (int step = 0; step < 12;) {

         // find current amplitude
         penPos=penPosition();
         if (penPos < lastPenPos) {
             // pendulum swinging clockwise
             if (dir==1) maxPos = lastPenPos;    // reduce maximum if we
lost energy
             dir= -1;
             if (penPos<minPos) {
                 minPos = penPos;
                 if (minPos<-4800) break;    // going over the top
             }
             lastPenPos=penPos;
         }
         else if (penPos > lastPenPos) {
             // pendulum swinging counter clockwise
             if (dir== -1) minPos = lastPenPos;    // reduce minimum if
we lost energy
             dir= 1;
             if (penPos>maxPos) {
                 maxPos = penPos;
                 if (maxPos>4800) break;    // going over the top
             }
             lastPenPos=penPos;
         }

         if (motPos<0) {
             // motor is in left position
             // if (penPos > minPos) start = true;    // immediately
after left top
             // if (dir== -1 && penPos < -0.3 * maxPos) start = true;
             if (dir== -1 && maxPos <=  2400 && penPos < -0.3 * maxPos)
start = true;
             if (dir==  1 && minPos <= -2400 && penPos > -2400) start =
true;
         }
         else {
             // motor is in right position
             // if (penPos < maxPos) start = true;    // immediately
after left top
             // if (dir==  1 && penPos > -0.3 * minPos) start = true;
             if (dir==  1 && minPos >= -2400 && penPos > -0.3 * minPos)
start = true;
             if (dir== -1 && minPos <= -2400 && penPos < 2400) start = true;
         }

         if (start) {
             // execute swing to opposite side
             swing(motPos, 6000, 24000, 60000);
             start=false;
             step++;
         }
     }

     /* switch to balancing ...
     unsigned long startAt = micros();
     for(int t=0;t<900;t++) {
         controlCurve[t][0]=micros()-startAt;
         controlCurve[t][1]=encoder.position;
         delay(1);
     }
     */

     motorOff();

}

// ===============================================================
detect and handle keyboard events

String cmds[] = { "", // 1 .. 16

     "TOGGLE WHEEL",        "--",                "LEARN",     "RESET",
     "SWING",            "--",                "--", "--",
     "CATCHER",            "SHOW",                "ACCEL -",     "CALIB
PENDULUM",
     "STOP",                "RELEASE",            "CALIB MOTOR",
"ZERO PENDULUM"
};


int getCmd(int key) {
     // rectangular keypad, producing analog values

     if         (key <  600)     return  0;

     else if (key <  800)     return  13;
     else if (key < 1000)     return   9;
     else if (key < 1200)     return   5;
     else if (key < 1500)     return   1;

     else if (key < 1600)     return  14;
     else if (key < 1680)     return  10;
     else if (key < 1760)     return   6;
     else if (key < 1900)     return   2;

     else if (key < 2100)     return  15;
     else if (key < 2200)     return  11;
     else if (key < 2400)     return   7;
     else if (key < 2700)     return   3;

     else if (key < 3000)     return  16;
     else if (key < 3500)     return  12;
     else if (key < 3900)     return   8;
     else                     return   4;
}

// ===============================================================
process Command

int    cmdM1=0;

void processCommand(int cmd) {
     // process the latest command

     if (cmd<=0) return;

     // ----------------------------------

     if         (cmd ==  1 ) {     // MANUAL CONTROL
         wheelMode= !wheelMode;
         show("wheel control : "+String(wheelMode));
     }
     else if (cmd ==  2 ) { // SWING UP CONTROL
     }
     else if (cmd ==  3 ) { // OBSERVE
         observe();
     }
     else if (cmd ==  4 ) { // RESET
     }

     // ----------------------------------

     else if (cmd ==  5 ) { // SWING
         if (motorPosition()<0)     swing(1800, 6000, 24000, 60000);
         else                     swing(1800, 6000, 24000, 60000);
     }
     else if (cmd ==  6 ) { // SWING AUTO
         swingUpAuto();
     }
     else if (cmd ==  7 ) { //
     }
     else if (cmd ==  8 ) { //
     }

     // ----------------------------------

     else if (cmd ==  9 ) { // CATCHER
         show("cmd: "+cmds[cmd]);
         catcher();
     }
     else if (cmd ==  10 ) { // SHOW
         show(
             "penPos="    + String(penPosition())
             +"\tmotPos="+String(motorPosition())
         );
     }
     else if (cmd ==  11 ) { // MOTOR ACCEL DECREASE
         if (motorAccel>200) motorAccel-=200;
         show("accel="+String(motorAccel));
         motor.setAcceleration(motorAccel);
     }
     else if (cmd ==  12 ) { // CALIBRATE PENDULUM
         show("cmd: "+ cmds[cmd]);
         encCalibrate();
     }

     // ----------------------------------

     else if    (cmd ==  13 ) {     // STOP
         show("cmd: "+ cmds[cmd]);
         motorOn();
         motor.setAcceleration(motorAccel);
         motor.setMaxSpeed(motorSpeed);
            motor.setCurrentPosition(0);
            wheelReset();
     }
     else if (cmd ==  14 ) { // RELEASE
         show("cmd: "+ cmds[cmd]);
         motorOff();
     }
     else if (cmd ==  15 ) {                                     //
CALIB MOTOR
         show("cmd: "+ cmds[cmd]);
         motorCalibrate();
            delay(1000);
         encoder.setPosition(132);        // estimation
         Serial.println("pendulum is assumed to be at position
"+String(penPosition()));
     }
     else if (cmd ==  16 ) { // ZERO PENDULUM
         show("cmd: "+ cmds[cmd]);
         encoder.setPosition(0);
     }

}

// ===============================================================
handle Keys

int                keyCount      = 0;
unsigned long     keyReadAt     = 0,
                 keySum        = 0;

int keyboardCheck() {

     // we expect to have an AC component of 50Hz on the key signal
     // because the cables will catch the frequency of the power line.
     // therefore we collect signals for 20 msec (1 full wave) and use
the average
     // this will ideally eliminate the AC component.

     int key  = analogRead(33);        // keypad is connected to PIN #33

     // if we see a small value: NO KEY is currently pressed
     if (key < 600) {
         keyReadAt=0;        // wait for the next moment of reading a
key signal
         return cmdM1=0;     // last key was released
     }

     // if we see a large value; calculate key value immediately because
the effect
     // of AC grid voltage overlay can be neglected
     if (key >= 2700) {
         keyReadAt = 0;
         int cmd = getCmd(key);
         return (cmdM1==cmd) ? -1 : cmdM1=cmd;    // return -1 if same
key pressed
     }

     // else start collecting values
     if (keyReadAt==0) {
         keyReadAt=now;    // note when we saw the first signal
         keySum=key;        // and store it
         keyCount=1;
         return -3;        // started to collect values
     }

     // collect and sum up key values for 20 msec (= 1 wave at 50 Hz) =
20000 usec
     if (now < keyReadAt + 20000) {
         keySum+=key;
         keyCount++;
         return -2;        // still collecting values
     }

     // assign the command number based on the average key value we have
seen
     keyReadAt = 0;
     int cmd = getCmd(keySum / keyCount);
     // if (cmd!=cmdM1) show("@ "+String(now)+"\tread@
"+String(keyReadAt)+"\tkeyCount="+String(keyCount)+"\tkeyAvg="+String(keySum/keyCount)+"
--> cmd="+String(cmd)+"\t"+cmds[cmd]);
     return (cmdM1==cmd) ? -1 : cmdM1=cmd;    // return -1 if same key
pressed
}

void setup() {

     Serial.begin (115200);
     delay(100);

     resetButtonSetup();
     ledsSetup();
     motorSetup();
        wheelSetup();
        encSetup();

     show("INVERTED PENDULUM starting..");
}

void loop() {
     now = micros();

     // check keyboard and process command if there was a key pressed
     processCommand(keyboardCheck());

     if ( progMode < 20 || progMode >= 30 ) {
         if (wheelMode) wheelCheck();
         encCheck();
     }

     if (motor.run()) {
         if ( progMode == 30 ) balanceM();
     }
     else {
         if ( progMode == 30 ) balance();
     }
     penPos=penPosition();
     leds[0]= (penPos >= 0) ?0x000033 : 0x000000;
     leds[1]= (penPos <= 0) ?0x000033 : 0x000000;
     FastLED.show();
}

// =============================================================== LOGGING

void record(int action) {

     if (action==0) {

         for(int r=0;r<REC_SIZE;r++)
rec[recSeriesNr][0][r]=rec[recSeriesNr][1][r]=0;
         show("series "+String(recSeriesNr));
         ledsShow(0x070000);

         // wait for the pendulum to reach the next speed check point
         unsigned long syncTime, lastSyncTime;
         while (penPosition() % recSpeedResolution!=0) {}
         lastSyncTime = micros();
         long penPos, posAbs, lastPenPos = penPosition();
         long gap;

         unsigned long startedAt = millis();
         for(;;) {
             penPos = penPosition();
             if (penPos != lastPenPos && penPos % recSpeedResolution == 0) {
                 syncTime=micros();

                 // calculate momentary time gap between two sync
points, use negative times for moving left
                 gap= (syncTime-lastSyncTime) * (penPos-lastPenPos > 0 ?
1 : -1);
                 lastSyncTime=syncTime;
                 lastPenPos=penPos;

                 // record the position if due
                 if (penPos % recResolution == 0) {
                     posAbs = penPos % 9600;
                     if (posAbs<0) posAbs += 9600;
                     posAbs=posAbs/recResolution;
                     // Serial.println("gap
"+String(penPos)+"\t"+String(posAbs)+"\t"+String(gap));
                     if (gap>=0) rec[recSeriesNr][0][posAbs] = gap;
                     if (gap<=0) rec[recSeriesNr][1][posAbs] = gap;

                     // at bottom
                     if (posAbs==0) {

                         Serial.println("elapsed
"+String(millis()-startedAt));

                         ledsShow(0x070700);

                         // start new series, wrap around
                         if (++recSeriesNr>=REC_NO_SERIES) recSeriesNr=0;
                         for(int r=0;r<REC_SIZE;r++)
rec[recSeriesNr][0][r]=rec[recSeriesNr][1][r]=0;

                         if (millis() > startedAt + 3000) break;
                         ledsShow(0x070000);
                     }
                 }
             }
         }
         ledsShow(0x000700);
     }

     // show result
     for (int s=0; s<recSeriesNr;s++) {
         for (int r=0;r<REC_SIZE;r++)
Serial.print(String(rec[s][0][r])+"\t");
         Serial.println();
         for (int r=0;r<REC_SIZE;r++)
Serial.print(String(rec[s][1][r])+"\t");
         Serial.println();
     }
}

void show(String msg) {
  Serial.println(String(encDEG(penPos))+"\t"+String(motorDEG())+"\t"+msg);
}

// =============================================================== OBSERVE


void observe() {

     // user must have placed pendulum manually on the left side at some
height
     // before calling this function

     Serial.println("observing");

     #define OBS_MAX 5
     long obs[8*OBS_MAX][3];    // bottom, right peek, bottom, left peek
--- time, pos, gap

     long penPos, lastPenPos;
     unsigned long now_, lastNow_, then_, start_=0;

     for (int o=0;o<8*OBS_MAX;) {
         penPos     = penPosition();
         now_    = micros();
         if (penPos==lastPenPos) continue;                // wait until
pendulum position changes
         // Serial.println(String(penPos));
         if (                                            // before snap
shot position
                 (o%8==0 && penPos==   -48) ||
                 (o%8==1 && penPos==  1392) ||
                 (o%8==3 && penPos==  1488) ||
                 (o%8==4 && penPos==    48) ||
                 (o%8==5 && penPos== -1392) ||
                 (o%8==7 && penPos== -1488)
             ) {
             then_ = now_;
         }
         else if (                                        // after snap
shot position
                 (o%8==0 && penPos==    48) ||
                 (o%8==1 && penPos==  1488) ||
                 (o%8==3 && penPos==  1392) ||
                 (o%8==4 && penPos==   -48) ||
                 (o%8==5 && penPos== -1488) ||
                 (o%8==7 && penPos== -1392)
             ) {
             if (start_==0) start_ = (now_+then_) / 2;
             obs[o][0] = (now_+then_) / 2 - start_;        //
observation time
             obs[o][1] = penPos + ((o%8>=3&&o%8<=5) ? 48 : -48);    //
observed position
             obs[o][2] = now_ - then_;                    // observed
"speed" for 24 pulses = 96 ticks
             o++;
         }
         else if (                                        // directly
after peek position
             (o%8==2 && penPos<lastPenPos) ||
             (o%8==6 && penPos>lastPenPos) ) {
             obs[o][0] = lastNow_ - start_;            // observation
time of peek
             obs[o][1] = lastPenPos;                    // peek position
             obs[o][2] = now_ - then_;                // "speed" for a
single pulse = 4 ticks
             o++;
             Serial.println("peek");
         }
         lastPenPos=penPos;
         lastNow_ = now_;
     }

     for (int o=0;o<8*OBS_MAX;o++) {
         Serial.println(
             String((obs[o][0]+500)/1000)+"\t"+
             String(obs[o][1])+"\t"+
             String(obs[o][2])
         );
     }
}
