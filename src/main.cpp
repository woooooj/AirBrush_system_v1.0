#include <Arduino.h>
#include "SerialCommand.h"
SerialCommand sCmd;
int gg=40;
########################
########################
########################
########################
########################
########################

# some comment
# some comment 2

#define X_DIR_PIN          5
#define X_STEP_PIN         2
#define X_ENABLE_PIN       8
#define X_LIMIT            9

#define Y_DIR_PIN          6
#define Y_STEP_PIN         3
#define Y_ENABLE_PIN       8
#define Y_LIMIT            10

#define Z_DIR_PIN          7
#define Z_STEP_PIN         4
#define Z_ENABLE_PIN       8
#define Z_LIMIT            9

#define pinServo                11


#define X_STEP_HIGH        PORTD |=  0b00000100;
#define X_STEP_LOW         PORTD &= ~0b00000100;
#define Y_STEP_HIGH        PORTD |=  0b00001000;
#define Y_STEP_LOW         PORTD &= ~0b00001000;
#define Z_STEP_HIGH        PORTD |=  0b00010000;
#define Z_STEP_LOW         PORTD &= ~0b00010000;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);
#define NUM_STEPPERS 3
volatile byte remainingSteppersFlag = 0;
volatile byte nextStepperFlag = 0;
volatile int ind = 0;
volatile unsigned int intervals[100];

bool verbose = true;
bool FFirst = false;

struct stepperInfo {
  // externally defined parameters
  float acceleration;
  float SpeedSlow;
  float SpeedFast;
  volatile unsigned int minStepInterval;   // ie. max speed, smaller is faster
  void (*dirFunc)(int);
  void (*stepFunc)();

  // derived parameters
  unsigned int c0;                // step interval for first step, determines acceleration
  long stepPosition;              // current position of stepper (total of all movements taken so far)

  // per movement variables (only changed once per movement)
  volatile int dir;                        // current direction of movement, used to keep track of position
  volatile unsigned int totalSteps;        // number of steps requested for current movement
  volatile bool movementDone = false;      // true if the current movement has been completed (used by main program to wait for completion)
  volatile unsigned int rampUpStepCount;   // number of steps taken to reach either max speed, or half-way to the goal (will be zero until this number is known)

  // per iteration variables (potentially changed every interrupt)
  volatile unsigned int n;                 // index in acceleration curve, used to calculate next interval
  volatile float d;                        // current interval length
  volatile unsigned long di;               // above variable truncated
  volatile unsigned int stepCount;         // number of steps completed in current movement
  volatile int END_STOP_PIN;
  float RealPosition;
  float StepRatio;
  int MaxPosition;
};
volatile stepperInfo steppers[NUM_STEPPERS];

void xStep() {
  X_STEP_HIGH
  X_STEP_LOW
}
void xDir(int dir) {
  digitalWrite(X_DIR_PIN, dir);
}

void yStep() {
  Y_STEP_HIGH
  Y_STEP_LOW
}
void yDir(int dir) {
  digitalWrite(Y_DIR_PIN, dir);
}

void zStep() {
  Z_STEP_HIGH
  Z_STEP_LOW
}
void zDir(int dir) {
  digitalWrite(Z_DIR_PIN, dir);
}

void resetStepperInfo( stepperInfo& si ) {
  si.n = 0;
  si.d = 0;
  si.di = 0;
  si.stepCount = 0;
  si.rampUpStepCount = 0;
  si.totalSteps = 0;
  si.stepPosition = 0;
  si.movementDone = false;
}
void zero();
void amove();
void rmove();
void servo();
void paintt();

void setup() {
  Serial.begin(9600);
  FFirst = true;
  pinMode(X_STEP_PIN,   OUTPUT);
  pinMode(X_DIR_PIN,    OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(X_LIMIT, INPUT_PULLUP);

  pinMode(Y_STEP_PIN,   OUTPUT);
  pinMode(Y_DIR_PIN,    OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Y_LIMIT, INPUT_PULLUP);

  pinMode(Z_STEP_PIN,   OUTPUT);
  pinMode(Z_DIR_PIN,    OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);

  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);


  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 1000;                             // compare value
  TCCR1B |= (1 << WGM12);                   // CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));    // 64 prescaler
  interrupts();

  steppers[0].dirFunc = xDir;
  steppers[0].stepFunc = xStep;
  steppers[0].acceleration = 1000;
  steppers[0].minStepInterval = 100;
  steppers[0].END_STOP_PIN = X_LIMIT;
  steppers[0].RealPosition =0;
  steppers[0].StepRatio = 0.02;
  steppers[0].MaxPosition = 264; //mm

  steppers[1].dirFunc = yDir;
  steppers[1].stepFunc = yStep;
  steppers[1].acceleration = 1000;
  steppers[1].minStepInterval = 150;
  steppers[1].END_STOP_PIN = Y_LIMIT;
  steppers[1].RealPosition =0;
  steppers[1].StepRatio = 0.0025;
  steppers[1].MaxPosition = 110; //mm

  steppers[2].dirFunc = zDir;
  steppers[2].stepFunc = zStep;
  steppers[2].acceleration = 1000;
  steppers[2].minStepInterval = 450;
  steppers[2].END_STOP_PIN = X_LIMIT;
  steppers[2].RealPosition =0;
  steppers[2].StepRatio = 1;
  steppers[2].MaxPosition = 500; //mm

  sCmd.addCommand("zero",    zero);
  sCmd.addCommand("amove",   amove);
  sCmd.addCommand("rmove",   rmove);
  sCmd.addCommand("servo",   servo);
  sCmd.addCommand("paint",   paintt);


}


void resetStepper(volatile stepperInfo& si) {
  si.c0 = si.acceleration;
  si.d = si.c0;
  si.di = si.d;
  si.stepCount = 0;
  si.n = 0;
  si.rampUpStepCount = 0;
  si.movementDone = false;
}

void prepareMovement(int whichMotor, long steps) {
  volatile stepperInfo& si = steppers[whichMotor];
  steps = steps;
  si.dirFunc( steps < 0 ? HIGH : LOW );
  si.dir = steps > 0 ? 1 : -1;
  si.totalSteps = abs(steps);
  resetStepper(si);
  remainingSteppersFlag |= (1 << whichMotor);
}

void setNextInterruptInterval() {


  unsigned int mind = 999999;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di < mind ) {
      mind = steppers[i].di;
    }
  }

  nextStepperFlag = 0;
  for (int i = 0; i < NUM_STEPPERS; i++) {

    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di == mind )
      nextStepperFlag |= (1 << i);
  }

  if ( remainingSteppersFlag == 0 ) {
    OCR1A = 65500;
  }

  OCR1A = mind;
}

ISR(TIMER1_COMPA_vect){
  unsigned int tmpCtr = OCR1A;

  OCR1A = 65500;

  for (int i = 0; i < NUM_STEPPERS; i++) {

    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;

    if ( ! (nextStepperFlag & (1 << i)) ) {
      steppers[i].di -= tmpCtr;
      continue;
    }

    volatile stepperInfo& s = steppers[i];

    if ( s.stepCount < s.totalSteps ) {
      s.stepFunc();
      s.stepCount++;
      s.stepPosition += s.dir;
      s.RealPosition += s.dir*s.StepRatio;
      // Serial.print(s.END_STOP_PIN);
      // Serial.print(" :: ");
      // Serial.println(digitalRead(s.END_STOP_PIN));
      if ( s.stepCount >= s.totalSteps || digitalRead(s.END_STOP_PIN)==0) {
        s.movementDone = true;
        remainingSteppersFlag &= ~(1 << i);
      }
    }

    if ( s.rampUpStepCount == 0 ) {
      s.n++;
      s.d = s.d - (2 * s.d) / (4 * s.n + 1);
      if ( s.d <= s.minStepInterval ) {
        s.d = s.minStepInterval;
        s.rampUpStepCount = s.stepCount;
      }
      if ( s.stepCount >= s.totalSteps / 2 ) {
        s.rampUpStepCount = s.stepCount;
      }
    }
    else if ( s.stepCount >= s.totalSteps - s.rampUpStepCount ) {
      s.d = (s.d * (4 * s.n + 1)) / (4 * s.n + 1 - 2);
      s.n--;
    }

    s.di = s.d; // integer
  }

  setNextInterruptInterval();

  TCNT1  = 0;
}

void runAndWait() {
  setNextInterruptInterval();
  while ( remainingSteppersFlag );
}

void MoveServo(int x){
    int del=(11*(180-x))+500;
    for (int pulseCounter=0; pulseCounter<=50; pulseCounter++){
        digitalWrite(pinServo,HIGH);
        delayMicroseconds(del);
        digitalWrite(pinServo,LOW);
        delay(20); // between pulses
    }
}

void MotorMove(int whichMotor,long steps){
  // Serial.print("\nMotorMove :: ");
  // Serial.print(whichMotor);
  // Serial.print(" :: ");
  // Serial.print(steps);

  volatile stepperInfo& si = steppers[whichMotor];
  prepareMovement(whichMotor,steps);
  setNextInterruptInterval();
  while ( remainingSteppersFlag );

  // Serial.print(":: MOTOR: "); Serial.print(whichMotor);
  // Serial.print(":: new pos.: "); Serial.print(si.RealPosition);
  // Serial.print(" ["); Serial.print(si.stepPosition);
  // Serial.println("]");

}

void MotorMove_ABS( int whichMotor,int newposition){
  volatile stepperInfo& si = steppers[whichMotor];

  if( newposition <= si.MaxPosition  ){
      MotorMove(whichMotor,(newposition-si.RealPosition)/si.StepRatio);
    } else{
      Serial.print("\n:: ERR: "); Serial.println(whichMotor);
      Serial.print(" :: ERR: "); Serial.print(newposition);Serial.print(" >> "); Serial.println(si.RealPosition );
    }
    if(verbose){
      Serial.print("\nMotor :: ");
      Serial.print(whichMotor);
      Serial.print(" Position :: ");
      Serial.print(si.RealPosition);
    }
}

void MotorMove_REL( int whichMotor,float newposition){
  volatile stepperInfo& si = steppers[whichMotor];
  if( newposition+si.RealPosition <= si.MaxPosition  ){
      MotorMove(whichMotor,(newposition/si.StepRatio) );
    } else{
      Serial.print("\n:: ERR: "); Serial.println(whichMotor);
      Serial.print(" :: ERR: "); Serial.print(newposition);Serial.print(" >> "); Serial.println(si.RealPosition );
    }
    if(verbose){
      Serial.print("\nMotor :: ");
      Serial.print(whichMotor);
      Serial.print(" Position :: ");
      Serial.print(si.RealPosition);    Serial.print(" mm ");
    }


}

void FindZero(int whichMotor){
  volatile stepperInfo& si = steppers[whichMotor];
  while(digitalRead(si.END_STOP_PIN)==1) MotorMove(whichMotor,-5000);
  while(digitalRead(si.END_STOP_PIN)!=1) MotorMove(whichMotor,1);
  si.stepPosition = 0;
  si.RealPosition = 0;
  Serial.println(">>> zero <<<");
}

void Paint(long X_start, int X_stop,int multi=1){
  volatile stepperInfo& s1 = steppers[0];
  volatile stepperInfo& s3 = steppers[2];
  verbose = false;

  FindZero(0);
  MoveServo(45);
  delay(200);

  MotorMove_ABS(0,X_start);
  prepareMovement(2,gg);
  while(multi>=1){
    MotorMove_ABS(0,X_stop);
    MotorMove_ABS(0,X_start);
    multi--;
  }

  delay(100);

  prepareMovement(2,-1*gg);
  MotorMove_ABS(0,0.1);

  delay(200);
  MoveServo(0);
  delay(200);
  MoveServo(0);

  verbose = true;
}

void zero(){
  char *arg;
  Serial.println("\n >>> ZERO <<< \n");
  arg = sCmd.next();
  Serial.println(atoi(arg));
  FindZero(atoi(arg));
}

void amove(){
  int arg1;
  long arg2;
  Serial.println("\n >>> A_MOVE <<< \n");
  arg1 = atoi(sCmd.next());
  arg2 = atol(sCmd.next());
  Serial.println(arg1);
  Serial.println(arg2);
  MotorMove_ABS(arg1,arg2);
}
void rmove(){
  int arg1;
  long arg2;
  Serial.println("\n >>> R_MOVE <<< \n");
  arg1 = atoi(sCmd.next());
  arg2 = atol(sCmd.next());
  Serial.println(arg1);
  Serial.println(arg2);
  MotorMove_REL(arg1,arg2);
}

void servo(){
  Serial.println("\n >>> SERVO <<< \n");
  int arg1 = atoi(sCmd.next());
  int arg2 = atol(sCmd.next());
  MoveServo(arg1);
}

void paintt(){
  Serial.println("\n >>> PAINT <<< \n");
  int arg1 = atoi(sCmd.next());
  int arg2 = atol(sCmd.next());
  Paint(arg1,arg2);

}

int one = 0;

void loop() {
  TIMER1_INTERRUPTS_ON


  sCmd.readSerial();

  if (FFirst){
    Paint(70,230,1);
    delay(5*1000);
    Paint(70,230,3);
    FFirst=false;
    delay(100*1000);
  }

  // int w = 110;
  // FindZero(1);
  //
  // if (one == 0){
  //   one = 100;
  //   while(w>=10){
  //     MotorMove_ABS(1,w);
  //     gg=45;
  //     Paint(50,150,1);
  //     delay(1000);
  //     w-=25;
  //   }

    // gg=20;
    // Paint(50,150);
    // delay(1000);
    //
    // gg=25;
    // Paint(50,150);
    // delay(1000);
    //
    // gg=30;
    // Paint(50,150);
    // delay(1000);
    //
    // gg=35;
    // Paint(50,150);
    // delay(1000);
    //
    // gg=40;
    // Paint(50,150);
    // delay(1000);
    //
    // gg=45;
    // Paint(50,150);
    // delay(1000);
    //
    // gg=50;
    // Paint(50,150);
    // delay(1000);
    //
    // gg=55;
    // Paint(50,150);
    // delay(1000);
    //
    // gg=60;
    // Paint(50,150);
    // delay(1000);
    //
    // gg=65;
    // Paint(50,150);
    // delay(1000);
    //
    // gg=70;
    // Paint(50,150);
    // delay(1000);


  // }

  // FindZero(0);
  // FindZero(1);
  // FindZero(2);
  // Paint(100,200);
  //
  // MotorMove_ABS(0,100);
  // MotorMove_ABS(0,200);
  // MotorMove_ABS(0,100);
  // MotorMove_ABS(0,50);
  //   MotorMove_ABS(0,500);
  // MotorMove_ABS(1,50);


  // MoveServo(0);
  // delay(20);
  // MoveServo(20);
  // delay(20);
  // MoveServo(0);
  // delay(20);
  // MoveServo(40);
  // delay(20);
  // // MotorMove(0,5000);
  // MotorMove(0,-50);
  // MotorMove(1,5000);
  // MotorMove(1,-50);
  // FindZero(0);
  // FindZero(1);
  //
  // MotorMove_REL(0,150);


  // Paint(50,260);

  // MoveServo(20);
  // MoveServo(0);
  // MotorMove_REL(0,150);
  //   MoveServo(20);
  // MotorMove_REL(1,10);
  //   MoveServo(0);
  // MotorMove_REL(0,10);
  //   MoveServo(20);
  // MotorMove_REL(1,5);

  // while (true);

}
