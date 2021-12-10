#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>
#include <MultiStepper.h>


const unsigned int NUM_MOTORS = 4;
long pos_cmds[NUM_MOTORS];
long pos_curs[NUM_MOTORS];
double MAX_SPEED = 20.;

// ------------------------------------------------------------------------------
// Stepper Setup
// ------------------------------------------------------------------------------
Adafruit_MotorShield AFMS0 = Adafruit_MotorShield(0x60);
//Adafruit_MotorShield AFMS1 = Adafruit_MotorShield(0x61);

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port N
Adafruit_StepperMotor *ne = AFMS0.getStepper(200, 1);
Adafruit_StepperMotor *nw = AFMS0.getStepper(200, 2);
//Adafruit_StepperMotor *stepper2 = AFMS1.getStepper(200, 3);
//Adafruit_StepperMotor *stepper3 = AFMS1.getStepper(200, 4);

MultiStepper steppers;

void forwardstepne() {
  ne->onestep(FORWARD, DOUBLE);
}
void backwardstepne() {
  ne->onestep(BACKWARD, DOUBLE);
}
void forwardstepnw() {
  nw->onestep(FORWARD, DOUBLE);
}
void backwardstepnw() {
  nw->onestep(BACKWARD, DOUBLE);
}
//void forwardstepse() {
//  se->onestep(FORWARD, MICROSTEP);
//}
//void backwardstepse() {
//  se->onestep(BACKWARD, MICROSTEP);
//}
//void forwardstepsw() {
//  sw->onestep(FORWARD, MICROSTEP);
//}
//void backwardstepsw() {
//  sw->onestep(BACKWARD, MICROSTEP);
//}

AccelStepper Astepper0(forwardstepne, backwardstepne); // use functions to step
AccelStepper Astepper1(forwardstepnw, backwardstepnw);
//AccelStepper Astepper2(forwardstep2, backwardstep2);
//AccelStepper Astepper3(forwardstep3, backwardstep3);

void setup()
{
  Serial.begin(115200); // baud

  if (!AFMS0.begin(16000)) { // Hz
    while (1);
  }
  Serial.println("Motor Shield 0 init.");
//  if (!AFMS1.begin(16000)) { // Hz
//    while (1);
//  }
//  Serial.println("Motor Shield 1 init.");

  steppers.addStepper(Astepper0);
  steppers.addStepper(Astepper1);
//  steppers.addStepper(Astepper2);
//  steppers.addStepper(Astepper3);

  Astepper0.setMaxSpeed(MAX_SPEED);
  Astepper1.setMaxSpeed(MAX_SPEED);
//  Astepper2.setMaxSpeed(MAX_SPEED);
//  Astepper3.setMaxSpeed(MAX_SPEED);

  for(unsigned int i=0; i<NUM_MOTORS; ++i)
  {
    pos_cmds[i] = 0;
    pos_curs[i] = 0;
  }
}

void loop()
{
  while (Serial.available() > 0){
    // read in delta step commands from serial
    for(unsigned int i=0; i<NUM_MOTORS; ++i)
    {
      // ECM: note that it is possible for pos_curs to overflow if serial commands drive it 
      // in one direction long enough
      // I am ignoring it here because for this application, a physical limit for the
      //machine will be hit before enough steps are taken to overflow a long type
      pos_curs[i] += (long)Serial.parseInt();
      pos_cmds[i] = pos_curs[i];
    }
    if (Serial.read() == '\n')
    {
      for(unsigned int i=0; i<NUM_MOTORS; ++i)
      {
        Serial.print(pos_cmds[i]);
        Serial.print(" ");
      }
      Serial.print('\n');
      
      steppers.moveTo(pos_cmds);
      steppers.runSpeedToPosition();
    }
  }
}
