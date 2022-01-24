#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// ------------------------------------------------------------------------------
// Stepper Setup
// ------------------------------------------------------------------------------
const unsigned int NUM_MOTORS = 1;
long MAX_SPEED = 2000L;

Adafruit_MotorShield AFMS0 = Adafruit_MotorShield(0x60);
// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port N
Adafruit_StepperMotor *afStepper = AFMS0.getStepper(200, 1);

// with microstepping, 200 * 16 = 3200 steps per revolution
void forwardStep()
{
  afStepper->onestep(BACKWARD, MICROSTEP);
}
void backwardStep()
{
  afStepper->onestep(FORWARD, MICROSTEP);
}

AccelStepper rot(forwardStep, backwardStep); // use functions to step

// steps between measurements
unsigned long steps = 800U;
long steps_taken = 0;

// ------------------------------------------------------------------------------
// distance sensor setup
// ------------------------------------------------------------------------------
const int pingPin = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 6; // Echo Pin of Ultrasonic Sensor

// speed of sound at 74 F
const double c = 345.09; // m/s

// buffer measurements for rejection
const unsigned int BUFSIZE = 20U;
double buf[BUFSIZE] = {100.}; // init to full of 100 m
unsigned int i = 0;
double avg = 0.;
double stddev = 0.;

// a buffer for data take commands
const byte numChars = 3;
char receivedChars[numChars];
bool newData = false;

// https://forum.arduino.cc/t/serial-input-basics-updated/382007/2
void recvWithStartEndMarkers()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();

      if (recvInProgress == true) {
          if (rc != endMarker) {
              receivedChars[ndx] = rc;
              ndx++;
              if (ndx >= numChars) {
                  ndx = numChars - 1;
              }
          }
          else {
              receivedChars[ndx] = '\0'; // terminate the string
              recvInProgress = false;
              ndx = 0;
              newData = true;
          }
      }

      else if (rc == startMarker) {
          recvInProgress = true;
      }
  }
}

double microsec_to_m(double microseconds){
  return c * (microseconds / 1000000. / 2.);
}

void setup() {
    Serial.begin(115200); // Starting Serial Terminal

    // Stepper init
    if (!AFMS0.begin(12000)) // Hz
    {
      while (1);
    }
//    Serial.println("Motor Shield 0 init.");
    rot.setAcceleration(2000.);
    rot.setCurrentPosition(0L);
    rot.setMaxSpeed(MAX_SPEED);
}

void loop() {
    // continuously buffer distance measurements
    double duration, m;
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pingPin, LOW);
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
   
    m = microsec_to_m(duration);
    Serial.print(m * 100., 6);
    Serial.print("\n");
    buf[i] = m;
    ++i;
    i %= BUFSIZE;
    
    // check for human input
    recvWithStartEndMarkers();
    if (newData){
        // take data: average the last 100 measurements
        double sum = 0.;
        for(unsigned int j=0U; j<BUFSIZE; ++j){
            sum += buf[j];
        }
        avg = sum / BUFSIZE;

        // estimate the variance
        double sqerrs = 0.;
        for(unsigned int j=0U; j<BUFSIZE; ++j){
            sqerrs += pow((buf[j] - avg), 2.);
        }
        stddev = sqrt(sqerrs / BUFSIZE);

//        Serial.print("steps taken:");
//        Serial.print(steps_taken);
//        Serial.print(",");
//        Serial.print("distance:");
        Serial.print(avg, 6);
        Serial.print(",");
//        Serial.print("std:");
//        Serial.print(stddev, 6);
        Serial.println();

        newData = false;
        
        unsigned int i = 0U;
        // determine which direction the motor should move
        if(!strcmp(receivedChars, "F")){ // move motor fwd: play out string
            rot.move(steps);
            rot.runToPosition();
            steps_taken += steps;
        }
        else{ // move motor in reverse: spool up string
            rot.move(-1 * steps);
            rot.runToPosition();
            steps_taken -= steps;
        }
    }
    delay(30);
}
