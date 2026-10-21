/***************************************************************************
*   Copyright (C) 2024 by DTU
*   jcan@dtu.dk
*
*   Base Teensy firmware
*   build for Teensy 4.1,
*   intended for digital control course
*
* The MIT License (MIT)  https://mit-license.org/
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

void printLog();

#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <IntervalTimer.h>

// Support functions
#include "src/udisplay.h"
#include "src/urobot.h"
#include "src/uusb.h"
#include "src/umotor.h"
#include "src/uimu2.h"
#include "src/uencoder.h"


/**
 * Global variables
 */

// Sample time can not go lower than 300us
const uint32_t sampleTimeUs = 500; // desired sample time in us - CHANGED
const float ts = sampleTimeUs * 1e-6; // sample time in seconds

// Robot configuration
const float gear = 9.6;
// const float wheelRadius = 0.0315; // (m)
const float wheelRadius = 0.03380; // calibrated effective radius (m) - needed to
                                   // accomplish the physical desired distance - MIGHT NEED A RECHECK
const float wheelBase = 0.147; // (m)

// Controller Gains
const float Kp   = 0.08f;
const float taui = 0.4f;
const float Ki   = Kp / taui;
const float voltageMax = 9.0f;

// Turn-rate PI gains
const float KpTurn = 5.0f;      // To be tuned
const float tauiTurn = 0.2f;   // To be tuned
const float KiTurn = KpTurn / tauiTurn;

const float turnrateMax = 4.0f; // rad/s - To be checked

struct Integrator
{
  float value;
  float previousInput;
};

Integrator distanceIntegral = {0, 0};
Integrator tripIntegral = {0, 0};
Integrator headingIntegral = {0, 0};
Integrator xIntegral = {0, 0};
Integrator yIntegral = {0, 0};

Integrator leftPI  = {0, 0};
Integrator rightPI = {0, 0};

Integrator turnratePI = {0, 0};

// pose values
float distA = 0; // trip distance
float pose[4]     = {0, 0, 0, 0}; // x, y, heading, signed distance
float poseVel[2]  = {0, 0}; // forward velocity, turn rate
float wheelVel[2] = {0, 0}; // left and right wheel linear velocities
bool velSaturated = false;
bool firstPoseSample = true;
bool piControlActive = false;


// ////////////////////////////////////////
// define a structure for log elements

typedef struct
{
  float time;
  int state;
  float desiredValue;
  float desiredHeading;
  float desiredTurnrate;
  float motorVel[2];
  float motorVolt[2];
  int32_t encoder[2];
  float motorCurrent[2];
  float battery;
  float pose[4];
  float distA;
  float poseVel[2];
  float wheelVel[2];
  float inVelLeft;
  float inVelRight;

  // Gyroscope data:
  // gyro[0], gyro[1], gyro[2]
  float gyro[3];

  float headU;
  float accVel;
  float accTurn;

} LogData;


// allocate memory for data log in memory block 2 available only through malloc/new
// (specific for this micro processor, a Teensy 4)
const int logsMax = 400000 / sizeof(LogData); // allocates 400kBytes
LogData * logs = (LogData*)malloc(logsMax * sizeof(LogData));
int logsCnt = 0; // next log entry to use / i.e. number of used log entries
const int logEvery = 10; // use logEvery = 1 for the initial steps and spikes experiment
int logSampleCounter = 0;


void printLog()
{
  LogData * d = logs;

  Serial.print("% Basebot log for ");
  Serial.println(robot.getRobotName());

  // hard-coded settings
  Serial.print("% Sample time ");
  Serial.print(sampleTimeUs);

  Serial.print(" us, Gear ");
  Serial.print(gear);

  Serial.print(", Wheels ");
  Serial.print(wheelRadius);

  Serial.print(" m, PPR ");
  Serial.println(encoder.pulsPerRev);

  Serial.print("% maximum log samples ");
  Serial.print(logsMax);

  Serial.print(" each sized ");
  Serial.print(sizeof(LogData));
  Serial.println(" bytes");

  if (piControlActive)
    Serial.println("% Mode: wheel PI; column 3 is velocity reference (m/s)");
  else
    Serial.println("% Mode: direct voltage; column 3 is voltage reference (V)");

  Serial.print("% Nominal log interval (s): ");
  Serial.println(logEvery * ts, 4);


 // Columns

  Serial.println("% 1 time (sec)");
  Serial.println("% 2 state");
  Serial.println("% 3 Desired value");

  Serial.println("% 4-5 motor voltage (left,right) (V)");
  Serial.println("% 6-7 Encoder count (left,right)");
  Serial.println("% 8-9 motor vel (left,right) (rad/s)");

  Serial.println("% 10-11 motor current (left,right) (A)");

  Serial.println("% 12 Battery voltage (V)");

  // gyro columns
  Serial.println("% 13 gyro[0]");
  Serial.println("% 14 gyro[1]");
  Serial.println("% 15 gyro[2]");

  Serial.println("% 16 Turnrate reference (rad/s)");
  Serial.println("% 17 Forward velocity (m/s)");
  Serial.println("% 18 Turnrate (rad/s)");
  Serial.println("% 19 x (m)");
  Serial.println("% 20 y (m)");
  Serial.println("% 21 Heading (rad)");
  Serial.println("% 22 Signed distance (m)");
  Serial.println("% 23 Total distance travelled (m)");
  Serial.println("% 24-25 Wheel velocity (left,right) (m/s)");
  Serial.println("% 26-27 Wheel velocity reference (left,right) (m/s)");
  Serial.println("% Columns 16 and 26-27 are motion references for PI mode only");


  for (int i = 0; i < logsCnt; i++)
  {
    Serial.print(d->time,4);                            // 1
    Serial.print(" ");
    Serial.print(d->state);                             // 2

    Serial.print(" ");
    Serial.print(d->desiredValue,2);                    // 3

    Serial.print(" ");
    Serial.print(d->motorVolt[0],2);                    // 4 (V)

    Serial.print(" ");
    Serial.print(d->motorVolt[1],2);                    // 5 (V)

    Serial.print(" ");
    Serial.print(d->encoder[0]);                        // 6

    Serial.print(" ");
    Serial.print(d->encoder[1]);                        // 7

    Serial.print(" ");
    Serial.print(d->motorVel[0],2);                     // 8 (rad/s)

    Serial.print(" ");
    Serial.print(d->motorVel[1],2);     // 9 (rad/s)

    //ADDED CURRENT VALUES (4 LINES LEFT AND RIGHT)
    Serial.print(" ");
    Serial.print(d->motorCurrent[0],4);

    Serial.print(" ");
    Serial.print(d->motorCurrent[1],4);

    Serial.print(" ");
    Serial.print(d->battery,2);                         // 10 (V)


    // ADDED: print gyro measurements
    Serial.print(" ");
    Serial.print(d->gyro[0],4);                         // 11

    Serial.print(" ");
    Serial.print(d->gyro[1],4);                         // 12

    Serial.print(" ");
    Serial.print(d->gyro[2],4);                         // 13

    Serial.print(" ");
    Serial.print(d->desiredTurnrate, 5);

    for (int j = 0; j < 2; j++)
    {
      Serial.print(" ");
      Serial.print(d->poseVel[j],5);
    }

    for (int j = 0; j < 4; j++)
    {
      Serial.print(" ");
      Serial.print(d->pose[j],5);
    }

    Serial.print(" ");
    Serial.print(d->distA,5);

    for (int j = 0; j < 2; j++)
    {
      Serial.print(" ");
      Serial.print(d->wheelVel[j],5);
    }

    Serial.print(" ");
    Serial.print(d->inVelLeft,5);

    Serial.print(" ");
    Serial.print(d->inVelRight,5);

    Serial.println("");

    d++;
  }

  Serial.print("% sent ");
  Serial.print(logsCnt);
  Serial.println(" log lines");
}


// ////////////////////////////////////////

/**
 * Global variables for timing
 */

bool isSampleTime = false;
float time_sec = 0;

// Timer that controls sample time (interrupt based)
IntervalTimer sampleTimer;


void timerInterrupt()
{
  isSampleTime = true;
  time_sec += sampleTimeUs / 1e6;
}


/**
 * declaration, that there is a reset function
 */
void reset();


/**
 * Global variables for state machine
 */

int state = 0;               // actual sequence state
float endTime = 0;           // for current state
float desiredValue = 0;      // desired (reference) value send to controller
float desiredHeading = 0;    // rad

float desiredTurnrate = 0; // rad/s
float inVelLeft = 0;       // m/s
float inVelRight = 0;      // m/s

float integrate(Integrator &memory, float input, float dt)
{
  memory.value += 0.5f * dt * (input + memory.previousInput);
  memory.previousInput = input;
  return memory.value;
}

float wheelPI(Integrator &memory, float error)
{
  const float oldIntegral = memory.value;

  integrate(memory, error, ts);

  float voltage = Kp * error + Ki * memory.value;

  // Check if this is really necessary. It is used to prevent anti-windup
  const float integralChange = Ki * (memory.value - oldIntegral);

  if ((voltage > voltageMax && integralChange > 0) || (voltage < -voltageMax && integralChange < 0))
  {
    memory.value = oldIntegral;
    voltage = Kp * error + Ki * memory.value;
  }

  return fmaxf(-voltageMax, fminf(voltageMax, voltage));

}

float turnrateControl(float error) // Because it only has one Integrator variable to control - Integrator turnratePI
{
  const float oldIntegral = turnratePI.value;

  integrate(turnratePI, error, ts);

  float turnrateCommand = KpTurn * error + KiTurn * turnratePI.value;

  // Check if this is really necessary. It is used to prevent anti-windup
  const float integralChange = Ki * (turnratePI.value - oldIntegral);

  if ((turnrateCommand > turnrateMax && integralChange > 0) || (turnrateCommand < -turnrateMax && integralChange < 0))
  {
    turnratePI.value = oldIntegral;
    turnrateCommand = KpTurn * error + KiTurn * turnratePI.value;
  }

  return fmaxf(-turnrateMax, fminf(turnrateMax, turnrateCommand));

}

void updatePose()
{
  wheelVel[0] = -encoder.motorVelocity[0] * wheelRadius / gear;
  wheelVel[1] =  encoder.motorVelocity[1] * wheelRadius / gear;

  poseVel[0] = 0.5f * (wheelVel[0] + wheelVel[1]);
  poseVel[1] = (wheelVel[1] - wheelVel[0]) / wheelBase;

  if (firstPoseSample)
  {
    distanceIntegral.previousInput = poseVel[0];
    tripIntegral.previousInput = fabsf(poseVel[0]);
    headingIntegral.previousInput = poseVel[1];
    xIntegral.previousInput = poseVel[0];
    yIntegral.previousInput = 0;

    firstPoseSample = false;
    return;
  }

  pose [2] = integrate(headingIntegral, poseVel[1], ts);

  pose[3] = integrate(distanceIntegral, poseVel[0], ts);
  distA   = integrate(tripIntegral, fabsf(poseVel[0]), ts);

  pose[0] = integrate(xIntegral, poseVel[0] * cosf(pose[2]), ts);
  pose[1] = integrate(yIntegral, poseVel[0] * sinf(pose[2]), ts);
}

void resetStates()
{
  distanceIntegral = {0, 0};
  tripIntegral = {0, 0};
  headingIntegral = {0, 0};
  xIntegral = {0, 0};
  yIntegral = {0, 0};

  for (int i = 0; i < 4; i++)
    pose[i] = 0;

  for (int i = 0; i < 2; i++)
  {
    poseVel[i] = 0;
    wheelVel[i] = 0;
  }

  distA = 0;
  firstPoseSample = true;

  leftPI  = {0, 0};
  rightPI = {0, 0};

  turnratePI = {0, 0};
}

void start()
{
  // Start timing

  // reset log and encoders
  logsCnt = 0;
  logSampleCounter = 0;

  resetStates();

  // change motor PWM frequency for sampling time test
  // should not be above 100000 (100kHz), default is 80kHz.
  motor.setPWMfrq(80000);

  time_sec = 0;

  encoder.encoder[0] = 0; // left motor encoder
  encoder.encoder[1] = 0; // right motor encoder

  // Update of display takes too long for fast sampling
  // so, disable during sequence.
  display.useDisplay = false;

  desiredTurnrate = 0;

}



void stop(float after)
{
  // stop, but allow continued logging for a while
  state = 2;
  desiredValue = 0;
  desiredTurnrate = 0;  
  // end log after an additional short time
  endTime = time_sec + after;
}



void finished()
{
  // prepare for a new start

  // stop motors
  motor.motorVoltage[0] = 0; // left motor
  motor.motorVoltage[1] = 0; // right motor

  // set default PWM frequency
  motor.setPWMfrq(80000);

  // start updating display
  display.useDisplay = true;

  // wait for next button press
  state = 0;

  Serial.print("% Finished. Saved samples: ");
  Serial.println(logsCnt);

  printLog();
}



/**
 * Simple 2-step sequence
 */

void sequenceTwoSteps()
{
  // this function is called at every sample time
  // and should never wait in a loop.
  // Update variables as needed and return.

  bool button;

  //
  // this is a state machine
  // state 0: wait for start button press
  // other states are part of a sequence

  switch (state)
  {
    // run mission, initial value
    case 0:
      // State 0 is just inactive, waiting for the start signal.

      button = digitalReadFast(PIN_START_BUTTON);

      if (button or robot.missionStart)
      {
        // starting by a short time to get zero velocity data to the log

        // reset for new run
        start();

        // Prepare next state
        desiredValue = 0; // reference value to the controller

        // to get start the log with no velocity
        endTime = time_sec + 0.020; // new state to end after 20ms

        state = 10;
      }

      break;


    case 10:
      // Waiting for first step (zero velocity to log) to finish.

      if (time_sec > endTime)
      {
        // change to next values - drive
        desiredValue = 1.5;
        // should be in meters/sec, but is motor voltage for now,

        endTime = time_sec + 0.50; //0.5 second

        state = 11;
      }

      break;


    case 11:
      // First step

      // test if ready for next state
      if (time_sec > endTime)
      {
        // change to next values
        desiredValue = 3;

        endTime = time_sec + 0.50;

        state = 12;
      }

      break;


    case 12:
      // Second step

      if (time_sec > endTime)
      {
        // change to next values
        desiredValue = 6;

        endTime = time_sec + 0.5;

        state = 100;
      }

      break;


    case 100:
      // Maintain last step

      // stop after end time
      if (time_sec > endTime)
      {
        // stop, but continue logging for a while (0.3 sec)
        stop(0.3);
      }

      break;


    default:
      // Hold until finished

      // this state is needed to enable a new start
      if (time_sec > endTime)
      {
        finished();
      }

      break;
  }
}


//ADDED
void sequenceSpikes()
{
  // this function is called at every sample time
  // and should never wait in a loop.
  // Update variables as needed and return.

  bool button;

  switch (state)
  {
    case 0: // Inactive, waiting for start signal
      button = digitalReadFast(PIN_START_BUTTON);

      if (button or robot.missionStart)
      {
        // Reset for new run
        start();

        // seq2 at t = 0.00 s
        desiredValue = 0.0;

        // Next change at t = 0.05 s
        endTime = time_sec + 0.05;

        state = 10;
      }
      break;


    case 10: // t = 0.00 -> 0.05 s, desiredValue = 0
      if (time_sec > endTime)
      {
        // seq2 at t = 0.05 s
        desiredValue = 6.0;

        // Next change at t = 0.06 s
        endTime = time_sec + 0.01;

        state = 11;
      }
      break;


    case 11: // t = 0.05 -> 0.06 s, desiredValue = 6
      if (time_sec > endTime)
      {
        // seq2 at t = 0.06 s
        desiredValue = 0.0;

        // Next change at t = 0.30 s
        endTime = time_sec + 0.24;

        state = 12;
      }
      break;


    case 12: // t = 0.06 -> 0.30 s, desiredValue = 0
      if (time_sec > endTime)
      {
        // seq2 at t = 0.30 s
        desiredValue = 6.0;

        // Next change at t = 0.31 s
        endTime = time_sec + 0.01;

        state = 13;
      }
      break;


    case 13: // t = 0.30 -> 0.31 s, desiredValue = 6
      if (time_sec > endTime)
      {
        // seq2 at t = 0.31 s
        desiredValue = 0.0;

        // Next change at t = 0.60 s
        endTime = time_sec + 0.29;

        state = 14;
      }
      break;


    case 14: // t = 0.31 -> 0.60 s, desiredValue = 0
      if (time_sec > endTime)
      {
        // seq2 at t = 0.60 s
        desiredValue = 6.0;

        // Next change at t = 0.61 s
        endTime = time_sec + 0.01;

        state = 15;
      }
      break;


    case 15: // t = 0.60 -> 0.61 s, desiredValue = 6
      if (time_sec > endTime)
      {
        // seq2 at t = 0.61 s
        desiredValue = 0.0;

        // Continue logging for 0.3 s
        stop(0.3);
      }
      break;


    default: // Hold until finished
      if (time_sec > endTime)
        finished();

      break;
  }
}

void sequenceVelocityTest()
{
  switch (state)
  {
    case 0:
      if (digitalReadFast(PIN_START_BUTTON) || robot.missionStart)
      {
        start();

        desiredValue = 0;
        desiredTurnrate = 0;

        endTime = time_sec + 0.1f;
        state = 10;
      }
      break;

    case 10:
      if (time_sec >= endTime)
      {
        desiredValue = 0.30f;
        desiredTurnrate = 0;

        endTime = time_sec + 3.0f;
        state = 11;
      }
      break;

    case 11:
      if (time_sec >= endTime)
      {
        stop(0.5f);
      }
      break;

    case 2:
      if (time_sec >= endTime)
      {
        finished();
      }
      break;
  }
}

void sequenceTurnrateTest()
{
  switch (state)
  {
    case 0:
      if (digitalReadFast(PIN_START_BUTTON) || robot.missionStart)
      {
        start();

        desiredValue = 0.0f;
        desiredTurnrate = 0.0f;

        endTime = 0.5f;
        state = 10;
      }
      break;

    case 10:
      if (time_sec >= endTime)
      {
        desiredValue = 0.3f;
        desiredTurnrate = 0.0f;

        endTime = 2.0f;
        state = 11;
      }
      break;

    case 11:
      if (time_sec >= endTime)
      {
        desiredTurnrate = 0.5f;

        endTime = 4.0f;
        state = 12;
      }
      break;

    case 12:
      if (time_sec >= endTime)
      {
        desiredTurnrate = 0.0f;

        endTime = 5.0f;
        state = 13;
      }
      break;

    case 13:
      if (time_sec >= endTime)
      {
        desiredTurnrate = -0.5f;

        endTime = 7.0f;
        state = 14;
      }
      break;

    case 14:
      if (time_sec >= endTime)
      {
        desiredTurnrate = 0.0f;

        endTime = 8.0f;
        state = 15;
      }
      break;

    case 15:
      if (time_sec >= endTime)
      {
        desiredTurnrate = 1.0f;

        endTime = 10.0f;
        state = 16;
      }
      break;

    case 16:
      if (time_sec >= endTime)
      {
        desiredTurnrate = 0.0f;

        endTime = 11.0f;
        state = 17;
      }
      break;

    case 17:
      if (time_sec >= endTime)
      {
        stop(1.0f);
      }
      break;

    case 2:
      if (time_sec >= endTime)
      {
        finished();
      }
      break;
  }
}


/**
 * make the control
 */

void controlUpdateVoltage() // For the straightforward sequencetwosteps or sequencespikes
{
  piControlActive = false;

  // do control during a mission only.
  // called at every tick

  float velRef = desiredValue;

  //
  // For a start, the desiredValue
  // is just used as motor voltage
  //

  // Left motor voltage (-9V to +9V)
  motor.motorVoltage[0] = -velRef;

  // Right motor voltage (-9V to +9V)
  motor.motorVoltage[1] = velRef;

  //
  // in the main loop, there is a motor.tick() that actually send the
  // values to the motor.
}

void controlUpdatePI() // For fancy trajectory missions
{
  piControlActive = true;

  const float measuredTurnrate = poseVel[1];
  const float turnrateError = desiredTurnrate - measuredTurnrate;

  const float turnrateCommand = turnrateControl(turnrateError);

  inVelLeft  = desiredValue - 0.5f * wheelBase * turnrateCommand;
  inVelRight = desiredValue + 0.5f * wheelBase * turnrateCommand;

  const float omegaRefLeft  = inVelLeft  * gear / wheelRadius;
  const float omegaRefRight = inVelRight * gear / wheelRadius;

  const float omegaLeft  = -encoder.motorVelocity[0];
  const float omegaRight =  encoder.motorVelocity[1];

  const float errorLeft  = omegaRefLeft  - omegaLeft;
  const float errorRight = omegaRefRight - omegaRight;

  const float voltageLeft  = wheelPI(leftPI,  errorLeft);
  const float voltageRight = wheelPI(rightPI, errorRight);

  motor.motorVoltage[0] = -voltageLeft;
  motor.motorVoltage[1] =  voltageRight;
}


void updateLog()
{
  // step response mode

  if (state <= 0)
    return;

  if (logSampleCounter++ % logEvery != 0)
    return;

  if (state > 0 and logsCnt < logsMax)
  {
    // save data for this sample into dataLog structure

    logs[logsCnt].time = time_sec;

    logs[logsCnt].state = state;

    logs[logsCnt].desiredValue = desiredValue;

    logs[logsCnt].motorVolt[0] = -motor.motorVoltage[0];
    // The left motor runs backwards, therefore the sign.

    logs[logsCnt].motorVolt[1] = motor.motorVoltage[1];

    logs[logsCnt].encoder[0] = -encoder.encoder[0];

    logs[logsCnt].encoder[1] = encoder.encoder[1];

    logs[logsCnt].motorVel[0] = -encoder.motorVelocity[0];

    logs[logsCnt].motorVel[1] = encoder.motorVelocity[1];

    logs[logsCnt].battery = robot.batteryVoltage;


    // the last part is offset of current measurement (in Amps)
    // take the initial current readings and add or subtract as needed
    // these values fits Leonora

    logs[logsCnt].desiredHeading = desiredHeading;

    logs[logsCnt].motorCurrent[0] =
        -(analogRead(A1) - 2048) * 0.0045 + 0.3195;

    logs[logsCnt].motorCurrent[1] =
         (analogRead(A0) - 2048) * 0.0045 - 0.3465;

    logs[logsCnt].poseVel[0] = poseVel[0];

    logs[logsCnt].poseVel[1] = poseVel[1];

   //saving the current in logData

    logs[logsCnt].gyro[0] = imu2.gyro[0];
    logs[logsCnt].gyro[1] = imu2.gyro[1];
    logs[logsCnt].gyro[2] = imu2.gyro[2];


    // add more items as needed, NB! also the struct definition at top.

    logs[logsCnt].desiredTurnrate = desiredTurnrate;

    for (int i = 0; i < 4; i++)
    {
      logs[logsCnt].pose[i] = pose[i];
    }

    logs[logsCnt].distA = distA;

    logs[logsCnt].wheelVel[0] = wheelVel[0];
    logs[logsCnt].wheelVel[1] = wheelVel[1];

    logs[logsCnt].inVelLeft = inVelLeft;
    logs[logsCnt].inVelRight = inVelRight;

    logsCnt++;
  }
}



void setup()
{
  // INITIALIZATION

  // to be able to print to USB interface
  Serial.begin(12000000);

  sampleTimer.begin(timerInterrupt, sampleTimeUs);

  // initialize sensors, safety and display
  robot.setup(); // alive LED, display and battery

  motor.setup(); // motor voltage

  encoder.setup(); // motor encoders to velocity

  encoder.pulsPerRev = 48;

  imu2.setup(); // gyro and accelerometer

  motor.setPWMfrq(0x13000); // 0x13000 = 77824 Hz

  usb.setup();

  display.useDisplay = true;
}



/**
 * Main loop
 */

void loop(void)
{
  // init sample time

  while (true)
  {
    // main loop

    // loop until time for next sample
    if (isSampleTime)
    {
      // start of new control cycle

      isSampleTime = false;

      // read sensors
      imu2.tick();
      encoder.tick();

      // Select one sequence:
      // sequenceTwoSteps();        // Original voltage steps
      // sequenceVelocityTest();    // Velocity reference test
      sequenceTurnrateTest();    // Turnrate reference test


      if (state > 0)
      {
        // Only if started
        updatePose();

        // Select the matching control:
        // controlUpdateVoltage(); // Direct voltage
        controlUpdatePI();      // Wheel PI control
      }

      // give value to actuators
      motor.tick();

      // save relevant values
      updateLog();

      // support functions
      robot.tick();   // measure battery voltage etc.

      display.tick(); // update O-LED display
    }

    usb.tick(); // listen to incoming from USB
  }
}


/////////////////////////////////////////////////////////////////