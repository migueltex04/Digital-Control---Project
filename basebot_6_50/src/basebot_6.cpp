/***************************************************************************
 * DTU DIGITAL CONTROL
 * BASEBOT - HEADING CONTROL WITH GYROSCOPE FEEDBACK
 *
 * Teensy 4.1
 *
 * Same trajectory and controller parameters as original experiment.
 *
 * Main change:
 * Heading feedback uses the integrated gyroscope instead of encoders.
 *
 * Wheel velocity PI controllers still use encoders.
 *
 * Logging:
 * Columns 1-28: original format
 * Column 29: gyroscope heading [rad]
 *
 * Send L to retrieve logs.
 ***************************************************************************/

#include <Arduino.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <IntervalTimer.h>

#include "src/udisplay.h"
#include "src/urobot.h"
#include "src/uusb.h"
#include "src/umotor.h"
#include "src/uimu2.h"
#include "src/uencoder.h"


// ============================================================
// 1. PARAMETERS
// ============================================================

const uint32_t sampleTimeUs = 500;
const float ts = sampleTimeUs * 1e-6f;

// Robot geometry

const float gear = 9.6f;
const float wheelRadius = 0.03380f;
const float wheelBase = 0.147f;

// Wheel PI

const float Kp = 0.040f;
const float taui = 0.160f;
const float Ki = Kp / taui;

const float voltageMax = 9.0f;

// Turnrate PI (not used in this experiment)

const float KpTurn = 0.0443322593f;
const float tauiTurn = 0.00778527218f;
const float KiTurn = KpTurn / tauiTurn;

const bool useTurnrateCascade = false;

const float turnrateMax = 4.0f;

// Heading phase-lead

const float KphHeading = 2.07f;
const float TleadHeading = 0.2052f;
const float alphaHeading = 0.9421f;

// Acceleration limiter

const float Kacc = 25.0f;
const float miu  =  0.3f;
const float g    = 9.81f;
const float accMax = miu * g;  // m/s^2

// The acceleration limiter has a problem when impleemnted here.
// It restricts the motor to a maxium linear acceleration and a maximum turning acceleration,
// but if we are accelerating linearly and tell it to turn at the same time, in one of the wheels those
// accelerations will sum and that wheel will slip. The way to prevent this is to put this acceleration
// controller/limiter after the mixer, at the input of each wheel velocity reference;
// however, this may be problematic when starting to speed up. But it may be worthy to test though,
// if the mission starts getting too specific and we tell Leonora to slow down before a turn
// and accelerate and change heading at the same time.

// Gyroscope inclination

const float gyroTilt = 22.0f;

// Set to +1 if positive physical yaw gives positive gyro Z.
// If the signs are opposite, this must be -1.
// Do not run closed-loop heading control with an unverified sign.

const float gyroSign = 1.0f;


// ============================================================
// 2. CONTROLLER STATES
// ============================================================

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

Integrator leftPI = {0, 0};
Integrator rightPI = {0, 0};

Integrator turnratePI = {0, 0};

struct HeadingLeadState
{
    float previousError;
    float previousOutput;
};

HeadingLeadState headingLead = {0, 0};

float linearLimiter = 0.0f;
float turningLimiter = 0.0f;


// ============================================================
// 3. ROBOT VARIABLES
// ============================================================

float distA = 0;

float pose[4] = {0, 0, 0, 0};

float poseVel[2] = {0, 0};

float wheelVel[2] = {0, 0};

bool firstPoseSample = true;

bool piControlActive = false;


// ============================================================
// 4. GYROSCOPE VARIABLES
// ============================================================

float headingGyro = 0.0f;

float turnrateGyro = 0.0f;

float previousTurnrateGyro = 0.0f;

bool firstGyroSample = true;


// ============================================================
// 5. EXPERIMENT STATE
// ============================================================

int state = 0;

float endTime = 0;

float desiredValue = 0;

float desiredHeading = 0;

float desiredTurnrate = 0;

float inVelLeft = 0;

float inVelRight = 0;


// ============================================================
// 6. TIMER
// ============================================================

bool isSampleTime = false;

float time_sec = 0;

IntervalTimer sampleTimer;

void timerInterrupt()
{
    isSampleTime = true;

    time_sec += sampleTimeUs / 1e6f;
}


// ============================================================
// 7. LOG STRUCTURE
// ============================================================

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

    float gyro[3];

    // NEW: integrated gyroscope heading

    float headingGyro;

} LogData;


// ============================================================
// 8. LOG MEMORY
// ============================================================

const int logsMax = 400000 / sizeof(LogData);

LogData *logs = (LogData *)malloc(
    logsMax * sizeof(LogData));

int logsCnt = 0;

const int logEvery = 10;

int logSampleCounter = 0;

bool usbMonitorPreviouslyConnected = false;


// ============================================================
// 9. INTEGRATION
// ============================================================

float integrate(
    Integrator &memory,
    float input,
    float dt)
{
    memory.value +=
        0.5f * dt *
        (input + memory.previousInput);

    memory.previousInput = input;

    return memory.value;
}


// ============================================================
// 10. WHEEL PI
// ============================================================

float wheelPI(Integrator &memory, float error)
{
    const float oldIntegral = memory.value;

    integrate(memory, error, ts);

    float voltage =
        Kp * error + Ki * memory.value;

    const float integralChange =
        Ki * (memory.value - oldIntegral);

    if ((voltage > voltageMax && integralChange > 0) ||
        (voltage < -voltageMax && integralChange < 0))
    {
        memory.value = oldIntegral;

        voltage =
            Kp * error + Ki * memory.value;
    }

    return fmaxf(
        -voltageMax,
        fminf(voltageMax, voltage));
}


// ============================================================
// 11. TURNRATE PI
// ============================================================

float turnrateControl(float error)
{
    const float oldIntegral = turnratePI.value;

    integrate(turnratePI, error, ts);

    float turnrateCommand =
        KpTurn * error + KiTurn * turnratePI.value;

    const float integralChange =
        KiTurn * (turnratePI.value - oldIntegral);

    if ((turnrateCommand > turnrateMax &&
         integralChange > 0) ||
        (turnrateCommand < -turnrateMax &&
         integralChange < 0))
    {
        turnratePI.value = oldIntegral;

        turnrateCommand =
            KpTurn * error + KiTurn * turnratePI.value;
    }

    return fmaxf(
        -turnrateMax,
        fminf(turnrateMax, turnrateCommand));
}


// ============================================================
// 12. HEADING PHASE-LEAD
// ============================================================

float headingControl(float error)
{
    const float d =
        2.0f * alphaHeading * TleadHeading + ts;

    const float b0 =
        (2.0f * TleadHeading + ts) / d;

    const float b1 =
        (ts - 2.0f * TleadHeading) / d;

    const float a1 =
        (ts - 2.0f * alphaHeading * TleadHeading) / d;

    const float leadOutput =
          b0 * error
        + b1 * headingLead.previousError
        - a1 * headingLead.previousOutput;

    headingLead.previousError = error;

    headingLead.previousOutput = leadOutput;

    const float turnrateCommand =
        KphHeading * leadOutput;

    return fmaxf(
        -turnrateMax,
        fminf(turnrateMax, turnrateCommand));
}

// ============================================================
// ACCELERATION CONTROLLER AND LIMITER
// ============================================================

float accelerationLimiter(float &memory, float reference)
{
    const float error = reference - memory;

    const float acceleration = Kacc * error;

    const float limitedAcceleration =
        fmaxf(-accMax, fminf(accMax, acceleration));

    memory += limitedAcceleration * ts;

    return memory;
}


// ============================================================
// 13. UPDATE GYROSCOPE HEADING
// ============================================================

void updateGyroHeading()
{
    // imu2.gyro[2] is already calibrated by uimu2.cpp
    // Units: degrees/second

    const float gyroZ = imu2.gyro[2];

    // Convert deg/s to rad/s
    // Correct approximately for the 22-degree tilt

    turnrateGyro =
        gyroSign *
        gyroZ *
        (PI / 180.0f) /
        cosf(gyroTilt * PI / 180.0f);

    // Avoid using an uninitialized previous sample

    if (firstGyroSample)
    {
        previousTurnrateGyro = turnrateGyro;

        firstGyroSample = false;

        return;
    }

    // Trapezoidal integration

    headingGyro +=
        0.5f * ts *
        (turnrateGyro + previousTurnrateGyro);

    previousTurnrateGyro = turnrateGyro;
}


// ============================================================
// 14. UPDATE ENCODER ODOMETRY
// ============================================================

void updatePose()
{
    wheelVel[0] =
        -encoder.motorVelocity[0] *
        wheelRadius / gear;

    wheelVel[1] =
         encoder.motorVelocity[1] *
         wheelRadius / gear;

    poseVel[0] =
        0.5f * (wheelVel[0] + wheelVel[1]);

    poseVel[1] =
        (wheelVel[1] - wheelVel[0]) / wheelBase;

    if (firstPoseSample)
    {
        distanceIntegral.previousInput = poseVel[0];

        tripIntegral.previousInput =
            fabsf(poseVel[0]);

        headingIntegral.previousInput = poseVel[1];

        xIntegral.previousInput = poseVel[0];

        yIntegral.previousInput = 0;

        firstPoseSample = false;

        return;
    }

    // Encoder heading is retained for comparison

    pose[2] =
        integrate(headingIntegral, poseVel[1], ts);

    pose[3] =
        integrate(distanceIntegral, poseVel[0], ts);

    distA =
        integrate(tripIntegral, fabsf(poseVel[0]), ts);

    pose[0] =
        integrate(
            xIntegral,
            poseVel[0] * cosf(pose[2]),
            ts);

    pose[1] =
        integrate(
            yIntegral,
            poseVel[0] * sinf(pose[2]),
            ts);
}


// ============================================================
// 15. RESET CONTROLLER STATES
// ============================================================

void resetStates()
{
    distanceIntegral = {0, 0};
    tripIntegral = {0, 0};
    headingIntegral = {0, 0};
    xIntegral = {0, 0};
    yIntegral = {0, 0};

    for (int i = 0; i < 4; i++)
    {
        pose[i] = 0;
    }

    for (int i = 0; i < 2; i++)
    {
        poseVel[i] = 0;
        wheelVel[i] = 0;
    }

    distA = 0;

    firstPoseSample = true;

    leftPI = {0, 0};
    rightPI = {0, 0};

    turnratePI = {0, 0};

    headingLead = {0, 0};

    linearLimiter = 0.0f;
    turningLimiter = 0.0f;

    // NEW: reset gyroscope integration

    headingGyro = 0.0f;

    turnrateGyro = 0.0f;

    previousTurnrateGyro = 0.0f;

    firstGyroSample = true;
}


// ============================================================
// 16. START
// ============================================================

void start()
{
    logsCnt = 0;

    logSampleCounter = 0;

    resetStates();

    motor.setPWMfrq(80000);

    time_sec = 0;

    encoder.encoder[0] = 0;
    encoder.encoder[1] = 0;

    display.useDisplay = false;

    desiredTurnrate = 0;

    desiredHeading = 0;
}


// ============================================================
// 17. STOP
// ============================================================

void stop(float after)
{
    state = 2;

    desiredValue = 0;

    desiredTurnrate = 0;

    // Keep the reference consistent with the feedback source

    desiredHeading = headingGyro;

    endTime = time_sec + after;
}


// ============================================================
// 18. FINISHED
// ============================================================

void finished()
{
    motor.motorVoltage[0] = 0;
    motor.motorVoltage[1] = 0;

    motor.setPWMfrq(80000);

    display.useDisplay = true;

    state = 0;

    if (Serial)
    {
        Serial.print("% Finished. Saved samples: ");
        Serial.println(logsCnt);
    }
}


// ============================================================
// 19. ORIGINAL HEADING SEQUENCE
// ============================================================

void sequenceHeadingTest()
{
    switch (state)
    {
        case 0:

            if (digitalReadFast(PIN_START_BUTTON) ||
                robot.missionStart)
            {
                // Prevent running with uncalibrated gyroscope

                if (!imu2.gyroOffsetDone)
                {
                    break;
                }

                start();

                desiredValue = 0.0f;

                desiredHeading = 0.0f;

                endTime = 0.5f;

                state = 10;
            }

            break;


        case 10:

            if (time_sec >= endTime)
            {
                desiredValue = 0.75f;

                endTime = 1.5f;

                state = 11;
            }

            break;


        case 11:

            if (time_sec >= endTime)
            {
                desiredHeading = PI / 2.0f;

                endTime = 2.5f;

                state = 12;
            }

            break;


        case 12:

            if (time_sec >= endTime)
            {
                desiredHeading = PI;

                endTime = 3.5f;

                state = 13;
            }

            break;


        case 13:

            if (time_sec >= endTime)
            {
                desiredHeading = 3.0f * PI / 2.0f;

                endTime = 4.5f;

                state = 14;
            }

            break;


        case 14:

            if (time_sec >= endTime)
            {
                desiredHeading = 2.0f * PI;

                endTime = 5.5f;

                state = 15;
            }

            break;


        case 15:

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


// ============================================================
// 20. CONTROL UPDATE
// ============================================================

void controlUpdatePI()
{
    piControlActive = true;

    if (state == 2)
    {
        inVelLeft = 0;
        inVelRight = 0;

        motor.motorVoltage[0] = 0;
        motor.motorVoltage[1] = 0;

        return;
    }

    // MAIN CHANGE: GYROSCOPE HEADING FEEDBACK

    // Old:
    // const float headingError = desiredHeading - pose[2];

    // New:

    const float headingError =
        desiredHeading - headingGyro;

    const float headingTurnrateRef =
        headingControl(headingError);

    float turnrateCommand = headingTurnrateRef;

    if (useTurnrateCascade)
    {
        const float measuredTurnrate = poseVel[1];

        const float turnrateError =
            headingTurnrateRef - measuredTurnrate;

        turnrateCommand =
            turnrateControl(turnrateError);
    }

    desiredTurnrate = headingTurnrateRef;

    // Acceleration limiters before mixer

    const float limitedVelocity =
        accelerationLimiter(linearLimiter, desiredValue);

    const float turningVelocity =
        0.5f * wheelBase * turnrateCommand;

    const float limitedTurningVelocity =
        accelerationLimiter(turningLimiter, turningVelocity);

    // Mixer

    inVelLeft = limitedVelocity - limitedTurningVelocity;
    inVelRight = limitedVelocity + limitedTurningVelocity;

    // Convert wheel linear velocities to motor angular velocities

    const float omegaRefLeft =
        inVelLeft * gear / wheelRadius;

    const float omegaRefRight =
        inVelRight * gear / wheelRadius;

    // Encoders still control wheel velocity

    const float omegaLeft =
        -encoder.motorVelocity[0];

    const float omegaRight =
         encoder.motorVelocity[1];

    // Wheel velocity errors

    const float errorLeft =
        omegaRefLeft - omegaLeft;

    const float errorRight =
        omegaRefRight - omegaRight;

    // Wheel PI controllers

    const float voltageLeft =
        wheelPI(leftPI, errorLeft);

    const float voltageRight =
        wheelPI(rightPI, errorRight);

    // Motor commands

    motor.motorVoltage[0] = -voltageLeft;
    motor.motorVoltage[1] = voltageRight;
}


// ============================================================
// 21. UPDATE LOG
// ============================================================

void updateLog()
{
    if (state <= 0)
        return;

    if (logSampleCounter++ % logEvery != 0)
        return;

    if (logs == nullptr || logsCnt >= logsMax)
        return;

    LogData &d = logs[logsCnt];

    d.time = time_sec;

    d.state = state;

    d.desiredValue = desiredValue;

    d.motorVolt[0] = -motor.motorVoltage[0];

    d.motorVolt[1] = motor.motorVoltage[1];

    d.encoder[0] = -encoder.encoder[0];

    d.encoder[1] = encoder.encoder[1];

    d.motorVel[0] = -encoder.motorVelocity[0];

    d.motorVel[1] = encoder.motorVelocity[1];

    d.motorCurrent[0] =
        -(analogRead(A1) - 2048) * 0.0045f + 0.3195f;

    d.motorCurrent[1] =
         (analogRead(A0) - 2048) * 0.0045f - 0.3465f;

    d.battery = robot.batteryVoltage;

    d.gyro[0] = imu2.gyro[0];
    d.gyro[1] = imu2.gyro[1];
    d.gyro[2] = imu2.gyro[2];

    d.desiredTurnrate = desiredTurnrate;

    for (int i = 0; i < 4; i++)
    {
        d.pose[i] = pose[i];
    }

    d.distA = distA;

    for (int i = 0; i < 2; i++)
    {
        d.poseVel[i] = poseVel[i];

        d.wheelVel[i] = wheelVel[i];
    }

    d.inVelLeft = inVelLeft;

    d.inVelRight = inVelRight;

    d.desiredHeading = desiredHeading;

    // NEW: save integrated gyro heading

    d.headingGyro = headingGyro;

    logsCnt++;
}


// ============================================================
// 22. PRINT LOG
// ============================================================

void printLog()
{
    if (logs == nullptr)
        return;

    Serial.println("% BASEBOT - GYROSCOPE HEADING FEEDBACK");

    Serial.println("% Original heading sequence");

    Serial.println("% Wheel PI: Kp=0.040, tauI=0.160");

    Serial.println("% Heading phase lead: Kp=3.4, T=0.2052, alpha=0.9421");

    Serial.println("% Feedback source: integrated gyroscope");

    Serial.println("% Gyroscope readings: deg/s");

    Serial.println("% Column 21: encoder heading [rad]");

    Serial.println("% Column 29: gyroscope heading [rad]");

    Serial.println("% Columns:");

    Serial.println("% 1 time [s]");
    Serial.println("% 2 state");
    Serial.println("% 3 velocity reference [m/s]");
    Serial.println("% 4-5 motor voltage [V]");
    Serial.println("% 6-7 encoder counts");
    Serial.println("% 8-9 motor angular velocities [rad/s]");
    Serial.println("% 10-11 motor currents [A]");
    Serial.println("% 12 battery voltage [V]");
    Serial.println("% 13-15 gyro X,Y,Z [deg/s]");
    Serial.println("% 16 turnrate reference [rad/s]");
    Serial.println("% 17 forward velocity [m/s]");
    Serial.println("% 18 encoder turnrate [rad/s]");
    Serial.println("% 19-20 encoder position X,Y [m]");
    Serial.println("% 21 encoder heading [rad]");
    Serial.println("% 22 signed distance [m]");
    Serial.println("% 23 total distance [m]");
    Serial.println("% 24-25 wheel velocities [m/s]");
    Serial.println("% 26-27 wheel velocity references [m/s]");
    Serial.println("% 28 heading reference [rad]");
    Serial.println("% 29 gyroscope heading [rad]");

    for (int i = 0; i < logsCnt; i++)
    {
        const LogData &d = logs[i];

        Serial.print(d.time, 4);
        Serial.print(" ");

        Serial.print(d.state);
        Serial.print(" ");

        Serial.print(d.desiredValue, 4);
        Serial.print(" ");

        Serial.print(d.motorVolt[0], 4);
        Serial.print(" ");

        Serial.print(d.motorVolt[1], 4);
        Serial.print(" ");

        Serial.print(d.encoder[0]);
        Serial.print(" ");

        Serial.print(d.encoder[1]);
        Serial.print(" ");

        Serial.print(d.motorVel[0], 4);
        Serial.print(" ");

        Serial.print(d.motorVel[1], 4);
        Serial.print(" ");

        Serial.print(d.motorCurrent[0], 4);
        Serial.print(" ");

        Serial.print(d.motorCurrent[1], 4);
        Serial.print(" ");

        Serial.print(d.battery, 4);
        Serial.print(" ");

        Serial.print(d.gyro[0], 4);
        Serial.print(" ");

        Serial.print(d.gyro[1], 4);
        Serial.print(" ");

        Serial.print(d.gyro[2], 4);
        Serial.print(" ");

        Serial.print(d.desiredTurnrate, 5);

        for (int j = 0; j < 2; j++)
        {
            Serial.print(" ");
            Serial.print(d.poseVel[j], 5);
        }

        for (int j = 0; j < 4; j++)
        {
            Serial.print(" ");
            Serial.print(d.pose[j], 5);
        }

        Serial.print(" ");
        Serial.print(d.distA, 5);

        for (int j = 0; j < 2; j++)
        {
            Serial.print(" ");
            Serial.print(d.wheelVel[j], 5);
        }

        Serial.print(" ");
        Serial.print(d.inVelLeft, 5);

        Serial.print(" ");
        Serial.print(d.inVelRight, 5);

        Serial.print(" ");
        Serial.print(d.desiredHeading, 5);

        // Column 29

        Serial.print(" ");
        Serial.println(d.headingGyro, 5);
    }

    Serial.print("% sent ");
    Serial.print(logsCnt);
    Serial.println(" log lines");
}


// ============================================================
// 23. SETUP
// ============================================================

void setup()
{
    Serial.begin(12000000);

    sampleTimer.begin(timerInterrupt, sampleTimeUs);

    robot.setup();

    motor.setup();

    encoder.setup();

    encoder.pulsPerRev = 48;

    imu2.setup();

    motor.setPWMfrq(0x13000);

    usb.setup();

    display.useDisplay = true;
}


// ============================================================
// 24. MAIN LOOP
// ============================================================

void loop(void)
{
    while (true)
    {
        if (isSampleTime)
        {
            isSampleTime = false;

            // Read sensors

            imu2.tick();

            encoder.tick();

            // Original heading sequence

            sequenceHeadingTest();

            if (state > 0)
            {
                // Encoder odometry retained for logging

                updatePose();

                // NEW: gyroscope integration

                updateGyroHeading();

                // Heading feedback uses gyroscope

                controlUpdatePI();
            }

            // Apply motor voltages

            motor.tick();

            // Save data

            updateLog();

            // DTU support functions

            robot.tick();

            display.tick();
        }

        // ================================================
        // USB LOG RETRIEVAL
        // ================================================

        const bool usbMonitorConnected = (bool)Serial;

        if (state == 0 && usbMonitorConnected)
        {
            if (!usbMonitorPreviouslyConnected &&
                logsCnt > 0)
            {
                Serial.print("% Reconnected. Saved samples: ");
                Serial.println(logsCnt);

                printLog();
            }

            if (Serial.available() > 0)
            {
                const int incoming = Serial.peek();

                if (incoming == 'L' || incoming == 'l')
                {
                    Serial.read();

                    if (logsCnt > 0)
                    {
                        Serial.print("% Requested saved samples: ");
                        Serial.println(logsCnt);

                        printLog();
                    }
                    else
                    {
                        Serial.println("% No RAM logs.");
                    }
                }
            }
        }

        usbMonitorPreviouslyConnected = usbMonitorConnected;

        usb.tick();
    }
}