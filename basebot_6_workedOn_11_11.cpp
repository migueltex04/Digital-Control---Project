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
 * Column 30: independent trajectory turnrate [rad/s]
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

// Circle grip test parameters
// Change this parameter between runs: 1.0, 2.0, 3.0, ...
const float aCentTest = 2.2f;       // Requested centripetal acceleration [m/s^2]

const float circleRadius = 0.50f;   // Radius of the track centreline [m]
const float circleChord = 0.30f;    // Distance between arc endpoints [m]
const float circleTangAccel = 1.0f; // Tangential acceleration/braking [m/s^2]

// Start on the right, aligned with the entry tangent.
// Positive heading corresponds to counterclockwise rotation.
const float circleAngle =
    2.0f * PI - 2.0f * asinf(circleChord / (2.0f * circleRadius));

const float circleLength = circleRadius * circleAngle;
const float circleSpeed = sqrtf(aCentTest * circleRadius);

const float circleRampTime = circleSpeed / circleTangAccel;
const float circleRampDistance =
    0.5f * circleTangAccel * circleRampTime * circleRampTime;

const float circleFlatDistance = circleLength - 2.0f * circleRampDistance;
const float circleFlatTime = circleFlatDistance / circleSpeed;
const float circleBrakeStart = circleRampTime + circleFlatTime;
const float circleEndTime = circleBrakeStart + circleRampTime;

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

const bool useTurnrateCascade = true;

const float turnrateMax = 4.0f;

// Heading phase-lead

const float KphHeading = 2.07f;
const float TleadHeading = 0.2052f;
const float alphaHeading = 0.9421f;

// Acceleration limiter

const float Kacc = 25.0f;
const float mu  =  0.2f;
const float g    = 9.81f;
const float accMax = mu * g;  // m/s^2

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
// FULL PATH: 1 m entry, major circular arc, 1.5 m exit
// Times are measured from start(); no measured-position switching.
// ============================================================
const float fullStartHold = 0.5f;
const float fullHeadingWait = 0.3f;
const float fullCircleWheelAccel = 0.20f * 9.81f; // MATLAB floor parameter
const float fullVelocityRampStart = 5.1f;        // absolute mission time [s]
const float fullVelocityRampEnd = 5.4f;
const float fullEntryCorrectionDeg = 0.0f;       // current standalone 1 m test
const float fullExitCorrectionDeg = 0.0f;        // current standalone 1.5 m test

// Independent trajectory derivative, reserved for future feedforward.
// NOT added to the heading/turnrate controller. Heading steps are excluded.
float trajectoryTurnrateRef = 0.0f;

struct FullStraightProfile
{
    float acceleration, ta, tb, peak, exponent;
};

const FullStraightProfile fullEntry = {
    1.5f, 0.85f,
    (1.2278481f + 1.0f) * (1.0f - 0.5f*1.5f*0.85f*0.85f) / (1.5f*0.85f),
    1.5f*0.85f, 1.2278481f
};
const FullStraightProfile fullExit = {
    1.5f, 1.1f, 0.8f, 1.5f*1.1f,
    (1.5f*1.1f)*0.8f/(1.5f-0.5f*1.5f*1.1f*1.1f)-1.0f
};

const float fullEntryStart = fullStartHold;
const float fullEntryEnd = fullEntryStart + fullEntry.ta + fullEntry.tb;
const float fullCircleStart = fullEntryEnd + fullHeadingWait;
const float fullCircleAccel = fullCircleWheelAccel / (1.0f + wheelBase/(2.0f*circleRadius));
const float fullCirclePeak = fminf(circleSpeed, sqrtf(fullCircleAccel*circleLength));
const float fullCircleTa = fullCirclePeak/fullCircleAccel;
const float fullCircleTc = fmaxf(0.0f,
    (circleLength-fullCirclePeak*fullCirclePeak/fullCircleAccel)/fullCirclePeak);
const float fullCircleDuration = 2.0f*fullCircleTa + fullCircleTc;
const float fullCircleEnd = fullCircleStart + fullCircleDuration;
const float fullExitStart = fullCircleEnd + fullHeadingWait;
const float fullExitEnd = fullExitStart + fullExit.ta + fullExit.tb;
const float fullCircleHeadingStart = -PI/2.0f + asinf(circleChord/(2.0f*circleRadius));

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

volatile bool isSampleTime = false;

volatile float time_sec = 0;

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

    float trajectoryTurnrateRef; // appended column 30

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

bool startRequested()
{
    return (digitalReadFast(PIN_START_BUTTON) || robot.missionStart)
           && imu2.gyroOffsetDone;
}

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

    trajectoryTurnrateRef = 0.0f;

    desiredValue = 0;

    desiredTurnrate = 0;

    desiredHeading = 0;
}


// ============================================================
// 17. STOP
// ============================================================

void stop(float after)
{
    state = 2;

    trajectoryTurnrateRef = 0.0f;

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

void waitForWheelsToStop()
{
    desiredValue = 0;

    const bool settled =
        fabsf(linearLimiter) < 0.002f &&
        fabsf(wheelVel[0]) < 0.015f &&
        fabsf(wheelVel[1]) < 0.015f;

    if (settled || time_sec >= endTime)
        stop(0.5f);
}

// ============================================================
// 9. NEW MISSION: CIRCLE GRIP TEST
// ============================================================

void sequenceCircleGripTest()
{
    switch (state)
    {
        case 0:
            if (startRequested())
            {
                // The experiment needs a constant-speed section.
                if (aCentTest <= 0.0f || circleFlatDistance <= 0.0f)
                {
                    if (Serial)
                        Serial.println("% Invalid circle profile: no constant-speed section.");
                    return;
                }

                start();
                state = 30;
            }
            break;

        case 30:  // Accelerate along the arc
        case 31:  // Constant-speed section used for the grip comparison
        case 32:  // Brake along the arc
        {
            const float t = time_sec;
            float sRef;

            if (t < circleRampTime)
            {
                state = 30;
                desiredValue = circleTangAccel * t;
                sRef = 0.5f * circleTangAccel * t * t;
            }
            else if (t < circleBrakeStart)
            {
                state = 31;
                desiredValue = circleSpeed;
                sRef = circleRampDistance
                     + circleSpeed * (t - circleRampTime);
            }
            else if (t < circleEndTime)
            {
                state = 32;
                const float dt = t - circleBrakeStart;

                desiredValue = circleSpeed - circleTangAccel * dt;
                sRef = circleRampDistance + circleFlatDistance
                     + circleSpeed * dt
                     - 0.5f * circleTangAccel * dt * dt;
            }
            else
            {
                state = 33;
                desiredValue = 0;
                sRef = circleLength;
                endTime = t + 2.0f;
            }

            // Continuous gyro heading: allow angles greater than pi.
            desiredHeading = sRef / circleRadius;
            break;
        }

        case 33:  // Hold final arc heading while the wheels settle
            desiredHeading = circleAngle;
            waitForWheelsToStop();
            break;

        case 2:
            if (time_sec >= endTime)
                finished();
            break;
    }
}

// ============================================================
// 10. PREVIOUS MISSIONS
// ============================================================

void sequenceTest()
{
    const float vMax = 1.50f;
    const float rampTime = 1.0f;

    if (state == 10 || state == 11 || state == 12)
        desiredHeading = (time_sec < 0.50f) ? 0.0f : -PI / 180.0f;

    switch (state)
    {
        case 0:
            if (startRequested())
            {
                start();
                state = 10;
            }
            break;

        case 10:
            if (time_sec < rampTime)
                desiredValue = vMax * time_sec / rampTime;
            else
            {
                desiredValue = vMax;
                state = 11;
            }
            break;

        case 11:
            if (time_sec < 2.0f * rampTime)
                desiredValue = vMax * (2.0f * rampTime - time_sec) / rampTime;
            else
            {
                desiredValue = 0;
                endTime = time_sec + 2.0f;
                state = 12;
            }
            break;

        case 12:
            waitForWheelsToStop();
            break;

        case 2:
            if (time_sec >= endTime)
                finished();
            break;
    }
}

void sequenceDistance150Test()
{
    const float acceleration = 1.5f;
    const float tAccel = 1.1f;
    const float vMax = acceleration * tAccel;
    const float tBrake = 0.8f;
    const float distanceTotal = 1.5f;

    const float distanceAccel = 0.5f * acceleration * tAccel * tAccel;
    const float distanceBrake = distanceTotal - distanceAccel;
    const float brakeExponent = vMax * tBrake / distanceBrake - 1.0f;

    switch (state)
    {
        case 0:
            if (startRequested())
            {
                start();
                state = 10;
            }
            break;

        case 10:
        case 11:
        {
            const float t = time_sec;
            desiredHeading = (t < 0.5f) ? 0.0f : 0.0f * PI / 180.0f; //0.0 no chão de casa e -2.0 no da DTU

            if (t < tAccel)
            {
                state = 10;
                desiredValue = acceleration * t;
            }
            else if (t < tAccel + tBrake)
            {
                state = 11;
                const float u = (t - tAccel) / tBrake;
                desiredValue = vMax * powf(1.0f - u, brakeExponent);
            }
            else
            {
                state = 12;
                desiredValue = 0;
                endTime = t + 2.0f;
            }
            break;
        }

        case 12:
            waitForWheelsToStop();
            break;

        case 2:
            if (time_sec >= endTime)
                finished();
            break;
    }
}

void sequenceDistance100Test()
{
    const float acceleration = 1.5f;
    const float tAccel = 0.85f;
    const float vMax = acceleration * tAccel;
    const float distanceTotal = 1.0f;

    const float brakeExponent = 1.2278481f;

    const float distanceAccel =
        0.5f * acceleration * tAccel * tAccel;

    const float distanceBrake = distanceTotal - distanceAccel;

    const float tBrake =
        (brakeExponent + 1.0f) * distanceBrake / vMax;

    switch (state)
    {
        case 0:
            if (startRequested())
            {
                start();
                state = 10;
            }
            break;

        case 10:
        case 11:
        {
            const float t = time_sec;

            desiredHeading =
                (t < 0.4f) ? 0.0f : 0.0f * PI / 180.0f; //0.0 no chão de casa e -1.0 no da DTU (por testar)

            if (t < tAccel)
            {
                state = 10;
                desiredValue = acceleration * t;
            }
            else if (t < tAccel + tBrake)
            {
                state = 11;
                const float u = (t - tAccel) / tBrake;

                desiredValue =
                    vMax * powf(1.0f - u, brakeExponent);
            }
            else
            {
                state = 12;
                desiredValue = 0.0f;
                endTime = t + 2.0f;
            }
            break;
        }

        case 12:
            waitForWheelsToStop();
            break;

        case 2:
            if (time_sec >= endTime)
                finished();
            break;
    }
}

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

float fullStraightVelocity(float t, const FullStraightProfile &p)
{
    if (t < 0.0f || t >= p.ta+p.tb) return 0.0f;
    if (t < p.ta) return p.acceleration*t;
    const float u = (t-p.ta)/p.tb;
    return p.peak*powf(fmaxf(0.0f,1.0f-u),p.exponent);
}

void fullCircleReference(float t, float &distance, float &velocity)
{
    distance = 0.0f;
    velocity = 0.0f;
    if (t <= 0.0f) return;
    if (t >= fullCircleDuration)
    {
        distance = circleLength;
        return;
    }
    if (t < fullCircleTa)
    {
        distance = 0.5f*fullCircleAccel*t*t;
        velocity = fullCircleAccel*t;
    }
    else if (t < fullCircleTa+fullCircleTc)
    {
        distance = 0.5f*fullCircleAccel*fullCircleTa*fullCircleTa
                 + fullCirclePeak*(t-fullCircleTa);
        velocity = fullCirclePeak;
    }
    else
    {
        const float remaining = fullCircleDuration-t;
        distance = circleLength-0.5f*fullCircleAccel*remaining*remaining;
        velocity = fullCircleAccel*remaining;
    }
}

void sequenceFullPath()
{
    if (state == 0)
    {
        if (startRequested())
        {
            if (!(fullVelocityRampStart >= fullCircleStart &&
                  fullVelocityRampEnd > fullVelocityRampStart &&
                  fullVelocityRampEnd <= fullCircleEnd))
            {
                if (Serial) Serial.println("% Full path: velocity ramp must fit inside circle timing.");
                return;
            }
            start();
            trajectoryTurnrateRef = 0.0f;
            state = 40;
        }
        return;
    }
    trajectoryTurnrateRef = 0.0f;
    if (state == 2)
    {
        if (time_sec >= endTime) finished();
        return;
    }
    if (state == 47)
    {
        waitForWheelsToStop();
        return;
    }

    const float t = time_sec;
    desiredValue = 0.0f;
    if (t < fullEntryStart)
    {
        state = 40;
        desiredHeading = 0.0f;
    }
    else if (t < fullEntryEnd)
    {
        state = 41;
        const float local = t-fullEntryStart;
        desiredValue = fullStraightVelocity(local,fullEntry);
        desiredHeading = (local < 0.4f) ? 0.0f : fullEntryCorrectionDeg*PI/180.0f;
    }
    else if (t < fullCircleStart)
    {
        state = 42;
        desiredHeading = fullCircleHeadingStart;
    }
    else if (t < fullCircleEnd)
    {
        state = 43;
        float sCircle, vCircle;
        fullCircleReference(t-fullCircleStart,sCircle,vCircle);
        desiredHeading = fullCircleHeadingStart+sCircle/circleRadius;
        trajectoryTurnrateRef = vCircle/circleRadius;
        desiredValue = vCircle;
        // Reproduce the MATLAB FORWARD-velocity override independently.
        // The heading derivative above intentionally keeps its nominal timing.
        if (t >= fullVelocityRampEnd)
            desiredValue = 0.0f;
        else if (t >= fullVelocityRampStart)
        {
            float unusedDistance, rampSpeed;
            fullCircleReference(fullVelocityRampStart-fullCircleStart,
                                unusedDistance,rampSpeed);
            desiredValue = rampSpeed*(fullVelocityRampEnd-t)
                         /(fullVelocityRampEnd-fullVelocityRampStart);
        }
    }
    else if (t < fullExitStart)
    {
        state = 44;
        desiredHeading = PI;
    }
    else if (t < fullExitEnd)
    {
        state = 45;
        const float local = t-fullExitStart;
        desiredValue = fullStraightVelocity(local,fullExit);
        desiredHeading = PI+((local < 0.5f) ? 0.0f : fullExitCorrectionDeg*PI/180.0f);
    }
    else
    {
        state = 47;
        desiredHeading = PI+fullExitCorrectionDeg*PI/180.0f;
        endTime = t+2.0f;
    }
}

// ============================================================
// MISSION SELECTION -- change ONLY the function name below
// Options: sequenceHeadingTest, sequenceTest, sequenceCircleGripTest,
//          sequenceDistance100Test, sequenceDistance150Test, sequenceFullPath
// Full path selected; all standalone missions remain available.
// ============================================================
const auto selectedMission = sequenceFullPath;

const char *selectedMissionName()
{
    if (selectedMission == sequenceHeadingTest) return "sequenceHeadingTest";
    if (selectedMission == sequenceTest) return "sequenceTest";
    if (selectedMission == sequenceCircleGripTest) return "sequenceCircleGripTest";
    if (selectedMission == sequenceDistance100Test) return "sequenceDistance100Test";
    if (selectedMission == sequenceDistance150Test) return "sequenceDistance150Test";
    if (selectedMission == sequenceFullPath) return "sequenceFullPath";
    return "Custom mission";
}

void printMissionSettings()
{
    Serial.print("% Selected mission: ");
    Serial.println(selectedMissionName());
    Serial.print("% Sample time [s] = "); Serial.println(ts, 6);
    Serial.print("% Wheel PI Kp = "); Serial.println(Kp, 6);
    Serial.print("% Wheel PI taui [s] = "); Serial.println(taui, 6);
    Serial.print("% Heading Kph = "); Serial.println(KphHeading, 6);
    Serial.print("% Heading Tlead [s] = "); Serial.println(TleadHeading, 6);
    Serial.print("% Heading alpha = "); Serial.println(alphaHeading, 6);
    Serial.print("% Turnrate cascade enabled = "); Serial.println(useTurnrateCascade);
    Serial.print("% mu configured in reference limiter = "); Serial.println(mu, 4);
    Serial.print("% Reference acceleration limit [m/s^2] = "); Serial.println(accMax, 4);
    Serial.print("% Wheel radius [m] = "); Serial.println(wheelRadius, 6);
    Serial.print("% Wheel base [m] = "); Serial.println(wheelBase, 6);
    Serial.print("% Gyro tilt [deg] = "); Serial.println(gyroTilt, 3);
    Serial.print("% Gyro sign = "); Serial.println(gyroSign, 1);
    Serial.print("% Log interval [s] = "); Serial.println(ts * logEvery, 6);
    Serial.print("% Log capacity [samples] = "); Serial.println(logsMax);
    Serial.print("% Log capacity reached = "); Serial.println(logsCnt >= logsMax);

    if (selectedMission == sequenceCircleGripTest)
    {
        Serial.print("% Requested centripetal acceleration [m/s^2] = ");
        Serial.println(aCentTest, 4);
        Serial.print("% Circle speed reference [m/s] = "); Serial.println(circleSpeed, 4);
        Serial.print("% Circle radius [m] = "); Serial.println(circleRadius, 4);
        Serial.print("% Arc angle [deg] = "); Serial.println(circleAngle * 180.0f / PI, 3);
        Serial.print("% Nominal constant-speed duration [s] = "); Serial.println(circleFlatTime, 4);
        Serial.println("% Circle states: 30 accelerate, 31 constant speed, 32 brake, 33 settle");
    }
    if (selectedMission == sequenceDistance100Test ||
        selectedMission == sequenceDistance150Test)
    {
        Serial.println("% Straight states: 10 accelerate, 11 brake, 12 settle");
        Serial.println("% Straight acceleration [m/s^2] = 1.5");
        if (selectedMission == sequenceDistance100Test)
        {
            Serial.println("% Distance [m] = 1.0; tAccel [s] = 0.85; peak speed [m/s] = 1.275");
            Serial.println("% Brake exponent = 1.2278481; tBrake [s] = 0.8004964");
            Serial.println("% Heading correction [deg] = 0 at local time [s] = 0.4");
        }
        else
        {
            Serial.println("% Distance [m] = 1.5; tAccel [s] = 1.1; peak speed [m/s] = 1.65");
            Serial.println("% Brake exponent = 1.2278481; tBrake [s] = 0.8");
            Serial.println("% Heading correction [deg] = 0 at local time [s] = 0.5");
        }
    }
    if (selectedMission == sequenceFullPath)
    {
        Serial.println("% Full path: 1 m entry, major arc R=0.5 m/chord=0.3 m, 1.5 m exit");
        Serial.print("% Entry start/end [s] = "); Serial.print(fullEntryStart,6);
        Serial.print(" "); Serial.println(fullEntryEnd,6);
        Serial.print("% Circle start/end [s] = "); Serial.print(fullCircleStart,6);
        Serial.print(" "); Serial.println(fullCircleEnd,6);
        Serial.print("% Exit start/end [s] = "); Serial.print(fullExitStart,6);
        Serial.print(" "); Serial.println(fullExitEnd,6);
        Serial.print("% Forward velocity ramp start/end [s] = "); Serial.print(fullVelocityRampStart,6);
        Serial.print(" "); Serial.println(fullVelocityRampEnd,6);
        Serial.print("% Entry/exit heading corrections [deg] = "); Serial.print(fullEntryCorrectionDeg,3);
        Serial.print(" "); Serial.println(fullExitCorrectionDeg,3);
        Serial.println("% Full-path states: 40 initial hold, 41 entry, 42 heading wait, 43 circle, 44 heading wait, 45 exit, 47 settle");
        Serial.println("% Column 30 = nominal circle heading derivative; feedforward is not enabled.");
    }
    Serial.println("% State 2 = motor voltages off; state 0 = idle");
}

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
    d.trajectoryTurnrateRef = trajectoryTurnrateRef;

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

    printMissionSettings();

    Serial.println("% Wheel PI: Kp=0.040, tauI=0.160");

    Serial.print("% Heading phase lead: Kph=");
    Serial.print(KphHeading, 6);
    Serial.print(", T=");
    Serial.print(TleadHeading, 6);
    Serial.print(", alpha=");
    Serial.println(alphaHeading, 6);

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
    Serial.println("% 30 trajectory turnrate for future feedforward [rad/s]");

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
        Serial.print(d.headingGyro, 5);
        Serial.print(" ");
        Serial.println(d.trajectoryTurnrateRef, 5);
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

            // Run the mission chosen in MISSION SELECTION.

            selectedMission();

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