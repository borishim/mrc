#include "Arduino.h"

// ---- Servos ---- on Teensy PWM pins are 3-6 9-10 20-21
#define pin_servo_0 3
#define pin_servo_1 4
#define pin_servo_2 5
#define pin_servo_3 6
#define pin_servo_4 9
#define pin_servo_5 10

#define pin_additional_servo_6 20
#define pin_additional_servo_7 -1

// pinNumber, maxAngularVel degree/sec, calibMin, calibMax, angleDegMin, angleDegMax, home position
// minAngle must be less than maxAngle. To flip the direction of rotation do:
// {minFreq <-> maxFreq, minAngle * -1 <-> maxAngle * -1}
const float servoConfig[6][7] =
//{
//    { pin_servo_0, 150 * DEG_TO_RAD,  852, 2091,  -90 * DEG_TO_RAD,  90 * DEG_TO_RAD, 0 },
//    { pin_servo_1, 150 * DEG_TO_RAD,  710, 1780,  -70 * DEG_TO_RAD,  90 * DEG_TO_RAD, 0 },
//    { pin_servo_2, 150 * DEG_TO_RAD, 2099,  571,  (-90+1) * DEG_TO_RAD, (135-1) * DEG_TO_RAD, 0 },
//    { pin_servo_3, 150 * DEG_TO_RAD,  650, 2370,  -90 * DEG_TO_RAD,  75 * DEG_TO_RAD, 0 },
//    { pin_servo_4, 150 * DEG_TO_RAD, 2370,  860, -127 * DEG_TO_RAD,  14 * DEG_TO_RAD, 0 },
//    { pin_servo_5, 150 * DEG_TO_RAD, 2290,  570,  -75 * DEG_TO_RAD,  86 * DEG_TO_RAD, 0 }
//};
//{
//{ pin_servo_0,  150*DEG_TO_RAD, 540.00, 2400.00, -1.05, 1.05, 0 },
//{ pin_servo_1,  150*DEG_TO_RAD, 540.00, 2400.00, -0.79, 0.79, 0 },
//{ pin_servo_2,  150*DEG_TO_RAD, 1790.00, 600.00, -0.65, 0.70, 0 },
//{ pin_servo_3,  150*DEG_TO_RAD, 540.00, 2400.00, -1.52, 1.57, 0 },
//{ pin_servo_4,  150*DEG_TO_RAD, 540.00, 2400.00, -1.66, 0.17, 0 },
//{ pin_servo_5,  150*DEG_TO_RAD, 2400.00, 540.00, -0.79, 0.79, 0 }
//};
{
{ pin_servo_0,  150*DEG_TO_RAD, 750.00, 2120.00, -1.57, 1.57, 0 },
{ pin_servo_1,  150*DEG_TO_RAD, 690.00, 2370.00, -1.26, 1.57, 0 },
{ pin_servo_2,  150*DEG_TO_RAD, 1740.00, 780.00, -0.77, 1.33, 0 },
{ pin_servo_3,  150*DEG_TO_RAD, 560.00, 2370.00, -1.48, 1.48, 0 },
{ pin_servo_4,  150*DEG_TO_RAD, 610.00, 2390.00, -3.14, -0.24, 0 },
{ pin_servo_5,  150*DEG_TO_RAD, 2390.00, 600.00, -1.41, 1.57, 0 }
 };


// geometry
float geometry[5][3] = {
    {    4.6, 0,    7.9 },
    {      0, 0,   11.7 },
    {      1, 0,    1.7 },
    {  11.4, 0,      0 },
    {      0, 0,     -3. }
};

// E.g. joint 0 cant be < 90° to not crash into itself
float logicAngleLimits[6][2] = {
    { servoConfig[0][4],
      servoConfig[0][5] },
    { servoConfig[1][4],
      servoConfig[1][5] },
    {  -25 * DEG_TO_RAD,
       48 * DEG_TO_RAD  },
    { servoConfig[3][4],
      servoConfig[3][5] },
    { servoConfig[4][4],
      servoConfig[4][5] },
    { servoConfig[5][4],
      servoConfig[5][5] }
};

float v_1 = 11.7;
float h_1 = 3.7;
float h_2 = 11.7;
float h_3 = 3.0;

// relation between physical and logical angles based on robot kinematic coupling.
void logicalToPhysicalAngles(float angles[6]) {
    float alpha  = angles[2];
    float alpha_ = alpha + PI / 2;
    float diag_2 = sqrt(v_1 * v_1 - 2 * v_1 * h_3 * cos(alpha_) + h_3 * h_3);
    float beta_1 = acos((-h_2 * h_2 + h_1 * h_1 + diag_2 * diag_2) / (2 * h_1 * diag_2));
    float beta_2 = acos((-h_3 * h_3 + v_1 * v_1 + diag_2 * diag_2) / (2 * v_1 * diag_2));

    angles[2] = PI / 2 - (beta_1 + beta_2);

    angles[2] += angles[1];
}

void physicalToLogicalAngles(float angles[6]) {
    angles[2] -= angles[1];

    float beta    = angles[2];
    float beta_   = (PI / 2 - beta);
    float diag_1  = sqrt(v_1 * v_1 - 2 * v_1 * h_1 * cos(beta_) + h_1 * h_1);
    float alpha_1 = acos((-h_2 * h_2 + h_3 * h_3 + diag_1 * diag_1) / (2 * h_3 * diag_1));
    float alpha_2 = acos((-h_1 * h_1 + v_1 * v_1 + diag_1 * diag_1) / (2 * v_1 * diag_1));
    angles[2] = alpha_1 + alpha_2 - PI / 2;
}

// configure additional axis such as grippers and other devices
// pinNumber, maxAngularVel degree/sec, calibMin, calibMax, angleDegMin, angleDegMax, home position || angleDegMin/Max can be any value you
// want to map the min and max frequency to.
// It may be usefull to map a gripper to 0-100 based on the percentage of opening.
const float additionalAxisServoConfig[2][7] = {
    { pin_additional_servo_6, 160.00 * DEG_TO_RAD, 1888.00,    1122,    0.00 * DEG_TO_RAD,  1.00 * DEG_TO_RAD, 0 }, // mapped to 0-1, to
                                                                                                                    // open:1, closed:0
    { pin_additional_servo_7, 160.00 * DEG_TO_RAD,    1000, 2000.00,  -90.00 * DEG_TO_RAD, 90.00 * DEG_TO_RAD, 0 }
};

unsigned int pinMap[10] = { 8, 11, 12, 7, 0, 1, 0, 0, 0, 0 }; // todo
