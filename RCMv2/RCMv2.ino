//   This program is template code for programming small esp32 powered wifi controlled robots.
//   https://github.com/personalized-hand-tool/electric-prototype-1
//   https://github.com/rcmgames/RCMv2
//   for information see this page: https://github.com/RCMgames
//   runs on an ESP32-C3 QT Py

#include "ICM20948_helper.h"

/**
uncomment one of the following lines depending on which communication method you want to use
*/
#define RCM_COMM_METHOD RCM_COMM_EWD // use the normal communication method for RCM robots
// #define RCM_COMM_METHOD RCM_COMM_ROS // use the ROS communication method

#include "rcm.h" //defines pins

// set up motors and anything else you need here
// See this page for how to set up servos and motors for each type of RCM board:
// https://github.com/RCMgames/useful-code/tree/main/boards
// See this page for information about how to set up a robot's drivetrain using the JMotor library
// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain
// all the servo drivers
JMotorDriverEsp32Servo servo1Driver = JMotorDriverEsp32Servo(port1);
JMotorDriverEsp32Servo servo2Driver = JMotorDriverEsp32Servo(port2);
JMotorDriverEsp32Servo servo3Driver = JMotorDriverEsp32Servo(port3);
JMotorDriverEsp32Servo servo4Driver = JMotorDriverEsp32Servo(port4);

// all the motor drivers
JMotorDriverTMC7300 motor1Driver = JMotorDriverTMC7300(portA);
JMotorDriverTMC7300 motor2Driver = JMotorDriverTMC7300(portB);
JMotorDriverTMC7300 motor3Driver = JMotorDriverTMC7300(portC);
JMotorDriverTMC7300 motor4Driver = JMotorDriverTMC7300(portD);

// variables for the drivers
float servo1Val = 0;
float servo2Val = 0;
float servo3Val = 0;
float servo4Val = 0;

float motor1Val = 0;
float motor2Val = 0;
float motor3Val = 0;
float motor4Val = 0;

void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
    nibbleSetupImu();
    delay(10);
    servo4Driver.enable();
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions should be put here
    // (but only the "top level", for example if you call drivetrainController.run() you shouldn't also call leftMotorController.run())
    runIMU();
    rsl_color = voltageComp.getSupplyVoltage() < 7 ? 0xdd0000 : 0x000000;
        servo4Driver.set(-imu.roll / 90);
    delay(1);
}

#include "rcmutil.h"
