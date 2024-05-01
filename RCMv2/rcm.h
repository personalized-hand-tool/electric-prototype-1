#ifndef _RCM_H_
#define _RCM_H_

#include <Arduino.h>
#include <JMotor.h> //https://github.com/joshua-8/JMotor

#define RCM_COMM_EWD 1
#define RCM_COMM_ROS 2

#include <Adafruit_NeoPixel.h>
#include <TMC7300.h>

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

int rsl_color = 0xf02d00; // orange

inline void setRSL(boolean on)
{
    if (on)
        pixels.fill(rsl_color);
    else
        pixels.fill(0);
    pixels.show();
}

#define batMonitorPin A3
#define uartPin MISO

#define motorsEnablePin MOSI

#define buttonPin 0

#define auxPin1 RX
#define auxPin2 SCK

#define port1Pin A0
#define port2Pin A1
#define port3Pin A2
#define port4Pin TX
//          PWM_CH, PIN
#define port1 0, port1Pin
#define port2 1, port2Pin
#define port3 2, port3Pin
#define port4 3, port4Pin

//            chip address, motor address
#ifndef MOTOR_DRIVER_BAUD
#define MOTOR_DRIVER_BAUD 110000
#endif

#define portAB uartPin, 0, MOTOR_DRIVER_BAUD
#define portCD uartPin, 1, MOTOR_DRIVER_BAUD

TMC7300IC TMC7300_AB = TMC7300IC(portAB);
TMC7300IC TMC7300_CD = TMC7300IC(portCD);

#define portA TMC7300_AB, 1
#define portB TMC7300_AB, 0
#define portC TMC7300_CD, 1
#define portD TMC7300_CD, 0

JMotorDriverTMC7300 motorDriverA = JMotorDriverTMC7300(portA);
JMotorDriverTMC7300 motorDriverB = JMotorDriverTMC7300(portB);
JMotorDriverTMC7300 motorDriverC = JMotorDriverTMC7300(portC);
JMotorDriverTMC7300 motorDriverD = JMotorDriverTMC7300(portD);

void setupMotors()
{
    pinMode(motorsEnablePin, OUTPUT);
    digitalWrite(motorsEnablePin, LOW);
    TMC7300_AB.begin();
    TMC7300_CD.begin();
    digitalWrite(motorsEnablePin, HIGH);
}
#ifndef OVERRIDE_DEFAULT_VOLTAGE_COMP
const int dacUnitsPerVolt = 350; // increasing this number decreases the calculated voltage
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt);
#endif

boolean enabled = false;
boolean wasEnabled = false;

#if RCM_COMM_METHOD == RCM_COMM_EWD

#ifndef EWDmaxWifiSendBufSize
#define EWDmaxWifiSendBufSize 200
#endif
#ifndef EWDmaxWifiRecvBufSize
#define EWDmaxWifiRecvBufSize 200
#endif

#include <ESP32_easy_wifi_data.h> //https://github.com/joshua-8/ESP32_easy_wifi_data >=v1.0.0

#elif RCM_COMM_METHOD == RCM_COMM_ROS

#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

unsigned long lastEnableSentMillis = 0;
boolean ROSCheckFail = false;

#include "rcmros.h"

#endif

#endif
