#ifndef RCMUTIL_H
#define RCMUTIL_H

// contains functions common to all RCMv2 projects

#include <Arduino.h>

#include "rcm.h"

extern void PowerOn();
extern void Enable();
extern void Disable();
extern void Enabled();
extern void Always();
extern void configWifi();
extern void WifiDataToParse();
extern void WifiDataToSend();
extern void setupMotors();

void setupRSL()
{
    pixels.begin();
}
void wifiFailRSL()
{
#ifndef OVERWRITE_RSL
    if (millis() % 1000 <= 100) {
        setRSL(true);
    } else {
        setRSL(false);
    }
#endif
}

void setup()
{
    Serial.begin(115200);
    setupRSL();
    setupMotors();
    PowerOn();
}

extern void ROSrun();

void loop()
{
    Always();
    wifiFailRSL(); //            digitalWrite(ONBOARD_LED, millis() % 1000 <= 100); // short flash, wifi connection fail
}

#endif // RCMUTIL_H
