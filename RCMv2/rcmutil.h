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
void enabledRSL()
{
#ifndef OVERWRITE_RSL
    if (millis() % 500 < 250) {
        setRSL(true);
    } else {
        setRSL(false);
    }
#endif
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
void wifiDisconnectedRSL()
{
#ifndef OVERWRITE_RSL
    if (millis() % 1000 >= 100) {
        setRSL(true);
    } else {
        setRSL(false);
    }
#endif
}
void disabledRSL()
{
#ifndef OVERWRITE_RSL
    setRSL(true);
#endif
}


void setup()
{
    Serial.begin(115200);
    setupRSL();
    setupMotors();
    PowerOn();
    Disable();
#if RCM_COMM_METHOD == RCM_COMM_EWD
    configWifi();
    EWD::setupWifi(WifiDataToParse, WifiDataToSend);
#elif RCM_COMM_METHOD == RCM_COMM_ROS
    setupROS();
#endif
}

boolean connectedToWifi()
{
#if RCM_COMM_METHOD == RCM_COMM_EWD
    return EWD::wifiConnected;
#elif RCM_COMM_METHOD == RCM_COMM_ROS
    return !ROSCheckFail;
#endif
}
boolean connectionTimedOut()
{
#if RCM_COMM_METHOD == RCM_COMM_EWD
    return EWD::timedOut();
#elif RCM_COMM_METHOD == RCM_COMM_ROS
    return (millis() - lastEnableSentMillis) > rosWifiTimeout;
#endif
}

extern void ROSrun();

void loop()
{
#if RCM_COMM_METHOD == RCM_COMM_EWD
    EWD::runWifiCommunication();
#elif RCM_COMM_METHOD == RCM_COMM_ROS
    ROSrun();
#endif
    if (!connectedToWifi() || connectionTimedOut()) {
        enabled = false;
    }
    Always();
    if (enabled && !wasEnabled) {
#if RCM_HARDWARE_VERSION == RCM_BYTE_V2 || RCM_HARDWARE_VERSION == RCM_NIBBLE_V1
#ifndef RCM_BYTE_DO_NOT_USE_SAFE_DISABLE
        digitalWrite(motorsEnablePin, HIGH);
#endif
#endif

        Enable();
    }
    if (!enabled && wasEnabled) {
        Disable();

#if RCM_HARDWARE_VERSION == RCM_BYTE_V2 || RCM_HARDWARE_VERSION == RCM_NIBBLE_V1
#ifndef RCM_BYTE_DO_NOT_USE_SAFE_DISABLE
        digitalWrite(motorsEnablePin, LOW);
#endif
#endif
    }
    if (enabled) {
        Enabled();
        enabledRSL(); //        digitalWrite(ONBOARD_LED, millis() % 500 < 250); // flash, enabled
    } else {
        if (!connectedToWifi())
            wifiFailRSL(); //            digitalWrite(ONBOARD_LED, millis() % 1000 <= 100); // short flash, wifi connection fail
        else if (connectionTimedOut())
            wifiDisconnectedRSL(); //            digitalWrite(ONBOARD_LED, millis() % 1000 >= 100); // long flash, no driver station connected
        else
            disabledRSL(); //            digitalWrite(ONBOARD_LED, HIGH); // on, disabled
    }
    wasEnabled = enabled;
}

#endif // RCMUTIL_H
