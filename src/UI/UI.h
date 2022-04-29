#include <Arduino.h>
#include "globals.h"
#include <GyverOLED.h>

namespace UI
{

    void Power(bool power);
    void Setup();
     void DrawSplash();
     void MainUIFrame();
     void SetWifi(uint8_t bars);
     void SetBattery(uint8_t bars);
     void SetIMUStatus(uint8_t imuID, bool Status);
     void SetImuCount(uint8_t IMUs);
     void SetMessage(uint8_t MessageID);

}