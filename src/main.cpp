/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "Wire.h"
#include "ota.h"
#include "sensors/SensorManager.h"
#include "configuration/Configuration.h"
#include "network/network.h"
#include "globals.h"
#include "credentials.h"
#include <i2cscan.h>
#include "serial/serialcommands.h"
#include "LEDManager.h"
#include "status/StatusManager.h"
#include "batterymonitor.h"
#include "logging/Logger.h"

#if !ESP8266
#include "esp_wifi.h"
#include "esp_sleep.h"
#endif

SlimeVR::Logging::Logger logger("SlimeVR");
SlimeVR::Sensors::SensorManager sensorManager;
SlimeVR::LEDManager ledManager(LED_PIN);
SlimeVR::Status::StatusManager statusManager;
SlimeVR::Configuration::Configuration configuration;

int sensorToCalibrate = -1;
bool blinking = false;
unsigned long blinkStart = 0;
unsigned long loopTime = 0;
bool secondImuActive = false;
BatteryMonitor battery;

void setup()
{
    setCpuFrequencyMhz(80);
    pinMode(32, OUTPUT);
    digitalWrite(32, HIGH);
    pinMode(33, OUTPUT);
    digitalWrite(33, LOW);

    delay(1000);

//    pinMode(13, OUTPUT);
//    digitalWrite(13, LOW);

//    // need power management
//    battery.Setup();
//    if (battery.Loop(true)) {
//        pinMode(13, INPUT);
//        //esp_sleep_enable_ext1_wakeup(1ULL<<39, ESP_EXT1_WAKEUP_ANY_HIGH);
//        //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
//        esp_deep_sleep_start();
//    }
//    //add pin power (mosfet) for imus

    Serial.begin(serialBaudRate);
    Serial.println();
    Serial.println();
    Serial.println();

    logger.info("SlimeVR v" FIRMWARE_VERSION " starting up...");

    //wifi_set_sleep_type(NONE_SLEEP_T);

    statusManager.setStatus(SlimeVR::Status::LOADING, true);

    ledManager.setup();
    configuration.setup();

    SerialCommands::setUp();

    // Setup i2c
    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL_A);
#ifdef ESP8266
    Wire.setClockStretchLimit(150000L); // Default stretch limit 150mS
#endif
    Wire.setClock(I2C_SPEED);

    sensorManager.setup();

    Network::setUp();
    OTA::otaSetup(otaPassword);

    statusManager.setStatus(SlimeVR::Status::LOADING, false);

    sensorManager.postSetup();

    loopTime = micros();
}

void loop()
{
    SerialCommands::update();
    OTA::otaUpdate();
    sensorManager.update();
    Network::update(sensorManager.get());
//    if (battery.Loop()) {
//        pinMode(13, INPUT);
//        sensorManager.sleepSensors(true);
//        sensorManager.setPinsInput();
//        esp_wifi_stop();
//        //esp_sleep_enable_ext1_wakeup(1ULL<<39, ESP_EXT1_WAKEUP_ANY_HIGH);
//        //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
//        esp_deep_sleep_start();
//    }
    ledManager.update();
}
