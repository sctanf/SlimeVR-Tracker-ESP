#include <UI\UI.h>
namespace UI
{

    GyverOLED<SSD1306_128x32> oled;

// 'interslime_l', 48x7px
const unsigned char interslime_l [] PROGMEM = {
	0x7f, 0x45, 0x7f, 0x43, 0x7b, 0x7b, 0x47, 0x7f, 0x7b, 0x41, 0x5b, 0x7f, 0x67, 0x4b, 0x53, 0x77, 
	0x7f, 0x43, 0x77, 0x7b, 0x7f, 0x00, 0x28, 0x2c, 0x34, 0x14, 0x00, 0x3e, 0x00, 0x3a, 0x00, 0x3c, 
	0x04, 0x3c, 0x04, 0x38, 0x00, 0x18, 0x34, 0x2c, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'interslime_u', 48x7px
const unsigned char interslime_u [] PROGMEM = {
	0x7f, 0x5d, 0x41, 0x5d, 0x7f, 0x41, 0x7b, 0x77, 0x41, 0x7f, 0x7d, 0x41, 0x7d, 0x7f, 0x41, 0x55, 
	0x5d, 0x7f, 0x41, 0x6d, 0x6d, 0x53, 0x7f, 0x00, 0x24, 0x2a, 0x2a, 0x12, 0x00, 0x3e, 0x20, 0x20, 
	0x00, 0x22, 0x3e, 0x22, 0x00, 0x3e, 0x04, 0x08, 0x04, 0x3e, 0x00, 0x3e, 0x2a, 0x22, 0x00, 0x00
};

    void Power(bool power) {
        Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL_A);
        oled.setPower(power);
    }
    void Setup()
    {
        Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL_A);
        oled.init();
        oled.clear();
        oled.update();
    }

    void MainUIFrame()
    {
        Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL_A);
        oled.clear();
        //logo
        oled.drawBitmap(0, 0, interslime_u, 48, 7, BITMAP_NORMAL, BUF_ADD);
        //wifi
        //for (uint8_t i=0; i<5; i++)
        //    oled.dot(102 + (i * 2), 5, true);
        //battery
        //oled.rect(115, 1, 126, 5, true);
        //oled.rect(127, 2, 127, 4, true);
        //oled.rect(116, 2, 125, 4, false);
        //sensors
        for (uint8_t i=0; i<16; i++)
            oled.dot(19 + (i * 6), 15, true);
        oled.update();
    }

    void SetWifi(uint8_t bars)
    {
        Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL_A);
        oled.rect(101, 1, 110, 5, false);
        for (uint8_t i=0; i<5; i++)
            oled.dot(102 + (i * 2), 5, true);
        for (uint8_t i=0; i<bars; i++)
            oled.rect(101 + (i * 2), 5 - i, 102 + (i * 2), 5, true);
        oled.update();
    }

    void SetBattery(uint8_t bars)
    {
        Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL_A);
        oled.rect(115, 1, 126, 5, true);
        oled.rect(127, 2, 127, 4, true);
        oled.rect(116, 2, 125, 4, false);
        oled.rect(115, 2, 115 + bars, 4, true);
        oled.update();
    }

    void SetIMUStatus(uint8_t imuID, bool Status)
    {
        Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL_A);
        oled.rect(17 + (imuID * 6), 12, 21 + (imuID * 6), 18, (Status) ? true : false);
        oled.rect(17 + (imuID * 6), 15, 21 + (imuID * 6), 15, true);
        oled.update();
    }

    void SetImuCount(uint8_t IMUs)
    {
        Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL_A);
        oled.setScale(1);
        oled.rect(0, 24, 127, 31, false);
        oled.printf("%u  Sensors Active", IMUs);
        oled.setCursorXY(15, 19);
        oled.update();
    }

    void SetMessage(uint8_t MessageID)
    {
        Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL_A);
        oled.setScale(1);
        oled.rect(0, 24, 127, 31, false);

        switch (MessageID)
        {
        case 1:
            oled.setCursorXY(0, 24);
            oled.print("Scanning Sensors");
            break;

        case 2:
            oled.setCursorXY(0, 24);
            oled.print("Connecting To WIFI");
            break;

        case 3:
            oled.setCursorXY(0, 24);
            oled.print("WIFI Connect FAILED");
            break;

        case 4:
            oled.setCursorXY(0, 24);
            oled.print("Connecting To Server");
            break;

        case 5:
            oled.setCursorXY(0, 24);
            oled.print("Server lost");
            break;

        case 6:
            oled.setCursorXY(0, 24);
            oled.print("Sending Data");
            break;

        }

        oled.update();
    }

}