#define ANALOGBUTTONS_MAX_SIZE 4
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <RTClib.h>
#include <Adafruit_NeoPixel.h>
#include <AnalogButtons.h>
#include <TM1637Display.h>
#include "pins.h"

// #define BLUEOOTH_SETUP
#define BLUETOOTH_NAME_CMD "AT+NAME=WClock A9F4"
#define USE_RTC

#pragma region BLUETOOTH
SoftwareSerial BT(ARDUINO_RX, ARDUINO_TX);
#define BUFFER_SIZE 5
byte buffer[BUFFER_SIZE];
byte cursor = 0;
boolean newData = false;
#define START_MARKER 0x0F
#pragma endregion

#pragma region TIME
DS1302 rtc(RTC_CE, RTC_SCK, RTC_IO);
#define TIME_PERIOD 1000

#ifdef USE_RTC
DateTime lastUpdate;
#else
unsigned long lastMillis = 0;
#endif
#pragma endregion

#pragma region LED
#define NUM_LEDS 96
#define TWO_PI 6.2831853072
#define LED_INITIAL_BRIGHTNESS 4 // 1-2-3-4 * 25%
#define LED_THRESHOLD 10         // Divided by 10
#define LED_MAP_MIN 10           // Divided by 10
double INCREMENT = 1440.0 / (double)NUM_LEDS;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_DAT, NEO_GRB + NEO_KHZ800);
boolean isRunning = false;
double minuteCounter = 0;
#pragma endregion

#pragma region BUTTONS
AnalogButtons analogButtons(ANALOG_PIN, INPUT);
Button b1 = Button(1000, &btn_setSpeed);
Button b2 = Button(700, &btn_setSpeed);
Button b3 = Button(400, &btn_setBrightness);
Button b4 = Button(100, &btn_setBrightness);
#pragma endregion

#pragma region DISPLAY
#define BLINK_DELAY 500
TM1637Display display(DISPLAY_CLK, DISPLAY_DATA);
boolean isSetMode_hour = false;
boolean isSetMode_min = false;
#pragma endregion

/*
 Comm protocol:

 Set Power
 Cmd: 0x01 1 byte
 State: 0-1 1 byte

Set Brightness
 Cmd: 0x02 1 byte
 Brightness: 1-4 1 byte 25% Step

 Set Time
 Cmd: 0x03 1 byte 
 Y: Year 2 byte //TODO: FF Endmarker
 M: 01-12 1 byte
 D: 01-31 1 byte
 H: 00-24 1 byte
 M: 00-60 1 byte
 S: 00-60 1 byte
 DayOfWeek: 01-07 1 byte enum

Set Disable BT-Power
 Cmd: 0x04 1 byte

Get Status
 Cmd: 0x06 1 byte
 Power state: 0-1 1 byte
 Brightness: 0-254 1 byte
 Time : struct 8 byte
 Name : string ? byte
*/

#define CMD_POWER 0x01
#define CMD_BRIGHTNESS 0x02
#define CMD_TIME 0x03
#define CMD_BT_PWR 0x04
#define CMD_NAME 0x05
#define CMD_STATUS 0x06
#define CMD_SPEED 0x07     // FOR DEBUG
#define CMD_MAPMIN 0x08    // FOR DEBUG
#define CMD_THRESHOLD 0x09 // FOR DEBUG
#define CMD_LED_COUNT 0x10 // FOR DEBUG

#pragma pack(1)
typedef struct Status
{
    uint8_t check;
    uint8_t led_state;
    uint8_t led_brightness;
    uint8_t speed;     // FOR DEBUG
    uint8_t threshold; // FOR DEBUG
    uint8_t mapMin;    // FOR DEBUG
    uint8_t ledCount;  // FOR DEBUG
    uint8_t hour;
    uint8_t minute;
    uint16_t second;
};
#pragma pop()

Status status;

void setup()
{
    Serial.begin(9600);
    BT.begin(38400);
    delay(1000);
#ifdef BLUEOOTH_SETUP
    bt_setup();
#else
    while (BT.available())
        Serial.print(BT.readString());
    BT.println("AT+E=0");
    if (BT.available())
    {
        while (BT.available())
            Serial.print(BT.readString());

        Serial.println();
    }
    delay(1000);
    BT.println("AT+NOTIFY=0");
    if (BT.available())
    {
        while (BT.available())
            Serial.print(BT.readString());

        Serial.println();
    }
    delay(1000);
    BT.println("AT+FORCEC=0");
    while (BT.available())
        Serial.print(BT.read());
    delay(1000);
#endif

    analogButtons.add(b1);
    analogButtons.add(b2);
    analogButtons.add(b3);
    analogButtons.add(b4);

    status.check = 0;
    status.led_state = 1;
    status.led_brightness = LED_INITIAL_BRIGHTNESS; // N * 25%
    status.speed = 0;                               // FOR DEBUG
    status.threshold = LED_THRESHOLD;               // FOR DEBUG
    status.ledCount = NUM_LEDS;                     // FOR DEBUG
    status.mapMin = LED_MAP_MIN;                    // FOR DEBUG

#ifdef USE_RTC
    rtc.begin();
    Serial.println(rtc.now().second());
    delay(2000);
    Serial.println(rtc.now().second());
#else
    status.hour = 12;
    status.minute = 0;
    status.second = 0;
#endif

    strip.begin();
    strip.setBrightness(255);
    strip.clear();
    strip.show();

    led_start_animation();
    led_stop_animation();
    strip.clear();

#ifndef USE_RTC
    lastMillis = millis();
#endif
    isRunning = true;
}

void loop()
{
#ifdef USE_RTC
    updateSecond_rtc();
#else
    updateSecond();
#endif
    analogButtons.check();
    recv();
}

#ifndef USE_RTC
void updateSecond()
{
    unsigned long currentMillis = millis();
    long gap = currentMillis - lastMillis;
    if (gap > TIME_PERIOD)
    {
        lastMillis = currentMillis;
        unsigned long gapInSeconds = gap / 1000;
        if (status.speed == 0)
        {
            status.second += gapInSeconds;
        }
        else if (status.speed == 1)
        {
            status.second += (60 * gapInSeconds);
        }
        else if (status.speed == 2)
        {
            status.second += (600 * gapInSeconds);
        }

        uint16_t extra_min = (uint16_t)status.second / 60;
        status.second = (uint16_t)status.second % 60;

        status.minute += extra_min;
        minuteCounter += extra_min;

        if (minuteCounter >= 1440)
            minuteCounter -= 1440;

        if (status.minute >= 60)
        {
            status.hour++;
            status.minute = status.minute % 60;

            if (status.hour == 24)
                status.hour = 0;
        }

        if (isRunning)
            coordinate();
    }
}
#else
void updateSecond_rtc()
{
    DateTime now = rtc.now();
    TimeDelta diff = now - lastUpdate;
    if (status.speed == 1)
    {
        // Add 1 minute
        now = now + TimeDelta(diff.totalseconds() * 60);
    }
    else if (status.speed == 2)
    {
        // Add 10 minute
        now = now + TimeDelta(diff.totalseconds() * 600);
    }

    minuteCounter = (uint16_t)now.hour() * (uint16_t)now.minute();

    if (isRunning)
        coordinate();
}
#endif

void recv()
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    byte rc;
    while (BT.available() > 0 && newData == false)
    {
        rc = BT.read();
        Serial.write(rc);
        Serial.println();
        if (recvInProgress == true)
        {
            buffer[ndx] = rc;
            ndx++;
            if (ndx >= BUFFER_SIZE)
            {
                recvInProgress = false;
                ndx = 0;
                newData = true;
                check_data();
            }
        }

        else if (rc == START_MARKER)
        {
            recvInProgress = true;
        }
    }
}

void check_data()
{
    if (newData == true)
    {
        parse_data();
        newData = false;
    }
}

void parse_data()
{
    byte command = buffer[0];
    Serial.println(command);
    if (command == CMD_POWER)
    {
        if (status.led_state == 0)
            status.led_state = 1;
        else
            status.led_state = 0;
        transmitStatus();
        toggle_led();
    }
    else if (command == CMD_BRIGHTNESS)
    {
        byte brightness = buffer[1];
        if (brightness == 0)
            brightness = 1;
        else if (brightness > 4)
            brightness = 4;
        status.led_brightness = brightness;
        transmitStatus();
    }
    else if (command == CMD_TIME)
    {
#ifdef USE_RTC
        DateTime now = rtc.now();
        now.sethour(buffer[1]);
        now.setminute(buffer[2]);
        minuteCounter = (double)now.hour() * (double)now.minute();
#else
        status.hour = buffer[1];
        status.minute = buffer[2];
        status.second = 0;
        minuteCounter = status.minute * status.hour;
        if (minuteCounter >= 1440)
            minuteCounter = 0;
        else if (minuteCounter < 0)
            minuteCounter = 0;
#endif

        transmitStatus();
    }
    // else if (command == CMD_BT_PWR)
    // {
    //     // Irreversible
    //     // digitalWrite(BT_ENABLE, LOW);
    // }
    else if (command == CMD_SPEED)
    {
        status.speed = buffer[1];
        if (status.speed > 2)
            status.speed = 2;
        transmitStatus();
    }
    else if (command == CMD_THRESHOLD)
    {
        status.threshold = buffer[1];
        if (status.threshold >= 12)
            status.threshold = 8;
        transmitStatus();
    }
    else if (command == CMD_MAPMIN)
    {
        status.mapMin = buffer[1];
        if (status.mapMin > 12)
            status.mapMin = 8;
        transmitStatus();
    }
    else if (command == CMD_LED_COUNT)
    {
        status.ledCount = buffer[1];
        updateLedCount();
        transmitStatus();
    }
    else if (command == CMD_STATUS)
    {
        transmitStatus();
    }
}

void transmitStatus()
{
    status.check = 1;
    byte structSize = sizeof(status);
    BT.write((byte *)&status, structSize);
    BT.flush();
    status.check = 0;
}

void toggle_led()
{
    if (status.led_state == 1)
        show_led();
    else
        close_led();
}

void updateLedCount()
{
    isRunning = false;
    strip = Adafruit_NeoPixel(status.ledCount, LED_DAT, NEO_GRB + NEO_KHZ800);
    INCREMENT = 1440.0 / (double)status.ledCount;
    strip.begin();
    strip.show();
    isRunning = true;
}

void awake_bt()
{
    // digitalWrite(BT_ENABLE, HIGH);
    // delay(1000);
}

void show_led()
{
    led_start_animation();

    strip.clear();
    isRunning = true;
}

void close_led()
{
    led_stop_animation();

    isRunning = false;
}

void coordinate()
{
    const double mapped_t = mapf(minuteCounter, 0.0, 1440.0, 0.0, TWO_PI);
    const byte max_brightness = (byte)(62 * status.led_brightness);
    const double threshold = (double)status.threshold / 10.0;
    const double mapMin = (double)status.mapMin / 10.0;
    for (byte i = 0; i < status.ledCount; i++)
    {
        double ledVal = (double)i * INCREMENT;
        double mapped_y = mapf(ledVal, 0.0, 1440 - INCREMENT, 0.0, TWO_PI);
        double cosVal = cos(mapped_t - mapped_y) + 1;
        byte rgb = 0;
        if (cosVal < threshold)
            rgb = 0;
        else
            rgb = (byte)mapf(cosVal, mapMin, 2.0, 1.0, max_brightness);

        strip.setPixelColor(i, rgb, rgb, rgb);
    }
    strip.show();
}

double mapf(double val, double in_min, double in_max, double out_min, double out_max)
{
    return out_min + ((val - in_min) * (out_max - out_min) / (in_max - in_min));
}

// void coordinate()
// {
//     for (byte i = 0; i < NUM_LEDS; i++)
//     {
//         double sinVal = sin(0.004363325 * ((minuteCounter + (i * 48)))) * 1785;

//         if (sinVal > 255)
//             sinVal = 255;
//         if (sinVal < 0)
//             sinVal = 0;

//         byte val = (byte)(sinVal);
//         strip.setPixelColor(i, val, val, val);
//     }
//     strip.show();
// }

void led_start_animation()
{
    for (byte v = 0; v < 254; v++)
    {
        for (byte i = 0; i < status.ledCount; i++)
        {
            strip.setPixelColor(i, 0, 0, v);
        }
        strip.show();
        delay(5);
    }
}

void led_stop_animation()
{
    for (byte v = 254; v > 0; v--)
    {
        for (byte i = 0; i < status.ledCount; i++)
        {
            strip.setPixelColor(i, 0, 0, v - 1);
        }
        strip.show();
        delay(5);
    }
}

void btn_setBrightness()
{
    status.led_brightness = (status.led_brightness % 4) + 1;
}

void btn_setSpeed()
{
    status.speed++;
    if (status.speed > 2)
        status.speed = 0;
}

void btn_setHour()
{
    if (isSetMode_hour == false)
    {
        isSetMode_hour = true;
    }
}

void btn_setMinute()
{
    status.speed++;
    if (status.speed > 2)
        status.speed = 0;
}

#ifdef BLUEOOTH_SETUP
void bt_setup()
{
    // Set AT Mode
    BT.println("AT+FORCEC=1");
    delay(10);
    BT.println("AT+SAVE");
    delay(100);
    BT.println("AT+RESET");
    delay(2000);

    BT.println("AT+SLEEP=0");
    delay(10);
    BT.println("AT+SAVE");
    delay(100);

    // Set module BT name
    BT.println(BLUETOOTH_NAME_CMD);
    delay(10);
    BT.println("AT+SAVE");
    delay(100);
    BT.println("AT+RESET");
    delay(3000);

    // Disable echo
    BT.println("AT+E=0");
    delay(10);
    BT.println("AT+SAVE");
    delay(100);

    // Set broadcast parameters
    // BT.println("AT+BTPARAM=3200,70,90,0,200");
    // delay(10);
    // BT.println("AT+SAVE");
    // delay(100);

    // Disable notify
    BT.println("AT+NOTIFY=0");
    delay(10);
    BT.println("AT+SAVE");
    delay(100);

    // Set automatic pairing
    BT.println("AT+PAIRM=2");
    delay(10);
    BT.println("AT+SAVE");
    delay(100);
    BT.println("AT+RESET");
    delay(3000);

    // Set peripheral mode
    BT.println("AT+ROLE=2");
    delay(10);
    BT.println("AT+SAVE");
    delay(100);
    BT.println("AT+RESET");
    delay(3000);

    BT.flush();
    while (1)
    {
    }
}
#endif