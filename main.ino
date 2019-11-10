#define TASKER_MAX_TASKS 3
#include <Tasker.h>

#include "TM1637Display.h"
#include <DS1302.h>
#include <FastLED.h>
#include <SoftwareSerial.h>

#define BT_RX 2
#define BT_TX 3
#define BT_ENABLE 9
#define BT_ENABLE_INTERRUPT A0
SoftwareSerial BT(BT_RX, BT_TX); // RX | TX
#define BUFFER_SIZE 12
byte buffer[BUFFER_SIZE];
byte cursor = 0;
boolean newData = false;
#define START_MARKER 0x0F
#define END_MARKER 0xFF

#pragma region RTC
#define DS_CLK 4 // Clock
#define DS_DAT 5 // Data
#define DS_RST 6 // Reset
DS1302 RTC(DS_RST, DS_DAT, DS_CLK);
#pragma endregion

#pragma region Display
#define TM_CLK 7 // Clock
#define TM_DIO 8 // Data
TM1637Display display(TM_CLK, TM_DIO);
#pragma endregion

#pragma region LED
#define LED_DAT 11  // Data
#define LED_CLK 10  // Clock
#define NUM_LEDS 50 // # of Leds in strip
#define LEDS_PER_MERIDIAN 4

#define FADED_BRIGHTNESS 10
#define MAX_BRIGHTNESS 255

boolean isRunning = false;

int current_led = 0;
CRGB fadeColor(FADED_BRIGHTNESS, 0, 0);
CRGB brightColor(MAX_BRIGHTNESS, 0, 0);
CRGB darkColor(0, 0, 0);

CRGB leds[NUM_LEDS];
#pragma endregion

Tasker tasker;

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

#pragma pack(1)
typedef struct TimeStatus
{
    uint16_t year;
    byte month;
    byte date;
    byte hour;
    byte minute;
    byte second;
};
#pragma pop()
typedef struct Status
{
    byte check;
    byte led_state;
    byte led_brightness;
    TimeStatus time;
};

Time currentTime(2019, 0, 0, 0, 0, 0, Time::kSunday);
Status status;

void setup()
{
    pinMode(BT_ENABLE, OUTPUT);
    pinMode(BT_ENABLE_INTERRUPT, INPUT_PULLUP);

    pinMode(LED_DAT, OUTPUT);
    pinMode(LED_CLK, OUTPUT);

    digitalWrite(BT_ENABLE, HIGH);
    attachInterrupt(BT_ENABLE_INTERRUPT, awake_bt, LOW);

    BT.begin(9600);
    Serial.begin(9600);

    // setupBLE();

    status.check = 0;
    status.led_state = 1;
    status.led_brightness = 2; // N * 25%

    display.setBrightness(1, true);

    // Init WS2801 LED Strip
    FastLED.addLeds<WS2801, LED_DAT, LED_CLK, GRB>(leds, NUM_LEDS);
    set_brightness(status.led_brightness);


    show_led();
    tasker.setInterval(update_time, 5000);
    tasker.setInterval(change_region_task, 10000);
}

void loop()
{
    tasker.loop();
    recv();
}

void setupBLE()
{
    BT.println("AT+ROLE0");
    delay(100);
    // Flush buffer
    while (BT.available())
    {
    }
    delay(500);
    // sendCommand("AT+NAMEClock");
    BT.print("AT+NAME");
    BT.println("WClock A9F54");
    delay(2000);
}

void recv()
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    byte rc;

    while (BT.available() > 0 && newData == false)
    {
        rc = BT.read();

        if (recvInProgress == true)
        {
            buffer[ndx] = rc;
            ndx++;
            if (ndx >= BUFFER_SIZE - 1)
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
    if (command == CMD_POWER)
    {
        toggle_led();
        transmitStatus();
    }
    else if (command == CMD_BRIGHTNESS)
    {
        byte brightness = buffer[1];
        set_brightness(brightness);
        transmitStatus();
    }
    else if (command == CMD_TIME)
    {
        byte *p;
        p = (byte *)&status.time;
        for (byte i = 0; i < sizeof(TimeStatus); i++)
            p[i] = buffer[i + 1];

        currentTime.yr = status.time.year;
        currentTime.mon = status.time.month;
        currentTime.date = status.time.date;
        currentTime.hr = status.time.hour;
        currentTime.min = status.time.minute;
        currentTime.sec = status.time.second;
        RTC.time(currentTime);
        transmitStatus();
    }
    else if (command == CMD_BT_PWR)
    {
        // Irreversible
        digitalWrite(BT_ENABLE, LOW);
    }
    else if (command == CMD_STATUS)
    {
        update_time();
        transmitStatus();
    }
}

void transmitStatus()
{
    status.check = 1;
    byte structSize = sizeof(status);
    BT.write((byte *)&status, structSize);
    BT.println();
    BT.flush();
    status.check = 0;
}

void update_time()
{
    currentTime = RTC.time();
    status.time = {currentTime.yr, currentTime.mon, currentTime.date, currentTime.hr, currentTime.min, currentTime.sec};
    int time = 0;
    time += currentTime.hr * 100;
    time += currentTime.min;
    display.showNumberDecEx(time, 0b01000000, true);
}

void change_region_task()
{
    current_led++;
    if (current_led >= NUM_LEDS)
        current_led = 0;
}

void toggle_led()
{
    status.led_state = (status.led_state + 1) % 2;
    if (status.led_state == 0)
    {
        tasker.clearInterval(show_led_task);
        FastLED.clear();
        FastLED.show();
        isRunning = false;
    }
    else
    {
        show_led();
    }
}

void set_brightness(byte value)
{
    status.led_brightness = value;
    FastLED.setBrightness((byte)(value * 25 * 2.5));
    // FastLED.show();
}

void awake_bt()
{
    digitalWrite(BT_ENABLE, HIGH);
    delay(1000);
}

void show_led()
{
    if (!isRunning)
    {
        tasker.setInterval(show_led_task, 400);
        isRunning = true;
    }
}

void show_led_task()
{
    // cur + 1 --> DIM
    fadeTowardColor(leds[normalize(current_led + 3)], CRGB::Black, 1);
    fadeTowardColor(leds[normalize(current_led + 2)], fadeColor, 1);
    fadeTowardColor(leds[normalize(current_led + 1)], fadeColor, 1);
    // cur --> FULL
    fadeTowardColor(leds[current_led], brightColor, 2);
    // cur - 1 --> DIM
    fadeTowardColor(leds[normalize(current_led - 1)], fadeColor, 1);
    // cur - 2 --> DARK
    fadeTowardColor(leds[normalize(current_led - 2)], fadeColor, 1);
    fadeTowardColor(leds[normalize(current_led - 3)], CRGB::Black, 1);

    FastLED.show();
}

void nblendU8TowardU8(uint8_t &cur, const uint8_t target, uint8_t amount)
{
    if (cur == target)
        return;

    if (cur < target)
    {
        uint8_t delta = target - cur;
        delta = scale8_video(delta, amount);
        cur += amount;
    }
    else
    {
        uint8_t delta = cur - target;
        delta = scale8_video(delta, amount);
        cur -= amount;
    }
}

// Blend one CRGB color toward another CRGB color by a given amount.
// Blending is linear, and done in the RGB color space.
// This function modifies 'cur' in place.
CRGB fadeTowardColor(CRGB &cur, const CRGB &target, uint8_t amount)
{
    nblendU8TowardU8(cur.red, target.red, amount);
    return cur;
}

void fade()
{
    for (uint8_t li = 3; li < NUM_LEDS + 2; li++)
    {
        for (uint8_t i = 0; i < FADED_BRIGHTNESS; i++)
        {
            leds[li - 3].r -= 1;
            leds[li - 2].r -= 18;
            leds[li - 1].r += 18;
            leds[li].r += 1;
            FastLED.show();
            delay(25);
        }
        delay(250);
    }

    fade();
}

uint8_t normalize(int value)
{
    if (value < 0)
    {
        return value + NUM_LEDS;
    }
    else if (value >= NUM_LEDS)
    {
        return value - NUM_LEDS;
    }
    else
    {
        return value;
    }
}