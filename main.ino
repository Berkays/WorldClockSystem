#define TASKER_MAX_TASKS 5
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
#define LED_DAT 11 // Data
#define LED_CLK 10 // Clock
#define NUM_LEDS 50 // # of Leds in strip
#define LEDS_PER_MERIDIAN 4

#define FADED_BRIGHTNESS 20
#define MAX_BRIGHTNESS 180

int current_led = 0;
CRGB fadeColor(FADED_BRIGHTNESS, 0, 0);
CRGB brightColor(MAX_BRIGHTNESS, 0, 0);
CRGB darkColor(0, 0, 0);

CRGB leds[NUM_LEDS];

byte LedPwr = 0;
byte LedBrightness = 0;
#pragma endregion

Tasker tasker;

/*
 48 Leds divided into 12 meridians.
 48/12 = 4 Leds per meridians.
 UTC -7: Los Angeles
 UTC -4: New York
 UTC -3: Rio de Janeiro
 UTC +0:
 UTC +1: Londra 
 UTC +2: Paris
 UTC +3: Moskova/Istanbul
 UTC +4: Dubai
 UTC +5.5: Hindistan
 UTC +8: Singapur/Pekin
 UTC +9: Tokyo
 UTC +10: Sidney
 */

/*
 Comm protocol:

 Set Power
 Cmd: 0x01 1 byte
 State: 1-2 byte

Set Brightness
 Cmd: 0x02 1 byte
 Brightness: 0-254 1 byte

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

Set Name
 Cmd: 0x05 1 byte
 Name: ? byte

Get Status
 Cmd: 0x06 1 byte
 Power state: 0-1 1 byte
 Brightness: 0-254 1 byte
 Time : struct 8 byte
 Name : string ? byte

 After each CMD
 Check: 0xAA 1 byte
 NewValue: ???????
*/

#define CMD_POWER 0x01
#define CMD_BRIGHTNESS 0x02
#define CMD_TIME 0x03
#define CMD_BT_PWR 0x04
#define CMD_NAME 0x05
#define CMD_STATUS 0x06
#define CHECK 0xAA

#pragma pack(1)
typedef struct TimeStatus
{
    uint16_t year;
    byte month;
    byte date;
    byte hour;
    byte minute;
    byte second;
    byte dayOfWeek;
};
#pragma pop()
typedef struct Status
{
    byte led_state;
    byte led_brightness;
    TimeStatus time;
};

Time currentTime(2099, 1, 1, 0, 0, 0, Time::kSunday);
Status status;

void setup()
{
    pinMode(BT_ENABLE, OUTPUT);
    pinMode(BT_ENABLE_INTERRUPT, INPUT_PULLUP);

    pinMode(LED_DAT, OUTPUT);
    pinMode(LED_CLK, OUTPUT);
    
    digitalWrite(BT_ENABLE, HIGH);
    // attachInterrupt(BT_ENABLE_INTERRUPT, awake_bt, LOW);

    BT.begin(9600);
    Serial.begin(9600);

    setupBLE();

    display.setBrightness(1, true);

    // Init WS2801 LED Strip
    FastLED.addLeds<WS2801, LED_DAT, LED_CLK, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(96);
    FastLED.clear();
    test_strip();

    tasker.setInterval(update_time, 5000);
    tasker.setInterval(check_data, 1000);
}

void loop()
{
    tasker.loop();
    recv();
    test_strip();
}

void setupBLE()
{
    mySerial.println("AT");
    delay(100);
    mySerial.println("AT+ROLE0");
    delay(100);
    mySerial.println("AT+UUID0xFF00");
    delay(100);
    mySerial.println("AT+CHAR0xFF01");
    delay(100);
    mySerial.println("AT+NAMEClock");
    delay(100);
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
            if (rc != END_MARKER)
            {
                buffer[ndx] = rc;
                ndx++;
                if (ndx >= BUFFER_SIZE)
                {
                    ndx = BUFFER_SIZE - 1;
                }
            }
            else
            {
                buffer[ndx] = '\0'; //terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
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
    Serial.println("CMD");
    byte command = buffer[0];
    Serial.println(command);
    if (command == CMD_POWER)
    {
        byte state = buffer[1];
        if (state == 1)
            open_led();
        else
            close_led();
        BT.println(CHECK);
    }
    else if (command == CMD_BRIGHTNESS)
    {
        byte brightness = buffer[1];
        Serial.print("Led brighntess:");
        Serial.println(brightness);
        set_brightness(brightness);
        BT.println(CHECK);
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
        currentTime.day = status.time.dayOfWeek;
        RTC.time(currentTime);
        Serial.println("RTC Time:");
        Serial.println(status.time.hour);
        BT.println(CHECK);
    }
    else if (command == CMD_BT_PWR)
    {
        digitalWrite(BT_ENABLE, LOW);
    }
    else if (command == CMD_NAME)
    {
        // byte name = buffer[1];
        // Serial.println("Set name");
        //TODO: Set name
        BT.println(CHECK);
    }
    else if (command == CMD_STATUS)
    {
        currentTime = RTC.time();
        status.time = {currentTime.yr, currentTime.mon, currentTime.date, currentTime.hr, currentTime.min, currentTime.sec, (byte)currentTime.day};
        Serial.println("Get status");
        Serial.println(status.led_state);
        Serial.println(status.led_brightness);
        Serial.println(status.time.hour);
        BT.println(CHECK);
    }
}

void update_time()
{
    Time ct = RTC.time();

    int time = 0;
    time += ct.hr * 100;
    time += ct.min;
    display.showNumberDecEx(time, 0b01000000, true);
}

void test_strip()
{
    for (uint8_t i = 0; i < 255; i++)
    {
        // cur + 1 --> DIM
        fadeTowardColor(leds[normalize(current_led + 3)], CRGB::Black, 1);
        fadeTowardColor(leds[normalize(current_led + 2)], fadeColor, 1);
        fadeTowardColor(leds[normalize(current_led + 1)], fadeColor, 1);
        // cur --> FULL
        fadeTowardColor(leds[current_led], brightColor, 2);
        // cur - 1 --> DIM
        fadeTowardColor(leds[normalize(current_led - 1)], fadeColor, 2);
        // cur - 2 --> DARK
        // leds[normalize(current_led - 2)] = CRGB::Black;
        // leds[normalize(current_led - 1)] = fadeColor;
        // leds[normalize(current_led)] = brightColor;
        // leds[normalize(current_led + 1)] = fadeColor;
        fadeTowardColor(leds[normalize(current_led - 2)], fadeColor, 1);
        fadeTowardColor(leds[normalize(current_led - 3)], CRGB::Black, 1);

        FastLED.show();
        FastLED.delay(10);
    }

    FastLED.delay(40);
    current_led++;
    if (current_led >= NUM_LEDS)
        current_led = 0;
}

void close_led()
{
    status.led_state = 0;
    FastLED.clear();
}

void open_led()
{
    status.led_state = 1;
    //TODO: Fastleds.start();
}

void set_brightness(byte value)
{
    status.led_brightness = value;
    FastLED.setBrightness(value);
}

void awake_bt()
{
    digitalWrite(BT_ENABLE, HIGH);
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