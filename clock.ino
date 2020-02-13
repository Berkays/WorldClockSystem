
#define ANALOGBUTTONS_MAX_SIZE 4
#include <SoftwareSerial.h>
#include <DS1302.h>
#include <TM1637Display.h>
#include "AnalogButtons.h"
#include "Pins.h"
#include "leds.h"
#include "functions.h"

// #define BLUEOOTH_SETUP
// #define BUTTON_SETUP
// #define BLUETOOTH_NAME_CMD F("AT+NAME=WClock A9F4")
#define USE_RTC

#pragma region BLUETOOTH
SoftwareSerial BT(ARDUINO_RX, ARDUINO_TX);
#define BUFFER_SIZE 3
#define START_MARKER 0x0F
byte buffer[BUFFER_SIZE];
boolean newData = false;
#pragma endregion

#pragma region TIME
#define TIME_PERIOD 2000
unsigned long lastMillis;

#ifdef USE_RTC
DS1302 rtc(RTC_CE, RTC_IO, RTC_SCK);
DateTime lastUpdate;
#endif
#pragma endregion

#pragma region LED
#define NUM_LEDS 230
#define LED_INITIAL_BRIGHTNESS 2 // 1-2-3-4 * 25%
#define LED_THRESHOLD 1          // Half the leds
#define LED_MAP_MIN 1
#define STEP_COUNT 1440.0 // 24 * 60
double INCREMENT = STEP_COUNT / (double)NUM_LEDS;

boolean isRunning = false;
double minuteCounter = 0;
#pragma endregion

#pragma region BUTTONS
AnalogButtons analogButtons(ANALOG_PIN, INPUT);
Button b1 = Button(0, &btn_setSpeed);
Button b2 = Button(320, &btn_setLedCount);
Button b3 = Button(485, &btn_setBrightness);
Button b4 = Button(588, &btn_setTemperature);
#pragma endregion

#pragma region DISPLAY
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
#define CMD_SPEED 0x07  // FOR DEBUG
#define CMD_MAPMIN 0x08 // FOR DEBUG
#define CMD_TEMPERATURE 0x09
#define CMD_LED_COUNT 0x10 // FOR DEBUG

#pragma pack(1)
typedef struct Status
{
    uint8_t check;
    uint8_t led_state;
    uint8_t led_brightness;
    uint8_t led_temperature;
    uint8_t hour;
    uint8_t minute;
    uint8_t speed;    // FOR DEBUG
    uint8_t mapMin;   // FOR DEBUG
    uint8_t ledCount; // FOR DEBUG
};
#pragma pop()

Status status;

void setup()
{
    // Serial.begin(9600);
    BT.begin(38400);
    delay(8000); // Wait for bluetooth module
#ifdef BLUEOOTH_SETUP
    bt_setup();
#else
    BT.println(F("AT+E=0"));
    while (BT.available())
        BT.readString();
    delay(1000);
    BT.println(F("AT+NOTIFY=0"));
    while (BT.available())
        BT.readString();
    delay(1000);
    BT.println(F("AT+FORCEC=0"));
    while (BT.available())
        BT.readString();
    delay(2000);
#endif

    analogButtons.add(b1);
    analogButtons.add(b2);
    analogButtons.add(b3);
    analogButtons.add(b4);

    status.check = 0;
    status.led_state = 1;
    status.led_brightness = LED_INITIAL_BRIGHTNESS; // N * 25%
    status.led_temperature = 3;
    status.speed = 0;            // FOR DEBUG
    status.ledCount = NUM_LEDS;  // FOR DEBUG
    status.mapMin = LED_MAP_MIN; // FOR DEBUG

#ifdef USE_RTC
    rtc.writeProtect(false);
    rtc.halt(false);
#else
    status.hour = 12;
    status.minute = 0;
    status.second = 0;
#endif

    led_setup();
    // colorWipe(0, 0, 255, 0);
    // colorWipe(255, 0, 0, 0);
    // colorWipe(0, 255, 0, 0);
    // led_start_animation();
    // led_stop_animation();

    lastMillis = millis();
#ifdef USE_RTC
    lastUpdate = rtc.time();
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
#ifdef BUTTON_SETUP
    btn_setup();
#endif
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
    if (millis() - lastMillis < 1000)
        return;
    DateTime now = rtc.time();

    TimeDelta diff = now - lastUpdate;
    uint32_t totalseconds = diff.totalseconds();
    // if (totalseconds == 0)
    //     return;

    DateTime dt = DateTime(now.unixtime() + 6000L);
    // if (status.speed == 1)
    // {
    //     // Add 1 minute
    //     now = now + TimeDelta(totalseconds * 60);
    // }
    // else if (status.speed == 2)
    // {
    //     // Add 10 minute
    //     now = now + TimeDelta(totalseconds * 600);
    // }

    minuteCounter = (uint16_t)dt.hour() * (uint16_t)dt.minute();
    rtc.time(dt);

    if (isRunning)
        coordinate();

    lastUpdate = dt;
    lastMillis = millis();
    delay(10);
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
        if (recvInProgress == true)
        {
            buffer[ndx] = rc;
            ndx++;
            if (ndx >= BUFFER_SIZE)
            {
                recvInProgress = false;
                ndx = 0;
                newData = true;
                parse_data();
                newData = false;
            }
        }

        else if (rc == START_MARKER)
        {
            recvInProgress = true;

            // Clear buffer
            for (uint8_t i = 0; i < BUFFER_SIZE; i++)
                buffer[i] = 0;
        }
    }
}

void parse_data()
{
    byte command = buffer[0];
    if (command == CMD_POWER)
    {
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
    }
    else if (command == CMD_TIME)
    {
#ifdef USE_RTC
        DateTime now = rtc.time();
        byte hour = buffer[1];
        byte minute = buffer[2];
        if (hour > 24)
            hour = 0;
        if (minute > 60)
            minute = 0;
        now.sethour(hour);
        now.setminute(minute);
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
    }
    else if (command == CMD_TEMPERATURE)
    {
        status.led_temperature = buffer[1];
        if (status.led_temperature > 3)
            status.led_temperature = 3;
    }
    else if (command == CMD_MAPMIN)
    {
        status.mapMin = buffer[1];
        if (status.mapMin > 12)
            status.mapMin = 8;
    }
    else if (command == CMD_LED_COUNT)
    {
        status.ledCount = buffer[1];
        updateLedCount();
    }
    else if (command == CMD_STATUS)
    {
        // transmitStatus();
    }

    transmitStatus();
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
    if (status.led_state == 0)
        status.led_state = 1;
    else
        status.led_state = 0;

    if (status.led_state == 1)
        show_led();
    else
        close_led();
}

void updateLedCount()
{
    isRunning = false;
    INCREMENT = STEP_COUNT / (double)status.ledCount;
    isRunning = true;
}

void awake_bt()
{
    // digitalWrite(BT_ENABLE, HIGH);
    // delay(1000);
}

void show_led()
{
    // led_start_animation();
    isRunning = true;
}

void close_led()
{
    // led_stop_animation();
    isRunning = false;
}

void coordinate()
{
    byte cache[status.ledCount];
    const double mapped_t = mapf(minuteCounter, 0.0, STEP_COUNT, 0.0, TWO_PI);
    const byte max_brightness = (byte)(63 * status.led_brightness);

    for (byte i = 0; i < status.ledCount; i++)
    {
        double ledVal = (double)i * INCREMENT;
        double mapped_y = mapf(ledVal, 0.0, STEP_COUNT - INCREMENT, 0.0, TWO_PI);
        double cosVal = cos(mapped_t - mapped_y) + 1;
        if (cosVal < LED_THRESHOLD)
        {
            cache[i] = 0;
        }
        else
        {
            double rgb = mapf(cosVal, LED_MAP_MIN, 2.0, 1.0, max_brightness);
            const byte red = (byte)rgb;
            byte green = 255;
            byte blue = 255;
            if (status.led_temperature == 0)
            {
                green = (byte)(rgb * 0.42);
                blue = 0;
            }
            else if (status.led_temperature == 1)
            {
                green = (byte)(rgb * 0.7);
                blue = (byte)(rgb * 0.42);
            }
            else if (status.led_temperature == 2)
            {
                green = (byte)(rgb * 0.8);
                blue = (byte)(rgb * 0.6);
                /* code */
            }
            else if (status.led_temperature == 3)
            {
                green = (byte)(rgb);
                blue = (byte)(rgb);
                /* code */
            }
            cache[i] = red;
        }
    }
    cli();
    for (int i = 0; i < NUM_LEDS; i++)
    {
        sendPixel(a[i], a[i], a[i]);
    }
    sei();
    show();
}

double mapf(double val, double in_min, double in_max, double out_min, double out_max)
{
    return out_min + ((val - in_min) * (out_max - out_min) / (in_max - in_min));
}

void led_start_animation()
{
    for (byte v = 0; v < 200; v++)
    {
        cli();
        for (byte i = 0; i < status.ledCount; i++)
        {
            sendPixel(0, 0, v);
        }
        sei();
        show();
        delay(5);
    }
}

void led_stop_animation()
{
    for (byte v = 200; v >= 0; v--)
    {
        cli();
        for (byte i = 0; i < status.ledCount; i++)
        {
            sendPixel(0, 0, v);
        }
        sei();
        show();
        delay(5);
    }
}

void btn_setBrightness()
{
    status.led_brightness = (status.led_brightness % 4) + 1;

    if (isSetMode_hour || isSetMode_min)
    {
        isSetMode_hour = false;
        isSetMode_min = false;
        display.clear();
        display.setBrightness(5, false);
    }
}

void btn_setSpeed()
{
    status.speed = (status.speed + 1) % 3;
}

void btn_setTemperature()
{
    status.led_temperature = (status.led_temperature + 1) % 4;

    if (isSetMode_hour || isSetMode_min)
    {
        isSetMode_hour = false;
        isSetMode_min = false;
        display.clear();
        display.setBrightness(5, false);
    }
}

void btn_setLedCount()
{
    status.ledCount++;
    if (status.ledCount == 255)
        status.ledCount = 210;
}

void btn_setHour()
{
    if (isSetMode_hour == false)
    {
        if (isSetMode_min == true)
        {
            // Decrement minute
            DateTime now = rtc.time();
            now = now - TimeDelta(0, 0, 1, 0);
            rtc.time(now);
            display.showNumberDec((now.hour() * 1000) + now.minute());
        }
        else
        {
            isSetMode_hour = true;
            display.setBrightness(5);
            DateTime now = rtc.time();
            display.showNumberDec((now.hour() * 1000) + now.minute());
        }
    }
    else
    {
        // Decrement hour
        DateTime now = rtc.time();
        now = now - TimeDelta(0, 1, 0, 0);
        rtc.time(now);
        display.showNumberDec((now.hour() * 1000) + now.minute());
    }
}

void btn_setMinute()
{
    if (isSetMode_min == false)
    {
        if (isSetMode_hour == true)
        {
            // Increment hour
            DateTime now = rtc.time();
            now = now + TimeDelta(0, 1, 0, 0);
            rtc.time(now);
            display.showNumberDec((now.hour() * 1000) + now.minute());
        }
        else
        {
            isSetMode_min = true;
            // display.setBrightness(5);
            DateTime now = rtc.time();
            display.showNumberDec((now.hour() * 1000) + now.minute());
        }
    }
    else
    {
        // Increment minute
        DateTime now = rtc.time();
        now = now + TimeDelta(0, 0, 1, 0);
        rtc.time(now);
        display.showNumberDec((now.hour() * 1000) + now.minute());
    }
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

#ifdef BUTTON_SETUP
void btn_setup()
{
    unsigned int value = analogRead(ANALOG_PIN);
    Serial.println(value);
    delay(250);
}
#endif