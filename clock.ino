#include "Arduino.h"
#include <Adafruit_NeoPixel.h>
// #ifdef __AVR__
// #include <avr/power.h>
// #endif
#include <AnalogButtons.h>

// #define ARDUINO_RX 7
// #define ARDUINO_TX 8
#define BT_ENABLE 9
// #define BT_ENABLE_INTERRUPT A0
// SoftwareSerial BT(ARDUINO_RX, ARDUINO_TX);
#define BUFFER_SIZE 5
byte buffer[BUFFER_SIZE];
byte cursor = 0;
boolean newData = false;
#define START_MARKER 0x0F

#define TIME_PERIOD 1000

#pragma region LED
#define LED_DAT 7
#define NUM_LEDS 96
#define TWO_PI 6.2831853072
double INCREMENT = 1440.0 / (double)NUM_LEDS;
#define LED_THRESHOLD 1
boolean isRunning = false;
double minuteCounter = 0;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_DAT, NEO_GRB + NEO_KHZ800);
#pragma endregion

#pragma region BUTTONS
#define ANALOG_PIN A3

AnalogButtons analogButtons(ANALOG_PIN, INPUT);
Button b1 = Button(360, &btn_setSpeed);
Button b2 = Button(40, &btn_setBrightness);
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
unsigned long lastMillis = 0;

void setup()
{
    Serial.begin(9600);
    // pinMode(BT_ENABLE, OUTPUT);
    analogButtons.add(b1);
    analogButtons.add(b2);

    // pinMode(BT_ENABLE_INTERRUPT, INPUT_PULLUP);
    pinMode(8, OUTPUT);
    digitalWrite(8, HIGH);
    // digitalWrite(BT_ENABLE, HIGH);
    // attachInterrupt(BT_ENABLE_INTERRUPT, awake_bt, LOW);

    status.check = 0;
    status.led_state = 1;
    status.led_brightness = 2; // N * 25%
    status.speed = 0;
    status.threshold = 10;      // FOR DEBUG
    status.ledCount = NUM_LEDS; // FOR DEBUG
    status.mapMin = 7;          // FOR DEBUG
    status.hour = 12;
    status.minute = 0;
    status.second = 0;

    strip.begin();
    strip.setBrightness(255);
    strip.clear();
    strip.show();

    test_strip();
    lastMillis = millis();
    isRunning = true;
}

void loop()
{
    updateSecond();
    analogButtons.check();
    recv();
}

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

void setupBLE()
{
    Serial.println("AT+ROLE0");
    delay(100);
    // Flush buffer
    while (Serial.available())
    {
    }
    delay(500);
    // sendCommand("AT+NAMEClock");
    Serial.print("AT+NAME");
    Serial.println("WClock A9F54");
    delay(2000);
}

void recv()
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    byte rc;
    while (Serial.available() > 0 && newData == false)
    {
        rc = Serial.read();
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
        // strip.setBrightness((byte)(63.75 * status.led_brightness));
    }
    else if (command == CMD_TIME)
    {
        status.hour = buffer[1];
        status.minute = buffer[2];
        status.second = 0;
        minuteCounter = status.minute * status.hour;
        if (minuteCounter >= 1440)
            minuteCounter = 0;
        else if (minuteCounter < 0)
            minuteCounter = 0;

        transmitStatus();
    }
    else if (command == CMD_BT_PWR)
    {
        // Irreversible
        // digitalWrite(BT_ENABLE, LOW);
    }
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
        if (status.threshold >= 20)
            status.threshold = 10;
        transmitStatus();
    }
    else if (command == CMD_MAPMIN)
    {
        status.mapMin = buffer[1];
        if (status.mapMin > 15)
            status.mapMin = 7;
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
    Serial.write((byte *)&status, structSize);
    Serial.flush();
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
    for (byte v = 0; v < 200; v++)
    {
        for (byte i = 0; i < status.ledCount; i++)
        {
            strip.setPixelColor(i, 0, 0, v);
        }
        strip.show();
        delay(5);
    }

    strip.clear();
    isRunning = true;
}

void close_led()
{
    isRunning = false;

    for (byte v = 200; v > 0; v--)
    {
        for (byte i = 0; i < status.ledCount; i++)
        {
            strip.setPixelColor(i, 0, 0, v - 1);
        }
        strip.show();
        delay(5);
    }
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

void test_strip()
{
    for (int i = 0; i < status.ledCount; i++)
    {
        strip.setPixelColor(i, 200);
        strip.show();
        delay(50);
    }
}

void btn_setBrightness()
{
    status.led_brightness = (status.led_brightness % 4) + 1;
    // strip.setBrightness((byte)(63.75 * status.led_brightness));
}

void btn_setSpeed()
{
    status.speed++;
    if (status.speed > 2)
        status.speed = 0;
}