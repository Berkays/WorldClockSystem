#define TASKER_MAX_TASKS 2
#include <Button.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_WS2801.h>
#include <SPI.h>
#include <Tasker.h>
Button brightBtn = Button(A5);
Button speedBtn = Button(A4);
// #include <DS1302.h>

#define BT_RX 7
#define BT_TX 8
#define BT_ENABLE 9
// #define BT_ENABLE_INTERRUPT A0
SoftwareSerial BT(BT_RX, BT_TX); // RX | TX
#define BUFFER_SIZE 9
byte buffer[BUFFER_SIZE];
byte cursor = 0;
boolean newData = false;
#define START_MARKER 0x0F

#define TIME_PERIOD 1000
// #pragma region RTC
// #define DS_CLK 2 // Clock
// #define DS_DAT 3 // Data
// #define DS_RST 4 // Reset
// DS1302 RTC(DS_RST, DS_DAT, DS_CLK);
// #pragma endregion

#pragma region LED
#define LED_DAT A2  // Data
#define LED_CLK A3  // Clock
#define NUM_LEDS 30 // # of Leds in strip
#define Increment 48
#define STEP 3
#define Kh 28
#define Km 2

boolean isRunning = true;
double minuteCounter = 0;
// int current_led = 0;
// double testmin = 0;
// byte a = 0;
Adafruit_WS2801 strip = Adafruit_WS2801(NUM_LEDS, LED_DAT, LED_CLK);
Tasker tasker;
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
#define CMD_SPEED 0x07

#pragma pack(1)
typedef struct TimeStatus
{
    byte hour;
    byte minute;
    double second;
};
#pragma pop()
typedef struct Status
{
    byte check;
    byte led_state;
    byte led_brightness;
    byte speed;
    TimeStatus time;
};

Status status;
unsigned long lastMillis = 0;

void setup()
{
    // pinMode(BT_ENABLE, OUTPUT);
    speedBtn.begin();
    brightBtn.begin();

    // pinMode(BT_ENABLE_INTERRUPT, INPUT_PULLUP);

    // digitalWrite(BT_ENABLE, HIGH);
    // attachInterrupt(BT_ENABLE_INTERRUPT, awake_bt, LOW);

    // BT.begin(9600);
    // setupBLE();

    status.check = 0;
    status.led_state = 1;
    status.led_brightness = 4; // N * 25%
    status.speed = 0;
    status.time.hour = 12;
    status.time.minute = 0;
    status.time.second = 0;

    strip.begin();
    strip.show();

    test_strip();
    lastMillis = millis();
    tasker.setInterval(checkBtn, 10);
}

void loop()
{
    updateSecond();
    tasker.loop();
    // recv();
}

void checkBtn()
{
    if (speedBtn.pressed())
    {
        btn_setSpeed();
    }
    if (brightBtn.pressed())
    {
        btn_setBrightness();
    }
}

void updateSecond()
{
    unsigned long currentMillis = millis();
    long gap = currentMillis - lastMillis;
    if (gap > TIME_PERIOD)
    {
        lastMillis = currentMillis;

        if (status.speed == 0)
        {
            status.time.second++;
        }
        else if (status.speed == 1)
        {
            status.time.second += 60;
        }
        else if (status.speed == 2)
        {
            status.time.second += 600;
        }

        uint16_t extra_min = (uint16_t)status.time.second / 60;
        status.time.second = (uint16_t)status.time.second % 60;

        status.time.minute += extra_min;
        minuteCounter += extra_min;

        if (minuteCounter >= 1440)
            minuteCounter -= 1440;

        if (status.time.minute >= 60)
        {
            status.time.hour++;
            status.time.minute = status.time.minute % 60;

            if (status.time.hour == 24)
                status.time.hour = 0;
        }

        if (isRunning)
            coordinate();
    }
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
        // set_brightness(brightness);
        transmitStatus();
    }
    else if (command == CMD_TIME)
    {
        byte *p;
        p = (byte *)&status.time;
        for (byte i = 0; i < sizeof(TimeStatus); i++)
            p[i] = buffer[i + 1];

        transmitStatus();
    }
    else if (command == CMD_BT_PWR)
    {
        // Irreversible
        digitalWrite(BT_ENABLE, LOW);
    }
    else if (command == CMD_SPEED)
    {
        status.speed = buffer[1];
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
    BT.println();
    BT.flush();
    status.check = 0;
}

void toggle_led()
{
    status.led_state = (status.led_state + 1) % 2;
    if (status.led_state == 0)
        close_led();
    else
        show_led();
}

void awake_bt()
{
    digitalWrite(BT_ENABLE, HIGH);
    delay(1000);
}

void show_led()
{
    for (byte i = 0; i < NUM_LEDS; i++)
    {
        for (byte v = 0; v < 200; v++)
        {
            strip.setPixelColor(i, v);
            delay(20);
            strip.show();
        }
    }
    isRunning = true;
}

void close_led()
{
    isRunning = false;

    for (byte v = 100; v > 0; v--)
    {

        for (byte i = 0; i < NUM_LEDS; i++)
            strip.setPixelColor(i, v - 1);
        delay(25);
        strip.show();
    }
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

void coordinate()
{
    double c = (double)status.led_brightness / 4.0;
    for (byte i = 0; i < NUM_LEDS; i++)
    {
        double sinVal = sin(0.004363325 * ((minuteCounter + (i * 48)))) * 1785;

        if (sinVal > 255)
            sinVal = 255;
        if (sinVal < 0)
            sinVal = 0;

        int a = (int)(sinVal * c);
        strip.setPixelColor(i, a, a, a);
    }
    strip.show();
}

void test_strip()
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        strip.setPixelColor(i, 255, 255, 255);
        strip.show();
        delay(50);
        strip.setPixelColor(i, 0, 0, 0);
        strip.show();
        delay(50);
    }
}

void btn_setBrightness()
{
    status.led_brightness = (status.led_brightness % 4) + 1;
    coordinate();
}

void btn_setSpeed()
{
    status.speed++;
    if (status.speed > 2)
        status.speed = 0;
}