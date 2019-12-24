#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#include <AnalogButtons.h>

#include <SoftwareSerial.h>

#define ARDUINO_RX 1
#define ARDUINO_TX 0
// #define BT_ENABLE_INTERRUPT A0
SoftwareSerial BT(ARDUINO_RX, ARDUINO_TX); // ARDUINO_RX | ARDUINO_TX
#define BUFFER_SIZE 9
byte buffer[BUFFER_SIZE];
byte cursor = 0;
boolean newData = false;
#define START_MARKER 0x0F

#define TIME_PERIOD 1000

#pragma region LED
#define LED_DAT 3
#define NUM_LEDS 30

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_DAT, NEO_GRB + NEO_KHZ800);

boolean isRunning = false;
double minuteCounter = 0;
#pragma endregion

#pragma region BUTTONS
#define ANALOG_PIN A2

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
    BT.begin(9600);
    // setupBLE();
    analogButtons.add(b1);
    analogButtons.add(b2);

    status.check = 0;
    status.led_state = 1;
    status.led_brightness = 2; // N * 25%
    status.speed = 0;
    status.time.hour = 12;
    status.time.minute = 0;
    status.time.second = 0;

    strip.begin();
    strip.setBrightness(127);
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
        byte gapInSeconds = (byte)gap / 1000;
        if (status.speed == 0)
        {
            status.time.second += gapInSeconds;
        }
        else if (status.speed == 1)
        {
            status.time.second += (60 * gapInSeconds);
        }
        else if (status.speed == 2)
        {
            status.time.second += (600 * gapInSeconds);
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
        if (brightness == 0)
            brightness = 1;
        else if (brightness > 4)
            brightness = 4;
        status.led_brightness = brightness;
        strip.setBrightness((byte)(63.75 * status.led_brightness));
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
        // digitalWrite(BT_ENABLE, LOW);
    }
    else if (command == CMD_SPEED)
    {
        status.speed = buffer[1];
        if (status.speed > 2)
            status.speed = 2;
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
    BT.println();
    BT.flush();
    status.check = 0;
}

void toggle_led()
{
    if (status.led_state == 0)
    {
        status.led_state = 1;
        show_led();
    }
    else
    {
        status.led_state = 0;
        close_led();
    }
}

void awake_bt()
{
    // digitalWrite(BT_ENABLE, HIGH);
    // delay(1000);
}

void show_led()
{
    for (byte i = 0; i < NUM_LEDS; i++)
    {
        for (byte v = 0; v < 200; v++)
        {
            strip.setPixelColor(i, 0, 0, v);
            delay(10);
            strip.show();
        }
    }
    strip.clear();

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
    for (byte i = 0; i < NUM_LEDS; i++)
    {
        double sinVal = sin(0.004363325 * ((minuteCounter + (i * 48)))) * 1785;

        if (sinVal > 255)
            sinVal = 255;
        if (sinVal < 0)
            sinVal = 0;

        strip.setPixelColor(i, sinVal, sinVal, sinVal);
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
    strip.setBrightness((byte)(63.75 * status.led_brightness));
}

void btn_setSpeed()
{
    status.speed++;
    if (status.speed > 2)
        status.speed = 0;
}