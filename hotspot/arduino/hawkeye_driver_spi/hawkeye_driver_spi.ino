// This program writes a byte from serial input to an SPI output.
// The SPI peripheral is a shift register, 74HC595.
#include <Adafruit_DotStar.h>
#include <SPI.h>


// payload from serial
char data;
// ST_CP of 74HC595
int latchPin = 2;
SPISettings settings;
// activity indicators
int ledPin = 13;
Adafruit_DotStar onBoard = Adafruit_DotStar(1, INTERNAL_DS_DATA, INTERNAL_DS_CLK, DOTSTAR_BGR);


void setup()
{
    Serial.begin(9600);

    SPI.begin();
    settings = SPISettings(1500000, MSBFIRST, SPI_MODE0);
    SPI.beginTransaction(settings);

    pinMode(latchPin, OUTPUT);
    pinMode(ledPin, OUTPUT);
    onBoard.begin();
    onBoard.show();
}


void writeByteSPI(long data)
{
    SPI.beginTransaction(settings);
    digitalWrite(latchPin, LOW);
    SPI.transfer(data);
    digitalWrite(latchPin, HIGH);
    SPI.endTransaction();
}


void loop()
{
    while (Serial.available() > 0)
    {
        // No timeout; intentionally hangs until next byte comes in.
        data = (char)Serial.parseInt();
        writeByteSPI(data);
        if (data > 0L)
        {
            digitalWrite(ledPin, HIGH);
            // interpret the 3 bits of the 8-bit char as channels in RGB.
            onBoard.setPixelColor(0, \
                (data & 0b001) * 255,\
                (data & 0b010) * 255,\
                (data & 0b100) * 255\
            );
            onBoard.setBrightness(100);
            onBoard.show();
        }
        else
        {
          digitalWrite(ledPin, LOW);
          onBoard.setBrightness(50);
          onBoard.show();
        }
    }
}
