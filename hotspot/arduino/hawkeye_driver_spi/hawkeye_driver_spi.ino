#include <SPI.h>

//latchPin is connected to ST_CP of 74HC595
int latchPin = 2;
//activity indicator
int ledPin = 13;
SPISettings settings;

void setup() {
  //set pins to output so you can control the shift register
  pinMode(latchPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // hardware SPI
  SPI.begin();
  settings = SPISettings(1500000, MSBFIRST, SPI_MODE0);
  SPI.beginTransaction(settings);
}

void loop() {
  digitalWrite(ledPin, LOW);
  SPI.beginTransaction(settings);
  digitalWrite(latchPin, LOW);
  SPI.transfer(0);
  digitalWrite(latchPin, HIGH);
  SPI.endTransaction();
  delay(150);
}
