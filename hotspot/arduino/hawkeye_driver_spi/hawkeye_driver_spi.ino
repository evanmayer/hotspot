//**************************************************************//
//  Name    : shiftOutCode, Hello World
//  Author  : Carlyn Maw,Tom Igoe, David A. Mellis
//  Date    : 25 Oct, 2006
//  Modified: 23 Mar 2010
//  Version : 2.0
//  Notes   : Code for using a 74HC595 Shift Register           //
//          : to count from 0 to 255
//****************************************************************
#include <SPI.h>

//Pin connected to ST_CP of 74HC595
int latchPin = 0;

void setup() {
  //set pins to output so you can control the shift register
  pinMode(latchPin, OUTPUT);

  SPI.begin();
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
}

void loop() {
  digitalWrite(latchPin, LOW);
  SPI.transfer(0);
  digitalWrite(latchPin, HIGH);
  delay(250);
  
  digitalWrite(latchPin, LOW);
  SPI.transfer(7);
  digitalWrite(latchPin, HIGH);
  delay(250);
}
