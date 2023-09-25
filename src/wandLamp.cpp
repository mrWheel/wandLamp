
/*
**  ATtiny85 Draadloze WandLamp controller
** 
**  Copyright (c) 2023 Willem Aandewiel
**  TERMS OF USE: MIT License. See bottom of file.
**
**  Version 1.0  22-09-2023
** 
**  Use PROJECT TASKS -> Platform -> "Set Fuses":
**        Extended  : 0xFF
**
**              Serial program downloading (SPI) enabled
**              brown-out Detection 1.8v (0xDE)
**              brown-out detection 2.7v (0xDD)
**              brown-out detection 4.3v (0xDC)
**        High      : 0xDD 
**
**              Clock source Int.RC Osc. 8MHz PWRDWN/RESET: 6 CK/1
**        Low       : 0xE2
**
**    [Program] -> [Verify] -> [Read]
**
** -->> AVRfuses: Serial programdownloading (SPI) enabled
** -->> AVRfuses: Brown-out Detector trigger level: VCC=1.8 V
** -->> AVRfuses: Select Clock source: Int. RC Osc. 8MHz; 
**                  Start-up time PWRDWN/RESET: 6 CK/1 4CK +64 ms
** 
** Board              : "ATtiny25/45/85 (no bootloader)"
** Chip               : "ATtiny85"
** Clock Source       : "8MHz (internal)"
** Timer 1 Clock      : "CPU (CPU Frequency)"
** LTO(1.6.11+ only)  : "disabled"
** millis()/micros()  : "Enabled"                                      
** save EEPROM        : "EEPROM not retained"
** B.O.D. Level (Only set on bootload): "B.O.D. Enabled (1.8v)"
** 
** ATMEL ATTINY85
**                        +--\/--+
**             RESET PB5 1|      |8 VCC
**    -->[AnalogIn]  PB3 2|      |7 PB2  --> SW2 (Color)
**           LED <-- PB4 3|      |6 PB1  --> SW1 (On/Off)
**                   GND 4|      |5 PB0  --> NC 
**                        +------+
**
*/

#include <arduino.h>
// Inclusief de watchdog bibliotheek
#include <tinyNeoPixel.h>
#include <EEPROM.h>
#include <avr/sleep.h>    // Sleep Modes
#include <avr/power.h>

// ---[ Arduino Pin Manipulation Macros ]---
#define SET_PIN_MODE(port, pin, mode) \
                if (mode == INPUT) { \
                  DDR ## port &= ~(1 << pin); \
                } else if (mode == OUTPUT) { \
                  DDR ## port |= (1 << pin); \
                }
#define SET_PIN_HIGH(port, pin) (PORT ## port |= (1 << pin))
#define SET_PIN_LOW(port, pin)  ((PORT ## port) &= ~(1 << (pin)))
#define PIN_READ(port, pin)     (PIN ## port & (1 << pin))

#define SW1               PIN_PB1
#define SW2               PIN_PB2
#define YELLOW_LED        PIN_PB4
#define NUMLEDS            30 
#define MAXCOLORS          11
#define MAXBRIGHT         255
#define MINBRIGHT          20

tinyNeoPixel leds = tinyNeoPixel(NUMLEDS, PIN_PB0, NEO_GRBW);

struct eepromDataStruct
{
  bool        ledsOn     = false;
  uint8_t     ledsColor  =   0;
  int16_t     ledsBright = 120;
  int8_t      stepBright =   1;
};

eepromDataStruct  eepromData;
eepromDataStruct  savedData;

volatile bool     sw1Pressed, sw2Pressed;
uint32_t          startPush;


//----------------------------------------------------------------
void writePORTB(uint8_t gpioPin, uint8_t state) 
{
  if (state)
    SET_PIN_HIGH(B, gpioPin);
  else
    SET_PIN_LOW(B, gpioPin);

} //  writePORTB()


//-----------------------------------------------------------------------------
ISR(PCINT0_vect) 
{
  // Check which pin caused the interrupt
  if (PINB & (1 << SW1)) 
  {
    sw1Pressed = true;
  } 
  else if (PINB & (1 << SW2)) 
  {
    sw2Pressed = true;
  }

} //  ISR()

//-----------------------------------------------------------------------------
void goToSleep() 
{
  //-- Enable Pin Change Interrupts
  GIMSK |= _BV(PCIE);                     
  //-- Use PB1 as interrupt pin
  PCMSK |= _BV(PCINT1);                   
  //-- Use PB2 as interrupt pin
  PCMSK |= _BV(PCINT2);                   
  //-- ADC off
  ADCSRA &= ~_BV(ADEN);                   
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  //-- Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sleep_enable();                         
  //-- Enable interrupts
  sei();                                  
  //-- after next statement CPU goes to sleep ...
  sleep_cpu();                            
  //-- code continues here after an interrupt has occured

  //-- Disable interrupts
  cli();                                  
  //-- Turn off PB2 as interrupt pin
  PCMSK &= ~_BV(PCINT2);                  
  //-- Turn off PB1 as interrupt pin
  PCMSK &= ~_BV(PCINT1); 
  //-- Clear SE bit
  sleep_disable();                        
  //-- ADC on
  ADCSRA |= _BV(ADEN);                    

  //-- Enable interrupts
  sei();                                  

} // goToSleep

//----------------------------------------------------------------
void blinkYellowLed(uint8_t noBlinks, uint8_t wait) 
{
  for (int t=0; t<noBlinks; t++)
  {
    writePORTB(YELLOW_LED, !PIN_READ(B, YELLOW_LED));
    delay(wait);
  }

} //  blinkYellowLed()

//----------------------------------------------------------------
void setColor(uint8_t color) 
{
  leds.setBrightness(eepromData.ledsBright);

  for(int n=0; n<NUMLEDS; n++)
  {
    switch(color)
    {
      case  1: leds.setPixelColor(n,255,0,0,0);    break;
      case  2: leds.setPixelColor(n,255,255,0,0);  break;
      case  3: leds.setPixelColor(n,0,255,0,0);    break;
      case  4: leds.setPixelColor(n,0,255,255,0);  break;
      case  5: leds.setPixelColor(n,0,0,255,0);    break;
      case  6: leds.setPixelColor(n,255,0,255,0);  break;
      case  7: leds.setPixelColor(n,255,0,0,255);  break;
      case  8: leds.setPixelColor(n,0,255,0,255);  break;
      case  9: leds.setPixelColor(n,0,0,255,255);  break;
      case 10: leds.setPixelColor(n,255,255,255,255);  break;
      default: leds.setPixelColor(n,0,0,0,255);    break;
    } //  switch ledsColor
  }
  leds.show();

} //  setColor

//----------------------------------------------------------------
void handleSW1() 
{
  bool longPress = false;

  startPush = millis();

  //-- check for short- or long-push
  while (PIN_READ(B, SW1))
  {
    //-- less than 1000ms is a short-push
    if ((millis() - startPush) > 1000)
    {
      longPress = true;
      break;
    }
  }
  //-- if it was a short-push tockle led state
  if (!longPress || !eepromData.ledsOn)
  {
    eepromData.ledsOn = !eepromData.ledsOn;
  }

  if (longPress && eepromData.ledsOn)
  {
    //-- toggle step
    if (eepromData.stepBright == 1) { eepromData.stepBright = -1;}
    else                            { eepromData.stepBright =  1;}
    //-- re-toggle step at limits
    if (eepromData.stepBright == 1)
    {
      if (eepromData.ledsBright >= MAXBRIGHT)  { eepromData.stepBright = -1; }
    }
    else
    {
      if (eepromData.ledsBright <= MINBRIGHT)  { eepromData.stepBright = 1; }
    }
  }

  //-- only if the leds are "On" then continue
  if (eepromData.ledsOn)
  {
    SET_PIN_HIGH(B, YELLOW_LED);

    setColor(eepromData.ledsColor);
    //-- SW1 still pushed, then change brightness of the NEO's
    while(PIN_READ(B, SW1))
    {
      eepromData.ledsBright += eepromData.stepBright;
      if (eepromData.ledsBright >= MAXBRIGHT)
      {
        //-- max brightness reached
        leds.setBrightness(10);
        leds.show();
        delay(50);
        eepromData.ledsBright = MAXBRIGHT;
        leds.setBrightness(eepromData.ledsBright);
        leds.show();
        eepromData.stepBright = -1;
        //-- Wait till SW1 is released
        while (PIN_READ(B, SW1)) {delay(10); }
      }
      if (eepromData.ledsBright <= MINBRIGHT)
      {
        //-- min brightness reached
        leds.setBrightness(200);
        leds.show();
        delay(50);
        eepromData.ledsBright = MINBRIGHT;
        leds.setBrightness(eepromData.ledsBright);
        leds.show();
        eepromData.stepBright =  1;
        //-- Wait till SW1 is released
        while (PIN_READ(B, SW1)) {delay(10); }
      }
      leds.setBrightness(eepromData.ledsBright);
      leds.show();
      //-- SW1 still pushed, so wait till released
      if (PIN_READ(B, SW1)) {delay(10); }
    }
  }
  else 
  {
    //-- swich NEO's "Off"
    SET_PIN_LOW(B, YELLOW_LED);
    leds.clear();   // LED's Off
  }
  leds.show();

} //  handleSW1()

//----------------------------------------------------------------
void handleSW2() 
{
  //-- bit of debouncing
  if (PIN_READ(B, SW2)) {delay(10);}
  //-- loop through colors
  while(PIN_READ(B, SW2)) 
  {
    eepromData.ledsColor++;
    if (eepromData.ledsColor >= MAXCOLORS) eepromData.ledsColor = 0;
    setColor(eepromData.ledsColor);
    if (PIN_READ(B, SW2)) {delay(500);}
  }

} //  handleSW2()


//----------------------------------------------------------------
void setup() 
{
  //pinMode(YELLOW_LED, OUTPUT);
  SET_PIN_MODE(B, YELLOW_LED, OUTPUT);
  //digitalWrite(YELLOW_LED, LOW);
  SET_PIN_LOW(B, YELLOW_LED);
  //pinMode(SW1, INPUT);
  SET_PIN_MODE(B, SW1, INPUT);
  //pinMode(SW2, INPUT);
  SET_PIN_MODE(B, SW2, INPUT);

  blinkYellowLed(10, 250);

  eepromData.ledsOn     = false;
  eepromData.ledsColor  =  0;
  eepromData.ledsBright = 25;
  eepromData.stepBright =  1;
  EEPROM.put(0, eepromData);

  SET_PIN_LOW(B, YELLOW_LED);

  leds.begin();
  leds.clear();
  leds.setBrightness(eepromData.ledsBright);
  leds.show();

} // setup()


//--------------------------------------------------------
void loop()
{
  goToSleep();
  
  EEPROM.get(0, eepromData);

  if (sw1Pressed) 
  {
    handleSW1();
  }

  if (sw2Pressed && eepromData.ledsOn) 
  {
    handleSW2();
  }

  EEPROM.get(0, savedData);
  if (    (savedData.ledsOn     != eepromData.ledsOn)
       || (savedData.ledsColor  != eepromData.ledsColor)
       || (savedData.ledsBright != eepromData.ledsBright)
       || (savedData.stepBright != eepromData.stepBright)
      )
  {
    EEPROM.put(0, eepromData);
  }

  sw1Pressed = false;
  sw2Pressed = false;

} //  loop()

/***************************************************************************
**
** Permission is hereby granted, free of charge, to any person obtaining a
** copy of this software and associated documentation files (the
** "Software"), to deal in the Software without restriction, including
** without limitation the rights to use, copy, modify, merge, publish,
** distribute, sublicense, and/or sell copies of the Software, and to permit
** persons to whom the Software is furnished to do so, subject to the
** following conditions:
**
** The above copyright notice and this permission notice shall be included
** in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
** OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
** MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
** CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
** THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**
****************************************************************************
*/
// eof //
