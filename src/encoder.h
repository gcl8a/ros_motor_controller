/*
 * For managing quadrature encoders
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include <Arduino.h>

//motor:channel
const uint8_t ENCODER_1A = 9;
const uint8_t ENCODER_1B = 8;
const uint8_t ENCODER_2A = 7;
const uint8_t ENCODER_2B = 6;

class Encoder
{
  volatile int16_t currTicks = 0; //the current encoder value
  volatile int16_t snapTicks = 0; //frozen in an interrupt for speed calculations later
  volatile int16_t prevTicks = 0; //previous value for calculating speed

  const uint8_t pinIntA;
  const uint8_t pinIntB;  

public:
  Encoder(int pinA, int pinB) 
  : pinIntA(pinA), pinIntB(pinB)
  {}

  void ProcessInterrupt(uint8_t pinInt) volatile
  {
    int8_t del = -1;           //default for pin A
    if (pinInt == pinIntB) del = 1; //if it's pin B, reverse everything

    if (digitalRead(pinIntA) == digitalRead(pinIntB)) //hard-wire to speed up
    { 
      del *= -1; //turning the other way
    }

    currTicks += del;
  }

  int16_t TakeSnapshot(void) volatile
  {
    snapTicks = currTicks;
    return snapTicks;
  }

  int16_t CalcDelta(void) volatile
  {
    int16_t delta = snapTicks - prevTicks;
    prevTicks = snapTicks;

    return delta;
  }
};

extern volatile Encoder encoder1;
extern volatile Encoder encoder2;

void SetupEncoders(void);

#endif
