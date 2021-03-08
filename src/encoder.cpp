#include "encoder.h"

volatile Encoder encoder1(ENCODER_1A, ENCODER_1B);
volatile Encoder encoder2(ENCODER_2A, ENCODER_2B);


void EncoderHandler1A(void) {encoder1.ProcessInterrupt(ENCODER_1A);}
void EncoderHandler1B(void) {encoder1.ProcessInterrupt(ENCODER_1B);}
void EncoderHandler2A(void) {encoder2.ProcessInterrupt(ENCODER_2A);}
void EncoderHandler2B(void) {encoder2.ProcessInterrupt(ENCODER_2B);}

void SetupEncoders(void)
{
  //(almost) every pin is an interrupt on the SAMD, so use attachInterrupt()
  attachInterrupt(ENCODER_1A, EncoderHandler1A, CHANGE);
  attachInterrupt(ENCODER_1B, EncoderHandler1B, CHANGE);
  attachInterrupt(ENCODER_2A, EncoderHandler2A, CHANGE);
  attachInterrupt(ENCODER_2B, EncoderHandler2B, CHANGE);
}
