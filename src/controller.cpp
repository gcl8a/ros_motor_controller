#include "controller.h"

volatile uint8_t readyToPID = 0;

void MotionController::Init(void)
{
#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.println("MotionController::Init");
#endif 

  SetupEncoders();

  /*
    * Set up TC3 for periodically calling the PID routine 
    */
  // Feed GCLK0 (already enabled) to TCC2 and TC3
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable 
                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK0
                     GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed clock to TCC2 and TC3
  while (GCLK->STATUS.bit.SYNCBUSY) {};           // Wait for synchronization

  // The type cast must fit with the selected timer mode (defaults to 16-bit)
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
  
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;    // Disable TC
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 

  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Mode to 16 bits (defaults to 16-bit, but why not?)
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync -- UNNEEDED, according to datasheet

  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC mode
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync -- UNNEEDED, according to datasheet
  
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;   // Set prescaler
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync -- UNNEEDED, according to datasheet

  //by setting TOP, we'll change the frequency to:
  //freq = 48e6 / [(TOP + 1) * prescaler]
  //freq * prescaler / 48e6 - 1 => TOP
    //set compare value: freq = 48e6 / [3750 * 256] = ~50Hz ==> period of 20 ms
  TC->CC[0].reg = 48000000ul / (LOOP_RATE * 256) - 1; //3749; 
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  
  // Interrupts
  TC->INTENCLR.reg = 0x3B;           // clear all interrupts on this TC
  TC->INTENSET.bit.OVF = 1;          // enable overflow interrupt
  
  // Enable InterruptVector
  NVIC_EnableIRQ(TC3_IRQn);  //not sure what the n is for
  
  // Enable TC
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;   //enable
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  
#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.println("/MotionController::Init");
#endif
}

void TC3_Handler()  // Interrupt on overflow
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
  
  if (TC->INTFLAG.bit.OVF == 1)   // An overflow caused the interrupt
  {
    TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag

    encoder1.TakeSnapshot();
    encoder2.TakeSnapshot();

    readyToPID = 1;
  }
}
