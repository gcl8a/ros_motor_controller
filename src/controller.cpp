#include "controller.h"

void TC3_Handler()  // Interrupt on overflow
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
  
  if (TC->INTFLAG.bit.OVF == 1)   // An overflow caused the interrupt
  {
    TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag

    //take a snapshot of the encoders for precise measurement
    encoder1.TakeSnapshot();
    encoder2.TakeSnapshot();

    //set PID flag to 1 to calculate control values at our leisure
    readyToPID = 1;
  }
}
