
/*
 * Reads sensors (encoders) to calculate current velocity estimate (we'll leave pose for higher level functions).
 * Calculates actuator (motor) inputs using PI.
 */

#ifndef __ESTIMATOR_H
#define __ESTIMATOR_H

#include <vector_uC.h>

//#define ivector TVector<int16_t>

#include "encoder.h"
#include "comm.h"

extern volatile uint8_t readyToPID;

#define INTEGRAL_CAP 24000 //note that the comparison is sum > (INTEGRAL_CAP / Ki) so that changing Ki doesn't affect the cap
#define KP_DEF 64
#define KI_DEF 16

#define LOOP_RATE 50 //Hz

class MotionController
{ 
protected: 
  ivector target;   //target speed, using integer math to speed up the processing
  ivector estimate; //wheel speed estimate

  uint16_t Kp = KP_DEF;
  uint16_t Ki = KI_DEF;

public:
  MotionController(void) : target(2), estimate(2) {}

  void Init(void);
  // {
  //   DEBUG_SERIAL.println("MotionController::Init");
  //   SetupEncoders();

  //   /*
  //    * Set up TC3 for periodically calling the PID routine 
  //    */
  //   // Feed GCLK0 (already enabled) to TCC2 and TC3
  //   REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable 
  //                      GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK0
  //                      GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed clock to TCC2 and TC3
  //   while (GCLK->STATUS.bit.SYNCBUSY) {};           // Wait for synchronization
  
  //   // The type cast must fit with the selected timer mode (defaults to 16-bit)
  //   TcCount16* TC = (TcCount16*) TC3; // get timer struct
    
  //   TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;    // Disable TC
  //   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
  
  //   TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits (defaults to 16-bit, but why not?)
  //   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync -- UNNEEDED, according to datasheet
  
  //   TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC mode
  //   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync -- UNNEEDED, according to datasheet
    
  //   TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;   // Set prescaler
  //   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync -- UNNEEDED, according to datasheet
  
  //   //by setting TOP, we'll change the frequency to:
  //   //freq = 48e6 / [(TOP + 1) * prescaler]
  //   //so TOP = 48e6 / [freq * prescaler] - 1
  //   uint32_t TOP = 48000000ul / (LOOP_RATE * 256) - 1;

  //   TC->CC[0].reg = TOP; 
  //   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
    
  //   // Interrupts
  //   TC->INTENCLR.reg = 0x3B;           // clear all interrupts on this TC
  //   TC->INTENSET.bit.OVF = 1;          // enable overflow interrupt
   
  //   // Enable InterruptVector
  //   NVIC_EnableIRQ(TC3_IRQn);  //not sure what the n is for
    
  //   // Enable TC
  //   TC->CTRLA.reg |= TC_CTRLA_ENABLE;   //enable
  //   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
    
  //   DEBUG_SERIAL.println("/MotionController::Init");
  // }

  ivector CalcMotorSpeeds(void)
  {
    estimate[0] = encoder1.CalcDelta();
    estimate[1] = encoder2.CalcDelta();

    return estimate;
  }

  ivector CalcError(void)
  {
    return target - estimate;
  }

/*
 * CalcEffort uses PI control to try to reach target speed. Note that we're using integer maths, which
 * means that we do everything with ints scaled by 128. That is, Kp and Ki are 128 times larger than they
 * would normally be and then we divide by 128 at the end to get the effort, which is sent to the sabertooth.
 */
  ivector CalcEffort(void) 
  {
    static ivector sumError(2);

    ivector error = CalcError();
    sumError += error;
    
    if(abs(sumError[0]) > (INTEGRAL_CAP / Ki)) sumError[0] -= error[0]; //cap the sum of the errors 
    if(abs(sumError[1]) > (INTEGRAL_CAP / Ki)) sumError[1] -= error[1]; //cap the sum of the errors 

    ivector effort = (error * Kp + sumError * Ki) / 128; //Kp and Ki in 128's to make integer math work out

    return effort;
  }

  ivector SetTarget(const ivector& t) //in ticks / interval
  {
    target = t;

#ifdef __USE_DEBUG__
    DEBUG_SERIAL.print(target[0]);
    DEBUG_SERIAL.print('\t');
    DEBUG_SERIAL.print(target[1]);
    DEBUG_SERIAL.print('\n');
#endif

    return target;
  }
};

#endif
