
/*
 * Reads sensors (encoders) to calculate current velocity estimate (we'll leave pose for higher level functions).
 * Calculates actuator (motor) inputs using PI.
 */

#ifndef __ESTIMATOR_H
#define __ESTIMATOR_H

#include <vector_uC.h>

#define ivector TVector<int16_t>

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

  ivector SetTarget(const ivector& t) //in ticks / second
  {
    target = t / LOOP_RATE;

//    DEBUG_SERIAL.print(target[0]);
//    DEBUG_SERIAL.print('\t');
//    DEBUG_SERIAL.print(target[1]);
//    DEBUG_SERIAL.print('\n');

    return target;
  }
};

class PositionController
{ 
protected: 
  ivector target;   //target position, using integer math to speed up the processing
  ivector estimate; //wheel position estimate

  uint16_t Kp = KP_DEF;
  uint16_t Ki = KI_DEF;

public:
  PositionController(void) : target(2), estimate(2) {}

  void Init(void);

  ivector CalcMotorPositions(void)
  {
    estimate[0] = encoder1.CalcPosition();
    estimate[1] = encoder2.CalcPosition();

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

  ivector SetTarget(const ivector& t) //in ticks / second
  {
    target = t;

//    DEBUG_SERIAL.print(target[0]);
//    DEBUG_SERIAL.print('\t');
//    DEBUG_SERIAL.print(target[1]);
//    DEBUG_SERIAL.print('\n');

    return target;
  }
};

#endif
