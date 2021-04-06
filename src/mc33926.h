#ifndef __MC33926_H
#define __MC33926_H

#include "motor_driver.h"

#define M1A 11
#define M1B 12
#define PWM1 13

#define M2A 3
#define M2B 4
#define PWM2 5

class MC33926 : public MotorDriver
{
protected:
  uint8_t commMode = COMM_NONE; 
  
public:
  MC33926(void)
  {
    //don't set pins or registers here since this gets called before standard Arduino setup
    //use Init instead
  }

  void Init(uint8_t mode)
  {
    DEBUG_SERIAL.println("MC33926::Init");
    MotorDriver::Init();
    commMode = mode;
 
    if(commMode == COMM_PWM)
    {
      pinMode(M1A, OUTPUT);
      pinMode(M1B, OUTPUT);
      pinMode(PWM1, OUTPUT);

      pinMode(M2A, OUTPUT);
      pinMode(M2B, OUTPUT);
      pinMode(PWM2, OUTPUT);
    }

    else 
    {
      commMode = COMM_NONE;
      DEBUG_SERIAL.println("Invalid comm mode.");
    }
    DEBUG_SERIAL.println("/MC33926::Init");
  }

  void EmergencyStop(void)
  {
    MotorDriver::EmergencyStop();
  }

protected:  
  void SendPowers(int16_t powerA, int16_t powerB)
  {   
    if(commMode == COMM_PWM)
    {
      if(powerA > 0) 
      {
        digitalWrite(M1A, HIGH);
        digitalWrite(M1B, LOW);
      }
      else
      {
        digitalWrite(M1A, LOW);
        digitalWrite(M1B, HIGH);
      }
      
      powerA = abs(powerA);
      analogWrite(PWM1, powerA);

      if(powerB > 0) 
      {
        digitalWrite(M2A, HIGH);
        digitalWrite(M2B, LOW);
      }
      else
      {
        digitalWrite(M2A, LOW);
        digitalWrite(M2B, HIGH);
      }
      
      powerB = abs(powerB);
      analogWrite(PWM2, powerB);
    }
  }
};

#endif

 
