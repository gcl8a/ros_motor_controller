/* 
 * File:   robot.h
 * Author: greg
 *
 * Created on 3. november 2012, 22:53
 */

#ifndef __ROBOT_H
#define __ROBOT_H

#include "comm.h"
#include "mc33926.h"
#include "controller.h"

 /*
 * Calculation for RADIANS_PER_TICK:
 * 
 * NEEDS TO BE UPDATED FOR 37MM MOTOR!!!!!!!!!!!!!
 * 
 * 7 magnets = 28 ticks / rotation
 * Gear ratio of 71:1 -> 1988 ticks / rotation 
 * or 2pi / 1988 = 0.00316 rad / tick
 * 
 */

//#define RADIANS_PER_TICK (0.00316)
//#define RADIUS_WHEEL (0.081)
//const uint16_t TICKS_PER_METER = 3906;
//const float ROBOT_RADIUS = 0.227;
//const float ROBOT_RADIUS_IN_TICKS = 887; //(ROBOT_RADIUS * TICKS_PER_METER)

class UGV
{  
protected:
  CMD_SRC cmdSource = CMD_SRC_ROS; //start out using ROS
  
  ivector motorPositions;
  //ivector motorSpeeds;
  ivector effort;
  
  //motor driver;
  MC33926 driver;
  PositionController controller;

public:
  UGV(void) : motorPositions(2), effort(2) 
  {}
  
  virtual void Init(void)
  {
    DEBUG_SERIAL.println("UGV::Init");

    driver.Init(COMM_PWM);
    controller.Init();
    
    DEBUG_SERIAL.println("/UGV::Init");
  }

  void Idle(void)
  {
    driver.FullStop();
  }

  virtual void MainLoop(void)
  {  
    if(readyToPID) 
    {
      ProcessPID();
      readyToPID = 0;
    }
  }

  virtual void ProcessPID(void)
  {
//    DEBUG_SERIAL.print("readyToPID");
//    DEBUG_SERIAL.print('\n');
      
      //motorSpeeds = controller.CalcMotorSpeeds(); //wheel velocity is ticks / period
      motorPositions = controller.CalcMotorPositions(); //hideous, but it works

      DEBUG_SERIAL.print(millis());
      DEBUG_SERIAL.print('\t');

      DEBUG_SERIAL.print(motorPositions[0]);
      DEBUG_SERIAL.print('\t');
      DEBUG_SERIAL.print(motorPositions[1]);
      DEBUG_SERIAL.print('\t');

      effort = controller.CalcEffort();
      CommandMotors(effort);

      DEBUG_SERIAL.print(effort[0]);
      DEBUG_SERIAL.print('\t');
      DEBUG_SERIAL.print(effort[1]);
      DEBUG_SERIAL.print('\n');        
  }
  
  void SetTargetPositions(int16_t left, int16_t right) //could
  {
    //integer vector -- positons are in integral numbers of ticks
    ivector pos(2); 
    pos[0] = left;
    pos[1] = right;
        
    controller.SetTarget(pos);
  }

  ivector CommandMotors(const ivector& effort) //actuators, generically
  {
    driver.SetPowers(effort[0], effort[1]);
    
    return effort;
  }

  CMD_SRC SetSource(uint16_t newSource) {return cmdSource = (CMD_SRC)newSource;}
};

#endif	/* ROBOT_H */
