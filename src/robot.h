/* 
 * File:   robot.h
 * Author: greg
 *
 * Created on 3. november 2012, 22:53
 */

#ifndef __ROBOT_H
#define __ROBOT_H

#include "comm.h"
#include "sabertooth.h"
//#include "mc33926.h"
#include "controller.h"

 /*
 * Calculation for RADIANS_PER_TICK:
 * 7 magnets = 28 ticks / rotation
 * Gear ratio of 71:1 -> 1988 ticks / rotation 
 * or 2pi / 1988 = 0.00316 rad / tick
 * 
 */

#define RADIANS_PER_TICK (0.00316)
//#define RADIUS_WHEEL (0.081)
//const uint16_t TICKS_PER_METER = 3906;
//const float ROBOT_RADIUS = 0.227;
//const float ROBOT_RADIUS_IN_TICKS = 887; //(ROBOT_RADIUS * TICKS_PER_METER)

class UGV
{  
protected:
  CMD_SRC cmdSource = CMD_SRC_ROS; //start out using ROS
  
  ivector motorSpeeds; //ticks per interval
  ivector effort;
  
  //motor driver;
  Sabertooth driver;
  MotionController controller;

public:
  UGV(void) : motorSpeeds(2), effort(2) 
  {}
  
  virtual void Init(void)
  {
    DEBUG_SERIAL.println("UGV::Init");

    //opMode = IDLE;
    driver.Init(COMM_PACKET_SERIAL);
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

    // if(imu.IsAvailableAccelAndGyro())
    // {
    //   imu.ProcessReadings();
    //   String dataStr = imu.CalcRPY().MakeDataString();// + '\n';
    //   DEBUG_SERIAL.println(dataStr);
    // }
  }

  virtual void ProcessPID(void)
  {
//    DEBUG_SERIAL.print("readyToPID");
//    DEBUG_SERIAL.print('\n');
      //////////!!!!!!!!!
      DEBUG_SERIAL.print(millis());
      DEBUG_SERIAL.print('\t');

      motorSpeeds = controller.CalcMotorSpeeds(); //wheel velocity is ticks / period

      DEBUG_SERIAL.print(motorSpeeds[0]);
      DEBUG_SERIAL.print('\t');
      DEBUG_SERIAL.print(motorSpeeds[1]);
      DEBUG_SERIAL.print('\t');

//      if(cmdMode == CMD_VEL)
      {
        effort = controller.CalcEffort();
        CommandMotors(effort);
  
        DEBUG_SERIAL.print(effort[0]);
        DEBUG_SERIAL.print('\t');
        DEBUG_SERIAL.print(effort[1]);
        DEBUG_SERIAL.print('\n');
      }


  }
  
  void SetTargetMotorSpeeds(float left, float right) //in mrad/sec
  {
    //integer vector -- speeds are in integral numbers of ticks -- ignore the digitization error for now...
    TVector<float> speed(2); 
    speed[0] = left / (1000. * RADIANS_PER_TICK * LOOP_RATE);
    speed[1] = right / (1000. * RADIANS_PER_TICK * LOOP_RATE);
        
    controller.SetTarget(speed);
  }

  ivector CommandMotors(const ivector& effort) //actuators, generically
  {
    driver.SetPowers(effort[0], effort[1]);
    
    return effort;
  }

  CMD_SRC SetSource(uint16_t newSource) {return cmdSource = (CMD_SRC)newSource;}
};

#endif	/* ROBOT_H */
