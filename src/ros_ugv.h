#ifndef __ROS_UGV_H
#define __ROS_UGV_H

#include "robot.h"

#include <ros.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>

void CmdModeCallback(const std_msgs::UInt16&);
void CmdMotorTargetCallback(const std_msgs::UInt32&);

class ROSUGV : public UGV
{
protected:
  //node handler
  ros::NodeHandle nh;

  ros::Subscriber<std_msgs::UInt32> subMotorTargets;
  ros::Publisher pubMotorSpeeds;
  std_msgs::UInt32 motorDatum; //use a uint32 to pass motor speeds

  ros::Subscriber<std_msgs::UInt16> subCmdMode;
  ros::Publisher pubCmdSource;
  std_msgs::UInt16 cmdSource; //use a uint16 to keep everything lined up

public:
  ROSUGV(void) :  subMotorTargets("motor_targets", CmdMotorTargetCallback), //motor target is in encoder ticks per second
                  pubMotorSpeeds("motor_speeds", &motorDatum), //motor speed is in encoder ticks per second
                  subCmdMode("cmd_mode", CmdModeCallback),
                  pubCmdSource("cmd_source", &cmdSource)
  {}
  
  void Init(void)
  {
    UGV::Init();

    nh.initNode();

    nh.subscribe(subMotorTargets);
    nh.advertise(pubMotorSpeeds);

    nh.subscribe(subCmdMode);
    nh.advertise(pubCmdSource);
  }

  void MainLoop(void)
  {
    if(readyToPID) 
    {
      ProcessPID();
      readyToPID = 0;
    }

    nh.spinOnce();

    //really, we should call UGV to keep things separate, but taken over for now
    //UGV::MainLoop();
  }

  void ProcessPID(void)
  {
    UGV::ProcessPID();

    //publish motor speeds
    ivector motorSpeedsPerSecond = motorSpeeds * LOOP_RATE;
    memcpy(&motorDatum.data, &motorSpeedsPerSecond[0], 4);
    pubMotorSpeeds.publish(&motorDatum);

    cmdSource.data = UGV::cmdSource;
    pubCmdSource.publish(&cmdSource);

    return;
  }

  void HandleMotorTargetCommand(const std_msgs::UInt32& motor_targets)
  {
    if(UGV::cmdSource == CMD_SRC_ROS)
    {
      int16_t targets[2];
      memcpy(targets, &motor_targets.data, 4);
    
      SetTargetMotorSpeeds(targets[0], targets[1]);
    }
  }
};

extern ROSUGV robot;

#endif
