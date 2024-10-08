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
  ROSUGV(void) :  
      subMotorTargets("motor_targets", CmdMotorTargetCallback), //motor target is in encoder ticks per second
      pubMotorSpeeds("motor_speeds", &motorDatum), //motor speed is in mrad per second
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
    UGV::MainLoop();

    nh.spinOnce();
  }

  void ProcessPID(void)
  {
    UGV::ProcessPID();

    //publish motor speeds
    ivector mRadPerSec = motorSpeeds * (LOOP_RATE * RADIANS_PER_TICK * 1000);
    uint32_t datum;
    memcpy(&datum, &mRadPerSec[0], 4);
    motorDatum.data = datum;

    pubMotorSpeeds.publish(&motorDatum);

    cmdSource.data = UGV::cmdSource;
    pubCmdSource.publish(&cmdSource);

    return;
  }

  void HandleMotorTargetCommand(const std_msgs::UInt32& motor_targets)
  {
    //motor_targets in mrad/sec for consistency
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
