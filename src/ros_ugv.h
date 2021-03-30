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
  ros::Publisher pubMotorPositions;
  std_msgs::UInt32 motorDatum; //use a uint32 to pass motor speeds

  ros::Subscriber<std_msgs::UInt16> subCmdMode;
  ros::Publisher pubCmdSource;
  std_msgs::UInt16 cmdSource; //use a uint16 to keep everything lined up

  ros::Publisher pubAtTarget;

public:
  ROSUGV(void) :  subMotorTargets("motor_targets", CmdMotorTargetCallback), //motor target is in encoder ticks per second
                  pubMotorPositions("motor_positions", &motorDatum), //in encoder ticks
                  subCmdMode("cmd_mode", CmdModeCallback),
                  pubCmdSource("cmd_source", &cmdSource),
                  pubAtTarget("at_target", &pubAtTarget)
  {}

  void Init(void)
  {
    UGV::Init();

    nh.initNode();

    nh.subscribe(subMotorTargets);
    nh.advertise(pubMotorPositions);

    nh.subscribe(subCmdMode);
    nh.advertise(pubCmdSource);

    nh.advertise(pubAtTarget);
  }

  void MainLoop(void)
  {
    if(readyToPID) 
    {
      DEBUG_SERIAL.println("PID");
      ProcessPID();
      readyToPID = 0;
    }

    nh.spinOnce();
    //UGV::MainLoop();
  }

  void ProcessPID(void)
  {
    UGV::ProcessPID();

    //publish motor positions
    //ivector motorPositions = motorPositions;
    memcpy(&motorDatum.data, &motorPositions[0], 4);
    pubMotorPositions.publish(&motorDatum);

    cmdSource.data = UGV::cmdSource;
    pubCmdSource.publish(&cmdSource);

    return;
  }

  void HandleMotorTargetCommand(const std_msgs::UInt32& motor_targets)
  {
    if(UGV::cmdSource == CMD_SRC_ROS)
    {
      ivector targets(2);
      memcpy(&targets[0], &motor_targets.data, 4);
    
      SetTargetPositions(targets[0], targets[1]);
    }
  }
};

extern ROSUGV robot;

#endif
