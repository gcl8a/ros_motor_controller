#include "ros_ugv.h"

void CmdModeCallback(const std_msgs::UInt16& cmd_mode)
{
  robot.SetSource(cmd_mode.data);
}

void CmdMotorTargetCallback(const std_msgs::UInt32& motor_targets)
{
  robot.HandleMotorTargetCommand(motor_targets);
}
