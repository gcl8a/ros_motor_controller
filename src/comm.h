#ifndef __COMM_H
#define __COMM_H

#include <Arduino.h>
//#define USE_USBCON //comment out to use Serial1 as the ROS interface; otherwise SerialUSB

enum CMD_SRC : uint16_t {CMD_SRC_NONE, CMD_SRC_ROS, CMD_SRC_RADIO};

#ifdef USE_USBCON
  #define DEBUG_SERIAL Serial1
#else
  #define DEBUG_SERIAL SerialUSB
#endif

bool CheckDebugSerial(void);

#endif
