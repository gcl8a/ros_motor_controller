#ifndef __SABERTOOTH_H
#define __SABERTOOTH_H

#include "motor_driver.h"
#include <wiring_private.h> 

#define RC_LOW    100
#define RC_CENTER 185
#define RC_HIGH   270

#define EMERGENCY_PIN 3
#define ADDRESS_BYTE 128

//declare a UART SERCOM for communicating through packet serial
extern Uart Serial2; // on SERCOM3
void SERCOM3_Handler();

class Sabertooth : public MotorDriver
{
protected:
  uint8_t commMode = COMM_NONE;
  
public:
  Sabertooth(void)
  {
    //don't set pins or registers here since this gets called before standard Arduino setup
    //use Init instead
  }

  void Init(uint8_t mode)
  {
    DEBUG_SERIAL.println("Sabertooth::Init");
    MotorDriver::Init();
    commMode = mode;
 
    if(commMode == COMM_RC)
    {}

    else if(commMode == COMM_PACKET_SERIAL)
    {
      Serial2.begin(9600);
  
      //Assign pin 2 SERCOM functionality -- why does this have to be after begin() -- who dropped the ball on that one?
      pinPeripheral(6, PIO_SERCOM_ALT);

      //Don't mux pin 7, but pull it HIGH
      pinMode(EMERGENCY_PIN, INPUT_PULLUP);
    }

    else commMode = COMM_NONE;
    DEBUG_SERIAL.println("/Sabertooth::Init");
  }

  void EmergencyStop(void)
  {
    if(commMode == COMM_PACKET_SERIAL)
    {
      //in packet mode, we can disable using the second input
      pinMode(EMERGENCY_PIN, OUTPUT);
      digitalWrite(EMERGENCY_PIN, LOW);
    }

    MotorDriver::EmergencyStop();
  }

protected:  
  void SendPowers(int16_t powerA, int16_t powerB)
  {   
    if(commMode == COMM_RC)
    {}

    else if(commMode == COMM_PACKET_SERIAL)
    {
      uint8_t valueA = (powerA > 0) ? powerA : -powerA;
      if(valueA > 127) valueA = 127;
      
      uint8_t cmdA = 0;
      cmdA = (powerA > 0) ? 4 : 5;

      uint8_t valueB = (powerB > 0) ? powerB : -powerB;
      if(valueB > 127) valueB = 127;
      
      uint8_t cmdB = 0;
      cmdB = (powerB > 0) ? 0 : 1;

      Serial2.write(ADDRESS_BYTE);
      Serial2.write(cmdA);
      Serial2.write(valueA);
      Serial2.write((ADDRESS_BYTE + cmdA + valueA) & 0x7F);
      
      Serial2.write(ADDRESS_BYTE);
      Serial2.write(cmdB);
      Serial2.write(valueB);
      Serial2.write((ADDRESS_BYTE + cmdB + valueB) & 0x7F);
    }
  }

  uint16_t PowerToRC(int16_t power)
  {
    uint16_t rc = RC_CENTER + power / 4;
    return constrain(rc, RC_LOW, RC_HIGH);
  }
};

/*
 * This is not being used...just thinking how it would look...
 */
class SabertoothPacketized : public Sabertooth
{
protected:
  uint8_t address = 128;

public:
  void SendPowers(int16_t powerA, int16_t powerB)
  {   
      uint8_t valueA = (powerA > 0) ? powerA : -powerA;
      if(valueA > 127) valueA = 127;
      
      uint8_t cmdA = 0;
      cmdA = (powerA > 0) ? 4 : 5;

      uint8_t valueB = (powerB > 0) ? powerB : -powerB;
      if(valueB > 127) valueB = 127;
      
      uint8_t cmdB = 0;
      cmdB = (powerB > 0) ? 0 : 1;

      Serial2.write(address);
      Serial2.write(cmdA);
      Serial2.write(valueA);
      Serial2.write((address + cmdA + valueA) & 0x7F);
      
      Serial2.write(address);
      Serial2.write(cmdB);
      Serial2.write(valueB);
      Serial2.write((address + cmdB + valueB) & 0x7F);
  }
};

#endif
