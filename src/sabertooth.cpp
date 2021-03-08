#include "sabertooth.h"

//declare a UART SERCOM for communicating through packet serial
Uart Serial2 (&sercom2, 3, 2, SERCOM_RX_PAD_1, UART_TX_PAD_2);
void SERCOM2_Handler()
{
  Serial2.IrqHandler();
}