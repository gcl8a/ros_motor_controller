#include "sabertooth.h"

// SAMD21 mini:
// //declare a UART SERCOM for communicating through packet serial
// Uart Serial2 (&sercom2, 3, 2, SERCOM_RX_PAD_1, UART_TX_PAD_2);
// void SERCOM2_Handler()
// {
//   Serial2.IrqHandler();
// }

// MKR1000
// we don't use the RX for motor control, but instead for E-stop
// we include it here to make the constructor happy, but we don't
// change the mux in the init later
Uart Serial2 (&sercom3, 7, 6, SERCOM_RX_PAD_3, UART_TX_PAD_2); //both ALT
void SERCOM3_Handler()
{
  Serial2.IrqHandler();
}

