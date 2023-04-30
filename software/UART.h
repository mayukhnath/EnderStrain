#ifndef INC_UART_H_
#define INC_UART_H_

#include <stdint.h>
#include "stm32g0xx_ll_rcc.h"
#include "CRC.h"

void uart1Init();

void uart1TransmitPacket(uint8_t * buffer);

#endif /* INC_UART_H_ */