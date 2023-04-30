#include "UART.h"
#include "printf.h"

void uart1Init()
{
  SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);

  /*  set to  output*/
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE9_Msk, (0b01 << GPIO_MODER_MODE9_Pos));

  /*  setto second lowest speed (0b01)*/
  MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED9_Msk, (0b01 << GPIO_OSPEEDR_OSPEED9_Pos));

  /*  enable pullldowns*/
  MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD9_Msk, (0b10 << GPIO_PUPDR_PUPD9_Pos));

  SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR9);

  SET_BIT(RCC->APBENR2, RCC_APBENR2_USART1EN);

  //configure PB6 and PB7 for UART1

  //set PB6 and PB7 to alternate mode
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE6_Msk, (0b10 << GPIO_MODER_MODE6_Pos));
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE7_Msk, (0b10 << GPIO_MODER_MODE7_Pos));
  //set PB6 (TX) to pushpull, PB7 doesn't matter - set to reset value
  CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT6);
  CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT7);
  //set PB6 and PB7 to slowest speed
  MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED6_Msk, (0b00 << GPIO_OSPEEDR_OSPEED6_Pos));
  MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED7_Msk, (0b00 << GPIO_OSPEEDR_OSPEED7_Pos));
  //enable pulldowns for PB6 and PB7
  MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD6_Msk, (0b01 << GPIO_PUPDR_PUPD6_Pos));
  MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD7_Msk, (0b01 << GPIO_PUPDR_PUPD7_Pos));
  //set PB6 and PB7 to AF0
  MODIFY_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL6_Msk, 0);
  MODIFY_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL7_Msk, 0);




  //configure UART1 for .5 Mbps baud rate, 8 data bits, 1 start bit, 1 stop bit
  //enable interrupt on rx and overrun detection
  //UART1->CR1 = 0x00000020;
  WRITE_REG(USART1->CR1, USART_CR1_FIFOEN | USART_CR1_RXFFIE);
  //UART1->CR2 = 0x00000000;
  WRITE_REG(USART1->CR2, USART_CR2_SWAP);
  //UART1->CR3 = 0x00001001;
  WRITE_REG(USART1->CR3, 0);
  //UART1->BRR = 128;//SYSCLK (64000000)  /128 = 500000;
  WRITE_REG(USART1->BRR, 128);
  //UART1->GTPR = 0;
  WRITE_REG(USART1->GTPR, 0);
  //UART1->RTOR = 0;
  WRITE_REG(USART1->RTOR, 0);
  //UART1->RQR = 0x00000000;
  WRITE_REG(USART1->RQR, 0);
  //UART1->ICR = 0x00000000;
  WRITE_REG(USART1->ICR, 0);

  //timeout functionality TEST THIS!!!!!!! -could be useful -nvmd tried it and it seems to be fucking useless
  /*UART1->RTOR = 230400;// 2s / (1/115200)s
  UART1->CR1 |= 0b1 << 26;
  UART1->CR2 |= 0b1 << 23;//enable timeout interrupt*/

  //enable uart1 interrupt in NVIC
  //*((uint32_t *) NVIC_ISER_ADDR + 1) = 0b1 << 5;
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_ClearPendingIRQ(USART1_IRQn);
  NVIC_EnableIRQ(USART1_IRQn);

  //enable uart operation (rx)
  //USART1->CR1 |= 0x000000D;
  MODIFY_REG(USART1->CR1, 0, USART_CR1_UE | USART_CR1_RE );
  crcInit();
}

//expects an 8 byte array
void uart1TransmitPacket(uint8_t * buffer)//could also just pass in a long
{
  NVIC_DisableIRQ(USART1_IRQn);

  uint8_t packet[9] = {buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], 0};
  packet[8] = crcBlock(packet, 8);//CRC

  MODIFY_REG(USART1->CR1, USART_CR1_RE, USART_CR1_TE );//disable receiver, enable transmitter
  //SET_BIT(USART1->RQR, USART_RQR_TXFRQ);//flush tx
  uint32_t i = 0;
  for(i = 0; i < 8; ++i)
  {
      USART1->TDR = packet[i];
  }
  while(!(READ_BIT(USART1->ISR, USART_ISR_TXE_TXFNF)));
  USART1->TDR = packet[8];
  //Wait for tranmission to finish, then disable tranceiver, reenable reciver
  while(!(READ_BIT(USART1->ISR, USART_ISR_TC)));

  NVIC_ClearPendingIRQ(USART1_IRQn);
  NVIC_EnableIRQ(USART1_IRQn);

  MODIFY_REG(USART1->CR1, USART_CR1_TE, USART_CR1_RE );//disable transmitter, enanble receiver
}