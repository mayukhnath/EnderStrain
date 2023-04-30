/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define CALIBRATION_INPUT//uncomment to enable calibration requests (driving PA11 High generates the request)
#define UART_OUTPUT
#define THRESHOLD 10000
#define BUFFER_SIZE 20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t readADC(void)
{
  uint32_t count;
  uint32_t i;
  GPIOB->BSRR = 0b1 << 31;
  count = 0;
  volatile int j = 0;//used later to waste cycles in order to meed HX711/HX717 timing requirements
  while(GPIOB->IDR & (0b1 << 14));
  for (i=0;i<24;i++)
  {
    GPIOB->BSRR = 0b1 << 15;
    count=count<<1;
    ++j;
    ++j;//wati cycles
    GPIOB->BSRR = 0b1 << 31;
    count = count + ((GPIOB->IDR & (0b1 << 14)) >> 14);
  }
  GPIOB->BSRR = 0b1 << 15;
  count = count^0x800000;//converts to unsigned
  ++j;
  ++j;//waste cycles
  GPIOB->BSRR = 0b1 << 31;
  return count;
}

void initADC()
{
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER14_Msk, 0);
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER15_Msk, 0b01 << GPIO_MODER_MODER15_Pos);
  CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_15);
  MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEEDR15_Msk, 0b01 << GPIO_OSPEEDR_OSPEEDR15_Pos);
  MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPDR14_Msk, 0);
  MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPDR15_Msk, 0);

  readADC();
  readADC();
  readADC();
  readADC();//dummy reads - takes 4 cycles to init the HX711/HX717
}

void initTriggerOutput(void)
{
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER12_Msk, 0b01 << GPIO_MODER_MODER12_Pos);
  SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_12);
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR12_Msk, 0b01 << GPIO_OSPEEDR_OSPEEDR12_Pos);
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPDR12_Msk, 0b10 << GPIO_PUPDR_PUPDR12_Pos);
}

void initCalibrationInput(void)
{
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER11_Msk, 0);
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPDR11_Msk, 0b01 << GPIO_PUPDR_PUPDR11_Pos);
}

volatile uint32_t buffer[BUFFER_SIZE];
volatile uint32_t buffer_pos= 0;

uint32_t initializeBuffer(void)
{
  uint32_t sum = 0;
  buffer_pos = 0;
  for(uint32_t i = 0; i < BUFFER_SIZE; ++i) sum += (buffer[i] = readADC());
  return sum / BUFFER_SIZE;
}


void initUART()
{
  MODIFY_REG(RCC->CFGR3, RCC_CFGR3_USART1SW_Msk, RCC_CFGR3_USART1SW_PCLK);
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);

  //configure PA9 and PA10 for USART1

  //set PA9 and PA10 to alternate mode
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER9_Msk, (0b10 << GPIO_MODER_MODER9_Pos));
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER10_Msk, (0b10 << GPIO_MODER_MODER10_Pos));
  //set PA9 (TX) to pushpull, PA10 doesn't matter - set to reset value
  CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_9);
  CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_10);
  //set PA9 and PA10 to second slowest speed
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR9_Msk, (0b01 << GPIO_OSPEEDR_OSPEEDR9_Pos));
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR10_Msk, (0b01 << GPIO_OSPEEDR_OSPEEDR10_Pos));
  //enable pullups for PA9 and PA10
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPDR9_Msk, (0b01 << GPIO_PUPDR_PUPDR9_Pos));
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPDR10_Msk, (0b01 << GPIO_PUPDR_PUPDR10_Pos));
  //set PA9 and PA10 to AF1
  MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL9_Msk, 1 << GPIO_AFRH_AFSEL9_Pos);
  MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL10_Msk, 1 << GPIO_AFRH_AFSEL10_Pos);

  //configure USART1 for 11.52 kbps baud rate, 8 data bits, 1 start bit, 1 stop bit
  WRITE_REG(USART1->CR1, 0);
  WRITE_REG(USART1->CR2, 0);
  WRITE_REG(USART1->CR3, 0);
  //UART1->BRR =  0x1A1 -> 115.2k baud;
  WRITE_REG(USART1->BRR,  0x1A1);
  WRITE_REG(USART1->GTPR, 0);
  WRITE_REG(USART1->RTOR, 0);
  WRITE_REG(USART1->RQR, 0);
  WRITE_REG(USART1->ICR, 0);

  //enable uart operation (tx)
  SET_BIT(USART1->CR1, USART_CR1_UE | USART_CR1_TE );
}

void send16bUART(uint16_t val)
{
  //we can send 2 in rapid sucession as the first will immediately go to the transmitter hardware, and second will be stored in TDR reg
  USART1->TDR = (uint8_t) (val >> 8);
  USART1->TDR = (uint8_t) (val & 0xF);
}

void send8bUART(uint8_t val)
{
  USART1->TDR = val;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  GPIOA->BSRR = 0b1 << 28;
  initADC();
  initTriggerOutput();
  #ifdef UART_OUTPUT
  initUART();
  #endif
  #ifdef CALIBRATION_INPUT
  initCalibrationInput();
  #endif
  uint32_t baseline = initializeBuffer();
  uint32_t sum = baseline * BUFFER_SIZE;
  uint32_t val = baseline;
  while (1)
  {
	  val = readADC();
    if(((val > baseline) && ((val - baseline) > THRESHOLD)) || ((val < baseline) && ((baseline - val) > THRESHOLD))) GPIOA->BSRR = 0b1 << 12;
    else GPIOA->BSRR = 0b1 << 28;
    
    //instead of summing the contents of the buffer everytime, we simply subtract the oldest reading from sum and add the newest
    sum -= buffer[buffer_pos];
    sum += (buffer[buffer_pos++] = val);
    if(buffer_pos == BUFFER_SIZE) buffer_pos = 0;
    baseline = sum / BUFFER_SIZE;

    #ifdef CALIBRATION_INPUT
    if(GPIOA->IDR & (0b1 << 11))
    {
      volatile int i = 0;
      for(; i < 100; ++i);//wait a little bit and double check (i.e. debounce)
      if(GPIOA->IDR & (0b1 << 11))
      {
        baseline = initializeBuffer();
        sum = baseline * BUFFER_SIZE;
      }
    }
    #endif
    #ifdef UART_OUTPUT
    send16bUART((uint16_t) (val >> 8));
    #endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(48000000);
  LL_SetSystemCoreClock(48000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
