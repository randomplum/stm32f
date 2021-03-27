/**
 ******************************************************************************
 * @file    app/src/main.c
 * @author  MCD Application Team
 * @version V1.5.0
 * @date    14-April-2017
 * @brief
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *notice, this list of conditions and the following disclaimer in the
 *documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef UartHandle;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

void Pinmux_Config(void)
{
	GPIO_InitTypeDef gpio_config;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	USARTx_RX_GPIO_CLK_ENABLE();
	USARTx_TX_GPIO_CLK_ENABLE();

	gpio_config.Pull = GPIO_NOPULL;
	gpio_config.Speed = GPIO_SPEED_FREQ_HIGH;

	/* UART Begin */
	gpio_config.Mode = GPIO_MODE_AF_PP;
	gpio_config.Pin = USARTx_TX_PIN;
	gpio_config.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &gpio_config);
	gpio_config.Mode = GPIO_MODE_INPUT;
	gpio_config.Pin = USARTx_RX_PIN;
	gpio_config.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &gpio_config);
	/* UART End */
}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{

	HAL_Init();

	/* Configure the system clock to 72 MHz */
	SystemClock_Config();

	/* Configure the pin muxing */
	Pinmux_Config();

	USARTx_CLK_ENABLE()
	UartHandle.Instance = USARTx;
	UartHandle.Init.BaudRate = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = UART_PARITY_NONE;
	UartHandle.Init.Mode = UART_MODE_TX_RX;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}
	printf("\n\r");

	printf("\n\rInitialized\n\r");
	/* Infinite loop */
	while (1)
	{
		printf("Hi!\r\n");
		HAL_Delay(1024);
	}
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of
	 * transmission */
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

	return ch;
}

void SystemClock_Config(void)
{

	/* Reset the RCC clock configuration to the default reset state ------------*/
	/* Set HSION bit */
	RCC->CR |= (uint32_t)0x00000001U;

#if defined(STM32F051x8) || defined(STM32F058x8)
	/* Reset SW[1:0], HPRE[3:0], PPRE[2:0], ADCPRE and MCOSEL[2:0] bits */
	RCC->CFGR &= (uint32_t)0xF8FFB80CU;
#else
	/* Reset SW[1:0], HPRE[3:0], PPRE[2:0], ADCPRE, MCOSEL[2:0], MCOPRE[2:0] and PLLNODIV bits */
	RCC->CFGR &= (uint32_t)0x08FFB80CU;
#endif /* STM32F051x8 or STM32F058x8 */

	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)0xFEF6FFFFU;

	/* Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFFU;

	/* Reset PLLSRC, PLLXTPRE and PLLMUL[3:0] bits */
	RCC->CFGR &= (uint32_t)0xFFC0FFFFU;

	/* Reset PREDIV[3:0] bits */
	RCC->CFGR2 &= (uint32_t)0xFFFFFFF0U;

#if defined(STM32F072xB) || defined(STM32F078xx)
	/* Reset USART2SW[1:0], USART1SW[1:0], I2C1SW, CECSW, USBSW and ADCSW bits */
	RCC->CFGR3 &= (uint32_t)0xFFFCFE2CU;
#elif defined(STM32F071xB)
	/* Reset USART2SW[1:0], USART1SW[1:0], I2C1SW, CECSW and ADCSW bits */
	RCC->CFGR3 &= (uint32_t)0xFFFFCEACU;
#elif defined(STM32F091xC) || defined(STM32F098xx)
	/* Reset USART3SW[1:0], USART2SW[1:0], USART1SW[1:0], I2C1SW, CECSW and ADCSW bits */
	RCC->CFGR3 &= (uint32_t)0xFFF0FEACU;
#elif defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F030xC)
	/* Reset USART1SW[1:0], I2C1SW and ADCSW bits */
	RCC->CFGR3 &= (uint32_t)0xFFFFFEECU;
#elif defined(STM32F051x8) || defined(STM32F058xx)
	/* Reset USART1SW[1:0], I2C1SW, CECSW and ADCSW bits */
	RCC->CFGR3 &= (uint32_t)0xFFFFFEACU;
#elif defined(STM32F042x6) || defined(STM32F048xx)
	/* Reset USART1SW[1:0], I2C1SW, CECSW, USBSW and ADCSW bits */
	RCC->CFGR3 &= (uint32_t)0xFFFFFE2CU;
#elif defined(STM32F070x6) || defined(STM32F070xB)
	/* Reset USART1SW[1:0], I2C1SW, USBSW and ADCSW bits */
	RCC->CFGR3 &= (uint32_t)0xFFFFFE6CU;
	/* Set default USB clock to PLLCLK, since there is no HSI48 */
	RCC->CFGR3 |= (uint32_t)0x00000080U;
#else
#warning "No target selected"
#endif

	/* Reset HSI14 bit */
	RCC->CR2 &= (uint32_t)0xFFFFFFFEU;

	/* Disable all interrupts */
	RCC->CIR = 0x00000000U;
	SystemCoreClockUpdate();
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
	while (1)
	{
	}
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line
	   number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
	   line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
