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
I2C_HandleTypeDef I2CHandle;

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

	gpio_config.Pull = GPIO_NOPULL;
	gpio_config.Speed = GPIO_SPEED_FREQ_HIGH;

	/* UART Begin */
	gpio_config.Mode = GPIO_MODE_AF_PP;
	gpio_config.Pin = GPIO_PIN_9;
	gpio_config.Alternate = GPIO_AF1_USART1;
	HAL_GPIO_Init(GPIOA, &gpio_config);
	gpio_config.Mode = GPIO_MODE_INPUT;
	gpio_config.Pin = GPIO_PIN_10;
	gpio_config.Alternate = GPIO_AF1_USART1;
	HAL_GPIO_Init(GPIOA, &gpio_config);
	/* UART End */

	/* ADC Begin */
	/* PA4 and PA6 INA199 out*/
	gpio_config.Mode = GPIO_MODE_ANALOG;
	gpio_config.Pin = GPIO_PIN_4;
	gpio_config.Alternate = 0;
	HAL_GPIO_Init(GPIOA, &gpio_config);
	gpio_config.Mode = GPIO_MODE_ANALOG;
	gpio_config.Pin = GPIO_PIN_6;
	gpio_config.Alternate = 0;
	HAL_GPIO_Init(GPIOA, &gpio_config);
	/* PA7 input Voltage */
	gpio_config.Mode = GPIO_MODE_ANALOG;
	gpio_config.Pin = GPIO_PIN_7;
	gpio_config.Alternate = 0;
	HAL_GPIO_Init(GPIOA, &gpio_config);
	/* ADC End */

	/* PF0 -> OLED_SDA PF1 -> OLED_SCL */
	gpio_config.Mode = GPIO_MODE_AF_OD;
	gpio_config.Pin = GPIO_PIN_0;
	gpio_config.Alternate = GPIO_AF1_I2C1;
	HAL_GPIO_Init(GPIOF, &gpio_config);

	gpio_config.Mode = GPIO_MODE_AF_OD;
	gpio_config.Pin = GPIO_PIN_1;
	gpio_config.Alternate = GPIO_AF1_I2C1;
	HAL_GPIO_Init(GPIOF, &gpio_config);
	/* PA3 and PA5 INA199 power */
	/* PA0 ? OLED_RST? */
	gpio_config.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_config.Pin = GPIO_PIN_0;
	gpio_config.Alternate = 0;
	HAL_GPIO_Init(GPIOA, &gpio_config);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

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

	UartHandle.Instance = USARTx;
	UartHandle.Init.BaudRate = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode = UART_MODE_TX_RX;

	if (HAL_UART_Init(&UartHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
	printf("\n\r");

	I2CHandle.Instance = I2C1;
	I2CHandle.Init.Timing = 0x50330309;
	I2CHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

	if (HAL_I2C_Init(&I2CHandle) != HAL_OK) {
		printf("I2C Init failed\r\n");
		Error_Handler();
	}

	ssd1306_Init();

	printf("\n\rInitialized\n\r");
	/* Infinite loop */
	while (1) {
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

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 72000000
 *            HCLK(Hz)                       = 72000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 2
 *            APB2 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 16000000
 *            HSE PREDIV1                    = 5
 *            HSE PREDIV2                    = 5
 *            PLL2MUL                        = 8
 *            Flash Latency(WS)              = 2
 * @param  None
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef clkinitstruct = {0};
	RCC_OscInitTypeDef oscinitstruct = {0};

	/* Configure PLLs
	 * ------------------------------------------------------*/

	/* PLL configuration: PLLCLK = (HSE / HSEPrediv2Value) * PLLMUL = (16/2)
	 * * 9 = 72 MHz */

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	oscinitstruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	oscinitstruct.HSEState = RCC_HSE_OFF;
	oscinitstruct.HSIState = RCC_HSI_ON;
	oscinitstruct.PLL.PLLState = RCC_PLL_ON;
	oscinitstruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	oscinitstruct.PLL.PLLMUL = RCC_PLL_MUL12;
	if (HAL_RCC_OscConfig(&oscinitstruct) != HAL_OK) {
		/* Initialization Error */
		while (1)
			;
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and
	   PCLK2 clocks dividers */
	clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
				   | RCC_CLOCKTYPE_PCLK1);
	clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_1)
	    != HAL_OK) {
		/* Initialization Error */
		while (1)
			;
	}
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
	while (1) {
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
	while (1) {
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
