/* USER CODE BEGIN Header */
/**
 ******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* Copyright (c) 2025 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"
#include "io.h"
#include "uart.h"
#include "string.h"
#include "pwm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

/* USER CODE END 0 */

uint8_t buf[20] = "hello world\n\r";

/**
 * @brief  The application entry point.
 * @retval int
 */

// USART_Handle_t *USART1_Handle;
// UART_HandleTypeDef *USART1_Handle;

USART_Handle_t uart2_h;
io_handle_t btn_h;

void init_led_gpio(void)
{
	io_handle_t led_h;
	led_h.GPIOx = GPIOA;

	led_h.IO_Confg.PIN_NO = IO_PIN_5;
	led_h.IO_Confg.PIN_MODE = IO_MODE_ALT_FUN;
	led_h.IO_Confg.PIN_ALT_FUN_MODE = IO_ALT_FUN_MODE1;
	led_h.IO_Confg.PIN_SPEED = IO_SPEED_FAST;
	led_h.IO_Confg.PIN_OPTYPE = IO_OPTYPE_PP;
	led_h.IO_Confg.PIN_RESISTANCE = IO_RES_NOPUPD;

	IO_Config(&led_h);
}

void init_btn_gpio(void)
{
	btn_h.GPIOx = GPIOC;

	btn_h.IO_Confg.PIN_MODE = IO_MODE_INPUT;
	btn_h.IO_Confg.PIN_NO = IO_PIN_13;
	btn_h.IO_Confg.PIN_OPTYPE = IO_OPTYPE_PP;
	btn_h.IO_Confg.PIN_RESISTANCE = IO_RES_NOPUPD;
	btn_h.IO_Confg.PIN_SPEED = IO_SPEED_FAST;

	IO_Config(&btn_h);
}

void init_usart_gpio(void)
{
	io_handle_t usart_io_pins;
	usart_io_pins.GPIOx = GPIOA;
	usart_io_pins.IO_Confg.PIN_MODE = IO_MODE_ALT_FUN;
	usart_io_pins.IO_Confg.PIN_ALT_FUN_MODE = IO_ALT_FUN_MODE7;
	usart_io_pins.IO_Confg.PIN_NO = IO_PIN_2;
	usart_io_pins.IO_Confg.PIN_OPTYPE = IO_OPTYPE_PP;
	usart_io_pins.IO_Confg.PIN_RESISTANCE = IO_RES_NOPUPD;
	usart_io_pins.IO_Confg.PIN_SPEED = IO_SPEED_FAST;

	IO_Config(&usart_io_pins);

	usart_io_pins.IO_Confg.PIN_NO = IO_PIN_3;
	IO_Config(&usart_io_pins);
}

void usart1_init(void)
{
	uart2_h.TX_STATE = USART_TX_READY;
	uart2_h.USARTx = USART2;
	uart2_h.USART_Confg.USART_BaudRate = 115200;
	uart2_h.USART_Confg.USART_Mode = 0;
	uart2_h.USART_Confg.USART_HWFlowCtrl = USART_NO_HWFlowCtrl;
	uart2_h.USART_Confg.USART_ParityCtrl = USART_PARITY_CRL_DI;
	uart2_h.USART_Confg.USART_WordLen = USART_WORD_LEN8;

	usart_init(&uart2_h);
}

void led_blink(void)
{
	for (volatile int i = 0; i < 350000; ++i);
	GPIOA->ODR ^= (0x1U << GPIO_ODR_OD5_Pos);
	usart_send_data(&uart2_h, (uint8_t*)buf, strlen((char*)buf));
}

int main(void)
{
	SystemClock_Config();
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	// RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	// RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	// init_btn_gpio();
	init_led_gpio();
	// init_usart_gpio();
	// usart1_init();

	// IO_InitIT(&btn_h, IO_INTERRUPT_RT, led_blink);
	// IO_SetInterruptPriority(EXTI15_10_IRQ_NO, 1);
	// IO_IRQEnableInterrupt(EXTI15_10_IRQ_NO);

	// IO_SetInterruptPriority(USART2_IRQ_NO, 2);
	// USART_IRQEnableInterrupt(USART2_IRQ_NO);

	const uint8_t duty_cycles[] = {100, 75, 25, 1, 0};
	// const uint16_t delay = 3000;
	pwm_init();
	uint8_t i = 0;
	while(1)
	{
		for (volatile int j = 0; j < 350000; ++j);
		pwm_set_duty_cycle(PWM_TB6612FNG_LEFT, duty_cycles[i]);
		i = ((i + 1) % 5);
	}

	return 0;
}

void USART2_IRQHandler(void)
{
    usart_irq_handler(&uart2_h);
}

/**
 * @brief System Clock Configuration
 * @retval None
 */

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/**
 	* Configure the main internal regulator output voltage
	*/
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
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
