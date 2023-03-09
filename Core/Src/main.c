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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifndef pack_head
#define pack_head 						0xEF,0x01						//包头
#define fingerprint_address				0xff,0xff,0xff,0xff				//芯片地址
//over there is tatle auto
//under there is lcd_cmd
#define lcd_close 						0x00							//关灯
#define lcd_color_red 					0x04							//红灯
#define lcd_color_blue					0x01							//蓝灯
#define lcd_color_green					0x02							//绿灯
#define lcd_color_red_green 			0x06							//绿红灯
#define lcd_color_green_red				0x06							//绿红灯
#define lcd_color_red_blue				0x05							//蓝红灯
#define lcd_color_blue_red				0x05							//蓝红灯
#define lcd_color_green_blue			0x03							//蓝绿灯
#define lcd_color_blue_green			0x03							//蓝绿灯
#define lcd_color_all_in				0x07							//红蓝绿灯
#define pack_long_lcd_h					0x00							//lcd命令----包长度高八位
#define pack_long_lcd_l					0x07							//lcd命令----包长度低八位
#define pack_sign_lcd					0x01							//包标志
#define num_of_cycle_infinite_lcd		0x00							//lcd循环次数
#define instruction_code_lcd			0x3C							//lcd指令码
#endif
//检查指纹模块是否能够正常使用
void finger_print_check(void);
//lcd简易控制
void lcd_control(uint8_t ,uint8_t ,uint8_t , uint8_t );
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//lcd亮灯模式，------->>>>>>功能码
enum {
	breath = 0x01,//0x01 呼吸灯，
	flashing_light = 0x02,//0x02 闪烁灯
	everbright = 0x03,//0x03 常亮
	everclose = 0x04,//0x04 长灭
	involote_lamp = 0x05,//0x05 渐开灯
	involote_close = 0x06,//0x06 渐灭灯
}function_code_lcd;

//校验和
uint8_t check_sum_h=0x00;
uint8_t check_sum_l=0x00;


//最后两位是校验位，计算方法：
//从包标志到校验位之前一位所有的和，超过的二进制进位忽略													

uint8_t message_lcd_return[11];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//lcd校验和固定长度：1+7+3c === 44+lcd亮灯模式+起始颜色+结束颜色+循环次数
//lcd灯控制函数
					//lcd亮灯模式，					//起始颜色   // 结束颜色    //循环次数
void lcd_control(uint8_t function_code_lcd,uint8_t color_begin,uint8_t color_end, uint8_t time)
{
	uint32_t check_sum = pack_sign_lcd+pack_long_lcd_h+pack_long_lcd_l+instruction_code_lcd+function_code_lcd+color_begin+color_end+time;
	check_sum_l = check_sum&0xff;//获取di八位
	check_sum>>=8;
	check_sum_h = check_sum&0xff; //获取gao八位
	uint8_t cmd[]={pack_head,fingerprint_address,pack_sign_lcd,pack_long_lcd_h,pack_long_lcd_l,instruction_code_lcd,function_code_lcd,color_begin,color_end,time,check_sum_h,check_sum_l};
	HAL_UART_Transmit(&huart3,cmd,16,200);//lcd的基本控制命令只有16个字节
//	HAL_UART_Receive(&huart3,message_lcd_return,11,200);
//	HAL_UART_Transmit(&huart1,cmd,16,200);	
//	HAL_Delay(10);
//	HAL_UART_Transmit(&huart1,message_lcd_return,11,200);
}

//检查指纹模块是否可以使用！！！
//向串口一返回值！！自行查看返回值与手册对照判断指纹是否有效
void finger_print_check()
{
	//校验和= 包标志+……加指令码
  uint32_t check_sum = pack_sign_lcd+0x03+0x36;
  check_sum_l = check_sum&0xff;//获取di八位
  check_sum>>=8;
  check_sum_h = check_sum&0xff; //获取gao八位
	//				包头			设备地址			包标志		两位包长度	指令码	校验和	
  uint8_t cmd[] = {pack_head,fingerprint_address,pack_sign_lcd,0x00,0x03,0x36,check_sum_h,check_sum_l};
  HAL_UART_Transmit_IT(&huart3,cmd,12);
  HAL_UART_Receive_IT(&huart3,message_lcd_return,12);
  HAL_UART_Transmit_IT(&huart1,message_lcd_return,12);
}
//设置指纹模块进入休眠模式并且开启PB1的上升沿中断
void set_sleep_finger_print()
{
  uint8_t cmd[] = {pack_head,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x33,0x00,0x37};
  HAL_UART_Transmit(&huart3,cmd,12,100);
  HAL_Delay(20);
  HAL_UART_Transmit(&huart1,cmd,12,100);
  HAL_UART_Receive(&huart3,cmd,12,100);
  HAL_UART_Transmit(&huart1,cmd,12,100);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);//开启PB1的上升沿中断
}
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //初始化先关闭外部中断PB1引脚，，，，
  HAL_NVIC_DisableIRQ(EXTI1_IRQn);


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  lcd_control(flashing_light,lcd_color_blue,lcd_color_blue,10);
   //把PB1中断屏蔽，并且把指纹模块休眠
  set_sleep_finger_print();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
