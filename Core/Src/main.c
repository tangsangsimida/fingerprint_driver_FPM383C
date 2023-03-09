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
#define pack_head 						0xEF,0x01						//��ͷ
#define fingerprint_address				0xff,0xff,0xff,0xff				//оƬ��ַ
//over there is tatle auto
//under there is lcd_cmd
#define lcd_close 						0x00							//�ص�
#define lcd_color_red 					0x04							//���
#define lcd_color_blue					0x01							//����
#define lcd_color_green					0x02							//�̵�
#define lcd_color_red_green 			0x06							//�̺��
#define lcd_color_green_red				0x06							//�̺��
#define lcd_color_red_blue				0x05							//�����
#define lcd_color_blue_red				0x05							//�����
#define lcd_color_green_blue			0x03							//���̵�
#define lcd_color_blue_green			0x03							//���̵�
#define lcd_color_all_in				0x07							//�����̵�
#define pack_long_lcd_h					0x00							//lcd����----�����ȸ߰�λ
#define pack_long_lcd_l					0x07							//lcd����----�����ȵͰ�λ
#define pack_sign_lcd					0x01							//����־
#define num_of_cycle_infinite_lcd		0x00							//lcdѭ������
#define instruction_code_lcd			0x3C							//lcdָ����
#endif
//���ָ��ģ���Ƿ��ܹ�����ʹ��
void finger_print_check(void);
//lcd���׿���
void lcd_control(uint8_t ,uint8_t ,uint8_t , uint8_t );
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//lcd����ģʽ��------->>>>>>������
enum {
	breath = 0x01,//0x01 �����ƣ�
	flashing_light = 0x02,//0x02 ��˸��
	everbright = 0x03,//0x03 ����
	everclose = 0x04,//0x04 ����
	involote_lamp = 0x05,//0x05 ������
	involote_close = 0x06,//0x06 �����
}function_code_lcd;

//У���
uint8_t check_sum_h=0x00;
uint8_t check_sum_l=0x00;


//�����λ��У��λ�����㷽����
//�Ӱ���־��У��λ֮ǰһλ���еĺͣ������Ķ����ƽ�λ����													

uint8_t message_lcd_return[11];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//lcdУ��͹̶����ȣ�1+7+3c === 44+lcd����ģʽ+��ʼ��ɫ+������ɫ+ѭ������
//lcd�ƿ��ƺ���
					//lcd����ģʽ��					//��ʼ��ɫ   // ������ɫ    //ѭ������
void lcd_control(uint8_t function_code_lcd,uint8_t color_begin,uint8_t color_end, uint8_t time)
{
	uint32_t check_sum = pack_sign_lcd+pack_long_lcd_h+pack_long_lcd_l+instruction_code_lcd+function_code_lcd+color_begin+color_end+time;
	check_sum_l = check_sum&0xff;//��ȡdi��λ
	check_sum>>=8;
	check_sum_h = check_sum&0xff; //��ȡgao��λ
	uint8_t cmd[]={pack_head,fingerprint_address,pack_sign_lcd,pack_long_lcd_h,pack_long_lcd_l,instruction_code_lcd,function_code_lcd,color_begin,color_end,time,check_sum_h,check_sum_l};
	HAL_UART_Transmit(&huart3,cmd,16,200);//lcd�Ļ�����������ֻ��16���ֽ�
//	HAL_UART_Receive(&huart3,message_lcd_return,11,200);
//	HAL_UART_Transmit(&huart1,cmd,16,200);	
//	HAL_Delay(10);
//	HAL_UART_Transmit(&huart1,message_lcd_return,11,200);
}

//���ָ��ģ���Ƿ����ʹ�ã�����
//�򴮿�һ����ֵ�������в鿴����ֵ���ֲ�����ж�ָ���Ƿ���Ч
void finger_print_check()
{
	//У���= ����־+������ָ����
  uint32_t check_sum = pack_sign_lcd+0x03+0x36;
  check_sum_l = check_sum&0xff;//��ȡdi��λ
  check_sum>>=8;
  check_sum_h = check_sum&0xff; //��ȡgao��λ
	//				��ͷ			�豸��ַ			����־		��λ������	ָ����	У���	
  uint8_t cmd[] = {pack_head,fingerprint_address,pack_sign_lcd,0x00,0x03,0x36,check_sum_h,check_sum_l};
  HAL_UART_Transmit_IT(&huart3,cmd,12);
  HAL_UART_Receive_IT(&huart3,message_lcd_return,12);
  HAL_UART_Transmit_IT(&huart1,message_lcd_return,12);
}
//����ָ��ģ���������ģʽ���ҿ���PB1���������ж�
void set_sleep_finger_print()
{
  uint8_t cmd[] = {pack_head,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x33,0x00,0x37};
  HAL_UART_Transmit(&huart3,cmd,12,100);
  HAL_Delay(20);
  HAL_UART_Transmit(&huart1,cmd,12,100);
  HAL_UART_Receive(&huart3,cmd,12,100);
  HAL_UART_Transmit(&huart1,cmd,12,100);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);//����PB1���������ж�
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
  //��ʼ���ȹر��ⲿ�ж�PB1���ţ�������
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
   //��PB1�ж����Σ����Ұ�ָ��ģ������
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
