/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "motor.h"
#include "encoder.h"
#include "PID.h"
#include "IIC.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "sr04.h"
#include "stdio.h"
#include "serial_plot.h"
#include "ai.h"
#include "bluetooth.h"
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
extern float roll;
extern  float pitch;
extern float  yaw;
extern short  gyrox;
extern int Encoder_Left,Encoder_Right;
extern  float distance;

uint8_t display_buf[20];
uint32_t sys_tick;
extern int vertical_out;
extern int velocity_out;
extern int MOTO1;

extern uint8_t rx_buf[2];

//////////////////////////////////////////////////

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Read(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */


  // ↓↓↓释放PB3/PB4 ↓↓↓
  __HAL_RCC_AFIO_CLK_ENABLE();          // 开启AFIO时钟
  __HAL_AFIO_REMAP_SWJ_NOJTAG();        // 关闭JTAG，释放PB3/PB4
  HAL_Delay(20);                        // 给引脚稳定时间



  OLED_Init();
  OLED_Clear();

  MPU_Init();
  mpu_dmp_init();

  //OLED_ShowString(0,00,(uint8_t*)"Init Sucess",16);

  //////////////////////////////////////
  
   HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  ////////////////////////////
  ///
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
  HAL_UART_Receive_IT(&huart3,rx_buf,1);
  Load(0,0);
////
OLED_Clear();
  // AI_UART_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    // HAL_Delay(500);
   // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    //HAL_Delay(500);

    //HAL_UART_Transmit(&huart3,(uint8_t*)"Hello World!\r\n",12,HAL_MAX_DELAY);
    // //////////////chao sheng bo

    // GET_Distance();
    // sprintf((char *) display_buf, "distance:%.2f   ", distance);
    // OLED_ShowString(0, 4, display_buf, 16);
    // ////////////////////////////////////////////////////////////////////////

    ////////////Encoder_Value
     // Read();
     // sprintf((char*)display_buf,"Encoder_L:%d   ",Encoder_Left);
     // OLED_ShowString(0,4,display_buf,16);
     // sprintf((char*)display_buf,"Encoder_R:%d   ",Encoder_Right);
     // OLED_ShowString(0,6,display_buf,16);



    // sprintf((char*)display_buf,"test:%.2f",test_num);
    // OLED_ShowString(0,2,display_buf,16); // 显示在第8行，不影响原有内容
    // HAL_Delay(2000); // 停留2秒看结果

/////////////////////陀螺仪
    // HAL_Delay(10);
    // mpu_dmp_get_data(&pitch,&roll,&yaw);
    //
    // sprintf((char*)display_buf,"pitch:%.2f   ",pitch);
    // OLED_ShowString(0,2,display_buf,16);
    //
    // sprintf((char*)display_buf,"roll:%.2f     ",roll);
    // OLED_ShowString(0,4,display_buf,16);
    //
    // sprintf((char*)display_buf,"yaw:%.2f      ",yaw);
    // OLED_ShowString(0,6,display_buf,16);

    // ////////////////////////////////////////////
    // ///声波加编码器

       Read();
      sprintf((char *)display_buf,"Encoder_L:%d     ",Encoder_Left);
      OLED_ShowString(0,0,display_buf,16);
      sprintf((char *)display_buf,"Encoder_R:%d     ",Encoder_Right);
      OLED_ShowString(0,2,display_buf,16);
      sprintf((char *)display_buf,"roll:%.2f     ",roll);
      OLED_ShowString(0,4,display_buf,16);
       GET_Distance();
       sprintf((char *)display_buf,"distance:%.1f  ",distance);
      OLED_ShowString(0,6,display_buf,12);





   // SerialPlot_SendWave(roll, Vertical_out, MOTO1);//波形图

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

////////
//du qu dian ji sudu
void Read(void)
{
  if(uwTick-sys_tick<10)
    return;
  sys_tick=uwTick;
  Encoder_Left=Read_Speed(&htim2);
  Encoder_Right=-Read_Speed(&htim4);//-是方向相反
}
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
