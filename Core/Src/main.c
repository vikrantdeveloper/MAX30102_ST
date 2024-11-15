
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
UART_HandleTypeDef huart3;
I2C_HandleTypeDef hi2c2;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "erlog.h"
#include "max30102.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
log_t log_console;     /*log the data onto window console*/
max30102_t max30102;   /*MAX30102 object*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void sd_open();
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float temp , spo2 = 0;
float beatsPerMinute = 0;
float beatAvg = 0;
volatile bool temperature =  false;
UINT bytesWrote;
// Variables for FatFs
FATFS FatFs;    // FatFs handle
FIL fil;        // File handle
FRESULT fres;   // Result after operations


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
     if (GPIO_Pin == GPIO_PIN_0)
     {
		temperature = true;
   	  }
}


void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len, -1);

}
void sd_init()
{

	 myprintf("\r\n~ SD card started ~\r\n\r\n");
    HAL_Delay(1000);
	    // Mount the file system
	    fres = f_mount(&FatFs, "/", 1); // 1 = mount immediately
	    if (fres != FR_OK) {
	        myprintf("f_mount error (%i)\r\n", fres);
	        // Infinite loop on failure
	    } else {
	        myprintf("f_mount success (%i)\r\n", fres);
	    }

	    // Variables for free space calculation
	    DWORD free_clusters, free_sectors, total_sectors;
	    FATFS* getFreeFs;

	    // Get free space
	    fres = f_getfree("", &free_clusters, &getFreeFs);
	    if (fres != FR_OK) {
	        myprintf("f_getfree error (%i)\r\n", fres);
	        // Infinite loop on failure
	    }

	    // Calculate total and free space (formula from ChaN's documentation)
	    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
	    free_sectors = free_clusters * getFreeFs->csize;

	    myprintf("SD card stats:\r\n");
	    myprintf("%10lu KiB total drive space.\r\n", total_sectors / 2);
	    myprintf("%10lu KiB available.\r\n", free_sectors / 2);

	    // Open file for writing
	    sd_open();



}
void sd_write()
{
	fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_APPEND);
	if (fres == FR_OK)
			    {
			        myprintf("Opened 'write.txt' for writing successfully.\r\n");
			    } else
			    {
			        myprintf("f_open error (%i)\r\n", fres);
			        while (1); // Infinite loop on failure
			    }
			 fres = f_write(&fil, log_console.msg, log_console.msg_len, &bytesWrote);
			 if (fres == FR_OK)
			 		    {
			 		        myprintf("Opened 'write.txt' for writing successfully.\r\n");
			 		    } else
			 		    {
			 		        myprintf("f_open error (%i)\r\n", fres);
			 		        while (1); // Infinite loop on failure
			 		    }
	f_close(&fil);
}
void sd_open()
{
	fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
		    if (fres == FR_OK)
		    {
		        myprintf("Opened 'write.txt' for writing successfully.\r\n");
		    } else
		    {
		        myprintf("f_open error (%i)\r\n", fres);
		        while (1); // Infinite loop on failure
		    }
	f_close(&fil);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  int ir_values = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_GPIO_Init();
  erlog_init(&log_console, &huart3);
  max30102_init(&max30102 , &hi2c2);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_SPI1_Init();
  MX_FATFS_Init();
  sd_init();
  /* USER CODE BEGIN 2 */

  read_register(&max30102, MAX30102_REVISIONID , &max30102.revision_id);
  read_register(&max30102, MAX30102_PARTID , &max30102.part_id);
  log_console.msg_len = sprintf((char *)log_console.msg,"MAX30102 Revision_id: %x, Part_id: %x\r\n", max30102.revision_id, max30102.part_id);
  erlog_write(&log_console);
  erlog_clear(&log_console);


  max30102_clear_fifo(&max30102);
  max30102_softReset(&max30102);
  max30102_set_fifoaverage(&max30102 , max30102_smp_ave_4);
  max30102_enableFIFORollover(&max30102);

  max30102_setpulsewidth(&max30102 , max30102_pw_18_bit);
  max30102_setadcrange(&max30102, max30102_adc_4096);
  max30102_setsamplerate(&max30102, max30102_sr_400);
  max30102_setledmode(&max30102 , max30102_led_irg);
  max30102_set_pulseamplitude(&max30102, 0x1F, RED_COLOUR);   // configure heartbeat sensor colours
  max30102_set_pulseamplitude(&max30102, 0x1F, GREEN_COLOUR);
  max30102_set_pulseamplitude(&max30102, 0x1F, IR);
  max30102_set_pulseamplitude(&max30102, 0x1F, PROXIMITY);

  max30102_enableSlot(&max30102 , 3, SLOT_GREEN_LED);
  max30102_set_pulseamplitude(&max30102, 0x0A, RED_COLOUR);   // configure heartbeat sensor colours
  max30102_set_pulseamplitude(&max30102, 0x00, GREEN_COLOUR);
  EXTI_Init(&max30102);
  max30102_enableDIETEMPRDY(&max30102);
  HAL_Delay(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE BEGIN 3 */
  while (1)
  {

	 /*measure temperature values*/
	 if(temperature == true)
	 {
		 temp = max30102_readtemp(&max30102);
		 log_console.msg_len= sprintf((char *)log_console.msg,"Temp :- %0.2f C \r\n", temp);
		 sd_write();
		 temperature = false;

	 }

	 erlog_write(&log_console);
	 HAL_Delay(100);
	 erlog_clear(&log_console);
     HAL_Delay(100);

     /*measure heartrate & spo2 values*/
	 ir_values = max30102_safeCheck(&max30102);
	 if(ir_values > 50000)
	 {
		 checkbeat(ir_values);
		 Spo2AvgInit(&max30102);
		 log_console.msg_len= sprintf((char *)log_console.msg,"Finger Detected , Heartbeat:- %f , spo2 - %f\r\n", beatsPerMinute, spo2);


	 }
	 else
	 {

		 HighPassFilter_reset(&high_pass_filter);
		 LowPassFilter_reset(&low_pass_filter);
		 beatsPerMinute = 0;
		 beatAvg = 0;
		 strcpy(log_console.msg , "No finger detected \r\n");
		 log_console.msg_len = strlen(log_console.msg);
	 }
	 erlog_write(&log_console);
	 HAL_Delay(100);
	 sd_write();
	 erlog_clear(&log_console);
  }
  /* USER CODE END 3 */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  while (1)
  {

	 /*measure temperature values*/
	 if(temperature == true)
	 {
		 temp = max30102_readtemp(&max30102);
	 	 log_console.msg_len= sprintf((char *)log_console.msg,"Temp :- %0.2f C \r\n", temp);
	     //sd_write();
	 	 temperature = false;
	 }
	 erlog_write(&log_console);
	 HAL_Delay(100);
	 erlog_clear(&log_console);
     HAL_Delay(100);

     /*measure heartrate & spo2 values*/
	 ir_values = max30102_safeCheck(&max30102);
	 if(ir_values > 50000)
	 {
		 checkbeat(ir_values);
		 Spo2AvgInit(&max30102);
		 log_console.msg_len= sprintf((char *)log_console.msg,"Finger Detected , body temp - %f, Heartbeat:- %f , spo2 - %f\r\n", temp, beatsPerMinute, spo2);


	 }
	 else
	 {

		 HighPassFilter_reset(&high_pass_filter);
		 LowPassFilter_reset(&low_pass_filter);
		 beatsPerMinute = 0;
		 beatAvg = 0;
		 strcpy(log_console.msg , "No finger detected \r\n");
		 log_console.msg_len = strlen(log_console.msg);
	 }
	 erlog_write(&log_console);
	 sd_write();
	 HAL_Delay(100);
	 erlog_clear(&log_console);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Interrupt on falling edge
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
