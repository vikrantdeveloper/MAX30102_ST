

#include "erlog.h"

static void uart_error_handler()
{
	 __disable_irq();
	  while (1)
	  {
	  }
}
static void uart_init(UART_HandleTypeDef *huart3)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3->Instance = USART3;
  huart3->Init.BaudRate = 115200;
  huart3->Init.WordLength = UART_WORDLENGTH_8B;
  huart3->Init.StopBits = UART_STOPBITS_1;
  huart3->Init.Parity = UART_PARITY_NONE;
  huart3->Init.Mode = UART_MODE_TX_RX;
  huart3->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3->Init.OverSampling = UART_OVERSAMPLING_16;
  huart3->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(huart3) != HAL_OK)
  {
	  uart_error_handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void erlog_init(log_t *log_console, UART_HandleTypeDef *huart3)
{
	log_console->uhart = huart3;
	log_console->msg_len = 0;
	memset(log_console->msg, 0 , sizeof(log_console->msg_len));
	uart_init(log_console->uhart);
}

void erlog_write(log_t *log_console)
{
	HAL_UART_Transmit(log_console->uhart, (uint8_t *)log_console->msg, log_console->msg_len, HAL_MAX_DELAY);
}
void erlog_clear(log_t *log_console)
{
	memset(log_console->msg , 0, sizeof(log_console->msg_len));
}
