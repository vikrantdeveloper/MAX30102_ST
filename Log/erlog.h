#ifndef __ERLOG__
#define __ERLOG__

#include "stm32f7xx_hal.h"
#include <string.h>


typedef struct{
	UART_HandleTypeDef  *uhart;
	uint16_t msg_len;
	char msg[256];
}log_t;

void erlog_init(log_t *log_console, UART_HandleTypeDef *huart3);

void erlog_write(log_t *log_console);

#endif
