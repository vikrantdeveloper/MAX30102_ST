
#include "stm32f7xx_hal.h"
#include "max30102.h"
#include "erlog.h"



static void i2c_error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
static void i2c_init(I2C_HandleTypeDef *hi2c1)
{

  /*PF0 & PF1 pins configured*/

  /* USER CODE BEGIN I2C1_Init 1 */
  hi2c1->Instance = I2C2;
  hi2c1->Init.Timing = 400000;
  hi2c1->Init.OwnAddress1 = 0;
  hi2c1->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1->Init.OwnAddress2 = 0;
  hi2c1->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(hi2c1) != HAL_OK)
  {
	  i2c_error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
	  i2c_error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(hi2c1, 0) != HAL_OK)
  {
	  i2c_error_Handler();
  }
  /* USER CODE END I2C1_Init 1 */

}
void max30102_init(max30102_t *obj, I2C_HandleTypeDef *i2chandler)
{
	 obj->ui2c = i2chandler;
	 obj->intr_flag = 0;
	 memset(obj->_ir_samples, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));
	 memset(obj->_red_samples, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));
	 i2c_init(obj->ui2c);  /*initialise the i2c peripheral for MAX3212*/

}
bool read_register(max30102_t *obj, uint8_t addr, uint8_t *value)
{
	uint8_t rx_buff;
	uint8_t rx_address;
	rx_address = (MAX30102_I2C_ADDR << 1);
	if (HAL_I2C_Mem_Read(obj->ui2c, rx_address , addr, 1, &rx_buff, 1, 5000) == HAL_OK)
	{
		*value = rx_buff;
		return true;
	}
	else
		return false;

}
bool write_register(max30102_t *obj, uint8_t addr, uint8_t value)
{
	uint8_t tx_address;
	tx_address = (MAX30102_I2C_ADDR << 1);
	if (HAL_I2C_Mem_Write(obj->ui2c, tx_address, addr, 1, &value, 1, 10000) == HAL_OK)
		return false;
	else
		return true;
}
void bitMask(max30102_t *obj, uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = 0;

  read_register(obj, reg, &originalContents);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  write_register(obj, reg, originalContents | thing);
}

void max30102_enableDIETEMPRDY(max30102_t *obj)
{
  bitMask(obj, MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_ENABLE);
}

float max30102_readtemp(max30102_t *obj)
{

  uint8_t response = 0;
  uint8_t tempInt = 0;
  uint8_t tempFrac = 0;

  write_register(obj, MAX30102_DIETEMPCONFIG, 0x01);

  do
  {
	  read_register(obj, MAX30102_INTSTAT2, &response);
	  HAL_Delay(1);
  }while((response & MAX30102_INTENABLE1) == 0);

  read_register(obj, MAX30102_DIETEMPINT, &tempInt);
  read_register(obj, MAX30102_DIETEMPFRAC, &tempFrac);

  // Step 3: Calculate temperature (datasheet pg. 23)
  return (float)tempInt + ((float)tempFrac * 0.0625);
}


