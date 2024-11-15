
#include "stm32f7xx_hal.h"
#include "max30102.h"
#include "erlog.h"

uint8_t dataBuffer[32];
uint8_t readPointer = 0;
uint8_t writePointer = 0;

extern float temp;

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
static void bitMask(max30102_t *obj, uint8_t reg, uint8_t mask, uint8_t thing)
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
void max3012_disableDIETEMPRDY(max30102_t *obj)
{
  bitMask(obj, MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_DISABLE);
}


void max30102_enableAFULL(max30102_t *obj) {
  bitMask(obj , MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_ENABLE);
}
void max30102_disableAFULL(max30102_t *obj) {
  bitMask(obj, MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_DISABLE);
}

void max30102_enableDATARDY(max30102_t *obj) {
  bitMask(obj , MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_ENABLE);
}
void max30102_disableDATARDY(max30102_t *obj) {
  bitMask(obj, MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_DISABLE);
}

void max30102_enableALCOVF(max30102_t *obj) {
  bitMask(obj, MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_ENABLE);
}
void max30102_disableALCOVF(max30102_t *obj) {
  bitMask(obj, MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_DISABLE);
}

void max30102_enablePROXINT(max30102_t *obj) {
  bitMask(obj, MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_ENABLE);
}
void max30102_disablePROXINT(max30102_t *obj) {
  bitMask(obj, MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_DISABLE);
}

void max30102_clear_fifo(max30102_t *obj)
{
    write_register(obj, MAX30102_FIFOWRITEPTR, 0);
    write_register(obj, MAX30102_FIFOREADPTR, 0);
    write_register(obj, MAX30102_FIFOOVERFLOW, 0);
}

void max30102_set_fifoaverage(max30102_t *obj, uint8_t numberOfSamples)
{
	bitMask(obj, MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples);
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

  return (float)tempInt + ((float)tempFrac * 0.0625); // Step 3: Calculate temperature (datasheet pg. 23)
}

void max30102_setledmode(max30102_t *obj, uint8_t mode)
{
  bitMask(obj, MAX30102_MODECONFIG, MAX30102_MODE_MASK, mode); // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  // See datasheet, page 19
}

void max30102_setadcrange(max30102_t *obj, uint8_t adcRange)
{
  bitMask(obj ,MAX30102_PARTICLECONFIG, MAX30102_ADCRANGE_MASK, adcRange); // adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
}

void max30102_setsamplerate(max30102_t *obj, uint8_t sampleRate)
{
  bitMask(obj, MAX30102_PARTICLECONFIG, MAX30102_SAMPLERATE_MASK, sampleRate); // sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
}

void max30102_setpulsewidth(max30102_t *obj, uint8_t pulseWidth)
{
  bitMask(obj, MAX30102_PARTICLECONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth); // pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
}
void max30102_enableFIFORollover(max30102_t *obj)
{
  bitMask(obj, MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}

void max30102_enableSlot(max30102_t *obj , uint8_t slotNumber, uint8_t device)
{

  switch (slotNumber)
  {
    case (1):
      bitMask(obj, MAX30102_MULTILEDCONFIG1, MAX30102_SLOT1_MASK, device);
      break;
    case (2):
      bitMask(obj, MAX30102_MULTILEDCONFIG1, MAX30102_SLOT2_MASK, device << 4);
      break;
    case (3):
      bitMask(obj, MAX30102_MULTILEDCONFIG2, MAX30102_SLOT3_MASK, device);
      break;
    case (4):
      bitMask(obj, MAX30102_MULTILEDCONFIG2, MAX30102_SLOT4_MASK, device << 4);
      break;
    default:
      //Shouldn't be here!
      break;
  }
}

void max30102_set_pulseamplitude(max30102_t *obj, uint8_t amplitude, max30102_colour_t colour_mode)
{
	uint8_t reg_val = 0;
	switch(colour_mode)
	{
		case RED_COLOUR:
		{
			reg_val = MAX30102_LED1_PULSEAMP;
			break;
		}
		case GREEN_COLOUR:
		{
			reg_val = MAX30102_LED3_PULSEAMP;
			break;
		}
		case IR:
		{
			reg_val = MAX30102_LED2_PULSEAMP;
			break;
		}
		case PROXIMITY:
		{
			reg_val = MAX30102_LED_PROX_AMP;
			break;
		}
	}
	write_register(obj , reg_val , amplitude);
}

void max30102_softReset(max30102_t *obj)
{
	uint8_t response = 0;
	bitMask(obj, MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET);
	do
	  {
		  read_register(obj, MAX30102_MODECONFIG, &response);
		  HAL_Delay(1);
	  }while((response & MAX30102_RESET) != 0);

}




uint32_t max30102_checksamples(max30102_t *obj)
{
	uint8_t command;
	int bytesLeftToRead = 0;
	int toGet = 0;
	int numberOfSamples = 0;

    read_register(obj, MAX30102_FIFOREADPTR , &readPointer);
    read_register(obj, MAX30102_FIFOWRITEPTR , &writePointer);

  // Check if there is new data
  if (readPointer != writePointer)
  {
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; // Wrap around condition

    bytesLeftToRead = numberOfSamples * max30102_led_irg * 3;
    command = MAX30102_FIFODATA;

    // Send the register address (FIFODATA) to read from
    HAL_I2C_Master_Transmit(obj->ui2c, MAX30102_I2C_ADDR << 1, &command, 1, HAL_MAX_DELAY);

    while (bytesLeftToRead > 0)
    {
      toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (max30102_led_irg * 3)); // Trim to multiple of samples
      }

      bytesLeftToRead -= toGet;
      HAL_I2C_Master_Receive(obj->ui2c, MAX30102_I2C_ADDR << 1, dataBuffer, toGet, HAL_MAX_DELAY);

      int index = 0;
      while (toGet > 0)
      {
        obj->_head++;
        obj->_head = (obj->_head + 1) % STORAGE_SIZE;  // Wrap condition

        uint8_t temp[sizeof(uint32_t)] = {0};
        uint32_t tempLong = 0;

        // Burst read three bytes for RED
        temp[2] = dataBuffer[index++];
        temp[1] = dataBuffer[index++];
        temp[0] = dataBuffer[index++];

        memcpy(&tempLong, temp, sizeof(tempLong));
        tempLong &= 0x3FFFF; // Zero out all but 18 bits
        obj->_red_samples[obj->_head] = tempLong;

        if (max30102_led_irg > 1)
        {
          // Burst read three bytes for IR
          temp[2] = dataBuffer[index++];
          temp[1] = dataBuffer[index++];
          temp[0] = dataBuffer[index++];
          memcpy(&tempLong, temp, sizeof(tempLong));
          tempLong &= 0x3FFFF;
          obj->_ir_samples[obj->_head] = tempLong;
        }

        if (max30102_led_irg > 2)
        {
          // Burst read three bytes for Green
          temp[2] = dataBuffer[index++];
          temp[1] = dataBuffer[index++];
          temp[0] = dataBuffer[index++];
          memcpy(&tempLong, temp, sizeof(tempLong));
          tempLong &= 0x3FFFF;
          obj->_green[obj->_head] = tempLong;
        }

        toGet -=max30102_led_irg * 3;
      }
    }
  }
  return numberOfSamples;

}

uint32_t max30102_safeCheck(max30102_t *obj)
{
	const uint8_t maxTimeToCheck = 250;
    uint32_t markTime = HAL_GetTick();  // Get the current system tick (in ms)

    while (1)
    {
        if (HAL_GetTick() - markTime > maxTimeToCheck)
        {
            return false;  // Timeout occurred
        }

        if (max30102_checksamples(obj) == true)  // Check if new data is available (assuming `check()` is implemented elsewhere)
        {
        	return obj->_ir_samples[obj->_head];  // New data found
        }
        HAL_Delay(1);  // Delay for 1 millisecond
    }
}

/*
 * Temperature Interrupts
 */
void EXTI_Init(max30102_t *obj)
{

    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 2);  // Set priority (lower number means higher priority)
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);          // Enable EXTI line 0 interrupt
}






