

#ifndef MAX_30102_H
#define MAX_30102_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32f7xx_hal.h"
#include "heartRate.h"
#include "filters.h"

/*
 * MODE CONFIGURATION MACROS
 */

#define STORAGE_SIZE 4
#define I2C_BUFFER_LENGTH 32
#define MAX30102_I2C_TIMEOUT 1000

#define MAX30102_BYTES_PER_SAMPLE 6
#define MAX30102_SAMPLE_LEN_MAX 32

#define MAX30102_INTERRUPT_STATUS_1 0x00
#define MAX30102_INTERRUPT_STATUS_2 0x01
#define MAX30102_INTERRUPT_ENABLE_1 0x02
#define MAX30102_INTERRUPT_ENABLE_2 0x03
#define MAX30102_INTERRUPT_A_FULL 7
#define MAX30102_INTERRUPT_PPG_RDY 6
#define MAX30102_INTERRUPT_ALC_OVF 5
#define MAX30102_INTERRUPT_DIE_TEMP_RDY 1

#define MAX30102_FIFO_DATA 0x07

#define MAX30102_FIFO_CONFIG 0x08
#define MAX30102_FIFO_CONFIG_SMP_AVE 5
#define MAX30102_FIFO_CONFIG_ROLL_OVER_EN 4
#define MAX30102_FIFO_CONFIG_FIFO_A_FULL 0

#define MAX30102_MODE_CONFIG 0x09
#define MAX30102_MODE_SHDN 7
#define MAX30102_MODE_RESET 6
#define MAX30102_MODE_MODE 0

#define MAX30102_SPO2_CONFIG 0x0a
#define MAX30102_SPO2_ADC_RGE 5
#define MAX30102_SPO2_SR 2
#define MAX30102_SPO2_LEW_PW 0

#define MAX30102_LED_IR_PA1 0x0c
#define MAX30102_LED_RED_PA2 0x0d

#define MAX30102_MULTI_LED_CTRL_1 0x11
#define MAX30102_MULTI_LED_CTRL_SLOT2 4
#define MAX30102_MULTI_LED_CTRL_SLOT1 0
#define MAX30102_MULTI_LED_CTRL_2 0x12
#define MAX30102_MULTI_LED_CTRL_SLOT4 4
#define MAX30102_MULTI_LED_CTRL_SLOT3 0

#define MAX30102_DIE_TINT 0x1f
#define MAX30102_DIE_TFRAC 0x20
#define MAX30102_DIE_TFRAC_INCREMENT 0.0625f
#define MAX30102_DIE_TEMP_CONFIG 0x21
#define MAX30102_DIE_TEMP_EN 1


/*
 * REGISTER MAP
 */

#define MAX30102_I2C_ADDR 0x57  //I2C address

//Status Registers
#define MAX30102_INTSTAT1        0x00//Interrupt Status1
#define MAX30102_INTSTAT2        0x01//Interrupt Status2
#define MAX30102_INTENABLE1      0x02//Interrupt Enable1
#define MAX30102_INTENABLE2      0x03//Interrupt Enable2

//FIFO Registers
#define MAX30102_FIFOWRITEPTR    0x04//FIFO Write Pointer, The FIFO Write Pointer points to the location where the MAX30102 writes the next sample. This pointer advances for each sample pushed on to the FIFO. It can also be changed through the I2C interface when MODE[2:0] is 010, 011, or 111.
#define MAX30102_FIFOOVERFLOW    0x05//FIFO Overflow Counter, When the FIFO is full, samples are not pushed on to the FIFO, samples are lost. OVF_COUNTER counts the number of samples lost. It saturates at 0x1F. When a complete sample is “popped” (i.e., removal of old FIFO data and shifting the samples down) from the FIFO (when the read pointer advances), OVF_COUNTER is reset to zero.
#define MAX30102_FIFOREADPTR     0x06//FIFO Read Pointer, The FIFO Read Pointer points to the location from where the processor gets the next sample from the FIFO through the I2C interface. This advances each time a sample is popped from the FIFO. The processor can also write to this pointer after reading the samples to allow rereading samples from the FIFO if there is a data communication error.
#define MAX30102_FIFODATA        0x07//FIFO Data Register, The circular FIFO depth is 32 and can hold up to 32 samples of data. The sample size depends on the number of LED channels (a.k.a. channels) configured as active. As each channel signal is stored as a 3-byte data signal, the FIFO width can be 3 bytes or 6 bytes in size.

//Configuration Registers
#define MAX30102_FIFOCONFIG      0x08//FIFO Configuration
#define MAX30102_MODECONFIG      0x09//Mode Configuration
#define MAX30102_PARTICLECONFIG  0x0A//SpO2 Configuration
#define MAX30102_LED1_PULSEAMP   0x0C//LED1 Pulse Amplitude
#define MAX30102_LED2_PULSEAMP   0x0D//LED2 Pulse Amplitude
#define MAX30102_LED3_PULSEAMP   0x0E//RESERVED
#define MAX30102_LED_PROX_AMP    0x10//RESERVED
#define MAX30102_MULTILEDCONFIG1 0x11//Multi-LED Mode Control Registers
#define MAX30102_MULTILEDCONFIG2 0x12//Multi-LED Mode Control Registers

//Die Temperature Registers
#define MAX30102_DIETEMPINT      0x1F//Die Temp Integer
#define MAX30102_DIETEMPFRAC     0x20//Die Temp Fraction
#define MAX30102_DIETEMPCONFIG   0x21//Die Temperature Config
#define MAX30102_PROXINTTHRESH   0x30//RESERVED
//Part ID Registers
#define MAX30102_REVISIONID      0xFE//Revision ID
#define MAX30102_PARTID          0xFF//Part ID:0x15
#define MAX30102_EXPECTED_PARTID  0x15

#define MAX30102_MODE_MASK 			0xF8
#define MAX30102_ADCRANGE_MASK 		0x9F
#define MAX30102_SAMPLERATE_MASK 	0xE3
#define MAX30102_PULSEWIDTH_MASK 	0xFC

static const uint8_t MAX30102_RESET_MASK = 		0xBF;
static const uint8_t MAX30102_RESET = 			0x40;

static const uint8_t MAX30102_INT_DIE_TEMP_RDY_MASK = (uint8_t)~0b00000010;

static const uint8_t MAX30102_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30102_INT_DIE_TEMP_RDY_DISABLE = 0x00;


static const uint8_t MAX30102_INT_A_FULL_MASK =		(uint8_t)~0b10000000;

static const uint8_t MAX30102_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX30102_INT_A_FULL_DISABLE = 	0x00;

static const uint8_t MAX30102_INT_DATA_RDY_MASK = (uint8_t)~0b01000000;

static const uint8_t MAX30102_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX30102_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX30102_INT_ALC_OVF_MASK = (uint8_t)~0b00100000;

static const uint8_t MAX30102_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX30102_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX30102_INT_PROX_INT_MASK = (uint8_t)~0b00010000;

static const uint8_t MAX30102_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX30102_INT_PROX_INT_DISABLE = 0x00;


static const uint8_t MAX30102_SAMPLEAVG_MASK =	(uint8_t)~0b11100000;

static const uint8_t MAX30102_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX30102_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX30102_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX30102_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX30102_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX30102_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX30102_ROLLOVER_MASK = 	0xEF;

static const uint8_t MAX30102_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30102_ROLLOVER_DISABLE = 0x00;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30102_SLOT1_MASK = 		0xF8;
static const uint8_t MAX30102_SLOT2_MASK = 		0x8F;
static const uint8_t MAX30102_SLOT3_MASK = 		0xF8;
static const uint8_t MAX30102_SLOT4_MASK = 		0x8F;
static const uint8_t SLOT_NONE = 				0x00;
static const uint8_t SLOT_RED_LED = 			0x01;
static const uint8_t SLOT_IR_LED = 				0x02;
static const uint8_t SLOT_GREEN_LED = 			0x03;
static const uint8_t SLOT_NONE_PILOT = 			0x04;
static const uint8_t SLOT_RED_PILOT =			0x05;
static const uint8_t SLOT_IR_PILOT = 			0x06;
static const uint8_t SLOT_GREEN_PILOT = 		0x07;



/*
 * HANDLERS STRUCTURES
 */

typedef enum max30102_mode_t
{
    max30102_heart_rate = 0x02,
    max30102_spo2 = 0x03,
    max30102_multi_led = 0x07
} max30102_mode_t;

typedef enum max30102_smp_ave_t
{
    max30102_smp_ave_1 = 0x00,
    max30102_smp_ave_2 = 0x20,
    max30102_smp_ave_4 = 0x40,
    max30102_smp_ave_8 = 0x60,
    max30102_smp_ave_16 = 0x80,
    max30102_smp_ave_32 = 0xA0
} max30102_smp_ave_t;

typedef enum max30102_sr_t
{
    max30102_sr_50 = 0x00,
    max30102_sr_100 = 0x04,
    max30102_sr_200 = 0x08,
    max30102_sr_400 = 0x0C,
    max30102_sr_800 = 0x10,
    max30102_sr_1000 = 0x14,
    max30102_sr_1600 = 0x18,
    max30102_sr_3200 = 0x1C
} max30102_sr_t;

typedef enum max30102_led_pw_t
{
    max30102_pw_15_bit = 0x00,
    max30102_pw_16_bit = 0x01,
    max30102_pw_17_bit = 0x02,
    max30102_pw_18_bit = 0x03
} max30102_led_pw_t;

typedef enum max30102_adc_t
{
    max30102_adc_2048 = 0x00,
    max30102_adc_4096 = 0x20,
    max30102_adc_8192 = 0x40,
    max30102_adc_16384 = 0x60
} max30102_adc_t;

typedef enum max30102_multi_led_ctrl_t
{
    max30102_led_off = 0,
    max30102_led_red = 1,
    max30102_led_ir =  2,
	max30102_led_irg= 3
} max30102_multi_led_ctrl_t;

typedef enum max30102_colour_t
{
	RED_COLOUR = 1,
	GREEN_COLOUR = 2,
	IR = 3,
	PROXIMITY = 4
}max30102_colour_t;

typedef struct max30102_t
{
    I2C_HandleTypeDef *ui2c;
    uint32_t _ir_samples[32];
    uint32_t _red_samples[32];
    uint32_t _green[32];
    uint16_t _head;
    volatile uint8_t intr_flag;
    uint8_t part_id;
    uint8_t revision_id;
} max30102_t;

/*
* FUNCTIONS
*/
//Initialisation
void max30102_init(max30102_t *obj, I2C_HandleTypeDef *i2chandler);

// Low Level i2c communication
bool read_register(max30102_t *obj, uint8_t addr, uint8_t *value);
bool write_register(max30102_t *obj, uint8_t addr, uint8_t value);

// Interrupts //
void max30102_enableDIETEMPRDY(max30102_t *obj);
void max30102_enableAFULL(max30102_t *obj);
void max30102_disableAFULL(max30102_t *obj);
void max30102_enableDATARDY(max30102_t *obj);
void max30102_disableDATARDY(max30102_t *obj);
void max30102_enableALCOVF(max30102_t *obj);
void max30102_disableALCOVF(max30102_t *obj);
void max30102_enablePROXINT(max30102_t *obj);
void max30102_disablePROXINT(max30102_t *obj);



float max30102_readtemp(max30102_t *obj);

void max30102_set_pulseamplitude(max30102_t *obj, uint8_t amplitude, max30102_colour_t colour_mode);


void max30102_set_fifoaverage(max30102_t *obj, uint8_t numberOfSamples);
void max30102_clear_fifo(max30102_t *obj);
void max30102_setledmode(max30102_t *obj, uint8_t mode);
void max30102_setadcrange(max30102_t *obj, uint8_t adcRange);
void max30102_setsamplerate(max30102_t *obj, uint8_t sampleRate);
void max30102_setpulsewidth(max30102_t *obj, uint8_t pulseWidth);
void max30102_enableSlot(max30102_t *obj , uint8_t slotNumber, uint8_t device);
void max30102_enableFIFORollover(max30102_t *obj);

uint32_t max30102_checksamples(max30102_t *obj);
uint32_t max30102_safeCheck(max30102_t *obj);
void max30102_softReset(max30102_t *obj);




#endif
