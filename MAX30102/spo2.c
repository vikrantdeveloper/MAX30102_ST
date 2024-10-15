/** ile spo2_algorithm.cpp
* Description: SpO2 and Heart Rate Calculation Algorithm for STM32
*/

#include "spo2.h"
#include "max30102.h"
#include "stm32f7xx_hal.h"  // HAL for STM32



float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

extern float spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;


void initStatistic(MinMaxAvgStatistic *stat)
{
    stat->min = 0.0;
    stat->max = 0.0;
    stat->sum = 0.0;
    stat->count = 0;
}

void Spo2AvgProcess(MinMaxAvgStatistic *stat, float value)
{
    if (isnan(stat->min))
    {
        stat->min = value;
        stat->max = value;
    }
    else
    {
        stat->min = fminf(stat->min, value);
        stat->max = fmaxf(stat->max, value);
    }
    stat->sum += value;
    stat->count++;
}


float average(MinMaxAvgStatistic *stat)
{
    return (stat->count > 0) ? stat->sum / stat->count : 0.0;
}


float minimum(MinMaxAvgStatistic *stat)
{
    return stat->min;
}

float maximum(MinMaxAvgStatistic *stat)
{
    return stat->max;
}


void initMinMaxAvgStatistic(MinMaxAvgStatistic *stat) {
    stat->min = 0.0;
    stat->max = 0.0;
    stat->sum = 0.0;
    stat->count = 0;
}

void Spo2AvgInit(max30102_t *obj)
{
	float rred , rir, r = 0;
	float sample[4];
	initMinMaxAvgStatistic(&stat_red);
	initMinMaxAvgStatistic(&stat_ir);

	sample[0] = LowPassFilter_process(&low_pass_filter, obj->_red_samples[0]);
	sample[1] = LowPassFilter_process(&low_pass_filter, obj->_red_samples[2]);
	sample[2] = LowPassFilter_process(&low_pass_filter, obj->_ir_samples[0]);
	sample[3] = LowPassFilter_process(&low_pass_filter, obj->_ir_samples[2]);


	Spo2AvgProcess(&stat_red, sample[0]);
	Spo2AvgProcess(&stat_red, sample[1]);
	Spo2AvgProcess(&stat_ir, sample[3]);
	Spo2AvgProcess(&stat_ir, sample[4]);

	rred = (maximum(&stat_red) - minimum(&stat_red)) / average(&stat_red);
	rir = (maximum(&stat_ir) - minimum(&stat_ir)) / average(&stat_ir);
	r = rred / rir;
	spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
}


