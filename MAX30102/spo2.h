/** ile spo2_algorithm.h
* Description: This module calculates the heart rate/SpO2 level for STM32 platform
*/

#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include "stm32f7xx_hal.h"  // Adjust based on your STM32 series
#include "filters.h"

typedef struct {
    float min;
    float max;
    float sum;
    int count;
} MinMaxAvgStatistic;


extern float kSpO2_A, kSpO2_B, kSpO2_C;

void Spo2AvgInit();
void Spo2AvgProcess(MinMaxAvgStatistic *stat, float value);
float maximum(MinMaxAvgStatistic *stat);
float minimum(MinMaxAvgStatistic *stat);
float average(MinMaxAvgStatistic *stat);


#endif /* ALGORITHM_H_ */
