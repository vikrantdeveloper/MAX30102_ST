
#ifndef HEARTRATE_H
#define HEARTRATE_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "max30102.h"

#define RATE_SIZE 4  // Adjust the size as per your needs

bool checkForBeat(int32_t sample);
int16_t averageDCEstimator(int32_t *p, uint16_t x);
int16_t lowPassFIRFilter(int16_t din);
int32_t mul16(int16_t x, int16_t y);

void processHeartBeat(max30102_t *obj);

extern float beatAvg;
extern float beatsPerMinute;
extern float spo2Avg = 0;

#endif
