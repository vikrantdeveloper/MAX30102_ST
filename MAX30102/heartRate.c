#include "max30102.h"
#include "stm32f7xx_hal.h"  // Adjust based on your STM32 model



uint32_t lastBeat = 0;  // Time of the last detected beat in milliseconds
uint32_t delta = 0;     // Time difference between beats
uint8_t rateSpot = 0;   // Index for storing BPM readings
uint8_t rates[RATE_SIZE];  // Array to store BPM values
float beatsPerMinute = 0;
float beatAvg = 0;

// Variables from the original algorithm
int16_t IR_AC_Max = 20;
int16_t IR_AC_Min = -20;
int16_t IR_AC_Signal_Current = 0;
int16_t IR_AC_Signal_Previous;
int16_t IR_AC_Signal_min = 0;
int16_t IR_AC_Signal_max = 0;
int16_t IR_Average_Estimated;

int16_t positiveEdge = 0;
int16_t negativeEdge = 0;
int32_t ir_avg_reg = 0;

int16_t cbuf[32];
uint8_t offset = 0;

static const uint16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};

//  Heart Rate Monitor functions
bool checkForBeat(int32_t sample)
{
    bool beatDetected = false;

    IR_AC_Signal_Previous = IR_AC_Signal_Current;
    IR_Average_Estimated = averageDCEstimator(&ir_avg_reg, sample);
    IR_AC_Signal_Current = lowPassFIRFilter(sample - IR_Average_Estimated);

    if ((IR_AC_Signal_Previous < 0) && (IR_AC_Signal_Current >= 0)) {
        IR_AC_Max = IR_AC_Signal_max;
        IR_AC_Min = IR_AC_Signal_min;

        positiveEdge = 1;
        negativeEdge = 0;
        IR_AC_Signal_max = 0;

        if ((IR_AC_Max - IR_AC_Min) > 20 && (IR_AC_Max - IR_AC_Min) < 1000) {
            beatDetected = true;
        }
    }

    if ((IR_AC_Signal_Previous > 0) && (IR_AC_Signal_Current <= 0)) {
        positiveEdge = 0;
        negativeEdge = 1;
        IR_AC_Signal_min = 0;
    }

    if (positiveEdge && (IR_AC_Signal_Current > IR_AC_Signal_Previous)) {
        IR_AC_Signal_max = IR_AC_Signal_Current;
    }

    if (negativeEdge && (IR_AC_Signal_Current < IR_AC_Signal_Previous)) {
        IR_AC_Signal_min = IR_AC_Signal_Current;
    }

    return beatDetected;
}

// Average DC Estimator
int16_t averageDCEstimator(int32_t *p, uint16_t x)
{
    *p += ((((long) x << 15) - *p) >> 4);
    return (*p >> 15);
}

// Low Pass FIR Filter
int16_t lowPassFIRFilter(int16_t din)
{
    cbuf[offset] = din;

    int32_t z = mul16(FIRCoeffs[11], cbuf[(offset - 11) & 0x1F]);

    for (uint8_t i = 0; i < 11; i++) {
        z += mul16(FIRCoeffs[i], cbuf[(offset - i) & 0x1F] + cbuf[(offset - 22 + i) & 0x1F]);
    }

    offset++;
    offset %= 32;

    return (z >> 15);
}

// Integer multiplier
int32_t mul16(int16_t x, int16_t y)
{
    return ((long) x * (long) y);
}

void processHeartBeat(max30102_t *obj)
{
    uint32_t currentMillis = HAL_GetTick();  // Get current time in milliseconds
    delta = currentMillis - lastBeat;  // Calculate the time difference between current and last beat
    lastBeat = currentMillis;          // Update the time of the last beat

    beatsPerMinute = (60.0 / (delta / 1000.0)) - 100.0;  // Calculate beats per minute

    // Filter valid BPM values
    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
        rates[rateSpot++] = (uint8_t)beatsPerMinute;  // Store BPM in the array
        rateSpot %= RATE_SIZE;  // Wrap the array index if it exceeds RATE_SIZE

        // Calculate the average BPM
        beatAvg = 0;
        for (uint8_t x = 0; x < RATE_SIZE; x++)
        {
            beatAvg += rates[x];
        }
        beatAvg /= RATE_SIZE;  // Compute the average BPM
        spo2Avg = (obj->_ir_samples[0] + obj->_red_samples[0]) / RATE_SIZE;
    }
}
