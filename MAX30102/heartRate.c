#include "max30102.h"
#include "stm32f7xx_hal.h"  // Adjust based on your STM32 model

#include "filters.h"

uint32_t lastBeat = 0;  // Time of the last detected beat in milliseconds
uint32_t delta = 0;     // Time difference between beats
uint8_t rateSpot = 0;   // Index for storing BPM readings
uint8_t rates[RATE_SIZE];  // Array to store BPM values


void processHeartBeat(float current_diff)
{
	const float kEdgeThreshold = -1000.0;
    uint32_t currentMillis = HAL_GetTick();  // Get current time in milliseconds
    delta = currentMillis - lastBeat;  // Calculate the time difference between current and last beat
    lastBeat = currentMillis;          // Update the time of the last beat

    beatsPerMinute = (60.0 / (delta / 1000.0)) + 25.0;  // Calculate beats per minute

    // Filter valid BPM values
    if (beatsPerMinute < 255 && beatsPerMinute > 20 && current_diff < kEdgeThreshold)
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
    }
}

bool checkbeat(int32_t sample)
{
	// Averaging
	const float kSamplingFrequency = 400.0;

	// Filters
	float current_value = sample;
	const float kLowPassCutoff = 5.0;
	const float kHighPassCutoff = 0.5;

	LowPassFilter_init_with_cutoff(&low_pass_filter, kLowPassCutoff, kSamplingFrequency);
	HighPassFilter_init_with_cutoff(&high_pass_filter, kHighPassCutoff, kSamplingFrequency);
	Differentiator_init(&differentiator, kSamplingFrequency);  // Pass sampling frequency
	current_value = LowPassFilter_process(&low_pass_filter, current_value);
	current_value = HighPassFilter_process(&high_pass_filter, current_value);
	float current_diff = Differentiator_process(&differentiator, current_value);

	if (current_diff > 0.0)
	{
	    	processHeartBeat(current_diff);
	    	return true;
	 }
   else
   {
	    	return false;
	}


}
