#include "filters.h"

LowPassFilter low_pass_filter;
HighPassFilter high_pass_filter;
Differentiator differentiator;

/* High Pass Filter Functions */
void HighPassFilter_init(HighPassFilter* filter, float samples) {
    filter->kX = expf(-1.0f / samples);
    filter->kA0 = (1.0f + filter->kX) / 2.0f;
    filter->kA1 = -filter->kA0;
    filter->kB1 = filter->kX;
    filter->last_filter_value = 0.0;
    filter->last_raw_value = 0.0;
}

void HighPassFilter_init_with_cutoff(HighPassFilter* filter, float cutoff, float sampling_frequency) {
    HighPassFilter_init(filter, sampling_frequency / (cutoff * 2.0 * M_PI));
}

float HighPassFilter_process(HighPassFilter* filter, float value)
{
//    if (isnanf(filter->last_filter_value) || isnanf(filter->last_raw_value)) {
//        filter->last_filter_value = 0.0f;
//    } else {
        filter->last_filter_value = filter->kA0 * value + filter->kA1 * filter->last_raw_value + filter->kB1 * filter->last_filter_value;
    //}
    filter->last_raw_value = value;
    return filter->last_filter_value;
}

void HighPassFilter_reset(HighPassFilter* filter) {
    filter->last_raw_value = 0.0;
    filter->last_filter_value = 0.0;
}

/* Low Pass Filter Functions */
void LowPassFilter_init(LowPassFilter* filter, float samples) {
    filter->kX = expf(-1.0f / samples);
    filter->kA0 = 1.0f - filter->kX;
    filter->kB1 = filter->kX;
    filter->last_value = 0.0;
}

void LowPassFilter_init_with_cutoff(LowPassFilter* filter, float cutoff, float sampling_frequency) {
    LowPassFilter_init(filter, sampling_frequency / (cutoff * 2.0f * M_PI));
}

float LowPassFilter_process(LowPassFilter* filter, float value) {
    if (isnanf(filter->last_value)) {
        filter->last_value = value;
    } else {
        filter->last_value = filter->kA0 * value + filter->kB1 * filter->last_value;
    }
    return filter->last_value;
}

void LowPassFilter_reset(LowPassFilter* filter) {
    filter->last_value = 0.0;
}

/* Differentiator Functions */
void Differentiator_init(Differentiator* differentiator, float sampling_frequency) {
    differentiator->kSamplingFrequency = sampling_frequency;
    differentiator->last_value = 0.0;
}

float Differentiator_process(Differentiator* differentiator, float value) {
    float diff = (value - differentiator->last_value) * differentiator->kSamplingFrequency;
    differentiator->last_value = value;
    return diff;
}

void Differentiator_reset(Differentiator* differentiator) {
    differentiator->last_value = 0.0;
}

/* Moving Average Filter Functions */
void MovingAverageFilter_init(MovingAverageFilter* filter) {
    filter->index = 0;
    filter->count = 0;
    for (int i = 0; i < kBufferSize; i++) {
        filter->values[i] = 0.0f;
    }
}

float MovingAverageFilter_process(MovingAverageFilter* filter, float value) {
    filter->values[filter->index] = value;
    filter->index = (filter->index + 1) % kBufferSize;
    if (filter->count < kBufferSize) {
        filter->count++;
    }
    float sum = 0.0f;
    for (int i = 0; i < filter->count; i++) {
        sum += filter->values[i];
    }
    return sum / filter->count;
}

void MovingAverageFilter_reset(MovingAverageFilter* filter) {
    filter->index = 0;
    filter->count = 0;
}

int MovingAverageFilter_count(const MovingAverageFilter* filter) {
    return filter->count;
}
