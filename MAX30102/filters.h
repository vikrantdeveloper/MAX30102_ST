#ifndef FILTERS_H
#define FILTERS_H

#include <math.h>
#include <float.h>  // For using NAN

/**
 * @brief High Pass Filter Structure
 */
typedef struct {
    float kX;
    float kA0;
    float kA1;
    float kB1;
    float last_filter_value;
    float last_raw_value;
} HighPassFilter;

/**
 * @brief Initializes the High Pass Filter
 * @param filter Pointer to the HighPassFilter structure
 * @param samples Number of samples until decay to 36.8 %
 */
void HighPassFilter_init(HighPassFilter* filter, float samples);

/**
 * @brief Initializes the High Pass Filter with cutoff frequency
 * @param filter Pointer to the HighPassFilter structure
 * @param cutoff Cutoff frequency
 * @param sampling_frequency Sampling frequency
 */
void HighPassFilter_init_with_cutoff(HighPassFilter* filter, float cutoff, float sampling_frequency);

/**
 * @brief Applies the high pass filter
 * @param filter Pointer to the HighPassFilter structure
 * @param value Current input value
 * @return Filtered value
 */
float HighPassFilter_process(HighPassFilter* filter, float value);

/**
 * @brief Resets the stored values of the filter
 */
void HighPassFilter_reset(HighPassFilter* filter);


/**
 * @brief Low Pass Filter Structure
 */
typedef struct {
    float kX;
    float kA0;
    float kB1;
    float last_value;
} LowPassFilter;

/**
 * @brief Initializes the Low Pass Filter
 * @param filter Pointer to the LowPassFilter structure
 * @param samples Number of samples until decay to 36.8 %
 */
void LowPassFilter_init(LowPassFilter* filter, float samples);

/**
 * @brief Initializes the Low Pass Filter with cutoff frequency
 * @param filter Pointer to the LowPassFilter structure
 * @param cutoff Cutoff frequency
 * @param sampling_frequency Sampling frequency
 */
void LowPassFilter_init_with_cutoff(LowPassFilter* filter, float cutoff, float sampling_frequency);

/**
 * @brief Applies the low pass filter
 * @param filter Pointer to the LowPassFilter structure
 * @param value Current input value
 * @return Filtered value
 */
float LowPassFilter_process(LowPassFilter* filter, float value);

/**
 * @brief Resets the stored values of the filter
 */
void LowPassFilter_reset(LowPassFilter* filter);


/**
 * @brief Differentiator Structure
 */
typedef struct {
    float kSamplingFrequency;
    float last_value;
} Differentiator;

/**
 * @brief Initializes the differentiator
 * @param differentiator Pointer to the Differentiator structure
 * @param sampling_frequency Sampling frequency
 */
void Differentiator_init(Differentiator* differentiator, float sampling_frequency);

/**
 * @brief Applies the differentiator
 * @param differentiator Pointer to the Differentiator structure
 * @param value Current input value
 * @return Rate of change (differentiated value)
 */
float Differentiator_process(Differentiator* differentiator, float value);

/**
 * @brief Resets the stored values of the differentiator
 */
void Differentiator_reset(Differentiator* differentiator);


/**
 * @brief Moving Average Filter Structure
 * @tparam buffer_size Number of samples to average over
 */
#define kBufferSize 10  // Define buffer size (example)

typedef struct {
    int index;
    int count;
    float values[kBufferSize];
} MovingAverageFilter;

/**
 * @brief Initializes the Moving Average Filter
 * @param filter Pointer to the MovingAverageFilter structure
 */
void MovingAverageFilter_init(MovingAverageFilter* filter);

/**
 * @brief Applies the moving average filter
 * @param filter Pointer to the MovingAverageFilter structure
 * @param value Current input value
 * @return Filtered value
 */
float MovingAverageFilter_process(MovingAverageFilter* filter, float value);

/**
 * @brief Resets the stored values of the moving average filter
 */
void MovingAverageFilter_reset(MovingAverageFilter* filter);

/**
 * @brief Get number of samples
 * @param filter Pointer to the MovingAverageFilter structure
 * @return Number of stored samples
 */
int MovingAverageFilter_count(const MovingAverageFilter* filter);

extern LowPassFilter low_pass_filter;
extern HighPassFilter high_pass_filter;
extern Differentiator differentiator;

#endif /* FILTERS_H */
