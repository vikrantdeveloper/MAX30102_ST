/** ile spo2_algorithm.cpp
* Description: SpO2 and Heart Rate Calculation Algorithm for STM32
*/

#include "spo2.h"
#include "stm32f7xx_hal.h"  // HAL for STM32

#define BUFFER_SIZE 100

static int32_t an_x[BUFFER_SIZE]; // IR
static int32_t an_y[BUFFER_SIZE]; // Red

void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid) {
    // Example calculation logic
    int32_t n_ir_mean = 0, n_red_mean = 0;

    // Calculate mean of sensor data (simplified logic)
    for (int i = 0; i < n_ir_buffer_length; i++) {
        n_ir_mean += pun_ir_buffer[i];
        n_red_mean += pun_red_buffer[i];
    }

    n_ir_mean /= n_ir_buffer_length;
    n_red_mean /= n_ir_buffer_length;

    // Process heart rate and SpO2 (simplified placeholder)
    *pn_heart_rate = (n_ir_mean + n_red_mean) / 2;
    *pn_spo2 = 95; // Dummy constant SpO2 value

    *pch_hr_valid = 1;
    *pch_spo2_valid = 1;
}

void maxim_find_peaks(int32_t *pn_locs, int32_t *n_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num)
{
	maxim_peaks_above_min_height( pn_locs, n_npks, pn_x, n_size, n_min_height );
	maxim_remove_close_peaks( pn_locs, n_npks, pn_x, n_min_distance );
	*n_npks = min( *n_npks, n_max_num );
}

void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *n_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height)
{
	int32_t i = 1, n_width;
	  *n_npks = 0;

	  while (i < n_size-1){
	    if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]){      // find left edge of potential peaks
	      n_width = 1;
	      while (i+n_width < n_size && pn_x[i] == pn_x[i+n_width])  // find flat peaks
	        n_width++;
	      if (pn_x[i] > pn_x[i+n_width] && (*n_npks) < 15 ){      // find right edge of peaks
	        pn_locs[(*n_npks)++] = i;
	        // for flat peaks, peak location is left edge
	        i += n_width+1;
	      }
	      else
	        i += n_width;
	    }
	    else
	      i++;
	  }
}

void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
{
	  int32_t i, j, n_old_npks, n_dist;

	  /* Order peaks from large to small */
	  maxim_sort_indices_descend( pn_x, pn_locs, *pn_npks );

	  for ( i = -1; i < *pn_npks; i++ ){
	    n_old_npks = *pn_npks;
	    *pn_npks = i+1;
	    for ( j = i+1; j < n_old_npks; j++ ){
	      n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
	      if ( n_dist > n_min_distance || n_dist < -n_min_distance )
	        pn_locs[(*pn_npks)++] = pn_locs[j];
	    }
	  }

	  // Resort indices int32_to ascending order
	  maxim_sort_ascend( pn_locs, *pn_npks );
}

void maxim_sort_ascend(int32_t *pn_x, int32_t n_size)
{
	int32_t i, j, n_temp;
	  for (i = 1; i < n_size; i++) {
	    n_temp = pn_x[i];
	    for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
	        pn_x[j] = pn_x[j-1];
	    pn_x[j] = n_temp;
	  }
}

void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size)
{
	int32_t i, j, n_temp;
	  for (i = 1; i < n_size; i++) {
	    n_temp = pn_indx[i];
	    for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j-1]]; j--)
	      pn_indx[j] = pn_indx[j-1];
	    pn_indx[j] = n_temp;
	  }
}
