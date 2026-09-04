#ifndef ADC_TRIGGER_H_
#define ADC_TRIGGER_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "trigger_engine.h"

struct adc_trigger_stats {
	uint32_t samples_acquired;
	uint32_t dma_buffers_processed;
	uint32_t queue_overruns;
	uint32_t saadc_errors;
	uint32_t dma_interval_min_us;
	uint32_t dma_interval_max_us;
	int16_t latest_raw;
	int16_t baseline;
	enum trigger_state trigger_state;
	bool trigger_armed;
};

int adc_trigger_init(struct trigger_engine *engine, struct k_msgq *hit_queue);
int adc_trigger_start(void);
void adc_trigger_get_stats(struct adc_trigger_stats *stats);
uint32_t adc_trigger_get_sample_rate_hz(void);
uint16_t adc_trigger_get_dma_buffer_samples(void);

#endif /* ADC_TRIGGER_H_ */
