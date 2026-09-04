#include "adc_trigger.h"

#include <errno.h>
#include <zephyr/sys/util.h>

#if CONFIG_DRUM_SAADC_STREAM

#include <limits.h>

#include <nrfx_saadc.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/dt-bindings/adc/nrf-saadc.h>
#include <zephyr/sys/atomic.h>

#define ADC_DEVICE_NODE  DT_NODELABEL(adc)
#define ADC_CHANNEL_NODE DT_CHILD(ADC_DEVICE_NODE, channel_0)

#define ADC_SAMPLE_PERIOD_US      ((uint32_t)CONFIG_DRUM_ADC_SAMPLE_PERIOD_US)
#define ADC_SAMPLE_RATE_HZ        (1000000U / ADC_SAMPLE_PERIOD_US)
#define ADC_DMA_BUFFER_SAMPLES    8U
#define SAADC_TIMER_FREQUENCY_MHZ 16U
#define SAADC_TIMER_CC            ((ADC_SAMPLE_PERIOD_US * SAADC_TIMER_FREQUENCY_MHZ) - 1U)

BUILD_ASSERT(DT_NODE_HAS_STATUS(ADC_DEVICE_NODE, okay), "SAADC must be enabled");
BUILD_ASSERT(DT_NODE_EXISTS(ADC_CHANNEL_NODE), "SAADC channel@0 must exist");
BUILD_ASSERT(DT_PROP(ADC_CHANNEL_NODE, zephyr_input_positive) == NRF_SAADC_AIN0,
	     "kick input must remain on AIN0 / P0.02");
BUILD_ASSERT(DT_STRING_TOKEN(ADC_CHANNEL_NODE, zephyr_gain) == ADC_GAIN_1_6,
	     "kick SAADC gain must remain 1/6");
BUILD_ASSERT(DT_STRING_TOKEN(ADC_CHANNEL_NODE, zephyr_reference) == ADC_REF_INTERNAL,
	     "kick SAADC must use the internal reference");
BUILD_ASSERT(ADC_ACQ_TIME_UNIT(DT_PROP(ADC_CHANNEL_NODE, zephyr_acquisition_time)) ==
		     ADC_ACQ_TIME_MICROSECONDS,
	     "kick SAADC acquisition time must be in microseconds");
BUILD_ASSERT(ADC_ACQ_TIME_VALUE(DT_PROP(ADC_CHANNEL_NODE, zephyr_acquisition_time)) == 20U,
	     "kick SAADC acquisition time must remain 20 us");
BUILD_ASSERT(DT_PROP(ADC_CHANNEL_NODE, zephyr_resolution) == 12U,
	     "kick SAADC resolution must remain 12-bit");
BUILD_ASSERT(DT_PROP(ADC_CHANNEL_NODE, zephyr_oversampling) == 0U,
	     "continuous kick acquisition currently requires no oversampling");
BUILD_ASSERT((1000000U % ADC_SAMPLE_PERIOD_US) == 0U,
	     "sample period must produce an integer sample rate");
BUILD_ASSERT(ADC_SAMPLE_PERIOD_US <= NRFX_SAADC_INTERNAL_TIMER_INTERVAL_MAX_US,
	     "sample period exceeds the SAADC internal timer range");
BUILD_ASSERT(SAADC_TIMER_CC <= UINT16_MAX, "SAADC internal timer compare does not fit");

struct adc_runtime {
	struct trigger_engine *engine;
	struct k_msgq *hit_queue;
	atomic_t samples_acquired;
	atomic_t dma_buffers_processed;
	atomic_t queue_overruns;
	atomic_t saadc_errors;
	atomic_t dma_interval_min_us;
	atomic_t dma_interval_max_us;
	atomic_t latest_raw;
	atomic_t baseline;
	atomic_t trigger_state;
	atomic_t trigger_armed;
	uint32_t previous_done_cycle;
	uint8_t next_buffer_index;
	bool have_previous_done_cycle;
	bool initialized;
	bool started;
};

static struct adc_runtime runtime;
static nrf_saadc_value_t sample_buffers[2][ADC_DMA_BUFFER_SAMPLES];

static uint32_t uptime_us_32(void)
{
	return (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
}

static void atomic_update_min(atomic_t *target, uint32_t value)
{
	atomic_val_t previous = atomic_get(target);

	while ((value < (uint32_t)previous) && !atomic_cas(target, previous, (atomic_val_t)value)) {
		previous = atomic_get(target);
	}
}

static void atomic_update_max(atomic_t *target, uint32_t value)
{
	atomic_val_t previous = atomic_get(target);

	while ((value > (uint32_t)previous) && !atomic_cas(target, previous, (atomic_val_t)value)) {
		previous = atomic_get(target);
	}
}

static void record_dma_timing(void)
{
#if CONFIG_DRUM_ADC_TIMING_DIAGNOSTICS
	uint32_t now_cycle = k_cycle_get_32();

	if (runtime.have_previous_done_cycle) {
		uint32_t interval_us = k_cyc_to_us_floor32(now_cycle - runtime.previous_done_cycle);

		atomic_update_min(&runtime.dma_interval_min_us, interval_us);
		atomic_update_max(&runtime.dma_interval_max_us, interval_us);
	}

	runtime.previous_done_cycle = now_cycle;
	runtime.have_previous_done_cycle = true;
#endif
}

static void process_completed_buffer(const nrfx_saadc_done_evt_t *done)
{
	uint32_t sample_timestamp_us =
		uptime_us_32() - (((uint32_t)done->size - 1U) * ADC_SAMPLE_PERIOD_US);

	record_dma_timing();

	for (uint16_t i = 0U; i < done->size; ++i) {
		struct drum_hit_event event;
		int16_t raw = (int16_t)NRFX_SAADC_SAMPLE_GET(done->p_buffer, i);

		/* Match Zephyr's single-ended ADC behavior for rare sub-ground noise. */
		if (raw < 0) {
			raw = 0;
		}

		if (trigger_engine_process_sample(runtime.engine, raw, sample_timestamp_us,
						  &event)) {
			if (k_msgq_put(runtime.hit_queue, &event, K_NO_WAIT) != 0) {
				atomic_inc(&runtime.queue_overruns);
			}
		}

		sample_timestamp_us += ADC_SAMPLE_PERIOD_US;
		atomic_set(&runtime.latest_raw, raw);
	}

	atomic_add(&runtime.samples_acquired, done->size);
	atomic_inc(&runtime.dma_buffers_processed);
	atomic_set(&runtime.baseline, trigger_engine_get_baseline(runtime.engine));
	atomic_set(&runtime.trigger_state, trigger_engine_get_state(runtime.engine));
	atomic_set(&runtime.trigger_armed, trigger_engine_is_armed(runtime.engine));
}

static void saadc_event_handler(const nrfx_saadc_evt_t *event)
{
	int err;

	switch (event->type) {
	case NRFX_SAADC_EVT_READY:
		/* Sampling is now paced entirely by the SAADC internal timer. */
		break;

	case NRFX_SAADC_EVT_BUF_REQ:
		err = nrfx_saadc_buffer_set(sample_buffers[runtime.next_buffer_index],
					    ADC_DMA_BUFFER_SAMPLES);
		if (err != 0) {
			atomic_inc(&runtime.saadc_errors);
		} else {
			runtime.next_buffer_index ^= 1U;
		}
		break;

	case NRFX_SAADC_EVT_DONE:
		process_completed_buffer(&event->data.done);
		break;

	case NRFX_SAADC_EVT_FINISHED:
		/* Continuous mode should only finish if a replacement buffer was missed. */
		atomic_inc(&runtime.saadc_errors);
		break;

	default:
		break;
	}
}

int adc_trigger_init(struct trigger_engine *engine, struct k_msgq *hit_queue)
{
	nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(
		DT_PROP(ADC_CHANNEL_NODE, zephyr_input_positive), DT_REG_ADDR(ADC_CHANNEL_NODE));
	nrfx_saadc_adv_config_t advanced_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
	int err;

	if ((engine == NULL) || (hit_queue == NULL)) {
		return -EINVAL;
	}

	runtime = (struct adc_runtime){
		.engine = engine,
		.hit_queue = hit_queue,
		.dma_interval_min_us = ATOMIC_INIT(INT_MAX),
		.next_buffer_index = 1U,
	};

	/*
	 * We own the SAADC directly through nrfx, therefore
	 * we must connect the peripheral IRQ ourselves.
	 */
	IRQ_CONNECT(DT_IRQN(ADC_DEVICE_NODE),
	    DT_IRQ(ADC_DEVICE_NODE, priority),
	    nrfx_isr,
	    nrfx_saadc_irq_handler,
	    0);

	err = nrfx_saadc_init(0U);
	if (err != 0) {
		return err;
	}

	/* Blocking calibration is acceptable during initialization, before sampling starts. */
	err = nrfx_saadc_offset_calibrate(NULL);
	if (err != 0) {
		goto fail;
	}

	channel.channel_config.gain = NRF_SAADC_GAIN1_6;
	channel.channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
	channel.channel_config.acq_time = NRF_SAADC_ACQTIME_20US;
	channel.channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;
	channel.channel_config.burst = NRF_SAADC_BURST_DISABLED;

	err = nrfx_saadc_channel_config(&channel);
	if (err != 0) {
		goto fail;
	}

	advanced_config.internal_timer_cc = (uint16_t)SAADC_TIMER_CC;
	advanced_config.start_on_end = true;

	err = nrfx_saadc_advanced_mode_set(BIT(DT_REG_ADDR(ADC_CHANNEL_NODE)),
					   NRF_SAADC_RESOLUTION_12BIT, &advanced_config,
					   saadc_event_handler);
	if (err != 0) {
		goto fail;
	}

	err = nrfx_saadc_buffer_set(sample_buffers[0], ADC_DMA_BUFFER_SAMPLES);
	if (err != 0) {
		goto fail;
	}

	runtime.initialized = true;
	return 0;

fail:
	nrfx_saadc_uninit();
	return err;
}

int adc_trigger_start(void)
{
	int err;

	if (!runtime.initialized) {
		return -EACCES;
	}

	if (runtime.started) {
		return -EALREADY;
	}

	err = nrfx_saadc_mode_trigger();
	if (err != 0) {
		return err;
	}

	runtime.started = true;
	return 0;
}

void adc_trigger_get_stats(struct adc_trigger_stats *stats)
{
	uint32_t minimum;

	if (stats == NULL) {
		return;
	}

	minimum = (uint32_t)atomic_get(&runtime.dma_interval_min_us);
	if (minimum == (uint32_t)INT_MAX) {
		minimum = 0U;
	}

	*stats = (struct adc_trigger_stats){
		.samples_acquired = (uint32_t)atomic_get(&runtime.samples_acquired),
		.dma_buffers_processed = (uint32_t)atomic_get(&runtime.dma_buffers_processed),
		.queue_overruns = (uint32_t)atomic_get(&runtime.queue_overruns),
		.saadc_errors = (uint32_t)atomic_get(&runtime.saadc_errors),
		.dma_interval_min_us = minimum,
		.dma_interval_max_us = (uint32_t)atomic_get(&runtime.dma_interval_max_us),
		.latest_raw = (int16_t)atomic_get(&runtime.latest_raw),
		.baseline = (int16_t)atomic_get(&runtime.baseline),
		.trigger_state = (enum trigger_state)atomic_get(&runtime.trigger_state),
		.trigger_armed = atomic_get(&runtime.trigger_armed) != 0,
	};
}

uint32_t adc_trigger_get_sample_rate_hz(void)
{
	return ADC_SAMPLE_RATE_HZ;
}

uint16_t adc_trigger_get_dma_buffer_samples(void)
{
	return ADC_DMA_BUFFER_SAMPLES;
}

#else

int adc_trigger_init(struct trigger_engine *engine, struct k_msgq *hit_queue)
{
	ARG_UNUSED(engine);
	ARG_UNUSED(hit_queue);
	return -ENOTSUP;
}

int adc_trigger_start(void)
{
	return -ENOTSUP;
}

void adc_trigger_get_stats(struct adc_trigger_stats *stats)
{
	if (stats != NULL) {
		*stats = (struct adc_trigger_stats){0};
	}
}

uint32_t adc_trigger_get_sample_rate_hz(void)
{
	return 0U;
}

uint16_t adc_trigger_get_dma_buffer_samples(void)
{
	return 0U;
}

#endif /* CONFIG_DRUM_SAADC_STREAM */
