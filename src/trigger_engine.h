#ifndef TRIGGER_ENGINE_H_
#define TRIGGER_ENGINE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drum_types.h"

#define TRIGGER_SLOPE_HISTORY_MAX_SAMPLES 8U

enum trigger_polarity {
	TRIGGER_POLARITY_NEGATIVE = 0,
	TRIGGER_POLARITY_POSITIVE,
	TRIGGER_POLARITY_BOTH,
};

enum trigger_state {
	TRIGGER_STATE_IDLE = 0,
	TRIGGER_STATE_PEAK_CAPTURE,
	TRIGGER_STATE_HARD_MASK,
	TRIGGER_STATE_REARM,
	TRIGGER_STATE_RELEASE_RECOVERY,
};

struct drum_trigger_config {
	enum drum_id drum;
	enum trigger_polarity polarity;
	/* Amplitudes are centered ADC counts; slopes are counts per sample. */
	uint16_t trigger_threshold;
	uint16_t rearm_threshold;
	uint16_t onset_slope_threshold;
	uint16_t onset_history_slope_threshold;
	uint16_t retrigger_slope_threshold;
	uint8_t slope_history_samples;
	uint16_t release_threshold;
	uint16_t release_slope_threshold;
	uint16_t release_rearm_threshold;
	uint16_t release_guard_samples;
	uint16_t release_stable_samples;
	uint16_t baseline_track_threshold;
	uint16_t adc_clip_low_threshold;
	uint16_t adc_clip_high_threshold;
	uint16_t peak_window_samples;
	uint16_t hard_mask_samples;
	uint16_t rearm_stable_samples;
	uint16_t baseline_settle_samples;
	uint16_t velocity_min_peak;
	uint16_t velocity_max_peak;
	uint8_t baseline_filter_shift;
	const uint8_t *velocity_curve;
	size_t velocity_curve_size;
};

struct trigger_engine_diagnostics {
	uint32_t release_detected_count;
	uint32_t release_suppressed_count;
	uint32_t release_retrigger_accept_count;
	uint32_t polarity_reject_count;
	uint32_t slope_reject_count;
	int16_t last_slope_reject_delta1;
	int16_t last_slope_reject_history_delta;
	uint32_t release_sequence;
	int16_t last_release_initial_centered;
	uint16_t last_release_peak;
	uint16_t last_release_max_slope;
	uint16_t last_release_ringback_peak;
	uint16_t last_release_ringback_slope1;
	uint16_t last_release_ringback_history_slope;
	uint32_t last_release_recovery_us;
};

struct trigger_engine {
	const struct drum_trigger_config *config;
	enum trigger_state state;
	int32_t baseline_q8;
	int32_t previous_centered;
	int32_t centered_history[TRIGGER_SLOPE_HISTORY_MAX_SAMPLES];
	uint32_t threshold_timestamp_us;
	int16_t onset_delta;
	int16_t onset_history_delta;
	enum drum_hit_accept_path accept_path;
	uint16_t positive_peak;
	uint16_t negative_peak;
	uint16_t absolute_peak;
	uint16_t min_raw;
	uint16_t max_raw;
	uint16_t peak_samples;
	uint16_t state_samples;
	uint16_t rearm_stable_samples;
	uint16_t settle_samples;
	uint8_t centered_history_next;
	uint8_t centered_history_count;
	uint32_t release_timestamp_us;
	int16_t release_initial_centered;
	uint16_t release_peak;
	uint16_t release_max_slope;
	uint16_t release_ringback_peak;
	uint16_t release_ringback_slope1;
	uint16_t release_ringback_history_slope;
	struct trigger_engine_diagnostics diagnostics;
	bool baseline_initialized;
	bool previous_centered_valid;
	bool clip_low;
	bool clip_high;
	bool release_ringback_suppressed;
	bool armed;
};

int trigger_engine_init(struct trigger_engine *engine, const struct drum_trigger_config *config);

/*
 * Process exactly one uniformly spaced ADC sample. Returns true only when a
 * completed hit has been written to event_out.
 */
bool trigger_engine_process_sample(struct trigger_engine *engine, int16_t raw,
				   uint32_t timestamp_us, struct drum_hit_event *event_out);

int16_t trigger_engine_get_baseline(const struct trigger_engine *engine);
enum trigger_state trigger_engine_get_state(const struct trigger_engine *engine);
bool trigger_engine_is_armed(const struct trigger_engine *engine);
void trigger_engine_get_diagnostics(const struct trigger_engine *engine,
				    struct trigger_engine_diagnostics *diagnostics);
const char *trigger_engine_state_name(enum trigger_state state);
const char *trigger_engine_polarity_name(enum trigger_polarity polarity);

#endif /* TRIGGER_ENGINE_H_ */
