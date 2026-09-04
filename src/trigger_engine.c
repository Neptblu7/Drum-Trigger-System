#include "trigger_engine.h"

#include <errno.h>
#include <limits.h>

#include <zephyr/irq.h>

#define BASELINE_FRACTION_BITS  8U
#define BASELINE_FRACTION_SCALE (1L << BASELINE_FRACTION_BITS)

static int16_t baseline_counts(const struct trigger_engine *engine)
{
	return (int16_t)(engine->baseline_q8 / BASELINE_FRACTION_SCALE);
}

static uint16_t magnitude_from_centered(int32_t centered)
{
	return (uint16_t)((centered < 0) ? -centered : centered);
}

static uint16_t positive_u16(int32_t value)
{
	return (value > 0) ? (uint16_t)value : 0U;
}

static void update_baseline(struct trigger_engine *engine, int16_t raw)
{
	int32_t target_q8 = (int32_t)raw * BASELINE_FRACTION_SCALE;
	int32_t divisor = 1L << engine->config->baseline_filter_shift;

	engine->baseline_q8 += (target_q8 - engine->baseline_q8) / divisor;
}

static int32_t history_delta(const struct trigger_engine *engine, int32_t centered)
{
	uint8_t history_samples = engine->config->slope_history_samples;
	uint8_t history_index;

	if (engine->centered_history_count < history_samples) {
		return centered - engine->previous_centered;
	}

	history_index = (uint8_t)((engine->centered_history_next +
				   TRIGGER_SLOPE_HISTORY_MAX_SAMPLES - history_samples) %
				  TRIGGER_SLOPE_HISTORY_MAX_SAMPLES);
	return centered - engine->centered_history[history_index];
}

static void remember_centered(struct trigger_engine *engine, int32_t centered)
{
	engine->centered_history[engine->centered_history_next] = centered;
	engine->centered_history_next =
		(uint8_t)((engine->centered_history_next + 1U) % TRIGGER_SLOPE_HISTORY_MAX_SAMPLES);
	if (engine->centered_history_count < TRIGGER_SLOPE_HISTORY_MAX_SAMPLES) {
		engine->centered_history_count++;
	}
}

static void reset_centered_history(struct trigger_engine *engine, int32_t centered)
{
	for (uint8_t i = 0U; i < TRIGGER_SLOPE_HISTORY_MAX_SAMPLES; ++i) {
		engine->centered_history[i] = centered;
	}
	engine->centered_history_next = 0U;
	engine->centered_history_count = TRIGGER_SLOPE_HISTORY_MAX_SAMPLES;
}

static void update_peak(struct trigger_engine *engine, int16_t raw, int32_t centered)
{
	uint16_t magnitude = magnitude_from_centered(centered);
	uint16_t raw_u16 = (uint16_t)raw;

	if (magnitude > engine->absolute_peak) {
		engine->absolute_peak = magnitude;
	}

	if ((centered > 0) && ((uint16_t)centered > engine->positive_peak)) {
		engine->positive_peak = (uint16_t)centered;
	} else if ((centered < 0) && ((uint16_t)-centered > engine->negative_peak)) {
		engine->negative_peak = (uint16_t)-centered;
	}

	if (raw_u16 < engine->min_raw) {
		engine->min_raw = raw_u16;
	}
	if (raw_u16 > engine->max_raw) {
		engine->max_raw = raw_u16;
	}
	if (raw_u16 <= engine->config->adc_clip_low_threshold) {
		engine->clip_low = true;
	}
	if (raw_u16 >= engine->config->adc_clip_high_threshold) {
		engine->clip_high = true;
	}
}

static uint16_t onset_excursion(const struct drum_trigger_config *config, int32_t centered)
{
	switch (config->polarity) {
	case TRIGGER_POLARITY_NEGATIVE:
		return positive_u16(-centered);
	case TRIGGER_POLARITY_POSITIVE:
		return positive_u16(centered);
	case TRIGGER_POLARITY_BOTH:
		return magnitude_from_centered(centered);
	default:
		return 0U;
	}
}

static uint16_t onset_slope(const struct drum_trigger_config *config, int32_t centered,
			    int32_t delta)
{
	switch (config->polarity) {
	case TRIGGER_POLARITY_NEGATIVE:
		return positive_u16(-delta);
	case TRIGGER_POLARITY_POSITIVE:
		return positive_u16(delta);
	case TRIGGER_POLARITY_BOTH:
		/* The slope must move in the same direction as the excursion. */
		return (centered < 0) ? positive_u16(-delta) : positive_u16(delta);
	default:
		return 0U;
	}
}

static uint16_t release_excursion(const struct drum_trigger_config *config, int32_t centered)
{
	if (config->polarity == TRIGGER_POLARITY_NEGATIVE) {
		return positive_u16(centered);
	}

	if (config->polarity == TRIGGER_POLARITY_POSITIVE) {
		return positive_u16(-centered);
	}

	return 0U;
}

static uint16_t release_slope(const struct drum_trigger_config *config, int32_t delta)
{
	if (config->polarity == TRIGGER_POLARITY_NEGATIVE) {
		return positive_u16(delta);
	}

	if (config->polarity == TRIGGER_POLARITY_POSITIVE) {
		return positive_u16(-delta);
	}

	return 0U;
}

static bool onset_amplitude_met(const struct drum_trigger_config *config, int32_t centered)
{
	return onset_excursion(config, centered) >= config->trigger_threshold;
}

static bool valid_onset(const struct drum_trigger_config *config, int32_t centered, int32_t delta1,
			int32_t history_delta_value)
{
	return onset_amplitude_met(config, centered) &&
	       (onset_slope(config, centered, delta1) >= config->onset_slope_threshold) &&
	       (onset_slope(config, centered, history_delta_value) >=
		config->onset_history_slope_threshold);
}

static bool valid_release(const struct drum_trigger_config *config, int32_t centered, int32_t delta)
{
	return (config->release_threshold > 0U) &&
	       (release_excursion(config, centered) >= config->release_threshold) &&
	       (release_slope(config, delta) >= config->release_slope_threshold);
}

static uint16_t selected_peak(const struct trigger_engine *engine)
{
	switch (engine->config->polarity) {
	case TRIGGER_POLARITY_NEGATIVE:
		return engine->negative_peak;
	case TRIGGER_POLARITY_POSITIVE:
		return engine->positive_peak;
	case TRIGGER_POLARITY_BOTH:
	default:
		return engine->absolute_peak;
	}
}

static uint8_t peak_to_velocity(const struct drum_trigger_config *config, uint16_t peak)
{
	uint32_t numerator;
	uint32_t denominator;
	uint32_t index;

	if (peak <= config->velocity_min_peak) {
		return 1U;
	}

	if (peak >= config->velocity_max_peak) {
		return 127U;
	}

	numerator = (uint32_t)peak - config->velocity_min_peak;
	denominator = (uint32_t)config->velocity_max_peak - config->velocity_min_peak;

	if ((config->velocity_curve != NULL) && (config->velocity_curve_size >= 2U)) {
		index = ((numerator * (config->velocity_curve_size - 1U)) + (denominator / 2U)) /
			denominator;
		return config->velocity_curve[index];
	}

	return (uint8_t)(1U + ((126U * numerator) / denominator));
}

static void begin_peak_capture(struct trigger_engine *engine, int16_t raw, int32_t centered,
			       int32_t delta1, int32_t history_delta_value,
			       enum drum_hit_accept_path accept_path, uint32_t timestamp_us)
{
	engine->state = TRIGGER_STATE_PEAK_CAPTURE;
	engine->threshold_timestamp_us = timestamp_us;
	engine->onset_delta = (int16_t)delta1;
	engine->onset_history_delta = (int16_t)history_delta_value;
	engine->accept_path = accept_path;
	engine->positive_peak = 0U;
	engine->negative_peak = 0U;
	engine->absolute_peak = 0U;
	engine->min_raw = UINT16_MAX;
	engine->max_raw = 0U;
	engine->clip_low = false;
	engine->clip_high = false;
	engine->peak_samples = 1U;
	update_peak(engine, raw, centered);
}

static void begin_release_recovery(struct trigger_engine *engine, int32_t centered, int32_t delta,
				   uint32_t timestamp_us)
{
	engine->state = TRIGGER_STATE_RELEASE_RECOVERY;
	engine->state_samples = 1U;
	engine->rearm_stable_samples = 0U;
	engine->release_timestamp_us = timestamp_us;
	engine->release_initial_centered = (int16_t)centered;
	engine->release_peak = release_excursion(engine->config, centered);
	engine->release_max_slope = release_slope(engine->config, delta);
	engine->release_ringback_peak = 0U;
	engine->release_ringback_slope1 = 0U;
	engine->release_ringback_history_slope = 0U;
	engine->release_ringback_suppressed = false;
	engine->diagnostics.release_detected_count++;
	engine->diagnostics.polarity_reject_count++;
}

static bool finish_peak_capture(struct trigger_engine *engine, uint32_t timestamp_us,
				struct drum_hit_event *event_out)
{
	uint16_t musical_peak = selected_peak(engine);

	event_out->drum = engine->config->drum;
	event_out->accept_path = engine->accept_path;
	event_out->peak = musical_peak;
	event_out->positive_peak = engine->positive_peak;
	event_out->negative_peak = engine->negative_peak;
	event_out->onset_delta = engine->onset_delta;
	event_out->onset_history_delta = engine->onset_history_delta;
	event_out->min_raw = engine->min_raw;
	event_out->max_raw = engine->max_raw;
	event_out->velocity = peak_to_velocity(engine->config, musical_peak);
	event_out->clip_low = engine->clip_low;
	event_out->clip_high = engine->clip_high;
	event_out->timestamp_us = engine->threshold_timestamp_us;
	event_out->peak_complete_timestamp_us = timestamp_us;

	/* TRIGGER_EVENT is an instantaneous transition, not a blocking state. */
	engine->state = TRIGGER_STATE_HARD_MASK;
	engine->state_samples = 0U;
	engine->rearm_stable_samples = 0U;
	return true;
}

static void finish_release_diagnostics(struct trigger_engine *engine, uint32_t timestamp_us)
{
	engine->diagnostics.last_release_initial_centered = engine->release_initial_centered;
	engine->diagnostics.last_release_peak = engine->release_peak;
	engine->diagnostics.last_release_max_slope = engine->release_max_slope;
	engine->diagnostics.last_release_ringback_peak = engine->release_ringback_peak;
	engine->diagnostics.last_release_ringback_slope1 = engine->release_ringback_slope1;
	engine->diagnostics.last_release_ringback_history_slope =
		engine->release_ringback_history_slope;
	engine->diagnostics.last_release_recovery_us = timestamp_us - engine->release_timestamp_us;
	engine->diagnostics.release_sequence++;
}

static void update_release_recovery(struct trigger_engine *engine, int16_t raw, int32_t centered,
				    int32_t delta1, int32_t history_delta_value, uint16_t magnitude,
				    uint32_t timestamp_us)
{
	uint16_t current_release_peak = release_excursion(engine->config, centered);
	uint16_t current_release_slope = release_slope(engine->config, delta1);
	uint16_t current_ringback_peak = onset_excursion(engine->config, centered);
	uint16_t current_ringback_slope1 = onset_slope(engine->config, centered, delta1);
	uint16_t current_ringback_history_slope =
		onset_slope(engine->config, centered, history_delta_value);

	if (current_release_peak > engine->release_peak) {
		engine->release_peak = current_release_peak;
	}
	if (current_release_slope > engine->release_max_slope) {
		engine->release_max_slope = current_release_slope;
	}
	if (current_ringback_peak > engine->release_ringback_peak) {
		engine->release_ringback_peak = current_ringback_peak;
	}
	if (current_ringback_slope1 > engine->release_ringback_slope1) {
		engine->release_ringback_slope1 = current_ringback_slope1;
	}
	if (current_ringback_history_slope > engine->release_ringback_history_slope) {
		engine->release_ringback_history_slope = current_ringback_history_slope;
	}

	if (engine->state_samples < UINT16_MAX) {
		engine->state_samples++;
	}

	if ((engine->state_samples >= engine->config->release_guard_samples) &&
	    onset_amplitude_met(engine->config, centered) &&
	    (onset_slope(engine->config, centered, history_delta_value) >=
	     engine->config->retrigger_slope_threshold)) {
		engine->diagnostics.release_retrigger_accept_count++;
		finish_release_diagnostics(engine, timestamp_us);
		begin_peak_capture(engine, raw, centered, delta1, history_delta_value,
				   DRUM_HIT_ACCEPT_RELEASE_RECOVERY, timestamp_us);
		return;
	}

	if (!engine->release_ringback_suppressed &&
	    (current_ringback_peak >= engine->config->trigger_threshold)) {
		engine->release_ringback_suppressed = true;
		engine->diagnostics.release_suppressed_count++;
	}

	if (engine->state_samples < engine->config->release_guard_samples) {
		engine->rearm_stable_samples = 0U;
		return;
	}

	if (magnitude <= engine->config->release_rearm_threshold) {
		if (++engine->rearm_stable_samples >= engine->config->release_stable_samples) {
			engine->state = TRIGGER_STATE_IDLE;
			engine->rearm_stable_samples = 0U;
			reset_centered_history(engine, centered);
			finish_release_diagnostics(engine, timestamp_us);
		}
	} else {
		engine->rearm_stable_samples = 0U;
	}
}

int trigger_engine_init(struct trigger_engine *engine, const struct drum_trigger_config *config)
{
	if ((engine == NULL) || (config == NULL)) {
		return -EINVAL;
	}

	if ((config->polarity < TRIGGER_POLARITY_NEGATIVE) ||
	    (config->polarity > TRIGGER_POLARITY_BOTH) || (config->trigger_threshold == 0U) ||
	    (config->rearm_threshold >= config->trigger_threshold) ||
	    (config->slope_history_samples == 0U) ||
	    (config->slope_history_samples > TRIGGER_SLOPE_HISTORY_MAX_SAMPLES) ||
	    (config->baseline_track_threshold > config->rearm_threshold) ||
	    (config->adc_clip_low_threshold >= config->adc_clip_high_threshold) ||
	    (config->peak_window_samples == 0U) || (config->hard_mask_samples == 0U) ||
	    (config->rearm_stable_samples == 0U) || (config->baseline_settle_samples == 0U) ||
	    (config->velocity_max_peak <= config->velocity_min_peak) ||
	    (config->baseline_filter_shift == 0U) || (config->baseline_filter_shift >= 31U)) {
		return -EINVAL;
	}

	if ((config->release_threshold > 0U) &&
	    ((config->release_rearm_threshold >= config->release_threshold) ||
	     (config->release_stable_samples == 0U))) {
		return -EINVAL;
	}

	*engine = (struct trigger_engine){
		.config = config,
		.state = TRIGGER_STATE_IDLE,
	};

	return 0;
}

bool trigger_engine_process_sample(struct trigger_engine *engine, int16_t raw,
				   uint32_t timestamp_us, struct drum_hit_event *event_out)
{
	int32_t centered;
	int32_t delta;
	int32_t delta_history;
	uint16_t magnitude;
	bool event_ready = false;

	if ((engine == NULL) || (engine->config == NULL) || (event_out == NULL)) {
		return false;
	}

	if (!engine->baseline_initialized) {
		engine->baseline_q8 = (int32_t)raw * BASELINE_FRACTION_SCALE;
		engine->baseline_initialized = true;
		engine->previous_centered = 0;
		engine->previous_centered_valid = true;
		remember_centered(engine, 0);
		return false;
	}

	centered = (int32_t)raw - baseline_counts(engine);
	delta = engine->previous_centered_valid ? centered - engine->previous_centered : 0;
	delta_history = history_delta(engine, centered);
	magnitude = magnitude_from_centered(centered);

	if (!engine->armed) {
		/* Startup learning is intentionally separate from the armed-state quiet gate. */
		update_baseline(engine, raw);
		if (++engine->settle_samples >= engine->config->baseline_settle_samples) {
			engine->armed = true;
		}
		goto done;
	}

	switch (engine->state) {
	case TRIGGER_STATE_IDLE:
		if (valid_onset(engine->config, centered, delta, delta_history)) {
			begin_peak_capture(engine, raw, centered, delta, delta_history,
					   DRUM_HIT_ACCEPT_NORMAL, timestamp_us);
		} else if (valid_release(engine->config, centered, delta)) {
			begin_release_recovery(engine, centered, delta, timestamp_us);
		} else {
			if (onset_amplitude_met(engine->config, centered) &&
			    ((onset_slope(engine->config, centered, delta) <
			      engine->config->onset_slope_threshold) ||
			     (onset_slope(engine->config, centered, delta_history) <
			      engine->config->onset_history_slope_threshold)) &&
			    !onset_amplitude_met(engine->config, engine->previous_centered)) {
				engine->diagnostics.slope_reject_count++;
				engine->diagnostics.last_slope_reject_delta1 = (int16_t)delta;
				engine->diagnostics.last_slope_reject_history_delta =
					(int16_t)delta_history;
			}

			if (magnitude <= engine->config->baseline_track_threshold) {
				update_baseline(engine, raw);
			}
		}
		break;

	case TRIGGER_STATE_PEAK_CAPTURE:
		update_peak(engine, raw, centered);
		if (++engine->peak_samples >= engine->config->peak_window_samples) {
			event_ready = finish_peak_capture(engine, timestamp_us, event_out);
		}
		break;

	case TRIGGER_STATE_HARD_MASK:
		if (++engine->state_samples >= engine->config->hard_mask_samples) {
			engine->state = TRIGGER_STATE_REARM;
			engine->rearm_stable_samples =
				(magnitude <= engine->config->rearm_threshold) ? 1U : 0U;
		}
		break;

	case TRIGGER_STATE_REARM:
		if (magnitude <= engine->config->rearm_threshold) {
			if (++engine->rearm_stable_samples >=
			    engine->config->rearm_stable_samples) {
				engine->state = TRIGGER_STATE_IDLE;
				engine->rearm_stable_samples = 0U;
				reset_centered_history(engine, centered);
			}
		} else {
			engine->rearm_stable_samples = 0U;
		}
		break;

	case TRIGGER_STATE_RELEASE_RECOVERY:
		update_release_recovery(engine, raw, centered, delta, delta_history, magnitude,
					timestamp_us);
		break;

	default:
		engine->state = TRIGGER_STATE_IDLE;
		break;
	}

done:
	engine->previous_centered = centered;
	engine->previous_centered_valid = true;
	remember_centered(engine, centered);
	return event_ready;
}

int16_t trigger_engine_get_baseline(const struct trigger_engine *engine)
{
	if ((engine == NULL) || !engine->baseline_initialized) {
		return 0;
	}

	return baseline_counts(engine);
}

enum trigger_state trigger_engine_get_state(const struct trigger_engine *engine)
{
	return (engine != NULL) ? engine->state : TRIGGER_STATE_IDLE;
}

bool trigger_engine_is_armed(const struct trigger_engine *engine)
{
	return (engine != NULL) && engine->armed;
}

void trigger_engine_get_diagnostics(const struct trigger_engine *engine,
				    struct trigger_engine_diagnostics *diagnostics)
{
	unsigned int key;

	if ((engine == NULL) || (diagnostics == NULL)) {
		return;
	}

	/* The engine is written by the SAADC IRQ; take a coherent thread-side snapshot. */
	key = irq_lock();
	*diagnostics = engine->diagnostics;
	irq_unlock(key);
}

const char *trigger_engine_state_name(enum trigger_state state)
{
	switch (state) {
	case TRIGGER_STATE_IDLE:
		return "IDLE";
	case TRIGGER_STATE_PEAK_CAPTURE:
		return "PEAK_CAPTURE";
	case TRIGGER_STATE_HARD_MASK:
		return "HARD_MASK";
	case TRIGGER_STATE_REARM:
		return "REARM";
	case TRIGGER_STATE_RELEASE_RECOVERY:
		return "RELEASE_RECOVERY";
	default:
		return "UNKNOWN";
	}
}

const char *trigger_engine_polarity_name(enum trigger_polarity polarity)
{
	switch (polarity) {
	case TRIGGER_POLARITY_NEGATIVE:
		return "NEGATIVE";
	case TRIGGER_POLARITY_POSITIVE:
		return "POSITIVE";
	case TRIGGER_POLARITY_BOTH:
		return "BOTH";
	default:
		return "UNKNOWN";
	}
}
