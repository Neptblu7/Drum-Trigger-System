#ifndef SAMPLE_ENGINE_H_
#define SAMPLE_ENGINE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "drum_types.h"

struct velocity_layer {
	uint8_t velocity_min;
	uint8_t velocity_max;
	const uint16_t *tracks;
	size_t track_count;
	int16_t gain_db;
};

enum velocity_gain_scope {
	VELOCITY_GAIN_FULL_RANGE = 0,
	VELOCITY_GAIN_WITHIN_LAYER,
};

struct velocity_gain_config {
	bool enabled;
	enum velocity_gain_scope scope;
	int16_t minimum_db;
	int16_t maximum_db;
};

struct drum_sample_config {
	enum drum_id drum;
	const struct velocity_layer *layers;
	size_t layer_count;
	struct velocity_gain_config velocity_gain;
};

struct sample_selection {
	uint16_t track;
	int16_t gain_db;
	int16_t velocity_gain_db;
	uint8_t layer_index;
	uint8_t round_robin_index;
};

struct sample_engine_stats {
	uint32_t hits_processed;
	uint32_t selection_errors;
	uint32_t playback_errors;
	uint32_t last_command_latency_us;
	uint32_t last_i2c_duration_us;
};

int sample_engine_start(struct k_msgq *hit_queue, const struct drum_sample_config *configs,
			size_t config_count);

/* Deterministic selection API; advances the selected layer's round-robin state. */
int sample_engine_select(const struct drum_hit_event *event, struct sample_selection *selection);

void sample_engine_get_stats(struct sample_engine_stats *stats);

#endif /* SAMPLE_ENGINE_H_ */
