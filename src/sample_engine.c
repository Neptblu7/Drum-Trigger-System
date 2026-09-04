#include "sample_engine.h"

#include <errno.h>
#include <stdbool.h>

#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include "wav_trigger.h"

#define SAMPLE_ENGINE_MAX_LAYERS           8U
#define SAMPLE_ENGINE_MAX_TRACKS_PER_LAYER 32U
#define SAMPLE_ENGINE_TRACK_MIN            1U
#define SAMPLE_ENGINE_TRACK_MAX            4096U
#define SAMPLE_ENGINE_GAIN_MIN_DB          (-80)
#define SAMPLE_ENGINE_GAIN_MAX_DB          0

struct sample_engine_runtime {
	struct k_msgq *hit_queue;
	const struct drum_sample_config *configs;
	size_t config_count;
	uint16_t next_round_robin[DRUM_ID_COUNT][SAMPLE_ENGINE_MAX_LAYERS];
	atomic_t hits_processed;
	atomic_t selection_errors;
	atomic_t playback_errors;
	atomic_t last_command_latency_us;
	atomic_t last_i2c_duration_us;
	bool started;
};

static struct sample_engine_runtime runtime;
static struct k_thread playback_thread;
K_THREAD_STACK_DEFINE(playback_stack, CONFIG_DRUM_PLAYBACK_THREAD_STACK_SIZE);

static uint32_t uptime_us_32(void)
{
	return (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
}

static const char *drum_name(enum drum_id drum)
{
	return (drum == DRUM_ID_KICK) ? "KICK" : "UNKNOWN";
}

static const char *accept_path_name(enum drum_hit_accept_path path)
{
	return (path == DRUM_HIT_ACCEPT_RELEASE_RECOVERY) ? "RELEASE" : "NORMAL";
}

static int validate_sample_configs(const struct drum_sample_config *configs, size_t config_count)
{
	bool configured_drums[DRUM_ID_COUNT] = {false};

	if ((configs == NULL) || (config_count == 0U)) {
		return -EINVAL;
	}

	for (size_t drum_index = 0U; drum_index < config_count; ++drum_index) {
		const struct drum_sample_config *config = &configs[drum_index];
		uint16_t expected_velocity = 1U;

		if ((config->drum < 0) || (config->drum >= DRUM_ID_COUNT) ||
		    configured_drums[config->drum] || (config->layers == NULL) ||
		    (config->layer_count == 0U) ||
		    (config->layer_count > SAMPLE_ENGINE_MAX_LAYERS)) {
			return -EINVAL;
		}

		if (config->velocity_gain.enabled &&
		    ((config->velocity_gain.scope < VELOCITY_GAIN_FULL_RANGE) ||
		     (config->velocity_gain.scope > VELOCITY_GAIN_WITHIN_LAYER) ||
		     (config->velocity_gain.minimum_db > config->velocity_gain.maximum_db) ||
		     (config->velocity_gain.minimum_db < SAMPLE_ENGINE_GAIN_MIN_DB) ||
		     (config->velocity_gain.maximum_db > SAMPLE_ENGINE_GAIN_MAX_DB))) {
			return -EINVAL;
		}

		configured_drums[config->drum] = true;

		for (size_t layer_index = 0U; layer_index < config->layer_count; ++layer_index) {
			const struct velocity_layer *layer = &config->layers[layer_index];

			if ((layer->velocity_min != expected_velocity) ||
			    (layer->velocity_max < layer->velocity_min) ||
			    (layer->tracks == NULL) || (layer->track_count == 0U) ||
			    (layer->track_count > SAMPLE_ENGINE_MAX_TRACKS_PER_LAYER) ||
			    (layer->gain_db < SAMPLE_ENGINE_GAIN_MIN_DB) ||
			    (layer->gain_db > SAMPLE_ENGINE_GAIN_MAX_DB)) {
				return -EINVAL;
			}

			if (config->velocity_gain.enabled &&
			    (((int32_t)layer->gain_db + config->velocity_gain.minimum_db <
			      SAMPLE_ENGINE_GAIN_MIN_DB) ||
			     ((int32_t)layer->gain_db + config->velocity_gain.maximum_db >
			      SAMPLE_ENGINE_GAIN_MAX_DB))) {
				return -EINVAL;
			}

			for (size_t track_index = 0U; track_index < layer->track_count;
			     ++track_index) {
				if ((layer->tracks[track_index] < SAMPLE_ENGINE_TRACK_MIN) ||
				    (layer->tracks[track_index] > SAMPLE_ENGINE_TRACK_MAX)) {
					return -EINVAL;
				}
			}

			expected_velocity = (uint16_t)layer->velocity_max + 1U;
		}

		if (expected_velocity != 128U) {
			return -EINVAL;
		}
	}

	return 0;
}

static const struct drum_sample_config *find_drum_config(enum drum_id drum)
{
	for (size_t i = 0U; i < runtime.config_count; ++i) {
		if (runtime.configs[i].drum == drum) {
			return &runtime.configs[i];
		}
	}

	return NULL;
}

static int16_t velocity_gain_db(const struct drum_sample_config *config,
				const struct velocity_layer *layer, uint8_t velocity)
{
	uint8_t velocity_min = 1U;
	uint8_t velocity_max = 127U;
	uint32_t velocity_position;
	uint32_t velocity_range;
	uint32_t gain_range;
	int32_t scaled_gain;

	if (!config->velocity_gain.enabled) {
		return 0;
	}

	if (config->velocity_gain.scope == VELOCITY_GAIN_WITHIN_LAYER) {
		velocity_min = layer->velocity_min;
		velocity_max = layer->velocity_max;
	}

	if (velocity_max == velocity_min) {
		return config->velocity_gain.maximum_db;
	}

	velocity_position = (uint32_t)velocity - velocity_min;
	velocity_range = (uint32_t)velocity_max - velocity_min;
	gain_range = (uint32_t)((int32_t)config->velocity_gain.maximum_db -
				(int32_t)config->velocity_gain.minimum_db);
	scaled_gain = (int32_t)(((velocity_position * gain_range) + (velocity_range / 2U)) /
				velocity_range);

	return (int16_t)((int32_t)config->velocity_gain.minimum_db + scaled_gain);
}

int sample_engine_select(const struct drum_hit_event *event, struct sample_selection *selection)
{
	const struct drum_sample_config *config;

	if ((event == NULL) || (selection == NULL) || !runtime.started || (event->velocity < 1U) ||
	    (event->velocity > 127U)) {
		return -EINVAL;
	}

	config = find_drum_config(event->drum);
	if (config == NULL) {
		return -ENOENT;
	}

	for (size_t layer_index = 0U; layer_index < config->layer_count; ++layer_index) {
		const struct velocity_layer *layer = &config->layers[layer_index];

		if ((event->velocity >= layer->velocity_min) &&
		    (event->velocity <= layer->velocity_max)) {
			uint16_t round_robin = runtime.next_round_robin[event->drum][layer_index];
			int16_t dynamic_gain = velocity_gain_db(config, layer, event->velocity);

			selection->track = layer->tracks[round_robin];
			selection->velocity_gain_db = dynamic_gain;
			selection->gain_db = (int16_t)(layer->gain_db + dynamic_gain);
			selection->layer_index = (uint8_t)layer_index;
			selection->round_robin_index = (uint8_t)round_robin;

			runtime.next_round_robin[event->drum][layer_index] =
				(uint16_t)((round_robin + 1U) % layer->track_count);
			return 0;
		}
	}

	return -ENOENT;
}

static void playback_thread_entry(void *arg1, void *arg2, void *arg3)
{
	struct drum_hit_event event;
	uint32_t previous_hit_timestamp_us = 0U;
	bool have_previous_hit = false;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		struct sample_selection selection;
		uint32_t command_timestamp_us;
		uint32_t capture_duration_us;
		uint32_t dispatch_latency_us;
		uint32_t command_latency_us;
		uint32_t i2c_duration_us;
		uint32_t hit_interval_ms = 0U;
		int err;

		(void)k_msgq_get(runtime.hit_queue, &event, K_FOREVER);
		atomic_inc(&runtime.hits_processed);

		err = sample_engine_select(&event, &selection);
		if (err != 0) {
			atomic_inc(&runtime.selection_errors);
#if CONFIG_DRUM_DIAGNOSTICS
			printk("%s sample selection failed: %d\n", drum_name(event.drum), err);
#endif
			continue;
		}

		command_timestamp_us = uptime_us_32();
		capture_duration_us = event.peak_complete_timestamp_us - event.timestamp_us;
		dispatch_latency_us = command_timestamp_us - event.peak_complete_timestamp_us;
		command_latency_us = command_timestamp_us - event.timestamp_us;
		err = wav_trigger_play_track(selection.track, selection.gain_db);
		i2c_duration_us = uptime_us_32() - command_timestamp_us;

		atomic_set(&runtime.last_command_latency_us, (atomic_val_t)command_latency_us);
		atomic_set(&runtime.last_i2c_duration_us, (atomic_val_t)i2c_duration_us);

		if (err != 0) {
			atomic_inc(&runtime.playback_errors);
		}

		if (have_previous_hit) {
			hit_interval_ms = (event.timestamp_us - previous_hit_timestamp_us) / 1000U;
		}
		previous_hit_timestamp_us = event.timestamp_us;
		have_previous_hit = true;

#if CONFIG_DRUM_DIAGNOSTICS
		printk("%s peak=%u pos=%u neg=%u vel=%u delta1=%d delta3=%d path=%s "
		       "min_raw=%u max_raw=%u clip_low=%u clip_high=%u layer=%u rr=%u track=%u "
		       "gain_db=%d vel_gain_db=%d dt=%ums capture_us=%u dispatch_us=%u "
		       "latency_us=%u i2c_us=%u err=%d\n",
		       drum_name(event.drum), event.peak, event.positive_peak, event.negative_peak,
		       event.velocity, event.onset_delta, event.onset_history_delta,
		       accept_path_name(event.accept_path), event.min_raw, event.max_raw,
		       event.clip_low, event.clip_high, selection.layer_index,
		       selection.round_robin_index, selection.track, selection.gain_db,
		       selection.velocity_gain_db, hit_interval_ms, capture_duration_us,
		       dispatch_latency_us, command_latency_us, i2c_duration_us, err);
#endif
	}
}

int sample_engine_start(struct k_msgq *hit_queue, const struct drum_sample_config *configs,
			size_t config_count)
{
	int err;

	if ((hit_queue == NULL) || runtime.started) {
		return -EINVAL;
	}

	err = validate_sample_configs(configs, config_count);
	if (err != 0) {
		return err;
	}

	runtime = (struct sample_engine_runtime){
		.hit_queue = hit_queue,
		.configs = configs,
		.config_count = config_count,
		.started = true,
	};

	(void)k_thread_create(&playback_thread, playback_stack,
			      K_THREAD_STACK_SIZEOF(playback_stack), playback_thread_entry, NULL,
			      NULL, NULL, CONFIG_DRUM_PLAYBACK_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&playback_thread, "sample_playback");

	return 0;
}

void sample_engine_get_stats(struct sample_engine_stats *stats)
{
	if (stats == NULL) {
		return;
	}

	*stats = (struct sample_engine_stats){
		.hits_processed = (uint32_t)atomic_get(&runtime.hits_processed),
		.selection_errors = (uint32_t)atomic_get(&runtime.selection_errors),
		.playback_errors = (uint32_t)atomic_get(&runtime.playback_errors),
		.last_command_latency_us = (uint32_t)atomic_get(&runtime.last_command_latency_us),
		.last_i2c_duration_us = (uint32_t)atomic_get(&runtime.last_i2c_duration_us),
	};
}
