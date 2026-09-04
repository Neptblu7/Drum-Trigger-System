#ifndef DRUM_TYPES_H_
#define DRUM_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

enum drum_id {
	DRUM_ID_KICK = 0,
	DRUM_ID_COUNT,
};

enum drum_hit_accept_path {
	DRUM_HIT_ACCEPT_NORMAL = 0,
	DRUM_HIT_ACCEPT_RELEASE_RECOVERY,
};

struct drum_hit_event {
	enum drum_id drum;
	enum drum_hit_accept_path accept_path;
	uint16_t peak;
	uint16_t positive_peak;
	uint16_t negative_peak;
	int16_t onset_delta;
	int16_t onset_history_delta;
	uint16_t min_raw;
	uint16_t max_raw;
	uint8_t velocity;
	bool clip_low;
	bool clip_high;
	uint32_t timestamp_us;
	uint32_t peak_complete_timestamp_us;
};

#endif /* DRUM_TYPES_H_ */
