#ifndef WAV_TRIGGER_H_
#define WAV_TRIGGER_H_

#include <stdint.h>

/*
 * Initialize the WAV Trigger Pro transport. This only validates that the
 * Zephyr I2C controller is ready; use wav_trigger_get_num_tracks() to probe
 * the playback module itself.
 */
int wav_trigger_init(void);

/* Query the number of numbered WAV tracks on the microSD card. */
int wav_trigger_get_num_tracks(uint16_t *num_tracks);

/*
 * Start a new, one-shot polyphonic instance of track. Existing voices,
 * including another copy of the same track, are intentionally left playing.
 */
int wav_trigger_play_track(uint16_t track, int16_t gain_db);

int wav_trigger_stop_all(void);
int wav_trigger_set_output_gain(int16_t gain_db);

/* Total failed I2C transfers since boot. */
uint32_t wav_trigger_get_error_count(void);

#endif /* WAV_TRIGGER_H_ */
