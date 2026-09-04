#include "wav_trigger.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/irq.h>

#define WAV_TRIGGER_I2C_NODE DT_NODELABEL(i2c1)

#if !DT_NODE_HAS_STATUS(WAV_TRIGGER_I2C_NODE, okay)
#error "i2c1 is not enabled in devicetree"
#endif

/* Robertsonics WAV Trigger Pro Qwiic protocol (7-bit address). */
#define WAV_TRIGGER_I2C_ADDRESS         0x13U
#define WAV_TRIGGER_CMD_GET_NUM_TRACKS  2U
#define WAV_TRIGGER_CMD_TRACK_PLAY_POLY 3U
#define WAV_TRIGGER_CMD_STOP_ALL        8U
#define WAV_TRIGGER_CMD_SET_OUTPUT_GAIN 13U

#define WAV_TRIGGER_TRACK_MIN         1U
#define WAV_TRIGGER_TRACK_MAX         4096U
#define WAV_TRIGGER_GAIN_MIN_DB       (-80)
#define WAV_TRIGGER_GAIN_MAX_DB       0
#define WAV_TRIGGER_BALANCE_CENTER    64U
#define WAV_TRIGGER_RESPONSE_DELAY_MS 2U

static const struct device *const i2c_device = DEVICE_DT_GET(WAV_TRIGGER_I2C_NODE);
static atomic_t i2c_error_count;
static bool transport_ready;

static int write_command(const uint8_t *data, size_t length)
{
	if (!transport_ready) {
		atomic_inc(&i2c_error_count);
		return -ENODEV;
	}

	int err = i2c_write(i2c_device, data, length, WAV_TRIGGER_I2C_ADDRESS);

	if (err != 0) {
		atomic_inc(&i2c_error_count);
	}

	return err;
}

static int read_response(uint8_t *data, size_t length)
{
	if (!transport_ready) {
		atomic_inc(&i2c_error_count);
		return -ENODEV;
	}

	int err = i2c_read(i2c_device, data, length, WAV_TRIGGER_I2C_ADDRESS);

	if (err != 0) {
		atomic_inc(&i2c_error_count);
	}

	return err;
}

static void put_le16(uint8_t *destination, int16_t value)
{
	uint16_t raw = (uint16_t)value;

	destination[0] = (uint8_t)(raw & 0xffU);
	destination[1] = (uint8_t)(raw >> 8);
}

int wav_trigger_init(void)
{
	transport_ready = false;
	atomic_set(&i2c_error_count, 0);

	if (!device_is_ready(i2c_device)) {
		return -ENODEV;
	}

	transport_ready = true;
	return 0;
}

int wav_trigger_get_num_tracks(uint16_t *num_tracks)
{
	uint8_t command = WAV_TRIGGER_CMD_GET_NUM_TRACKS;
	uint8_t response[2] = {0};
	int err;

	if (num_tracks == NULL) {
		return -EINVAL;
	}

	err = write_command(&command, sizeof(command));
	if (err != 0) {
		return err;
	}

	/* The playback module prepares its two-byte response after the command ACK. */
	k_msleep(WAV_TRIGGER_RESPONSE_DELAY_MS);

	err = read_response(response, sizeof(response));
	if (err != 0) {
		return err;
	}

	*num_tracks = (uint16_t)response[0] | ((uint16_t)response[1] << 8);
	return 0;
}

int wav_trigger_play_track(uint16_t track, int16_t gain_db)
{
	uint8_t command[11] = {
		WAV_TRIGGER_CMD_TRACK_PLAY_POLY,
		(uint8_t)(track & 0xffU),
		(uint8_t)(track >> 8),
		0U,
		0U,
		WAV_TRIGGER_BALANCE_CENTER,
		0U, /* attack time, LSB */
		0U, /* attack time, MSB */
		0U, /* pitch offset, LSB */
		0U, /* pitch offset, MSB */
		0U, /* no loop, voice lock, or pitch-bend flags */
	};

	if ((track < WAV_TRIGGER_TRACK_MIN) || (track > WAV_TRIGGER_TRACK_MAX)) {
		return -EINVAL;
	}

	if ((gain_db < WAV_TRIGGER_GAIN_MIN_DB) || (gain_db > WAV_TRIGGER_GAIN_MAX_DB)) {
		return -ERANGE;
	}

	put_le16(&command[3], gain_db);
	return write_command(command, sizeof(command));
}

int wav_trigger_stop_all(void)
{
	uint8_t command = WAV_TRIGGER_CMD_STOP_ALL;

	return write_command(&command, sizeof(command));
}

int wav_trigger_set_output_gain(int16_t gain_db)
{
	uint8_t command[3] = {WAV_TRIGGER_CMD_SET_OUTPUT_GAIN, 0U, 0U};

	if ((gain_db < WAV_TRIGGER_GAIN_MIN_DB) || (gain_db > WAV_TRIGGER_GAIN_MAX_DB)) {
		return -ERANGE;
	}

	put_le16(&command[1], gain_db);
	return write_command(command, sizeof(command));
}

uint32_t wav_trigger_get_error_count(void)
{
	return (uint32_t)atomic_get(&i2c_error_count);
}
