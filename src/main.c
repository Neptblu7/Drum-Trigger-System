
// west flash -d build\Trigger01 --runner jlink

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "adc_trigger.h"
#include "drum_config.h"
#include "sample_engine.h"
#include "trigger_engine.h"
#include "wav_trigger.h"

#define LED0_NODE DT_ALIAS(led0)

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "led0 alias is not defined for this board"
#endif

#define HIT_EVENT_QUEUE_DEPTH         8U
#define WAV_TRIGGER_POWER_UP_DELAY_MS 500U
#define HEALTH_REPORT_PERIOD_MS       5000U
#define SUPERVISOR_PERIOD_MS          250U
#define FAKE_HIT_PERIOD_MS            2000U

K_MSGQ_DEFINE(hit_event_queue, sizeof(struct drum_hit_event), HIT_EVENT_QUEUE_DEPTH,
	      sizeof(uint32_t));

static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static struct trigger_engine kick_engine;

#if CONFIG_DRUM_FAKE_TRIGGER_TEST
static uint32_t uptime_us_32(void)
{
	return (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
}
#endif

static bool initialize_status_led(void)
{
	int err;

	if (!gpio_is_ready_dt(&status_led)) {
		printk("LED GPIO not ready\n");
		return false;
	}

	err = gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		printk("LED configure failed: %d\n", err);
		return false;
	}

	return true;
}

static void initialize_wav_trigger(void)
{
	uint16_t num_tracks = 0U;
	int err = wav_trigger_init();

	if (err != 0) {
		printk("I2C not ready for WAV Trigger Pro: %d; acquisition will continue\n", err);
		return;
	}

	printk("I2C ready on XIAO D4/D5\n");

	/* Preserve the proven power-up settling time before probing the module. */
	k_msleep(WAV_TRIGGER_POWER_UP_DELAY_MS);

	err = wav_trigger_get_num_tracks(&num_tracks);
	if (err != 0) {
		printk("WAV Trigger Pro did not respond: %d; acquisition will continue\n", err);
		return;
	}

	printk("WAV Trigger Pro connected; tracks=%u\n", num_tracks);

	err = wav_trigger_stop_all();
	if (err != 0) {
		printk("WAV Trigger Stop All failed: %d\n", err);
	}

	err = wav_trigger_set_output_gain(0);
	if (err != 0) {
		printk("WAV Trigger output gain failed: %d\n", err);
	}
}

static int initialize_trigger_pipeline(void)
{
	int err;

	err = trigger_engine_init(&kick_engine, &kick_trigger_config);
	if (err != 0) {
		printk("Kick trigger configuration invalid: %d\n", err);
		return err;
	}

	err = sample_engine_start(&hit_event_queue, drum_sample_configs, drum_sample_config_count);
	if (err != 0) {
		printk("Sample engine start failed: %d\n", err);
		return err;
	}

#if CONFIG_DRUM_SAADC_STREAM
	err = adc_trigger_init(&kick_engine, &hit_event_queue);
	if (err != 0) {
		printk("SAADC initialization failed: %d\n", err);
		return err;
	}

	err = adc_trigger_start();
	if (err != 0) {
		printk("SAADC start failed: %d\n", err);
		return err;
	}

	printk("ADC ready: A0/AIN0 rate=%uS/s dma=%u samples\n", adc_trigger_get_sample_rate_hz(),
	       adc_trigger_get_dma_buffer_samples());
	printk("Kick baseline settling; trigger will arm without blocking acquisition\n");
#else
	printk("SAADC stream disabled; fake trigger test only\n");
#endif

	return 0;
}

#if CONFIG_DRUM_FAKE_TRIGGER_TEST
static void submit_fake_hit(void)
{
	static const uint8_t velocities[] = {20U, 40U, 70U, 100U, 127U};
	static size_t velocity_index;
	uint32_t timestamp_us = uptime_us_32();
	struct drum_hit_event event = {
		.drum = DRUM_ID_KICK,
		.accept_path = DRUM_HIT_ACCEPT_NORMAL,
		.peak = 1000U,
		.positive_peak = 1000U,
		.onset_delta = -100,
		.onset_history_delta = -300,
		.min_raw = 1000U,
		.max_raw = 1000U,
		.velocity = velocities[velocity_index],
		.timestamp_us = timestamp_us,
		.peak_complete_timestamp_us = timestamp_us,
	};

	if (k_msgq_put(&hit_event_queue, &event, K_NO_WAIT) != 0) {
		printk("Fake hit queue full\n");
	}

	velocity_index = (velocity_index + 1U) % ARRAY_SIZE(velocities);
}
#endif

static void report_health(const struct adc_trigger_stats *adc_stats,
			  const struct trigger_engine_diagnostics *trigger_diagnostics)
{
#if CONFIG_DRUM_ADC_TIMING_DIAGNOSTICS
	printk("ADC health samples=%u raw=%d baseline=%d state=%s dma_dt_us=%u..%u "
	       "queue_drop=%u saadc_err=%u wav_err=%u release=%u release_suppressed=%u "
	       "release_retrigger_accept=%u polarity_reject=%u slope_reject=%u "
	       "last_reject_delta1=%d last_reject_delta3=%d\n",
	       adc_stats->samples_acquired, adc_stats->latest_raw, adc_stats->baseline,
	       trigger_engine_state_name(adc_stats->trigger_state), adc_stats->dma_interval_min_us,
	       adc_stats->dma_interval_max_us, adc_stats->queue_overruns, adc_stats->saadc_errors,
	       wav_trigger_get_error_count(), trigger_diagnostics->release_detected_count,
	       trigger_diagnostics->release_suppressed_count,
	       trigger_diagnostics->release_retrigger_accept_count,
	       trigger_diagnostics->polarity_reject_count, trigger_diagnostics->slope_reject_count,
	       trigger_diagnostics->last_slope_reject_delta1,
	       trigger_diagnostics->last_slope_reject_history_delta);
#else
	ARG_UNUSED(adc_stats);
	ARG_UNUSED(trigger_diagnostics);
#endif
}

int main(void)
{
	struct adc_trigger_stats adc_stats = {0};
	struct trigger_engine_diagnostics trigger_diagnostics = {0};
	uint32_t last_health_report_ms = 0U;
	uint32_t last_fake_hit_ms = 0U;
	uint32_t last_release_sequence = 0U;
	uint32_t previous_queue_overruns = 0U;
	uint32_t previous_saadc_errors = 0U;
	uint32_t previous_wav_errors = 0U;
	bool armed_reported = false;
	bool led_ready;
	int err;

	printk("Trigger01 production kick engine\n");

	led_ready = initialize_status_led();
	initialize_wav_trigger();

	err = initialize_trigger_pipeline();
	if (err != 0) {
		return err;
	}

	while (true) {
		uint32_t now_ms;
		uint32_t wav_errors;

		k_msleep(SUPERVISOR_PERIOD_MS);
		now_ms = (uint32_t)k_uptime_get();

		if (led_ready) {
			(void)gpio_pin_toggle_dt(&status_led);
		}

		adc_trigger_get_stats(&adc_stats);
		trigger_engine_get_diagnostics(&kick_engine, &trigger_diagnostics);
		wav_errors = wav_trigger_get_error_count();

		if (!armed_reported && adc_stats.trigger_armed) {
			armed_reported = true;
			printk("Kick trigger armed: polarity=%s threshold=%u delta1=%u delta%u=%u "
			       "rearm=%u peak_window=%u samples\n",
			       trigger_engine_polarity_name(kick_trigger_config.polarity),
			       kick_trigger_config.trigger_threshold,
			       kick_trigger_config.onset_slope_threshold,
			       kick_trigger_config.slope_history_samples,
			       kick_trigger_config.onset_history_slope_threshold,
			       kick_trigger_config.rearm_threshold,
			       kick_trigger_config.peak_window_samples);
			printk("Kick release recovery: threshold=%u slope=%u retrigger_delta%u=%u "
			       "rearm=%u guard=%u stable=%u baseline_gate=%u samples\n",
			       kick_trigger_config.release_threshold,
			       kick_trigger_config.release_slope_threshold,
			       kick_trigger_config.slope_history_samples,
			       kick_trigger_config.retrigger_slope_threshold,
			       kick_trigger_config.release_rearm_threshold,
			       kick_trigger_config.release_guard_samples,
			       kick_trigger_config.release_stable_samples,
			       kick_trigger_config.baseline_track_threshold);
		}

#if CONFIG_DRUM_DIAGNOSTICS
		if (trigger_diagnostics.release_sequence != last_release_sequence) {
			last_release_sequence = trigger_diagnostics.release_sequence;
			printk("KICK RELEASE initial=%d release_peak=%u release_slope=%u "
			       "neg_ring=%u ring_slope1=%u ring_slope3=%u recovery_us=%u count=%u "
			       "suppressed=%u retrigger_accept=%u\n",
			       trigger_diagnostics.last_release_initial_centered,
			       trigger_diagnostics.last_release_peak,
			       trigger_diagnostics.last_release_max_slope,
			       trigger_diagnostics.last_release_ringback_peak,
			       trigger_diagnostics.last_release_ringback_slope1,
			       trigger_diagnostics.last_release_ringback_history_slope,
			       trigger_diagnostics.last_release_recovery_us,
			       trigger_diagnostics.release_detected_count,
			       trigger_diagnostics.release_suppressed_count,
			       trigger_diagnostics.release_retrigger_accept_count);
		}
#else
		ARG_UNUSED(last_release_sequence);
#endif

		if (adc_stats.queue_overruns != previous_queue_overruns) {
			printk("Hit-event queue overrun count=%u\n", adc_stats.queue_overruns);
			previous_queue_overruns = adc_stats.queue_overruns;
		}

		if (adc_stats.saadc_errors != previous_saadc_errors) {
			printk("SAADC stream error count=%u\n", adc_stats.saadc_errors);
			previous_saadc_errors = adc_stats.saadc_errors;
		}

		if (wav_errors != previous_wav_errors) {
			printk("WAV Trigger I2C error count=%u\n", wav_errors);
			previous_wav_errors = wav_errors;
		}

		if ((now_ms - last_health_report_ms) >= HEALTH_REPORT_PERIOD_MS) {
			last_health_report_ms = now_ms;
			report_health(&adc_stats, &trigger_diagnostics);
		}

#if CONFIG_DRUM_FAKE_TRIGGER_TEST
		if ((now_ms - last_fake_hit_ms) >= FAKE_HIT_PERIOD_MS) {
			last_fake_hit_ms = now_ms;
			submit_fake_hit();
		}
#else
		ARG_UNUSED(last_fake_hit_ms);
#endif
	}

	return 0;
}
