#include <zephyr/ztest.h>

#include "trigger_engine.h"

static const uint8_t three_point_curve[] = {1U, 64U, 127U};

static const struct drum_trigger_config test_config = {
	.drum = DRUM_ID_KICK,
	.polarity = TRIGGER_POLARITY_NEGATIVE,
	.trigger_threshold = 100U,
	.rearm_threshold = 20U,
	.onset_slope_threshold = 10U,
	.onset_history_slope_threshold = 10U,
	.retrigger_slope_threshold = 300U,
	.slope_history_samples = 3U,
	.release_threshold = 80U,
	.release_slope_threshold = 10U,
	.release_rearm_threshold = 20U,
	.release_guard_samples = 2U,
	.release_stable_samples = 2U,
	.baseline_track_threshold = 10U,
	.adc_clip_low_threshold = 5U,
	.adc_clip_high_threshold = 4090U,
	.peak_window_samples = 3U,
	.hard_mask_samples = 2U,
	.rearm_stable_samples = 2U,
	.baseline_settle_samples = 2U,
	.velocity_min_peak = 200U,
	.velocity_max_peak = 400U,
	.baseline_filter_shift = 2U,
	.velocity_curve = three_point_curve,
	.velocity_curve_size = ARRAY_SIZE(three_point_curve),
};

static void arm_at_baseline(struct trigger_engine *engine, int16_t baseline)
{
	struct drum_hit_event unused;

	zassert_false(trigger_engine_process_sample(engine, baseline, 0U, &unused));
	zassert_false(trigger_engine_process_sample(engine, baseline, 100U, &unused));
	zassert_false(trigger_engine_process_sample(engine, baseline, 200U, &unused));
	zassert_true(trigger_engine_is_armed(engine));
	zassert_equal(trigger_engine_get_baseline(engine), baseline);
}

ZTEST(trigger_engine, test_peak_velocity_and_baseline_freeze)
{
	struct trigger_engine engine;
	struct drum_hit_event event;

	zassert_ok(trigger_engine_init(&engine, &test_config));
	arm_at_baseline(&engine, 1000);

	/* Crossing sample is part of the three-sample, nonblocking peak window. */
	zassert_false(trigger_engine_process_sample(&engine, 880, 300U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_PEAK_CAPTURE);
	zassert_false(trigger_engine_process_sample(&engine, 700, 400U, &event));
	zassert_true(trigger_engine_process_sample(&engine, 1250, 500U, &event));

	zassert_equal(event.drum, DRUM_ID_KICK);
	zassert_equal(event.peak, 300U);
	zassert_equal(event.positive_peak, 250U);
	zassert_equal(event.negative_peak, 300U);
	zassert_equal(event.onset_delta, -120);
	zassert_equal(event.onset_history_delta, -120);
	zassert_equal(event.accept_path, DRUM_HIT_ACCEPT_NORMAL);
	zassert_equal(event.min_raw, 700U);
	zassert_equal(event.max_raw, 1250U);
	zassert_false(event.clip_low);
	zassert_false(event.clip_high);
	zassert_equal(event.velocity, 64U);
	zassert_equal(event.timestamp_us, 300U);
	zassert_equal(event.peak_complete_timestamp_us, 500U);
	zassert_equal(trigger_engine_get_baseline(&engine), 1000);
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_HARD_MASK);
}

ZTEST(trigger_engine, test_hard_mask_and_stable_rearm_reject_ringing)
{
	struct trigger_engine engine;
	struct drum_hit_event event;

	zassert_ok(trigger_engine_init(&engine, &test_config));
	arm_at_baseline(&engine, 1000);

	zassert_false(trigger_engine_process_sample(&engine, 850, 300U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 800, 400U, &event));
	zassert_true(trigger_engine_process_sample(&engine, 850, 500U, &event));

	/* Above-threshold ringing cannot create events during the hard mask. */
	zassert_false(trigger_engine_process_sample(&engine, 1300, 600U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 1300, 700U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_REARM);

	/* One quiet sample is not enough, and a ring resets the stable count. */
	zassert_false(trigger_engine_process_sample(&engine, 1005, 800U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 1080, 900U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 995, 1000U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_REARM);
	zassert_false(trigger_engine_process_sample(&engine, 1000, 1100U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_IDLE);

	/* Only after rearm may a new threshold crossing start another peak window. */
	zassert_false(trigger_engine_process_sample(&engine, 850, 1200U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_PEAK_CAPTURE);
}

ZTEST(trigger_engine, test_trigger_threshold_is_independent_of_velocity_minimum)
{
	struct trigger_engine engine;
	struct drum_hit_event event;

	zassert_ok(trigger_engine_init(&engine, &test_config));
	arm_at_baseline(&engine, 1000);

	zassert_false(trigger_engine_process_sample(&engine, 890, 300U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 850, 400U, &event));
	zassert_true(trigger_engine_process_sample(&engine, 860, 500U, &event));

	zassert_equal(event.peak, 150U);
	zassert_equal(event.velocity, 1U);
}

ZTEST(trigger_engine, test_release_ringback_is_suppressed_until_bipolar_settling)
{
	struct trigger_engine_diagnostics diagnostics;
	struct trigger_engine engine;
	struct drum_hit_event event;

	zassert_ok(trigger_engine_init(&engine, &test_config));
	arm_at_baseline(&engine, 1000);

	/* Positive release enters recovery but never emits a musical event. */
	zassert_false(trigger_engine_process_sample(&engine, 1120, 300U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_RELEASE_RECOVERY);
	zassert_false(trigger_engine_process_sample(&engine, 1150, 400U, &event));

	/* A negative threshold-crossing ringback is observed and suppressed. */
	zassert_false(trigger_engine_process_sample(&engine, 850, 500U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_RELEASE_RECOVERY);

	/* Recovery requires two consecutive bipolar-quiet samples after the guard. */
	zassert_false(trigger_engine_process_sample(&engine, 1005, 600U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 995, 700U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_IDLE);
	zassert_equal(trigger_engine_get_baseline(&engine), 1000);

	trigger_engine_get_diagnostics(&engine, &diagnostics);
	zassert_equal(diagnostics.release_detected_count, 1U);
	zassert_equal(diagnostics.release_suppressed_count, 1U);
	zassert_equal(diagnostics.polarity_reject_count, 1U);
	zassert_equal(diagnostics.release_sequence, 1U);
	zassert_equal(diagnostics.last_release_initial_centered, 120);
	zassert_equal(diagnostics.last_release_peak, 150U);
	zassert_equal(diagnostics.last_release_max_slope, 120U);
	zassert_equal(diagnostics.last_release_ringback_peak, 150U);
	zassert_equal(diagnostics.last_release_recovery_us, 400U);

	/* A real new negative onset is accepted immediately after safe recovery. */
	zassert_false(trigger_engine_process_sample(&engine, 880, 800U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_PEAK_CAPTURE);
}

ZTEST(trigger_engine, test_release_recovery_accepts_strong_retrigger_after_guard)
{
	struct trigger_engine_diagnostics diagnostics;
	struct trigger_engine engine;
	struct drum_hit_event event;

	zassert_ok(trigger_engine_init(&engine, &test_config));
	arm_at_baseline(&engine, 1000);

	zassert_false(trigger_engine_process_sample(&engine, 1120, 300U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 1150, 400U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_RELEASE_RECOVERY);

	/* Guard has elapsed; amplitude plus strong delta3 starts capture immediately. */
	zassert_false(trigger_engine_process_sample(&engine, 600, 500U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_PEAK_CAPTURE);
	zassert_false(trigger_engine_process_sample(&engine, 500, 600U, &event));
	zassert_true(trigger_engine_process_sample(&engine, 800, 700U, &event));
	zassert_equal(event.accept_path, DRUM_HIT_ACCEPT_RELEASE_RECOVERY);
	zassert_equal(event.onset_delta, -550);
	zassert_equal(event.onset_history_delta, -400);

	trigger_engine_get_diagnostics(&engine, &diagnostics);
	zassert_equal(diagnostics.release_retrigger_accept_count, 1U);
	zassert_equal(diagnostics.release_suppressed_count, 0U);
	zassert_equal(diagnostics.release_sequence, 1U);
}

ZTEST(trigger_engine, test_onset_slope_reject_and_polarity_selected_velocity_peak)
{
	struct trigger_engine_diagnostics diagnostics;
	struct trigger_engine engine;
	struct drum_hit_event event;

	zassert_ok(trigger_engine_init(&engine, &test_config));
	arm_at_baseline(&engine, 1000);

	/* A slow first crossing is diagnosed but cannot initiate peak capture. */
	zassert_false(trigger_engine_process_sample(&engine, 905, 300U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 899, 400U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_IDLE);
	trigger_engine_get_diagnostics(&engine, &diagnostics);
	zassert_equal(diagnostics.slope_reject_count, 1U);

	/* Reset below threshold, then supply a qualified negative onset. */
	zassert_false(trigger_engine_process_sample(&engine, 1000, 500U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 880, 600U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 750, 700U, &event));
	zassert_true(trigger_engine_process_sample(&engine, 1300, 800U, &event));

	/* Positive rebound is larger, but negative polarity selects the negative peak. */
	zassert_equal(event.positive_peak, 300U);
	zassert_equal(event.negative_peak, 250U);
	zassert_equal(event.peak, 250U);
	zassert_equal(event.velocity, 64U);
}

ZTEST(trigger_engine, test_normal_onset_requires_configured_history_slope)
{
	struct drum_trigger_config config = test_config;
	struct trigger_engine_diagnostics diagnostics;
	struct trigger_engine engine;
	struct drum_hit_event event;

	config.onset_slope_threshold = 1U;
	config.onset_history_slope_threshold = 110U;
	zassert_ok(trigger_engine_init(&engine, &config));
	arm_at_baseline(&engine, 1000);

	zassert_false(trigger_engine_process_sample(&engine, 970, 300U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 940, 400U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 900, 500U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_IDLE);

	trigger_engine_get_diagnostics(&engine, &diagnostics);
	zassert_equal(diagnostics.slope_reject_count, 1U);
}

ZTEST(trigger_engine, test_armed_baseline_tracks_only_inside_quiet_gate)
{
	struct trigger_engine engine;
	struct drum_hit_event event;

	zassert_ok(trigger_engine_init(&engine, &test_config));
	arm_at_baseline(&engine, 1000);

	/* Sub-release mechanical motion outside the quiet gate cannot move baseline. */
	zassert_false(trigger_engine_process_sample(&engine, 1050, 300U, &event));
	zassert_equal(trigger_engine_get_state(&engine), TRIGGER_STATE_IDLE);
	zassert_equal(trigger_engine_get_baseline(&engine), 1000);

	/* Quiet drift is still followed by the configured IIR. */
	zassert_false(trigger_engine_process_sample(&engine, 1004, 400U, &event));
	zassert_equal(trigger_engine_get_baseline(&engine), 1001);
}

ZTEST(trigger_engine, test_peak_window_reports_adc_headroom_and_clipping)
{
	struct trigger_engine engine;
	struct drum_hit_event event;

	zassert_ok(trigger_engine_init(&engine, &test_config));
	arm_at_baseline(&engine, 2000);

	zassert_false(trigger_engine_process_sample(&engine, 1800, 300U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 4, 400U, &event));
	zassert_true(trigger_engine_process_sample(&engine, 4092, 500U, &event));
	zassert_equal(event.min_raw, 4U);
	zassert_equal(event.max_raw, 4092U);
	zassert_true(event.clip_low);
	zassert_true(event.clip_high);
}

ZTEST(trigger_engine, test_positive_and_both_polarities_select_the_configured_peak)
{
	struct drum_trigger_config config = test_config;
	struct trigger_engine engine;
	struct drum_hit_event event;

	config.polarity = TRIGGER_POLARITY_POSITIVE;
	zassert_ok(trigger_engine_init(&engine, &config));
	arm_at_baseline(&engine, 1000);
	zassert_false(trigger_engine_process_sample(&engine, 1120, 300U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 700, 400U, &event));
	zassert_true(trigger_engine_process_sample(&engine, 1250, 500U, &event));
	zassert_equal(event.positive_peak, 250U);
	zassert_equal(event.negative_peak, 300U);
	zassert_equal(event.peak, 250U);

	config.polarity = TRIGGER_POLARITY_BOTH;
	zassert_ok(trigger_engine_init(&engine, &config));
	arm_at_baseline(&engine, 1000);
	zassert_false(trigger_engine_process_sample(&engine, 1120, 300U, &event));
	zassert_false(trigger_engine_process_sample(&engine, 700, 400U, &event));
	zassert_true(trigger_engine_process_sample(&engine, 1250, 500U, &event));
	zassert_equal(event.peak, 300U);
}

ZTEST(trigger_engine, test_invalid_hysteresis_is_rejected)
{
	struct trigger_engine engine;
	struct drum_trigger_config invalid = test_config;

	invalid.rearm_threshold = invalid.trigger_threshold;
	zassert_equal(trigger_engine_init(&engine, &invalid), -EINVAL);
}

ZTEST_SUITE(trigger_engine, NULL, NULL, NULL, NULL, NULL);
