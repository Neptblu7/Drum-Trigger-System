#include "drum_config.h"

#include <zephyr/sys/util.h>

/*
 * 128-entry velocity curve generated as:
 *   velocity = round(1 + 126 * pow(index / 127, 0.75))
 *
 * The lookup occurs once per completed hit, so the sample path uses no float.
 * Replace this table to audition another response curve without touching the
 * detector or sample-selection code.
 */
static const uint8_t kick_velocity_curve_gamma_075[] = {
	1U,   4U,   7U,   9U,   10U,  12U,  14U,  15U,  17U,  18U,  20U,  21U,  22U,  24U,  25U,
	26U,  28U,  29U,  30U,  31U,  32U,  34U,  35U,  36U,  37U,  38U,  39U,  40U,  42U,  43U,
	44U,  45U,  46U,  47U,  48U,  49U,  50U,  51U,  52U,  53U,  54U,  55U,  56U,  57U,  58U,
	59U,  60U,  61U,  62U,  63U,  64U,  65U,  65U,  66U,  67U,  68U,  69U,  70U,  71U,  72U,
	73U,  74U,  75U,  75U,  76U,  77U,  78U,  79U,  80U,  81U,  82U,  82U,  83U,  84U,  85U,
	86U,  87U,  88U,  88U,  89U,  90U,  91U,  92U,  93U,  93U,  94U,  95U,  96U,  97U,  98U,
	98U,  99U,  100U, 101U, 102U, 102U, 103U, 104U, 105U, 106U, 106U, 107U, 108U, 109U, 109U,
	110U, 111U, 112U, 113U, 113U, 114U, 115U, 116U, 116U, 117U, 118U, 119U, 119U, 120U, 121U,
	122U, 123U, 123U, 124U, 125U, 126U, 126U, 127U,
};

const struct drum_trigger_config kick_trigger_config = {
	.drum = DRUM_ID_KICK,
	.polarity = TRIGGER_POLARITY_NEGATIVE,
	.trigger_threshold = 700U,
	.rearm_threshold = 300U,
	/* Initial separation derived from real kick/ringback RTT measurements. */
	.onset_slope_threshold = 80U,
	.onset_history_slope_threshold = 180U,
	.retrigger_slope_threshold = 300U,
	.slope_history_samples = 3U, /* delta3 spans 300 us at 10 kS/s */
	.release_threshold = 500U,
	.release_slope_threshold = 10U,
	.release_rearm_threshold = 300U,
	.release_guard_samples = 80U,  /* 8.0 ms at 10 kS/s */
	.release_stable_samples = 15U, /* 1.5 ms at 10 kS/s */
	.baseline_track_threshold = 100U,
	.adc_clip_low_threshold = 8U,
	.adc_clip_high_threshold = 4087U,
	.peak_window_samples = 20U,       /* 2.0 ms at 10 kS/s */
	.hard_mask_samples = 150U,        /* 15 ms at 10 kS/s */
	.rearm_stable_samples = 15U,      /* 1.5 ms at 10 kS/s */
	.baseline_settle_samples = 1000U, /* 100 ms at 10 kS/s */
	.velocity_min_peak = 800U,
	.velocity_max_peak = 1800U,
	.baseline_filter_shift = 11U, /* quiet baseline alpha = 1/2048 */
	.velocity_curve = kick_velocity_curve_gamma_075,
	.velocity_curve_size = ARRAY_SIZE(kick_velocity_curve_gamma_075),
};

/* Track 1 (0001.wav) is the complete initial kick library. */
static const uint16_t kick_all_tracks[] = {1U};

static const struct velocity_layer kick_layers[] = {
	{
		.velocity_min = 1U,
		.velocity_max = 127U,
		.tracks = kick_all_tracks,
		.track_count = ARRAY_SIZE(kick_all_tracks),
		.gain_db = 0,
	},
};

const struct drum_sample_config drum_sample_configs[] = {
	{
		.drum = DRUM_ID_KICK,
		.layers = kick_layers,
		.layer_count = ARRAY_SIZE(kick_layers),
		.velocity_gain =
			{
				.enabled = true,
				.scope = VELOCITY_GAIN_FULL_RANGE,
				.minimum_db = -18,
				.maximum_db = 0,
			},
	},
};

const size_t drum_sample_config_count = ARRAY_SIZE(drum_sample_configs);
