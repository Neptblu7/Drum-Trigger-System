# Trigger01 kick drum engine

Trigger01 is native Zephyr / nRF Connect SDK C firmware for a Seeed Studio XIAO
nRF52840 and a Robertsonics/SparkFun Qwiic WAV Trigger Pro. The MCU detects and
classifies piezo hits; the WAV Trigger Pro is used as a polyphonic playback
engine. No MIDI-note-to-track preset mapping is required by the hit path.

## Current milestone

- One kick input on XIAO A0 / P0.02 / SAADC AIN0
- 10 kS/s hardware-timed SAADC acquisition
- Configurable delta1/delta3 onset qualification and polarity-selected velocity
- Nonblocking peak capture, hard mask, release recovery, and hysteretic stable rearm
- Fixed-point/LUT velocity response equivalent to gamma 0.75
- Optional per-drum integer velocity-to-gain scaling
- Data-driven velocity layers and per-layer sequential round robin
- Direct polyphonic Track Play over I2C at address `0x13`
- Initial all-velocity sample library: Track 1 (`0001.wav`)
- ISR-safe hit queue and dedicated I2C playback thread
- RTT startup, per-hit latency, and health diagnostics

## Architecture

| Module | Responsibility | Execution context |
| --- | --- | --- |
| `main.c` | Initialization and low-rate health supervision | Main thread |
| `adc_trigger.c` | SAADC internal timer, EasyDMA buffers, queue submission | SAADC ISR |
| `trigger_engine.c` | Polarity/slope classification, baseline, peak, recovery, velocity | Called in SAADC ISR |
| `sample_engine.c` | Layer/round-robin selection, velocity gain, playback | Priority-1 playback thread |
| `wav_trigger.c` | WAV Trigger Pro I2C packet protocol | Playback/main thread only |
| `drum_config.c` | Kick tuning, velocity curve, layers, track tables | Const data |

The real-time path is:

```text
SAADC internal timer (100 us)
  -> EasyDMA ping-pong buffer (8 samples)
  -> integer trigger state machine
  -> k_msgq (K_NO_WAIT)
  -> sample playback thread
  -> velocity layer + per-layer round robin
  -> direct polyphonic Track Play I2C command
```

The SAADC callback never performs I2C or `printk`, and there are no sleeps,
busy waits, floating-point operations, or allocations in the acquisition/hit
path.

## Build and debug

The production configuration is in `prj.conf`:

```powershell
west build -b xiao_ble/nrf52840 -d build --pristine
west flash -d build --runner jlink
```

The existing VS Code nRF Connect launch configuration remains valid for J-Link
debugging. RTT is the selected console backend.

To exercise the queue, sample engine, and direct Track 1 packet without a piezo
hit, add the supplied configuration fragment:

```powershell
west build -b xiao_ble/nrf52840 -d build --pristine -- -DEXTRA_CONF_FILE=config/fake_trigger.conf
```

The fragment disables SAADC so this is an isolated playback-path test. A fake
event is generated every two seconds and uses the same queue, worker, sample
selection, and I2C driver as a real hit. It does not stop an already-playing
voice.

## Expected RTT output

Startup should include messages equivalent to:

```text
I2C ready on XIAO D4/D5
WAV Trigger Pro connected; tracks=1
ADC ready: A0/AIN0 rate=10000S/s dma=8 samples
Kick trigger armed: polarity=NEGATIVE threshold=700 delta1=80 delta3=180 rearm=300 peak_window=20 samples
Kick release recovery: threshold=500 slope=10 retrigger_delta3=300 rearm=300 guard=80 stable=15 baseline_gate=100 samples
```

A hit line includes the acquisition-to-command breakdown:

```text
KICK peak=1482 pos=420 neg=1482 vel=91 delta1=-220 delta3=-680 path=NORMAL min_raw=388 max_raw=2290 clip_low=0 clip_high=0 layer=0 rr=0 track=1 gain_db=-5 vel_gain_db=-5 dt=143ms capture_us=1900 dispatch_us=450 latency_us=2350 i2c_us=1100 err=0
```

A recognized release prints from the low-rate supervisor after recovery, never
from the SAADC callback:

```text
KICK RELEASE initial=620 release_peak=1280 release_slope=740 neg_ring=810 recovery_us=47000 count=1 suppressed=1 retrigger_accept=0
```

`capture_us` is the intentional peak window, `dispatch_us` covers the remainder
of the DMA batch plus scheduling, `latency_us` ends immediately before the I2C
call, and `i2c_us` measures the command transaction. WAV Trigger Pro SD/audio
startup latency cannot be measured by the MCU without an external audio timing
signal.

The five-second health line should show `samples` increasing by approximately
50,000 and a nominal DMA callback interval of 800 us. Callback jitter does not
change the hardware-timed 100 us sample spacing.

## Kick tuning

All first-milestone tuning is together in `kick_trigger_config` in
`src/drum_config.c`:

- `trigger_threshold = 700`
- `rearm_threshold = 300`
- `onset_slope_threshold = 80` counts/1 sample
- `slope_history_samples = 3` (300 us)
- `onset_history_slope_threshold = 180` counts/3 samples
- `retrigger_slope_threshold = 300` counts/3 samples
- `release_threshold = 500`
- `release_slope_threshold = 10` counts/sample
- `release_rearm_threshold = 300`
- `release_guard_samples = 80` (8.0 ms)
- `release_stable_samples = 15` (1.5 ms)
- `baseline_track_threshold = 100`
- ADC clipping diagnostics at raw `<= 8` and `>= 4087`
- `peak_window_samples = 20` (2.0 ms at 10 kS/s)
- `hard_mask_samples = 150` (15 ms)
- `rearm_stable_samples = 15` (1.5 ms)
- `velocity_min_peak = 800`
- `velocity_max_peak = 1800`
- velocity gain maps velocity 1 to -18 dB and velocity 127 to 0 dB
- quiet-signal baseline filter alpha = 1/2048

The trigger threshold and minimum velocity peak are intentionally separate.
Hits with peaks from 700 through 800 are accepted but map to velocity 1.
For the kick's negative polarity, `peak` and velocity use the negative peak;
positive and absolute peaks remain available in diagnostics. After startup,
the baseline tracks only while IDLE and within the 100-count quiet gate. It is
frozen through significant transients, peak capture, hard mask, rearm, and
release recovery.

A positive release must satisfy both its amplitude and counts-per-sample slope
criteria. It then enters `RELEASE_RECOVERY`, where slow negative ringback cannot
generate a hit. After the 8 ms guard, amplitude 700 plus a negative delta3 of at
least 300 starts peak capture immediately, even if the waveform has not fully
settled. Otherwise recovery still completes after 15 consecutive samples inside
the bipolar 300-count window.

Tune in this order on the actual drum:

1. Confirm the idle raw value is stable near the expected 1.65 V bias and does
   not approach ADC rails during hard strikes.
2. Raise `trigger_threshold` until vibration/noise no longer creates candidates,
   then confirm the weakest intentional hit still crosses it.
3. Set `rearm_threshold` above idle noise but well below the trigger threshold.
4. Compare `delta1` and `delta3` from weak intentional kicks against rejected
   ringback before changing either normal slope threshold.
5. Use the `KICK RELEASE` line to tune release amplitude/slope and recovery
   thresholds without lengthening the normal hard mask.
6. Record weak and hard intentional negative peaks, then set `velocity_min_peak` and
   `velocity_max_peak` from those observations.
7. Replace the velocity LUT if a different gamma/response is desired.

For a 20 kS/s experiment, set `CONFIG_DRUM_ADC_SAMPLE_PERIOD_US=50` and double
all duration-based sample counts to retain the same milliseconds (peak 40,
mask 300, stable rearm 30, release guard 160, release stable 30, baseline settle
2000). Use six history samples to retain the 300 us delta window. Slope
thresholds also require fresh tuning when the sample period changes. Confirm
the analog front end settles within the retained 20 us SAADC acquisition time
before using 20 kS/s in production.

## Adding sample layers and round robin

Only configuration data needs to change. For example:

```c
static const uint16_t kick_soft[] = {10U, 11U, 12U};
static const uint16_t kick_medium[] = {20U, 21U, 22U};

static const struct velocity_layer kick_layers[] = {
    {1U, 30U, kick_soft, ARRAY_SIZE(kick_soft), 0},
    {31U, 127U, kick_medium, ARRAY_SIZE(kick_medium), 0},
};
```

Layer ranges must cover 1 through 127 without gaps. Each layer owns an
independent next-track index, so a multi-track layer cycles deterministically
and cannot select the same track on consecutive hits until its list wraps.
Velocity gain is configured per drum. `VELOCITY_GAIN_FULL_RANGE` currently
provides -18 to 0 dB across velocities 1 through 127. With true sample layers,
switch to `VELOCITY_GAIN_WITHIN_LAYER` and use a smaller gain range to retain
subtle dynamics inside each layer.

## Tests and hardware validation

Detector tests live under `tests/trigger_engine` and cover polarity-selected
peaks, delta1/delta3 onset slope, release/ringback suppression, strong recovery
retrigger acceptance, ADC clipping, baseline freeze, immediate post-recovery
hits, hard mask, rearm stability, and velocity mapping:

```powershell
west twister -T tests/trigger_engine -p qemu_cortex_m3
```

The automated build cannot prove analog settling, real sample cadence, false
double-trigger performance, I2C ACK behavior, or acoustic latency. Validate
those on the XIAO/WAV hardware with J-Link RTT. In particular, check that the
protected front end's source impedance is compatible with the configured 20 us
acquisition time and that hard hits remain inside the 0 to 3.6 V equivalent ADC
range.
