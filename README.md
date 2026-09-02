# Drum-Trigger-System

# Hardware / platform

MCU:

- Seeed Studio XIAO nRF52840
- Zephyr / nRF Connect SDK
- Board target: `xiao_ble/nrf52840`
- Development in VS Code
- SEGGER J-Link Base available for SWD/debug/RTT

Current analog trigger input:

- Piezo/PZT trigger front end
- ADC input is XIAO `A0 / D0`
- nRF52840 pin: `P0.02`
- SAADC input: `AIN0`
- Piezo is NOT connected directly to the MCU
         It goes through a protected, biased analog front end
- ADC node is nominally biased around 1.65 V
- ADC is currently configured approximately as:
    - 12-bit
    - internal 0.6 V reference
    - gain 1/6
    - approximately 3.6 V full-scale equivalent

WAV playback hardware:
- SparkFun / Robertsonics Qwiic WAV Trigger Pro
- Connected directly by I2C
- XIAO D4 = SDA
- XIAO D5 = SCL
- Shared ground
- WAV Trigger Pro I2C address currently used: `0x13`
