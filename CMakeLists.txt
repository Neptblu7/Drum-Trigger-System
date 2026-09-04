cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Trigger01)

target_sources(app PRIVATE
	src/adc_trigger.c
	src/drum_config.c
	src/main.c
	src/sample_engine.c
	src/trigger_engine.c
	src/wav_trigger.c
)
