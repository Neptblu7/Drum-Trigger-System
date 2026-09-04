#ifndef DRUM_CONFIG_H_
#define DRUM_CONFIG_H_

#include <stddef.h>

#include "sample_engine.h"
#include "trigger_engine.h"

extern const struct drum_trigger_config kick_trigger_config;
extern const struct drum_sample_config drum_sample_configs[];
extern const size_t drum_sample_config_count;

#endif /* DRUM_CONFIG_H_ */
