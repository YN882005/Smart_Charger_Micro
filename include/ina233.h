#ifndef INA233_H
#define INA233_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

#define INA233_CHANNEL_COUNT ACTIVE_CHANNELS

typedef struct {
	uint16_t bus_voltage_raw;
	uint16_t current_raw;
	float bus_voltage_v;
	float current_ma;
} ina233_raw_sample_t;

typedef struct {
	uint8_t address;
	float current_lsb;
	bool initialized;
} ina233_device_t;

/* Energy accumulator result from READ_EIN (6-byte block) */
typedef struct {
	uint32_t raw_power_sum;
	uint32_t sample_count;
	uint16_t accumulator_16bit;
	uint8_t rollover_count;
	float avg_power_w;
	float energy_wh;
	float v_avg;
	uint32_t capacity_mah;
} ina233_energy_t;

bool ina233_device_init(ina233_device_t *device, uint8_t address,
	float r_shunt, float i_max);
bool ina233_read_sample(const ina233_device_t *device,
	ina233_raw_sample_t *sample);
bool ina233_read_ein(const ina233_device_t *device, ina233_energy_t *energy);
bool ina233_reset_ein(const ina233_device_t *device);

#endif /* INA233_H */
