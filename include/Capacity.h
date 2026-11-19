#ifndef CAPACITY_H
#define CAPACITY_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "ina233.h"

void capacity_init(void);
void capacity_start(uint8_t channel, uint16_t charge_current_ma);
void capacity_stop(uint8_t channel);
void capacity_task(void);
capacity_state_t capacity_get_state(uint8_t channel);
bool capacity_is_active(uint8_t channel);
bool capacity_is_done(uint8_t channel);

bool capacity_menu_enabled(uint8_t channel);
void capacity_menu_enable(uint8_t channel);
void capacity_menu_disable(uint8_t channel);

bool capacity_test_complete(uint8_t channel);
uint32_t capacity_measured_mah(uint8_t channel);
float capacity_measured_energy_wh(uint8_t channel);
uint32_t capacity_get_stage2_elapsed_ms(uint8_t channel);

void capacity_set_device(uint8_t channel, ina233_device_t *device);

/* Advance capacity state machine - called from scheduler with telemetry */
void capacity_advance(uint8_t channel, uint16_t v_bat_mv, int16_t i_bat_ma,
	ina233_device_t *device);

#endif /* CAPACITY_H */
