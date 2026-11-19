#ifndef DISCHARGING_H
#define DISCHARGING_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

#define DSG_CUTOFF_MAX_ITEMS 12u

void discharging_init(void);
void discharging_start(uint8_t channel, uint16_t cutoff_voltage_mv);
void discharging_stop(uint8_t channel);
void discharging_task(void);
discharging_state_t discharging_get_state(uint8_t channel);
bool discharging_is_active(uint8_t channel);
bool discharging_is_done(uint8_t channel);
bool discharging_outputs_are_safe(void);

/* Control step: call every 50 ms from scheduler. Returns true if discharge continues. */
bool discharging_control_step(uint8_t channel, uint16_t v_bat_mv);
uint16_t discharging_get_cutoff_menu_item(uint8_t channel, uint8_t index);
uint8_t discharging_get_cutoff_menu_count(void);

#endif /* DISCHARGING_H */
