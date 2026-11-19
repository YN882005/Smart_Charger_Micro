#ifndef CHARGING_H
#define CHARGING_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

void charging_init(void);
void charging_start(uint8_t channel, uint16_t current_ma, uint16_t target_voltage_mv);
void charging_stop(uint8_t channel);
void charging_task(void);
charging_state_t charging_get_state(uint8_t channel);
bool charging_is_active(uint8_t channel);
bool charging_is_done(uint8_t channel);
bool charging_outputs_are_safe(void);

/* Control step: call every 20 ms from scheduler. Returns PWM value (0..PWM_MAX_DUTY). */
uint8_t charging_control_step(uint8_t channel, uint16_t v_bat_mv, int16_t i_bat_ma);

#endif /* CHARGING_H */
