#ifndef STORAGE_H
#define STORAGE_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

void storage_init(void);
void storage_start(uint8_t channel, uint16_t charge_current_ma);
void storage_stop(uint8_t channel);
void storage_task(void);
storage_state_t storage_get_state(uint8_t channel);
bool storage_is_active(uint8_t channel);
bool storage_is_done(uint8_t channel);

/* Advance storage state machine - called from scheduler with telemetry */
void storage_advance(uint8_t channel, uint16_t v_bat_mv);

#endif /* STORAGE_H */
