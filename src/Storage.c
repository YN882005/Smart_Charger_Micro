#include "Storage.h"

#include <stdint.h>
#include <stdlib.h>

#include "config.h"
#include "Charging.h"
#include "Discharging.h"
#include "Buzzer.h"

static storage_state_t stg_state[TOTAL_CHANNELS] = {
	STG_STATE_IDLE, STG_STATE_IDLE, STG_STATE_IDLE, STG_STATE_IDLE
};
static uint16_t stg_charge_current_ma[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static uint32_t stg_settle_start_millis[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};

void storage_init(void)
{
	uint8_t ch;
	for (ch = 0u; ch < TOTAL_CHANNELS; ++ch) {
		stg_state[ch] = STG_STATE_IDLE;
		stg_charge_current_ma[ch] = 0u;
		stg_settle_start_millis[ch] = 0u;
	}
}

void storage_start(uint8_t channel, uint16_t charge_current_ma)
{
	if (channel >= TOTAL_CHANNELS) {
		return;
	}

	stg_charge_current_ma[channel] = charge_current_ma;
	stg_state[channel] = STG_STATE_ASSESS;
	stg_settle_start_millis[channel] = sys_millis();

	/* Ensure all outputs are OFF during 200 ms settling */
	charging_stop(channel);
	discharging_stop(channel);
}

void storage_stop(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return;
	}

	charging_stop(channel);
	discharging_stop(channel);
	stg_state[channel] = STG_STATE_IDLE;
	stg_settle_start_millis[channel] = 0u;
}

void storage_task(void)
{
	/* Advanced via storage_advance */
}

storage_state_t storage_get_state(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return STG_STATE_IDLE;
	}
	return stg_state[channel];
}

bool storage_is_active(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return false;
	}
	return stg_state[channel] == STG_STATE_ASSESS
		|| stg_state[channel] == STG_STATE_SUB_CHARGE
		|| stg_state[channel] == STG_STATE_SUB_DISCHARGE;
}

bool storage_is_done(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return false;
	}
	return stg_state[channel] == STG_STATE_DONE;
}

void storage_advance(uint8_t channel, uint16_t v_bat_mv)
{
	if (channel >= TOTAL_CHANNELS) {
		return;
	}

	if (stg_state[channel] == STG_STATE_IDLE || stg_state[channel] == STG_STATE_DONE) {
		return;
	}

	uint16_t v_storage = (channel == 0u) ? CH1_STORAGE_MV : CH2_STORAGE_MV;

	switch (stg_state[channel]) {
	case STG_STATE_ASSESS:
		/* 200 ms settling with FETs OFF */
		if (sys_millis() - stg_settle_start_millis[channel] >= 200u) {
			int32_t diff = (int32_t)v_bat_mv - (int32_t)v_storage;
			if (diff < 0) diff = -diff;

			if (diff <= (int32_t)STORAGE_DEADBAND_MV) {
				/* Case A: Already at optimal storage voltage -> finish immediately */
				charging_stop(channel);
				discharging_stop(channel);
				stg_state[channel] = STG_STATE_DONE;
				Buzzer_PlayCompleteMelody();
			} else if (v_bat_mv < (v_storage - STORAGE_DEADBAND_MV)) {
				/* Case B: Under storage voltage -> charge smoothly up to V_STORAGE */
				stg_state[channel] = STG_STATE_SUB_CHARGE;
				charging_start(channel, stg_charge_current_ma[channel], v_storage);
			} else {
				/* Case C: Over storage voltage -> discharge down to V_STORAGE */
				stg_state[channel] = STG_STATE_SUB_DISCHARGE;
				discharging_start(channel, v_storage);
			}
		}
		break;

	case STG_STATE_SUB_CHARGE:
		if (charging_is_done(channel)) {
			charging_stop(channel);
			stg_state[channel] = STG_STATE_DONE;
			Buzzer_PlayCompleteMelody();
		}
		break;

	case STG_STATE_SUB_DISCHARGE:
		if (discharging_is_done(channel)) {
			discharging_stop(channel);
			stg_state[channel] = STG_STATE_DONE;
			Buzzer_PlayCompleteMelody();
		}
		break;

	default:
		break;
	}
}
