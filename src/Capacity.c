#include "Capacity.h"

#include <stdint.h>

#include "config.h"
#include "Charging.h"
#include "Discharging.h"
#include "ina233.h"
#include "Buzzer.h"

/* Internal states for 2-stage sequence */
enum {
	CAP_SUBSTATE_NONE = 0,
	CAP_SUBSTATE_STAGE1_DSG,
	CAP_SUBSTATE_OCV_PAUSE,
	CAP_SUBSTATE_STAGE2_CHG,
	CAP_SUBSTATE_COMPLETE
};

static uint8_t  cap_substate[TOTAL_CHANNELS] = {0, 0, 0, 0};
static uint16_t cap_charge_current_ma[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static bool     cap_menu_enabled_flag[TOTAL_CHANNELS] = {false, false, false, false};
static bool     cap_complete_flag[TOTAL_CHANNELS] = {false, false, false, false};
static uint32_t cap_measured_mah_val[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static float    cap_measured_wh_val[TOTAL_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};

static uint32_t cap_pause_start_ms[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static uint32_t cap_stage2_start_ms[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static uint32_t cap_stage2_elapsed_ms_val[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};

static uint32_t cap_vbus_sum_mv[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static uint16_t cap_vbus_sample_count[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static uint32_t cap_last_ein_poll_ms[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static ina233_energy_t cap_last_energy[TOTAL_CHANNELS];
static ina233_device_t *cap_device[TOTAL_CHANNELS] = {0, 0, 0, 0};

void capacity_init(void)
{
	uint8_t ch;
	for (ch = 0u; ch < TOTAL_CHANNELS; ++ch) {
		cap_substate[ch] = CAP_SUBSTATE_NONE;
		cap_charge_current_ma[ch] = 0u;
		cap_menu_enabled_flag[ch] = false;
		cap_complete_flag[ch] = false;
		cap_measured_mah_val[ch] = 0u;
		cap_measured_wh_val[ch] = 0.0f;
		cap_pause_start_ms[ch] = 0u;
		cap_stage2_start_ms[ch] = 0u;
		cap_stage2_elapsed_ms_val[ch] = 0u;
		cap_vbus_sum_mv[ch] = 0u;
		cap_vbus_sample_count[ch] = 0u;
		cap_last_ein_poll_ms[ch] = 0u;
		cap_device[ch] = 0;
	}
}

void capacity_start(uint8_t channel, uint16_t charge_current_ma)
{
	if (channel >= TOTAL_CHANNELS) {
		return;
	}

	cap_charge_current_ma[channel] = charge_current_ma;
	cap_substate[channel] = CAP_SUBSTATE_STAGE1_DSG;
	cap_complete_flag[channel] = false;
	cap_measured_mah_val[channel] = 0u;
	cap_measured_wh_val[channel] = 0.0f;
	cap_stage2_start_ms[channel] = 0u;
	cap_stage2_elapsed_ms_val[channel] = 0u;
	cap_vbus_sum_mv[channel] = 0u;
	cap_vbus_sample_count[channel] = 0u;

	/* Stage 1: Pre-Discharge to chemistry minimum cutoff */
	charging_stop(channel);
	discharging_start(channel, (channel == 0u) ? CH1_SAFE_CUTOFF_MV : CH2_SAFE_CUTOFF_MV);
}

void capacity_stop(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return;
	}

	charging_stop(channel);
	discharging_stop(channel);
	cap_substate[channel] = CAP_SUBSTATE_NONE;
}

void capacity_task(void)
{
	/* Advanced via capacity_advance */
}

capacity_state_t capacity_get_state(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return CAP_STATE_IDLE;
	}
	if (cap_substate[channel] == CAP_SUBSTATE_STAGE1_DSG
		|| cap_substate[channel] == CAP_SUBSTATE_OCV_PAUSE) {
		return CAP_STATE_STAGE1_DISCHARGE;
	}
	if (cap_substate[channel] == CAP_SUBSTATE_STAGE2_CHG) {
		return CAP_STATE_STAGE2_CHARGE;
	}
	if (cap_substate[channel] == CAP_SUBSTATE_COMPLETE) {
		return CAP_STATE_DONE;
	}
	return CAP_STATE_IDLE;
}

bool capacity_is_active(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return false;
	}
	return cap_substate[channel] != CAP_SUBSTATE_NONE
		&& cap_substate[channel] != CAP_SUBSTATE_COMPLETE;
}

bool capacity_is_done(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return false;
	}
	return cap_substate[channel] == CAP_SUBSTATE_COMPLETE;
}

bool capacity_test_complete(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return false;
	}
	return cap_complete_flag[channel];
}

bool capacity_menu_enabled(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return false;
	}
	return cap_menu_enabled_flag[channel];
}

void capacity_menu_enable(uint8_t channel)
{
	if (channel < TOTAL_CHANNELS) {
		cap_menu_enabled_flag[channel] = true;
	}
}

void capacity_menu_disable(uint8_t channel)
{
	if (channel < TOTAL_CHANNELS) {
		cap_menu_enabled_flag[channel] = false;
	}
}

uint32_t capacity_measured_mah(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return 0u;
	}
	return cap_measured_mah_val[channel];
}

float capacity_measured_energy_wh(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return 0.0f;
	}
	return cap_measured_wh_val[channel];
}

uint32_t capacity_get_stage2_elapsed_ms(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return 0u;
	}
	if (cap_substate[channel] == CAP_SUBSTATE_STAGE2_CHG && cap_stage2_start_ms[channel] > 0u) {
		return sys_millis() - cap_stage2_start_ms[channel];
	}
	return cap_stage2_elapsed_ms_val[channel];
}

void capacity_set_device(uint8_t channel, ina233_device_t *device)
{
	if (channel < TOTAL_CHANNELS) {
		cap_device[channel] = device;
	}
}

void capacity_advance(uint8_t channel, uint16_t v_bat_mv, int16_t i_bat_ma,
	ina233_device_t *device)
{
	(void)i_bat_ma;

	if (channel >= TOTAL_CHANNELS) {
		return;
	}

	cap_device[channel] = device;
	uint32_t now = sys_millis();

	switch (cap_substate[channel]) {
	case CAP_SUBSTATE_STAGE1_DSG:
		if (discharging_is_done(channel)) {
			/* Stage 1 cutoff reached -> turn NMOS OFF and pause 2.0 s for OCV relaxation */
			discharging_stop(channel);
			cap_substate[channel] = CAP_SUBSTATE_OCV_PAUSE;
			cap_pause_start_ms[channel] = now;
		}
		break;

	case CAP_SUBSTATE_OCV_PAUSE:
		if (now - cap_pause_start_ms[channel] >= 2000u) {
			/* OCV pause complete -> zero accumulator at t=0 and start Stage 2 charge */
			if (device != 0) {
				ina233_reset_ein(device);
			}
			cap_stage2_start_ms[channel] = now;
			cap_vbus_sum_mv[channel] = 0u;
			cap_vbus_sample_count[channel] = 0u;
			cap_last_ein_poll_ms[channel] = now;

			charging_start(channel, cap_charge_current_ma[channel],
				(channel == 0u) ? CH1_FULL_CHARGE_MV : CH2_FULL_CHARGE_MV);

			cap_substate[channel] = CAP_SUBSTATE_STAGE2_CHG;
		}
		break;

	case CAP_SUBSTATE_STAGE2_CHG: {
		/* Accumulate VBUS samples */
		if (cap_vbus_sample_count[channel] < 65535u) {
			cap_vbus_sum_mv[channel] += v_bat_mv;
			cap_vbus_sample_count[channel]++;
		}

		/* Poll READ_EIN periodically (every 50 ms) to avoid accumulator overflow */
		if (device != 0 && (now - cap_last_ein_poll_ms[channel] >= 50u)) {
			cap_last_ein_poll_ms[channel] = now;
			ina233_read_ein(device, &cap_last_energy[channel]);
		}

		uint32_t elapsed_s = (now - cap_stage2_start_ms[channel]) / 1000u;
		cap_stage2_elapsed_ms_val[channel] = now - cap_stage2_start_ms[channel];

		/* Energy_Wh = Avg_Power_W * (Elapsed_Seconds / 3600.0f) */
		cap_measured_wh_val[channel] = cap_last_energy[channel].avg_power_w
			* ((float)elapsed_s / 3600.0f);

		if (cap_vbus_sample_count[channel] > 0u) {
			float v_avg = ((float)cap_vbus_sum_mv[channel] / (float)cap_vbus_sample_count[channel]) / 1000.0f;
			if (v_avg > 0.1f) {
				cap_measured_mah_val[channel] = (uint32_t)((cap_measured_wh_val[channel] / v_avg) * 1000.0f);
			}
		}

		/* Full charge termination check */
		if (charging_is_done(channel)) {
			charging_stop(channel);
			if (device != 0) {
				ina233_read_ein(device, &cap_last_energy[channel]);
			}
			cap_substate[channel] = CAP_SUBSTATE_COMPLETE;
			cap_complete_flag[channel] = true;
			Buzzer_PlayCompleteMelody();
		}
		break;
	}

	case CAP_SUBSTATE_COMPLETE:
	default:
		break;
	}
}
