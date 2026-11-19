#include "Discharging.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

#include "config.h"
#include "Charging.h"
#include "Buzzer.h"

#define DISCHARGE_PORTB_MASK \
	(_BV(DISCHARGE1_BIT) | _BV(DISCHARGE2_BIT) | _BV(DISCHARGE3_BIT))

#define DSG_DEADTIME_MS 5u

static discharging_state_t dsg_state[TOTAL_CHANNELS] = {
	DSG_STATE_IDLE, DSG_STATE_IDLE, DSG_STATE_IDLE, DSG_STATE_IDLE
};
static uint16_t dsg_cutoff_voltage_mv[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static uint8_t  dsg_below_cutoff_count[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static bool     dsg_pending[TOTAL_CHANNELS] = {false, false, false, false};
static uint32_t dsg_deadtime_until[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};

/* Descending cutoff tables in PROGMEM with wrap-around support:
 * Item 0: "Discharge to Max" (maps to min cutoff: 3.00V CH1 Li-Ion, 2.50V CH2 LiFePO4) */
static const uint16_t dsg_cutoff_ch1_mv[DSG_CUTOFF_MAX_ITEMS] PROGMEM = {
	3000u, 4000u, 3900u, 3800u, 3700u, 3600u, 3500u, 3400u, 3300u, 3200u, 3100u, 3000u
};

static const uint16_t dsg_cutoff_ch2_mv[DSG_CUTOFF_MAX_ITEMS] PROGMEM = {
	2500u, 3500u, 3400u, 3300u, 3200u, 3100u, 3000u, 2900u, 2800u, 2700u, 2600u, 2500u
};

static void discharging_hardware_drive(uint8_t channel, bool on)
{
	if (on) {
		switch (channel) {
		case 0u: PORTB |= _BV(DISCHARGE1_BIT); break;
		case 1u: PORTB |= _BV(DISCHARGE2_BIT); break;
		case 2u: PORTB |= _BV(DISCHARGE3_BIT); break;
		case 3u: PORTC |= _BV(DISCHARGE4_BIT); break;
		default: break;
		}
	} else {
		switch (channel) {
		case 0u: PORTB &= (uint8_t)~_BV(DISCHARGE1_BIT); break;
		case 1u: PORTB &= (uint8_t)~_BV(DISCHARGE2_BIT); break;
		case 2u: PORTB &= (uint8_t)~_BV(DISCHARGE3_BIT); break;
		case 3u: PORTC &= (uint8_t)~_BV(DISCHARGE4_BIT); break;
		default: break;
		}
	}
}

void discharging_init(void)
{
	uint8_t ch;

	/* Pre-latch discharge NMOS gates LOW (NMOS OFF) */
	PORTB &= (uint8_t)~DISCHARGE_PORTB_MASK;
	PORTC &= (uint8_t)~_BV(DISCHARGE4_BIT);

	/* Configure pins as outputs */
	DDRB |= DISCHARGE_PORTB_MASK;
	DDRC |= _BV(DISCHARGE4_BIT);

	for (ch = 0u; ch < TOTAL_CHANNELS; ++ch) {
		dsg_state[ch] = DSG_STATE_IDLE;
		dsg_cutoff_voltage_mv[ch] = 0u;
		dsg_below_cutoff_count[ch] = 0u;
		dsg_pending[ch] = false;
		dsg_deadtime_until[ch] = 0u;
	}
}

void discharging_start(uint8_t channel, uint16_t cutoff_voltage_mv)
{
	if (channel >= TOTAL_CHANNELS) {
		return;
	}

	/* Strict safety interlock: guarantee CHG_MOSFETx is OFF before discharging */
	charging_stop(channel);
	discharging_hardware_drive(channel, false);

	dsg_cutoff_voltage_mv[channel] = cutoff_voltage_mv;
	dsg_below_cutoff_count[channel] = 0u;
	dsg_state[channel] = DSG_STATE_ACTIVE;
	dsg_pending[channel] = true;
	dsg_deadtime_until[channel] = sys_millis() + DSG_DEADTIME_MS;
}

void discharging_stop(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return;
	}

	discharging_hardware_drive(channel, false);
	dsg_state[channel] = DSG_STATE_IDLE;
	dsg_below_cutoff_count[channel] = 0u;
	dsg_pending[channel] = false;
}

void discharging_task(void)
{
	/* Handled cooperatively via discharging_control_step */
}

discharging_state_t discharging_get_state(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return DSG_STATE_IDLE;
	}
	return dsg_state[channel];
}

bool discharging_is_active(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return false;
	}
	return dsg_state[channel] == DSG_STATE_ACTIVE;
}

bool discharging_is_done(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return false;
	}
	return dsg_state[channel] == DSG_STATE_DONE;
}

bool discharging_outputs_are_safe(void)
{
	return (PORTB & DISCHARGE_PORTB_MASK) == 0u
		&& (PORTC & _BV(DISCHARGE4_BIT)) == 0u;
}

uint16_t discharging_get_cutoff_menu_item(uint8_t channel, uint8_t index)
{
	if (channel >= TOTAL_CHANNELS || index >= DSG_CUTOFF_MAX_ITEMS) {
		return 0u;
	}

	if (channel == 0u) {
		return pgm_read_word(&dsg_cutoff_ch1_mv[index]);
	} else if (channel == 1u) {
		return pgm_read_word(&dsg_cutoff_ch2_mv[index]);
	}
	return 0u;
}

uint8_t discharging_get_cutoff_menu_count(void)
{
	return DSG_CUTOFF_MAX_ITEMS;
}

/* 50 ms discharge control step */
bool discharging_control_step(uint8_t channel, uint16_t v_bat_mv)
{
	if (channel >= TOTAL_CHANNELS) {
		return false;
	}

	if (dsg_state[channel] != DSG_STATE_ACTIVE) {
		return false;
	}

	/* Enforce >= 5 ms deadtime before turning on */
	if (dsg_pending[channel]) {
		if (sys_millis() < dsg_deadtime_until[channel]) {
			return true;
		}
		discharging_hardware_drive(channel, true);
		dsg_pending[channel] = false;
	}

	uint16_t cutoff = dsg_cutoff_voltage_mv[channel];
	if (cutoff == 0u) {
		cutoff = (channel == 0u) ? CH1_SAFE_CUTOFF_MV : CH2_SAFE_CUTOFF_MV;
	}

	/* Cutoff Trigger: When V_bat <= v_cutoff (3 consecutive filtered samples) */
	if (v_bat_mv <= cutoff) {
		dsg_below_cutoff_count[channel]++;
		if (dsg_below_cutoff_count[channel] >= 3u) {
			discharging_hardware_drive(channel, false);
			dsg_state[channel] = DSG_STATE_DONE;
			Buzzer_PlayCompleteMelody();
			return false;
		}
	} else {
		dsg_below_cutoff_count[channel] = 0u;
	}

	return true;
}
