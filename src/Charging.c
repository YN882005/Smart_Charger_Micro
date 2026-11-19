#include "Charging.h"

#include <avr/io.h>
#include <util/atomic.h>

#include "config.h"
#include "ina233.h"
#include "Buzzer.h"
#include "Discharging.h"

#define CHARGE_PORTB_MASK \
	(_BV(CHARGE1_BIT) | _BV(CHARGE2_BIT) | _BV(CHARGE3_BIT))

/* Per-channel charging state */
static charging_state_t chg_state[TOTAL_CHANNELS] = {
	CHG_STATE_IDLE, CHG_STATE_IDLE, CHG_STATE_IDLE, CHG_STATE_IDLE
};
static uint16_t chg_current_set_ma[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static uint16_t chg_target_voltage_mv[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static uint32_t chg_cv_taper_start_ms[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static int32_t  chg_pid_integrator[TOTAL_CHANNELS] = {0, 0, 0, 0};
static uint8_t  chg_current_pwm[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};
static uint32_t chg_deadtime_until[TOTAL_CHANNELS] = {0u, 0u, 0u, 0u};

static void charging_set_hardware_pwm(uint8_t channel, uint8_t ocr_value)
{
	if (ocr_value > PWM_MAX_DUTY) {
		ocr_value = PWM_MAX_DUTY;
	}

	switch (channel) {
	case 0u: OCR1A = ocr_value; break;
	case 1u: OCR1B = ocr_value; break;
	case 2u: OCR2A = ocr_value; break;
	case 3u: OCR2B = ocr_value; break;
	default: break;
	}
}

void charging_init(void)
{
	uint8_t ch;

	/* Pre-latch HIGH (PMOS OFF) */
	PORTB |= CHARGE_PORTB_MASK;
	PORTD |= _BV(CHARGE4_BIT);

	DDRB |= CHARGE_PORTB_MASK;
	DDRD |= _BV(CHARGE4_BIT);

	/* Initialize Timer1: Fast PWM 8-bit (TOP=0x00FF), prescaler=1, inverting mode.
	 * 14.7456 MHz / 256 = 57.600 kHz EXACT */
	TCCR1A = _BV(WGM10) | _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
	TCCR1B = _BV(WGM12) | _BV(CS10);
	TIMSK1 = 0;
	OCR1A = 0;
	OCR1B = 0;
	TCNT1 = 0;

	/* Initialize Timer2: Fast PWM 8-bit (TOP=0xFF), prescaler=1, inverting mode.
	 * 14.7456 MHz / 256 = 57.600 kHz EXACT */
	TCCR2A = _BV(WGM20) | _BV(WGM21) | _BV(COM2A1) | _BV(COM2A0) | _BV(COM2B1) | _BV(COM2B0);
	TCCR2B = _BV(CS20);
	TIMSK2 = 0;
	OCR2A = 0;
	OCR2B = 0;
	TCNT2 = 0;

	for (ch = 0u; ch < TOTAL_CHANNELS; ++ch) {
		chg_state[ch] = CHG_STATE_IDLE;
		chg_current_set_ma[ch] = 0u;
		chg_target_voltage_mv[ch] = 0u;
		chg_cv_taper_start_ms[ch] = 0u;
		chg_pid_integrator[ch] = 0;
		chg_current_pwm[ch] = 0u;
		chg_deadtime_until[ch] = 0u;
	}
}

void charging_start(uint8_t channel, uint16_t current_ma, uint16_t target_voltage_mv)
{
	if (channel >= TOTAL_CHANNELS) {
		return;
	}

	/* Strict safety interlock: ensure discharge MOSFET is OFF and enforce >= 5 ms dead-time */
	discharging_stop(channel);
	charging_set_hardware_pwm(channel, 0u);
	chg_current_pwm[channel] = 0u;
	chg_deadtime_until[channel] = sys_millis() + 5u;

	chg_current_set_ma[channel] = current_ma;
	chg_target_voltage_mv[channel] = target_voltage_mv;
	chg_cv_taper_start_ms[channel] = 0u;
	chg_pid_integrator[channel] = 0;

	/* Check initial voltage to decide if trickle preconditioning is needed */
	uint16_t v_trickle = (channel == 0u) ? CH1_TRICKLE_ENTER_MV : CH2_TRICKLE_ENTER_MV;
	extern volatile ina233_raw_sample_t g_ina233_samples[INA233_CHANNEL_COUNT];
	extern volatile bool g_ina233_sample_valid[INA233_CHANNEL_COUNT];

	uint16_t v_now_mv = 0u;
	if (channel < (uint8_t)INA233_CHANNEL_COUNT && g_ina233_sample_valid[channel]) {
		v_now_mv = (uint16_t)(g_ina233_samples[channel].bus_voltage_v * 1000.0f);
	}

	if (v_now_mv > 0u && v_now_mv < v_trickle) {
		chg_state[channel] = CHG_STATE_TRICKLE;
	} else {
		chg_state[channel] = CHG_STATE_CC;
	}
}

void charging_stop(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return;
	}

	charging_set_hardware_pwm(channel, 0u);
	chg_current_pwm[channel] = 0u;
	chg_state[channel] = CHG_STATE_IDLE;
	chg_pid_integrator[channel] = 0;
	chg_cv_taper_start_ms[channel] = 0u;
}

void charging_task(void)
{
	/* Handled cooperatively via charging_control_step */
}

charging_state_t charging_get_state(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return CHG_STATE_IDLE;
	}
	return chg_state[channel];
}

bool charging_is_active(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return false;
	}
	return chg_state[channel] == CHG_STATE_TRICKLE
		|| chg_state[channel] == CHG_STATE_CC
		|| chg_state[channel] == CHG_STATE_CV;
}

bool charging_is_done(uint8_t channel)
{
	if (channel >= TOTAL_CHANNELS) {
		return false;
	}
	return chg_state[channel] == CHG_STATE_DONE;
}

bool charging_outputs_are_safe(void)
{
	return (PORTB & CHARGE_PORTB_MASK) == CHARGE_PORTB_MASK
		&& (PORTD & _BV(CHARGE4_BIT)) != 0u;
}

/* 20 ms PID control step */
uint8_t charging_control_step(uint8_t channel, uint16_t v_bat_mv, int16_t i_bat_ma)
{
	if (channel >= TOTAL_CHANNELS) {
		return 0u;
	}

	/* Enforce >= 5 ms deadtime on transitions */
	if (sys_millis() < chg_deadtime_until[channel]) {
		charging_set_hardware_pwm(channel, 0u);
		return 0u;
	}

	uint16_t v_target = chg_target_voltage_mv[channel];
	if (v_target == 0u) {
		v_target = (channel == 0u) ? CH1_FULL_CHARGE_MV : CH2_FULL_CHARGE_MV;
	}

	uint8_t pwm = chg_current_pwm[channel];

	switch (chg_state[channel]) {
	case CHG_STATE_IDLE:
	case CHG_STATE_DONE:
	case CHG_STATE_FAULT:
		pwm = 0u;
		break;

	case CHG_STATE_TRICKLE: {
		uint16_t trickle_exit = (channel == 0u) ? CH1_TRICKLE_EXIT_MV : CH2_TRICKLE_EXIT_MV;
		if (v_bat_mv >= trickle_exit) {
			/* Transition automatically to CC charge */
			chg_state[channel] = CHG_STATE_CC;
			chg_pid_integrator[channel] = 0;
			break;
		}

		/* Regulate current at min(I_set, 100 mA) */
		uint16_t i_trickle_limit = chg_current_set_ma[channel];
		if (i_trickle_limit > 100u) {
			i_trickle_limit = 100u;
		}

		int16_t error = (int16_t)i_trickle_limit - i_bat_ma;
		chg_pid_integrator[channel] += (int32_t)error;
		if (chg_pid_integrator[channel] > 24000) chg_pid_integrator[channel] = 24000;
		if (chg_pid_integrator[channel] < 0) chg_pid_integrator[channel] = 0;

		int32_t out = (error / 8) + (chg_pid_integrator[channel] / 100);
		if (out < 0) out = 0;
		if (out > (int32_t)PWM_MAX_DUTY) out = PWM_MAX_DUTY;

		/* Soft-start step: ramp up by max 2 counts per 20 ms */
		if (out > (int32_t)pwm + 2) {
			pwm += 2u;
		} else if (out < (int32_t)pwm - 2) {
			pwm -= 2u;
		} else {
			pwm = (uint8_t)out;
		}
		break;
	}

	case CHG_STATE_CC: {
		/* CC Phase: check if voltage reached target -> enter CV */
		if (v_bat_mv >= v_target) {
			chg_state[channel] = CHG_STATE_CV;
			chg_cv_taper_start_ms[channel] = 0u;
			chg_pid_integrator[channel] = (int32_t)pwm * 100;
			break;
		}

		int16_t error = (int16_t)chg_current_set_ma[channel] - i_bat_ma;
		chg_pid_integrator[channel] += (int32_t)error;
		if (chg_pid_integrator[channel] > 24000) chg_pid_integrator[channel] = 24000;
		if (chg_pid_integrator[channel] < 0) chg_pid_integrator[channel] = 0;

		int32_t out = (error / 8) + (chg_pid_integrator[channel] / 100);
		if (out < 0) out = 0;
		if (out > (int32_t)PWM_MAX_DUTY) out = PWM_MAX_DUTY;

		/* Soft-start ramp */
		if (out > (int32_t)pwm + 2) {
			pwm += 2u;
		} else if (out < (int32_t)pwm - 2) {
			pwm -= 2u;
		} else {
			pwm = (uint8_t)out;
		}
		break;
	}

	case CHG_STATE_CV: {
		/* CV Phase: Hold V_bat == v_target by smoothly tapering duty cycle down */
		int16_t v_err = (int16_t)v_target - (int16_t)v_bat_mv;
		chg_pid_integrator[channel] += (int32_t)v_err * 2;
		if (chg_pid_integrator[channel] > 24000) chg_pid_integrator[channel] = 24000;
		if (chg_pid_integrator[channel] < 0) chg_pid_integrator[channel] = 0;

		int32_t out = (v_err / 4) + (chg_pid_integrator[channel] / 100);
		if (out < 0) out = 0;
		if (out > (int32_t)PWM_MAX_DUTY) out = PWM_MAX_DUTY;

		/* Never exceed current limit */
		if (i_bat_ma > (int16_t)chg_current_set_ma[channel] && out > (int32_t)pwm) {
			out = pwm;
		}

		if (out > (int32_t)pwm + 1) {
			pwm += 1u;
		} else if (out < (int32_t)pwm - 1) {
			pwm -= 1u;
		} else {
			pwm = (uint8_t)out;
		}

		/* Termination condition:
		 * I_measured <= max(50 mA, 0.05 * I_set) for 3 consecutive seconds */
		uint16_t i_cutoff = (uint16_t)((uint32_t)chg_current_set_ma[channel] * 5UL / 100UL);
		if (i_cutoff < 50u) {
			i_cutoff = 50u;
		}

		if (i_bat_ma <= (int16_t)i_cutoff) {
			uint32_t now = sys_millis();
			if (chg_cv_taper_start_ms[channel] == 0u) {
				chg_cv_taper_start_ms[channel] = now;
			} else if (now - chg_cv_taper_start_ms[channel] >= 3000u) {
				/* Full charge complete! */
				chg_state[channel] = CHG_STATE_DONE;
				pwm = 0u;
				charging_set_hardware_pwm(channel, 0u);
				Buzzer_PlayCompleteMelody();
			}
		} else {
			chg_cv_taper_start_ms[channel] = 0u;
		}
		break;
	}
	}

	chg_current_pwm[channel] = pwm;
	charging_set_hardware_pwm(channel, pwm);
	return pwm;
}
