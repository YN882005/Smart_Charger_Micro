#include <stdbool.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "Buzzer.h"
#include "Charging.h"
#include "Capacity.h"
#include "Discharging.h"
#include "Encoder.h"
#include "OLED_Screen.h"
#include "Storage.h"
#include "config.h"
#include "ina233.h"
#include "twi.h"

/* ---- System timebase (Timer0, 1 ms tick) ---- */
volatile uint32_t g_sys_millis = 0u;
volatile uint32_t g_sys_micros = 0u;

ISR(TIMER0_COMPA_vect)
{
	g_sys_millis += TIMER0_TICK_MS;
	g_sys_micros += 1000u;
}

uint32_t sys_millis(void)
{
	uint32_t ms;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		ms = g_sys_millis;
	}
	return ms;
}

uint32_t sys_micros(void)
{
	uint32_t us;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		us = g_sys_micros;
	}
	return us;
}

/* ---- SoC Lookup Tables (PROGMEM) ---- */
/* CH1 Li-Ion: 3000mV=0%, 3500mV=10%, 3700mV=25%, 3800mV=50%, 3950mV=75%, 4100mV=90%, 4200mV=100% */
const uint16_t CH1_SOC_VOLTAGE_MV[CH1_SOC_LUT_COUNT] PROGMEM = {
	3000u, 3500u, 3700u, 3800u, 3950u, 4100u, 4200u
};
const uint8_t CH1_SOC_PERCENT[CH1_SOC_LUT_COUNT] PROGMEM = {
	0u, 10u, 25u, 50u, 75u, 90u, 100u
};

/* CH2 LiFePO4: 2500mV=0%, 3000mV=10%, 3200mV=20%, 3280mV=50%, 3330mV=80%, 3450mV=95%, 3650mV=100% */
const uint16_t CH2_SOC_VOLTAGE_MV[CH2_SOC_LUT_COUNT] PROGMEM = {
	2500u, 3000u, 3200u, 3280u, 3330u, 3450u, 3650u
};
const uint8_t CH2_SOC_PERCENT[CH2_SOC_LUT_COUNT] PROGMEM = {
	0u, 10u, 20u, 50u, 80u, 95u, 100u
};

/* ---- Global objects ---- */
volatile ina233_raw_sample_t g_ina233_samples[INA233_CHANNEL_COUNT];
volatile bool g_ina233_sample_valid[INA233_CHANNEL_COUNT];
volatile bool g_oled_ready;

/* Per-channel active modules */
bool g_charging_active[TOTAL_CHANNELS] = {false, false, false, false};
bool g_discharging_active[TOTAL_CHANNELS] = {false, false, false, false};
bool g_storage_active[TOTAL_CHANNELS] = {false, false, false, false};
bool g_capacity_active[TOTAL_CHANNELS] = {false, false, false, false};

static void refresh_samples(ina233_device_t devices[INA233_CHANNEL_COUNT],
	ina233_raw_sample_t samples[INA233_CHANNEL_COUNT],
	const bool device_ready[INA233_CHANNEL_COUNT],
	bool sample_valid[INA233_CHANNEL_COUNT])
{
	uint8_t channel;

	for (channel = 0u; channel < (uint8_t)INA233_CHANNEL_COUNT; ++channel) {
		sample_valid[channel] = device_ready[channel]
			&& ina233_read_sample(&devices[channel], &samples[channel]);
		g_ina233_sample_valid[channel] = sample_valid[channel];
		if (sample_valid[channel]) {
			g_ina233_samples[channel] = samples[channel];
		}
	}
}

int main(void)
{
	/* -----------------------------------------------------------------
	 * PROMPT BLOCK 02: SAFE BOOT INITIALIZATION
	 * Execute as FIRST lines of main() BEFORE configuring DDRx:
	 * Pre-latch all PMOS gates HIGH (OFF) and NMOS gates LOW (OFF)
	 * ----------------------------------------------------------------- */
	PORTB |= (1 << PB1) | (1 << PB2) | (1 << PB3);    /* Pre-latch CHG_MOSFET1..3 HIGH (PMOS OFF) */
	PORTD |= (1 << PD3);                              /* Pre-latch CHG_MOSFET4 HIGH (PMOS OFF) */
	PORTB &= ~((1 << PB0) | (1 << PB4) | (1 << PB5)); /* Pre-latch DSG_MOSFET1..3 LOW (NMOS OFF) */
	PORTC &= ~(1 << PC0);                             /* Pre-latch DSG_MOSFET4 LOW (NMOS OFF) */

	/* Safe output configuration with zero reset glitch */
	DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB0) | (1 << PB4) | (1 << PB5);
	DDRD |= (1 << PD3) | (1 << PD4);
	DDRC |= (1 << PC0);

	/* -----------------------------------------------------------------
	 * Configure Timer0 as 1.000 ms System Timebase (CTC mode)
	 * Prescaler = 64, OCR0A = 229 -> 14,745,600 / (64 * 230) = 1001.7 Hz
	 * ----------------------------------------------------------------- */
	TCCR0A = _BV(WGM01);                          /* CTC mode */
	TCCR0B = _BV(CS01) | _BV(CS00);               /* Prescaler = 64 */
	OCR0A = (uint8_t)TIMER0_OCR0A;                /* 229 -> 1.000 ms tick */
	OCR0B = 0;
	TCNT0 = 0;
	TIFR0 = _BV(OCF0A) | _BV(OCF0B);
	TIMSK0 = _BV(OCIE0A);                         /* Enable Compare Match A ISR */

	/* Initialize Subsystems */
	charging_init();
	discharging_init();
	storage_init();
	capacity_init();
	encoder_init();
	buzzer_init();

	/* INA233 ALERT line: PD5 input with internal pull-up */
	INA233_ALERT_DDR &= (uint8_t)~_BV(INA233_ALERT_BIT);
	INA233_ALERT_PORT |= _BV(INA233_ALERT_BIT);

	/* Hardware TWI (400 kHz) */
	twi_init();

	/* Initialize INA233 telemetry chips */
	ina233_device_t devices[INA233_CHANNEL_COUNT] = {{0}};
	ina233_raw_sample_t samples[INA233_CHANNEL_COUNT] = {{0}};
	bool device_ready[INA233_CHANNEL_COUNT] = {false, false};
	bool sample_valid[INA233_CHANNEL_COUNT] = {false, false};

	device_ready[0] = ina233_device_init(&devices[0], INA233_CH1_I2C_ADDRESS,
		INA233_R_SHUNT_OHMS, 1.5f);
	device_ready[1] = ina233_device_init(&devices[1], INA233_CH2_I2C_ADDRESS,
		INA233_R_SHUNT_OHMS, 1.5f);

	capacity_set_device(0, &devices[0]);
	capacity_set_device(1, &devices[1]);

	/* Initial telemetry read */
	refresh_samples(devices, samples, device_ready, sample_valid);

	/* Initialize OLED display */
	g_oled_ready = oled_screen_init();
	if (g_oled_ready) {
		bool alert_active = (INA233_ALERT_PIN & _BV(INA233_ALERT_BIT)) == 0u;
		oled_screen_render(sample_valid, samples, alert_active);
	}

	/* Enable global interrupts */
	sei();

	/* -----------------------------------------------------------------
	 * ZERO-BLOCKING COOPERATIVE SCHEDULER
	 * Telemetry: 50 ms, PID/Storage/Capacity: 20 ms, GUI: 100 ms, Buzzer: async
	 * ----------------------------------------------------------------- */
	uint32_t last_telemetry_ms = 0u;
	uint32_t last_pid_ms = 0u;
	uint32_t last_gui_ms = 0u;

	for (;;) {
		uint32_t now = sys_millis();

		/* Encoder switch polling (single press / double press events) */
		encoder_event_t event = encoder_poll();
		if (event != ENCODER_EVENT_NONE) {
			oled_screen_handle_encoder_event(event);
		}

		/* Always poll rotary step navigation */
		oled_screen_handle_encoder_event(ENCODER_EVENT_NONE);

		/* Telemetry Task: 50 ms */
		if (now - last_telemetry_ms >= 50u) {
			last_telemetry_ms = now;
			refresh_samples(devices, samples, device_ready, sample_valid);
		}

		/* PID / Storage / Capacity Task: 20 ms */
		if (now - last_pid_ms >= 20u) {
			last_pid_ms = now;

			for (uint8_t ch = 0u; ch < INA233_CHANNEL_COUNT; ++ch) {
				if (!sample_valid[ch]) {
					continue;
				}

				uint16_t v_mv = (uint16_t)(g_ina233_samples[ch].bus_voltage_v * 1000.0f);
				int16_t i_ma = (int16_t)g_ina233_samples[ch].current_ma;

				/* Charging regulation */
				if (g_charging_active[ch]) {
					charging_control_step(ch, v_mv, i_ma);
					if (charging_is_done(ch)) {
						g_charging_active[ch] = false;
					}
				}

				/* Discharging regulation */
				if (g_discharging_active[ch]) {
					bool continue_dsg = discharging_control_step(ch, v_mv);
					if (!continue_dsg) {
						g_discharging_active[ch] = false;
					}
				}

				/* Automated storage conditioning */
				if (g_storage_active[ch]) {
					storage_advance(ch, v_mv);
					if (storage_is_done(ch)) {
						g_storage_active[ch] = false;
					}
				}

				/* 2-stage capacity measurement */
				if (g_capacity_active[ch]) {
					capacity_advance(ch, v_mv, i_ma, &devices[ch]);
					if (capacity_is_done(ch)) {
						g_capacity_active[ch] = false;
					}
				}
			}
		}

		/* GUI Rendering Task: 100 ms */
		if (now - last_gui_ms >= 100u) {
			last_gui_ms = now;
			bool alert_active = (INA233_ALERT_PIN & _BV(INA233_ALERT_BIT)) == 0u;
			if (g_oled_ready) {
				oled_screen_render(sample_valid, samples, alert_active);
			}
		}

		/* Audio Buzzer / Melody Engine: async (ISR-driven countdown) */
		buzzer_task();
	}
}
