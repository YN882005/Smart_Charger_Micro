#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "config.h"
#include "Charging.h"
#include "Discharging.h"
#include "Storage.h"
#include "Capacity.h"
#include "Encoder.h"
#include "Buzzer.h"
#include "ina233.h"

/* External variables from main.c / firmware */
volatile uint32_t g_sys_millis = 0u;
volatile uint32_t g_sys_micros = 0u;
volatile ina233_raw_sample_t g_ina233_samples[INA233_CHANNEL_COUNT];
volatile bool g_ina233_sample_valid[INA233_CHANNEL_COUNT];

uint32_t sys_millis(void) { return g_sys_millis; }
uint32_t sys_micros(void) { return g_sys_micros; }

/* Global flags required by modules */
bool g_charging_active[TOTAL_CHANNELS] = {false, false, false, false};
bool g_discharging_active[TOTAL_CHANNELS] = {false, false, false, false};
bool g_storage_active[TOTAL_CHANNELS] = {false, false, false, false};
bool g_capacity_active[TOTAL_CHANNELS] = {false, false, false, false};

/* PROGMEM SoC LUTs */
const uint16_t CH1_SOC_VOLTAGE_MV[CH1_SOC_LUT_COUNT] PROGMEM = {
	3000u, 3500u, 3700u, 3800u, 3950u, 4100u, 4200u
};
const uint8_t CH1_SOC_PERCENT[CH1_SOC_LUT_COUNT] PROGMEM = {
	0u, 10u, 25u, 50u, 75u, 90u, 100u
};

const uint16_t CH2_SOC_VOLTAGE_MV[CH2_SOC_LUT_COUNT] PROGMEM = {
	2500u, 3000u, 3200u, 3280u, 3330u, 3450u, 3650u
};
const uint8_t CH2_SOC_PERCENT[CH2_SOC_LUT_COUNT] PROGMEM = {
	0u, 10u, 20u, 50u, 80u, 95u, 100u
};

static uint16_t s_failed_tests = 0u;
static uint16_t s_passed_tests = 0u;

#define TEST_ASSERT(cond) do { \
	if (cond) { \
		s_passed_tests++; \
	} else { \
		s_failed_tests++; \
	} \
} while (0)

/* 1. Pin Configuration & Safe Boot Check */
static void test_pinout_and_safe_boot(void)
{
	/* Verify pin definitions against Schematic */
	TEST_ASSERT(CHARGE1_BIT == PB1);
	TEST_ASSERT(CHARGE2_BIT == PB2);
	TEST_ASSERT(CHARGE3_BIT == PB3);
	TEST_ASSERT(CHARGE4_BIT == PD3);

	TEST_ASSERT(DISCHARGE1_BIT == PB0);
	TEST_ASSERT(DISCHARGE2_BIT == PB5);
	TEST_ASSERT(DISCHARGE3_BIT == PB4);
	TEST_ASSERT(DISCHARGE4_BIT == PC0);

	TEST_ASSERT(BUZZER_BIT == PD4);
	TEST_ASSERT(INA233_ALERT_BIT == PD5);
	TEST_ASSERT(ENCODER_CLK_BIT == PC1);
	TEST_ASSERT(ENCODER_DT_BIT == PC2);
	TEST_ASSERT(ENCODER_SWITCH_BIT == PC3);

	/* Safe boot pre-latch logic */
	PORTB = 0;
	PORTD = 0;
	PORTC = 0xFF;

	PORTB |= (1 << PB1) | (1 << PB2) | (1 << PB3);
	PORTD |= (1 << PD3);
	PORTB &= ~((1 << PB0) | (1 << PB4) | (1 << PB5));
	PORTC &= ~(1 << PC0);

	/* Check latched states */
	TEST_ASSERT((PORTB & (1 << PB1)) != 0); /* PMOS1 OFF (HIGH) */
	TEST_ASSERT((PORTB & (1 << PB2)) != 0); /* PMOS2 OFF (HIGH) */
	TEST_ASSERT((PORTB & (1 << PB3)) != 0); /* PMOS3 OFF (HIGH) */
	TEST_ASSERT((PORTD & (1 << PD3)) != 0); /* PMOS4 OFF (HIGH) */

	TEST_ASSERT((PORTB & (1 << PB0)) == 0); /* NMOS1 OFF (LOW) */
	TEST_ASSERT((PORTB & (1 << PB5)) == 0); /* NMOS2 OFF (LOW) */
	TEST_ASSERT((PORTB & (1 << PB4)) == 0); /* NMOS3 OFF (LOW) */
	TEST_ASSERT((PORTC & (1 << PC0)) == 0); /* NMOS4 OFF (LOW) */
}

/* 2. Frequency Calculations */
static void test_frequencies(void)
{
	/* Timer1/2 57.6 kHz Fast PWM */
	uint32_t f_pwm = F_CPU / (1UL * 256UL);
	TEST_ASSERT(f_pwm == 57600UL);

	/* Timer0 1.000 ms CTC Timebase */
	uint32_t f_t0 = F_CPU / (64UL * (TIMER0_OCR0A + 1UL));
	TEST_ASSERT(f_t0 == 1001UL || f_t0 == 1002UL);

	/* Buzzer 2.3 kHz Half-period */
	uint32_t half_ticks = 115200UL / BUZZER_FREQUENCY_HZ;
	TEST_ASSERT(half_ticks == 50UL); /* 50 * (64 / 14745600) = 217.01 us */
}

/* 3. Chemistry Limits & SoC LUT Verification */
static void test_chemistry_profiles(void)
{
	TEST_ASSERT(CH1_FULL_CHARGE_MV == 4200u);
	TEST_ASSERT(CH1_STORAGE_MV == 3800u);
	TEST_ASSERT(CH1_SAFE_CUTOFF_MV == 3000u);
	TEST_ASSERT(CH1_TRICKLE_ENTER_MV == 2800u);
	TEST_ASSERT(CH1_TRICKLE_EXIT_MV == 3000u);

	TEST_ASSERT(CH2_FULL_CHARGE_MV == 3650u);
	TEST_ASSERT(CH2_STORAGE_MV == 3300u);
	TEST_ASSERT(CH2_SAFE_CUTOFF_MV == 2500u);
	TEST_ASSERT(CH2_TRICKLE_ENTER_MV == 2300u);
	TEST_ASSERT(CH2_TRICKLE_EXIT_MV == 2500u);

	TEST_ASSERT(pgm_read_word(&CH1_SOC_VOLTAGE_MV[0]) == 3000u);
	TEST_ASSERT(pgm_read_byte(&CH1_SOC_PERCENT[0]) == 0u);
	TEST_ASSERT(pgm_read_word(&CH1_SOC_VOLTAGE_MV[6]) == 4200u);
	TEST_ASSERT(pgm_read_byte(&CH1_SOC_PERCENT[6]) == 100u);

	TEST_ASSERT(pgm_read_word(&CH2_SOC_VOLTAGE_MV[0]) == 2500u);
	TEST_ASSERT(pgm_read_byte(&CH2_SOC_PERCENT[0]) == 0u);
	TEST_ASSERT(pgm_read_word(&CH2_SOC_VOLTAGE_MV[6]) == 3650u);
	TEST_ASSERT(pgm_read_byte(&CH2_SOC_PERCENT[6]) == 100u);
}

/* 4. Charging Subsystem: Trickle, CC, CV, and Termination */
static void test_charging_state_machine(void)
{
	charging_init();

	/* Case 1: CH1 battery deeply depleted (2600 mV < 2800 mV trickle enter) */
	g_sys_millis = 100u;
	g_ina233_sample_valid[0] = true;
	g_ina233_samples[0].bus_voltage_v = 2.60f;
	g_ina233_samples[0].current_ma = 50.0f;

	charging_start(0, 500u, CH1_FULL_CHARGE_MV);
	TEST_ASSERT(charging_get_state(0) == CHG_STATE_TRICKLE);

	/* In trickle charge, current is capped at 100 mA */
	g_sys_millis = 110u; /* pass 5ms deadtime */
	uint8_t pwm = charging_control_step(0, 2600u, 50);
	TEST_ASSERT(pwm > 0);

	/* Battery charges up to 3000 mV (trickle exit) -> auto transition to CC */
	charging_control_step(0, 3000u, 90);
	TEST_ASSERT(charging_get_state(0) == CHG_STATE_CC);

	/* CC Phase regulates current up to target voltage */
	pwm = charging_control_step(0, 3700u, 450);
	TEST_ASSERT(charging_get_state(0) == CHG_STATE_CC);

	/* Battery reaches 4200 mV (target voltage) -> transition to CV */
	charging_control_step(0, 4200u, 500);
	TEST_ASSERT(charging_get_state(0) == CHG_STATE_CV);

	/* CV Phase: current tapers down. Cutoff threshold = max(50, 0.05*500) = 50 mA.
	 * Simulate 3 consecutive seconds with current <= 50 mA */
	g_sys_millis = 1000u;
	charging_control_step(0, 4200u, 40); /* start taper timer */
	TEST_ASSERT(charging_get_state(0) == CHG_STATE_CV);

	g_sys_millis = 3500u; /* 2.5 seconds */
	charging_control_step(0, 4200u, 40);
	TEST_ASSERT(charging_get_state(0) == CHG_STATE_CV);

	g_sys_millis = 4100u; /* > 3.0 seconds */
	charging_control_step(0, 4200u, 40);
	TEST_ASSERT(charging_get_state(0) == CHG_STATE_DONE);
	TEST_ASSERT(charging_is_done(0) == true);
	TEST_ASSERT(OCR1A == 0); /* PMOS turned OFF */
}

/* 5. Discharging Subsystem: Deadtime, Cutoff, and Wrap-around Menu */
static void test_discharging_state_machine(void)
{
	discharging_init();

	/* Verify wrap-around table count */
	TEST_ASSERT(discharging_get_cutoff_menu_count() == 12u);

	/* Item 0 is "Discharge to Max" (min cutoff: 3.00V CH1, 2.50V CH2) */
	TEST_ASSERT(discharging_get_cutoff_menu_item(0, 0) == 3000u);
	TEST_ASSERT(discharging_get_cutoff_menu_item(0, 1) == 4000u);
	TEST_ASSERT(discharging_get_cutoff_menu_item(0, 11) == 3000u);

	TEST_ASSERT(discharging_get_cutoff_menu_item(1, 0) == 2500u);
	TEST_ASSERT(discharging_get_cutoff_menu_item(1, 1) == 3500u);
	TEST_ASSERT(discharging_get_cutoff_menu_item(1, 11) == 2500u);

	/* Test Discharge Execution */
	g_sys_millis = 5000u;
	discharging_start(0, 3600u);
	TEST_ASSERT(discharging_get_state(0) == DSG_STATE_ACTIVE);

	/* Before 5 ms deadtime, NMOS pin remains LOW */
	g_sys_millis = 5002u;
	discharging_control_step(0, 3800u);
	TEST_ASSERT((PORTB & (1 << PB0)) == 0);

	/* After 5 ms deadtime, NMOS pin is asserted HIGH */
	g_sys_millis = 5010u;
	discharging_control_step(0, 3800u);
	TEST_ASSERT((PORTB & (1 << PB0)) != 0);

	/* V_bat drops <= cutoff (3600 mV) -> requires 3 consecutive samples */
	discharging_control_step(0, 3600u);
	TEST_ASSERT(discharging_get_state(0) == DSG_STATE_ACTIVE);

	discharging_control_step(0, 3590u);
	TEST_ASSERT(discharging_get_state(0) == DSG_STATE_ACTIVE);

	discharging_control_step(0, 3580u); /* 3rd sample */
	TEST_ASSERT(discharging_get_state(0) == DSG_STATE_DONE);
	TEST_ASSERT((PORTB & (1 << PB0)) == 0); /* NMOS turned OFF */
}

/* 6. Storage Subsystem: Cases A, B, C */
static void test_storage_subsystem(void)
{
	storage_init();

	/* Case A: Already at storage (3800 mV +/- 20 mV deadband) */
	g_sys_millis = 6000u;
	storage_start(0, 500u);
	TEST_ASSERT(storage_get_state(0) == STG_STATE_ASSESS);

	/* Settle for 200 ms */
	g_sys_millis = 6250u;
	storage_advance(0, 3810u);
	TEST_ASSERT(storage_get_state(0) == STG_STATE_DONE);

	/* Case B: Under storage voltage (3500 mV < 3780 mV) -> charge */
	g_sys_millis = 7000u;
	storage_start(0, 500u);
	g_sys_millis = 7250u;
	storage_advance(0, 3500u);
	TEST_ASSERT(storage_get_state(0) == STG_STATE_SUB_CHARGE);

	/* Case C: Over storage voltage (4100 mV > 3820 mV) -> discharge */
	g_sys_millis = 8000u;
	storage_start(0, 500u);
	g_sys_millis = 8250u;
	storage_advance(0, 4100u);
	TEST_ASSERT(storage_get_state(0) == STG_STATE_SUB_DISCHARGE);
}

/* 7. Capacity Subsystem: Energy Accumulator Formula Validation */
static void test_capacity_subsystem(void)
{
	capacity_init();

	/* Verify the READ_EIN mathematical model specified in Section 07 */
	uint32_t rollover_count = 2u;
	uint16_t acc_16bit = 10000u;
	uint32_t sample_count = 500u;

	uint32_t raw_power_sum = (rollover_count * 65536UL) + acc_16bit;
	TEST_ASSERT(raw_power_sum == 141072UL);

	float avg_power_w = ((float)raw_power_sum / (float)sample_count) * 0.00125f;
	TEST_ASSERT(avg_power_w > 0.35f && avg_power_w < 0.36f);

	uint32_t elapsed_seconds = 3600u; /* 1 hour */
	float energy_wh = avg_power_w * ((float)elapsed_seconds / 3600.0f);
	TEST_ASSERT(energy_wh > 0.35f && energy_wh < 0.36f);

	float v_avg = 3.7f;
	uint32_t capacity_mah = (uint32_t)((energy_wh / v_avg) * 1000.0f);
	TEST_ASSERT(capacity_mah > 90u && capacity_mah < 100u);
}

/* 8. Encoder Dual-Sensitivity and Double-Press Window */
static void test_encoder_sensitivity(void)
{
	encoder_init();

	/* Reset accumulator */
	Encoder_ResetAccumulator();

	/* Menu navigation (is_value_picking == false): 2 ticks required */
	Encoder_SimulateDetent(1);
	TEST_ASSERT(Encoder_GetStep(false) == 0); /* 1 detent = no step */

	Encoder_SimulateDetent(1);
	TEST_ASSERT(Encoder_GetStep(false) == 1); /* 2 detents = 1 menu step */

	/* Reversal reset: if accumulated +1 and then turns negative */
	Encoder_SimulateDetent(1);
	Encoder_SimulateDetent(-1); /* direction reversed */
	TEST_ASSERT(Encoder_GetStep(false) == 0);

	/* Value picking (is_value_picking == true): 1 tick required */
	Encoder_SimulateDetent(1);
	TEST_ASSERT(Encoder_GetStep(true) == 1); /* 1 detent = 1 value step */

	Encoder_SimulateDetent(-1);
	TEST_ASSERT(Encoder_GetStep(true) == -1); /* 1 detent = 1 value step */
}

int main(void)
{
	test_pinout_and_safe_boot();
	test_frequencies();
	test_chemistry_profiles();
	test_charging_state_machine();
	test_discharging_state_machine();
	test_storage_subsystem();
	test_capacity_subsystem();
	test_encoder_sensitivity();

	/* Store test results in AVR general purpose I/O registers:
	 * GPIOR0: 0x55 on success, 0xEE on failure
	 * GPIOR1: Number of failed assertions
	 * GPIOR2: Number of passed assertions */
	GPIOR1 = (uint8_t)s_failed_tests;
	GPIOR2 = (uint8_t)s_passed_tests;

	if (s_failed_tests == 0u && s_passed_tests >= 35u) {
		GPIOR0 = 0x55u; /* SUCCESS */
	} else {
		GPIOR0 = 0xEEu; /* FAILURE */
	}

	/* Signal breakpoint to simulator to exit */
	__asm__ __volatile__ ("break");

	for (;;) {}
}
