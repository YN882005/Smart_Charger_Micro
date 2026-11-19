#include "Encoder.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "config.h"

#define ENCODER_BUTTON_DEBOUNCE_MS 25u
#define ENCODER_DOUBLE_CLICK_MS    400u

static volatile int8_t s_raw_detents = 0;
static volatile uint8_t s_prev_state = 0u;

static int8_t s_menu_accum = 0;
static int8_t s_value_accum = 0;

/* Switch state machine variables */
static bool s_btn_latched = false;
static bool s_click_pending = false;
static bool s_suppress_click_on_release = false;
static uint32_t s_release_millis = 0u;
static uint8_t s_press_samples = 0u;

ISR(PCINT1_vect)
{
	static const int8_t transitions[16] = {
		 0, -1,  1,  0,
		 1,  0,  0, -1,
		-1,  0,  0,  1,
		 0,  1, -1,  0
	};

	uint8_t pins = ENCODER_PIN;
	uint8_t state = (uint8_t)(((pins >> ENCODER_DT_BIT) & 0x01u) << 1)
		| ((pins >> ENCODER_CLK_BIT) & 0x01u);
	uint8_t index = (uint8_t)((s_prev_state << 2) | state);

	s_raw_detents = (int8_t)(s_raw_detents + transitions[index & 0x0Fu]);
	s_prev_state = state;
}

void encoder_init(void)
{
	/* Inputs with pull-ups for HW-040 rotary encoder */
	ENCODER_DDR &= (uint8_t)~(_BV(ENCODER_CLK_BIT) | _BV(ENCODER_DT_BIT)
		| _BV(ENCODER_SWITCH_BIT));
	ENCODER_PORT |= _BV(ENCODER_CLK_BIT) | _BV(ENCODER_DT_BIT)
		| _BV(ENCODER_SWITCH_BIT);

	s_prev_state = (uint8_t)(((ENCODER_PIN >> ENCODER_DT_BIT) & 0x01u) << 1)
		| ((ENCODER_PIN >> ENCODER_CLK_BIT) & 0x01u);
	s_raw_detents = 0;
	s_menu_accum = 0;
	s_value_accum = 0;
	s_btn_latched = false;
	s_click_pending = false;
	s_suppress_click_on_release = false;
	s_release_millis = 0u;
	s_press_samples = 0u;

	PCICR |= _BV(PCIE1);
	PCMSK1 |= _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11);
}

void Encoder_ResetAccumulator(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		s_raw_detents = 0;
	}
	s_menu_accum = 0;
	s_value_accum = 0;
}

void Encoder_SimulateDetent(int8_t detents)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		s_raw_detents = (int8_t)(s_raw_detents + detents);
	}
}

int8_t Encoder_GetStep(bool is_value_picking)
{
	int8_t delta;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		delta = s_raw_detents;
		s_raw_detents = 0;
	}

	if (delta == 0) {
		/* No new raw pulses, but check existing accumulation */
	} else if (is_value_picking) {
		s_value_accum += delta;
	} else {
		/* Menu navigation mode: reset accumulator if direction reverses */
		if ((delta > 0 && s_menu_accum < 0) || (delta < 0 && s_menu_accum > 0)) {
			s_menu_accum = 0;
		}
		s_menu_accum += delta;
	}

	if (is_value_picking) {
		/* 1 physical detent per step */
		if (s_value_accum >= 1) {
			s_value_accum = 0;
			return 1;
		} else if (s_value_accum <= -1) {
			s_value_accum = 0;
			return -1;
		}
	} else {
		/* 2 physical detents per menu step */
		if (s_menu_accum >= 2) {
			s_menu_accum = 0;
			return 1;
		} else if (s_menu_accum <= -2) {
			s_menu_accum = 0;
			return -1;
		}
	}

	return 0;
}

int8_t encoder_get_step(bool is_value_picking)
{
	return Encoder_GetStep(is_value_picking);
}

encoder_event_t encoder_poll(void)
{
	uint32_t now = sys_millis();
	bool pressed = (ENCODER_PIN & _BV(ENCODER_SWITCH_BIT)) == 0u;

	if (pressed) {
		if (s_press_samples < 255u) {
			s_press_samples++;
		}
		if (s_press_samples >= ENCODER_BUTTON_DEBOUNCE_MS && !s_btn_latched) {
			s_btn_latched = true;
			/* If button pressed while a click is pending within 400ms -> double press! */
			if (s_click_pending && (now - s_release_millis <= ENCODER_DOUBLE_CLICK_MS)) {
				s_click_pending = false;
				s_suppress_click_on_release = true;
				return ENCODER_EVENT_DOUBLE_PRESS;
			}
		}
	} else {
		s_press_samples = 0u;
		if (s_btn_latched) {
			s_btn_latched = false;
			if (s_suppress_click_on_release) {
				s_suppress_click_on_release = false;
			} else {
				s_click_pending = true;
				s_release_millis = now;
			}
		}
	}

	/* If 400ms passed since release with no second click -> single press */
	if (s_click_pending && (now - s_release_millis >= ENCODER_DOUBLE_CLICK_MS)) {
		s_click_pending = false;
		return ENCODER_EVENT_PRESS;
	}

	return ENCODER_EVENT_NONE;
}
