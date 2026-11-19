#include "Buzzer.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "config.h"

static volatile uint8_t s_half_period_ticks = 0u;
static volatile bool s_is_active = false;

static const buzzer_note_t *s_melody_notes = 0;
static uint8_t s_melody_count = 0u;
static uint8_t s_melody_index = 0u;
static uint32_t s_note_start_ms = 0u;
static uint16_t s_note_duration_ms = 0u;

/* Standard 4-note celebratory completion melody */
static const buzzer_note_t s_complete_melody[BUZZER_MELODY_NOTE_COUNT] = {
	{ (uint16_t)BUZZER_NOTE_C6_HZ, (uint16_t)BUZZER_NOTE_SHORT_MS },
	{ (uint16_t)BUZZER_NOTE_E6_HZ, (uint16_t)BUZZER_NOTE_SHORT_MS },
	{ (uint16_t)BUZZER_NOTE_G6_HZ, (uint16_t)BUZZER_NOTE_SHORT_MS },
	{ (uint16_t)BUZZER_NOTE_C7_HZ, (uint16_t)BUZZER_NOTE_LONG_MS  }
};

/* Compare Match B ISR: toggles PD4 at the programmed half-period */
ISR(TIMER0_COMPB_vect)
{
	if (!s_is_active || s_half_period_ticks == 0u) {
		TIMSK0 &= (uint8_t)~_BV(OCIE0B);
		PORTD &= (uint8_t)~_BV(BUZZER_BIT);
		return;
	}

	/* Toggle PD4 pin directly using PIN register */
	PIND = _BV(BUZZER_BIT);

	/* Advance OCR0B for next toggle.
	 * Timer0 counts 0 to 229 (CTC mode with TOP=OCR0A=229). */
	uint16_t next = (uint16_t)OCR0B + s_half_period_ticks;
	if (next >= 230u) {
		next -= 230u;
	}
	OCR0B = (uint8_t)next;
}

void buzzer_init(void)
{
	/* Configure PD4 as output and drive LOW initially */
	BUZZER_DDR |= _BV(BUZZER_BIT);
	BUZZER_PORT &= (uint8_t)~_BV(BUZZER_BIT);

	s_half_period_ticks = 0u;
	s_is_active = false;
	s_melody_notes = 0;
	s_melody_count = 0u;
	s_melody_index = 0u;
	s_note_start_ms = 0u;
	s_note_duration_ms = 0u;
}

static void buzzer_start_tone(uint16_t freq_hz, uint16_t duration_ms)
{
	if (freq_hz == 0u || duration_ms == 0u) {
		return;
	}

	/* Calculate half-period in Timer0 ticks:
	 * Timer0 clock = 14,745,600 / 64 = 230,400 Hz.
	 * Half-period ticks = 230,400 / (2 * freq) = 115,200 / freq. */
	uint32_t ticks = 115200UL / (uint32_t)freq_hz;
	if (ticks == 0u) {
		ticks = 1u;
	} else if (ticks > 229u) {
		ticks = 229u;
	}

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		s_half_period_ticks = (uint8_t)ticks;
		s_note_start_ms = sys_millis();
		s_note_duration_ms = duration_ms;
		s_is_active = true;

		/* Initialize OCR0B ahead of current TCNT0 */
		uint16_t next = (uint16_t)TCNT0 + s_half_period_ticks;
		if (next >= 230u) {
			next -= 230u;
		}
		OCR0B = (uint8_t)next;
		TIFR0 |= _BV(OCF0B);
		TIMSK0 |= _BV(OCIE0B);
	}
}

static void buzzer_stop_tone(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		s_is_active = false;
		s_half_period_ticks = 0u;
		TIMSK0 &= (uint8_t)~_BV(OCIE0B);
		PORTD &= (uint8_t)~_BV(BUZZER_BIT);
	}
}

void buzzer_beep(uint16_t duration_ms)
{
	s_melody_notes = 0;
	s_melody_count = 0u;
	s_melody_index = 0u;
	buzzer_start_tone((uint16_t)BUZZER_FREQUENCY_HZ, duration_ms);
}

void Buzzer_PlayMenuBeep(void)
{
	buzzer_beep((uint16_t)BUZZER_BEEP_MS);
}

void buzzer_play_melody(const buzzer_note_t *notes, uint8_t count)
{
	if (notes == 0 || count == 0u) {
		return;
	}

	s_melody_notes = notes;
	s_melody_count = count;
	s_melody_index = 0u;
	buzzer_start_tone(notes[0].freq_hz, notes[0].duration_ms);
}

void Buzzer_PlayCompleteMelody(void)
{
	buzzer_play_melody(s_complete_melody, BUZZER_MELODY_NOTE_COUNT);
}

void buzzer_play_complete_melody(void)
{
	Buzzer_PlayCompleteMelody();
}

bool buzzer_is_beeping(void)
{
	return s_is_active;
}

void buzzer_task(void)
{
	if (!s_is_active) {
		return;
	}

	uint32_t now = sys_millis();
	if (now - s_note_start_ms >= (uint32_t)s_note_duration_ms) {
		if (s_melody_notes != 0 && (s_melody_index + 1u) < s_melody_count) {
			/* Advance to next melody note */
			s_melody_index++;
			buzzer_start_tone(s_melody_notes[s_melody_index].freq_hz,
				s_melody_notes[s_melody_index].duration_ms);
		} else {
			/* Melody or single tone finished */
			s_melody_notes = 0;
			s_melody_count = 0u;
			buzzer_stop_tone();
		}
	}
}
