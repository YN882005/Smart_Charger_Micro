#ifndef BUZZER_H
#define BUZZER_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

void buzzer_init(void);

/* Required prompt API names */
void Buzzer_PlayMenuBeep(void);
void Buzzer_PlayCompleteMelody(void);

/* Convenience / compatibility functions */
void buzzer_beep(uint16_t duration_ms);
void buzzer_play_melody(const buzzer_note_t *notes, uint8_t count);
void buzzer_play_complete_melody(void);
bool buzzer_is_beeping(void);
void buzzer_task(void);

#endif /* BUZZER_H */
