#ifndef OLED_SCREEN_H
#define OLED_SCREEN_H

#include <stdbool.h>
#include <stdint.h>

#include "Encoder.h"
#include "ina233.h"

/* OLED display driver functions */
bool oled_screen_init(void);
bool oled_screen_clear(void);

/* Screen rendering and user-interaction functions */
void oled_screen_handle_encoder_event(encoder_event_t event);
bool oled_screen_render(const bool sample_valid[INA233_CHANNEL_COUNT],
	const ina233_raw_sample_t samples[INA233_CHANNEL_COUNT], bool alert_active);

/* Backwards-compatibility aliases */
#define ssd1306_init oled_screen_init
#define ssd1306_clear oled_screen_clear
#define ui_init oled_screen_init
#define ui_handle_encoder_event oled_screen_handle_encoder_event
#define ui_render oled_screen_render

#endif /* OLED_SCREEN_H */
