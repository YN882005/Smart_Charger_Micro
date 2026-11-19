#include "OLED_Screen.h"

#include <avr/pgmspace.h>
#include <stdio.h>
#include <string.h>

#include "Buzzer.h"
#include "Charging.h"
#include "Discharging.h"
#include "Storage.h"
#include "Capacity.h"
#include "Encoder.h"
#include "config.h"
#include "twi.h"

/* Externs from main.c */
extern bool g_charging_active[TOTAL_CHANNELS];
extern bool g_discharging_active[TOTAL_CHANNELS];
extern bool g_storage_active[TOTAL_CHANNELS];
extern bool g_capacity_active[TOTAL_CHANNELS];

#define SSD1306_WIDTH      128u
#define SSD1306_PAGE_COUNT 8u
#define BATTERY_WIDTH      20u

/* -------------------------------------------------------------------------
 * PROGMEM 5x7 ASCII Font (subset covering 0-9, A-Z, punctuation)
 * ------------------------------------------------------------------------- */
static const uint8_t font5x7[][5] PROGMEM = {
	{0x00, 0x00, 0x00, 0x00, 0x00}, /* 0: space */
	{0x7E, 0x11, 0x11, 0x11, 0x7E}, /* 1: A */
	{0x7F, 0x49, 0x49, 0x49, 0x36}, /* 2: B */
	{0x3E, 0x41, 0x41, 0x41, 0x22}, /* 3: C */
	{0x7F, 0x41, 0x41, 0x22, 0x1C}, /* 4: D */
	{0x7F, 0x49, 0x49, 0x49, 0x41}, /* 5: E */
	{0x7F, 0x09, 0x09, 0x09, 0x01}, /* 6: F */
	{0x3E, 0x41, 0x49, 0x49, 0x7A}, /* 7: G */
	{0x7F, 0x08, 0x08, 0x08, 0x7F}, /* 8: H */
	{0x00, 0x41, 0x7F, 0x41, 0x00}, /* 9: I */
	{0x20, 0x40, 0x41, 0x3F, 0x01}, /* 10: J */
	{0x7F, 0x08, 0x14, 0x22, 0x41}, /* 11: K */
	{0x7F, 0x40, 0x40, 0x40, 0x40}, /* 12: L */
	{0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* 13: M */
	{0x7F, 0x04, 0x08, 0x10, 0x7F}, /* 14: N */
	{0x3E, 0x41, 0x41, 0x41, 0x3E}, /* 15: O */
	{0x7F, 0x09, 0x09, 0x09, 0x06}, /* 16: P */
	{0x3E, 0x41, 0x51, 0x21, 0x5E}, /* 17: Q */
	{0x7F, 0x09, 0x19, 0x29, 0x46}, /* 18: R */
	{0x46, 0x49, 0x49, 0x49, 0x31}, /* 19: S */
	{0x01, 0x01, 0x7F, 0x01, 0x01}, /* 20: T */
	{0x3F, 0x40, 0x40, 0x40, 0x3F}, /* 21: U */
	{0x1F, 0x20, 0x40, 0x20, 0x1F}, /* 22: V */
	{0x3F, 0x40, 0x38, 0x40, 0x3F}, /* 23: W */
	{0x63, 0x14, 0x08, 0x14, 0x63}, /* 24: X */
	{0x07, 0x08, 0x70, 0x08, 0x07}, /* 25: Y */
	{0x61, 0x51, 0x49, 0x45, 0x43}, /* 26: Z */
	{0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 27: 0 */
	{0x00, 0x42, 0x7F, 0x40, 0x00}, /* 28: 1 */
	{0x42, 0x61, 0x51, 0x49, 0x46}, /* 29: 2 */
	{0x21, 0x41, 0x45, 0x4B, 0x31}, /* 30: 3 */
	{0x18, 0x14, 0x12, 0x7F, 0x10}, /* 31: 4 */
	{0x27, 0x45, 0x45, 0x45, 0x39}, /* 32: 5 */
	{0x3C, 0x4A, 0x49, 0x49, 0x30}, /* 33: 6 */
	{0x01, 0x71, 0x09, 0x05, 0x03}, /* 34: 7 */
	{0x36, 0x49, 0x49, 0x49, 0x36}, /* 35: 8 */
	{0x06, 0x49, 0x49, 0x29, 0x1E}, /* 36: 9 */
	{0x00, 0x36, 0x36, 0x00, 0x00}, /* 37: : */
	{0x08, 0x08, 0x08, 0x08, 0x08}, /* 38: - */
	{0x00, 0x60, 0x60, 0x00, 0x00}, /* 39: . */
	{0x00, 0x41, 0x22, 0x14, 0x08}, /* 40: > */
	{0x00, 0x1C, 0x22, 0x41, 0x00}, /* 41: ( */
	{0x00, 0x41, 0x22, 0x1C, 0x00}, /* 42: ) */
	{0x00, 0x7F, 0x41, 0x41, 0x00}, /* 43: [ */
	{0x00, 0x41, 0x41, 0x7F, 0x00}, /* 44: ] */
	{0x00, 0x08, 0x3E, 0x08, 0x00}, /* 45: + */
	{0x23, 0x13, 0x08, 0x64, 0x62}, /* 46: % */
	{0x20, 0x10, 0x08, 0x04, 0x02}, /* 47: / */
	{0x14, 0x08, 0x3E, 0x08, 0x14}, /* 48: * */
	{0x00, 0x00, 0x5F, 0x00, 0x00}, /* 49: ! */
	{0x02, 0x01, 0x51, 0x09, 0x06}  /* 50: ? */
};

/* -------------------------------------------------------------------------
 * 20x40 Missing-Battery Bitmap in PROGMEM
 * ------------------------------------------------------------------------- */
/* X eyes are lowered and horizontally centered with the caret-shaped frown. */
static const uint8_t bmp_battery_dead[5][BATTERY_WIDTH] PROGMEM = {
	/* Page 1 (top cap) */
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0x00,0x00,0x00,0x00,0x00,0x00},
	/* Page 2 (top rim and lowered X eyes) */
	{0xFF,0x01,0x01,0x89,0x51,0x21,0x51,0x89,0x01,0x01,0x01,0x01,0x89,0x51,0x21,0x51,0x89,0x01,0x01,0xFF},
	/* Page 3 (clear space between the eyes and mouth) */
	{0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
	/* Page 4 (centered caret-shaped frown) */
	{0xFF,0x00,0x00,0x00,0x00,0x08,0x08,0x04,0x04,0x02,0x02,0x04,0x04,0x08,0x08,0x00,0x00,0x00,0x00,0xFF},
	/* Page 5 (bottom rim) */
	{0xFF,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0xFF}
};

static const uint8_t bmp_battery_segment_masks[5][5] PROGMEM = {
	/* Segment 1 (bottom), pages 1..5 */
	{0x00,0x00,0x00,0x00,0x3Eu},
	/* Segment 2 */
	{0x00,0x00,0x00,0xF8u,0x00},
	/* Segment 3 */
	{0x00,0x00,0xE0u,0x03u,0x00},
	/* Segment 4 */
	{0x00,0x80u,0x0Fu,0x00,0x00},
	/* Segment 5 (top) */
	{0x00,0x3Cu,0x00,0x00,0x00}
};

/* -------------------------------------------------------------------------
 * UI Screen State Machine
 * ------------------------------------------------------------------------- */
typedef enum {
	UI_SCREEN_MAIN = 0,          /* Menu 1: Default Main Split-Screen Dashboard */
	UI_SCREEN_CHANNEL,           /* Menu 2: Channel Select ([CH1] / [CH2]) */
	UI_SCREEN_FUNCTION,          /* Menu 3: Function Menu ([Charge], [Discharge], [Storage], [Capacity Test], [Stop]) */
	UI_SCREEN_SUB_CHARGE,        /* Sub-Menu 3A: Charge Current Limit */
	UI_SCREEN_SUB_DISCHARGE,     /* Sub-Menu 3B: Cutoff Voltage with wrap-around */
	UI_SCREEN_SUB_STORAGE,       /* Sub-Menu 3C: Storage Current Limit */
	UI_SCREEN_SUB_CAPACITY,      /* Sub-Menu 3D: Capacity Test Charge Current */
	UI_SCREEN_CAPACITY_RESULT    /* Menu 4: Conditional Capacity Analyzer Screen */
} ui_screen_t;

static ui_screen_t current_screen = UI_SCREEN_MAIN;
static uint8_t selected_channel = 0u;

static uint16_t selected_current_limit[INA233_CHANNEL_COUNT] = {
	CHG_CURRENT_DEFAULT_MA, CHG_CURRENT_DEFAULT_MA
};
static uint8_t selected_cutoff_index[INA233_CHANNEL_COUNT] = {0u, 0u};
static uint8_t g_function_index = 0u;

/* Animation timers */
static uint32_t g_anim_300ms_timer = 0u;
static bool     g_trickle_blink = false;

static uint32_t g_anim_350ms_timer = 0u;
static uint8_t  g_chg_flow_step = 0u;

static uint32_t g_anim_500ms_timer = 0u;
static bool     g_dsg_blink = false;

/* Elapsed cycle tracking */
static uint32_t g_cycle_start_millis[INA233_CHANNEL_COUNT] = {0u, 0u};
static bool     g_cycle_running[INA233_CHANNEL_COUNT] = {false, false};
static uint32_t g_cycle_locked_time[INA233_CHANNEL_COUNT] = {0u, 0u};

static uint8_t page_buffer[SSD1306_WIDTH];
static bool display_ready = false;

static uint8_t glyph_index(char character)
{
	if (character >= 'a' && character <= 'z') {
		character = (char)(character - 'a' + 'A');
	}
	if (character == ' ') return 0u;
	if (character >= 'A' && character <= 'Z') return (uint8_t)(character - 'A' + 1);
	if (character >= '0' && character <= '9') return (uint8_t)(character - '0' + 27);
	if (character == ':') return 37u;
	if (character == '-') return 38u;
	if (character == '.') return 39u;
	if (character == '>') return 40u;
	if (character == '(') return 41u;
	if (character == ')') return 42u;
	if (character == '[') return 43u;
	if (character == ']') return 44u;
	if (character == '+') return 45u;
	if (character == '%') return 46u;
	if (character == '/') return 47u;
	if (character == '*') return 48u;
	if (character == '!') return 49u;
	if (character == '?') return 50u;
	return 0u;
}

static void draw_text_to_page_part(uint8_t col_start, const char *str, uint8_t vertical_offset, bool overflow)
{
	uint8_t col = col_start;
	while (*str != '\0' && (uint16_t)col + 5u <= SSD1306_WIDTH) {
		uint8_t glyph = glyph_index(*str++);
		for (uint8_t i = 0u; i < 5u; ++i) {
			uint8_t column = pgm_read_byte(&font5x7[glyph][i]);
			page_buffer[col++] = overflow
				? (uint8_t)(column >> (8u - vertical_offset))
				: (uint8_t)(column << vertical_offset);
		}
		if (col < SSD1306_WIDTH) {
			page_buffer[col++] = 0x00u;
		}
	}
}

static void draw_text_to_page_offset(uint8_t col_start, const char *str, uint8_t vertical_offset)
{
	draw_text_to_page_part(col_start, str, vertical_offset, false);
}

static void draw_text_to_page(uint8_t col_start, const char *str)
{
	draw_text_to_page_offset(col_start, str, 0u);
}

static void draw_text_p_to_page(uint8_t col_start, PGM_P str_p)
{
	uint8_t col = col_start;
	char c;
	while ((c = pgm_read_byte(str_p++)) != '\0' && (uint16_t)col + 5u <= SSD1306_WIDTH) {
		uint8_t glyph = glyph_index(c);
		for (uint8_t i = 0u; i < 5u; ++i) {
			page_buffer[col++] = pgm_read_byte(&font5x7[glyph][i]);
		}
		if (col < SSD1306_WIDTH) {
			page_buffer[col++] = 0x00u;
		}
	}
}

static bool flush_page(uint8_t page)
{
	uint8_t cmd[3];
	cmd[0] = (uint8_t)(0xB0u | (page & 0x07u));
	cmd[1] = 0x00u; /* lower column start = 0 */
	cmd[2] = 0x10u; /* higher column start = 0 */

	if (!twi_write_packet(OLED_I2C_ADDRESS, 0x00u, cmd, 3u)) {
		return false;
	}
	return twi_write_packet(OLED_I2C_ADDRESS, 0x40u, page_buffer, SSD1306_WIDTH);
}

/* -------------------------------------------------------------------------
 * SoC Lookup
 * ------------------------------------------------------------------------- */
static uint8_t calc_soc(uint8_t channel, uint16_t voltage_mv)
{
	const uint16_t *lut_v = (channel == 0u) ? CH1_SOC_VOLTAGE_MV : CH2_SOC_VOLTAGE_MV;
	const uint8_t  *lut_p = (channel == 0u) ? CH1_SOC_PERCENT    : CH2_SOC_PERCENT;
	uint8_t count         = (channel == 0u) ? CH1_SOC_LUT_COUNT   : CH2_SOC_LUT_COUNT;

	for (uint8_t i = 0u; i < count; ++i) {
		uint16_t v_point = pgm_read_word(&lut_v[i]);
		if (voltage_mv <= v_point) {
			if (i == 0u) return 0u;
			uint16_t v_prev = pgm_read_word(&lut_v[i - 1]);
			uint8_t  p_prev = pgm_read_byte(&lut_p[i - 1]);
			uint8_t  p_curr = pgm_read_byte(&lut_p[i]);
			if (v_point == v_prev) return p_curr;
			return (uint8_t)(p_prev + ((uint32_t)(voltage_mv - v_prev) * (p_curr - p_prev)) / (v_point - v_prev));
		}
	}
	return 100u;
}

static uint8_t soc_to_segments(uint8_t soc)
{
	if (soc >= 85u) return 5u;
	if (soc >= 65u) return 4u;
	if (soc >= 45u) return 3u;
	if (soc >= 25u) return 2u;
	if (soc >= 5u)  return 1u;
	return 0u;
}

static void format_timer_str(char *buf, uint8_t bufsize, uint32_t ms)
{
	uint32_t total_s = ms / 1000u;
	uint32_t hours   = total_s / 3600u;
	uint32_t mins    = (total_s % 3600u) / 60u;
	uint32_t secs    = total_s % 60u;

	if (hours > 0u) {
		snprintf(buf, bufsize, "%lu:%02lu:%02lu", (unsigned long)hours, (unsigned long)mins, (unsigned long)secs);
	} else {
		snprintf(buf, bufsize, "%02lu:%02lu", (unsigned long)mins, (unsigned long)secs);
	}
}

static void set_screen(ui_screen_t next)
{
	if (current_screen != next) {
		current_screen = next;
		Encoder_ResetAccumulator();
		Buzzer_PlayMenuBeep();
	}
}

/* -------------------------------------------------------------------------
 * Battery Icon Rendering (20x40 pixels across pages 1..5)
 * ------------------------------------------------------------------------- */
static void render_battery_icon_page(uint8_t channel, uint8_t page, uint8_t col_start,
	bool is_disconnected, charging_state_t chg, discharging_state_t dsg, uint8_t seg_count)
{
	(void)channel;
	uint8_t p_idx = page - 1u; /* 0..4 */

	if (is_disconnected) {
		/* Render dead/missing broken battery bitmap */
		for (uint8_t c = 0u; c < BATTERY_WIDTH; ++c) {
			page_buffer[col_start + c] = pgm_read_byte(&bmp_battery_dead[p_idx][c]);
		}
		return;
	}

	/* Keep the battery silhouette aligned with the missing-battery icon. */
	for (uint8_t c = 0u; c < BATTERY_WIDTH; ++c) {
		uint8_t b = 0x00u;

		if (p_idx == 0u) {
			if (c >= 7u && c <= 13u) b |= 0xE0u;
		} else if (p_idx == 4u) {
			if (c == 0u || c == 19u) b |= 0xFFu;
			else b |= 0x80u;
		} else {
			if (c == 0u || c == 19u) b |= 0xFFu;
			else if (p_idx == 1u) b |= 0x01u;
		}

		for (uint8_t seg_num = 1u; seg_num <= 5u; ++seg_num) {
			bool fill_bar = false;

			if (chg == CHG_STATE_TRICKLE) {
				if (seg_num == 1u && g_trickle_blink) {
					fill_bar = true;
				}
			} else if (chg == CHG_STATE_CC || chg == CHG_STATE_CV) {
				if (seg_num <= seg_count) {
					fill_bar = true;
				} else {
					uint8_t flow_bar = seg_count + 1u + g_chg_flow_step;
					if (flow_bar > 5u) flow_bar = seg_count + 1u;
					if (seg_num == flow_bar) {
						fill_bar = true;
					}
				}
			} else if (dsg == DSG_STATE_ACTIVE) {
				if (seg_num < seg_count) {
					fill_bar = true;
				} else if (seg_num == seg_count && g_dsg_blink) {
					fill_bar = true;
				}
			} else if (seg_num <= seg_count) {
				fill_bar = true;
			}

			if (fill_bar && c >= 2u && c <= 17u) {
				b |= pgm_read_byte(&bmp_battery_segment_masks[seg_num - 1u][p_idx]);
			}
		}

		page_buffer[col_start + c] = b;
	}
}

/* -------------------------------------------------------------------------
 * MENU 1: Split Screen Dashboard Rendering
 * ------------------------------------------------------------------------- */
static bool render_dashboard_screen(const bool sample_valid[INA233_CHANNEL_COUNT],
	const ina233_raw_sample_t samples[INA233_CHANNEL_COUNT])
{
	uint32_t now = sys_millis();

	/* Animation state machine timers */
	if (now - g_anim_300ms_timer >= 300u) {
		g_anim_300ms_timer = now;
		g_trickle_blink = !g_trickle_blink;
	}
	if (now - g_anim_350ms_timer >= 350u) {
		g_anim_350ms_timer = now;
		g_chg_flow_step = (g_chg_flow_step + 1u) % 5u;
	}
	if (now - g_anim_500ms_timer >= 500u) {
		g_anim_500ms_timer = now;
		g_dsg_blink = !g_dsg_blink;
	}

	/* Channel state evaluations */
	bool is_disc[2];
	uint16_t v_mv[2];
	uint8_t soc[2];
	uint8_t segs[2];
	const char *badge[2];

	for (uint8_t ch = 0u; ch < 2u; ++ch) {
		charging_state_t cs = charging_get_state(ch);
		discharging_state_t ds = discharging_get_state(ch);
		storage_state_t ss = storage_get_state(ch);
		capacity_state_t ks = capacity_get_state(ch);

		bool active_power = charging_is_active(ch) || discharging_is_active(ch);
		uint16_t mv = 0u;
		if (sample_valid[ch]) {
			mv = (uint16_t)(samples[ch].bus_voltage_v * 1000.0f);
		}
		v_mv[ch] = mv;

		/* Disconnected: FETs OFF and V_bat < 1.00V */
		if (!active_power && mv < BATT_PRESENCE_THRESHOLD_MV) {
			is_disc[ch] = true;
			soc[ch] = 0u;
			segs[ch] = 0u;
			badge[ch] = "DISC";
			g_cycle_running[ch] = false;
			g_cycle_locked_time[ch] = 0u;
		} else {
			is_disc[ch] = false;
			soc[ch] = calc_soc(ch, mv);
			segs[ch] = soc_to_segments(soc[ch]);

			bool done = charging_is_done(ch) || discharging_is_done(ch)
				|| storage_is_done(ch) || capacity_is_done(ch);

			if (done) {
				badge[ch] = "DONE";
				if (g_cycle_running[ch]) {
					g_cycle_running[ch] = false;
					g_cycle_locked_time[ch] = now - g_cycle_start_millis[ch];
				}
			} else if (ks == CAP_STATE_STAGE1_DISCHARGE || ks == CAP_STATE_STAGE2_CHARGE) {
				badge[ch] = "CAP";
			} else if (ss == STG_STATE_SUB_CHARGE || ss == STG_STATE_SUB_DISCHARGE || ss == STG_STATE_ASSESS) {
				badge[ch] = "STR";
			} else if (cs == CHG_STATE_TRICKLE) {
				badge[ch] = "TRK";
			} else if (cs == CHG_STATE_CC || cs == CHG_STATE_CV) {
				badge[ch] = "CHG";
			} else if (ds == DSG_STATE_ACTIVE) {
				badge[ch] = "DSG";
			} else {
				badge[ch] = "IDLE";
			}
		}
	}

	/* Render Page-by-Page across all 8 pages */
	for (uint8_t page = 0u; page < SSD1306_PAGE_COUNT; ++page) {
		memset(page_buffer, 0x00, SSD1306_WIDTH);

		/* Vertical dividing line between CH1 (cols 0..62) and CH2 (cols 64..127) */
		page_buffer[63] = 0xFFu;

		if (page == 0u) {
			/* Page 0: Headers and Mode Badges */
			char h1[16];
			char h2[16];
			snprintf(h1, sizeof(h1), "CH1 %-4s", badge[0]);
			snprintf(h2, sizeof(h2), "CH2 %-4s", badge[1]);
			draw_text_to_page(2u, h1);
			draw_text_to_page(66u, h2);
		} else if (page >= 1u && page <= 5u) {
			/* Pages 1..5: Animated Battery Icons (20x40 px) + Live Telemetry */
			render_battery_icon_page(0u, page, 4u, is_disc[0],
				charging_get_state(0), discharging_get_state(0), segs[0]);
			render_battery_icon_page(1u, page, 68u, is_disc[1],
				charging_get_state(1), discharging_get_state(1), segs[1]);

			/* Live Voltage and SoC text beside icons */
			if (page == 2u) {
				char txt1[10], txt2[10];
				if (!is_disc[0]) snprintf(txt1, sizeof(txt1), "%u.%02uV", v_mv[0]/1000u, (v_mv[0]%1000u)/10u);
				else snprintf(txt1, sizeof(txt1), "--.--V");
				if (!is_disc[1]) snprintf(txt2, sizeof(txt2), "%u.%02uV", v_mv[1]/1000u, (v_mv[1]%1000u)/10u);
				else snprintf(txt2, sizeof(txt2), "--.--V");
				draw_text_to_page(26u, txt1);
				draw_text_to_page(90u, txt2);
			} else if (page == 4u) {
				char txt1[10], txt2[10];
				if (!is_disc[0]) snprintf(txt1, sizeof(txt1), "%3u%%", soc[0]);
				else snprintf(txt1, sizeof(txt1), "NO BAT");
				if (!is_disc[1]) snprintf(txt2, sizeof(txt2), "%3u%%", soc[1]);
				else snprintf(txt2, sizeof(txt2), "NO BAT");
				draw_text_to_page(26u, txt1);
				draw_text_to_page(90u, txt2);
			}
		} else if (page == 6u || page == 7u) {
			/* Pages 6..7: Cycle Timers shifted three pixels below page 6's start */
			char t1[12], t2[12];
			uint32_t ms1 = g_cycle_running[0] ? (now - g_cycle_start_millis[0]) : g_cycle_locked_time[0];
			uint32_t ms2 = g_cycle_running[1] ? (now - g_cycle_start_millis[1]) : g_cycle_locked_time[1];
			format_timer_str(t1, sizeof(t1), ms1);
			format_timer_str(t2, sizeof(t2), ms2);
			uint8_t t1_width = (uint8_t)(strlen(t1) * 6u);
			uint8_t t2_width = (uint8_t)(strlen(t2) * 6u);
			uint8_t t1_col = (uint8_t)((64u - t1_width) / 2u);
			uint8_t t2_col = (uint8_t)(64u + (64u - t2_width) / 2u);
			bool overflow = page == 7u;
			draw_text_to_page_part(t1_col, t1, 3u, overflow);
			draw_text_to_page_part(t2_col, t2, 3u, overflow);
		}

		if (!flush_page(page)) {
			return false;
		}
	}

	return true;
}

/* -------------------------------------------------------------------------
 * Menu Screens Rendering
 * ------------------------------------------------------------------------- */
static bool render_menu_screen(void)
{
	char line[22];
	uint8_t page_offset = (current_screen == UI_SCREEN_CAPACITY_RESULT) ? 0u : 1u;

	for (uint8_t page = 0u; page < SSD1306_PAGE_COUNT; ++page) {
		uint8_t content_page = page;
		if (page_offset != 0u) {
			content_page = (page == 0u) ? UINT8_MAX : (uint8_t)(page - page_offset);
		}
		memset(page_buffer, 0x00, SSD1306_WIDTH);

		switch (current_screen) {
		case UI_SCREEN_CHANNEL:
			if (content_page == 0u) draw_text_to_page(16u, "[SELECT CHANNEL]");
			else if (content_page == 2u) draw_text_to_page(8u, (selected_channel == 0u) ? "> [CH1] Li-Ion" : "  [CH1] Li-Ion");
			else if (content_page == 4u) draw_text_to_page(8u, (selected_channel == 1u) ? "> [CH2] LiFePO4" : "  [CH2] LiFePO4");
			break;

		case UI_SCREEN_FUNCTION:
			if (content_page == 0u) {
				snprintf(line, sizeof(line), "CH%u FUNCTION MENU", selected_channel + 1u);
				draw_text_to_page(4u, line);
			} else if (content_page >= 1u && content_page <= 5u) {
				static const char f0[] PROGMEM = "[Charge]";
				static const char f1[] PROGMEM = "[Discharge]";
				static const char f2[] PROGMEM = "[Storage]";
				static const char f3[] PROGMEM = "[Capacity Test]";
				static const char f4[] PROGMEM = "[Stop / Idle]";
				static const char *const f_names[] PROGMEM = { f0, f1, f2, f3, f4 };
				uint8_t f_idx = content_page - 1u;
				draw_text_to_page(6u, (g_function_index == f_idx) ? "> " : "  ");
				draw_text_p_to_page(18u, (PGM_P)pgm_read_word(&f_names[f_idx]));
			}
			break;

		case UI_SCREEN_SUB_CHARGE:
			if (content_page == 0u) {
				snprintf(line, sizeof(line), "CH%u CHARGE CURRENT", selected_channel + 1u);
				draw_text_to_page(4u, line);
			} else if (content_page == 2u) {
				snprintf(line, sizeof(line), "CURRENT: %u mA", selected_current_limit[selected_channel]);
				draw_text_to_page(10u, line);
			} else if (content_page == 6u) {
				draw_text_to_page(8u, "(100 .. 1200 mA)");
			}
			break;

		case UI_SCREEN_SUB_DISCHARGE:
			if (content_page == 0u) {
				snprintf(line, sizeof(line), "CH%u DSG CUTOFF", selected_channel + 1u);
				draw_text_to_page(10u, line);
			} else if (content_page == 2u) {
				uint16_t cut = discharging_get_cutoff_menu_item(selected_channel, selected_cutoff_index[selected_channel]);
				if (selected_cutoff_index[selected_channel] == 0u) {
					snprintf(line, sizeof(line), "TARGET: MAX (%u.%02uV)", cut/1000u, (cut%1000u)/10u);
				} else {
					snprintf(line, sizeof(line), "TARGET: %u.%02u V", cut/1000u, (cut%1000u)/10u);
				}
				draw_text_to_page(6u, line);
			}
			break;

		case UI_SCREEN_SUB_STORAGE:
			if (content_page == 0u) {
				snprintf(line, sizeof(line), "CH%u STORAGE MODE", selected_channel + 1u);
				draw_text_to_page(10u, line);
			} else if (content_page == 2u) {
				uint16_t v_tgt = (selected_channel == 0u) ? CH1_STORAGE_MV : CH2_STORAGE_MV;
				snprintf(line, sizeof(line), "TARGET: %u.%02u V", v_tgt/1000u, (v_tgt%1000u)/10u);
				draw_text_to_page(12u, line);
			} else if (content_page == 4u) {
				snprintf(line, sizeof(line), "CHG LIMIT: %u mA", selected_current_limit[selected_channel]);
				draw_text_to_page(8u, line);
			}
			break;

		case UI_SCREEN_SUB_CAPACITY:
			if (content_page == 0u) {
				snprintf(line, sizeof(line), "CH%u CAPACITY TEST", selected_channel + 1u);
				draw_text_to_page(4u, line);
			} else if (content_page == 2u) {
				draw_text_to_page(8u, "STAGE 1: PRE-DSG");
			} else if (content_page == 3u) {
				snprintf(line, sizeof(line), "STAGE 2: CHG %u mA", selected_current_limit[selected_channel]);
				draw_text_to_page(8u, line);
			}
			break;

		case UI_SCREEN_CAPACITY_RESULT:
			if (content_page == 0u) {
				snprintf(line, sizeof(line), "CH%u CAPACITY RESULT", selected_channel + 1u);
				draw_text_to_page(4u, line);
			} else if (content_page == 2u) {
				capacity_state_t cs = capacity_get_state(selected_channel);
				const char *st_str = "COMPLETE";
				if (cs == CAP_STATE_STAGE1_DISCHARGE) st_str = "STAGE 1 DSG";
				else if (cs == CAP_STATE_STAGE2_CHARGE) st_str = "STAGE 2 CHG";
				snprintf(line, sizeof(line), "STAGE: %s", st_str);
				draw_text_to_page(6u, line);
			} else if (content_page == 3u) {
				char tbuf[12];
				format_timer_str(tbuf, sizeof(tbuf), capacity_get_stage2_elapsed_ms(selected_channel));
				snprintf(line, sizeof(line), "TIME:  %s", tbuf);
				draw_text_to_page(6u, line);
			} else if (content_page == 4u) {
				float wh = capacity_measured_energy_wh(selected_channel);
				uint16_t wh_int = (uint16_t)wh;
				uint16_t wh_dec = (uint16_t)((wh - (float)wh_int) * 100.0f);
				snprintf(line, sizeof(line), "ENERGY: %u.%02u Wh", wh_int, wh_dec);
				draw_text_to_page(6u, line);
			} else if (content_page == 5u) {
				uint32_t mah = capacity_measured_mah(selected_channel);
				snprintf(line, sizeof(line), "CAP:    %lu mAh", (unsigned long)mah);
				draw_text_to_page(6u, line);
			} else if (content_page == 7u) {
				draw_text_to_page(36u, "[ PRESS OK ]");
			}
			break;

		default:
			break;
		}

		if (!flush_page(page)) {
			return false;
		}
	}

	return true;
}

/* -------------------------------------------------------------------------
 * Public Driver API
 * ------------------------------------------------------------------------- */
bool oled_screen_clear(void)
{
	memset(page_buffer, 0x00, SSD1306_WIDTH);
	for (uint8_t page = 0u; page < SSD1306_PAGE_COUNT; ++page) {
		if (!flush_page(page)) {
			return false;
		}
	}
	return true;
}

bool oled_screen_init(void)
{
	static const uint8_t init_cmds[] PROGMEM = {
		0xAEu,       /* Display OFF */
		0xD5u, 0x80u,/* Set display clock divide ratio/oscillator freq */
		0xA8u, 0x3Fu,/* Set multiplex ratio (64) */
		0xD3u, 0x00u,/* Set display offset */
		0x40u,       /* Set start line address */
		0x8Du, 0x14u,/* Enable charge pump */
		0x20u, 0x02u,/* Page addressing mode */
		0xA1u,       /* Set segment re-map (col 127 mapped to SEG0) */
		0xC8u,       /* Set COM output scan direction (remapped) */
		0xDAu, 0x12u,/* Set COM pins hardware config */
		0x81u, 0xCFu,/* Set contrast control */
		0xD9u, 0xF1u,/* Set pre-charge period */
		0xDBu, 0x40u,/* Set VCOMH deselect level */
		0xA4u,       /* Entire display ON from RAM */
		0xA6u,       /* Normal display (non-inverted) */
		0xAFu        /* Display ON */
	};

	uint8_t buf[sizeof(init_cmds)];
	for (uint8_t i = 0u; i < sizeof(init_cmds); ++i) {
		buf[i] = pgm_read_byte(&init_cmds[i]);
	}

	if (!twi_write_packet(OLED_I2C_ADDRESS, 0x00u, buf, sizeof(init_cmds))) {
		display_ready = false;
		return false;
	}

	current_screen = UI_SCREEN_MAIN;
	selected_channel = 0u;
	g_function_index = 0u;
	selected_current_limit[0] = CHG_CURRENT_DEFAULT_MA;
	selected_current_limit[1] = CHG_CURRENT_DEFAULT_MA;
	selected_cutoff_index[0] = 0u;
	selected_cutoff_index[1] = 0u;

	display_ready = oled_screen_clear();
	return display_ready;
}

bool oled_screen_render(const bool sample_valid[INA233_CHANNEL_COUNT],
	const ina233_raw_sample_t samples[INA233_CHANNEL_COUNT], bool alert_active)
{
	(void)alert_active;
	if (!display_ready) {
		return false;
	}

	if (current_screen == UI_SCREEN_MAIN) {
		return render_dashboard_screen(sample_valid, samples);
	} else {
		return render_menu_screen();
	}
}

/* -------------------------------------------------------------------------
 * Encoder Event Router
 * ------------------------------------------------------------------------- */
void oled_screen_handle_encoder_event(encoder_event_t event)
{
	/* GLOBAL SHORTCUT: double-press from ANY screen returns to Menu 1 */
	if (event == ENCODER_EVENT_DOUBLE_PRESS) {
		set_screen(UI_SCREEN_MAIN);
		return;
	}

	/* -------------------------------------------------------------
	 * 1. ROTATION HANDLING (DUAL-SENSITIVITY)
	 * ------------------------------------------------------------- */
	bool is_value_screen = (current_screen == UI_SCREEN_SUB_CHARGE
		|| current_screen == UI_SCREEN_SUB_DISCHARGE
		|| current_screen == UI_SCREEN_SUB_STORAGE
		|| current_screen == UI_SCREEN_SUB_CAPACITY);

	int8_t step = Encoder_GetStep(is_value_screen);

	if (step != 0) {
		Buzzer_PlayMenuBeep();

		if (current_screen == UI_SCREEN_MAIN) {
			/* From Menu 1, rotate into Menu 2 (or Menu 4 if capacity test is active/complete) */
			if (capacity_menu_enabled(selected_channel)) {
				set_screen(UI_SCREEN_CAPACITY_RESULT);
			} else {
				set_screen(UI_SCREEN_CHANNEL);
			}
		} else if (current_screen == UI_SCREEN_CHANNEL) {
			if (step > 0) {
				selected_channel = (selected_channel + 1u) % ACTIVE_CHANNELS;
			} else {
				selected_channel = (selected_channel + ACTIVE_CHANNELS - 1u) % ACTIVE_CHANNELS;
			}
		} else if (current_screen == UI_SCREEN_FUNCTION) {
			if (step > 0) {
				g_function_index = (g_function_index + 1u) % 5u;
			} else {
				g_function_index = (g_function_index + 4u) % 5u;
			}
		} else if (current_screen == UI_SCREEN_SUB_CHARGE
			|| current_screen == UI_SCREEN_SUB_STORAGE
			|| current_screen == UI_SCREEN_SUB_CAPACITY) {
			/* 1 tick per 100 mA step */
			if (step > 0 && selected_current_limit[selected_channel] < CHG_CURRENT_MAX_MA) {
				selected_current_limit[selected_channel] += CHG_CURRENT_STEP_MA;
			} else if (step < 0 && selected_current_limit[selected_channel] > CHG_CURRENT_MIN_MA) {
				selected_current_limit[selected_channel] -= CHG_CURRENT_STEP_MA;
			}
		} else if (current_screen == UI_SCREEN_SUB_DISCHARGE) {
			/* 1 tick per voltage step with circular wrap-around */
			uint8_t count = discharging_get_cutoff_menu_count();
			if (step > 0) {
				selected_cutoff_index[selected_channel] =
					(selected_cutoff_index[selected_channel] + 1u) % count;
			} else {
				selected_cutoff_index[selected_channel] =
					(selected_cutoff_index[selected_channel] + count - 1u) % count;
			}
		} else if (current_screen == UI_SCREEN_CAPACITY_RESULT) {
			/* Can cycle back to main */
			set_screen(UI_SCREEN_MAIN);
		}
	}

	/* -------------------------------------------------------------
	 * 2. PUSH-BUTTON (PRESS) HANDLING
	 * ------------------------------------------------------------- */
	if (event != ENCODER_EVENT_PRESS) {
		return;
	}

	Buzzer_PlayMenuBeep();

	switch (current_screen) {
	case UI_SCREEN_MAIN:
		set_screen(UI_SCREEN_CHANNEL);
		break;

	case UI_SCREEN_CHANNEL:
		set_screen(UI_SCREEN_FUNCTION);
		break;

	case UI_SCREEN_FUNCTION:
		switch (g_function_index) {
		case 0u: set_screen(UI_SCREEN_SUB_CHARGE); break;
		case 1u: set_screen(UI_SCREEN_SUB_DISCHARGE); break;
		case 2u: set_screen(UI_SCREEN_SUB_STORAGE); break;
		case 3u: set_screen(UI_SCREEN_SUB_CAPACITY); break;
		case 4u: /* Stop / Idle */
			charging_stop(selected_channel);
			discharging_stop(selected_channel);
			storage_stop(selected_channel);
			capacity_stop(selected_channel);
			g_charging_active[selected_channel] = false;
			g_discharging_active[selected_channel] = false;
			g_storage_active[selected_channel] = false;
			g_capacity_active[selected_channel] = false;
			g_cycle_running[selected_channel] = false;
			set_screen(UI_SCREEN_MAIN);
			break;
		default:
			break;
		}
		break;

	case UI_SCREEN_SUB_CHARGE:
		/* Confirm charge current and start */
		g_charging_active[selected_channel] = true;
		charging_start(selected_channel, selected_current_limit[selected_channel],
			(selected_channel == 0u) ? CH1_FULL_CHARGE_MV : CH2_FULL_CHARGE_MV);
		g_cycle_start_millis[selected_channel] = sys_millis();
		g_cycle_running[selected_channel] = true;
		set_screen(UI_SCREEN_MAIN);
		break;

	case UI_SCREEN_SUB_DISCHARGE: {
		/* Confirm discharge cutoff and start */
		uint16_t cutoff = discharging_get_cutoff_menu_item(selected_channel,
			selected_cutoff_index[selected_channel]);
		g_discharging_active[selected_channel] = true;
		discharging_start(selected_channel, cutoff);
		g_cycle_start_millis[selected_channel] = sys_millis();
		g_cycle_running[selected_channel] = true;
		set_screen(UI_SCREEN_MAIN);
		break;
	}

	case UI_SCREEN_SUB_STORAGE:
		/* Confirm storage current limit and start bi-directional conditioning */
		g_storage_active[selected_channel] = true;
		storage_start(selected_channel, selected_current_limit[selected_channel]);
		g_cycle_start_millis[selected_channel] = sys_millis();
		g_cycle_running[selected_channel] = true;
		set_screen(UI_SCREEN_MAIN);
		break;

	case UI_SCREEN_SUB_CAPACITY:
		/* Confirm capacity test Stage 2 current, enable Menu 4, and start Stage 1 */
		g_capacity_active[selected_channel] = true;
		capacity_menu_enable(selected_channel);
		capacity_start(selected_channel, selected_current_limit[selected_channel]);
		g_cycle_start_millis[selected_channel] = sys_millis();
		g_cycle_running[selected_channel] = true;
		set_screen(UI_SCREEN_MAIN);
		break;

	case UI_SCREEN_CAPACITY_RESULT:
		/* Once capacity test is complete and user presses OK -> dismiss Menu 4 */
		if (capacity_test_complete(selected_channel)) {
			capacity_menu_disable(selected_channel);
		}
		set_screen(UI_SCREEN_MAIN);
		break;

	default:
		set_screen(UI_SCREEN_MAIN);
		break;
	}
}
