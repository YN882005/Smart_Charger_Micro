#ifndef CONFIG_H
#define CONFIG_H

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdint.h>

/* ATmega328P-PU external crystal frequency: 14.7456 MHz */
#ifndef F_CPU
#define F_CPU 14745600UL
#endif

/* -------------------------------------------------------------------------
 * I2C / TWI Configuration
 * ------------------------------------------------------------------------- */
#define TWI_CLOCK_HZ           400000UL
#define OLED_I2C_ADDRESS       0x3Cu

/* INA233 7-bit I2C addresses */
#define INA233_CH1_I2C_ADDRESS 0x40u
#define INA233_CH2_I2C_ADDRESS 0x45u
#define INA233_CH3_I2C_ADDRESS 0x41u
#define INA233_CH4_I2C_ADDRESS 0x44u

/* INA233 Register Map (PMBus) */
#define INA233_REG_CONFIG           0x00u
#define INA233_REG_SHUNT_VOLTAGE    0x01u
#define INA233_REG_BUS_VOLTAGE      0x88u
#define INA233_REG_POWER            0x03u
#define INA233_REG_CURRENT          0x89u
#define INA233_REG_CALIBRATION      0xD4u
#define INA233_REG_MASK_ENABLE      0x06u
#define INA233_REG_ALERT_LIMIT      0x07u
#define INA233_REG_MANUFACTURER_ID  0xFEu
#define INA233_REG_DIE_ID           0xFFu
#define INA233_REG_READ_EIN         0x86u
#define INA233_REG_READ_VBUS        0x88u
#define INA233_REG_READ_IIN         0x89u

/* Shunt resistance: 3 x 100 mOhm in parallel = 33.333 mOhm */
#define INA233_R_SHUNT_OHMS    0.033333333f
#define INA233_I_MAX_AMPS      2.0f

/* Power-stage control is enabled only after safe-boot output latching. */
#define POWER_STAGE_CONTROL_ENABLED 1u

/* Shared INA233 Alert line: PD5 (active-low, 5.1 kOhm pull-up) */
#define INA233_ALERT_DDR       DDRD
#define INA233_ALERT_PORT      PORTD
#define INA233_ALERT_PIN       PIND
#define INA233_ALERT_BIT       PD5

/* -------------------------------------------------------------------------
 * Rotary Encoder HW-040 Configuration
 * CLK = PC1, DT = PC2, Switch = PC3 (active-low, 100 nF hardware debounce C17)
 * ------------------------------------------------------------------------- */
#define ENCODER_DDR            DDRC
#define ENCODER_PORT           PORTC
#define ENCODER_PIN            PINC
#define ENCODER_CLK_BIT        PC1
#define ENCODER_DT_BIT         PC2
#define ENCODER_SWITCH_BIT     PC3

/* Encoder sensitivity: 2 ticks per menu step, 1 tick per value step */
#define ENCODER_MENU_DIVIDER   2
#define ENCODER_VALUE_DIVIDER  1

/* -------------------------------------------------------------------------
 * Buzzer Configuration
 * Pin: PD4 (T0) via 47 Ohm series resistor (R46)
 * Target tone: 2.3 kHz (2304 Hz exact with 14.7456 MHz /64 prescaler)
 * Menu transition tone duration: 400 ms
 * ------------------------------------------------------------------------- */
#define BUZZER_DDR             DDRD
#define BUZZER_PORT            PORTD
#define BUZZER_PIN             PIND
#define BUZZER_BIT             PD4
#define BUZZER_FREQUENCY_HZ    2304UL
#define BUZZER_BEEP_MS         400u

/* Buzzer melody note definitions (frequency in Hz, duration in ms) */
typedef struct {
    uint16_t freq_hz;
    uint16_t duration_ms;
} buzzer_note_t;

#define BUZZER_MELODY_NOTE_COUNT 4u
#define BUZZER_NOTE_C6_HZ  1047UL
#define BUZZER_NOTE_E6_HZ  1319UL
#define BUZZER_NOTE_G6_HZ  1568UL
#define BUZZER_NOTE_C7_HZ  2093UL
#define BUZZER_NOTE_SHORT_MS 90u
#define BUZZER_NOTE_LONG_MS  180u

/* -------------------------------------------------------------------------
 * Power Stages Pin Mapping
 * CH1 Charge PWM:     PB1 (OC1A) - Active-LOW P-MOSFET (AO3415)
 * CH1 Discharge Gate: PB0        - Active-HIGH N-MOSFET (SI2300DS)
 * CH2 Charge PWM:     PB2 (OC1B) - Active-LOW P-MOSFET (AO3415)
 * CH2 Discharge Gate: PB5        - Active-HIGH N-MOSFET (SI2300DS)
 *
 * Expansion pins:
 * CH3 Charge PWM:     PB3 (OC2A) - Active-LOW P-MOSFET
 * CH3 Discharge Gate: PB4        - Active-HIGH N-MOSFET
 * CH4 Charge PWM:     PD3 (OC2B) - Active-LOW P-MOSFET
 * CH4 Discharge Gate: PC0        - Active-HIGH N-MOSFET
 * ------------------------------------------------------------------------- */
#define CHARGE1_BIT            PB1
#define CHARGE2_BIT            PB2
#define CHARGE3_BIT            PB3
#define CHARGE4_BIT            PD3

#define DISCHARGE1_BIT         PB0
#define DISCHARGE2_BIT         PB5
#define DISCHARGE3_BIT         PB4
#define DISCHARGE4_BIT         PC0

/* Number of active channels (CH1 Li-Ion + CH2 LiFePO4 enabled by default) */
#define ACTIVE_CHANNELS        2u

/* Total schematic channels */
#define TOTAL_CHANNELS         4u

/* PWM safety clamp: max duty ~94% */
#define PWM_MAX_DUTY           240u

/* -------------------------------------------------------------------------
 * Timer0 System Timebase Configuration
 * 1.000 ms tick: Prescaler=64, OCR0A=229 -> 14,745,600 / (64 * 230) = 1001.7 Hz
 * ------------------------------------------------------------------------- */
#define TIMER0_PRESCALER       64UL
#define TIMER0_OCR0A           229UL
#define TIMER0_TICK_MS         1u

/* -------------------------------------------------------------------------
 * Battery Chemistry & Threshold Limits (in millivolts and milliamperes)
 * ------------------------------------------------------------------------- */
#define BATT_PRESENCE_THRESHOLD_MV 1000u /* < 1.0 V treated as disconnected */

/* Per-channel chemistry configuration */
typedef struct {
    uint16_t v_full_mv;
    uint16_t v_storage_mv;
    uint16_t v_min_cutoff_mv;
    uint16_t v_trickle_enter_mv;
    uint16_t v_trickle_exit_mv;
    uint16_t i_trickle_ma;
    uint8_t  soc_lut_count;
    const uint16_t *soc_voltage_mv;
    const uint8_t  *soc_percent;
} batt_chemistry_t;

/* CH1 (Li-Ion): Full 4.20V | Storage 3.80V | Cutoff 3.00V | Trickle <2.80V -> 3.00V */
#define CH1_FULL_CHARGE_MV     4200u
#define CH1_STORAGE_MV         3800u
#define CH1_SAFE_CUTOFF_MV     3000u
#define CH1_TRICKLE_ENTER_MV   2800u
#define CH1_TRICKLE_EXIT_MV    3000u
#define CH1_TRICKLE_CURRENT_MA 100u

#define CH1_SOC_LUT_COUNT 7u
extern const uint16_t CH1_SOC_VOLTAGE_MV[CH1_SOC_LUT_COUNT] PROGMEM;
extern const uint8_t  CH1_SOC_PERCENT[CH1_SOC_LUT_COUNT] PROGMEM;

/* CH2 (LiFePO4): Full 3.65V | Storage 3.30V | Cutoff 2.50V | Trickle <2.30V -> 2.50V */
#define CH2_FULL_CHARGE_MV     3650u
#define CH2_STORAGE_MV         3300u
#define CH2_SAFE_CUTOFF_MV     2500u
#define CH2_TRICKLE_ENTER_MV   2300u
#define CH2_TRICKLE_EXIT_MV    2500u
#define CH2_TRICKLE_CURRENT_MA 100u

#define CH2_SOC_LUT_COUNT 7u
extern const uint16_t CH2_SOC_VOLTAGE_MV[CH2_SOC_LUT_COUNT] PROGMEM;
extern const uint8_t  CH2_SOC_PERCENT[CH2_SOC_LUT_COUNT] PROGMEM;

/* Storage deadband: +/-0.02 V */
#define STORAGE_DEADBAND_MV    20u

/* GUI current range */
#define CHG_CURRENT_MIN_MA     100u
#define CHG_CURRENT_MAX_MA     1200u
#define CHG_CURRENT_STEP_MA    100u
#define CHG_CURRENT_DEFAULT_MA 500u
#define CHG_CUTOFF_CURRENT_MA  50u

/* -------------------------------------------------------------------------
 * Charging State Machine
 * ------------------------------------------------------------------------- */
typedef enum {
    CHG_STATE_IDLE = 0,
    CHG_STATE_TRICKLE,
    CHG_STATE_CC,
    CHG_STATE_CV,
    CHG_STATE_DONE,
    CHG_STATE_FAULT
} charging_state_t;

/* -------------------------------------------------------------------------
 * Discharging State Machine
 * ------------------------------------------------------------------------- */
typedef enum {
    DSG_STATE_IDLE = 0,
    DSG_STATE_ACTIVE,
    DSG_STATE_DONE,
    DSG_STATE_FAULT
} discharging_state_t;

/* -------------------------------------------------------------------------
 * Storage State Machine
 * ------------------------------------------------------------------------- */
typedef enum {
    STG_STATE_IDLE = 0,
    STG_STATE_ASSESS,
    STG_STATE_SUB_CHARGE,
    STG_STATE_SUB_DISCHARGE,
    STG_STATE_DONE
} storage_state_t;

/* -------------------------------------------------------------------------
 * Capacity Test State Machine
 * ------------------------------------------------------------------------- */
typedef enum {
    CAP_STATE_IDLE = 0,
    CAP_STATE_STAGE1_DISCHARGE,
    CAP_STATE_STAGE2_CHARGE,
    CAP_STATE_DONE
} capacity_state_t;

/* -------------------------------------------------------------------------
 * System timebase (provided by main.c / timer0 ISR)
 * ------------------------------------------------------------------------- */
extern volatile uint32_t g_sys_millis;
extern volatile uint32_t g_sys_micros;

uint32_t sys_millis(void);
uint32_t sys_micros(void);

#endif /* CONFIG_H */
