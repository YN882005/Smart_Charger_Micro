# Smart Battery Charger

Bare-metal firmware for an ATmega328P-PU smart battery charger and discharger
with capacity analysis. The project uses pure AVR-C, a 14.7456 MHz external
crystal, four power channels, four INA233 telemetry devices, a 128x64 I2C OLED,
an HW-040 rotary encoder, and a 2.3 kHz buzzer.

This repository supports **CH1 (Li-ion)** and **CH2 (LiFePO4)** by default.
CH3 and CH4 provide expansion slot pin mappings and can be configured for a
different chemistry profile.

## Features

- 57.6 kHz buck PWM control for charging
- Current-regulated trickle, CC, and CV charging
- Fixed-resistor discharge control
- Automatic storage-voltage conditioning
- Two-stage capacity measurement using INA233 energy accumulation
- 128x64 OLED dashboard and menu system
- Dual-sensitivity HW-040 encoder navigation
- Non-blocking buzzer menu tones and completion melodies
- Cooperative 1 ms timebase with a zero-blocking main loop
- Safe GPIO pre-latching on reset and 5 ms charge/discharge dead time

## Default Hardware

| Function | MCU pin | Device |
|---|---|---|
| TWI SDA / SCL | PC4 / PC5 | SSD1306 OLED and INA233 devices |
| OLED | I2C 0x3C | 128x64 monochrome display |
| CH1 charge PWM | PB1 / OC1A | Active-low P-MOSFET switch |
| CH2 charge PWM | PB2 / OC1B | Active-low P-MOSFET switch |
| CH3 charge PWM | PB3 / OC2A | Expansion channel |
| CH4 charge PWM | PD3 / OC2B | Expansion channel |
| CH1 discharge | PB0 | Active-high N-MOSFET |
| CH2 discharge | PB5 | Active-high N-MOSFET |
| CH3 discharge | PB4 | Expansion channel |
| CH4 discharge | PC0 | Expansion channel |
| Encoder CLK / DT / switch | PC1 / PC2 / PC3 | HW-040 |
| INA233 alert | PD5 | Shared active-low open-drain input |
| Buzzer | PD4 / T0 | 2.3 kHz transducer |

The power input is reverse protected and feeds a buck switch, 100 uH inductor,
SS24 Schottky diode, and 10 uF output capacitor per channel. Battery current is
measured across a 33.3 mOhm shunt. The discharge bank is nominally 11.75 ohms
per channel.

> **Safety:** The output stages can energize batteries or loads. Never enable
> charge or discharge hardware until the schematic, gate-driver polarity,
> protection circuits, battery limits, and thermal behavior have been verified.
> Always ensure that charge and discharge MOSFETs cannot conduct at the same
> time.

## INA233 Channels

| Channel | INA233 | Address | Chemistry | Full | Storage | Cutoff | Trickle |
|---|---|---|---|---|---|---|---|
| CH1 | U6 | 0x40 | Li-ion | 4.20 V | 3.80 V | 3.00 V | < 2.80 V -> 3.00 V |
| CH2 | U7 | 0x45 | LiFePO4 | 3.65 V | 3.30 V | 2.50 V | < 2.30 V -> 2.50 V |
| CH3 | U8 | 0x41 | Configurable | -- | -- | -- | -- |
| CH4 | U9 | 0x44 | Configurable | -- | -- | -- | -- |

Each INA233 uses a 33.3 mOhm shunt, a 50 uA current LSB, and a 1.25 mW power
LSB. The calibration register is initialized to `0x0C03`, corresponding to the
specified shunt and current scale.

State of charge is calculated from channel-specific flash-based lookup tables.
A voltage below 1.0 V with the FETs off is reported as disconnected.

## Charging

The Timer1 and Timer2 channels provide 57.6 kHz, 8-bit fast PWM from the
14.7456 MHz clock. PWM duty is clamped to `240` counts (approximately 94%) and
starts from zero.

The charging state machine performs:

1. Trickle charging below the chemistry trickle threshold.
2. Constant-current regulation toward the configured current limit.
3. Constant-voltage regulation at the full or storage target.
4. Charge termination after the current falls below
   `max(50 mA, 5% of I_set)` for three seconds.

Charge current can be selected in 100 mA steps from 100 mA to 1200 mA.

## Discharging

Discharging uses a fixed 11.75 ohm resistive load and an active-high N-MOSFET.
Before discharge is enabled, charging PWM is set off and at least 5 ms is
allowed for the transition.

The discharge cutoff menu wraps in a circular list. CH1 includes cutoffs from
3.0 V to 4.0 V; CH2 includes cutoffs from 2.5 V to 3.5 V. The cutoff
`Discharge to Max` maps to the chemistry minimum.

## Storage Conditioning

Storage mode measures the cell for 200 ms while both power directions are off.
It then:

- Finishes immediately if the voltage is already within +/-20 mV of target.
- Charges at the selected current if the voltage is too low.
- Discharges through the resistive load if the voltage is too high.

Storage targets are 3.80 V for Li-ion and 3.30 V for LiFePO4.

## Capacity Test

A capacity test uses a two-stage sequence:

1. Discharge to the chemistry minimum cutoff and relax for 2 seconds.
2. Reset elapsed time and the INA233 `READ_EIN` accumulator, then charge at the
   selected current until full-charge termination.

The result is calculated from average power, average bus voltage, elapsed
time, and the accumulated INA233 energy samples. The conditional result screen
shows channel, stage, elapsed time, energy, and calculated capacity.

## OLED and Encoder UI

### Menu 1: Main dashboard

The dashboard shows CH1 and CH2 status, chemistry mode badge, animated battery
icon, voltage, SoC, and cycle timer.

Badges include `CHG`, `DSG`, `TRK`, `CAP`, `STR`, `IDLE`, and `DONE`.

### Menu 2: Channel select

Rotate to highlight CH1 or CH2. Press the encoder switch to enter the function
menu.

### Menu 3: Function menu

- Charge
- Discharge
- Storage
- Capacity Test
- Stop / Idle

Charge, storage, and capacity test functions use current sub-menus from 100 mA
to 1200 mA in 100 mA steps. Discharge uses the chemistry-specific wrap-around
cutoff list.

### Menu 4: Capacity result

The capacity screen is reachable only after capacity-test mode is enabled.
It displays the measured capacity and remains hidden after `[OK]` is pressed
and the test is complete.

Navigation requires two physical encoder detents per menu step. Parameter
sub-menus require one detent per value step. A press is debounced for 25 ms.
Two presses within 400 ms act as one double press and immediately return the
UI to Menu 1 without stopping an active cycle.

## Audio Feedback

Every menu transition or selection plays a 400 ms, 2.3 kHz tone.

Completion sequences use:

`C6 1047 Hz / 90 ms -> E6 1319 Hz / 90 ms -> G6 1568 Hz / 90 ms -> C7 2093 Hz / 180 ms`

All buzzer patterns are generated without blocking the charging, discharge,
telemetry, or storage loops.

## Source Layout

```text
include/
  config.h
  twi.h
  ina233.h
  Charging.h
  Discharging.h
  Storage.h
  Capacity.h
  Encoder.h
  Buzzer.h
  OLED_Screen.h

src/
  main.c
  twi.c
  ina233.c
  Charging.c
  Discharging.c
  Storage.c
  Capacity.c
  Encoder.c
  Buzzer.c
  OLED_Screen.c

tests/
  test_firmware.c
```

## Build

The project includes a Windows AVR-GCC toolchain under `toolchain/`. A normal
build uses GNU Make and the AVR tools already on `PATH`:

```sh
make
```

The default MCU and crystal settings are:

```text
MCU:   atmega328p
F_CPU: 14745600 Hz
```

Build products are:

- `build/firmware.elf`
- `build/firmware.hex`

To build and run the firmware test target:

```sh
make test
```

To clean generated files:

```sh
make clean
```

Typical USBasp flash command:

```sh
avrdude -c usbasp -p m328p -U flash:w:build/firmware.hex:i
```

Do not change fuses or flash the board unless the fuse settings, clock source,
programmer, and target hardware have been independently verified.
